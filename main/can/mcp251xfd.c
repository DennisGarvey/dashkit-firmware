#include "mcp251xfd_rx_clock.h"
#include "mcp251xfd.h"
#ifdef CONFIG_DASHKIT_USB_TX
#include "mcp251xfd_classic_rx.h"
#endif
#include "usb_capture.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#ifdef CONFIG_DASHKIT_USB_TX
#include <stdatomic.h>
#endif
#include <string.h>

static const char *TAG = "mcp251xfd";

// ---------------------------------------------------------------------------
// MCP251xFD Register Addresses
// ---------------------------------------------------------------------------
#define REG_C1CON       0x000
#define REG_C1NBTCFG    0x004
#define REG_C1DBTCFG    0x008
#define REG_C1INT       0x01C
#define REG_C1TEFCON    0x040
#define REG_C1TEFSTA    0x044
#define REG_C1FIFOCON(n)  (0x05C + ((n) - 1) * 12)
#define REG_C1FIFOSTA(n)  (0x060 + ((n) - 1) * 12)
#define REG_C1FIFOUA(n)   (0x064 + ((n) - 1) * 12)
#define REG_C1FLTCON(n)   (0x1D0 + (n) * 4)
#define REG_C1FLTOBJ(n)   (0x1F0 + (n) * 8)
#define REG_C1MASK(n)     (0x1F4 + (n) * 8)
#define REG_C1TREC      0x034
#define REG_C1BDIAG0    0x038
#define REG_C1BDIAG1    0x03C
#define REG_OSC         0xE00

#define RAM_BASE        0x400

// SPI instructions (4-bit opcode in upper nibble of first byte)
#define SPI_CMD_RESET       0x0000
#define SPI_CMD_READ(addr)  (0x3000 | ((addr) & 0xFFF))
#define SPI_CMD_WRITE(addr) (0x2000 | ((addr) & 0xFFF))

// C1CON bits
#define C1CON_REQOP_SHIFT   24
#define C1CON_REQOP_MASK    (0x7 << C1CON_REQOP_SHIFT)
#define C1CON_OPMOD_SHIFT   21
#define C1CON_OPMOD_MASK    (0x7 << C1CON_OPMOD_SHIFT)
#define C1CON_TXQEN         (1 << 20)
#define C1CON_STEF          (1 << 19)
#define C1CON_RTXAT         (1 << 16)
#define C1CON_ISOCRCEN      (1 << 5)
#define C1CON_PXEDIS        (1 << 6)

#define OPMODE_CONFIG       0x04
#define OPMODE_LISTEN_ONLY  0x03  // Listen only mode
#define OPMODE_NORMAL_2_0   0x06  // CAN 2.0 mode
#define OPMODE_NORMAL_FD    0x00  // CAN FD mode

// C1INT bits
#define C1INT_RXIE          (1 << 17)
#define C1INT_RXIF          (1 << 1)
#define C1INT_TEFIE         (1 << 20)
#define C1INT_TEFIF         (1 << 4)
#define C1INT_TXATIE        (1 << 26)
#define C1INT_TXATIF        (1 << 10)
#ifdef CONFIG_DASHKIT_USB_TX
#define C1INT_ENABLED       (C1INT_RXIE | C1INT_TEFIE | C1INT_TXATIE)
#else
#define C1INT_ENABLED       C1INT_RXIE
#endif

// FIFO control bits
#define FIFOCON_TXEN        (1 << 7)
#define FIFOCON_UINC        (1 << 8)
#define FIFOCON_TXREQ       (1 << 9)
#define FIFOCON_FRESET      (1 << 10)
#define FIFOCON_TXATIE      (1 << 4)
#define TEFCON_TEFNEIE      (1 << 0)
#define FIFOCON_TFNRFNIE    (1 << 0)   // RX not empty / TX not full interrupt enable
#define FIFOCON_FSIZE_SHIFT 24
#define FIFOCON_PLSIZE_SHIFT 29

// FIFO status bits
#define FIFOSTA_TFNRFNIF    (1 << 0)   // Not empty (RX) / not full (TX)
#define TEFSTA_NOT_EMPTY   (1 << 0)

#define RX_FIFO_OBJECTS    24
#define RX_OBJECT_BYTES    76
#ifdef CONFIG_DASHKIT_USB_TX
#define RX_FIFO_BASE       8  // One untimestamped transmit event occupies 8 bytes.
#else
#define RX_FIFO_BASE       0
#endif

#define FIFOSTA_TXLARB       (1 << 6)
#define FIFOSTA_TXERR        (1 << 5)
#define FIFOSTA_TXABT        (1 << 7)
#define TREC_TXBO           (1 << 21)
#define FIFOSTA_TXATIF      (1 << 4)   // TX attempt limit reached
#define FIFOSTA_RXOVIF      (1 << 3)   // RX FIFO overflow: a frame was dropped
                                       // because the FIFO was full (sticky; the
                                       // app clears it by writing the bit to 0)

// How often the rx_task reports the accumulated RX FIFO overflow count.

// OSC register bits
#define OSC_OSCRDY          (1 << 10)
#define OSC_SCLKRDY         (1 << 12)

// ---------------------------------------------------------------------------
// Driver context
// ---------------------------------------------------------------------------
typedef struct {
    spi_device_handle_t spi_dev;
    gpio_num_t          pin_int;
    uint32_t            osc_freq_hz;
    uint32_t            bitrate;
    uint32_t            bitrate_data;
    uint8_t             bus_id;
    can_rx_callback_t   rx_cb;
    void               *rx_cb_ctx;
    TaskHandle_t        rx_task_handle;
    SemaphoreHandle_t   int_sem;
    SemaphoreHandle_t   spi_mutex;
    volatile bool       running;
    // Count of RX FIFO overflow events (each = at least one dropped frame). The
    // chip only flags that an overflow happened, not how many frames were lost.
    // Incremented in read_rx_fifo; reported by an independent esp_timer so the
    // log fires even when the drain loop is hogging the rx_task on a busy bus.
    volatile uint32_t   rx_overflow_events;
    uint32_t            rx_overflow_reported;  // last value the timer logged
    esp_timer_handle_t  overflow_timer;
    uint32_t           rx_fifo_config;
#ifdef CONFIG_DASHKIT_USB_TX
    _Alignas(4) uint8_t spi_tx[72];
#else
    _Alignas(4) uint8_t spi_tx[64];
#endif
#ifdef CONFIG_DASHKIT_USB_TX
    mcp251xfd_classic_rx_t classic_rx;
    atomic_bool classic_rx_ready;
    _Alignas(4) uint8_t spi_rx[MCP_CLASSIC_CHUNK_BYTES];
#else
    _Alignas(4) uint8_t spi_rx[64];
#endif
    mcp251xfd_rx_clock_t rx_clock;
    int64_t rx_clock_sample_due;
} mcp251xfd_ctx_t;

// ---------------------------------------------------------------------------
// SPI low-level
// ---------------------------------------------------------------------------
// Separate command and DMA-aligned data buffers avoid allocating/copying a DMA
// bounce buffer for every small register transaction.
static esp_err_t spi_write_reg(mcp251xfd_ctx_t *ctx, uint16_t addr, const uint8_t *data, size_t len)
{
    if (len > sizeof(ctx->spi_tx)) return ESP_ERR_INVALID_SIZE;
    xSemaphoreTake(ctx->spi_mutex, portMAX_DELAY);
    memcpy(ctx->spi_tx, data, len);
    spi_transaction_t t = {
        .cmd = SPI_CMD_WRITE(addr),
        .length = len * 8,
        .tx_buffer = ctx->spi_tx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->spi_dev, &t);
    xSemaphoreGive(ctx->spi_mutex);
    if (err == ESP_OK) usb_capture_account_spi(ctx->bus_id, (len + 2) * 8);
    return err;
}

static esp_err_t spi_read_reg(mcp251xfd_ctx_t *ctx, uint16_t addr, uint8_t *data, size_t len)
{
    if (len > sizeof(ctx->spi_rx)) return ESP_ERR_INVALID_SIZE;
    xSemaphoreTake(ctx->spi_mutex, portMAX_DELAY);
    spi_transaction_t t = {
        .cmd = SPI_CMD_READ(addr),
        .length = len * 8,
        .rx_buffer = ctx->spi_rx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->spi_dev, &t);
    if (err == ESP_OK) memcpy(data, ctx->spi_rx, len);
    xSemaphoreGive(ctx->spi_mutex);
    if (err == ESP_OK) usb_capture_account_spi(ctx->bus_id, (len + 2) * 8);
    return err;
}

static esp_err_t write_reg32(mcp251xfd_ctx_t *ctx, uint16_t addr, uint32_t val)
{
    uint8_t d[4] = { val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF };
    return spi_write_reg(ctx, addr, d, 4);
}

static esp_err_t read_reg32(mcp251xfd_ctx_t *ctx, uint16_t addr, uint32_t *val)
{
    *val = 0;
    uint8_t d[4];
    esp_err_t err = spi_read_reg(ctx, addr, d, 4);
    if (err == ESP_OK) {
        *val = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    }
    return err;
}

static esp_err_t spi_reset(mcp251xfd_ctx_t *ctx)
{
    spi_transaction_t t = { .cmd = SPI_CMD_RESET };
    return spi_device_polling_transmit(ctx->spi_dev, &t);
}

// ---------------------------------------------------------------------------
// DLC helpers
// ---------------------------------------------------------------------------
static uint8_t dlc_to_len(uint8_t dlc)
{
    static const uint8_t map[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return (dlc < 16) ? map[dlc] : 64;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static esp_err_t wait_for_osc(mcp251xfd_ctx_t *ctx)
{
    for (int i = 0; i < 100; i++) {
        uint32_t osc = 0;
        read_reg32(ctx, REG_OSC, &osc);
        if (osc & OSC_OSCRDY) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "Oscillator not ready");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t set_mode(mcp251xfd_ctx_t *ctx, uint8_t mode)
{
    uint32_t con;
    esp_err_t err = read_reg32(ctx, REG_C1CON, &con);
    if (err != ESP_OK) return err;

    con &= ~C1CON_REQOP_MASK;
    con |= ((uint32_t)mode << C1CON_REQOP_SHIFT);
    err = write_reg32(ctx, REG_C1CON, con);
    if (err != ESP_OK) return err;

    for (int i = 0; i < 100; i++) {
        read_reg32(ctx, REG_C1CON, &con);
        uint8_t opmod = (con & C1CON_OPMOD_MASK) >> C1CON_OPMOD_SHIFT;
        if (opmod == mode) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGE(TAG, "Failed to enter mode 0x%02x", mode);
    return ESP_ERR_TIMEOUT;
}

static void sample_rx_clock(mcp251xfd_ctx_t *ctx)
{
    const int64_t before = esp_timer_get_time();
    if (before < ctx->rx_clock_sample_due) return;
    ctx->rx_clock_sample_due = before + 1000000;
    uint32_t counter;
    if (read_reg32(ctx, MCP251XFD_TBC, &counter) == ESP_OK) {
        mcp251xfd_rx_clock_sample(&ctx->rx_clock, counter, before, esp_timer_get_time());
    }
}

static esp_err_t configure_bitrate(mcp251xfd_ctx_t *ctx)
{
    // Target: 20 TQ per bit, 80% sample point (industry standard)
    // Bitrate = osc_freq / ((BRP+1) * (Sync + TSEG1 + TSEG2))
    // Sync = 1 TQ always

    uint32_t target_tq = 20;

    // Find BRP that gives us close to target_tq
    uint32_t brp = (ctx->osc_freq_hz / (ctx->bitrate * target_tq)) - 1;

    // Verify
    uint32_t actual_tq = ctx->osc_freq_hz / (ctx->bitrate * (brp + 1));
    if (actual_tq < 8 || actual_tq > 40) {
        ESP_LOGE(TAG, "Cannot find valid bit timing for %lu bps", (unsigned long)ctx->bitrate);
        return ESP_ERR_INVALID_ARG;
    }

    // 80% sample point: sample at Sync + TSEG1
    uint8_t tseg1 = (actual_tq * 80 / 100) - 1;
    uint8_t tseg2 = actual_tq - tseg1 - 1;
    uint8_t sjw = tseg2;  // SJW = TSEG2 for maximum tolerance

    // C1NBTCFG: [31:24]=BRP, [23:16]=TSEG1, [14:8]=TSEG2, [6:0]=SJW (all n-1)
    uint32_t nbtcfg = ((sjw - 1) & 0x7F) |
                      (((uint32_t)(tseg2 - 1) & 0x7F) << 8) |
                      (((uint32_t)(tseg1 - 1) & 0xFF) << 16) |
                      (((uint32_t)brp & 0xFF) << 24);

    esp_err_t err = write_reg32(ctx, REG_C1NBTCFG, nbtcfg);
    if (err != ESP_OK) return err;

    // Data bitrate for CAN FD (default to 2Mbps if not specified)
    uint32_t data_rate = ctx->bitrate_data > 0 ? ctx->bitrate_data : (ctx->bitrate * 4);
    uint32_t dbrp = (ctx->osc_freq_hz / (data_rate * 10)) - 1;  // ~10 TQ
    uint32_t dactual_tq = ctx->osc_freq_hz / (data_rate * (dbrp + 1));
    if (dactual_tq >= 5 && dactual_tq <= 25) {
        uint8_t dtseg1 = (dactual_tq * 75 / 100) - 1;
        uint8_t dtseg2 = dactual_tq - dtseg1 - 1;
        uint8_t dsjw = dtseg2;

        // C1DBTCFG: [31:24]=DBRP, [20:16]=DTSEG1, [11:8]=DTSEG2, [3:0]=DSJW (all n-1)
        uint32_t dbtcfg = ((dsjw - 1) & 0x0F) |
                          (((uint32_t)(dtseg2 - 1) & 0x0F) << 8) |
                          (((uint32_t)(dtseg1 - 1) & 0x1F) << 16) |
                          (((uint32_t)dbrp & 0xFF) << 24);

        write_reg32(ctx, REG_C1DBTCFG, dbtcfg);
    }

    return ESP_OK;
}

#ifdef CONFIG_DASHKIT_USB_TX
static esp_err_t classic_read(void *io, uint16_t addr, uint8_t *data, size_t len)
{
    return spi_read_reg(io, addr, data, len);
}

static esp_err_t classic_write(void *io, uint16_t addr, uint32_t value)
{
    return write_reg32(io, addr, value);
}

static void classic_deliver(void *io, const can_frame_t *frame, uint32_t counter)
{
    mcp251xfd_ctx_t *ctx = io;
    if (!ctx->rx_cb) return;
    const can_tagged_frame_t tagged = {
        .frame = *frame,
        .bus_id = ctx->bus_id,
        .capture_timestamp_us = mcp251xfd_rx_clock_time(&ctx->rx_clock, counter),
    };
    ctx->rx_cb(&tagged, ctx->rx_cb_ctx);
}

static void classic_overflow(void *io)
{
    mcp251xfd_ctx_t *ctx = io;
    usb_capture_overflow(ctx->bus_id);
}
#endif

static esp_err_t configure_fifos(mcp251xfd_ctx_t *ctx)
{
#ifdef CONFIG_DASHKIT_USB_TX
    ctx->classic_rx = (mcp251xfd_classic_rx_t) {
        .io = ctx,
        .read = classic_read,
        .write = classic_write,
        .deliver = classic_deliver,
        .overflow = classic_overflow,
        .has_tx_event = true,
    };
    // FIFO 2 remains single-slot TX. The Classic RX layout leaves space
    // for its 72-byte object and the 8-byte transmit event object.
    esp_err_t err = write_reg32(ctx, REG_C1FIFOCON(2),
                                (7u << FIFOCON_PLSIZE_SHIFT) | FIFOCON_TXEN | FIFOCON_TXATIE);
    return err == ESP_OK ? mcp251xfd_classic_configure(&ctx->classic_rx) : err;
#else
    // FIFO 1: RX FIFO - 32 messages deep (max for FSIZE field), 8 byte payload.
    // 32 * 16 bytes = 512 B of the chip's 2 KB RAM. TX FIFO uses another 64 B.
    // Larger depth absorbs SPI/scheduling jitter (both CAN chips share SPI2,
    // and the rx_task only runs once per FreeRTOS tick).
    // 24 * (8-byte header + 64-byte payload), alongside one TX object and
    // one untimestamped transmit event in the TX profile.
    uint32_t rxfifo_con = (7u << FIFOCON_PLSIZE_SHIFT) |
                          ((RX_FIFO_OBJECTS - 1u) << FIFOCON_FSIZE_SHIFT) | FIFOCON_TFNRFNIE;
    rxfifo_con |= MCP251XFD_RX_TIMESTAMP_ENABLE;
    ctx->rx_fifo_config = rxfifo_con;
    esp_err_t err = write_reg32(ctx, REG_C1FIFOCON(1), rxfifo_con);
    if (err != ESP_OK) return err;

    // FIFO 2: TX FIFO - 4 messages deep, 8 byte payload
    uint32_t txfifo_con = (0 << FIFOCON_PLSIZE_SHIFT) |  // 8 bytes payload
                          (3 << FIFOCON_FSIZE_SHIFT)  |   // 4 messages
                          FIFOCON_TXEN;                    // TX mode
#ifdef CONFIG_DASHKIT_USB_TX
    txfifo_con = (7u << FIFOCON_PLSIZE_SHIFT) | FIFOCON_TXEN | FIFOCON_TXATIE;
#endif
    err = write_reg32(ctx, REG_C1FIFOCON(2), txfifo_con);
    if (err != ESP_OK) return err;

    // MIDE=0 and a zero ID mask accept both standard and extended identifiers.
    err = write_reg32(ctx, REG_C1FLTOBJ(0), 0);
    if (err != ESP_OK) return err;
    err = write_reg32(ctx, REG_C1MASK(0), 0);
    if (err != ESP_OK) return err;
    for (uint8_t w = 0; w < 8; w++) {
        err = write_reg32(ctx, REG_C1FLTCON(w), w == 0 ? 0x81 : 0);
        if (err != ESP_OK) return err;
    }

    return ESP_OK;
#endif
}

// ---------------------------------------------------------------------------
// Interrupt handling
// ---------------------------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(ctx->int_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void read_rx_fifo(mcp251xfd_ctx_t *ctx)
{
#ifdef CONFIG_DASHKIT_USB_TX
    if (!atomic_load(&ctx->classic_rx_ready)) return;
    // Give the other controller a scheduling opportunity between chunks.
    // Poll again immediately while frames remain on this controller.
    while (ctx->running && mcp251xfd_classic_drain(&ctx->classic_rx)) {
        sample_rx_clock(ctx);
        taskYIELD();
    }
#else
    const size_t payload_offset = 12;
    uint8_t msg[20];
    uint32_t ua;
    if (read_reg32(ctx, REG_C1FIFOUA(1), &ua) != ESP_OK) return;
    while (true) {
        // Check FIFO 1 status
        uint32_t fifosta;
        if (read_reg32(ctx, REG_C1FIFOSTA(1), &fifosta) != ESP_OK) return;

        // Overflow is sticky: a frame was dropped because the FIFO filled up.
        // Count it and clear the flag (write 0 to RXOVIF, preserving the rest).
        if (fifosta & FIFOSTA_RXOVIF) {
            ctx->rx_overflow_events++;
            usb_capture_overflow(ctx->bus_id);
            write_reg32(ctx, REG_C1FIFOSTA(1), fifosta & ~FIFOSTA_RXOVIF);
        }

        if (!(fifosta & FIFOSTA_TFNRFNIF)) {
            break;  // FIFO empty
        }

        // Get user address (RAM offset of next message)

        if (spi_read_reg(ctx, RAM_BASE + ua, msg, sizeof(msg)) != ESP_OK) return;

        // Parse message object
        uint32_t r0 = msg[0] | (msg[1] << 8) | (msg[2] << 16) | (msg[3] << 24);
        uint32_t r1 = msg[4] | (msg[5] << 8) | (msg[6] << 16) | (msg[7] << 24);

        can_tagged_frame_t tagged;
        memset(&tagged, 0, sizeof(tagged));
        tagged.bus_id = ctx->bus_id;
        sample_rx_clock(ctx);
        const uint32_t counter = (uint32_t)msg[8] | ((uint32_t)msg[9] << 8) |
                                 ((uint32_t)msg[10] << 16) | ((uint32_t)msg[11] << 24);
        tagged.capture_timestamp_us = mcp251xfd_rx_clock_time(&ctx->rx_clock, counter);

        // SID is in bits [10:0]
        tagged.frame.id = r0 & 0x7FF;
        tagged.frame.extended = (r1 >> 4) & 1;
        tagged.frame.rtr = (r1 >> 5) & 1;
        if (tagged.frame.extended) {
            uint32_t eid = (r0 >> 11) & 0x3FFFF;
            tagged.frame.id = (tagged.frame.id << 18) | eid;
        }

        tagged.frame.dlc = r1 & 0x0F;
        tagged.frame.fd = (r1 >> 7) & 1;
        tagged.frame.brs = (r1 >> 6) & 1;

        uint8_t data_len = dlc_to_len(tagged.frame.dlc);
        if (data_len > 8) {
            // For CAN FD frames > 8 bytes, read remaining data
            uint8_t extra[64];
            if (spi_read_reg(ctx, RAM_BASE + ua + payload_offset, extra, data_len) != ESP_OK) return;
            memcpy(tagged.frame.data, extra, data_len);
        } else {
            memcpy(tagged.frame.data, &msg[payload_offset], data_len);
        }

        // Increment FIFO head pointer
        if (write_reg32(ctx, REG_C1FIFOCON(1), ctx->rx_fifo_config | FIFOCON_UINC) != ESP_OK) return;
        ua += RX_OBJECT_BYTES;
        if (ua == RX_FIFO_BASE + RX_FIFO_OBJECTS * RX_OBJECT_BYTES) ua = RX_FIFO_BASE;

        // Deliver frame via callback
        if (ctx->rx_cb) {
            ctx->rx_cb(&tagged, ctx->rx_cb_ctx);
        }
    }
#endif
}

static void rx_task(void *arg)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)arg;
#ifdef CONFIG_DASHKIT_DEBUG_LOG
    uint32_t poll_count = 0;
#endif

    while (ctx->running) {
        // Block on the GPIO ISR semaphore (given on INT falling edge).
        // Fall through every 5 ms anyway as a safety net so we still drain the
        // FIFO if an interrupt is ever missed (e.g. semaphore lost during init,
        // or INT line already low when we enabled the IRQ).
        xSemaphoreTake(ctx->int_sem, pdMS_TO_TICKS(5));

        // INT is level-sensitive on the MCP2518FD: it stays asserted as long
        // as RXIF is set, so we drain the whole FIFO in one go. Reading the
        // FIFO clears RXIF, which releases INT.
        uint32_t intflag;
        read_reg32(ctx, REG_C1INT, &intflag);
#ifdef CONFIG_DASHKIT_USB_TX
        if (intflag & (C1INT_TEFIF | C1INT_TXATIF)) usb_capture_notify_tx_event();
#endif
        if (intflag & C1INT_RXIF) {
            read_rx_fifo(ctx);
        }

#ifdef CONFIG_DASHKIT_DEBUG_LOG
        if ((poll_count++ % 2000) == 0) {
            uint32_t con, fifosta, diag0, diag1;
            read_reg32(ctx, REG_C1CON, &con);
            read_reg32(ctx, REG_C1FIFOSTA(1), &fifosta);
            read_reg32(ctx, REG_C1BDIAG0, &diag0);
            read_reg32(ctx, REG_C1BDIAG1, &diag1);
            uint8_t opmod = (con >> C1CON_OPMOD_SHIFT) & 0x7;
            ESP_LOGI(TAG, "STATUS OPMOD=%d FIFOSTA=0x%08lx DIAG0=0x%08lx DIAG1=0x%08lx",
                     opmod, (unsigned long)fifosta,
                     (unsigned long)diag0, (unsigned long)diag1);
        }
#endif
    }

    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Interface implementation
// ---------------------------------------------------------------------------
static esp_err_t iface_init(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    // Reset the controller
    esp_err_t err = spi_reset(ctx);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Wait for oscillator ready
    err = wait_for_osc(ctx);
    if (err != ESP_OK) return err;

    // Verify we're in configuration mode after reset
    err = set_mode(ctx, OPMODE_CONFIG);
    if (err != ESP_OK) return err;

    // Configure bit timing
    err = configure_bitrate(ctx);
    if (err != ESP_OK) return err;
    err = write_reg32(ctx, MCP251XFD_TSCON,
                      MCP251XFD_TBC_ENABLE | (ctx->osc_freq_hz / 1000000u - 1u));
    if (err != ESP_OK) return err;
    ctx->rx_clock = (mcp251xfd_rx_clock_t){0};
    ctx->rx_clock_sample_due = 0;
    sample_rx_clock(ctx);

    // Enable ISO CRC for CAN FD, use default protocol exception handling
    uint32_t con;
    read_reg32(ctx, REG_C1CON, &con);
    con |= C1CON_ISOCRCEN;
#ifdef CONFIG_DASHKIT_USB_TX
    con |= C1CON_RTXAT | C1CON_STEF;  // Single attempt and successful-transmit events.
#else
    con &= ~C1CON_STEF;
#endif
    con &= ~C1CON_PXEDIS;  // Allow protocol exception (graceful CAN FD handling)
    con &= ~C1CON_TXQEN;   // Disable TXQ, we use FIFO 2 for TX
    err = write_reg32(ctx, REG_C1CON, con);
    if (err != ESP_OK) return err;

#ifdef CONFIG_DASHKIT_USB_TX
    // One completion slot, without controller timestamps.
    err = write_reg32(ctx, REG_C1TEFCON, TEFCON_TEFNEIE);
    if (err != ESP_OK) return err;
#endif
    // Configure FIFOs and filters
    err = configure_fifos(ctx);
    if (err != ESP_OK) return err;

    // Enable RX interrupt in C1INT
    uint32_t intcon;
    read_reg32(ctx, REG_C1INT, &intcon);
    intcon |= C1INT_ENABLED;
    write_reg32(ctx, REG_C1INT, intcon);

    ESP_LOGI(TAG, "Bus %d: MCP251xFD initialized (bitrate=%lu)", ctx->bus_id, (unsigned long)ctx->bitrate);
    return ESP_OK;
}

static esp_err_t iface_start(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    ctx->running = true;

    // Install GPIO ISR for INT pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ctx->pin_int),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // INT is active low
    };
    gpio_config(&io_conf);
    // ESP_ERR_INVALID_STATE = service already installed by an earlier interface; that's fine.
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(isr_err));
    }
    gpio_isr_handler_add(ctx->pin_int, gpio_isr_handler, ctx);

    // Create RX task
    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, ctx, 10, &ctx->rx_task_handle, 1);

    // Reset the receive FIFOs in config mode before enabling bus traffic.
    set_mode(ctx, OPMODE_CONFIG);

#ifdef CONFIG_DASHKIT_USB_TX
    esp_err_t reset_err = mcp251xfd_classic_reset(&ctx->classic_rx);
    if (reset_err != ESP_OK) return reset_err;
#else
    // Reset RX FIFO 1
    uint32_t fifocon_reset;
    read_reg32(ctx, REG_C1FIFOCON(1), &fifocon_reset);
    fifocon_reset |= FIFOCON_FRESET;
    write_reg32(ctx, REG_C1FIFOCON(1), fifocon_reset);
#endif
    vTaskDelay(pdMS_TO_TICKS(1));

    // Clear all interrupt flags
    write_reg32(ctx, REG_C1INT, C1INT_ENABLED);  // Keep enables, clear pending flags.
    write_reg32(ctx, REG_C1BDIAG0, 0);
    write_reg32(ctx, REG_C1BDIAG1, 0);

#if defined(CONFIG_DASHKIT_USB_TX)
    esp_err_t err = set_mode(ctx, OPMODE_NORMAL_2_0);
#else
    esp_err_t err = set_mode(ctx, OPMODE_LISTEN_ONLY);
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: failed to enter operating mode", ctx->bus_id);
        return err;
    }

#ifdef CONFIG_DASHKIT_USB_TX
    atomic_store(&ctx->classic_rx_ready, true);
#endif
    return ESP_OK;
}

static esp_err_t iface_stop(can_interface_t *self)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;
    ctx->running = false;
#ifdef CONFIG_DASHKIT_USB_TX
    atomic_store(&ctx->classic_rx_ready, false);
#endif

    gpio_isr_handler_remove(ctx->pin_int);

    // Back to config mode
    set_mode(ctx, OPMODE_CONFIG);

    ESP_LOGI(TAG, "Bus %d: stopped", ctx->bus_id);
    return ESP_OK;
}

#ifdef CONFIG_DASHKIT_USB_TX
static esp_err_t iface_poll_tx(can_interface_t *self, bool abort, can_tx_diagnostics_t *diagnostics)
{
    mcp251xfd_ctx_t *ctx = self->ctx;
    uint32_t events, status;
    if (read_reg32(ctx, REG_C1TEFSTA, &events) != ESP_OK) return ESP_FAIL;
    if (events & TEFSTA_NOT_EMPTY) {
        // USB capture owns the single outstanding TX slot, so the event
        // belongs to its current request. UINC consumes it without timestamps.
        return write_reg32(ctx, REG_C1TEFCON, TEFCON_TEFNEIE | FIFOCON_UINC);
    }
    if (read_reg32(ctx, REG_C1FIFOSTA(2), &status) != ESP_OK) return ESP_FAIL;
    if (!(status & FIFOSTA_TXATIF) && !abort) return ESP_ERR_NOT_FINISHED;

    diagnostics->fifo_status = status;
    diagnostics->valid = read_reg32(ctx, REG_C1BDIAG1, &diagnostics->bus_diagnostics) == ESP_OK &&
                         read_reg32(ctx, REG_C1TREC, &diagnostics->error_counters) == ESP_OK;

    uint32_t control;
    if (read_reg32(ctx, REG_C1FIFOCON(2), &control) != ESP_OK) return ESP_FAIL;
    control &= ~(FIFOCON_UINC | FIFOCON_TXREQ);
    if (write_reg32(ctx, REG_C1FIFOCON(2), control) != ESP_OK ||
        read_reg32(ctx, REG_C1FIFOCON(2), &control) != ESP_OK) return ESP_FAIL;
    if (control & FIFOCON_TXREQ) return ESP_ERR_NOT_FINISHED;

    // A successful transmission can race an abort. Consume its event
    // before allowing the slot to be reused.
    if (read_reg32(ctx, REG_C1TEFSTA, &events) != ESP_OK) return ESP_FAIL;
    if (events & TEFSTA_NOT_EMPTY) return write_reg32(ctx, REG_C1TEFCON, TEFCON_TEFNEIE | FIFOCON_UINC);

    if (write_reg32(ctx, REG_C1FIFOCON(2), control | FIFOCON_FRESET) != ESP_OK ||
        read_reg32(ctx, REG_C1FIFOCON(2), &control) != ESP_OK ||
        (control & (FIFOCON_FRESET | FIFOCON_TXREQ))) return ESP_FAIL;
    if ((status & FIFOSTA_TXATIF) &&
        write_reg32(ctx, REG_C1FIFOSTA(2), status & ~FIFOSTA_TXATIF) != ESP_OK) return ESP_FAIL;
    // TXERR describes this FIFO attempt. BDIAG1 is sticky across frames
    // and cannot identify whether this particular attempt saw an ACK error.
    diagnostics->arbitration_only = diagnostics->valid &&
        (status & (FIFOSTA_TXATIF | FIFOSTA_TXLARB | FIFOSTA_TXERR | FIFOSTA_TXABT)) ==
            (FIFOSTA_TXATIF | FIFOSTA_TXLARB) &&
        !(diagnostics->error_counters & TREC_TXBO);
    return (status & FIFOSTA_TXATIF) ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_RESPONSE;
}
#endif

#ifdef CONFIG_DASHKIT_USB_TX
static esp_err_t iface_send(can_interface_t *self, const can_frame_t *frame)
{
    if (frame->fd || frame->brs || frame->dlc > 8) return ESP_ERR_NOT_SUPPORTED;
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;

    // Check TX FIFO 2 has space
    uint32_t fifosta;
    if (read_reg32(ctx, REG_C1FIFOSTA(2), &fifosta) != ESP_OK) return ESP_FAIL;
    if (!(fifosta & FIFOSTA_TFNRFNIF)) {
        return ESP_ERR_NO_MEM;  // TX FIFO full
    }

    // Get user address for next TX slot
    uint32_t ua;
    if (read_reg32(ctx, REG_C1FIFOUA(2), &ua) != ESP_OK) return ESP_FAIL;

    // Build message object
    uint32_t t0 = frame->id & 0x7FF;
    if (frame->extended) {
        t0 = ((frame->id >> 18) & 0x7FF) | (((frame->id) & 0x3FFFF) << 11);
    }

    uint32_t t1 = frame->dlc & 0x0F;
    if (frame->extended) t1 |= (1 << 4);
    if (frame->fd)       t1 |= (1 << 7);
    if (frame->brs)      t1 |= (1 << 6);

    uint8_t data_len = dlc_to_len(frame->dlc);
    if (frame->rtr) t1 |= (1 << 5);
    uint8_t msg[72] = {0};
    msg[0] = t0 & 0xFF;
    msg[1] = (t0 >> 8) & 0xFF;
    msg[2] = (t0 >> 16) & 0xFF;
    msg[3] = (t0 >> 24) & 0xFF;
    msg[4] = t1 & 0xFF;
    msg[5] = (t1 >> 8) & 0xFF;
    msg[6] = (t1 >> 16) & 0xFF;
    msg[7] = (t1 >> 24) & 0xFF;
    memcpy(&msg[8], frame->data, data_len);

    if (spi_write_reg(ctx, RAM_BASE + ua, msg, sizeof(msg)) != ESP_OK) return ESP_FAIL;

    // Set UINC and TXREQ
    uint32_t fifocon;
    if (read_reg32(ctx, REG_C1FIFOCON(2), &fifocon) != ESP_OK) return ESP_FAIL;
    fifocon |= FIFOCON_UINC | FIFOCON_TXREQ;
    return write_reg32(ctx, REG_C1FIFOCON(2), fifocon);
}
#endif

static void iface_set_rx_callback(can_interface_t *self, can_rx_callback_t cb, void *user_ctx)
{
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)self->ctx;
    ctx->rx_cb = cb;
    ctx->rx_cb_ctx = user_ctx;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
can_interface_t *mcp251xfd_create(const mcp251xfd_config_t *config)
{
    can_interface_t *iface = calloc(1, sizeof(can_interface_t));
    mcp251xfd_ctx_t *ctx = calloc(1, sizeof(mcp251xfd_ctx_t));
    if (!iface || !ctx) {
        free(iface);
        free(ctx);
        return NULL;
    }

#ifdef CONFIG_DASHKIT_USB_TX
    atomic_init(&ctx->classic_rx_ready, false);
#endif
    ctx->pin_int = config->pin_int;
    ctx->osc_freq_hz = config->osc_freq_hz;
    ctx->bitrate = config->bitrate;
    ctx->bitrate_data = config->bitrate_data;
    ctx->bus_id = config->bus_id;
    ctx->rx_cb = NULL;
    ctx->rx_cb_ctx = NULL;
    ctx->running = false;
    ctx->int_sem = xSemaphoreCreateBinary();
    ctx->spi_mutex = xSemaphoreCreateMutex();

    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
#ifdef CONFIG_DASHKIT_USB_TX
        .max_transfer_sz = MCP_CLASSIC_CHUNK_BYTES + 4,
#else
        .max_transfer_sz = 76,  // 8 header + 4 cmd + 64 data
#endif
    };

    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    // ESP_ERR_INVALID_STATE = bus already initialized by an earlier device on the same host.
    // Sharing the SPI bus across multiple MCP2518FDs (different CS) is supported.
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        free(ctx);
        free(iface);
        return NULL;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
        .command_bits = 16,
    };

    err = spi_bus_add_device(config->spi_host, &dev_cfg, &ctx->spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        spi_bus_free(config->spi_host);
        free(ctx);
        free(iface);
        return NULL;
    }

    iface->init = iface_init;
    iface->start = iface_start;
    iface->stop = iface_stop;
#ifdef CONFIG_DASHKIT_USB_TX
    iface->send = iface_send;
    iface->poll_tx = iface_poll_tx;
#endif
    iface->set_rx_callback = iface_set_rx_callback;
    iface->bus_id = config->bus_id;
    iface->ctx = ctx;

    return iface;
}

void mcp251xfd_destroy(can_interface_t *iface)
{
    if (!iface) return;
    mcp251xfd_ctx_t *ctx = (mcp251xfd_ctx_t *)iface->ctx;
    if (ctx) {
        if (ctx->running) iface->stop(iface);
        if (ctx->int_sem) vSemaphoreDelete(ctx->int_sem);
        // SPI cleanup would go here
        free(ctx);
    }
    free(iface);
}
