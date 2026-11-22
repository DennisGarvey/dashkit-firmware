#include "usb_capture.h"
#include "capture_protocol.h"
#include "board.h"
#include <stdatomic.h>

#include "driver/usb_serial_jtag.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define CAPTURE_QUEUE_DEPTH 1024

typedef struct {
    can_tagged_frame_t frame;
    uint64_t timestamp_us;
    uint32_t sequence;
} capture_item_t;

static QueueHandle_t s_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_boot_id;
static uint32_t s_sequence[2], s_queue_drops[2], s_overflows[2], s_usb_aborts;

static const uint8_t s_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// SPI2 is shared by both MCP2518FD controllers. Count clocked bits, which
// excludes mutex waits and the time spent setting up transactions.
static atomic_uint s_spi_bits[2];
static uint32_t s_serial_bytes;
static uint32_t s_can_nominal_bits[2], s_can_data_bits[2];

void usb_capture_account_spi(uint8_t bus_id, uint32_t clock_bits)
{
    if (bus_id < 2) atomic_fetch_add_explicit(&s_spi_bits[bus_id], clock_bits, memory_order_relaxed);
}

// Caller holds s_lock. Include frame overhead and intermission, with a
// stuffing allowance on the fields eligible for variable bit stuffing.
static void account_can_frame(uint8_t bus, const can_frame_t *frame)
{
    const uint32_t dlc = frame->dlc & 15;
    const uint32_t bytes = frame->rtr ? 0 : (frame->fd ? s_lengths[dlc] : (dlc > 8 ? 8 : dlc));
    uint32_t nominal = frame->extended ? 54 : 34;
    uint32_t data = 0;
    if (frame->fd) {
        data = (5 + bytes * 8) * 6 / 5 + (bytes > 16 ? 33 : 28);
        if (!frame->brs) {
            nominal += data;
            data = 0;
        }
    } else {
        nominal = ((frame->extended ? 54 : 34) + bytes * 8) * 6 / 5 + 13;
    }
    s_can_nominal_bits[bus] += nominal;
    s_can_data_bits[bus] += data;
}

void usb_capture_overflow(uint8_t bus_id)
{
    if (bus_id > 1) return;
    portENTER_CRITICAL(&s_lock);
    s_overflows[bus_id]++;
    portEXIT_CRITICAL(&s_lock);
}

void usb_capture_submit(const can_tagged_frame_t *frame)
{
    if (!s_queue || frame->bus_id > 1) return;
    capture_item_t item = { .frame = *frame };
    portENTER_CRITICAL(&s_lock);
    account_can_frame(frame->bus_id, &frame->frame);
    item.sequence = ++s_sequence[frame->bus_id];
    item.timestamp_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&s_lock);
        s_queue_drops[frame->bus_id]++;
        portEXIT_CRITICAL(&s_lock);
    }
}

// Keep the USB endpoint supplied with full packets rather than submitting one
// short USB transfer for each CAN frame.
static uint8_t s_output[2048];
static size_t s_output_size;

static void flush_records(void)
{
    if (!s_output_size) return;
    size_t sent = 0;
    while (sent < s_output_size) {
        int n = usb_serial_jtag_write_bytes(s_output + sent, s_output_size - sent, pdMS_TO_TICKS(10));
        if (n <= 0) {
            s_usb_aborts++;
            break;
        }
        s_serial_bytes += (uint32_t)n;
        sent += (size_t)n;
    }
    s_output_size = 0;
}

static void write_record(uint8_t *raw, size_t size)
{
    if (sizeof(s_output) - s_output_size < DK_MAX_WIRE + 1) flush_records();
    s_output[s_output_size++] = 0;
    s_output_size += dk_encode(raw, size, s_output + s_output_size);
}

static void common_header(uint8_t *raw, uint8_t type, uint8_t bus,
                          uint32_t sequence, uint64_t timestamp_us)
{
    memset(raw, 0, DK_MAX_RAW);
    raw[0] = DK_VERSION;
    raw[1] = type;
    raw[2] = bus;
    dk_put(raw + 4, s_boot_id, 4);
    dk_put(raw + 8, sequence, 4);
    dk_put(raw + 12, timestamp_us, 8);
}

static uint16_t utilization_permille(uint64_t busy_us, uint64_t elapsed_us)
{
    return busy_us >= elapsed_us ? 1000 : busy_us * 1000 / elapsed_us;
}

static void write_utilization(int64_t now)
{
    static int64_t previous_time;
    static configRUN_TIME_COUNTER_TYPE previous_idle[2];
    configRUN_TIME_COUNTER_TYPE idle[2] = {
        ulTaskGetIdleRunTimeCounterForCore(0),
        ulTaskGetIdleRunTimeCounterForCore(1),
    };
    uint32_t nominal[2], data[2];
    portENTER_CRITICAL(&s_lock);
    for (int bus = 0; bus < 2; bus++) {
        nominal[bus] = s_can_nominal_bits[bus];
        data[bus] = s_can_data_bits[bus];
        s_can_nominal_bits[bus] = s_can_data_bits[bus] = 0;
    }
    portEXIT_CRITICAL(&s_lock);
    const uint32_t spi0 = atomic_exchange_explicit(&s_spi_bits[0], 0, memory_order_relaxed);
    const uint32_t spi1 = atomic_exchange_explicit(&s_spi_bits[1], 0, memory_order_relaxed);
    const uint32_t serial = s_serial_bytes;
    s_serial_bytes = 0;

    if (previous_time) {
        uint64_t elapsed = now - previous_time;
        uint8_t raw[DK_MAX_RAW];
        common_header(raw, DK_UTILIZATION, 0, 0, now);
        // Normalize both directions to the 8 Mbit/s logical serial setting (8N1).
        dk_put(raw + 20, utilization_permille((uint64_t)serial * 10 / 8, elapsed), 2);
        const uint64_t spi_us = (uint64_t)spi0 * 1000000 / CAN0_SPI_CLOCK_HZ +
                                (uint64_t)spi1 * 1000000 / CAN1_SPI_CLOCK_HZ;
        dk_put(raw + 26, utilization_permille(spi_us, elapsed), 2);
        for (int i = 0; i < 2; i++) {
            uint64_t idle_us = idle[i] - previous_idle[i];
            uint64_t can_us = (uint64_t)nominal[i] * 1000000 / CAN_NOMINAL_BITRATE +
                              (uint64_t)data[i] * 1000000 / CAN_FD_DATA_BITRATE;
            dk_put(raw + 22 + 2 * i, 1000 - utilization_permille(idle_us, elapsed), 2);
            dk_put(raw + 28 + 2 * i, utilization_permille(can_us, elapsed), 2);
        }
        write_record(raw, DK_UTILIZATION_SIZE);
    }
    previous_time = now;
    memcpy(previous_idle, idle, sizeof(previous_idle));
}

static void capture_task(void *arg)
{
    (void)arg;
    int64_t next_status = 0;
    int64_t next_flush = 0;
    capture_item_t item;
    while (true) {
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(2)) == pdTRUE) {
            uint8_t raw[DK_MAX_RAW];
            const can_frame_t *f = &item.frame.frame;
            common_header(raw, DK_FRAME, item.frame.bus_id, item.sequence, item.timestamp_us);
            raw[3] = (f->extended ? 1 : 0) | (f->rtr ? 2 : 0) | (f->fd ? 4 : 0) | (f->brs ? 8 : 0);
            dk_put(raw + 20, f->id, 4);
            raw[24] = f->fd ? s_lengths[f->dlc & 15] : (f->dlc > 8 ? 8 : f->dlc);
            if (!f->rtr) memcpy(raw + 25, f->data, raw[24]);
            write_record(raw, 25 + raw[24]);
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_status) {
            write_utilization(now);
            for (uint8_t bus = 0; bus < 2; bus++) {
                uint8_t raw[DK_MAX_RAW];
                portENTER_CRITICAL(&s_lock);
                common_header(raw, DK_STATUS, bus, s_sequence[bus], now);
                dk_put(raw + 20, s_queue_drops[bus], 4);
                dk_put(raw + 24, s_overflows[bus], 4);
                portEXIT_CRITICAL(&s_lock);
                dk_put(raw + 28, s_usb_aborts, 4);
                write_record(raw, DK_STATUS_SIZE);
            }
            next_status = now + 1000000;
        }
        if (now >= next_flush) {
            flush_records();
            next_flush = now + 2000;
        }
    }
}

esp_err_t usb_capture_init(void)
{
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = 16384,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&config);
    if (err != ESP_OK) return err;
    s_queue = xQueueCreate(CAPTURE_QUEUE_DEPTH, sizeof(capture_item_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
    s_boot_id = esp_random();
    if (xTaskCreatePinnedToCore(capture_task, "can2usb", 4096, NULL, 5, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
