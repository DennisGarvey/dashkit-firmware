#include "usb_capture.h"
#include "capture_protocol.h"
#ifdef CONFIG_DASHKIT_USB_TX
#include "can_manager.h"
#include <stdatomic.h>
#endif

#include "driver/usb_serial_jtag.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
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

#ifdef CONFIG_DASHKIT_USB_TX
static atomic_bool s_tx_ready;
static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

void usb_capture_enable_tx(void)
{
    atomic_store(&s_tx_ready, true);
}

static void write_tx_result(uint8_t bus, uint32_t sequence, uint8_t status)
{
    uint8_t raw[DK_MAX_RAW];
    common_header(raw, DK_TX_RESULT, bus, sequence, esp_timer_get_time());
    raw[20] = status;
    write_record(raw, 21);
}

static void read_tx_requests(void)
{
    static uint8_t pending[DK_MAX_WIRE];
    static size_t count;
    static bool discarding;
    static int64_t next_send;
    uint8_t bytes[256];
    int n = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), 0);
    for (int i = 0; i < n; ++i) {
        if (bytes[i]) {
            if (count == sizeof(pending)) discarding = true;
            if (!discarding) pending[count++] = bytes[i];
            continue;
        }
        uint8_t raw[DK_MAX_RAW];
        size_t size = discarding ? 0 : dk_decode(pending, count, raw);
        count = 0;
        discarding = false;
        if (!size || raw[1] != DK_TX_REQUEST || dk_get(raw + 4, 4) != s_boot_id) continue;

        uint32_t sequence = dk_get(raw + 8, 4);
        uint8_t bus = raw[2];
        int64_t now = esp_timer_get_time();
        if (!sequence || !atomic_load(&s_tx_ready)) continue;
        if (now < next_send) {
            write_tx_result(bus, sequence, DK_TX_RATE_LIMIT);
            continue;
        }
        next_send = now + 100000;
        can_frame_t frame = {
            .id = dk_get(raw + 20, 4),
            .extended = (raw[3] & 1) != 0,
            .rtr = (raw[3] & 2) != 0,
            .fd = (raw[3] & 4) != 0,
            .brs = (raw[3] & 8) != 0,
        };
        while (lengths[frame.dlc] != raw[24]) ++frame.dlc;
        if (!frame.rtr) memcpy(frame.data, raw + 25, raw[24]);
        esp_err_t result = can_manager_send(bus, &frame);
        uint8_t status = DK_TX_REJECTED;
        if (result == ESP_OK) status = DK_TX_QUEUED;
        else if (result == ESP_ERR_NO_MEM) status = DK_TX_FIFO_FULL;
        else if (result == ESP_ERR_TIMEOUT) status = DK_TX_PREVIOUS_FAILED;
        else if (result == ESP_FAIL) status = DK_TX_CONTROLLER_ERROR;
        write_tx_result(bus, sequence, status);
    }
}
#endif

static void capture_task(void *arg)
{
    (void)arg;
    int64_t next_status = 0;
    int64_t next_flush = 0;
    capture_item_t item;
    while (true) {
#ifdef CONFIG_DASHKIT_USB_TX
        read_tx_requests();
#endif
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(2)) == pdTRUE) {
            uint8_t raw[DK_MAX_RAW];
            const can_frame_t *f = &item.frame.frame;
            common_header(raw, DK_FRAME, item.frame.bus_id, item.sequence, item.timestamp_us);
            raw[3] = (f->extended ? 1 : 0) | (f->rtr ? 2 : 0) | (f->fd ? 4 : 0) | (f->brs ? 8 : 0);
            dk_put(raw + 20, f->id, 4);
            static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
            raw[24] = f->fd ? lengths[f->dlc & 15] : (f->dlc > 8 ? 8 : f->dlc);
            if (!f->rtr) memcpy(raw + 25, f->data, raw[24]);
            write_record(raw, 25 + raw[24]);
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_status) {
            for (uint8_t bus = 0; bus < 2; bus++) {
                uint8_t raw[DK_MAX_RAW];
                portENTER_CRITICAL(&s_lock);
                common_header(raw, DK_STATUS, bus, s_sequence[bus], now);
                dk_put(raw + 20, s_queue_drops[bus], 4);
                dk_put(raw + 24, s_overflows[bus], 4);
                portEXIT_CRITICAL(&s_lock);
                dk_put(raw + 28, s_usb_aborts, 4);
                write_record(raw, 32);
            }
#ifdef CONFIG_DASHKIT_USB_TX
            if (atomic_load(&s_tx_ready)) write_tx_result(0, 0, DK_TX_READY);
#endif
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
