#include "usb_capture.h"
#include "capture_protocol.h"

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
