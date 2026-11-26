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

// Bound capture memory so it can coexist with NimBLE on internal RAM.
#define CAPTURE_QUEUE_DEPTH 256

typedef struct {
    can_tagged_frame_t frame;
    uint64_t timestamp_us;
    uint32_t sequence, generation;
} capture_item_t;

static QueueHandle_t s_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_boot_id, s_next_generation;
static atomic_uint s_capture_generation;
static uint64_t s_session_id;
static int64_t s_session_deadline;
static uint32_t s_sequence[2], s_queue_drops[2], s_overflows[2], s_usb_aborts;

// SPI2 is shared by both MCP2518FD controllers. Count clocked bits, which
// excludes mutex waits and the time spent setting up transactions.
static atomic_uint s_spi_bits[2];
static uint32_t s_serial_bytes;
static uint32_t s_can_bits[2];

void usb_capture_account_spi(uint8_t bus_id, uint32_t clock_bits)
{
    if (bus_id < 2) atomic_fetch_add_explicit(&s_spi_bits[bus_id], clock_bits, memory_order_relaxed);
}

// Caller holds s_lock. Include frame overhead and intermission, with a
// stuffing allowance on the fields eligible for variable bit stuffing.
static void account_can_frame(uint8_t bus, const can_frame_t *frame)
{
    const uint32_t bytes = frame->dlc > 8 ? 8 : frame->dlc;
    const uint32_t header = frame->extended ? 54 : 34;
    s_can_bits[bus] += (header + bytes * 8) * 6 / 5 + 13;
}

void usb_capture_account_overflows(uint8_t bus_id, uint32_t count)
{
    if (bus_id > 1 || count == 0) return;
    portENTER_CRITICAL(&s_lock);
    s_overflows[bus_id] += count;
    portEXIT_CRITICAL(&s_lock);
}

void usb_capture_submit(const can_tagged_frame_t *frame)
{
    if (!s_queue || frame->bus_id > 1) return;
    const uint32_t generation = atomic_load_explicit(&s_capture_generation, memory_order_relaxed);
    if (!generation) return;
    capture_item_t item = { .frame = *frame, .generation = generation };
    portENTER_CRITICAL(&s_lock);
    if (generation != atomic_load_explicit(&s_capture_generation, memory_order_relaxed)) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
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

static bool flush_records(void)
{
    if (!s_output_size) return true;
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
    const bool complete = sent == s_output_size;
    s_output_size = 0;
    return complete;
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

static void set_capture_generation(uint32_t generation)
{
    portENTER_CRITICAL(&s_lock);
    atomic_store_explicit(&s_capture_generation, generation, memory_order_relaxed);
    portEXIT_CRITICAL(&s_lock);
}

static void stop_capture_session(void)
{
    set_capture_generation(0);
    s_session_id = 0;
    s_output_size = 0;
    xQueueReset(s_queue);
}

static void handle_session_request(uint64_t id, uint8_t opcode, int64_t now)
{
    if (opcode == DK_SESSION_START) {
        const bool new_session = id != s_session_id;
        if (new_session) stop_capture_session();
        uint8_t raw[DK_MAX_RAW];
        common_header(raw, DK_SESSION, 0, 0, id);
        raw[20] = DK_SESSION_READY;
        write_record(raw, 21);
        // ESP-IDF delivers any previously written bytes before this reply.
        if (flush_records()) {
            s_session_id = id;
            s_session_deadline = now + 3000000;
            if (new_session) {
                if (++s_next_generation == 0) ++s_next_generation;
                set_capture_generation(s_next_generation);
            }
        }
    } else if (id == s_session_id && now < s_session_deadline) {
        if (opcode == DK_SESSION_KEEPALIVE) s_session_deadline = now + 3000000;
        if (opcode == DK_SESSION_STOP) stop_capture_session();
    }
}

static void read_usb_requests(void)
{
    static uint8_t pending[DK_MAX_WIRE];
    static size_t count;
    static bool discarding;
    uint8_t bytes[256];
    int n = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), 0);
    if (n > 0) s_serial_bytes += (uint32_t)n;
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
        if (!size) continue;
        if (raw[1] == DK_SESSION) {
            handle_session_request(dk_get(raw + 12, 8), raw[20], esp_timer_get_time());
            continue;
        }
    }
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
    uint32_t can_bits[2];
    portENTER_CRITICAL(&s_lock);
    for (int bus = 0; bus < 2; bus++) {
        can_bits[bus] = s_can_bits[bus];
        s_can_bits[bus] = 0;
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
            uint64_t can_us = (uint64_t)can_bits[i] * 1000000 / CAN_NOMINAL_BITRATE;
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
    int64_t next_control_poll = 0;
    capture_item_t item;
    while (true) {
        const int64_t loop_time = esp_timer_get_time();
        if (s_session_id && loop_time >= s_session_deadline) stop_capture_session();
        if (loop_time >= next_control_poll) {
            read_usb_requests();
            next_control_poll = loop_time + 1000;
        }
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(2)) == pdTRUE &&
            item.generation == atomic_load_explicit(&s_capture_generation, memory_order_relaxed)) {
            uint8_t raw[DK_MAX_RAW];
            const can_frame_t *f = &item.frame.frame;
            common_header(raw, DK_FRAME, item.frame.bus_id, item.sequence, item.timestamp_us);
            raw[3] = (f->extended ? 1 : 0) | (f->fd ? 4 : 0) | (f->fd && f->brs ? 8 : 0);
            dk_put(raw + 20, f->id, 4);
            raw[24] = f->dlc > 8 ? 8 : f->dlc;
            memcpy(raw + 25, f->data, raw[24]);
            write_record(raw, 25 + raw[24]);
        }
        int64_t now = esp_timer_get_time();
        if (s_session_id && now >= next_status) {
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
        .tx_buffer_size = 8192,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&config);
    if (err != ESP_OK) return err;
    s_queue = xQueueCreate(CAPTURE_QUEUE_DEPTH, sizeof(capture_item_t));
    if (!s_queue) {
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }
    s_boot_id = esp_random();
    if (xTaskCreatePinnedToCore(capture_task, "can2usb", 4096, NULL, 5, NULL, 0) != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        usb_serial_jtag_driver_uninstall();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
