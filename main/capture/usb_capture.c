#include "usb_capture.h"
#include "capture_protocol.h"
#include "board.h"
#include <stdatomic.h>
#ifdef CONFIG_DASHKIT_USB_TX
#include "can_manager.h"
#endif

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
    item.timestamp_us = frame->timestamp_us ? frame->timestamp_us : esp_timer_get_time();
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

#ifdef CONFIG_DASHKIT_USB_TX
static atomic_bool s_tx_ready;

void usb_capture_enable_tx(void)
{
    atomic_store(&s_tx_ready, true);
}

static int64_t s_next_send;
static struct {
    can_frame_t frame;
    uint8_t mask[64];
    uint32_t sequence;
    uint8_t bus;
    bool automatic, pending;
    int64_t interval, next_repeat, end, lease;
} s_repeat;

static void write_tx_result(uint8_t type, uint8_t bus, uint32_t sequence, uint8_t status)
{
    uint8_t raw[DK_MAX_RAW];
    common_header(raw, type, bus, sequence, esp_timer_get_time());
    raw[20] = status;
    write_record(raw, 21);
}

static uint8_t send_frame(uint8_t bus, const can_frame_t *frame, int64_t now)
{
    if (now < s_next_send) return DK_TX_RATE_LIMIT;
    s_next_send = now + 100000;
    esp_err_t result = can_manager_send(bus, frame);
    if (result == ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        account_can_frame(bus, frame);
        portEXIT_CRITICAL(&s_lock);
        return DK_TX_QUEUED;
    }
    if (result == ESP_ERR_NO_MEM) return DK_TX_FIFO_FULL;
    if (result == ESP_ERR_TIMEOUT) return DK_TX_PREVIOUS_FAILED;
    if (result == ESP_FAIL) return DK_TX_CONTROLLER_ERROR;
    return DK_TX_REJECTED;
}

static void finish_repeat(uint8_t status)
{
    bool automatic = s_repeat.automatic;
    uint8_t bus = s_repeat.bus;
    uint32_t sequence = s_repeat.sequence;
    s_repeat.sequence = 0;
    s_repeat.pending = false;
    if (sequence)
        write_tx_result(automatic ? DK_TX_AUTO_RESULT : DK_TX_REPEAT_RESULT,
                        bus, sequence, status);
}

static void observe_repeat(const can_tagged_frame_t *received)
{
    if (s_repeat.sequence && received->bus_id == s_repeat.bus) {
        can_frame_t *wanted = &s_repeat.frame;
        const can_frame_t *actual = &received->frame;
        if (actual->id == wanted->id && actual->extended == wanted->extended) {
            uint8_t wanted_length = wanted->fd ? s_lengths[wanted->dlc & 15] : (wanted->dlc > 8 ? 8 : wanted->dlc);
            uint8_t actual_length = actual->fd ? s_lengths[actual->dlc & 15] : (actual->dlc > 8 ? 8 : actual->dlc);
            bool compatible = actual->rtr == wanted->rtr && actual->fd == wanted->fd &&
                actual->brs == wanted->brs && actual_length == wanted_length;
            bool changed = !compatible;
            if (compatible && !wanted->rtr)
                changed = dk_merge_masked(wanted->data, actual->data, s_repeat.mask, wanted_length);
            if (s_repeat.automatic && changed) s_repeat.pending = true;
        }
    }
}

static void service_repeat(void)
{
    int64_t now = esp_timer_get_time();
    if (!s_repeat.sequence) return;
    if (now >= s_repeat.end) {
        finish_repeat(DK_TX_REPEAT_FINISHED);
    } else if (now >= s_repeat.lease || !usb_serial_jtag_is_connected()) {
        finish_repeat(DK_TX_REPEAT_LEASE_EXPIRED);
    } else if ((s_repeat.automatic && s_repeat.pending) ||
               (!s_repeat.automatic && now >= s_repeat.next_repeat)) {
        uint8_t result = send_frame(s_repeat.bus, &s_repeat.frame, now);
        if (result == DK_TX_QUEUED) {
            s_repeat.pending = false;
            if (!s_repeat.automatic) s_repeat.next_repeat = now + s_repeat.interval;
        } else if (result != DK_TX_RATE_LIMIT) {
            finish_repeat(result);
        }
    }
}

static void read_tx_requests(void)
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
        if (!size || dk_get(raw + 4, 4) != s_boot_id || !atomic_load(&s_tx_ready)) continue;

        uint32_t sequence = dk_get(raw + 8, 4);
        uint8_t bus = raw[2];
        int64_t now = esp_timer_get_time();
        if (raw[1] == DK_TX_REPEAT_CONTROL) {
            bool matches = sequence == s_repeat.sequence && bus == s_repeat.bus;
            bool cancel = matches && raw[20] == DK_TX_CANCEL;
            if (matches && !cancel && now < s_repeat.lease) s_repeat.lease = now + 1000000;
            if (cancel) finish_repeat(DK_TX_REPEAT_CANCELLED);
            continue;
        }
        bool repeat = raw[1] == DK_TX_REPEAT;
        bool automatic = raw[1] == DK_TX_AUTO;
        if (!sequence || (!repeat && !automatic && raw[1] != DK_TX_REQUEST)) continue;
        uint8_t result_type = automatic ? DK_TX_AUTO_RESULT :
                              (repeat ? DK_TX_REPEAT_RESULT : DK_TX_RESULT);
        bool repeat_busy = s_repeat.sequence != 0;
        if (repeat_busy) {
            write_tx_result(result_type, bus, sequence,
                            repeat || automatic ? DK_TX_REPEAT_BUSY : DK_TX_RATE_LIMIT);
            continue;
        }
        if (!repeat && !automatic && now < s_next_send) {
            write_tx_result(result_type, bus, sequence, DK_TX_RATE_LIMIT);
            continue;
        }
        can_frame_t frame = {
            .id = dk_get(raw + 20, 4),
            .extended = (raw[3] & 1) != 0,
            .rtr = (raw[3] & 2) != 0,
            .fd = (raw[3] & 4) != 0,
            .brs = (raw[3] & 8) != 0,
        };
        while (s_lengths[frame.dlc] != raw[24]) ++frame.dlc;
        if (!frame.rtr) memcpy(frame.data, raw + 25, raw[24]);
        if (repeat || automatic) {
            s_repeat.frame = frame;
            memcpy(s_repeat.mask, raw + 25 + raw[24], raw[24]);
            s_repeat.bus = bus;
            s_repeat.sequence = sequence;
            s_repeat.automatic = automatic;
            s_repeat.pending = true;
            s_repeat.interval = automatic ? 0 : 1000000 / dk_get(raw + 12, 4);
            s_repeat.next_repeat = now;
            s_repeat.end = now + 1000 * dk_get(raw + 16, 4);
            s_repeat.lease = now + 1000000;
            write_tx_result(result_type, bus, sequence, DK_TX_REPEAT_STARTED);
        } else {
            uint8_t status = send_frame(bus, &frame, now);
            write_tx_result(result_type, bus, sequence, status);
        }
    }
}
#endif

static void write_capture_item(const capture_item_t *item)
{
    uint8_t raw[DK_MAX_RAW];
    const can_frame_t *f = &item->frame.frame;
    common_header(raw, DK_FRAME, item->frame.bus_id, item->sequence, item->timestamp_us);
    raw[3] = (f->extended ? 1 : 0) | (f->rtr ? 2 : 0) | (f->fd ? 4 : 0) | (f->brs ? 8 : 0);
    dk_put(raw + 20, f->id, 4);
    raw[24] = f->fd ? s_lengths[f->dlc & 15] : (f->dlc > 8 ? 8 : f->dlc);
    if (!f->rtr) memcpy(raw + 25, f->data, raw[24]);
    write_record(raw, 25 + raw[24]);
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
#ifdef CONFIG_DASHKIT_USB_TX
        read_tx_requests();
        service_repeat();
#endif
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(2)) == pdTRUE) {
#ifdef CONFIG_DASHKIT_USB_TX
            observe_repeat(&item.frame);
#endif
            write_capture_item(&item);
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
#ifdef CONFIG_DASHKIT_USB_TX
            if (atomic_load(&s_tx_ready))
                write_tx_result(DK_TX_RESULT, DK_TX_REPEAT_CAPABILITY, 0, DK_TX_READY);
            if (atomic_load(&s_tx_ready))
                write_tx_result(DK_TX_AUTO_RESULT, 0, 0, DK_TX_READY);
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
