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
    uint32_t sequence, generation;
} capture_item_t;

static QueueHandle_t s_queue;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_boot_id, s_next_generation;
static atomic_uint s_capture_generation;
static uint64_t s_session_id;
static int64_t s_session_deadline;
static uint32_t s_sequence[2], s_queue_drops[2], s_overflows[2];
static atomic_uint s_usb_aborts;
static const uint8_t s_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// SPI2 is shared by both MCP2518FD controllers. Count clocked bits, which
// excludes mutex waits and the time spent setting up transactions.
static atomic_uint s_spi_bits[2];
static atomic_uint s_serial_bytes;
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

#ifdef CONFIG_DASHKIT_USB_TX
static TaskHandle_t s_capture_task;
static QueueHandle_t s_usb_output_queue;

void usb_capture_notify_tx_event(void)
{
    if (s_capture_task) xTaskNotifyGive(s_capture_task);
}

static portMUX_TYPE s_repeat_lock = portMUX_INITIALIZER_UNLOCKED;
typedef struct {
    uint16_t start_bit;
    uint8_t size;
    bool little_endian;
} counter_descriptor_t;

static struct {
    can_frame_t frame;
    uint8_t mask[64], selector[64];
    uint32_t sequence, generation;
    uint8_t bus, finish_status;
    bool automatic, pending, retry_pending, automatic_needs_send, increment_counter, recalculate_checksum;
    uint8_t checksum_byte;
    counter_descriptor_t counter;
    int64_t interval, next_repeat, end, lease;
} s_repeat;

static void increment_counter(can_frame_t *frame, const counter_descriptor_t *counter)
{
    dk_increment_counter(frame->data, counter->start_bit, counter->size,
                         counter->little_endian);
}

// Refresh the payload at CAN ingress even if USB capture is backlogged.
static void observe_repeat(const can_tagged_frame_t *received, uint32_t generation)
{
    const can_frame_t *actual = &received->frame;
    can_frame_t *wanted = &s_repeat.frame;
    bool wake = false;
    portENTER_CRITICAL(&s_repeat_lock);
    if (s_repeat.sequence && s_repeat.generation == generation && received->bus_id == s_repeat.bus &&
        actual->id == wanted->id && actual->extended == wanted->extended &&
        !actual->rtr && actual->fd == wanted->fd && actual->brs == wanted->brs) {
        uint8_t actual_length = actual->fd ? s_lengths[actual->dlc & 15] : (actual->dlc > 8 ? 8 : actual->dlc);
        uint8_t wanted_length = s_lengths[wanted->dlc];
        if (actual_length == wanted_length &&
            dk_matches_mask(wanted->data, actual->data, s_repeat.selector, wanted_length)) {
            const bool changed = dk_merge_masked(wanted->data, actual->data, s_repeat.mask, wanted_length);
            if (s_repeat.automatic) {
                wake = changed && !s_repeat.pending;
                s_repeat.pending = changed;
                s_repeat.automatic_needs_send = changed;
                if (!changed) s_repeat.retry_pending = false;
            }
        }
    }
    portEXIT_CRITICAL(&s_repeat_lock);
    if (wake) usb_capture_notify_tx_event();
}
#endif

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
    const uint32_t generation = atomic_load_explicit(&s_capture_generation, memory_order_relaxed);
    if (!generation) return;
#ifdef CONFIG_DASHKIT_USB_TX
    observe_repeat(frame, generation);
#endif
    capture_item_t item = { .frame = *frame, .generation = generation };
    portENTER_CRITICAL(&s_lock);
    if (generation != atomic_load_explicit(&s_capture_generation, memory_order_relaxed)) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
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
typedef struct {
    size_t size;
    uint8_t bytes[2048];
} usb_output_t;

static usb_output_t s_output;

static bool send_usb_output(const usb_output_t *output)
{
    size_t sent = 0;
    while (sent < output->size) {
        int n = usb_serial_jtag_write_bytes(output->bytes + sent, output->size - sent, pdMS_TO_TICKS(10));
        if (n <= 0) {
            atomic_fetch_add_explicit(&s_usb_aborts, 1, memory_order_relaxed);
            return false;
        }
        atomic_fetch_add_explicit(&s_serial_bytes, (unsigned)n, memory_order_relaxed);
        sent += (size_t)n;
    }
    return true;
}

#ifdef CONFIG_DASHKIT_USB_TX
static void usb_output_task(void *arg)
{
    (void)arg;
    usb_output_t output;
    while (true) {
        if (xQueueReceive(s_usb_output_queue, &output, portMAX_DELAY) == pdTRUE) {
            send_usb_output(&output);
        }
    }
}
#endif

static bool flush_records(void)
{
    if (!s_output.size) return true;
#ifdef CONFIG_DASHKIT_USB_TX
    // Never let a blocked USB reader hold up CAN scheduling or ACK handling.
    const bool sent = xQueueSend(s_usb_output_queue, &s_output, 0) == pdTRUE;
    if (!sent) atomic_fetch_add_explicit(&s_usb_aborts, 1, memory_order_relaxed);
#else
    const bool sent = send_usb_output(&s_output);
#endif
    s_output.size = 0;
    return sent;
}

static void write_record(uint8_t *raw, size_t size)
{
    if (sizeof(s_output.bytes) - s_output.size < DK_MAX_WIRE + 1) flush_records();
    s_output.bytes[s_output.size++] = 0;
    s_output.size += dk_encode(raw, size, s_output.bytes + s_output.size);
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
    s_output.size = 0;
    xQueueReset(s_queue);
#ifdef CONFIG_DASHKIT_USB_TX
    xQueueReset(s_usb_output_queue);
    if (s_repeat.sequence && !s_repeat.finish_status)
        s_repeat.finish_status = DK_TX_REPEAT_LEASE_EXPIRED;
#endif
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
        // Only one writer hands batches to ESP-IDF. An older batch it already
        // took stays ahead of this reply, even across a queue reset.
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

static void write_can_frame(uint8_t type, uint8_t bus, uint32_t sequence,
                            uint64_t timestamp_us, const can_frame_t *frame)
{
    uint8_t raw[DK_MAX_RAW];
    common_header(raw, type, bus, sequence, timestamp_us);
    raw[3] = (frame->extended ? 1 : 0) | (frame->rtr ? 2 : 0) |
             (frame->fd ? 4 : 0) | (frame->brs ? 8 : 0);
    raw[24] = frame->fd ? s_lengths[frame->dlc & 15] : (frame->dlc > 8 ? 8 : frame->dlc);
    dk_put(raw + 20, frame->id, 4);
    if (!frame->rtr) memcpy(raw + 25, frame->data, raw[24]);
    write_record(raw, 25 + raw[24]);
}

#ifdef CONFIG_DASHKIT_USB_TX
static atomic_bool s_tx_ready;

void usb_capture_enable_tx(void)
{
    atomic_store(&s_tx_ready, true);
}

static int64_t s_next_manual_send;
static struct {
    can_frame_t frame;
    int64_t deadline;
    uint32_t sequence, generation;
    uint8_t bus;
} s_pending_tx;

static void write_tx_result(uint8_t type, uint8_t bus, uint32_t sequence, uint8_t status)
{
    uint8_t raw[DK_MAX_RAW];
    common_header(raw, type, bus, sequence, esp_timer_get_time());
    raw[20] = status;
    write_record(raw, 21);
}

static uint8_t send_frame(uint8_t bus, const can_frame_t *frame, uint32_t sequence)
{
    if (s_pending_tx.deadline) return DK_TX_FIFO_FULL;
    esp_err_t result = can_manager_send(bus, frame);
    if (result == ESP_OK) {
        s_pending_tx.frame = *frame;
        s_pending_tx.bus = bus;
        s_pending_tx.sequence = sequence;
        s_pending_tx.generation = s_next_generation;
        s_pending_tx.deadline = esp_timer_get_time() + 100000;
        return DK_TX_QUEUED;
    }
    if (result == ESP_ERR_NO_MEM) return DK_TX_FIFO_FULL;
    if (result == ESP_FAIL) return DK_TX_CONTROLLER_ERROR;
    return DK_TX_REJECTED;
}

static void update_repeat_stop(int64_t now)
{
    if (!s_repeat.sequence || s_repeat.finish_status) return;
    if (now >= s_repeat.end) {
        s_repeat.finish_status = DK_TX_REPEAT_FINISHED;
    } else if (now >= s_repeat.lease || !usb_serial_jtag_is_connected()) {
        s_repeat.finish_status = DK_TX_REPEAT_LEASE_EXPIRED;
    }
}

static void service_tx_feedback(void)
{
    if (!s_pending_tx.deadline) return;
    const uint32_t generation = atomic_load_explicit(&s_capture_generation, memory_order_relaxed);
    const bool session_ended = s_pending_tx.generation != generation;
    const bool stopping = session_ended ||
        (s_pending_tx.sequence && s_pending_tx.sequence == s_repeat.sequence && s_repeat.finish_status);
    const bool expired = esp_timer_get_time() >= s_pending_tx.deadline;
    const uint8_t bus = s_pending_tx.bus;
    can_tx_diagnostics_t diagnostics = {0};
    const esp_err_t result = can_manager_poll_tx(bus, expired || stopping, &diagnostics);
    const bool completed = result == ESP_OK || result == ESP_ERR_TIMEOUT ||
                           result == ESP_ERR_INVALID_RESPONSE;
    if (!completed && !expired) return;

    const uint32_t sequence = s_pending_tx.sequence;
    s_pending_tx.sequence = 0;
    // A failed SPI operation leaves the controller slot unavailable until
    // polling can confirm completion or clear it.
    if (completed) s_pending_tx.deadline = 0;
    if (result == ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        account_can_frame(bus, &s_pending_tx.frame);
        portEXIT_CRITICAL(&s_lock);
    }
    if (!sequence || session_ended) return;
    if (result == ESP_OK) {
        write_can_frame(DK_TX_FRAME, bus, sequence, esp_timer_get_time(), &s_pending_tx.frame);
    }
    const bool timed = sequence == s_repeat.sequence;
    if (timed && result == ESP_ERR_TIMEOUT && diagnostics.arbitration_only) {
        if (!stopping) {
            portENTER_CRITICAL(&s_repeat_lock);
            s_repeat.retry_pending = !s_repeat.automatic || s_repeat.automatic_needs_send;
            portEXIT_CRITICAL(&s_repeat_lock);
        }
        return;
    }
    // A deliberate abort may clear a pending frame without an outcome.
    if (stopping && result == ESP_ERR_INVALID_RESPONSE) return;
    const uint8_t feedback = result == ESP_OK ? DK_TX_ACKNOWLEDGED :
                             result == ESP_ERR_TIMEOUT ? DK_TX_UNCONFIRMED : DK_TX_OUTCOME_UNKNOWN;
    if (feedback != DK_TX_ACKNOWLEDGED && diagnostics.valid) {
        uint8_t raw[DK_MAX_RAW];
        common_header(raw, DK_TX_FEEDBACK, bus, sequence, esp_timer_get_time());
        raw[20] = feedback;
        dk_put(raw + 21, diagnostics.fifo_status, 4);
        dk_put(raw + 25, diagnostics.bus_diagnostics, 4);
        dk_put(raw + 29, diagnostics.error_counters, 4);
        write_record(raw, DK_TX_FEEDBACK_DIAGNOSTICS_SIZE);
    } else {
        write_tx_result(DK_TX_FEEDBACK, bus, sequence, feedback);
    }
    if (sequence == s_repeat.sequence && feedback != DK_TX_ACKNOWLEDGED && !s_repeat.finish_status)
        s_repeat.finish_status = feedback == DK_TX_UNCONFIRMED ? DK_TX_PREVIOUS_FAILED : DK_TX_CONTROLLER_ERROR;
}

static void finish_repeat(uint8_t status)
{
    bool automatic = s_repeat.automatic;
    uint8_t bus = s_repeat.bus;
    uint32_t sequence = s_repeat.sequence;
    portENTER_CRITICAL(&s_repeat_lock);
    s_repeat.sequence = 0;
    s_repeat.pending = false;
    portEXIT_CRITICAL(&s_repeat_lock);
    if (sequence && s_repeat.generation == atomic_load_explicit(&s_capture_generation, memory_order_relaxed))
        write_tx_result(automatic ? DK_TX_AUTO_RESULT : DK_TX_REPEAT_RESULT,
                        bus, sequence, status);
}

static void service_repeat(void)
{
    int64_t now = esp_timer_get_time();
    if (!s_repeat.sequence) return;
    update_repeat_stop(now);
    if (s_repeat.finish_status) {
        if (!s_pending_tx.sequence) finish_repeat(s_repeat.finish_status);
        return;
    }
    if (s_pending_tx.deadline) return;
    can_frame_t frame;
    portENTER_CRITICAL(&s_repeat_lock);
    const bool due = s_repeat.retry_pending ||
                     (s_repeat.automatic ? s_repeat.pending : now >= s_repeat.next_repeat);
    if (due) {
        frame = s_repeat.frame;
        if (s_repeat.increment_counter) increment_counter(&frame, &s_repeat.counter);
        if (s_repeat.recalculate_checksum)
            dk_recalculate_tesla_checksum(frame.data, s_lengths[frame.dlc], frame.id,
                                          s_repeat.checksum_byte);
        s_repeat.pending = false;
        s_repeat.retry_pending = false;
    }
    portEXIT_CRITICAL(&s_repeat_lock);
    if (!due) return;

    uint8_t result = send_frame(s_repeat.bus, &frame, s_repeat.sequence);
    if (result == DK_TX_QUEUED) {
        if (!s_repeat.automatic) s_repeat.next_repeat = now + s_repeat.interval;
    } else {
        finish_repeat(result);
    }
}

#endif

static void read_usb_requests(void)
{
    static uint8_t pending[DK_MAX_WIRE];
    static size_t count;
    static bool discarding;
    uint8_t bytes[256];
    int n = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), 0);
    if (n > 0) atomic_fetch_add_explicit(&s_serial_bytes, (unsigned)n, memory_order_relaxed);
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
#ifdef CONFIG_DASHKIT_USB_TX
        if (!s_session_id || dk_get(raw + 4, 4) != s_boot_id || !atomic_load(&s_tx_ready)) continue;

        uint32_t sequence = dk_get(raw + 8, 4);
        uint8_t bus = raw[2];
        int64_t now = esp_timer_get_time();
        if (raw[1] == DK_TX_REPEAT_CONTROL) {
            bool matches = sequence == s_repeat.sequence && bus == s_repeat.bus;
            bool cancel = matches && raw[20] == DK_TX_CANCEL;
            if (matches && !cancel && now < s_repeat.lease) s_repeat.lease = now + 1000000;
            if (cancel && !s_repeat.finish_status) s_repeat.finish_status = DK_TX_REPEAT_CANCELLED;
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
        if (s_pending_tx.deadline) {
            write_tx_result(result_type, bus, sequence, DK_TX_FIFO_FULL);
            continue;
        }
        if (!repeat && !automatic && now < s_next_manual_send) {
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
        const bool increment = (raw[3] & DK_TX_INCREMENT_COUNTER) != 0;
        const bool checksum = (raw[3] & DK_TX_RECALCULATE_CHECKSUM) != 0;
        counter_descriptor_t counter = {0};
        if (increment) {
            const size_t at = size - 2 - (checksum ? 1 : 0) - 3;
            counter.start_bit = (uint16_t)dk_get(raw + at, 2);
            counter.size = raw[at + 2] & 0x7f;
            counter.little_endian = (raw[at + 2] & 0x80) != 0;
        }
        const uint8_t checksum_byte = checksum ? raw[size - 3] : 0;
        if (repeat || automatic) {
            if (frame.rtr) {
                write_tx_result(result_type, bus, sequence, DK_TX_REJECTED);
                continue;
            }
            const bool has_selector = size == (size_t)(27 + 3 * raw[24]);
            portENTER_CRITICAL(&s_repeat_lock);
            s_repeat.frame = frame;
            memcpy(s_repeat.mask, raw + 25 + raw[24], raw[24]);
            memset(s_repeat.selector, 0, raw[24]);
            if (has_selector) memcpy(s_repeat.selector, raw + 25 + 2 * raw[24], raw[24]);
            s_repeat.bus = bus;
            s_repeat.sequence = sequence;
            s_repeat.generation = s_next_generation;
            s_repeat.automatic = automatic;
            s_repeat.pending = true;
            s_repeat.retry_pending = false;
            s_repeat.automatic_needs_send = true;
            s_repeat.increment_counter = increment;
            s_repeat.counter = counter;
            s_repeat.recalculate_checksum = checksum;
            s_repeat.checksum_byte = checksum_byte;
            portEXIT_CRITICAL(&s_repeat_lock);
            s_repeat.finish_status = 0;
            uint32_t hz = dk_get(raw + 12, 4);
            s_repeat.interval = automatic ? 0 : (1000000 + hz - 1) / hz;
            s_repeat.next_repeat = now;
            s_repeat.end = now + 1000 * dk_get(raw + 16, 4);
            s_repeat.lease = now + 1000000;
            write_tx_result(result_type, bus, sequence, DK_TX_REPEAT_STARTED);
        } else {
            s_next_manual_send = now + 100000;
            if (increment) increment_counter(&frame, &counter);
            if (checksum)
                dk_recalculate_tesla_checksum(frame.data, s_lengths[frame.dlc], frame.id,
                                              checksum_byte);
            uint8_t status = send_frame(bus, &frame, sequence);
            write_tx_result(result_type, bus, sequence, status);
        }
#endif
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
    const uint32_t serial = atomic_exchange_explicit(&s_serial_bytes, 0, memory_order_relaxed);

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
#ifndef CONFIG_DASHKIT_USB_TX
    int64_t next_control_poll = 0;
#endif
    capture_item_t item;
    while (true) {
        const int64_t loop_time = esp_timer_get_time();
        if (s_session_id && loop_time >= s_session_deadline) stop_capture_session();
#ifdef CONFIG_DASHKIT_USB_TX
        read_usb_requests();
        update_repeat_stop(esp_timer_get_time());
        service_tx_feedback();
        service_repeat();
        const bool timed = s_pending_tx.deadline || s_repeat.sequence;
        if (xQueueReceive(s_queue, &item, timed ? 0 : pdMS_TO_TICKS(2)) == pdTRUE) {
            if (item.generation == atomic_load_explicit(&s_capture_generation, memory_order_relaxed))
                write_can_frame(DK_FRAME, item.frame.bus_id, item.sequence, item.timestamp_us, &item.frame.frame);
        } else if (timed) {
            // RX and TX events can wake this immediately. A tick timeout
            // still services deadlines and covers missed GPIO edges.
            ulTaskNotifyTake(pdTRUE, 1);
        }
#else
        if (loop_time >= next_control_poll) {
            read_usb_requests();
            next_control_poll = loop_time + 1000;
        }
        if (xQueueReceive(s_queue, &item, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (item.generation == atomic_load_explicit(&s_capture_generation, memory_order_relaxed))
                write_can_frame(DK_FRAME, item.frame.bus_id, item.sequence, item.timestamp_us, &item.frame.frame);
        }
#endif
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
                dk_put(raw + 28, atomic_load_explicit(&s_usb_aborts, memory_order_relaxed), 4);
                write_record(raw, DK_STATUS_SIZE);
            }
#ifdef CONFIG_DASHKIT_USB_TX
            if (atomic_load(&s_tx_ready)) {
                write_tx_result(DK_TX_RESULT, DK_TX_CAPABILITY, 0, DK_TX_READY);
            }
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
#ifdef CONFIG_DASHKIT_USB_TX
    s_usb_output_queue = xQueueCreate(4, sizeof(usb_output_t));
    if (!s_usb_output_queue) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(usb_output_task, "usb_output", 4096, NULL, 6, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    TaskHandle_t *capture_handle = &s_capture_task;
#else
    TaskHandle_t *capture_handle = NULL;
#endif
    if (xTaskCreatePinnedToCore(capture_task, "can2usb", 4096, NULL, 5, capture_handle, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}
