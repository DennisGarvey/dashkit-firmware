#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "mcp251xfd_classic_rx.h"

#define TEST_FIFO_STA(n) (0x060 + ((n) - 1) * 12)

typedef struct {
    uint32_t registers[0x400 / 4];
    unsigned fifo, tail, waiting, delivered, overflow_count, releases_until_failure;
    unsigned reads, largest_read, released, flag_reads;
    uint32_t pending_override;
    bool invalid_address, read_failure, overflow, has_tx_event, fd;
} mock_t;

static uint32_t fifo_base(const mock_t *m)
{
    if (m->fifo == 1) return m->has_tx_event ? 12 : 0;
    return (m->has_tx_event ? 84 : 64) + (m->fifo - 2) * 480;
}

static void put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = v >> (i * 8);
}

static esp_err_t read_bus(void *io, uint16_t addr, uint8_t *data, size_t len)
{
    mock_t *m = io;
    if (addr == 0x020) {
        assert(len == 4);
        ++m->flag_reads;
        put32(data, m->pending_override ? m->pending_override : (m->waiting ? 1u << m->fifo : 0));
        return ESP_OK;
    }
    if (m->pending_override && len == 8 && addr >= TEST_FIFO_STA(1) && addr <= TEST_FIFO_STA(5)) {
        m->fifo = (addr - TEST_FIFO_STA(1)) / 12 + 1;
        assert(addr == TEST_FIFO_STA(m->fifo) && (m->pending_override & (1u << m->fifo)));
    }
    if (addr == TEST_FIFO_STA(m->fifo)) {
        assert(len == 8);
        // Deliberately bogus producer index. The flags still describe the FIFO.
        put32(data, 31u << 8 | (m->waiting != 0) | (m->waiting >= 12 ? 2 : 0) |
                        (m->waiting == 24 ? 4 : 0) | (m->overflow ? 8 : 0));
        put32(data + 4, fifo_base(m) + 20 * m->tail + m->invalid_address);
        return ESP_OK;
    }

    assert(addr == 0x400 + fifo_base(m) + 20 * m->tail);
    assert(len % 20 == 0 && len / 20 <= m->waiting && m->tail + len / 20 <= 24);
    if (m->read_failure) return ESP_FAIL;
    ++m->reads;
    if (len > m->largest_read) m->largest_read = len;
    memset(data, 0, len);
    for (unsigned i = 0; i < len / 20; ++i) {
        uint8_t *object = data + 20 * i;
        put32(object, 0x123 | (0x23456u << 11));
        put32(object + 4, m->fd ? 0x98 : 0x18);
        put32(object + 8, 5000 + m->released + i);
        memset(object + 12, m->released + i, 8);
    }
    return ESP_OK;
}

static esp_err_t write_bus(void *io, uint16_t addr, uint32_t value)
{
    mock_t *m = io;
    if (addr == TEST_FIFO_STA(m->fifo) - 4 && (value & (1u << 8))) {
        if (!m->releases_until_failure) return ESP_FAIL;
        --m->releases_until_failure;
        assert(m->waiting);
        m->tail = (m->tail + 1) % 24;
        --m->waiting;
        ++m->released;
    } else if (addr == TEST_FIFO_STA(m->fifo)) {
        assert(!(value & 8));
        m->overflow = false;
    }
    m->registers[addr / 4] = value;
    return ESP_OK;
}

static void deliver(void *io, const can_frame_t *f, uint32_t counter, uint8_t fifo)
{
    mock_t *m = io;
    assert(fifo == m->fifo);
    assert(m->released == m->delivered + 1);
    assert(counter == 5000 + m->delivered);
    assert(f->id == ((0x123u << 18) | 0x23456u));
    assert(f->extended && !f->fd && !f->rtr && f->dlc == 8);
    for (unsigned i = 0; i < 8; ++i) assert(f->data[i] == m->delivered);
    ++m->delivered;
}

static void overflow(void *io)
{
    ++((mock_t *)io)->overflow_count;
}

static void test_layout(bool has_tx_event)
{
    mock_t m = {.fifo = 1, .tail = 22, .waiting = 24, .overflow = true, .releases_until_failure = 100,
                .has_tx_event = has_tx_event};
    mcp251xfd_classic_rx_t rx = {.io = &m, .read = read_bus, .write = write_bus,
                               .deliver = deliver, .overflow = overflow, .has_tx_event = has_tx_event};
    assert(mcp251xfd_classic_configure(&rx) == ESP_OK);
    assert(m.registers[0x1d0 / 4] == 0x85848381u);
    for (unsigned lane = 0; lane < 4; ++lane) {
        const unsigned fifo = lane ? lane + 2 : 1;
        assert(m.registers[(TEST_FIFO_STA(fifo) - 4) / 4] == (23u << 24 | 0x21));
        assert(m.registers[(0x1f0 + 8 * lane) / 4] == lane * 16);
        assert(m.registers[(0x1f4 + 8 * lane) / 4] == 0x30);
    }
    assert(mcp251xfd_classic_reset(&rx) == ESP_OK);
    m.invalid_address = true;
    assert(!mcp251xfd_classic_drain(&rx, -1) && !m.reads);
    m.invalid_address = false;
    m.read_failure = true;
    assert(!mcp251xfd_classic_drain(&rx, -1) && m.waiting == 24 && !m.delivered);
    assert(m.overflow_count == 1);
    m.read_failure = false;
    assert(mcp251xfd_classic_drain(&rx, -1) && m.tail == 0 && m.delivered == 2);
    m.fifo = 5;
    m.releases_until_failure = 3;
    assert(mcp251xfd_classic_drain(&rx, -1) && m.delivered == 5 && m.waiting == 19);
    m.releases_until_failure = 100;
    while (mcp251xfd_classic_drain(&rx, -1)) {}
    assert(m.delivered == 24 && !m.waiting && m.largest_read == 160);
    m.waiting = 1;
    m.fd = true;
    assert(mcp251xfd_classic_drain(&rx, -1));
    assert(m.delivered == 24 && !m.waiting);
}

static void test_priority(void)
{
    mock_t m = {.releases_until_failure = 10, .has_tx_event = true,
                .pending_override = (1u << 1) | (1u << 3) | (1u << 4) | (1u << 5)};
    mcp251xfd_classic_rx_t rx = {.io = &m, .read = read_bus, .write = write_bus,
                               .deliver = deliver, .overflow = overflow, .has_tx_event = true};
    m.waiting = 1;
    assert(mcp251xfd_classic_drain_pending(&rx, 0x120, m.pending_override));
    assert(m.fifo == 4 && m.flag_reads == 0);

    const unsigned order[] = {1, 4, 3, 4, 5};
    for (unsigned i = 0; i < sizeof(order) / sizeof(order[0]); ++i) {
        // Keep all lanes pending to exercise the schedule under sustained load.
        m.waiting = 1;
        assert(mcp251xfd_classic_drain(&rx, 0x120));
        assert(m.fifo == order[i]);
    }
    m.pending_override = 1u << 4;
    for (unsigned i = 0; i < 2; ++i) {
        m.waiting = 1;
        assert(mcp251xfd_classic_drain(&rx, 0x120));
        assert(m.fifo == 4);
    }
}

int main(void)
{
    test_layout(false);
    test_layout(true);
    test_priority();
    puts("Classic RX: layout, wrap, invalid address, SPI read and release failures, payload and timestamp passed");
}
