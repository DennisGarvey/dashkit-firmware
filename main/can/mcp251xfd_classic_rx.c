#include "mcp251xfd_classic_rx.h"

#include <string.h>

#include "mcp251xfd_rx_clock.h"

#define REG_C1RXIF       0x020
#define REG_C1FIFOCON(n) (0x05C + ((n) - 1) * 12)
#define REG_C1FIFOSTA(n) (0x060 + ((n) - 1) * 12)
#define REG_C1FLTCON(n)  (0x1D0 + (n) * 4)
#define REG_C1FLTOBJ(n)  (0x1F0 + (n) * 8)
#define REG_C1MASK(n)    (0x1F4 + (n) * 8)
#define RAM_BASE         0x400

#define FIFO_NOT_EMPTY  (1u << 0)
#define FIFO_HALF_FULL  (1u << 1)
#define FIFO_FULL       (1u << 2)
#define FIFO_OVERFLOW   (1u << 3)
#define FIFO_INCREMENT  (1u << 8)
#define FIFO_RESET      (1u << 10)
#define FIFO_RX_CONFIG  (((MCP_CLASSIC_FIFO_DEPTH - 1u) << 24) | MCP251XFD_RX_TIMESTAMP_ENABLE | FIFO_NOT_EMPTY)

#define LANE_SHIFT      4
#define TEF_BYTES       12
#define USB_TX_BYTES    72
#define PASSIVE_TX_BYTES (4 * 16)
#define LANE_BYTES      (MCP_CLASSIC_FIFO_DEPTH * MCP_CLASSIC_RX_BYTES)

_Static_assert(TEF_BYTES + USB_TX_BYTES + MCP_CLASSIC_FIFO_COUNT * LANE_BYTES <= 2048,
               "MCP251xfd FIFO layout exceeds controller RAM");
_Static_assert(MCP_CLASSIC_CHUNK_FRAMES <= MCP_CLASSIC_FIFO_DEPTH / 2,
               "Receive bursts must fit within the half-full threshold");

static unsigned fifo_for_lane(unsigned lane)
{
    // FIFO 2 is reserved for TX.
    return lane == 0 ? 1 : lane + 2;
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

esp_err_t mcp251xfd_classic_configure(mcp251xfd_classic_rx_t *rx)
{
    for (unsigned word = 0; word < 8; ++word) {
        esp_err_t err = rx->write(rx->io, REG_C1FLTCON(word), 0);
        if (err != ESP_OK) return err;
    }

    uint32_t filters = 0;
    for (unsigned lane = 0; lane < MCP_CLASSIC_FIFO_COUNT; ++lane) {
        const unsigned fifo = fifo_for_lane(lane);
        esp_err_t err = rx->write(rx->io, REG_C1FIFOCON(fifo), FIFO_RX_CONFIG);
        if (err == ESP_OK) err = rx->write(rx->io, REG_C1FLTOBJ(lane), lane << LANE_SHIFT);
        if (err == ESP_OK) err = rx->write(rx->io, REG_C1MASK(lane), 3u << LANE_SHIFT);
        if (err != ESP_OK) return err;
        filters |= (0x80u | fifo) << (8 * lane);
    }
    return rx->write(rx->io, REG_C1FLTCON(0), filters);
}

esp_err_t mcp251xfd_classic_reset(mcp251xfd_classic_rx_t *rx)
{
    rx->next_lane = 0;
    rx->prefer_other_lane = false;
    for (unsigned lane = 0; lane < MCP_CLASSIC_FIFO_COUNT; ++lane) {
        esp_err_t err = rx->write(rx->io, REG_C1FIFOCON(fifo_for_lane(lane)), FIFO_RX_CONFIG | FIFO_RESET);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void deliver_frame(mcp251xfd_classic_rx_t *rx, const uint8_t *object, uint8_t fifo)
{
    const uint32_t id = read_le32(object);
    const uint32_t header = read_le32(object + 4);
    // Passive mode can see an FD object. Its payload may be truncated in a
    // Classic-sized FIFO, so leave it out of the Classic-only stream.
    if (header & (1u << 7)) return;
    can_frame_t frame = {
        .id = id & 0x7ff,
        .dlc = (header & 15) > 8 ? 8 : header & 15,
        .extended = (header & (1u << 4)) != 0,
        .rtr = (header & (1u << 5)) != 0,
    };
    if (frame.extended) frame.id = (frame.id << 18) | ((id >> 11) & 0x3ffff);
    if (!frame.rtr) memcpy(frame.data, object + 12, frame.dlc);
    rx->deliver(rx->io, &frame, read_le32(object + 8), fifo);
}

static bool drain_lane(mcp251xfd_classic_rx_t *rx, unsigned lane)
{
    const unsigned fifo = fifo_for_lane(lane);
    uint8_t status[8];
    if (rx->read(rx->io, REG_C1FIFOSTA(fifo), status, sizeof(status)) != ESP_OK) return false;

    const uint32_t sta = read_le32(status);
    const uint32_t ua = read_le32(status + 4);
    // Controller RAM is allocated in FIFO number order, after the TEF.
    const uint32_t start = rx->has_tx_event ? TEF_BYTES : 0;
    const uint32_t tx_bytes = rx->has_tx_event ? USB_TX_BYTES : PASSIVE_TX_BYTES;
    const uint32_t base = start + lane * LANE_BYTES + (lane ? tx_bytes : 0);
    if (ua < base || ua >= base + LANE_BYTES || ((ua - base) % MCP_CLASSIC_RX_BYTES) != 0) return false;

    if ((sta & FIFO_OVERFLOW) &&
        rx->write(rx->io, REG_C1FIFOSTA(fifo), sta & ~FIFO_OVERFLOW) == ESP_OK) {
        rx->overflow(rx->io);
    }
    if (!(sta & FIFO_NOT_EMPTY)) return false;

    // Avoid the silicon erratum that can misreport the producer index.
    // A half-full or full flag guarantees enough queued frames for a burst.
    unsigned count = (sta & (FIFO_HALF_FULL | FIFO_FULL)) ? MCP_CLASSIC_CHUNK_FRAMES : 1;
    const unsigned contiguous = (base + LANE_BYTES - ua) / MCP_CLASSIC_RX_BYTES;
    if (count > contiguous) count = contiguous;

    uint8_t objects[MCP_CLASSIC_CHUNK_BYTES];
    if (rx->read(rx->io, RAM_BASE + ua, objects, count * MCP_CLASSIC_RX_BYTES) != ESP_OK) return false;

    unsigned released = 0;
    while (released < count &&
           rx->write(rx->io, REG_C1FIFOCON(fifo), FIFO_RX_CONFIG | FIFO_INCREMENT) == ESP_OK) {
        deliver_frame(rx, objects + released * MCP_CLASSIC_RX_BYTES, fifo);
        ++released;
    }
    return released != 0;
}

bool mcp251xfd_classic_drain(mcp251xfd_classic_rx_t *rx, int preferred_sid)
{
    uint8_t flags[4];
    if (rx->read(rx->io, REG_C1RXIF, flags, sizeof(flags)) != ESP_OK) return false;

    return mcp251xfd_classic_drain_pending(rx, preferred_sid, read_le32(flags));
}

bool mcp251xfd_classic_drain_pending(mcp251xfd_classic_rx_t *rx, int preferred_sid, uint32_t pending)
{
    // The acceptance filters use the same SID bits for both frame formats.
    const unsigned preferred = preferred_sid < 0 ? MCP_CLASSIC_FIFO_COUNT :
                               ((unsigned)preferred_sid >> LANE_SHIFT) % MCP_CLASSIC_FIFO_COUNT;
    const bool pending_preferred = preferred < MCP_CLASSIC_FIFO_COUNT &&
                                   (pending & (1u << fifo_for_lane(preferred)));
    const bool try_preferred_first = pending_preferred && !rx->prefer_other_lane;
    if (try_preferred_first && drain_lane(rx, preferred)) {
        rx->prefer_other_lane = true;
        return true;
    }

    const unsigned first_lane = rx->next_lane;
    for (unsigned i = 0; i < MCP_CLASSIC_FIFO_COUNT; ++i) {
        const unsigned lane = (first_lane + i) % MCP_CLASSIC_FIFO_COUNT;
        if (lane != preferred && (pending & (1u << fifo_for_lane(lane))) && drain_lane(rx, lane)) {
            rx->prefer_other_lane = false;
            rx->next_lane = (lane + 1) % MCP_CLASSIC_FIFO_COUNT;
            return true;
        }
    }
    // If the other lanes had nothing to service, keep draining the preferred lane.
    if (pending_preferred && !try_preferred_first && drain_lane(rx, preferred)) return true;
    rx->prefer_other_lane = false;
    return false;
}
