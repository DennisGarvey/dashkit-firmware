#pragma once

#include <stdint.h>
#include <stdbool.h>

// CiTSCON enables the free-running counter. TSEOF is left clear for SOF.
#define MCP251XFD_TBC 0x010
#define MCP251XFD_TSCON 0x014
#define MCP251XFD_TBC_ENABLE (1u << 16)
#define MCP251XFD_RX_TIMESTAMP_ENABLE (1u << 5)

typedef struct {
    int64_t host_us;
    uint32_t counter;
    int32_t rate_ppm;
    bool valid;
} mcp251xfd_rx_clock_t;

static inline void mcp251xfd_rx_clock_sample(mcp251xfd_rx_clock_t *clock,
                                           uint32_t counter, int64_t before, int64_t after)
{
    const int64_t host = before + (after - before) / 2;
    if (clock->valid && after - before > 50 && host - clock->host_us < 10000000) return;
    const uint32_t ticks = counter - clock->counter;
    if (clock->valid && ticks >= 500000 && ticks < 10000000) {
        const int64_t ppm = ((host - clock->host_us) - ticks) * 1000000 / ticks;
        if (ppm >= -500 && ppm <= 500) {
            clock->rate_ppm = (3 * clock->rate_ppm + (int32_t)ppm) / 4;
        }
    }
    clock->host_us = host;
    clock->counter = counter;
    clock->valid = true;
}

static inline uint64_t mcp251xfd_rx_clock_time(const mcp251xfd_rx_clock_t *clock, uint32_t counter)
{
    if (!clock->valid) return 0;
    // A frame can precede the latest sample. Signed modular subtraction also
    // handles a counter wrap between capture and drain.
    const int32_t ticks = (int32_t)(counter - clock->counter);
    const int64_t time = clock->host_us + ticks + (int64_t)ticks * clock->rate_ppm / 1000000;
    return time > 0 ? (uint64_t)time : 0;
}
