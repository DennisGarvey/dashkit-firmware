#pragma once

#include <stddef.h>

#include "can_interface.h"

#define MCP_CLASSIC_FIFO_COUNT   4
#define MCP_CLASSIC_FIFO_DEPTH   24
#define MCP_CLASSIC_RX_BYTES     20
#define MCP_CLASSIC_CHUNK_FRAMES 8
#define MCP_CLASSIC_CHUNK_BYTES  (MCP_CLASSIC_RX_BYTES * MCP_CLASSIC_CHUNK_FRAMES)

typedef struct {
    void *io;
    esp_err_t (*read)(void *io, uint16_t addr, uint8_t *data, size_t len);
    esp_err_t (*write)(void *io, uint16_t addr, uint32_t value);
    void (*deliver)(void *io, const can_frame_t *frame, uint32_t counter);
    void (*overflow)(void *io);
    bool has_tx_event;
    uint8_t next_lane;
} mcp251xfd_classic_rx_t;

esp_err_t mcp251xfd_classic_configure(mcp251xfd_classic_rx_t *rx);
esp_err_t mcp251xfd_classic_reset(mcp251xfd_classic_rx_t *rx);
bool mcp251xfd_classic_drain(mcp251xfd_classic_rx_t *rx);
