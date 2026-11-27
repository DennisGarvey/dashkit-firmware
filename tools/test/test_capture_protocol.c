#include <assert.h>
#include <stdio.h>
#include "capture_protocol.h"

static inline uint16_t reference_crc(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int bit = 0; bit < 8; bit++)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

/* Appends CRC and COBS-encodes a payload of at most DK_MAX_RAW-2 bytes. */
static inline size_t reference_encode(uint8_t *raw, size_t n, uint8_t *wire)
{
    if (n > DK_MAX_RAW - 2) return 0;
    dk_put(raw + n, reference_crc(raw, n), 2);
    n += 2;
    size_t code_at = 0, out = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < n; i++) {
        if (raw[i] == 0) {
            wire[code_at] = code;
            code_at = out++;
            code = 1;
        } else {
            wire[out++] = raw[i];
            code++;
        }
    }
    wire[code_at] = code;
    wire[out++] = 0;
    return out;
}

static void check_wire_compatibility(void)
{
    uint32_t state = 0x12345678;
    for (size_t len = 0; len <= DK_MAX_RAW - 2; ++len) {
        for (unsigned pattern = 0; pattern < 32; ++pattern) {
            uint8_t raw[DK_MAX_RAW], reference_raw[DK_MAX_RAW];
            uint8_t wire[DK_MAX_WIRE], reference_wire[DK_MAX_WIRE];
            for (size_t i = 0; i < len; ++i) {
                state = state * 1664525u + 1013904223u;
                raw[i] = pattern == 0 ? 0 : pattern == 1 ? 0xff : (uint8_t)(state >> 24);
            }
            memcpy(reference_raw, raw, len);
            const size_t encoded = dk_encode(raw, len, wire);
            assert(encoded == reference_encode(reference_raw, len, reference_wire));
            assert(memcmp(wire, reference_wire, encoded) == 0);
        }
    }
}

int main(void)
{
    check_wire_compatibility();
    assert(dk_crc((const uint8_t *)"123456789", 9) == 0x29b1);
    const uint8_t lengths[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    for (size_t l = 0; l < sizeof(lengths); l++) {
        for (unsigned pattern = 0; pattern < 256; pattern++) {
            uint8_t raw[DK_MAX_RAW] = {1, DK_FRAME, 1, 1};
            uint8_t decoded[DK_MAX_RAW], wire[DK_MAX_WIRE];
            raw[3] |= lengths[l] > 8 ? 12 : 0;
            dk_put(raw + 4, 0x12345678, 4);
            dk_put(raw + 8, 0xffffffff, 4);
            dk_put(raw + 12, 0x100000000ULL + pattern, 8);
            dk_put(raw + 20, 0x1fffffff, 4);
            raw[24] = lengths[l];
            memset(raw + 25, pattern, raw[24]);
            size_t n = dk_encode(raw, 25 + raw[24], wire);
            assert(n <= DK_MAX_WIRE && wire[n - 1] == 0);
            assert(dk_decode(wire, n - 1, decoded) == 27u + raw[24]);
            assert(memcmp(raw, decoded, 27 + raw[24]) == 0);
            for (size_t i = 0; i < n - 1; i++) {
                wire[i] ^= 1;
                assert(dk_decode(wire, n - 1, decoded) == 0);
                wire[i] ^= 1;
            }
        }
    }
    uint8_t raw[DK_MAX_RAW] = {1, DK_STATUS, 0, 0};
    uint8_t decoded[DK_MAX_RAW], wire[DK_MAX_WIRE];
    assert(dk_encode(raw, 32, wire) == 36);
    assert(dk_decode(wire, 35, decoded) == 34);
    assert(dk_decode(wire, 34, decoded) == 0);
    assert(dk_decode(wire, 0, decoded) == 0);
    raw[1] = DK_UTILIZATION;
    dk_put(raw + 20, 123, 2);
    dk_put(raw + 30, 1000, 2);
    assert(dk_decode(wire, dk_encode(raw, DK_UTILIZATION_SIZE, wire) - 1, decoded) == DK_UTILIZATION_SIZE + 2);
    dk_put(raw + 30, 1001, 2);
    assert(dk_decode(wire, dk_encode(raw, DK_UTILIZATION_SIZE, wire) - 1, decoded) == 0);
    memset(raw, 0, sizeof(raw));
    raw[0] = DK_VERSION;
    raw[1] = DK_SESSION;
    for (uint8_t opcode = DK_SESSION_START; opcode <= DK_SESSION_STOP; opcode++) {
        raw[20] = opcode;
        dk_put(raw + 12, 0x0123456789abcdefULL, 8);
        assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 23);
        assert(dk_get(decoded + 12, 8) == 0x0123456789abcdefULL);
    }
    dk_put(raw + 12, 0, 8);
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 0);
    memset(raw, 0, sizeof(raw));
    raw[0] = DK_VERSION;
    raw[1] = DK_LOG;
    assert(dk_decode(wire, dk_encode(raw, 20, wire) - 1, decoded) == 0);
    memset(raw + 20, 'L', DK_LOG_MAX_TEXT);
    assert(dk_decode(wire, dk_encode(raw, 20 + DK_LOG_MAX_TEXT, wire) - 1, decoded) ==
           22 + DK_LOG_MAX_TEXT);
    assert(memcmp(raw + 20, decoded + 20, DK_LOG_MAX_TEXT) == 0);
    raw[3] = 1;
    assert(dk_decode(wire, dk_encode(raw, 20 + DK_LOG_MAX_TEXT, wire) - 1, decoded) == 0);
    puts("Capture protocol: 4096 frame roundtrips, single-bit corruption, status, truncation and firmware logs passed");
}
