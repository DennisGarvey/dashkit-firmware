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
    if ((raw[1] == DK_TX_REPEAT || raw[1] == DK_TX_AUTO) &&
        (n < 25 || raw[24] > 64 ||
         (n != (size_t)(25 + 2 * raw[24] + ((raw[3] & DK_TX_INCREMENT_COUNTER) ? 3 : 0) +
                         ((raw[3] & DK_TX_RECALCULATE_CHECKSUM) ? 1 : 0)) &&
          n != (size_t)(25 + 3 * raw[24] + ((raw[3] & DK_TX_INCREMENT_COUNTER) ? 3 : 0) +
                         ((raw[3] & DK_TX_RECALCULATE_CHECKSUM) ? 1 : 0))))) return 0;
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
    if (raw[1] == DK_TX_REPEAT || raw[1] == DK_TX_AUTO) {
        uint32_t hz = dk_get(raw + 12, 4), duration = dk_get(raw + 16, 4);
        if (!dk_get(raw + 8, 4) ||
            (raw[1] == DK_TX_REPEAT ? (!hz || hz > DK_TX_MAX_HZ) : hz != 0) ||
            duration < DK_TX_MIN_DURATION_MS || duration > DK_TX_MAX_DURATION_MS) return 0;
    }
    return out;
}

static void check_wire_compatibility(void)
{
    uint32_t state = 0x12345678;
    for (size_t len = 0; len <= DK_MAX_RAW - 2; ++len) {
        for (unsigned pattern = 0; pattern < 32; ++pattern) {
            uint8_t raw[DK_MAX_RAW] = {0}, reference_raw[DK_MAX_RAW] = {0};
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
    uint8_t wanted[] = {0xa5, 0x0f}, actual[] = {0x3c, 0xf0}, mask[] = {0xf0, 0x0f};
    assert(dk_merge_masked(wanted, actual, mask, 2));
    assert(wanted[0] == 0xac && wanted[1] == 0xff);
    const uint8_t current[] = {0xa1, 0x2f};
    assert(!dk_merge_masked(wanted, current, mask, 2));
    assert(wanted[0] == 0xa1 && wanted[1] == 0x2f);
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
    raw[1] = DK_TX_FEEDBACK;
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 23);
    dk_put(raw + 8, 42, 4);
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 0);
    raw[20] = DK_TX_ACKNOWLEDGED;
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 23);
    raw[20] = DK_TX_OUTCOME_UNKNOWN + 1;
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 0);
    raw[20] = DK_TX_UNCONFIRMED;
    dk_put(raw + 21, 0x30, 4);
    dk_put(raw + 25, 1u << 18, 4);
    dk_put(raw + 29, 8u << 8, 4);
    size_t feedback_size = dk_encode(raw, DK_TX_FEEDBACK_DIAGNOSTICS_SIZE, wire);
    assert(dk_decode(wire, feedback_size - 1, decoded) == DK_TX_FEEDBACK_DIAGNOSTICS_SIZE + 2);
    assert(dk_get(decoded + 25, 4) == (1u << 18));
    raw[20] = DK_TX_ACKNOWLEDGED;
    assert(dk_decode(wire, dk_encode(raw, DK_TX_FEEDBACK_DIAGNOSTICS_SIZE, wire) - 1, decoded) == 0);
    memset(raw, 0, sizeof(raw));
    raw[0] = DK_VERSION;
    raw[1] = DK_TX_REPEAT;
    dk_put(raw + 8, 42, 4);
    dk_put(raw + 12, DK_TX_MAX_HZ, 4);
    dk_put(raw + 16, DK_TX_MAX_DURATION_MS, 4);
    raw[24] = 1;
    raw[25] = 0xa5;
    raw[26] = 0x0f;
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 29);
    raw[27] = 0x80;  // Selector bits are compared, never overridden.
    assert(dk_decode(wire, dk_encode(raw, 28, wire) - 1, decoded) == 30);
    const uint8_t compatible[] = {0xdf}, different_selector[] = {0x5f};
    assert(dk_matches_mask(raw + 25, compatible, raw + 27, 1));
    assert(!dk_matches_mask(raw + 25, different_selector, raw + 27, 1));
    assert(dk_merge_masked(raw + 25, compatible, raw + 26, 1));
    assert(raw[25] == 0xd5);
    raw[27] = 0x81;
    assert(dk_decode(wire, dk_encode(raw, 28, wire) - 1, decoded) == 0);
    dk_put(raw + 12, DK_TX_MAX_HZ + 1, 4);
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 0);
    dk_put(raw + 12, 1, 4);
    dk_put(raw + 16, DK_TX_MIN_DURATION_MS - 1, 4);
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 0);
    dk_put(raw + 16, DK_TX_MIN_DURATION_MS, 4);
    raw[26] = 0;
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 0);
    raw[1] = DK_TX_AUTO;
    raw[26] = 0x0f;
    dk_put(raw + 12, 0, 4);
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 29);
    dk_put(raw + 12, 1, 4);
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 0);

    // Counter descriptors are validated and both DBC bit orders increment
    // from the supplied live-frame value with width-based rollover.
    raw[3] = DK_TX_INCREMENT_COUNTER;
    dk_put(raw + 12, 0, 4);
    raw[26] = 0x0f;
    dk_put(raw + 27, 0, 2);
    raw[29] = 0x84;  // Four-bit little-endian counter at bit zero.
    assert(dk_decode(wire, dk_encode(raw, 30, wire) - 1, decoded) == 32);
    uint8_t little[] = {0xaf};
    dk_increment_counter(little, 0, 4, true);
    assert(little[0] == 0xa0);
    uint8_t motorola[] = {0x0f, 0xf5};
    dk_increment_counter(motorola, 3, 8, false);
    assert(motorola[0] == 0x00 && motorola[1] == 0x05);
    raw[29] = 0x80;  // Zero-width descriptors are invalid.
    assert(dk_decode(wire, dk_encode(raw, 30, wire) - 1, decoded) == 0);
    raw[3] = DK_TX_INCREMENT_COUNTER | DK_TX_RECALCULATE_CHECKSUM;
    raw[29] = 0x84;
    raw[30] = 0;
    assert(dk_decode(wire, dk_encode(raw, 31, wire) - 1, decoded) == 33);
    raw[30] = 1;  // Checksum byte must be inside the payload.
    assert(dk_decode(wire, dk_encode(raw, 31, wire) - 1, decoded) == 0);
    uint8_t protected_frame[] = {0xaa, 0x0f, 0x00};
    dk_increment_counter(protected_frame, 8, 4, true);
    dk_recalculate_tesla_checksum(protected_frame, 3, 0x249, 0);
    assert(protected_frame[0] == 0x4b && protected_frame[1] == 0x00);

    memset(raw + 12, 0, 8);
    raw[3] = 0;
    raw[1] = DK_TX_REPEAT_CONTROL;
    raw[20] = DK_TX_CANCEL;
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 23);
    raw[20]++;
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 0);
    raw[1] = DK_TX_REPEAT_RESULT;
    raw[20] = DK_TX_REPEAT_CANCELLED;
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 23);
    raw[1] = DK_TX_AUTO_RESULT;
    raw[20] = DK_TX_READY;
    dk_put(raw + 8, 0, 4);
    assert(dk_decode(wire, dk_encode(raw, 21, wire) - 1, decoded) == 23);
    memset(raw, 0, sizeof(raw));
    raw[0] = DK_VERSION;
    raw[1] = DK_TX_FRAME;
    raw[2] = 1;
    raw[24] = 1;
    raw[25] = 42;
    assert(dk_decode(wire, dk_encode(raw, 26, wire) - 1, decoded) == 0);
    dk_put(raw + 8, 2, 4);
    assert(dk_decode(wire, dk_encode(raw, 26, wire) - 1, decoded) == 28);
    assert(decoded[25] == 42);
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
    raw[1] = DK_TX_LATENCY;
    raw[2] = 1;
    assert(dk_decode(wire, dk_encode(raw, DK_TX_LATENCY_SIZE, wire) - 1, decoded) == 0);
    dk_put(raw + 20, 5, 4);
    dk_put(raw + 36, 1234, 4);
    assert(dk_decode(wire, dk_encode(raw, DK_TX_LATENCY_SIZE, wire) - 1, decoded) == DK_TX_LATENCY_SIZE + 2);
    assert(dk_get(decoded + 36, 4) == 1234);
    puts("Capture protocol: frame roundtrips, corruption, masked timed modes, status, and truncation passed");
}
