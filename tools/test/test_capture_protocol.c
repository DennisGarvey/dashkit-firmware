#include <assert.h>
#include <stdio.h>
#include "capture_protocol.h"

int main(void)
{
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
    raw[1] = DK_TX_REPEAT;
    dk_put(raw + 8, 42, 4);
    dk_put(raw + 12, DK_TX_MAX_HZ, 4);
    dk_put(raw + 16, DK_TX_MAX_DURATION_MS, 4);
    raw[24] = 1;
    raw[25] = 0xa5;
    raw[26] = 0x0f;
    assert(dk_decode(wire, dk_encode(raw, 27, wire) - 1, decoded) == 29);
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

    memset(raw + 12, 0, 8);
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
    puts("Capture protocol: frame roundtrips, corruption, masked timed modes, status, and truncation passed");
}
