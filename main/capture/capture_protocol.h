#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* DashKit USB v1: COBS records terminated by zero, CRC16-CCITT (init FFFF).
 * All integers are little endian. Common header: version, type, bus, flags,
 * boot_id:u32, sequence:u32, timestamp_us:u64. Frame: id:u32, length:u8,
 * data[length], crc:u16. Status: queue_drops:u32, overflow_events:u32,
 * usb_aborts:u32, crc:u16. Sequence counts received frames per bus, before
 * queueing. Overflow events indicate at least one lost frame, not an exact count.
 * Utilization (bus, flags and sequence zero): serial, CPU cores 0/1, shared
 * SPI, and CAN buses 0/1, each as u16 per mille over the last sample interval.
 * Session control uses a nonzero u64 session ID in the time field and
 * one opcode at offset 20. Bus, flags and sequence are zero.
 * Flags: bit0 extended, bit1 RTR, bit2 FD, bit3 BRS.
 */
#define DK_VERSION 1
#define DK_FRAME 0
#define DK_STATUS 1
#define DK_STATUS_SIZE 32
#define DK_UTILIZATION 9
#define DK_UTILIZATION_SIZE 32
#define DK_SESSION 12
#define DK_SESSION_START 0
#define DK_SESSION_READY 1
#define DK_SESSION_KEEPALIVE 2
#define DK_SESSION_STOP 3
#define DK_MAX_RAW 91
#define DK_MAX_WIRE 93

static inline void dk_put(uint8_t *p, uint64_t value, size_t n)
{
    for (size_t i = 0; i < n; i++) p[i] = (uint8_t)(value >> (8 * i));
}

static inline uint64_t dk_get(const uint8_t *p, size_t n)
{
    uint64_t value = 0;
    for (size_t i = 0; i < n; i++) value |= (uint64_t)p[i] << (8 * i);
    return value;
}

static inline uint16_t dk_crc(const uint8_t *p, size_t n)
{
    // CRC-16/CCITT polynomial 0x1021, using one lookup per byte.
    static const uint16_t table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
    };
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; i++)
        crc = (uint16_t)((crc << 8) ^ table[(crc >> 8) ^ p[i]]);
    return crc;
}

/* Appends CRC and COBS-encodes a payload of at most DK_MAX_RAW-2 bytes. */
static inline size_t dk_encode(uint8_t *raw, size_t n, uint8_t *wire)
{
    if (n > DK_MAX_RAW - 2) return 0;
    dk_put(raw + n, dk_crc(raw, n), 2);
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

/* Input excludes delimiter. Returns zero for corrupt/unsupported records. */
static inline size_t dk_decode(const uint8_t *wire, size_t n, uint8_t *raw)
{
    if (!n || n >= DK_MAX_WIRE) return 0;
    size_t in = 0, out = 0;
    while (in < n) {
        uint8_t code = wire[in++];
        if (!code || in + code - 1 > n) return 0;
        for (unsigned i = 1; i < code; i++) {
            if (out >= DK_MAX_RAW || !wire[in]) return 0;
            raw[out++] = wire[in++];
        }
        if (in < n && code != 255) {
            if (out >= DK_MAX_RAW) return 0;
            raw[out++] = 0;
        }
    }
    if (out < 22 || raw[0] != DK_VERSION || raw[2] > 1 ||
        dk_get(raw + out - 2, 2) != dk_crc(raw, out - 2)) return 0;
    if (raw[1] == DK_SESSION)
        return out == 23 && !raw[2] && !raw[3] && !dk_get(raw + 8, 4) &&
            dk_get(raw + 12, 8) && raw[20] <= DK_SESSION_STOP ? out : 0;
    if (raw[1] == DK_STATUS)
        return out == DK_STATUS_SIZE + 2 && raw[3] == 0 ? out : 0;
    if (raw[1] == DK_UTILIZATION) {
        if (out != DK_UTILIZATION_SIZE + 2 || raw[2] || raw[3] || dk_get(raw + 8, 4)) return 0;
        for (size_t i = 20; i < DK_UTILIZATION_SIZE; i += 2)
            if (dk_get(raw + i, 2) > 1000) return 0;
        return out;
    }
    if (raw[1] != DK_FRAME || out < 27 || raw[24] > 64 ||
        out != (size_t)(27 + raw[24]) || (raw[3] & ~15)) return 0;
    if (!(raw[3] & 4) && raw[24] > 8) return 0;
    if ((raw[3] & 4) && ((raw[3] & 2) ||
        (raw[24] > 8 && raw[24] != 12 && raw[24] != 16 && raw[24] != 20 &&
         raw[24] != 24 && raw[24] != 32 && raw[24] != 48 && raw[24] != 64))) return 0;
    if ((raw[3] & 8) && !(raw[3] & 4)) return 0;
    if (dk_get(raw + 20, 4) > ((raw[3] & 1) ? 0x1fffffffU : 0x7ffU)) return 0;
    return out;
}
