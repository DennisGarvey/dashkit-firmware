# DashKit Custom USB Serial v1

This specification covers the DashKit-to-host CAN receive stream over native
Espressif USB Serial/JTAG, including capture session control, status,
utilization, and firmware logs. Open the serial port at a logical baud rate
of 8,000,000 with 8 data bits, no parity, and one stop bit.

## Record framing

A record consists of a 20-byte header, a type-specific body, and a two-byte CRC.
All multi-byte integers are unsigned and little-endian. Compute CRC-16/IBM-3740
(also called CRC-16/CCITT-FALSE) over the header and body with polynomial
`0x1021`, initial value `0xffff`, no reflection, and final XOR `0x0000`.
The check value for ASCII `123456789` is `0x29b1`. Append the CRC low byte
first, COBS-encode the result, and append a `00` delimiter. Senders may prefix
a record with an extra `00` to abandon any previous partial record.

USB reads can split or coalesce records. Accumulate bytes until `00`, ignore
empty segments, and discard segments longer than 224 encoded bytes through
the next delimiter. The common decoded size bounds, including CRC, are 22 to
223 bytes, with stricter lengths per type. Discard malformed COBS, bad CRCs,
unsupported versions or record types, and records that violate the field
constraints. Resume at the next delimiter.

## Common header

Offsets below refer to decoded bytes, before COBS encoding.

- Byte 0, `u8`: version, always 1.
- Byte 1, `u8`: record type.
- Byte 2, `u8`: DashKit CAN bus index, 0 or 1. Global records use 0.
- Byte 3, `u8`: flags, defined per record type.
- Bytes 4–7, `u32`: boot ID. The firmware generates a random value at startup
  and uses it in device records. Host session controls use 0.
- Bytes 8–11, `u32`: sequence number. Its meaning depends on the record type.
- Bytes 12–19, `u64`: monotonic device time in microseconds since boot.
  Session records use this field for a session ID.

## Record types

### CAN frame (0), device to host

Bytes 20–23 hold the CAN arbitration ID as a `u32`. Byte 24 holds the
payload length `N` in bytes. The next `N` bytes are the frame payload,
starting at byte 25. Decoded length is `27 + N`, including CRC.

Flags are extended ID (bit 0), RTR (bit 1), CAN FD (bit 2), and bit-rate
switch (bit 3). Bits 4–7 are zero. The ID limit is `0x7ff` for standard
frames or `0x1fffffff` for extended frames. Classical CAN allows lengths
0–8. CAN FD also allows 12, 16, 20, 24, 32, 48, and 64. RTR is invalid
with CAN FD, and bit-rate switch requires CAN FD. For RTR, the length can
represent the requested DLC. The record still carries exactly `N` bytes
after the length field. Ignore those bytes when RTR is set.

The device increments a separate 32-bit counter per bus before queueing
each received frame for capture. The first numbered frame after boot on
each bus has sequence 1, and the counter wraps modulo 2³². The timestamp
is expressed in device-uptime microseconds. Firmware can use the
controller receive time or the software capture time. Frame timestamps may
arrive out of order even when sequence numbers are contiguous. The
local stock-feature USB build emits up to eight payload bytes per frame and
leaves the RTR flag clear. Hosts should accept the full v1 format above.

### Capture status (1), device to host

Flags are zero. The header identifies the bus and snapshots its current
frame sequence. Three `u32` counters follow. Decoded length is 34 bytes.

- Bytes 20–23 hold CAN frames dropped from that bus because the software
  capture queue was full.
- Bytes 24–27 hold receive FIFO overflow events on that bus. Each event
  indicates at least one lost CAN frame.
- Bytes 28–31 hold USB output aborts across both buses. The same global
  counter is carried in each bus's status record.

Counters are cumulative since boot and wrap modulo 2³². A queue drop also
creates a sequence gap if the host receives surrounding frames. A USB abort
counts an aborted output batch and can affect multiple records. These
counters and observed gaps measure different events and can overlap.

### Utilization (9), device to host

Bus, flags, and sequence are zero. Six consecutive `u16` values starting at
byte 20 report utilization over the preceding sample interval. In order,
they represent USB serial, CPU core 0, CPU core 1, shared SPI, CAN bus 0,
and CAN bus 1. Each value is 0–1000 (per mille). Decoded length is 34 bytes.
Serial utilization counts both directions against the logical 8 Mbit/s
8N1 capacity. CAN utilization is estimated from frame sizes and nominal
bus bit rate. The stock-feature build samples about once per second,
with the first sample establishing a baseline.

### Session control (12), host and device

Bus, flags, and sequence are zero. Bytes 12–19 hold a nonzero 64-bit session
ID. Byte 20 holds the opcode. Decoded length is 23 bytes.

- **0 — START**, host to device. Requests a capture session. A different ID
  replaces the active session and clears its pending capture queue. The
  host can retry START with the same ID.
- **1 — READY**, device to host. Echoes the ID and carries the device boot
  ID. A new session starts streaming once the firmware successfully writes
  the reply.
- **2 — KEEPALIVE**, host to device. Refreshes the matching active
  session's three-second lease. No reply is sent.
- **3 — STOP**, host to device. Ends the matching active session and clears
  pending capture output. No reply is sent.

Use a fresh random nonzero ID when opening a session. Retry START every
500 ms until READY arrives with the same ID, then send KEEPALIVE every
second. After 2.5 seconds without valid stream activity in a ready session,
restart with a fresh ID. For compatibility with older firmware that streams
without a handshake, accept incoming frames and defer START retries while
frame or status records keep arriving.

### Firmware log (13), device to host, optional

Bus, flags, and sequence are zero. Bytes 20 through the byte before the
CRC contain 1–199 bytes of UTF-8 log text without a required terminator.
Decoded length is 23–221 bytes. The timestamp is the log entry time, so
buffered logs can carry earlier timestamps than records already received.
When decoding text, replace invalid UTF-8 sequences with the Unicode
replacement character.

## Host sequence tracking

Track only type 0 sequence numbers for gap counting, separately per bus.
Set the baseline from the first frame after a new boot ID or matching
READY. For subsequent frames compute `delta = (current - previous) mod 2³²`.
A delta of 1 is contiguous. Deltas from 2 through `0x7fffffff` indicate
`delta - 1` missing sequence numbers. Reject a delta of zero or at least
`0x80000000` as duplicate or out of order. Controller losses before firmware
sequence assignment are visible through status overflow events. Hosts that
convert timestamps to nanoseconds should check for overflow before
multiplying by 1,000.

## CAN transmission extension

**Firmware compatibility:** This section applies only to the `usb-tx` firmware
with USB transmission enabled. Receive-only hosts can skip these record types.
The extension uses the same v1 framing, CRC, and common header.

### Capability and request correlation

Establish a capture session and wait for a type 3 record with status 0,
sequence 0, and bus 1. This announces support for manual transmission,
controller feedback, fixed-rate repeat, and automatic reassertion. Bus 1 is a
capability marker in that announcement. In requests and their replies, the
bus field identifies the actual CAN bus.

Set the boot ID to the current device boot ID and give each request a nonzero
sequence. Correlate replies by boot ID, bus, and request sequence. Timed
transmissions reuse the initiating request's sequence for their per-attempt
feedback. Keep the capture session active throughout transmission.

### Host-to-device records

- **2 — Manual transmit.** Use the CAN frame body and flags, with optional
  modifiers described below. Set the time field to zero. The current firmware
  limits manual requests to one attempt every 100 ms.
- **4 — Fixed-rate repeat.** Replace the time field with frequency in Hz
  (`u32` at byte 12) and duration in milliseconds (`u32` at byte 16). The
  wire ranges are 1–1,000,000 Hz and 100–600,000 ms. Actual cadence depends
  on controller availability and bus arbitration.
- **7 — Automatic reassertion.** Use the same timed layout, with frequency
  zero and the same duration range. The firmware schedules an initial send
  and reasserts when matching received frames differ in the bits being held.
- **5 — Timed transmit control.** Use the timed request's bus and sequence,
  with flags and time both zero. Byte 20 is 0 to renew or 1 to cancel.
  Decoded length is 23 bytes. Renew before the one-second transmit lease
  elapses. This lease is separate from the capture session lease.

For types 4 and 7, put an `N`-byte overwrite mask immediately after the
`N`-byte payload. Set bits select the requested values to hold. Clear bits
follow the corresponding bits in matching received frames. The mask must
contain at least one set bit. An optional second `N`-byte selector mask
matches received bits against the requested values and must be disjoint
from the overwrite mask. Timed requests cannot use RTR. Only one timed
request can be active at a time.

For types 2, 4, and 7, flags bit 4 adds a three-byte counter descriptor
after the payload and any masks. It contains a little-endian `u16` start
bit and a byte with field width (1–64 bits) in bits 0–6 and a little-endian
bit-order indicator in bit 7. The whole field must fit in the payload.
Bit 5 adds a one-byte zero-based checksum index after the counter descriptor,
if present. The checksum is the sum modulo 256 of the two low arbitration
ID bytes and all payload bytes except the checksum byte. The firmware applies
the checksum after the counter increment. The checked-out firmware recognizes
a timed request's selector mask only when neither modifier is present.

### Device-to-host records

- **3 — Manual transmit result.** Byte 20 is a status code. Status 1 means
  the controller accepted the frame for transmission. Wait for controller
  feedback to determine its outcome.
- **6 — Fixed-rate repeat result; 8 — Automatic reassertion result.** Byte 20
  describes acceptance, rejection, or completion of the timed request.
- **10 — Controller feedback.** Byte 20 uses its own codes: 0 capability
  announcement, 1 acknowledged, 2 unconfirmed, and 3 outcome unknown. For
  codes 2 and 3, the firmware may append three `u32` diagnostic snapshots at
  offsets 21, 25, and 29: FIFO status, bus diagnostics, and error counters.
- **11 — Acknowledged transmit frame.** Uses the CAN frame body and flags,
  with the originating transmit request sequence. It carries the frame
  actually sent, including any payload changes made before transmission.

Types 3, 6, 8, and 10 use zero flags. Their decoded length is 23 bytes,
or 35 bytes for type 10 with diagnostics. The time field in device replies
contains the device timestamp in microseconds. Keep transmit replies out of
the receive frame gap calculation.

Result status codes for types 3, 6, and 8 are: 0 ready, 1 queued,
2 rejected, 3 rate limited, 4 transmit FIFO full, 5 controller error,
6 previous attempt failed, 7 timed request started, 8 duration elapsed,
9 cancelled, 10 another timed request is active, and 11 lease expired.
Type 3 uses codes 0–6. Types 6 and 8 use codes 2–11 for request results.
The v1 decoder also recognizes zero-sequence readiness announcements in
types 8 and 10. The current firmware announces readiness using type 3.

### Automatic transmit diagnostics (14), device to host

The bus field identifies the physical bus. Flags and sequence are zero,
and the timestamp marks the end of the reporting interval. Decoded length
is 114 bytes including CRC, or 110 bytes without the paired RX-to-TX peak.
The firmware emits this record when the
interval has an RX-triggered request, a timestamped success, or an
arbitration-only loss. The fields below are interval statistics.

- Byte 20: successful queue submissions with an eligible RX trigger.
- Bytes 24, 28: mean and peak from received frame start to overwrite detection.
- Bytes 32, 36: mean and peak from detection to the controller transmit request.
- Bytes 40, 44: mean and peak from frame start to receive-task wake. The wait
  is zero if the task was already awake when the frame started.
- Bytes 48, 52: mean and peak from receive-task wake (or frame start when
  already awake) to overwrite detection.
- Byte 56: RX-triggered samples whose frame had already started when the
  receive task woke on its polling timeout.
- Bytes 60, 64: mean and peak number of same-bus frames delivered between
  the trigger's frame start and overwrite detection, up to 32 per sample.
- Byte 68: successful transmit attempts with a hardware TX timestamp.
- Bytes 72, 76: mean and peak from the successful attempt's transmit request
  to its hardware TX timestamp.
- Byte 80: attempts that exhausted the hardware attempt limit and reported
  arbitration loss alone.
- Bytes 84, 88: count and mean time for successful corrections with paired
  RX and hardware TX timestamps, measured from the triggering frame start.
- Bytes 92, 96: mean and peak of the frames delivered ahead from the
  trigger's own FIFO.
- Bytes 100, 104: mean and peak of the frames delivered ahead from
  other FIFOs.
- Byte 108: peak time for the paired RX-to-TX samples (when present).

Each listed byte offset is the start of a little-endian `u32`. Durations
are in microseconds. Means are truncated to integers, and a timing group
with zero samples has zero mean and peak. Queue and successful transmit
samples can fall in different intervals. Use the directly paired measurement
at byte 88 for end-to-end averages.

## Test vectors

A START for session ID 1, including the optional leading delimiter and
mandatory trailing delimiter:

```text
00 03 01 0c 01 01 01 01 01 01 01 01 01 02 01 01 01 01 01 01 01 01 03 79 62 00
```

A bus 0 CAN frame with boot ID `0x12345678`, sequence 1, timestamp 1,000 µs,
standard ID `0x123`, and data `aa 00`. Decoded bytes including CRC:

```text
01 00 00 00 78 56 34 12 01 00 00 00 e8 03 00 00 00 00 00 00 23 01 00 00 02 aa 00 ae ec
```

COBS-encoded bytes including the trailing delimiter:

```text
02 01 01 01 06 78 56 34 12 01 01 01 03 e8 03 01 01 01 01 01 03 23 01 01 03 02 aa 03 ae ec 00
```
