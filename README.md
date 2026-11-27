# DashKit Firmware

Passive USB CAN capture firmware for DashKit.

## Building

### Prerequisites

- macOS with [Homebrew](https://brew.sh)
- USB cable for flashing

### Setup

```bash
brew install cmake ninja dfu-util
git submodule update --init --recursive
./esp-idf/install.sh esp32s3
```

Activate ESP-IDF in every new terminal session (or add it to your `~/.zshrc`):

```bash
. ./esp-idf/export.sh
```

### Passive USB streaming

The firmware streams received CAN frames over USB. Both MCP2518FD
controllers use a zero-mask acceptance filter for all standard and extended IDs,
with no software whitelist. Controllers run in listen-only mode.

Both buses capture Classic CAN at 500 kbit/s. Each controller has four
24-frame receive FIFOs for payloads up to eight bytes. CAN FD frames are
omitted from the capture.
Capturing a bus with different timing requires configuring matching timing first.

Use the repository's pinned ESP-IDF and keep its tools in this directory:

```sh
export IDF_TOOLS_PATH="$PWD/.idf-tools"
./esp-idf/install.sh esp32s3
./tools/build_capture.sh
```

The script builds `build-capture/dashkit-firmware.bin`, using
`build-capture/sdkconfig.generated` and the defaults in `sdkconfig.defaults`. It does not flash a device. The capture configuration disables
both console outputs so diagnostic text does not share the binary USB stream.

USB Serial/JTAG streams COBS-delimited records with CRC16-CCITT, a boot identifier,
per-bus sequence numbers, 64-bit controller receive timestamps in device-uptime microseconds, and CAN flags. The
format is defined in `main/capture/capture_protocol.h`. A separate USB task drains
a 1,024-frame queue without blocking the CAN receive tasks, batching records into
full USB transfers. The capture profile uses the 240 MHz CPU setting, performance
compiler optimization and DMA-aligned polling SPI transfers. Status records report
queue drops, hardware FIFO overflow events and aborted USB writes. Overflow events
are a lower bound on lost frames; sequence gaps and the other drop counters can
overlap and must not be added together. Frames before the host connects cannot be
recovered. Sustained capture throughput needs validation on the connected vehicle.
SPI-level CRC checking and a separate SPI error counter are not implemented; a
zero drop count does not independently verify every frame on the wire.

Host protocol tests:

```sh
./tools/test/run_capture_protocol_test.sh
```

Before installing capture firmware, preserve a verified flash backup and inspect
the installed partition layout, active OTA slot and flash security state. Preserve
NVS and an existing recovery image when selecting the application slot to write.
Do not use a full flash erase as part of capture installation.

## Project Layout

- `main/can/` contains the MCP251xFD driver and CAN manager.
- `main/capture/` contains the USB transport and record format.
- `tools/` contains the build script and host tests.

## Hardware

| Function | GPIO |
|----------|------|
| SPI MOSI | 11 |
| SPI SCLK | 12 |
| SPI MISO | 13 |
| CAN0 CS / INT | 10 / 9 |
| CAN1 CS / INT | 7 / 8 |
| LED Red / Green / Blue | 39 / 40 / 41 |

Both CAN controllers are MCP2518FD with 40 MHz crystals and share the SPI2 bus at 10 MHz.

## License

MIT. See [LICENSE](LICENSE).
