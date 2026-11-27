# DashKit USB to ZMQ bridge

Run `zmq_bridge.py` on the machine connected to DashKit over USB. It publishes received CAN frames on the standard Cabana ZMQ `can` endpoint, preserving the bus number, CAN ID, and payload. Cabana uses its built-in ZMQ input. Firmware logs and device metrics are displayed in the bridge terminal.

The bridge buffers frames for at least 10 ms while streaming. By default, it
publishes strictly increasing timestamps for upstream Cabana. A tied or late
timestamp is moved just past the last published frame. The bridge reports late
arrivals in its terminal output.

With a Cabana build that supports out-of-order live frames, pass
`--preserve-late-timestamps` to publish the original timestamps, including ties
and late arrivals.

## Requirements

- Python 3 with `pyserial`, `pycapnp`, and `pyzmq`.
- A DashKit device running one of the USB capture builds below.
- Cabana with ZMQ support.

## Compatible firmware

These firmware branches support [Custom USB Serial v1](PROTOCOL.md). Use the branch's capture-enabled build configuration.

- **`usb-rx-stock-featureset`** streams CAN frames, utilization, and firmware logs while keeping the standard DashKit features. Use the `sdkconfig.ble-usb` profile. [Firmware branch](https://github.com/DennisGarvey/dashkit-firmware/tree/usb-rx-stock-featureset)
- **`usb-rx-only`** streams CAN frames and utilization in listen-only mode. Use the `sdkconfig.capture` profile. [Firmware branch](https://github.com/DennisGarvey/dashkit-firmware/tree/usb-rx-only)
- **`usb-tx`** also streams CAN frames and utilization. Its `sdkconfig.capture` profile works with the bridge, which forwards received frames only. [Firmware branch](https://github.com/DennisGarvey/dashkit-firmware/tree/usb-tx)

The `usb-rx-stock-featureset` build is the one that sends firmware logs. The standard `main` build does not include this USB capture protocol.

## Run on one machine

From the directory containing this script, create a virtual environment and install the dependencies.

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install pyserial pycapnp pyzmq
python zmq_bridge.py
```

On Windows, use `py -m venv .venv` and run the install and bridge commands with `.venv\Scripts\python.exe` in place of `python`.

In another terminal, run:

```sh
cabana --zmq 127.0.0.1
```

The default listener is `127.0.0.1:62300`. Stop the bridge with Ctrl+C.

## Run Cabana on another machine

Bind to the USB host's LAN IPv4 address:

```sh
python zmq_bridge.py --bind 192.168.1.20
cabana --zmq 192.168.1.20
```

Replace the example address with the USB host's address and run the Cabana command on the client machine. For Tailscale, run `python zmq_bridge.py --tailscale` on the USB host and pass the IPv4 shown at startup to `cabana --zmq`. This mode requires a connected Tailscale CLI on the bridge host's `PATH`. Use either `--bind` or `--tailscale`.

The Cabana machine must be able to reach TCP port `62300` on the USB host. The ZMQ stream has no application-level authentication or encryption, so expose it only on a trusted network.

## Select a USB port

The bridge auto-detects a single native Espressif USB Serial/JTAG adapter. If you have multiple adapters or detection fails, list the ports and pass one explicitly:

```sh
python -m serial.tools.list_ports
python zmq_bridge.py /dev/ttyACM0
```

A port might appear as `/dev/cu.usbmodem101` on macOS or `COM3` on Windows. Make sure your account can open the port and close other programs using it. The default baud setting is `8000000`. Use `--baud` if your serial adapter requires a different setting.

## Read the bridge output

Firmware log lines appear with a `[FIRMWARE]` label. On color terminals, errors are red, warnings are yellow, and other firmware messages are cyan. The header shows the connection and CAN receive rates. Utilization and capture counters appear in a right-hand sidebar when the terminal has room, or in a footer on smaller terminals. A dash means a device metric is unavailable.

### Device Counters (Since Boot)

DashKit reports these counters. A DashKit reboot resets them. Reconnecting to the same device or restarting the bridge preserves the firmware totals.

- **Capture Queue Drops** count received CAN frames the firmware discarded because its software capture queue was full.
- **CAN Controller FIFO A/B Overflows** count hardware receive FIFO overflow events. Each event indicates at least one lost CAN frame.
- **USB Write Aborts** count batches the firmware could not fully send over USB. Depending on the firmware, this can include batches dropped because the USB output queue was full.

### Bridge Counters (Since Launch)

The bridge counts these events until it exits. Reconnecting to DashKit preserves the totals.

- **USB Frame Sequence Gaps** count missing CAN frame numbers in the stream received by the bridge.
- **Invalid USB Records** count records the bridge rejects, such as malformed records, CRC failures, or duplicate or out-of-order CAN frame numbers.
- **Device Reboots** count changes in the firmware boot ID observed by the bridge.

Use Up and Down to move the cursor through the log. The pane scrolls when the cursor reaches an edge. Left and Right move across a line. Home jumps to the oldest retained entry and End resumes following live output, with the cursor on the next blank line. The bridge retains the latest 2,000 log lines while it runs.

Use `--no-status` for plain-text logs and metrics when redirecting output or using a terminal without curses. Run `python zmq_bridge.py --help` to see all options.
