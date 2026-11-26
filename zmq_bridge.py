#!/usr/bin/env python3
"""Publish Dashkit Custom USB Serial v1 CAN frames for Cabana and show device diagnostics.

Install dependencies with: python3 -m pip install pyserial pycapnp pyzmq
Run on the USB host and connect locally with: cabana --zmq 127.0.0.1
For another machine, pass --bind <USB-host-IPv4> or use --tailscale to
look up the USB host's Tailscale IPv4. Give Cabana the address shown at startup.
Only bind to a trusted network. The stream is unauthenticated and unencrypted.

Requires a device running DashKit Custom USB Serial v1 firmware. The bridge
auto-detects a single native Espressif USB Serial/JTAG adapter. If detection
fails or finds multiple adapters, list ports with python3 -m serial.tools.list_ports
and pass the device path (for example /dev/ttyACM0, /dev/cu.usbmodem101 or COM3).
"""
import argparse
from collections import deque
try:
    import curses
except ImportError:
    curses = None
from binascii import crc_hqx
from ipaddress import IPv4Address
import os
from pathlib import Path
import re
from secrets import randbits
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory
import time


MAX_RAW = 223
MAX_ENCODED = 224
HEADER = struct.Struct('<BBBBIIQ')
FRAME, STATUS, UTILIZATION, SESSION, LOG = 0, 1, 9, 12, 13
SESSION_START, SESSION_READY, SESSION_KEEPALIVE, SESSION_STOP = range(4)
UTILIZATION_LABELS = ('Serial', 'CPU0', 'CPU1', 'SPI', 'CAN0', 'CAN1')
ANSI_CSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
FD_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)
ESP_USB_SERIAL_JTAG = (0x303a, 0x1001)

# Standalone subset of cereal/log.capnp. Keep the CAN union tag (4), the
# timestamp and list pointer slots, and the CanData field layout compatible.
CAN_SCHEMA = """@0xa490fa77760e6ca3;

struct Event {
  logMonoTime @0 :UInt64;
  union {
    initData @1 :Void;
    sentinel @2 :Void;
    boot @3 :Void;
    gpsNmea @4 :Void;
    can @5 :List(CanData);
  }
}

struct CanData {
  address @0 :UInt32;
  busTimeDeprecated @1 :UInt16;
  dat @2 :Data;
  src @3 :UInt8;
}
"""


def tailscale_ip():
    try:
        result = subprocess.run(['tailscale', 'ip', '-4'], capture_output=True,
                                text=True, check=True, timeout=5)
        return str(IPv4Address(result.stdout.strip()))
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        detail = getattr(error, 'stderr', None) or str(error)
        raise RuntimeError(f'Cannot get Tailscale IPv4 address: {detail.strip()}. '
                           'Check that the Tailscale CLI is on PATH and connected, or use --bind.') from error


def detect_port(ports):
    matches = sorted(port.device for port in ports if (port.vid, port.pid) == ESP_USB_SERIAL_JTAG)
    if not matches:
        raise OSError('No Espressif USB Serial/JTAG device found')
    if len(matches) > 1:
        raise OSError(f'Multiple Espressif USB Serial/JTAG devices found ({", ".join(matches)}). '
                      'Specify the serial path explicitly')
    return matches[0]


def can_port():
    # Match replay/cereal/messaging/impl_zmq.cc on 64-bit platforms.
    value = 0xcbf29ce484222325
    for byte in b'can':
        value = ((value ^ byte) * 0x100000001b3) & 0xffffffffffffffff
    return 8023 + value % (65535 - 8023)


def session_control(session_id, opcode):
    raw = HEADER.pack(1, SESSION, 0, 0, 0, 0, session_id) + bytes([opcode])
    raw += struct.pack('<H', crc_hqx(raw, 0xffff))
    blocks = raw.split(b'\0')
    return b'\0' + b''.join(bytes([len(block) + 1]) + block for block in blocks) + b'\0'


def decode(wire):
    """Decode one delimiter-free record using Cabana's v1 validation rules."""
    raw = bytearray()
    pos = 0
    while pos < len(wire):
        code = wire[pos]
        end = pos + code
        if not code or end > len(wire):
            return
        raw.extend(wire[pos + 1:end])
        pos = end
        if pos < len(wire) and code != 255:
            raw.append(0)
    if not 22 <= len(raw) <= MAX_RAW:
        return
    if crc_hqx(raw[:-2], 0xffff) != int.from_bytes(raw[-2:], 'little'):
        return
    version, kind, bus, flags, boot, sequence, timestamp = HEADER.unpack_from(raw)
    if version != 1 or bus > 1:
        return
    if kind == SESSION:
        if len(raw) == 23 and not (bus or flags or sequence) and timestamp and raw[20] <= SESSION_STOP:
            return kind, bus, boot, sequence, timestamp, raw[20]
        return
    if timestamp > 0xffffffffffffffff // 1000:
        return
    if kind == STATUS and flags == 0 and len(raw) == 34:
        return kind, bus, boot, sequence, timestamp, struct.unpack_from('<III', raw, 20)
    if kind == UTILIZATION and not (bus or flags or sequence) and len(raw) == 34:
        values = struct.unpack_from('<6H', raw, 20)
        if max(values) <= 1000:
            return kind, bus, boot, sequence, timestamp, values
        return
    if kind == LOG and not (bus or flags or sequence) and 22 < len(raw) <= 221:
        return kind, bus, boot, sequence, timestamp, raw[20:-2].decode('utf-8', errors='replace')
    if kind != FRAME or len(raw) < 27 or flags & ~15:
        return
    address, length = struct.unpack_from('<IB', raw, 20)
    extended, rtr, fd, brs = (flags & bit for bit in (1, 2, 4, 8))
    if len(raw) != 27 + length or address > (0x1fffffff if extended else 0x7ff):
        return
    if fd:
        if rtr or length not in FD_LENGTHS:
            return
    elif brs or length > 8:
        return
    return kind, bus, boot, sequence, timestamp, (address, b'' if rtr else bytes(raw[25:-2]))


class UsbStream:
    def __init__(self):
        self.boot = None
        self.sequences = [None, None]
        self.last_ns = 0
        self.offset_ns = 0
        self.invalid = 0
        self.restarts = 0
        self.gaps = [0, 0]
        self.frames = [0, 0]
        self.status = [None, None]
        self.usb_aborts = None
        self.utilization_time = 0.0
        self.reset_input()

    def reset_input(self):
        self.pending = b''
        self.discarding = False
        self.session_id = randbits(64) or 1
        self.session_ready = False
        self.control_due = 0.0
        self.last_activity = None
        self.utilization = None

    def maintain_session(self, usb):
        now = time.monotonic()
        if self.session_ready and now - self.last_activity >= 2.5:
            self.reset_input()
        if now < self.control_due:
            return
        if not self.session_ready and self.last_activity is not None and now - self.last_activity < 0.5:
            return  # Older firmware streams without session control.
        opcode = SESSION_KEEPALIVE if self.session_ready else SESSION_START
        request = session_control(self.session_id, opcode)
        if usb.write(request) != len(request):
            raise OSError('USB session control write incomplete')
        self.control_due = now + (1.0 if self.session_ready else 0.5)

    def feed(self, chunk, firmware_log=None):
        *records, tail = chunk.split(b'\0')
        if not records:
            self.pending = self.append(tail)
            return
        records[0] = self.append(records[0])
        for wire in records:
            if self.discarding or len(wire) > MAX_ENCODED:
                self.invalid += 1
                self.discarding = False
            elif wire:
                record = decode(wire)
                if record is None:
                    self.invalid += 1
                else:
                    frame = self.accept(record, firmware_log)
                    if frame is not None:
                        yield frame
        self.pending = b''
        self.pending = self.append(tail)

    def append(self, chunk):
        if self.discarding or len(self.pending) + len(chunk) > MAX_ENCODED:
            self.discarding = True
            return b''
        return self.pending + chunk

    def accept(self, record, firmware_log=None):
        kind, bus, boot, sequence, timestamp, payload = record
        if kind == SESSION:
            if not self.session_ready and payload == SESSION_READY and timestamp == self.session_id:
                self.session_ready = True
                self.last_activity = time.monotonic()
                self.control_due = self.last_activity + 1.0
                self.sequences = [None, None]
            return
        if kind in (FRAME, STATUS) or self.session_ready:
            self.last_activity = time.monotonic()
        device_ns = timestamp * 1000
        if boot != self.boot:
            if self.boot is not None:
                self.restarts += 1
            self.boot = boot
            self.sequences = [None, None]
            self.status = [None, None]
            self.usb_aborts = None
            self.utilization = None
            self.offset_ns = max(0, self.last_ns + 1 - device_ns)
        if kind == STATUS:
            self.status[bus] = payload
            self.usb_aborts = payload[2]
            return
        if kind == UTILIZATION:
            self.utilization = payload
            self.utilization_time = time.monotonic()
            return
        if kind == LOG:
            if firmware_log is not None:
                firmware_log(payload)
            return
        previous = self.sequences[bus]
        if previous is not None:
            delta = (sequence - previous) & 0xffffffff
            if not 0 < delta < 0x80000000:
                self.invalid += 1
                return
            self.gaps[bus] += delta - 1
        self.sequences[bus] = sequence
        self.frames[bus] += 1
        self.last_ns = max(1, self.last_ns, min(device_ns + self.offset_ns, 0xffffffffffffffff))
        return self.last_ns, bus, *payload


class TerminalStatus:
    """Scrolling event log and a fixed bottom status bar for the bridge."""

    def __init__(self, stream, enabled=True):
        self.stream = stream
        self.state = 'WAITING FOR USB'
        self.port = None
        self.logs = deque(maxlen=2000)
        self.log_offset = 0
        self.log_rows = 1
        self.log_columns = 1
        self.max_log_offset = 0
        self.screen = None
        self.firmware_colors = {}
        self.enabled = bool(enabled and curses is not None
                            and sys.stdin.isatty() and sys.stdout.isatty()
                            and os.environ.get('TERM', 'dumb') != 'dumb')
        self.last_update = time.monotonic()
        self.frames = stream.frames.copy()
        self.rates = [0.0, 0.0]
        self.gaps = stream.gaps.copy()
        self.invalid = stream.invalid
        self.restarts = stream.restarts
        self.device_drops = [None, None]
        self.device_aborts = None

        if self.enabled:
            try:
                self.screen = curses.initscr()
                curses.noecho()
                curses.cbreak()
                self.screen.keypad(True)
                self.screen.nodelay(True)
            except Exception:
                # Some serial consoles/TERM types do not support curses.
                if self.screen is not None:
                    try:
                        curses.endwin()
                    except curses.error:
                        pass
                self.screen = None
                self.enabled = False

        if self.enabled and curses.has_colors():
            try:
                curses.start_color()
                curses.use_default_colors()
                for pair, (level, color) in enumerate(
                        (('I', curses.COLOR_CYAN), ('W', curses.COLOR_YELLOW), ('E', curses.COLOR_RED)), 1):
                    curses.init_pair(pair, color, -1)
                    self.firmware_colors[level] = curses.color_pair(pair)
            except curses.error:
                self.firmware_colors.clear()

    def _poll_terminal(self):
        offset = self.log_offset
        while (key := self.screen.getch()) != -1:
            if key == curses.KEY_PPAGE:
                self.log_offset += self.log_rows
            elif key == curses.KEY_NPAGE:
                self.log_offset -= self.log_rows
            elif key == curses.KEY_HOME:
                self.log_offset = self.max_log_offset
            elif key == curses.KEY_END:
                self.log_offset = 0
            self.log_offset = min(max(0, self.log_offset), self.max_log_offset)
        changed = offset != self.log_offset
        try:
            size = os.get_terminal_size(sys.stdout.fileno())
        except OSError:
            return changed
        if size.lines > 0 and size.columns > 0 and curses.is_term_resized(size.lines, size.columns):
            curses.resizeterm(size.lines, size.columns)
            return True
        return changed

    def _utilization_fields(self):
        values = self.stream.utilization
        if values is None or time.monotonic() - self.stream.utilization_time >= 3.0:
            return [f'{label}: —' for label in UTILIZATION_LABELS]
        return [f'{label}: {value / 10:.1f}%'
                for label, value in zip(UTILIZATION_LABELS, values)]

    def _device_counter_fields(self):
        status = self.stream.status
        drops = ' / '.join(f'B{bus} {values[0]:,}' if values is not None else f'B{bus} —'
                           for bus, values in enumerate(status))
        fifos = [f'FIFO {label}: {values[1]:,}' if values is not None else f'FIFO {label}: —'
                 for label, values in zip('AB', status)]
        aborts = self.stream.usb_aborts
        return [f'Capture drops: {drops}', *fifos,
                f'USB aborts: {aborts:,}' if aborts is not None else 'USB aborts: —']

    @staticmethod
    def _wrap_fields(fields, width):
        lines = [fields[0]]
        for field in fields[1:]:
            line = f'{lines[-1]}  |  {field}'
            if len(line) < width:
                lines[-1] = line
            else:
                lines.append(field)
        return lines

    def _header_lines(self, width):
        b0, b1 = self.rates
        capture = [f' DashKit USB · {self.state}']
        if self.log_offset:
            capture.append('Log history · End to follow')
        if self.port:
            capture.append(self.port)
        receive = f'Total RX: {b0 + b1:,.0f} fps'
        breakdown = f'(Bus 0: {b0:,.0f} / Bus 1: {b1:,.0f} fps)'
        if len(f'{receive} {breakdown}') < width:
            capture.append(f'{receive} {breakdown}')
        else:
            capture.extend([receive, breakdown])
        capture.append(f'Total Frames: {sum(self.stream.frames):,}')
        if not self.log_offset:
            capture.append('PgUp for log history')
        return self._wrap_fields(capture, width)

    def _footer_lines(self, width):
        utilization = ['Utilization', *self._utilization_fields()]
        counters = ['Capture counters', f'USB seq gaps: {sum(self.stream.gaps):,}', *self._device_counter_fields(),
                    f'Invalid USB: {self.stream.invalid:,}', f'Device restarts: {self.stream.restarts:,}']
        return self._wrap_fields(utilization, width) + self._wrap_fields(counters, width)

    def _draw(self):
        if not self.enabled:
            return
        try:
            height, width = self.screen.getmaxyx()
            if height < 1 or width < 1:
                return
            self.screen.erase()
            header = self._header_lines(width)[:max(1, (height - 1) // 3)]
            footer = self._footer_lines(width)[:max(0, height - len(header) - 2)]
            footer_start = height - len(footer)
            divider = footer_start - bool(footer)
            self.log_rows = max(1, divider - len(header))
            columns = width - 1
            for row, line in enumerate(header):
                if columns:
                    self.screen.addnstr(row, 0, line, columns, curses.A_BOLD if row == 0 else 0)
            self.screen.chgat(0, 0, width, curses.A_BOLD | curses.A_REVERSE)
            if width > 1:
                self.log_columns = columns
                log_space = divider - len(header)
                wrapped = [(line[start:start + columns], style) for line, style in self.logs
                           for start in range(0, max(1, len(line)), columns)]
                self.max_log_offset = max(0, len(wrapped) - self.log_rows)
                self.log_offset = min(self.log_offset, self.max_log_offset)
                end = len(wrapped) - self.log_offset
                visible = wrapped[max(0, end - log_space):end]
                for row, (line, style) in enumerate(visible, len(header)):
                    self.screen.addnstr(row, 0, line, columns, style)
                if footer:
                    self.screen.hline(divider, 0, curses.ACS_HLINE, columns)
                for row, line in enumerate(footer, footer_start):
                    self.screen.addnstr(row, 0, line, columns)
            self.screen.refresh()
        except curses.error:
            # A resize can arrive during drawing; the next draw rebuilds it.
            pass

    def log(self, message, redraw=True, style=0):
        lines = str(message).split('\n')
        if not self.enabled:
            print('\n'.join(lines), flush=True)
            return
        timestamp = time.strftime('%H:%M:%S')
        lines = [f'[{timestamp}] {line}' if line else '' for line in lines]
        self.logs.extend((line, style) for line in lines)
        if self.log_offset:
            self.log_offset += sum(max(1, (len(line) + self.log_columns - 1) // self.log_columns)
                                   for line in lines)
        if redraw:
            self._draw()

    def firmware_log(self, message):
        message = ANSI_CSI.sub('', message).replace('\r\n', '\n').rstrip('\n')
        message = ''.join(char for char in message.replace('\t', '    ')
                          if char == '\n' or char.isprintable())
        level = 'I'
        for line in message.splitlines():
            severity = re.match(r'^([EWIDV]) +\(', line)
            if severity:
                level = severity[1]
            style = self.firmware_colors.get(level, self.firmware_colors.get('I', 0))
            self.log(f'[FIRMWARE] {line}', redraw=False, style=style)

    def set_state(self, state, port=None):
        if state != self.state:
            # Rates only cover the current connected interval, not downtime.
            self.last_update = time.monotonic()
            self.frames = self.stream.frames.copy()
            self.rates = [0.0, 0.0]
        self.state = state
        self.port = port
        self._draw()

    def update(self):
        try:
            redraw = self.enabled and self._poll_terminal()
        except curses.error:
            redraw = False
        now = time.monotonic()
        elapsed = now - self.last_update
        if elapsed < 1.0:
            if redraw:
                self._draw()
            return
        stream = self.stream
        self.rates = [(count - previous) / elapsed
                      for count, previous in zip(stream.frames, self.frames)]
        new_gaps = [count - previous
                    for count, previous in zip(stream.gaps, self.gaps)]
        new_invalid = stream.invalid - self.invalid
        new_restarts = stream.restarts - self.restarts
        self.frames = stream.frames.copy()
        self.gaps = stream.gaps.copy()
        self.invalid = stream.invalid
        self.restarts = stream.restarts
        self.last_update = now

        if new_restarts:
            self.log(f'USB device restarted ({stream.restarts:,} total).')
        if sum(new_gaps) or new_invalid:
            self.log(f'CAN/USB warning: +{sum(new_gaps):,} sequence gaps '
                     f'(B0 +{new_gaps[0]:,}, B1 +{new_gaps[1]:,}), '
                     f'+{new_invalid:,} invalid records.')
        for bus, values in enumerate(stream.status):
            if values is None:
                self.device_drops[bus] = None
                continue
            current = values[:2]  # Device-side queue drops, FIFO overflows.
            previous = self.device_drops[bus]
            increments = [max(0, new - old) for new, old in zip(current, previous)] \
                if previous is not None else list(current)
            if any(increments):
                self.log(f'Bus {bus} device drops: +{increments[0]:,} queue, '
                         f'+{increments[1]:,} FIFO (since last report).')
            self.device_drops[bus] = current
        if stream.usb_aborts is None:
            self.device_aborts = None
        else:
            previous_aborts = self.device_aborts or 0
            new_aborts = max(0, stream.usb_aborts - previous_aborts)
            if new_aborts:
                self.log(f'Device USB aborts: +{new_aborts:,}.')
            self.device_aborts = stream.usb_aborts
        if not self.enabled and stream.utilization is not None and now - stream.utilization_time < 3.0:
            self.log('Device metrics | ' + ' | '.join(self._utilization_fields() + self._device_counter_fields()))
        self._draw()

    def close(self):
        if not self.enabled:
            return
        try:
            self.screen.keypad(False)
            curses.nocbreak()
            curses.echo()
        finally:
            curses.endwin()
            self.enabled = False


def run(args):
    address = tailscale_ip() if args.tailscale else args.bind
    import capnp
    import serial
    from serial.tools import list_ports
    import zmq

    endpoint = f'tcp://{address}:{can_port()}'
    with TemporaryDirectory(prefix='dashkit-can-') as directory:
        schema = Path(directory) / 'dashkit_can.capnp'
        schema.write_text(CAN_SCHEMA, encoding='utf-8')
        event_type = capnp.load(str(schema)).Event
    stream = UsbStream()
    with zmq.Context() as context, context.socket(zmq.PUB) as publisher:
        publisher.linger = 0
        try:
            publisher.bind(endpoint)
        except zmq.ZMQError as error:
            raise RuntimeError(f'Cannot listen on {endpoint}: {error}') from error
        status = TerminalStatus(stream, enabled=not args.no_status)
        try:
            status.log('DashKit USB Serial -> Cabana / ZMQ')
            status.log(f'Publishing CAN at {endpoint}')
            status.log(f'USB port: {args.port or "auto-detect"} | Baud: {args.baud:,}')
            status.log('RX frame rates refresh every second. Ctrl+C to stop.')
            status.log('')
            connected_once = False
            while True:
                try:
                    port = args.port or detect_port(list_ports.comports())
                    with serial.Serial(port, args.baud, timeout=0.1, write_timeout=0.1) as usb:
                        connected_once = True
                        status.set_state('STREAMING', port)
                        status.log(f'USB connected: {port}')
                        try:
                            while True:
                                stream.maintain_session(usb)
                                for timestamp, bus, can_id, data in stream.feed(
                                        usb.read(min(usb.in_waiting, 65536) or 1), status.firmware_log):
                                    event = event_type.new_message(logMonoTime=timestamp)
                                    frame = event.init('can', 1)[0]
                                    frame.src = bus
                                    frame.address = can_id
                                    frame.dat = data
                                    publisher.send(event.to_bytes())
                                status.update()
                        except KeyboardInterrupt:
                            if stream.session_ready:
                                try:
                                    usb.write(session_control(stream.session_id, SESSION_STOP))
                                except (serial.SerialException, OSError):
                                    pass
                            raise
                except (serial.SerialException, OSError) as error:
                    stream.reset_input()
                    status.set_state('RECONNECTING' if connected_once else 'WAITING FOR USB')
                    status.log(f'Serial unavailable ({error}). Retrying in two seconds.')
                    time.sleep(2)
        finally:
            status.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', nargs='?', help='USB serial path (default: auto-detect)')
    binding = parser.add_mutually_exclusive_group()
    binding.add_argument('--bind', default='127.0.0.1',
                         help='Local IPv4 address to listen on (default: %(default)s)')
    binding.add_argument('--tailscale', action='store_true',
                         help="Listen on this host's Tailscale IPv4 address")
    parser.add_argument('--baud', type=int, default=8000000, help='Serial line rate (default: %(default)s)')
    parser.add_argument('--no-status', action='store_true',
                        help='Print logs and device metrics without the interactive status bar')
    args = parser.parse_args()
    if Path(__file__).name == 'zmq.py':
        parser.exit(1, 'Rename this script to zmq_bridge.py so Python can import pyzmq.\n')
    try:
        run(args)
    except ImportError as error:
        parser.exit(1, f'{error}. Install pyserial, pycapnp and pyzmq in the script environment.\n')
    except RuntimeError as error:
        parser.exit(1, f'{error}\n')
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
