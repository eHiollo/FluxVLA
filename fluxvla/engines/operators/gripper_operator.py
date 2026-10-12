"""Standalone A10 USB gripper driver (no BaseOperator or ROS dependency).

Ported from A10_new/gripper.cpp and a10_gripper_bridge.hpp. Widths are in
millimetres; speed is the servo's native unit, NOT mm/s. The 100 mm model's
A10 working range defaults to 0..85 mm. Importing/constructing never moves or
opens the device. Use connect()/disconnect() or a context manager::

    with GripperOperator() as gripper:
        print(gripper.read_state())
        gripper.set_position_mm(40, wait=True)
        gripper.open_gripper(wait=True)
        gripper.close_gripper(wait=True)

The default port follows the current Thor Type-C hub connection. Moving the
cable to another port requires an explicit port argument (CLI: --port).

SYNC_WRITE has no acknowledgement. A successful nonblocking command means
bytes were written, not that motion completed. wait=True checks live feedback.
No torque-disable/stop register is assumed: disconnect only closes the port.
Serial access is serialized, including blocking moves, within this instance.
Do not run the original A10 USB service against the same device concurrently.
"""

import math
import struct
import threading
import time
from dataclasses import dataclass


# Stable physical path for the current Thor Type-C hub port.
DEFAULT_PORT = '/dev/serial/by-path/platform-a80aa10000.usb-usb-0:1.1:1.0-port0'


_CALIBRATION = {
    '100mm': ((0, 3434), (10, 3060), (20, 2824), (30, 2646),
              (40, 2490), (50, 2354), (60, 2226), (70, 2095),
              (80, 1965), (90, 1810), (100, 1594)),
    '50mm': ((0, 2048), (5, 1900), (10, 1775), (15, 1645),
             (20, 1530), (25, 1430), (30, 1320), (35, 1215),
             (40, 1085), (45, 960), (50, 798)),
}


class GripperError(RuntimeError):
    """Connection, protocol or servo error; never interpreted as zero width."""


class GripperTimeoutError(GripperError, TimeoutError):
    """Serial response or physical arrival exceeded its deadline."""


@dataclass(frozen=True)
class GripperState:
    """Live hardware feedback; raw fields follow the original C++ decoding.

    speed_raw/load_raw are signed int16; voltage_raw and temperature_raw are
    bytes. Units/bit encodings are deliberately not inferred from other servos.
    timestamp is a monotonic host timestamp after receipt.
    """

    servo_id: int
    position_raw: int
    position_mm: float
    speed_raw: int
    load_raw: int
    voltage_raw: int
    temperature_raw: int
    timestamp: float


def _finite(value, name, minimum, maximum):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be within {minimum}..{maximum}')
    return value


def _integer(value, name, minimum, maximum):
    number = _finite(value, name, minimum, maximum)
    if number != int(number):
        raise ValueError(f'{name} must be an integer')
    return int(number)


class GripperOperator:
    """Independent serial driver for the A10 calibrated bus-servo gripper.

    Out-of-range targets raise ValueError instead of silently saturating.
    timeout bounds each serial transaction; move_timeout bounds arrival waits.
    pyserial is imported only when connecting (pip install pyserial).
    """

    def __init__(self, port=DEFAULT_PORT, *, baudrate=1000000,
                 servo_id=10, gripper_type='100mm', speed=800,
                 max_width_mm=None, timeout=0.15, move_timeout=5.0,
                 tolerance_mm=1.0, poll_interval=0.033):
        if not isinstance(port, str) or not port:
            raise ValueError('port must be a nonempty device path')
        if gripper_type not in _CALIBRATION:
            raise ValueError('gripper_type must be 100mm or 50mm')
        self.port = port
        self.baudrate = _integer(baudrate, 'baudrate', 1, 4000000)
        self.servo_id = _integer(servo_id, 'servo_id', 0, 253)
        self.gripper_type = gripper_type
        self.speed = _integer(speed, 'speed', 1, 65535)
        default_max = 85 if gripper_type == '100mm' else 50
        self.max_width_mm = _finite(
            default_max if max_width_mm is None else max_width_mm,
            'max_width_mm', 0.001, _CALIBRATION[gripper_type][-1][0])
        self.timeout = _finite(timeout, 'timeout', 0.001, 60)
        self.move_timeout = _finite(move_timeout, 'move_timeout', 0.001, 3600)
        self.tolerance_mm = _finite(tolerance_mm, 'tolerance_mm', 0.001, 10)
        self.poll_interval = _finite(poll_interval, 'poll_interval', 0.001, 10)
        self._serial = None
        self._lock = threading.RLock()
        self._target_mm = None

    @property
    def is_connected(self):
        """Whether the serial handle is open (use ping for live health)."""
        with self._lock:
            return self._serial is not None and self._serial.is_open

    @property
    def target_mm(self):
        """Last transmitted target, never substituted for measured position."""
        with self._lock:
            return self._target_mm

    def connect(self):
        """Open exclusively and verify ping + feedback without commanding motion."""
        with self._lock:
            if self.is_connected:
                return self
            try:
                import serial
            except ImportError as exc:
                raise ImportError('GripperOperator requires pyserial') from exc
            try:
                self._serial = serial.Serial(
                    self.port, baudrate=self.baudrate, timeout=self.timeout,
                    write_timeout=self.timeout, exclusive=True)
                self.ping()
                self.read_state()
            except Exception as exc:
                self.disconnect()
                if isinstance(exc, GripperError):
                    raise
                raise GripperError(f'Cannot connect to {self.port}: {exc}') from exc
            return self

    def disconnect(self):
        """Release serial resources; this does not stop motion or release torque."""
        with self._lock:
            serial_port, self._serial = self._serial, None
            self._target_mm = None
            if serial_port is not None:
                serial_port.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.disconnect()

    def position_to_raw(self, position_mm):
        """Interpolate the original calibration with C++ lround semantics."""
        mm = _finite(position_mm, 'position_mm', 0, self.max_width_mm)
        table = _CALIBRATION[self.gripper_type]
        for (m0, p0), (m1, p1) in zip(table, table[1:]):
            if m0 <= mm <= m1:
                return math.floor(p0 + (mm - m0) / (m1 - m0) * (p1 - p0) + 0.5)
        raise ValueError('Position outside calibration')

    def raw_to_position(self, position_raw):
        """Convert feedback using the full calibration, not the command limit."""
        raw = _integer(position_raw, 'position_raw', 0, 4096)
        table = _CALIBRATION[self.gripper_type]
        for (m0, p0), (m1, p1) in zip(table, table[1:]):
            if p1 <= raw <= p0:
                return m0 + (raw - p0) / (p1 - p0) * (m1 - m0)
        return float(table[0][0] if raw > table[0][1] else table[-1][0])

    def _read_exact(self, size, deadline):
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise GripperTimeoutError(f'Timeout reading {self.port}')
            self._serial.timeout = remaining
            chunk = self._serial.read(size - len(data))
            if not chunk:
                raise GripperTimeoutError(f'Incomplete response from {self.port}')
            data.extend(chunk)
        return bytes(data)

    def _transaction(self, instruction, params=b'', *, broadcast=False,
                     response_size=0, timeout=None):
        with self._lock:
            if not self.is_connected:
                raise GripperError('Gripper is disconnected; call connect() first')
            servo_id = 0xFE if broadcast else self.servo_id
            body = bytes((servo_id, len(params) + 2, instruction)) + params
            packet = b'\xff\xff' + body + bytes((~sum(body) & 0xFF,))
            duration = self.timeout if timeout is None else min(timeout, self.timeout)
            deadline = time.monotonic() + duration
            try:
                self._serial.write_timeout = duration
                self._serial.reset_input_buffer()
                if self._serial.write(packet) != len(packet):
                    raise GripperError('Incomplete serial write')
                if broadcast:
                    return b''
                header = self._read_exact(4, deadline)
                if header[:2] != b'\xff\xff' or header[2] != self.servo_id:
                    raise GripperError(f'Unexpected response header: {header.hex()}')
                if header[3] < 2:
                    raise GripperError('Invalid response length')
                payload = self._read_exact(header[3], deadline)
                if (sum(header[2:]) + sum(payload)) & 0xFF != 0xFF:
                    raise GripperError('Response checksum mismatch')
                if payload[0]:
                    raise GripperError(f'Servo error: 0x{payload[0]:02x}')
                if len(payload) != response_size + 2:
                    raise GripperError('Unexpected response data length')
                return payload[1:-1]
            except GripperError:
                raise
            except OSError as exc:
                raise GripperError(f'Serial I/O failed on {self.port}: {exc}') from exc
            finally:
                self._serial.timeout = self.timeout
                self._serial.write_timeout = self.timeout

    def ping(self):
        """Return True on a healthy response; raise on timeout/device error."""
        self._transaction(0x01)
        return True

    def read_state(self, *, timeout=None):
        """Read the eight feedback bytes at 0x38; failures always raise."""
        if timeout is not None:
            timeout = _finite(timeout, 'timeout', 0.000001, 60)
        data = self._transaction(0x02, b'\x38\x08', response_size=8,
                                 timeout=timeout)
        position, speed, load, voltage, temperature = struct.unpack('<hhhBB', data)
        if not 0 <= position <= 4096:
            raise GripperError(f'Invalid measured servo position: {position}')
        return GripperState(self.servo_id, position, self.raw_to_position(position),
                            speed, load, voltage, temperature, time.monotonic())

    def get_position_mm(self):
        """Read actual opening in mm, without caching or using commanded width."""
        return self.read_state().position_mm

    def set_position_mm(self, position_mm, *, speed=None, wait=False,
                        timeout=None, tolerance_mm=None):
        """Send a target; optionally return the measured state upon arrival.

        A wait timeout does not cancel the servo's outstanding motion.
        """
        raw = self.position_to_raw(position_mm)
        speed = self.speed if speed is None else _integer(speed, 'speed', 1, 65535)
        timeout, tolerance = self._wait_parameters(timeout, tolerance_mm)
        with self._lock:
            # A10_new: SYNC_WRITE 0x2A, six bytes: position, PWM=0x1000, speed.
            params = bytes((0x2A, 6, self.servo_id)) + struct.pack('<HHH', raw, 0x1000, speed)
            self._transaction(0x83, params, broadcast=True)
            self._target_mm = float(position_mm)
            if wait:
                return self.wait_until_reached(timeout=timeout, tolerance_mm=tolerance)
            return None

    def _wait_parameters(self, timeout, tolerance_mm):
        timeout = self.move_timeout if timeout is None else _finite(
            timeout, 'timeout', 0.001, 3600)
        tolerance = self.tolerance_mm if tolerance_mm is None else _finite(
            tolerance_mm, 'tolerance_mm', 0.001, 10)
        return timeout, tolerance

    def wait_until_reached(self, *, timeout=None, tolerance_mm=None):
        """Wait for the last target using hardware feedback, or raise."""
        timeout, tolerance = self._wait_parameters(timeout, tolerance_mm)
        with self._lock:
            if self._target_mm is None:
                raise GripperError('No target has been sent')
            deadline = time.monotonic() + timeout
            last = None
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise GripperTimeoutError(
                        f'Target {self._target_mm:.3f} mm not reached; '
                        f'last measured position={last}')
                state = self.read_state(timeout=max(remaining, 0.000001))
                last = state.position_mm
                if abs(last - self._target_mm) <= tolerance:
                    return state
                time.sleep(min(self.poll_interval, max(0, deadline - time.monotonic())))

    def open_gripper(self, **kwargs):
        """Open to the configured working limit (85 mm by default)."""
        return self.set_position_mm(self.max_width_mm, **kwargs)

    def close_gripper(self, **kwargs):
        """Close to 0 mm; does not disconnect the serial port."""
        return self.set_position_mm(0, **kwargs)


def main(argv=None):
    """Standalone check: python3 path/to/gripper_operator.py [status|move ...]."""
    import argparse
    import json
    from dataclasses import asdict

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs='?', default='status',
                        choices=('status', 'ping', 'move', 'open', 'close'))
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baudrate', type=int, default=1000000)
    parser.add_argument('--servo-id', type=int, default=10)
    parser.add_argument('--gripper-type', choices=tuple(_CALIBRATION), default='100mm')
    parser.add_argument('--position-mm', type=float)
    parser.add_argument('--speed', type=int, default=800)
    parser.add_argument('--move-timeout', type=float, default=5)
    args = parser.parse_args(argv)
    if args.command == 'move' and args.position_mm is None:
        parser.error('move requires --position-mm')
    if args.command != 'move' and args.position_mm is not None:
        parser.error('--position-mm is only valid for move')
    try:
        with GripperOperator(args.port, baudrate=args.baudrate,
                             servo_id=args.servo_id, gripper_type=args.gripper_type,
                             speed=args.speed, move_timeout=args.move_timeout) as gripper:
            if args.command == 'ping':
                print(json.dumps({'ping': gripper.ping()}))
                return 0
            if args.command == 'move':
                state = gripper.set_position_mm(args.position_mm, wait=True)
            elif args.command == 'open':
                state = gripper.open_gripper(wait=True)
            elif args.command == 'close':
                state = gripper.close_gripper(wait=True)
            else:
                state = gripper.read_state()
            print(json.dumps(asdict(state), indent=2))
        return 0
    except (GripperError, ValueError, ImportError) as exc:
        parser.exit(1, f'Gripper check failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
