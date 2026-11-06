"""Independent ChangingTek (知行) gripper driver for policy position targets.

Based on 执行器操作手册（伺服步进）V2.1.0. Run without FluxVLA/ROS imports::

    python3 fluxvla/engines/operators/zx_gripper.py
    python3 fluxvla/engines/operators/zx_gripper.py --id 2
    python3 fluxvla/engines/operators/zx_gripper.py --scan
    python3 fluxvla/engines/operators/zx_gripper.py --position-raw 4720
    python3 fluxvla/engines/operators/zx_gripper.py --reset-alarm
    python3 fluxvla/engines/operators/zx_gripper.py --state
    python3 fluxvla/engines/operators/zx_gripper.py --position 0.5 --no-wait

The manual's connection example uses 115200, 8N1, ID 1. Default operation is
read-only; --position-raw explicitly enables and triggers one temporary-area
move. Position arrival and torque arrival are separate completion reasons;
torque arrival indicates resistance, not proof that an object was grasped.
Timeout, exceptions, interruption and disconnect NEVER automatically disable
or open the gripper. The last device target and torque limit remain in effect;
hardware faults/power loss can still affect holding. Use disable()/--disable
explicitly to release motor torque. A wait timeout ends host waiting only.
Position units in the manual are inconsistent: targets/feedback are raw values,
not millimetres or an assumed 0..1000 range. Manufacturer ROS examples use
0..9000 for CTAG2F90/90D and different scales for other models:
https://gitee.com/Ice_Panda/changingtek/blob/master/crt_ctag2f90d_gripper_visualization/scripts/GUI2Robot_90d.py
The connected unit reached raw 0 exactly. Both 9000 and 9999 commands caused
alarm B=0x1000 near feedback 8993. The default software range is therefore
0..8900, leaving margin below that boundary; this is not a model calibration.
Use --max-position-raw only with a separately established hardware range.
Moves use speed 10%, torque 60% (the manual recommends 60% for gripping).
No homing or flash writes. Alarm reset is explicit; motion never silently clears alarms.
Importing or constructing the client does not open the port.
Requires pyserial (already listed in requirements-a10.txt).

Policy API: set_position_raw(id, raw) or set_position(id, value_0_to_1) returns
after serial acknowledgements, without waiting for physical arrival. A new
target supersedes the previous one; identical targets/parameters do not
retrigger. read_state(id) returns measured state and a monotonic timestamp.
0..1 means raw/max_position_raw, NOT calibrated millimetres or open/close.
No robot/VLA config is switched by this standalone module.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from numbers import Integral, Real
from pathlib import Path
import struct
import threading
import time


DEFAULT_PORT = (
    '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E7IOY-if00-port0'
)
STATUS_REGISTER = 0x0401
ALARM_REGISTER = 0x0402
STATUS_FLAGS = {
    'position_reached': 0x0001,
    'speed_reached': 0x0002,
    'zero_speed': 0x0004,
    'torque_reached': 0x0008,
    'ready': 0x0010,
    'enabled_status_bit': 0x8000,
}


class ZXGripperError(RuntimeError):
    """Transport or malformed Modbus response."""


class ZXGripperInputExpired(ZXGripperError):
    """Input expired before a motion write; never retry the stale target."""


class ZXGripperTimeout(ZXGripperError, TimeoutError):
    """No complete response before the transaction deadline."""


class ZXGripperDeviceError(ZXGripperError):
    """CRC-validated Modbus exception; not a successful register read."""

    def __init__(self, device_id, code):
        self.device_id = device_id
        self.code = code
        super().__init__(f'ID {device_id}: Modbus exception 0x{code:02X}')


@dataclass(frozen=True)
class ZXGripperState:
    """Live measurements; target is never substituted for actual position.

    enabled comes from 0x0100: this unit's 0x0401 bit 0x8000 does not reflect it.
    Speed/current are raw percentages per manual, not mm/s or measured force.
    """

    device_id: int
    position_raw: int
    position: float
    target_raw: int | None
    status: int
    alarm_a: int
    alarm_b: int
    enabled: bool
    speed_raw: int
    current_raw: int
    timestamp: float
    completion: str

    @property
    def position_reached(self):
        return bool(self.status & 0x0001)

    @property
    def torque_reached(self):
        return bool(self.status & 0x0008)

    @property
    def has_error(self):
        return bool(self.alarm_a or self.alarm_b)


def _wait_timeout(value):
    if (isinstance(value, bool) or not isinstance(value, Real)
            or not math.isfinite(value) or not 0 < value <= 60):
        raise ValueError('move_timeout must be within (0, 60] seconds')
    return value


def crc16(data):
    """Modbus CRC16; encoded low byte first on the wire."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def _integer(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f'{name} must be an integer')
    if not minimum <= value <= maximum:
        raise ValueError(f'{name} must be within {minimum}..{maximum}')
    return int(value)


class ZXGripper:
    """Exclusive Modbus client; transactions and target updates are serialized."""

    def __init__(self, port=DEFAULT_PORT, *, baudrate=115200, timeout=0.2,
                 trace=None, max_position_raw=8900,
                 expected_usb_serial=None, expected_usb_interface=None):
        if not isinstance(port, str) or not port:
            raise ValueError('port must be a nonempty device path')
        self.port = port
        self.expected_usb_serial = expected_usb_serial
        self.expected_usb_interface = expected_usb_interface
        self.baudrate = _integer(baudrate, 'baudrate', 1, 4000000)
        self.max_position_raw = _integer(max_position_raw, 'max_position_raw', 1, 0xFFFFFFFF)
        if (isinstance(timeout, bool) or not isinstance(timeout, Real)
                or not math.isfinite(timeout) or not 0 < timeout <= 60):
            raise ValueError('timeout must be finite and within (0, 60] seconds')
        self.timeout = timeout
        self.trace = trace
        self._serial = None
        self._last_activity = 0.0
        self._io_lock = threading.RLock()
        self._commands = {}
        self._command_sequence = 0

    def connect(self):
        with self._io_lock:
            return self._connect()

    def _connect(self):
        if self._serial is None:
            if self.expected_usb_serial is not None:
                self.verify_usb_identity()
            import serial
            try:
                self._serial = serial.Serial(
                    self.port, baudrate=self.baudrate, bytesize=8, parity='N',
                    stopbits=1, timeout=self.timeout,
                    write_timeout=self.timeout, exclusive=True)
            except (OSError, serial.SerialException) as exc:
                raise ZXGripperError(f'Cannot open {self.port}: {exc}') from exc
        return self

    def verify_usb_identity(self):
        node = Path('/sys/class/tty') / Path(self.port).resolve().name
        interface = None
        for parent in node.resolve().parents:
            if (parent / 'bInterfaceNumber').exists() and interface is None:
                interface = (parent / 'bInterfaceNumber').read_text().strip().lower()
            if (parent / 'idVendor').exists():
                serial_file = parent / 'serial'
                serial = serial_file.read_text().strip() if serial_file.exists() else None
                if serial != self.expected_usb_serial or interface != self.expected_usb_interface:
                    raise ZXGripperError('USB identity does not match the configured gripper')
                return
        raise ZXGripperError(f'Cannot verify USB identity: {self.port}')

    def disconnect(self):
        """Close the port only; never release a held object."""
        with self._io_lock:
            port, self._serial = self._serial, None
            self._commands.clear()
            if port is not None:
                port.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.disconnect()

    def read_registers(self, device_id, address, count=1):
        """Read holding registers, enforcing an overall response deadline."""
        _integer(device_id, 'device_id', 1, 247)
        _integer(address, 'address', 0, 0xFFFF)
        _integer(count, 'count', 1, 125)
        if address + count > 0x10000:
            raise ValueError('register range exceeds 0xFFFF')
        body = struct.pack('>BBHH', device_id, 3, address, count)
        response = self._transaction(device_id, body, read_count=count)
        return struct.unpack('>' + 'H' * count, response[3:-2])

    def write_register(self, device_id, address, value):
        """Function 06; verify the full echoed request."""
        _integer(device_id, 'device_id', 1, 247)
        _integer(address, 'address', 0, 0xFFFF)
        _integer(value, 'value', 0, 0xFFFF)
        self._transaction(device_id, struct.pack('>BBHH', device_id, 6, address, value))

    def write_registers(self, device_id, address, values):
        """Function 10; verify the echoed starting address and count."""
        _integer(device_id, 'device_id', 1, 247)
        _integer(address, 'address', 0, 0xFFFF)
        values = tuple(values)
        _integer(len(values), 'count', 1, 123)
        if address + len(values) > 0x10000:
            raise ValueError('register range exceeds 0xFFFF')
        for value in values:
            _integer(value, 'value', 0, 0xFFFF)
        body = struct.pack('>BBHHB', device_id, 16, address, len(values), len(values) * 2)
        body += struct.pack('>' + 'H' * len(values), *values)
        self._transaction(device_id, body)

    def read_position_raw(self, device_id):
        high, low = self.read_registers(device_id, 0x0418, 2)
        return (high << 16) | low

    def reset_alarms(self, device_id):
        """Explicit manual alarm reset (0x0110), verified by live readback."""
        with self._io_lock:
            self._commands.pop(device_id, None)
            self.write_register(device_id, 0x0110, 1)
            time.sleep(0.1)
            alarms = self.read_registers(device_id, ALARM_REGISTER, 2)
            if any(alarms):
                raise ZXGripperError(f'Alarm reset did not clear: A=0x{alarms[0]:04X}, '
                                     f'B=0x{alarms[1]:04X}')
            return alarms

    def read_state(self, device_id=1):
        """Poll actual position, status, alarms, speed/current and enable register."""
        with self._io_lock:
            status, alarm_a, alarm_b = self.read_registers(device_id, STATUS_REGISTER, 3)
            high, low, speed, current = self.read_registers(device_id, 0x0418, 4)
            enabled, = self.read_registers(device_id, 0x0100)
            raw = (high << 16) | low
            command = self._commands.get(device_id)
            target = command[0] if command else None
            if alarm_a or alarm_b:
                completion = 'fault'
            elif not enabled:
                completion = 'disabled'
            elif target is None:
                completion = 'idle'
            elif status & 1 and abs(raw - target) <= 5:
                completion = 'position_reached'
            elif status & 8 and status & 4:
                completion = 'torque_reached'
            else:
                completion = 'moving'
            return ZXGripperState(device_id, raw, raw / self.max_position_raw, target,
                                  status, alarm_a, alarm_b, bool(enabled), speed,
                                  current, time.monotonic(), completion)

    def disable(self, device_id=1):
        """Explicit torque release; may drop an object. Never invoked by timeouts."""
        with self._io_lock:
            self._commands.pop(device_id, None)
            self.write_register(device_id, 0x0100, 0)
            self.write_register(device_id, 0x0108, 0)
            enabled, = self.read_registers(device_id, 0x0100)
            if enabled:
                raise ZXGripperError('Disable was acknowledged but enable readback is nonzero')

    def set_position(self, device_id, position, **options):
        """Normalized policy target: 0 maps to raw 0, 1 to max_position_raw."""
        if (isinstance(position, bool) or not isinstance(position, Real)
                or not math.isfinite(position) or not 0 <= position <= 1):
            raise ValueError('position must be finite and within 0..1')
        return self.set_position_raw(
            device_id, int(position * self.max_position_raw + 0.5), **options)

    def set_position_raw(self, device_id, position, *, speed=10, torque=60,
                         wait=False, move_timeout=10, motion_deadline_s=None, motion_cancel_event=None):
        """Update the latest target, returning a command ID or (wait=True) state.

        wait=False waits for bounded serial ACKs only, not mechanical motion.
        Repeated identical targets retain the existing command and holding force.
        Optional monotonic motion_deadline_s / motion_cancel_event are checked
        before motion writes. Expiry does not undo a target already triggered.
        """
        device_id = _integer(device_id, 'device_id', 1, 247)
        position = _integer(position, 'position', 0, self.max_position_raw)
        speed = _integer(speed, 'speed', 1, 100)
        torque = _integer(torque, 'torque', 1, 100)
        if type(wait) is not bool:
            raise ValueError('wait must be bool')
        _wait_timeout(move_timeout)
        if motion_deadline_s is not None and (isinstance(motion_deadline_s, bool) or
                not isinstance(motion_deadline_s, Real) or not math.isfinite(motion_deadline_s)):
            raise ValueError('motion_deadline_s must be finite')
        def check_deadline():
            if ((motion_deadline_s is not None and time.monotonic() >= motion_deadline_s)
                    or (motion_cancel_event is not None and motion_cancel_event.is_set())):
                raise ZXGripperInputExpired('VR input expired/cancelled before motion write')
        with self._io_lock:
            check_deadline()
            alarm_a, alarm_b = self.read_registers(device_id, ALARM_REGISTER, 2)
            if alarm_a or alarm_b:
                raise ZXGripperError(f'Motion blocked by alarms: {alarm_a:#06x}, {alarm_b:#06x}')
            check_deadline()
            signature = (position, speed, torque)
            previous = self._commands.get(device_id)
            if previous is None or previous[:3] != signature:
                # An uncertain partial write must not leave a reusable cached target.
                self._commands.pop(device_id, None)
                self.write_register(device_id, 0x0108, 0)
                check_deadline()
                self.write_registers(device_id, 0x0102,
                                     (position >> 16, position & 0xFFFF, speed, torque, 1000, 1000))
                check_deadline()
                self.write_register(device_id, 0x0100, 1)
                check_deadline()
                self.write_register(device_id, 0x0108, 1)
                self._command_sequence += 1
                self._commands[device_id] = (*signature, self._command_sequence, time.monotonic())
            command_id = self._commands[device_id][3]
        if wait:
            return self.wait_for_motion(device_id, command_id=command_id, timeout=move_timeout)
        return command_id

    def wait_for_motion(self, device_id=1, *, command_id=None, timeout=10):
        """Wait for position OR sustained torque arrival at zero speed; retain enable.

        Faults fail the wait and block subsequent targets until explicit recovery.
        Superseded commands fail their waiter instead of attributing new feedback
        to the old target. Two successive feedback samples confirm completion.
        """
        _wait_timeout(timeout)
        with self._io_lock:
            command = self._commands.get(device_id)
            if command is None:
                raise ZXGripperError('No known command to wait for')
            expected = command[3] if command_id is None else command_id
        deadline = time.monotonic() + timeout
        consecutive = 0
        previous_reason = None
        last = None
        while time.monotonic() < deadline:
            with self._io_lock:
                command = self._commands.get(device_id)
                if command is None or command[3] != expected:
                    raise ZXGripperError('Motion wait superseded by a newer command or disable')
                last = self.read_state(device_id)
                if last.has_error:
                    raise ZXGripperError(
                        f'Alarm during motion at raw {last.position_raw}: '
                        f'A=0x{last.alarm_a:04X}, B=0x{last.alarm_b:04X}; target={last.target_raw}')
                if not last.enabled:
                    raise ZXGripperError('Motor became disabled during motion')
                # Ignore briefly lingering torque bits from the preceding target.
                settled = time.monotonic() - command[4] >= 0.05
                if last.completion in ('position_reached', 'torque_reached') and settled:
                    consecutive = consecutive + 1 if previous_reason == last.completion else 1
                    previous_reason = last.completion
                    if consecutive >= 2:
                        return last
                else:
                    consecutive = 0
                    previous_reason = None
            time.sleep(min(0.05, max(0, deadline - time.monotonic())))
        raise ZXGripperTimeout(
            f'Motion wait timeout after {timeout}s; last raw='
            f'{last.position_raw if last else None}; target={command[0]}; '
            'enable and target retained (no automatic release)')

    def move_raw(self, device_id, position, *, speed=10, torque=60, move_timeout=10):
        """Compatibility blocking move: return actual position, including contact."""
        state = self.set_position_raw(device_id, position, speed=speed, torque=torque,
                                      wait=True, move_timeout=move_timeout)
        return state.position_raw

    def _transaction(self, device_id, body, *, read_count=None):
        with self._io_lock:
            return self._transaction_locked(device_id, body, read_count=read_count)

    def _transaction_locked(self, device_id, body, *, read_count=None):
        if self._serial is None:
            raise ZXGripperError('Disconnected; call connect() first')
        function = body[1]
        request = body + struct.pack('<H', crc16(body))
        response = bytearray()
        # Modbus RTU recommends a fixed 1.75 ms gap above 19200 baud.
        gap = 0.00175 if self.baudrate > 19200 else 3.5 * 11 / self.baudrate
        time.sleep(max(0, gap - (time.monotonic() - self._last_activity)))
        try:
            self._serial.reset_input_buffer()
            self._serial.write_timeout = self.timeout
            if self.trace:
                self.trace('TX', request)
            if self._serial.write(request) != len(request):
                raise ZXGripperError('Incomplete serial write')
            # Include wire transmission time, also for user-specified low rates.
            deadline = time.monotonic() + self.timeout + len(request) * 10 / self.baudrate

            def read_exact(size):
                target = len(response) + size
                while len(response) < target:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise ZXGripperTimeout(f'ID {device_id}: response timeout')
                    self._serial.timeout = remaining
                    chunk = self._serial.read(target - len(response))
                    if not chunk:
                        raise ZXGripperTimeout(
                            f'ID {device_id}: timeout/incomplete response '
                            f'({len(response)} bytes received)')
                    response.extend(chunk)

            read_exact(3)
            if response[0] != device_id:
                raise ZXGripperError(f'Wrong response ID: {response[0]}, expected {device_id}')
            if response[1] == function | 0x80:
                read_exact(2)
            elif response[1] != function:
                raise ZXGripperError(f'Unexpected function: 0x{response[1]:02X}')
            elif function == 3:
                if response[2] != read_count * 2:
                    raise ZXGripperError(f'Unexpected byte count: {response[2]}')
                read_exact(response[2] + 2)
            else:
                read_exact(5)
            if crc16(response[:-2]) != int.from_bytes(response[-2:], 'little'):
                raise ZXGripperError('Response CRC mismatch')
            if response[1] == function | 0x80:
                raise ZXGripperDeviceError(device_id, response[2])
            if function in (6, 16) and response[:-2] != body[:6]:
                raise ZXGripperError('Write acknowledgement mismatch')
            return response
        except ZXGripperError:
            raise
        except OSError as exc:
            raise ZXGripperError(f'Serial I/O failed: {exc}') from exc
        finally:
            self._last_activity = time.monotonic()
            if self.trace:
                self.trace('RX', bytes(response))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--id', dest='device_id', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=0.2)
    commands = parser.add_mutually_exclusive_group()
    commands.add_argument('--scan', action='store_true', help='Read-only scan of IDs 1..247')
    commands.add_argument('--position-raw', type=int, help='Enable and move to this raw target')
    commands.add_argument('--position', type=float, help='Normalized target in 0..1 (raw/max)')
    commands.add_argument('--state', action='store_true', help='Read full live state as JSON')
    commands.add_argument('--disable', action='store_true', help='Explicitly release motor torque')
    parser.add_argument('--no-wait', action='store_true', help='Return after ACK, retaining target')
    parser.add_argument('--speed', type=int, default=10, help='Move speed percent (default 10)')
    parser.add_argument('--torque', type=int, default=60, help='Move torque percent (default 60)')
    parser.add_argument('--move-timeout', type=float, default=10)
    parser.add_argument('--max-position-raw', type=int, default=8900,
                        help='Software position limit for this unit (default 8900)')
    parser.add_argument('--reset-alarm', action='store_true',
                        help='Explicitly reset alarms and verify; may precede a move')
    args = parser.parse_args(argv)
    has_target = args.position_raw is not None or args.position is not None
    if args.scan and args.reset_alarm:
        parser.error('--scan cannot be combined with --reset-alarm')
    if args.disable and args.reset_alarm:
        parser.error('--disable cannot be combined with --reset-alarm')
    if args.no_wait and not has_target:
        parser.error('--no-wait requires --position-raw or --position')
    try:
        _integer(args.device_id, 'id', 1, 247)
        print(f'Port: {args.port}; {args.baudrate} baud, 8N1; '
              f'timeout: {args.timeout}s; '
              f"IDs: {'1..247' if args.scan else args.device_id}", flush=True)
        found = []
        failed_alarm_reads = []
        def trace(direction, packet):
            print(f'{direction}: {packet.hex(" ") if packet else "<empty>"}', flush=True)
        with ZXGripper(args.port, baudrate=args.baudrate,
                       timeout=args.timeout, trace=trace,
                       max_position_raw=args.max_position_raw) as gripper:
            if args.reset_alarm:
                gripper.reset_alarms(args.device_id)
                print(f'ID {args.device_id}: alarms cleared (verified).', flush=True)
            if args.disable:
                gripper.disable(args.device_id)
                print('Motor disabled (verified); holding torque released.', flush=True)
                return 0
            if args.state:
                state = gripper.read_state(args.device_id)
                print(json.dumps(asdict(state), indent=2), flush=True)
                return 1 if state.has_error else 0
            if has_target:
                method = gripper.set_position_raw if args.position_raw is not None else gripper.set_position
                result = method(args.device_id,
                                args.position_raw if args.position_raw is not None else args.position,
                                speed=args.speed, torque=args.torque, wait=not args.no_wait,
                                move_timeout=args.move_timeout)
                if args.no_wait:
                    print(f'Target acknowledged: command_id={result}; motion not awaited.', flush=True)
                else:
                    print(json.dumps(asdict(result), indent=2), flush=True)
                    print(f'Motion completed: {result.completion}; torque retained.', flush=True)
                return 0
            for device_id in (range(1, 248) if args.scan else (args.device_id,)):
                print(f'Probing ID {device_id}', flush=True)
                try:
                    status, = gripper.read_registers(device_id, STATUS_REGISTER)
                except ZXGripperError as exc:
                    print(f'ID {device_id}: {type(exc).__name__}: {exc}', flush=True)
                    # A transport failure should terminate, not repeat for every ID.
                    if isinstance(exc.__cause__, OSError):
                        raise
                    continue
                found.append(device_id)
                flags = ', '.join(f'{name}={bool(status & mask)}'
                                  for name, mask in STATUS_FLAGS.items())
                print(f'ID {device_id}: communication OK; status=0x{status:04X}; {flags}',
                      flush=True)
                try:
                    alarm_a, alarm_b = gripper.read_registers(device_id, ALARM_REGISTER, 2)
                    print(f'ID {device_id}: alarm_A=0x{alarm_a:04X}; '
                          f'alarm_B_raw={alarm_b} (0x{alarm_b:04X})', flush=True)
                except ZXGripperError as exc:
                    failed_alarm_reads.append(device_id)
                    print(f'ID {device_id}: alarm read failed: {exc}', flush=True)
            print(f'Confirmed IDs: {found}', flush=True)
            if not found:
                print('No valid status response. Check power, A/B wiring, port, '
                      'baudrate and ID; this alone does not prove a device fault.')
            return 0 if found and not failed_alarm_reads else 1
    except (ZXGripperError, ValueError, ImportError) as exc:
        print(f'Check failed: {exc}', flush=True)
        return 1
    except KeyboardInterrupt:
        print('Interrupted; serial port closed; target retained, no automatic release.', flush=True)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
