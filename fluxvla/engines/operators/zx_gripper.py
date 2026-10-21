"""ChangingTek (知行) servo-stepper gripper communication and single-move check.

Based on 执行器操作手册（伺服步进）V2.1.0. Run without FluxVLA/ROS imports::

    python3 fluxvla/engines/operators/zx_gripper.py
    python3 fluxvla/engines/operators/zx_gripper.py --id 2
    python3 fluxvla/engines/operators/zx_gripper.py --scan
    python3 fluxvla/engines/operators/zx_gripper.py --position-raw 4720
    python3 fluxvla/engines/operators/zx_gripper.py --reset-alarm

The manual's connection example uses 115200, 8N1, ID 1. Default operation is
read-only; --position-raw explicitly enables and triggers one temporary-area
move. Successful moves retain torque; disconnect only closes the serial port.
Failed/interrupted moves attempt disable (not a guaranteed emergency stop).
Position units in the manual are inconsistent: targets/feedback are raw values,
not millimetres or an assumed 0..1000 range. Manufacturer ROS examples use
0..9000 for CTAG2F90/90D and different scales for other models:
https://gitee.com/Ice_Panda/changingtek/blob/master/crt_ctag2f90d_gripper_visualization/scripts/GUI2Robot_90d.py
The connected unit reached raw 0 exactly. Both 9000 and 9999 commands caused
alarm B=0x1000 near feedback 8993. The default software range is therefore
0..8900, leaving margin below that boundary; this is not a model calibration.
Use --max-position-raw only with a separately established hardware range.
Moves use speed 10%, torque 60% (the manual recommends 60% for gripping).
No homing or flash
writes. Alarm reset is explicit; motion never silently clears alarms.
Importing or constructing the client does not open the port.
Requires pyserial (already listed in requirements-a10.txt).
"""

import argparse
import math
import struct
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
    'enabled': 0x8000,
}


class ZXGripperError(RuntimeError):
    """Transport or malformed Modbus response."""


class ZXGripperTimeout(ZXGripperError, TimeoutError):
    """No complete response before the transaction deadline."""


class ZXGripperDeviceError(ZXGripperError):
    """CRC-validated Modbus exception; not a successful register read."""

    def __init__(self, device_id, code):
        self.device_id = device_id
        self.code = code
        super().__init__(f'ID {device_id}: Modbus exception 0x{code:02X}')


def crc16(data):
    """Modbus CRC16; encoded low byte first on the wire."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def _integer(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{name} must be an integer')
    if not minimum <= value <= maximum:
        raise ValueError(f'{name} must be within {minimum}..{maximum}')
    return value


class ZXGripper:
    """Single-threaded, exclusive Modbus RTU client."""

    def __init__(self, port=DEFAULT_PORT, *, baudrate=115200, timeout=0.2,
                 trace=None, max_position_raw=8900):
        if not isinstance(port, str) or not port:
            raise ValueError('port must be a nonempty device path')
        self.port = port
        self.baudrate = _integer(baudrate, 'baudrate', 1, 4000000)
        self.max_position_raw = _integer(max_position_raw, 'max_position_raw', 1, 0xFFFFFFFF)
        if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                or not math.isfinite(timeout) or not 0 < timeout <= 60):
            raise ValueError('timeout must be finite and within (0, 60] seconds')
        self.timeout = timeout
        self.trace = trace
        self._serial = None
        self._last_activity = 0.0

    def connect(self):
        if self._serial is None:
            import serial
            try:
                self._serial = serial.Serial(
                    self.port, baudrate=self.baudrate, bytesize=8, parity='N',
                    stopbits=1, timeout=self.timeout,
                    write_timeout=self.timeout, exclusive=True)
            except (OSError, serial.SerialException) as exc:
                raise ZXGripperError(f'Cannot open {self.port}: {exc}') from exc
        return self

    def disconnect(self):
        port, self._serial = self._serial, None
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
        self.write_register(device_id, 0x0110, 1)
        time.sleep(0.1)
        alarms = self.read_registers(device_id, ALARM_REGISTER, 2)
        if any(alarms):
            raise ZXGripperError(f'Alarm reset did not clear: A=0x{alarms[0]:04X}, '
                                 f'B=0x{alarms[1]:04X}')
        return alarms

    def move_raw(self, device_id, position, *, speed=10, torque=60, move_timeout=10):
        """One temporary-area move; require live position arrival, not write ACK."""
        _integer(device_id, 'device_id', 1, 247)
        _integer(position, 'position', 0, self.max_position_raw)
        _integer(speed, 'speed', 1, 100)
        _integer(torque, 'torque', 1, 100)
        if (isinstance(move_timeout, bool) or not isinstance(move_timeout, (int, float))
                or not math.isfinite(move_timeout) or not 0 < move_timeout <= 60):
            raise ValueError('move_timeout must be within (0, 60] seconds')
        # Read before writing: faults or a broken connection must block motion.
        alarm_a, alarm_b = self.read_registers(device_id, ALARM_REGISTER, 2)
        if alarm_a or alarm_b:
            raise ZXGripperError(f'Motion blocked by alarms: {alarm_a:#06x}, {alarm_b:#06x}')
        start = self.read_position_raw(device_id)
        print(f'Move ID {device_id}: raw {start} -> {position}; '
              f'speed={speed}%, torque={torque}%', flush=True)
        try:
            self.write_register(device_id, 0x0108, 0)
            self.write_registers(device_id, 0x0102,
                                 (position >> 16, position & 0xFFFF, speed, torque, 1000, 1000))
            self.write_register(device_id, 0x0100, 1)
            self.write_register(device_id, 0x0108, 1)
            deadline = time.monotonic() + move_timeout
            consecutive = 0
            while time.monotonic() < deadline:
                status, alarm_a, alarm_b = self.read_registers(device_id, STATUS_REGISTER, 3)
                current = self.read_position_raw(device_id)
                print(f'Feedback: raw={current}, status=0x{status:04X}, '
                      f'alarms=0x{alarm_a:04X}/0x{alarm_b:04X}', flush=True)
                if alarm_a or alarm_b:
                    raise ZXGripperError(
                        f'Alarm during motion at raw {current}: '
                        f'A=0x{alarm_a:04X}, B=0x{alarm_b:04X}; target={position}')
                consecutive = consecutive + 1 if abs(current - position) <= 5 and status & 1 else 0
                if consecutive >= 2:
                    self.write_register(device_id, 0x0108, 0)
                    return current
                time.sleep(0.1)
            raise ZXGripperTimeout(f'Move to raw {position} did not arrive within {move_timeout}s')
        except (Exception, KeyboardInterrupt):
            try:
                self.write_register(device_id, 0x0100, 0)
            except Exception as cleanup_error:
                print(f'Disable attempt failed: {cleanup_error}', flush=True)
            raise

    def _transaction(self, device_id, body, *, read_count=None):
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
    parser.add_argument('--scan', action='store_true', help='Read-only scan of IDs 1..247')
    parser.add_argument('--position-raw', type=int, help='Enable and move once to this raw target')
    parser.add_argument('--speed', type=int, default=10, help='Move speed percent (default 10)')
    parser.add_argument('--torque', type=int, default=60, help='Move torque percent (default 60)')
    parser.add_argument('--move-timeout', type=float, default=10)
    parser.add_argument('--max-position-raw', type=int, default=8900,
                        help='Software position limit for this unit (default 8900)')
    parser.add_argument('--reset-alarm', action='store_true',
                        help='Explicitly reset alarms and verify; may precede a move')
    args = parser.parse_args(argv)
    if args.scan and (args.position_raw is not None or args.reset_alarm):
        parser.error('--scan cannot be combined with --position-raw or --reset-alarm')
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
            if args.position_raw is not None:
                position = gripper.move_raw(args.device_id, args.position_raw,
                                            speed=args.speed, torque=args.torque,
                                            move_timeout=args.move_timeout)
                print(f'Move complete: raw position={position}; torque retained.', flush=True)
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
        print('Interrupted; serial port closed.', flush=True)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
