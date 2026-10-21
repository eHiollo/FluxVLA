"""Standalone protocol tests: python3 -m unittest discover -s test -p test_zx_gripper.py."""

import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import unittest
from unittest.mock import patch


_spec = importlib.util.spec_from_file_location(
    'zx_gripper', Path(__file__).resolve().parents[1]
    / 'fluxvla/engines/operators/zx_gripper.py')
zx = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(zx)


def frame(body):
    return body + struct.pack('<H', zx.crc16(body))


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.rx = b''
        self.override = None
        self.short_write = False
        self.closed = False
        self.only_id = None
        self.position = 4820
        self.target = self.position
        self.follow = True
        self.alarms = (0x0030, 1000)

    def reset_input_buffer(self):
        self.rx = b''

    def write(self, request):
        self.writes.append(request)
        device_id, function, address, count = struct.unpack('>BBHH', request[:6])
        assert zx.crc16(request[:-2]) == int.from_bytes(request[-2:], 'little')
        if self.override is not None:
            self.rx = self.override
        elif function in (6, 16):
            self.rx = frame(request[:6])
            if function == 16:
                high, low = struct.unpack('>HH', request[7:11])
                self.target = (high << 16) | low
            elif address == 0x0108 and count == 1 and self.follow:
                self.position = self.target
        elif self.only_id is None or device_id == self.only_id:
            if address == zx.STATUS_REGISTER:
                values = (0x8011,) if count == 1 else (0x8011, *self.alarms)
            elif address == 0x0418:
                values = (self.position >> 16, self.position & 0xFFFF)
            else:
                values = self.alarms
            self.rx = frame(bytes((device_id, 3, count * 2))
                            + struct.pack('>' + 'H' * count, *values))
        return 1 if self.short_write else len(request)

    def read(self, count):
        data, self.rx = self.rx[:1], self.rx[1:]
        return data

    def close(self):
        self.closed = True


class ZXGripperTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        serial_patch = patch('serial.Serial', return_value=self.fake)
        self.factory = serial_patch.start()
        self.addCleanup(serial_patch.stop)
        self.client = zx.ZXGripper()
        self.addCleanup(self.client.disconnect)

    def test_manual_request_and_fragmented_response(self):
        with self.client as client:
            self.assertEqual(client.read_registers(1, 0x0401), (0x8011,))
            self.assertEqual(self.fake.writes[-1], bytes.fromhex('01 03 04 01 00 01 d4 fa'))
            self.assertEqual(client.read_registers(1, 0x0402, 2), (0x30, 1000))
        self.assertTrue(self.fake.closed)
        options = self.factory.call_args.kwargs
        self.assertEqual((options['baudrate'], options['bytesize'],
                          options['parity'], options['stopbits']), (115200, 8, 'N', 1))
        self.assertTrue(options['exclusive'])

    def test_construction_and_connection_do_not_send_commands(self):
        self.factory.assert_not_called()
        self.client.connect()
        self.assertEqual(self.fake.writes, [])

    def test_bad_responses(self):
        self.client.connect()
        responses = (
            (b'', zx.ZXGripperTimeout, 'timeout'),
            (b'\x01\x03', zx.ZXGripperTimeout, 'incomplete'),
            (frame(bytes.fromhex('02 03 02 00 01')), zx.ZXGripperError, 'ID'),
            (frame(bytes.fromhex('01 04 02 00 01')), zx.ZXGripperError, 'function'),
            (frame(bytes.fromhex('01 03 04 00 00 00 01')), zx.ZXGripperError, 'byte count'),
            (bytes.fromhex('01 03 02 00 01 00 00'), zx.ZXGripperError, 'CRC'),
            (frame(bytes.fromhex('01 83 02')), zx.ZXGripperDeviceError, 'exception'),
            (bytes.fromhex('01 83 02 00 00'), zx.ZXGripperError, 'CRC'),
        )
        for response, error, message in responses:
            with self.subTest(response=response):
                self.fake.override = response
                with self.assertRaisesRegex(error, message):
                    self.client.read_registers(1, 0x0401)

    def test_validation_and_short_write(self):
        for options in ({'timeout': float('nan')}, {'timeout': 0},
                        {'baudrate': 0}, {'timeout': True}):
            with self.assertRaises(ValueError):
                zx.ZXGripper(**options)
        self.client.connect()
        for args in ((0, 0x0401), (248, 0x0401), (True, 0x0401),
                     (1, -1), (1, 0xFFFF, 2), (1, 0x0401, 126)):
            with self.assertRaises(ValueError):
                self.client.read_registers(*args)
        self.assertEqual(self.fake.writes, [])
        self.fake.short_write = True
        with self.assertRaisesRegex(zx.ZXGripperError, 'Incomplete serial write'):
            self.client.read_registers(1, 0x0401)

    def test_cli_default_and_failed_alarm_read(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(zx.main([]), 0)
        self.assertIn('Confirmed IDs: [1]', output.getvalue())
        self.assertIn('alarm_A=0x0030', output.getvalue())
        self.assertEqual(len(self.fake.writes), 2)
        self.assertTrue(self.fake.closed)
        with patch.object(zx.ZXGripper, 'read_registers', side_effect=[
                (0x8011,), zx.ZXGripperTimeout('alarm timeout')]):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(zx.main([]), 1)

    def test_cli_scan_reports_all_ids_and_never_broadcasts(self):
        self.fake.only_id = 7
        output = io.StringIO()
        with contextlib.redirect_stdout(output), patch.object(zx.time, 'sleep'):
            self.assertEqual(zx.main(['--scan']), 0)
        self.assertIn('Confirmed IDs: [7]', output.getvalue())
        self.assertEqual(len(self.fake.writes), 248)  # 247 status + one alarm read
        self.assertTrue(all(1 <= p[0] <= 247 and p[1] == 3 for p in self.fake.writes))

    def test_cli_no_response_or_exception_is_not_success(self):
        for response in (b'', frame(bytes.fromhex('01 83 02'))):
            self.fake.override = response
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(zx.main([]), 1)
            self.assertIn('Confirmed IDs: []', output.getvalue())

    def test_io_failure_and_context_cleanup(self):
        with self.assertRaisesRegex(zx.ZXGripperError, 'Serial I/O failed'):
            with self.client:
                with patch.object(self.fake, 'read', side_effect=OSError('unplugged')):
                    self.client.read_registers(1, 0x0401)
        self.assertTrue(self.fake.closed)
        with self.assertRaisesRegex(zx.ZXGripperError, 'Disconnected'):
            self.client.read_registers(1, 0x0401)

    def test_manual_write_frames_and_bad_echo(self):
        self.client.connect()
        self.client.write_register(1, 0x0100, 1)
        self.assertEqual(self.fake.writes[-1], bytes.fromhex('01 06 01 00 00 01 49 f6'))
        self.client.write_registers(1, 0x0102, (0, 200))
        self.assertEqual(self.fake.writes[-1], bytes.fromhex(
            '01 10 01 02 00 02 04 00 00 00 c8 7e 70'))
        self.fake.override = frame(bytes.fromhex('01 06 01 00 00 00'))
        with self.assertRaisesRegex(zx.ZXGripperError, 'acknowledgement'):
            self.client.write_register(1, 0x0100, 1)
        self.fake.override = frame(bytes.fromhex('01 90 02'))
        with self.assertRaises(zx.ZXGripperDeviceError):
            self.client.write_registers(1, 0x0102, (0, 200))

    def test_motion_validated_feedback_and_parameters(self):
        self.fake.alarms = (0, 0)
        self.client.connect()
        with contextlib.redirect_stdout(io.StringIO()), patch.object(zx.time, 'sleep'):
            self.assertEqual(self.client.move_raw(1, 4720), 4720)
        writes = [p for p in self.fake.writes if p[1] != 3]
        self.assertEqual([p[1] for p in writes], [6, 16, 6, 6, 6])
        self.assertEqual(struct.unpack('>6H', writes[1][7:-2]), (0, 4720, 10, 60, 1000, 1000))
        self.assertEqual(writes[-1][2:6], bytes.fromhex('01 08 00 00'))

    def test_motion_alarms_validation_and_timeout(self):
        self.client.connect()
        with self.assertRaisesRegex(zx.ZXGripperError, 'blocked by alarms'):
            self.client.move_raw(1, 4720)
        self.assertTrue(all(p[1] == 3 for p in self.fake.writes))
        self.fake.writes.clear()
        for options in ({'speed': 0}, {'torque': 101}, {'move_timeout': float('nan')}):
            with self.assertRaises(ValueError):
                self.client.move_raw(1, 4720, **options)
        self.assertEqual(self.fake.writes, [])
        self.fake.alarms = (0, 0)
        self.fake.follow = False
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(zx.ZXGripperTimeout):
                self.client.move_raw(1, 4720, move_timeout=0.01)
        self.assertEqual(self.fake.writes[-1][1:6], bytes.fromhex('06 01 00 00 00'))

    def test_alarm_during_motion_disables_and_reports_raw_code(self):
        self.fake.alarms = (0, 0)
        self.client.connect()
        original_read = self.client.read_registers

        def read(device_id, address, count=1):
            if address == zx.STATUS_REGISTER and count == 3:
                self.fake.alarms = (0, 0x1000)
            return original_read(device_id, address, count)

        with patch.object(self.client, 'read_registers', side_effect=read):
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(zx.ZXGripperError, 'B=0x1000; target=4720'):
                    self.client.move_raw(1, 4720)
        self.assertEqual(self.fake.writes[-1][1:6], bytes.fromhex('06 01 00 00 00'))

    def test_explicit_reset_requires_zero_alarm_readback(self):
        self.client.connect()
        with self.assertRaisesRegex(zx.ZXGripperError, 'Alarm reset did not clear'):
            self.client.reset_alarms(1)
        self.assertEqual(self.fake.writes[0], bytes.fromhex('01 06 01 10 00 01 48 33'))
        self.fake.alarms = (0, 0)
        self.assertEqual(self.client.reset_alarms(1), (0, 0))

    def test_cli_motion_and_reset(self):
        self.fake.alarms = (0, 0)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(zx.main(['--reset-alarm', '--position-raw', '4720']), 0)
        self.assertEqual(self.fake.position, 4720)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                zx.main(['--scan', '--reset-alarm'])

    def test_observed_range_limit_precedes_any_io(self):
        self.client.connect()
        for target in (-1, 8901, 9000, 9999):
            with self.assertRaisesRegex(ValueError, 'position must be within 0..8900'):
                self.client.move_raw(1, target)
        self.assertEqual(self.fake.writes, [])
        with self.assertRaises(ValueError):
            zx.ZXGripper(max_position_raw=0)
        self.assertEqual(zx.ZXGripper(max_position_raw=1000).max_position_raw, 1000)


if __name__ == '__main__':
    unittest.main()
