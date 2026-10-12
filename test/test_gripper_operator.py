"""Protocol/failure tests without ROS, model dependencies or physical motion.

Run standalone: python3 -m unittest discover -s test -p test_gripper_operator.py
"""

import importlib.util
import os
from pathlib import Path
import pty
import struct
import sys
import threading
import unittest
from unittest.mock import patch

# This driver is intentionally usable without FluxVLA's model dependencies.
_spec = importlib.util.spec_from_file_location(
    '_standalone_gripper', Path(__file__).resolve().parents[1]
    / 'fluxvla/engines/operators/gripper_operator.py')
module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = module
_spec.loader.exec_module(module)
GripperOperator = module.GripperOperator
GripperError = module.GripperError


def reply(data=b'', error=0, servo_id=10):
    body = bytes((servo_id, len(data) + 2, error)) + data
    return b'\xff\xff' + body + bytes((255 - sum(body) % 256,))


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.timeout = kwargs.get('timeout', .15)
        self.write_timeout = self.timeout
        self.writes = []
        self.rx = b''
        self.position = 2490
        self.override = None
        self.follow = True
        self.short_write = False

    def close(self):
        self.is_open = False

    def reset_input_buffer(self):
        self.rx = b''

    def write(self, packet):
        self.writes.append(packet)
        if self.short_write:
            return 1
        if packet[4] == 0x83:
            if self.follow:
                self.position = int.from_bytes(packet[8:10], 'little')
        elif self.override is not None:
            self.rx = self.override
        elif packet[4] == 1:
            self.rx = reply()
        else:
            self.rx = reply(struct.pack('<hhhBB', self.position, -12, 30, 120, 25))
        return len(packet)

    def read(self, count):
        # Fragment every response to exercise exact-read accumulation.
        count = min(count, 1)
        data, self.rx = self.rx[:count], self.rx[count:]
        return data


class GripperTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        self.patcher = patch('serial.Serial', return_value=self.fake)
        self.factory = self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.gripper = GripperOperator(poll_interval=.001)
        self.addCleanup(self.gripper.disconnect)

    def test_connect_is_read_only_and_context_cleans_up(self):
        with self.gripper as g:
            self.assertTrue(g.is_connected)
            self.assertEqual(g.get_position_mm(), 40)
            self.assertIsNone(g.target_mm)
        self.assertFalse(g.is_connected)
        self.assertEqual(self.fake.writes[0], bytes.fromhex('ff ff 0a 02 01 f2'))
        self.assertEqual(self.fake.writes[1], bytes.fromhex('ff ff 0a 04 02 38 08 af'))
        self.assertTrue(all(p[4] in (1, 2) for p in self.fake.writes))
        self.assertTrue(self.factory.call_args.kwargs['exclusive'])

    def test_move_wire_and_real_feedback(self):
        g = self.gripper.connect()
        g.set_position_mm(40)
        self.assertEqual(self.fake.writes[-1], bytes.fromhex(
            'ff ff fe 0b 83 2a 06 0a ba 09 00 10 20 03 43'))
        self.fake.position = 2354
        state = g.read_state()
        self.assertEqual((state.position_mm, g.target_mm), (50, 40))
        self.assertEqual((state.speed_raw, state.load_raw,
                          state.voltage_raw, state.temperature_raw), (-12, 30, 120, 25))

    def test_open_close_and_wait(self):
        g = self.gripper.connect()
        self.assertAlmostEqual(g.open_gripper(wait=True).position_mm, 85, delta=.05)
        self.assertEqual(g.close_gripper(wait=True).position_mm, 0)
        count = len(self.fake.writes)
        g.close_gripper()
        self.assertEqual(len(self.fake.writes), count + 1)

    def test_wait_timeout_on_obstruction(self):
        g = self.gripper.connect()
        self.fake.follow = False
        with self.assertRaises(module.GripperTimeoutError):
            g.set_position_mm(0, wait=True, timeout=.01)
        self.assertEqual(g.get_position_mm(), 40)

    def test_feedback_errors_never_become_closed(self):
        g = self.gripper.connect()
        samples = [b'', reply()[:3], b'bad!', reply(bytes(8), servo_id=11),
                   reply(bytes(8), error=4), reply(bytes(7)),
                   reply(bytes(8))[:-1] + b'\x00', b'\xff\xff\x0a\x01',
                   reply(struct.pack('<hhhBB', -1, 0, 0, 0, 0))]
        for packet in samples:
            with self.subTest(packet=packet):
                self.fake.override = packet
                with self.assertRaises(GripperError):
                    g.get_position_mm()

    def test_connect_failure_closes_handle(self):
        self.fake.override = b''
        with self.assertRaises(module.GripperTimeoutError):
            self.gripper.connect()
        self.assertFalse(self.gripper.is_connected)
        self.assertFalse(self.fake.is_open)

    def test_disconnected_and_short_write(self):
        with self.assertRaises(GripperError):
            self.gripper.set_position_mm(10)
        g = self.gripper.connect()
        self.fake.short_write = True
        with self.assertRaises(GripperError):
            g.set_position_mm(10)
        self.assertIsNone(g.target_mm)

    def test_validation_precedes_motion(self):
        g = self.gripper.connect()
        count = len(self.fake.writes)
        for value in (-1, 86, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                g.set_position_mm(value)
        for options in ({'speed': 0}, {'speed': 1.5}, {'timeout': 0},
                        {'tolerance_mm': float('nan')}):
            with self.assertRaises(ValueError):
                g.set_position_mm(20, **options)
        self.assertEqual(len(self.fake.writes), count)

    def test_calibration_reference_and_rounding(self):
        g = self.gripper
        for mm, raw in ((0, 3434), (10, 3060), (40, 2490), (85, 1888)):
            self.assertEqual(g.position_to_raw(mm), raw)
            self.assertAlmostEqual(g.raw_to_position(raw), mm, delta=.04)
        self.assertEqual(g.raw_to_position(1594), 100)
        small = GripperOperator(gripper_type='50mm')
        self.assertEqual(small.position_to_raw(50), 798)
        self.assertEqual(small.position_to_raw(0), 2048)
        self.assertEqual(small.raw_to_position(1430), 25)
        with self.assertRaises(ValueError):
            GripperOperator(gripper_type='50mm', max_width_mm=85)


class SerialIntegrationTest(unittest.TestCase):
    def test_real_pyserial_over_pseudoterminal(self):
        """Exercise OS serial open/configure, framing, and actual read/write."""
        master, slave = pty.openpty()
        stop = threading.Event()
        errors = []

        def servo():
            import select
            buffer = b''
            position = 2490
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], .05)[0]:
                        continue
                    buffer += os.read(master, 256)
                    while len(buffer) >= 4 and len(buffer) >= buffer[3] + 4:
                        size = buffer[3] + 4
                        frame, buffer = buffer[:size], buffer[size:]
                        if frame[:2] != b'\xff\xff' or sum(frame[2:]) % 256 != 255:
                            raise AssertionError('Invalid request frame')
                        if frame[4] == 0x83:
                            position = int.from_bytes(frame[8:10], 'little')
                        elif frame[4] == 1:
                            os.write(master, reply())
                        elif frame[4] == 2:
                            os.write(master, reply(struct.pack('<hhhBB', position, 0, 0, 120, 25)))
                        else:
                            raise AssertionError('Unexpected instruction')
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=servo, daemon=True)
        thread.start()
        try:
            with GripperOperator(os.ttyname(slave)) as g:
                self.assertEqual(g.get_position_mm(), 40)
                self.assertAlmostEqual(g.set_position_mm(60, wait=True).position_mm, 60)
                self.assertAlmostEqual(g.open_gripper(wait=True).position_mm, 85, delta=.05)
                self.assertEqual(g.close_gripper(wait=True).position_mm, 0)
        finally:
            stop.set()
            thread.join(2)
            os.close(master)
            os.close(slave)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])


if __name__ == '__main__':
    unittest.main()
