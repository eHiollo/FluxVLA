"""A10Operator integration with loopback follower, fake cameras and USB servo."""

import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from fluxvla.engines.operators.a10_operator import A10Operator
from test_a10_operator import FakeController
from test_gripper_operator import FakeSerial


class A10PeripheralTest(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        self.capture = MagicMock()
        self.capture.isOpened.return_value = True
        self.capture.read.return_value = (True, np.full((6, 8, 3), [10, 20, 30], dtype=np.uint8))
        for name, value in [('serial.Serial', self.serial),
                            ('cv2.VideoCapture', self.capture)]:
            mock = patch(name, return_value=value)
            mock.start()
            self.addCleanup(mock.stop)
        self.controller = FakeController().__enter__()
        self.addCleanup(self.controller.__exit__, None, None, None)

    def operator(self, **overrides):
        options = dict(command_mode='joint', gripper_transport='usb',
                       gripper=dict(port='/fake/gripper', timeout=.015),
                       cameras=dict(base_0_rgb=dict(device='/fake/base'),
                                    right_wrist_0_rgb=dict(device='/fake/wrist')))
        options.update(overrides)
        return self.controller.operator(**options)

    def test_real_feedback_images_units_and_independent_times(self):
        operator = self.operator()
        self.serial.follow = False
        operator.send_gripper({'right': 50})
        frame = operator.get_frame()
        self.assertEqual(frame['gripper_mm'], 40)  # Actual, not the sent 50.
        self.assertEqual(frame['qpos'][-1], 40)
        np.testing.assert_allclose(frame['qpos'][:6], [0, np.pi/2, -np.pi/2, np.pi, 0, np.pi/4])
        np.testing.assert_array_equal(frame['base_0_rgb'][0, 0], [30, 20, 10])
        self.assertEqual(frame['right_wrist_0_rgb'].dtype, np.uint8)
        self.assertLessEqual(frame['received_monotonic_s'], frame['gripper_received_monotonic_s'])
        self.assertLessEqual(frame['gripper_received_monotonic_s'], frame['camera_received_monotonic_s'])
        self.assertIsNone(operator.get_state().qpos)  # Controller-only unchanged.
        self.assertFalse(any(c.startswith('g_move') for c in self.controller.commands))
        operator.close()
        self.assertFalse(self.serial.is_open)
        self.capture.release.assert_called()
        self.assertFalse(operator.is_connected)

    def test_usb_wait_range_and_read_only_gate(self):
        operator = self.operator()
        self.assertAlmostEqual(operator.send_gripper({'right': 55}, wait=True).position_mm, 55, delta=.05)
        count = len(self.serial.writes)
        with self.assertRaises(ValueError):
            operator.send_gripper({'right': 86})
        self.assertEqual(count, len(self.serial.writes))
        operator.allow_motion = False
        with self.assertRaises(PermissionError):
            operator.send_gripper({'right': 40})
        self.assertEqual(count, len(self.serial.writes))

    def test_serial_connect_failure_releases_network(self):
        self.serial.override = b''
        operator = A10Operator(host='127.0.0.1',
            ws_port=self.controller.server.socket.getsockname()[1],
            gripper_transport='usb', gripper=dict(port='/fake/gripper'))
        with self.assertRaises(Exception):
            operator.connect()
        self.assertFalse(operator.is_connected)
        self.assertFalse(self.serial.is_open)

    def test_sensor_failure_is_not_a_synthetic_frame(self):
        for sensor in ('usb', 'camera'):
            with self.subTest(sensor=sensor):
                self.serial.is_open = True
                self.serial.override = None
                self.capture.isOpened.return_value = True
                self.capture.read.return_value = (True, np.zeros((6, 8, 3), dtype=np.uint8))
                operator = self.operator()
                if sensor == 'usb':
                    self.serial.override = b''
                else:
                    self.capture.read.return_value = (False, None)
                with self.assertRaises(Exception):
                    operator.get_frame()
                self.assertFalse(operator.is_connected)
                self.assertFalse(self.serial.is_open)

    def test_invalid_configuration_no_io(self):
        for options in (dict(gripper_transport='usb'),
                        dict(gripper_transport='controller', gripper={}),
                        dict(cameras={'qpos': dict(device=0)})):
            with self.subTest(options=options), self.assertRaises(ValueError):
                A10Operator(host='127.0.0.1', **options)

    def test_close_attempts_every_resource_on_error(self):
        operator = self.operator()
        with patch.object(operator.camera_operator, 'close', side_effect=OSError('camera cleanup')):
            with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
                operator.close()
        self.assertFalse(operator.is_connected)
        self.assertFalse(self.serial.is_open)


if __name__ == '__main__':
    unittest.main()
