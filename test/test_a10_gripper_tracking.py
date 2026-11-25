"""100 Hz motion feedback tolerates dropped samples without arrival waits."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from test_a10_dual_inference import b, Clock, ROOT
from fluxvla.engines.operators.zx_gripper import ZXGripperCommunicationError


class GripperStreamingTests(unittest.TestCase):
    def fixture(self, cycles=3):
        clock = Clock()
        device = object.__new__(b.LiveDevices)
        device.lock = threading.Lock()
        device.write_locks = dict(left=threading.Lock())
        device.cancel = threading.Event()
        device.feedback, device.gripper_faults = {}, {}
        device.error, device.gripper_pause_required = None, False
        device.cfg = dict(bridge=dict(max_gripper_age_s=.2))
        device.log = Mock()
        row = np.zeros(14)
        row[6] = .25
        device.target = (row, 1.)
        initial = SimpleNamespace(position_raw=1000, has_error=False, enabled=True)
        motion = SimpleNamespace(position_raw=1200, speed_raw=5, current_raw=10)
        grip = Mock()
        grip.read_state.return_value = initial
        grip.read_motion_state.return_value = motion
        grip.set_position_raw.return_value = 1
        device.grips = dict(left=grip)

        class Quit:
            ticks = 0
            snapshots = []

            def is_set(self):
                return self.ticks >= cycles

            def wait(self, duration):
                self.snapshots.append(dict(device.feedback))
                self.ticks += 1
                clock.sleep(duration)

        device.quit = Quit()
        return device, clock, grip

    def run_worker(self, device, clock):
        with patch.object(b.time, 'monotonic', side_effect=clock.monotonic):
            device._gripper_worker('left', dict(device_id=1, speed=30, torque=60,
                                                io_hz=100, feedback_mode='motion', compact_trigger=True))

    def test_only_startup_reads_health_then_motion_at_100hz(self):
        device, clock, grip = self.fixture()
        self.run_worker(device, clock)
        self.assertIsNone(device.error)
        self.assertAlmostEqual(clock.now, .03)
        grip.read_state.assert_called_once()
        self.assertEqual(grip.read_motion_state.call_count, 2)
        self.assertEqual(grip.set_position_raw.call_count, 3)
        self.assertTrue(grip.set_position_raw.call_args.kwargs['compact_trigger'])
        self.assertEqual(device.feedback['left'][0], 1200)
        logs = [c.kwargs for c in device.log.call_args_list if c.args[0] == 'gripper_feedback']
        self.assertIsNone(logs[-1]['enabled'])
        self.assertIsNone(logs[-1]['alarm_a'])
        self.assertEqual(logs[-1]['health_received_s'], 0.)

    def test_single_bad_sample_keeps_old_timestamp_and_does_not_pause(self):
        device, clock, grip = self.fixture()
        grip.read_motion_state.side_effect = [ZXGripperCommunicationError('bad CRC'),
                                               SimpleNamespace(position_raw=1500)]
        self.run_worker(device, clock)
        self.assertIsNone(device.error)
        self.assertFalse(device.gripper_pause_required)
        grip.disconnect.assert_not_called()
        self.assertEqual(grip.read_motion_state.call_count, 2)
        self.assertEqual(device.quit.snapshots[0]['left'][1], device.quit.snapshots[1]['left'][1])
        self.assertEqual(device.feedback['left'][0], 1500)
        self.assertEqual(len([c for c in device.log.call_args_list
                              if c.args[0] == 'gripper_feedback_dropped']), 1)

    def test_prolonged_feedback_loss_still_pauses(self):
        device, clock, grip = self.fixture(cycles=2)

        def lost(_):
            clock.sleep(.25)
            raise ZXGripperCommunicationError('timeout')

        grip.read_motion_state.side_effect = lost
        self.run_worker(device, clock)
        self.assertTrue(device.gripper_pause_required)
        self.assertTrue(device.cancel.is_set())
        self.assertIsNone(device.target)
        grip.disconnect.assert_called_once()

    def test_latest_target_after_read_is_used_without_waiting_for_arrival(self):
        device, clock, grip = self.fixture(cycles=1)
        initial = grip.read_state.return_value

        def read(_):
            fresh = device.target[0].copy()
            fresh[6] = .5
            device.target = (fresh, 1.)
            return initial

        grip.read_state.side_effect = read
        self.run_worker(device, clock)
        self.assertEqual(grip.set_position_raw.call_args.args, (1, 4500))
        self.assertAlmostEqual(clock.now, .01)

    def test_invalid_streaming_config_rejected_before_hardware(self):
        from mmengine import Config
        from fluxvla.engines import build_runner_from_cfg

        for key, value in [('io_hz', 0), ('io_hz', float('nan')), ('io_hz', True),
                           ('serial_timeout_s', -1), ('feedback_mode', 'fastest'),
                           ('compact_trigger', 'yes')]:
            with self.subTest(key=key, value=value):
                cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_inference.py')
                cfg.inference.mode = 'preview'
                cfg.grippers.left[key] = value
                cfg.inference.cfg = cfg
                runner = build_runner_from_cfg(cfg.inference)
                with self.assertRaisesRegex(ValueError, key):
                    runner.run_setup()


if __name__ == '__main__':
    unittest.main()
