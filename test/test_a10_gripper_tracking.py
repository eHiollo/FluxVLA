"""100 Hz motion feedback tolerates dropped samples without arrival waits."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from test_a10_dual_inference import b, Clock, ROOT
from fluxvla.engines.operators.zx_gripper import ZXGripperCommunicationError


class GripperStreamingTests(unittest.TestCase):
    def test_reversal_guard_rejects_brief_revisions_then_releases_on_sustained_request(self):
        guard = b.GripperReversalGuard(450, .25)
        guard.acknowledge(1000)
        guard.acknowledge(2000)
        for t in (0., .1, .2):
            self.assertFalse(guard.allows(1000, t))
        self.assertTrue(guard.allows(2100, .21))
        guard.acknowledge(2100)
        self.assertFalse(guard.allows(1000, .3))
        self.assertFalse(guard.allows(900, .54))
        self.assertTrue(guard.allows(800, .55))
        guard.acknowledge(800)
        self.assertTrue(guard.allows(700, .56))
        self.assertEqual(guard.direction, -1)

    def test_reversal_guard_endpoint_confirms_even_within_amplitude_deadband(self):
        guard = b.GripperReversalGuard(450, .25)
        guard.acknowledge(0)
        guard.acknowledge(30)
        self.assertFalse(guard.allows(0, 0.))
        self.assertTrue(guard.allows(0, .25))
        guard.reset()
        self.assertTrue(guard.allows(8000, .26))
        self.assertEqual(guard.direction, 0)

    def test_worker_filters_short_reversals_without_blocking_or_losing_full_open(self):
        device, clock, grip = self.fixture(cycles=60)
        def sample(_):
            target = 4500 if clock.now < .02 or .14 <= clock.now < .2 else 1000
            if clock.now >= .2:
                target = 0
            row = device.target[0].copy(); row[6] = target/9000
            device.target = (row, 1.)
            return SimpleNamespace(position_raw=1200)
        grip.read_motion_state.side_effect = sample
        self.run_worker(device, clock, target_deadband_raw=45,
                        reversal_deadband_raw=450, reversal_confirm_s=.25)
        self.assertIsNone(device.error)
        self.assertEqual([c.args[1] for c in grip.set_position_raw.call_args_list if c.args[1] != 0],
                         [2250, 4500])
        self.assertEqual(grip.set_position_raw.call_args.args[1], 0)
        self.assertAlmostEqual(clock.now, .6)
        self.assertEqual(grip.read_motion_state.call_count, 59)

    def test_expired_reversal_does_not_acquire_confirmation_from_stale_time(self):
        device, clock, grip = self.fixture(cycles=45)
        def sample(_):
            row = device.target[0].copy()
            row[6] = .5 if clock.now < .02 else 0.
            device.target = (row, .05)
            return SimpleNamespace(position_raw=1200)
        grip.read_motion_state.side_effect = sample
        self.run_worker(device, clock, target_deadband_raw=45,
                        reversal_deadband_raw=450, reversal_confirm_s=.25)
        self.assertIsNone(device.error)
        self.assertEqual([c.args[1] for c in grip.set_position_raw.call_args_list], [2250, 4500])

    def test_explicit_final_target_is_issued_within_its_freshness_window(self):
        device, clock, grip = self.fixture(cycles=3)
        def sample(_):
            row = device.target[0].copy()
            row[6] = .5 if clock.now < .02 else 0.
            device.target = (row, .1)
            device._gripper_target_final = clock.now >= .02
            return SimpleNamespace(position_raw=1200)
        grip.read_motion_state.side_effect = sample
        self.run_worker(device, clock, target_deadband_raw=45,
                        reversal_deadband_raw=450, reversal_confirm_s=.25)
        self.assertEqual([c.args[1] for c in grip.set_position_raw.call_args_list], [2250, 4500, 0])

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

    def run_worker(self, device, clock, **options):
        with patch.object(b.time, 'monotonic', side_effect=clock.monotonic):
            config = dict(device_id=1, speed=30, torque=60,
                          io_hz=100, feedback_mode='motion', compact_trigger=True)
            config.update(options)
            device._gripper_worker('left', config)

    def test_deadband_suppresses_chatter_but_accumulates_slow_motion(self):
        device, clock, grip = self.fixture()
        def sample(value):
            row = device.target[0].copy(); row[6] = value/9000
            device.target = (row, 1.)
            return SimpleNamespace(position_raw=value)
        values = iter((2277, 2304))
        grip.read_motion_state.side_effect = lambda _: sample(next(values))
        self.run_worker(device, clock, target_deadband_raw=45)
        self.assertEqual([c.args[1] for c in grip.set_position_raw.call_args_list], [2250, 2304])
        self.assertEqual(len(device.feedback), 1)
        self.assertAlmostEqual(clock.now, .03)

    def test_deadband_passes_large_reversals_and_exact_open_endpoint(self):
        device, clock, grip = self.fixture()
        device.target[0][6] = 30/9000
        def sample(value):
            row = device.target[0].copy(); row[6] = value/9000
            device.target = (row, 1.)
            return SimpleNamespace(position_raw=value)
        values = iter((0, 7000))
        grip.read_motion_state.side_effect = lambda _: sample(next(values))
        self.run_worker(device, clock, target_deadband_raw=45)
        self.assertEqual([c.args[1] for c in grip.set_position_raw.call_args_list], [30, 0, 7000])

    def test_closure_bias_clamps_preserves_release_and_fades_near_open(self):
        config = dict(closure_bias_raw=450, closure_bias_fade_raw=900)
        for model, expected in [(0, 0), (30, 45), (450, 675), (900, 1350),
                                (8000, 8450), (8800, 9000), (9000, 9000)]:
            with self.subTest(model=model):
                self.assertEqual(b.gripper_target_raw(model / 9000, config), (model, expected))
        self.assertEqual(b.gripper_target_raw(8000 / 9000, {}), (8000, 8000))

    def test_closure_bias_does_not_accumulate_or_replace_actual_feedback(self):
        device, clock, grip = self.fixture()
        device.target[0][6] = 8000 / 9000
        self.run_worker(device, clock, closure_bias_raw=450, closure_bias_fade_raw=900)
        self.assertIsNone(device.error)
        self.assertEqual([call.args[1] for call in grip.set_position_raw.call_args_list],
                         [8450, 8450, 8450])
        self.assertAlmostEqual(device.target[0][6], 8000 / 9000)
        self.assertEqual(device.feedback['left'][0], 1200)
        logged = [c.kwargs for c in device.log.call_args_list if c.args[0] == 'gripper_command']
        self.assertEqual(logged[0]['model_target_raw'], 8000)
        self.assertEqual(logged[0]['target_raw'], 8450)
        self.assertEqual(logged[0]['applied_bias_raw'], 450)

    def test_biased_target_deadband_still_allows_full_open(self):
        device, clock, grip = self.fixture()
        device.target[0][6] = 8000 / 9000

        def sample(value):
            row = device.target[0].copy()
            row[6] = value / 9000
            device.target = (row, 1.)
            return SimpleNamespace(position_raw=value)

        values = iter((8020, 0))
        grip.read_motion_state.side_effect = lambda _: sample(next(values))
        self.run_worker(device, clock, closure_bias_raw=450, target_deadband_raw=45)
        self.assertEqual([c.args[1] for c in grip.set_position_raw.call_args_list], [8450, 0])

    def test_right_gripper_uses_same_positive_closing_bias(self):
        device, clock, grip = self.fixture(cycles=1)
        device.grips = dict(right=grip)
        device.write_locks = dict(right=threading.Lock())
        device.target[0][6] = 0
        device.target[0][13] = 8000 / 9000
        with patch.object(b.time, 'monotonic', side_effect=clock.monotonic):
            device._gripper_worker('right', dict(device_id=1, speed=30, torque=60,
                                                  io_hz=100, closure_bias_raw=450))
        self.assertIsNone(device.error)
        self.assertEqual(grip.set_position_raw.call_args.args, (1, 8450))

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
                           ('compact_trigger', 'yes'), ('target_deadband_raw', -1),
                           ('target_deadband_raw', True), ('target_deadband_raw', 1.5),
                           ('reversal_deadband_raw', -1), ('reversal_deadband_raw', True),
                           ('reversal_confirm_s', -1), ('reversal_confirm_s', float('nan')),
                           ('closure_bias_raw', -1), ('closure_bias_raw', True),
                           ('closure_bias_raw', 9001), ('closure_bias_raw', 1.5),
                           ('closure_bias_fade_raw', 0), ('closure_bias_fade_raw', True)]:
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
