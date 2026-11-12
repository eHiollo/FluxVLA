"""Dual VR -> ZX: fake USB ownership and synthetic feedback only."""
import copy
import threading
import tempfile
import json
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from mmengine import Config
from fluxvla.engines.operators.vr.runtime import _ZXGripperChannel, build_runtime
from fluxvla.engines.operators.zx_gripper import ZXGripperState, ZXGripper, ZXGripperInputExpired, ZXGripperCRCError
from test_vr_dual_runtime import configuration, hand, HandInput, DualRobot
from test_zx_gripper import FakeSerial

ROOT = Path(__file__).resolve().parents[1]


def wait_for(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('synthetic worker condition timed out')
        threading.Event().wait(.005)


def options():
    return dict(target_rate_raw_s=1800., max_lead_raw=300., io_hz=20.,
                deadzone=.1, direction=dict(left=1, right=1), input_timeout_s=.25, feedback_timeout_s=.5)


def device(side='left'):
    return Config.fromfile(str(ROOT / 'configs/a10/a10_devices.py')).grippers[side].to_dict()


class SerialTimeoutConfigTest(unittest.TestCase):
    def test_capture_timeout_is_configurable_without_changing_motion_settings(self):
        channel = _ZXGripperChannel('left', device(), dict(options(), serial_timeout_s=.05))
        self.assertEqual(channel.client.timeout, .05)
        self.assertEqual(channel.speed, device()['speed'])
        self.assertEqual(channel.torque, device()['torque'])
        for bad in (False, 0, .001, .21, float('nan')):
            with self.subTest(timeout=bad), self.assertRaises(ValueError):
                _ZXGripperChannel('left', device(), dict(options(), serial_timeout_s=bad))


class FakeZX:
    def __init__(self, position=4500):
        self.position = position
        self.targets = []
        self.connected = False
        self.max_position_raw = 9000
        self.alarm = False
        self.alarm_b = 0
        self.enabled = True
        self.reset_count = self.disable_count = 0
        self.reset_succeeds = True
        self.block = threading.Event()
        self.block.set()
        self.reading = threading.Event()
        self.read_count = 0

    def __enter__(self):
        self.connected = True
        return self

    def __exit__(self, *args):
        self.connected = False

    def read_state(self, device_id):
        self.read_count += 1
        if self.read_count > 1:
            self.reading.set()
            self.block.wait(1.)
        return ZXGripperState(device_id, self.position, self.position / 9000, None,
                              0x15, int(self.alarm), self.alarm_b, self.enabled, 0, 0, time.monotonic(), 'idle')

    def reset_alarms(self, device_id):
        self.reset_count += 1
        if not self.reset_succeeds:
            raise RuntimeError('alarm remains active')
        self.alarm = False; self.alarm_b = 0

    def disable(self, device_id):
        self.disable_count += 1
        self.enabled = False

    def set_position_raw(self, device_id, target, **kwargs):
        if kwargs['motion_cancel_event'].is_set() or time.monotonic() >= kwargs['motion_deadline_s']:
            raise ZXGripperInputExpired('expired fake input')
        self.targets.append(target)
        self.position = target


class ChannelTest(unittest.TestCase):
    def channel(self, *, side='left', opts=None):
        channel = _ZXGripperChannel(side, device(side), opts or options())
        self.addCleanup(channel.close)
        return channel

    def test_motor_speed_accepts_1_to_15_without_opening_usb(self):
        with patch('serial.Serial') as serial:
            for speed in (1, 5, 10, 15):
                with self.subTest(speed=speed):
                    settings = device(); settings['speed'] = speed
                    c = _ZXGripperChannel('left', settings, options())
                    self.assertEqual(c.speed, speed)
                    c.close()
            serial.assert_not_called()

    def test_motor_speed_rejects_out_of_range_and_non_integer_values(self):
        with patch('serial.Serial') as serial:
            for speed in (0, 16, 100, 1.5, True, '10'):
                with self.subTest(speed=speed):
                    settings = device(); settings['speed'] = speed
                    with self.assertRaisesRegex(ValueError, 'speed within 1..15%'):
                        _ZXGripperChannel('left', settings, options())
            settings = device(); settings.update(speed=15, torque=61)
            with self.assertRaises(ValueError):
                _ZXGripperChannel('left', settings, options())
            serial.assert_not_called()

    def test_worker_forwards_configured_15_percent_speed(self):
        settings = device(); settings['speed'] = 15
        c = _ZXGripperChannel('left', settings, options())
        self.addCleanup(c.close)
        fake = FakeZX(); c.client = fake
        with patch.object(fake, 'set_position_raw', wraps=fake.set_position_raw) as send:
            c.connect()
            c.submit(0., time.monotonic() + .5)
            wait_for(lambda: c.snapshot()['phase'] == 'centered')
            c.submit(-1., time.monotonic() + .5)
            wait_for(lambda: len(fake.targets) > 0)
            c.close()
            self.assertTrue(send.call_args_list)
            for call in send.call_args_list:
                self.assertEqual(call.kwargs['speed'], 15)
                self.assertEqual(call.kwargs['torque'], 60)

    def test_neutral_gate_open_close_direction_and_lead_limit(self):
        c = self.channel()
        self.assertEqual(c._choose_target(4500, 1., 1., 5.)[1], 'neutral_required')
        self.assertIsNone(c._choose_target(4500, 1.05, 0., 5.)[0])
        self.assertIsNone(c._choose_target(4500, 1.10, 1., 5.)[0])
        self.assertEqual(c._choose_target(4500, 1.15, 1., 5.)[0], 4410)
        for i in range(10):
            value, _ = c._choose_target(4500, 1.2 + .05 * i, 1., 5.)
        self.assertEqual(value, 4200)  # no unbounded error accumulation
        c._choose_target(4500, 2., 0., 5.)
        c._choose_target(4500, 2.05, -1., 5.)
        self.assertEqual(c._choose_target(4500, 2.1, -1., 5.)[0], 4590)

    def test_real_serial_interval_is_integrated_instead_of_fixed_100ms(self):
        opts = options(); opts['max_lead_raw'] = 600.
        c = self.channel(opts=opts)
        c._choose_target(4500, 1., 0., 3.)
        c._choose_target(4500, 1.05, -1., 3.)
        target, _ = c._choose_target(4500, 1.23, -1., 3.)
        self.assertEqual(target, 4824)  # 1800 raw/s * 0.18s, not 0.10s
        target, _ = c._choose_target(4500, 1.5, -1., 3.)
        self.assertEqual(target, 5100)  # bounded lead remains in force

    def test_center_and_stale_input_reset_target_and_require_new_center(self):
        c = self.channel()
        c._choose_target(4500, 1., 0., 2.)
        c._choose_target(4500, 1.05, -1., 2.)
        self.assertIsNone(c._choose_target(4500, 2., -1., 2.)[0])
        self.assertEqual(c._choose_target(4500, 2.05, -1., 3.)[1], 'neutral_required')
        self.assertEqual(c._choose_target(4500, 2.1, 0., 3.)[1], 'centered')

    def test_bounds_and_per_hand_reverse_direction(self):
        opts = options(); opts['direction']['right'] = -1
        c = self.channel(side='right', opts=opts)
        c._choose_target(8990, 1., 0., 3.)
        c._choose_target(8990, 1.05, 1., 3.)
        self.assertEqual(c._choose_target(8990, 1.1, 1., 3.)[0], 9000)
        c._choose_target(0, 1.15, 0., 3.)
        c._choose_target(0, 1.2, -1., 3.)
        self.assertEqual(c._choose_target(0, 1.25, -1., 3.)[0], 0)

    def test_worker_connect_and_center_never_move_then_deadline_stops_updates(self):
        c = self.channel(); fake = FakeZX(); c.client = fake
        c.connect()
        self.assertEqual(fake.targets, [])
        c.submit(0., time.monotonic() + .3)
        wait_for(lambda: c.snapshot()['phase'] == 'centered')
        self.assertEqual(fake.targets, [])
        c.submit(-1., time.monotonic() + .3)
        wait_for(lambda: len(fake.targets) > 0)
        self.assertTrue(all(4500 <= target <= 4800 for target in fake.targets))
        wait_for(lambda: c.snapshot()['phase'] == 'input_stale')
        count = len(fake.targets)
        c.submit(-1., time.monotonic() + .2)
        wait_for(lambda: c.snapshot()['phase'] == 'neutral_required')
        self.assertEqual(len(fake.targets), count)
        c.close()
        self.assertFalse(fake.connected)

    def test_pause_cancels_old_targets_and_requires_fresh_neutral_before_usb_can_resume(self):
        c = self.channel()
        c.submit(0., time.monotonic()+1.)
        c.pause()
        self.assertTrue(c._motion_cancel.is_set())
        self.assertEqual(c._input, (0., 0.))
        self.assertTrue(c._need_center)
        c.submit(-1., time.monotonic()+1.)
        self.assertTrue(c._motion_cancel.is_set())
        c.submit(0., 0.)  # Synthetic zero while paused is not a neutral observation.
        self.assertTrue(c._motion_cancel.is_set())
        c.submit(0., time.monotonic()+1.)
        self.assertFalse(c._motion_cancel.is_set())
        c.request_stop()
        c.submit(0., time.monotonic()+1.)
        self.assertTrue(c._motion_cancel.is_set())

    def test_slow_serial_does_not_block_submit_or_snapshot(self):
        c = self.channel(); fake = FakeZX(); c.client = fake
        fake.block.clear()
        c.connect()
        self.assertTrue(fake.reading.wait(1.))
        before = time.monotonic()
        c.submit(-1., time.monotonic() + .25)
        c.snapshot()
        self.assertLess(time.monotonic() - before, .05)
        c.request_stop()
        fake.block.set()
        c.close()
        self.assertEqual(fake.targets, [])

    def test_write_crc_waits_for_fresh_center_and_never_replays_held_target(self):
        c = self.channel(); fake = FakeZX(); c.client = fake
        c.connect()
        c.submit(0., time.monotonic() + 1.)
        wait_for(lambda: c.snapshot()['phase'] == 'centered')
        attempts = []
        real_send = fake.set_position_raw
        def crc_once(device_id, target, **kwargs):
            attempts.append(target)
            if len(attempts) == 1:
                raise ZXGripperCRCError(1, 6, 0x0108, b'badtx', b'badrx')
            return real_send(device_id, target, **kwargs)
        with patch.object(fake, 'set_position_raw', crc_once):
            c.submit(-1., time.monotonic() + 1.)
            wait_for(lambda: bool(attempts))
            wait_for(lambda: c.snapshot().get('last_io_error') is not None)
            c.submit(-1., time.monotonic() + 1.)
            wait_for(lambda: c.snapshot()['phase'] == 'neutral_required')
            c.check_health()
            for _ in range(4):
                c.submit(-1., time.monotonic() + 1.)
                time.sleep(.06)
            self.assertEqual(len(attempts), 1)
            self.assertIsNone(c.snapshot()['error'])
            self.assertIn('CRC', c.snapshot()['last_io_error'])
            c.submit(0., time.monotonic() + 1.)
            wait_for(lambda: c.snapshot()['phase'] == 'centered')
            c.submit(-1., time.monotonic() + 1.)
            wait_for(lambda: len(fake.targets) > 0)
        c.close()

    def test_driver_recovers_read_crc_without_killing_background_channel(self):
        c = self.channel()
        fake = FakeSerial(); fake.alarms = (0, 0)
        c.client.expected_usb_serial = None
        write = fake.write
        damaged = [False]
        def once(request):
            result = write(request)
            if not damaged[0] and request[1] == 3:
                damaged[0] = True
                fake.rx = fake.rx[:-1] + bytes([fake.rx[-1] ^ 1])
            return result
        with patch('serial.Serial', return_value=fake), patch.object(fake, 'write', once):
            c.connect()
            self.assertTrue(damaged[0])
            self.assertEqual(c.snapshot()['crc_errors'], 1)
            self.assertEqual(c.snapshot()['crc_read_retries'], 1)
            c.check_health()
            self.assertTrue(all(p[1] == 3 for p in fake.writes))
            c.close()

    def test_alarm_worker_failure_reports_error_without_opening_gripper(self):
        c = self.channel(); fake = FakeZX(); fake.alarm = True; c.client = fake
        c.connect()
        self.assertFalse(c.check_health())
        self.assertTrue(c.snapshot()['alarm_latched'])
        c.close()
        self.assertEqual(fake.targets, [])
        self.assertFalse(fake.connected)

    def test_alarm_readback_is_published_worker_stays_read_only_until_explicit_reset(self):
        c = self.channel(); fake = FakeZX(); c.client = fake; c.connect()
        fake.alarm_b = 0x1000
        wait_for(lambda: c.snapshot().get('alarm_latched'))
        self.assertEqual(c.snapshot()['alarm_b'], 0x1000)
        self.assertTrue(c._thread.is_alive())
        self.assertFalse(c.check_health())
        c.submit(-1., time.monotonic()+1.)
        time.sleep(.12)
        self.assertEqual(fake.targets, [])
        self.assertEqual(fake.reset_count, 0)
        c.request_alarm_action('disable')
        wait_for(lambda: not fake.enabled)
        self.assertEqual(fake.disable_count, 1)
        c.request_alarm_action('reset')
        wait_for(lambda: c.snapshot().get('alarm_latched') is False)
        self.assertEqual(fake.reset_count, 1)
        self.assertEqual(c.snapshot()['alarm_b'], 0)
        c.submit(-1., time.monotonic()+1.)
        time.sleep(.12)
        self.assertEqual(fake.targets, [])  # Holding the old axis never resumes writes.
        c.submit(0., time.monotonic()+1.)
        wait_for(lambda: c.snapshot()['phase'] == 'centered')
        c.submit(-1., time.monotonic()+1.)
        wait_for(lambda: bool(fake.targets))

    def test_failed_manual_reset_keeps_fault_latched_and_never_retries_it(self):
        c = self.channel(); fake = FakeZX(); c.client = fake; c.connect()
        fake.alarm_b = 0x1000; fake.reset_succeeds = False
        wait_for(lambda: c.snapshot().get('alarm_latched'))
        c.request_alarm_action('reset')
        wait_for(lambda: bool(c.snapshot().get('reset_error')))
        time.sleep(.12)
        self.assertEqual(fake.reset_count, 1)
        self.assertTrue(c.snapshot()['alarm_latched'])
        self.assertEqual(fake.targets, [])


class DualZXRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.logs = tempfile.TemporaryDirectory()
        self.addCleanup(self.logs.cleanup)

    def test_live_defaults_and_input_only_preview_never_open_usb(self):
        cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py')).teleop
        self.assertTrue(cfg.zx_grippers.enabled)
        self.assertFalse(cfg.vr_only)
        self.assertFalse(cfg.observe_only)
        self.assertTrue(cfg.record_controller)
        self.assertTrue(cfg.operator.allow_motion)
        self.assertEqual(cfg.zx_grippers.target_rate_raw_s, 3000.)
        self.assertEqual(cfg.zx_grippers.max_lead_raw, 600.)
        for side in ('left', 'right'):
            self.assertEqual(cfg.zx_grippers.devices[side].speed, 15)
            self.assertEqual(device(side)['speed'], 5)
        cfg.vr_only = True; cfg.record_controller = False
        with patch('serial.Serial') as serial:
            rt = build_runtime(cfg)
            self.assertTrue(rt.zx_grippers_enabled)
            self.assertEqual(rt._zx_channels, {})
            rt.close()
            cfg.vr_only = False; cfg.operator.allow_motion = True
            rt = build_runtime(cfg)
            self.assertEqual(set(rt._zx_channels), {'left', 'right'})
            self.assertEqual(rt._zx_channels['left'].client.expected_usb_serial, 'DU0EMMMM')
            rt.close()
            serial.assert_not_called()

    def test_short_default_command_preview_has_recording_and_grippers_but_no_io(self):
        from scripts.teleoperate import main
        import contextlib
        import io
        output = io.StringIO()
        with patch('socket.socket', side_effect=AssertionError('network IO')), \
                patch('serial.Serial', side_effect=AssertionError('USB IO')), \
                patch('subprocess.Popen', side_effect=AssertionError('service startup')), \
                contextlib.redirect_stdout(output):
            self.assertEqual(main(['--config', str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py'), '--preview']), 0)
        self.assertIn('vr_only=False', output.getvalue())
        self.assertIn('record_controller=True', output.getvalue())
        self.assertIn('zx_grippers=True', output.getvalue())

    def test_input_only_and_observe_overrides_do_not_inherit_recording(self):
        from scripts.teleoperate import main
        import contextlib
        import io
        for args in (['--cfg-options', 'teleop.vr_only=True'], ['--observe']):
            output = io.StringIO()
            with patch('socket.socket', side_effect=AssertionError('network IO')), \
                    patch('serial.Serial', side_effect=AssertionError('USB IO')), \
                    contextlib.redirect_stdout(output):
                self.assertEqual(main(['--config', str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py'),
                                       '--preview', *args]), 0)
            self.assertIn('record_controller=False', output.getvalue())

    def test_opt_in_observe_does_not_construct_active_channels(self):
        cfg = configuration(); cfg.zx_grippers.enabled = True
        cfg.vr_only = False; cfg.observe_only = True
        with patch('serial.Serial') as serial:
            rt = build_runtime(cfg)
            self.assertEqual(rt._zx_channels, {})
            rt.close()
            serial.assert_not_called()

    def test_mapping_uses_physical_hands_even_with_reversed_arm_names(self):
        cfg = configuration(); vr = build_runtime(cfg).teleoperator; vr.input = HandInput()
        vr.input.rows = {side: hand(1) for side in ('left', 'right')}
        vr.input.rows['left']['xlevr.thumbstick'] = {'x': .75}
        vr.input.rows['right']['xlevr.thumbstick'] = {'x': -.4}
        action = vr.get_action()
        self.assertFalse(action['left']['ee.enabled'])
        self.assertFalse(action['right']['ee.enabled'])
        self.assertEqual(action['right']['gripper.pos'], .75)  # left physical hand -> operator right arm
        self.assertEqual(action['left']['gripper.pos'], -.4)

    def test_missing_endpoints_duplicate_ports_and_invalid_options_rejected(self):
        for change in ('endpoint', 'port', 'rate', 'direction'):
            cfg = configuration(); cfg.zx_grippers.enabled = True
            if change == 'endpoint': cfg.zx_grippers.devices.left.check.open_raw = None
            if change == 'port': cfg.zx_grippers.devices.left.port = cfg.zx_grippers.devices.right.port
            if change == 'rate': cfg.zx_grippers.io_hz = 100
            if change == 'direction': cfg.zx_grippers.direction.left = 0
            with self.subTest(change=change), self.assertRaises(ValueError):
                build_runtime(cfg)

    def live(self):
        cfg = configuration(); cfg.zx_grippers.enabled = True
        cfg.vr_only = False; cfg.operator.allow_motion = True
        cfg.diagnostics_path = str(Path(self.logs.name) / 'teleop.jsonl')
        cfg.controller_speed_percent = None
        rt = build_runtime(cfg)
        rt.robot = DualRobot(); rt.cartesian_adapter = 'legacy_anchor'
        rt.teleoperator.input = HandInput(); rt.vr = rt.teleoperator.input
        for channel in rt._zx_channels.values(): channel.client = FakeZX()
        self.addCleanup(rt.close)
        rt.connect()
        return rt

    def test_live_joysticks_publish_without_clutch_and_without_arm_follower(self):
        rt = self.live()
        rows = {side: hand(1) for side in ('left', 'right')}
        rt.vr.rows = rows
        rt.step()
        wait_for(lambda: all(c.snapshot()['phase'] == 'centered' for c in rt._zx_channels.values()))
        rows['left']['xlevr.thumbstick'] = {'x': -1.}
        for row in rows.values(): row['xlevr.sample_receive_time_s'] = time.monotonic()
        rt.step()
        wait_for(lambda: len(rt._zx_channels['left'].client.targets) > 0)
        self.assertEqual(rt._zx_channels['right'].client.targets, [])
        self.assertNotIn('start', rt.robot.events)
        rt.close()
        self.assertTrue(all(not c.client.connected for c in rt._zx_channels.values()))
        records = [json.loads(line) for line in (Path(self.logs.name) / 'teleop.jsonl').read_text().splitlines()]
        self.assertTrue(any(set(r.get('gripper_command') or {}) == {'left', 'right'} for r in records))

    def test_startup_gripper_alarm_keeps_ui_and_read_only_polling_without_starting_arms(self):
        cfg = configuration(); cfg.zx_grippers.enabled = True
        cfg.vr_only = False; cfg.operator.allow_motion = True
        cfg.diagnostics_path = str(Path(self.logs.name) / 'teleop.jsonl')
        cfg.controller_speed_percent = None
        rt = build_runtime(cfg); rt.robot = DualRobot(); rt.cartesian_adapter = 'legacy_anchor'
        rt.teleoperator.input = HandInput(); rt.vr = rt.teleoperator.input
        for channel in rt._zx_channels.values(): channel.client = FakeZX()
        rt._zx_channels['right'].client.alarm = True
        rt.connect()
        self.addCleanup(rt.close)
        self.assertTrue(rt._connected)
        rt.vr.rows = {side: hand(1) for side in ('left', 'right')}
        rt.step()
        self.assertIsNone(rt.input_pause_status)
        self.assertNotIn('start', rt.robot.events)
        self.assertTrue(all(not c.client.targets for c in rt._zx_channels.values()))
        for seq in range(2, 7):
            rt.vr.rows = {side: hand(seq, pressed=True) for side in ('left', 'right')}
            rt.step()
        self.assertIn('start', rt.robot.events)  # An explicit arm request is independent of the accessory alarm.
        rt.close()
        self.assertTrue(all(not c.client.connected for c in rt._zx_channels.values()))
        self.assertIn('close', rt.robot.events)

    def test_gripper_alarm_only_locks_bad_gripper_arms_and_other_gripper_continue(self):
        rt = self.live()
        rt.vr.rows = {side: hand(1) for side in ('left', 'right')}
        rt.step()
        for seq in range(2, 9):
            rt.vr.rows = {side: hand(seq, pressed=True) for side in ('left', 'right')}
            rt.step()
            if 'start' in rt.robot.events:
                break
        self.assertIn('start', rt.robot.events)
        rt._zx_channels['left'].client.alarm = True
        wait_for(lambda: rt._zx_channels['left'].snapshot()['error'] is not None)
        rt.step()
        self.assertNotIn('stop', rt.robot.events)
        self.assertIsNone(rt.input_pause_status)
        self.assertTrue(all(c._thread.is_alive() for c in rt._zx_channels.values()))
        starts = rt.robot.events.count('start')
        seq = 20
        def tick(pressed):
            nonlocal seq
            seq += 1
            rt.vr.rows = {side: hand(seq, pressed=pressed) for side in ('left', 'right')}
            if pressed:
                rt.vr.rows['right']['xlevr.thumbstick'] = {'x': -1.}
            rt.step()
        count = len(rt.robot.targets)
        healthy_targets = len(rt._zx_channels['right'].client.targets)
        failed_targets = len(rt._zx_channels['left'].client.targets)
        for _ in range(3): tick(True); time.sleep(.06)
        self.assertGreater(len(rt.robot.targets), count)
        self.assertGreater(len(rt._zx_channels['right'].client.targets), healthy_targets)
        self.assertEqual(len(rt._zx_channels['left'].client.targets), failed_targets)
        self.assertTrue(rt.robot.follower_stop_pending)
        self.assertEqual(rt.robot.events.count('start'), starts)
        channel = rt._zx_channels['left']
        old_targets = len(channel.client.targets)
        tick(False)
        rt.prepare_gripper_maintenance()
        self.assertIn('wait_stopped', rt.robot.events)
        channel.request_alarm_action('reset')  # Explicit test operator action, same USB owner.
        wait_for(lambda: channel.snapshot().get('alarm_latched') is False)
        self.assertEqual(len(channel.client.targets), old_targets)
        tick(False)  # Explicit maintenance stop requires release, without a global fault latch.
        for _ in range(4): tick(True)
        self.assertEqual(rt.robot.events.count('start'), starts+1)

    def test_unexpected_gripper_worker_failure_locks_targets_but_keeps_ui_alive(self):
        rt = self.live()
        channel = rt._zx_channels['left']
        with patch.object(channel.client, 'read_state', side_effect=RuntimeError('synthetic USB worker failure')):
            wait_for(lambda: bool(channel.snapshot()['error']))
        rt.vr.rows = {side: hand(1) for side in ('left', 'right')}
        rt.step()
        self.assertTrue(rt._connected)
        self.assertIsNone(rt.input_pause_status)
        with self.assertRaisesRegex(RuntimeError, 'worker is not running'):
            channel.request_alarm_action('reset')

    def test_communication_error_only_pauses_one_gripper_and_vr_keeps_running(self):
        rt = self.live()
        seq = [0]
        def tick(right_axis=-1.):
            seq[0] += 1
            rt.vr.rows = {side: hand(seq[0], pressed=True) for side in ('left', 'right')}
            rt.vr.rows['right']['xlevr.thumbstick'] = {'x': right_axis}
            rt.vr.rows['left']['xlevr.thumbstick'] = {'x': -1.}
            rt.step()
        rt.vr.rows = {side: hand(1) for side in ('left', 'right')}
        rt.step()
        wait_for(lambda: all(c.snapshot()['phase'] == 'centered' for c in rt._zx_channels.values()))
        for _ in range(6): tick(); time.sleep(.025)
        self.assertIn('start', rt.robot.events)
        right = rt._zx_channels['right']
        left_count = len(rt._zx_channels['left'].client.targets)
        old_count = len(right.client.targets)
        with patch.object(right.client, 'read_state', side_effect=ZXGripperCRCError(1, 3, 0x0418, b'tx', b'rx')):
            wait_for(lambda: bool(right.snapshot()['communication_error']))
            for _ in range(8): tick(); time.sleep(.04)
            self.assertTrue(rt._connected)
            self.assertTrue(rt.robot.follower_stop_pending)
            self.assertNotIn('stop', rt.robot.events)
            self.assertEqual(len(right.client.targets), old_count)
            self.assertGreater(len(rt._zx_channels['left'].client.targets), left_count)
            self.assertFalse(right.check_health())
        wait_for(lambda: right.snapshot()['communication_error'] is None)
        for _ in range(3): tick(); time.sleep(.04)
        self.assertEqual(len(right.client.targets), old_count)  # No replay while held.
        tick(right_axis=0.)
        wait_for(lambda: right.snapshot()['phase'] == 'centered')
        for _ in range(4): tick(); time.sleep(.04)
        self.assertGreater(len(right.client.targets), old_count)

    def test_gripper_failure_and_arm_fault_cancel_both_channels(self):
        for origin in ('gripper', 'arm'):
            rt = self.live()
            rt.vr.rows = {side: hand(1) for side in ('left', 'right')}
            if origin == 'gripper':
                rt._zx_channels['left'].client.alarm = True
                wait_for(lambda: rt._zx_channels['left'].snapshot()['error'] is not None)
            else:
                rt.robot.frame_fault = True
            if origin == 'gripper':
                rt.step()
                self.assertIsNone(rt.input_pause_status)
                self.assertTrue(all(not c._stop.is_set() for c in rt._zx_channels.values()))
            else:
                with self.assertRaises(RuntimeError): rt.step()
                self.assertTrue(all(c._stop.is_set() for c in rt._zx_channels.values()))
            rt.close()


class DriverExpiryTests(unittest.TestCase):
    def test_wrong_usb_identity_blocks_open(self):
        client = ZXGripper('/dev/no-such-zx-device', expected_usb_serial='expected', expected_usb_interface='00')
        with patch('serial.Serial') as serial, self.assertRaisesRegex(RuntimeError, 'identity'):
            client.connect()
        serial.assert_not_called()

    def test_expired_input_before_alarm_or_after_alarm_never_writes_motion(self):
        for after_alarm in (False, True):
            fake = FakeSerial(); fake.alarms = (0, 0)
            client = ZXGripper('/fake/zx', max_position_raw=9000)
            stop = threading.Event()
            original_read = client.read_registers
            def read_then_cancel(*args):
                result = original_read(*args)
                stop.set()
                return result
            with patch('serial.Serial', return_value=fake):
                client.connect()
                if after_alarm:
                    client.read_registers = read_then_cancel
                else:
                    stop.set()
                with self.assertRaises(ZXGripperInputExpired):
                    client.set_position_raw(1, 5000, motion_cancel_event=stop)
                client.disconnect()
            self.assertTrue(all(packet[1] == 3 for packet in fake.writes))

    def test_deadline_expiring_during_alarm_read_prevents_motion_writes(self):
        client = ZXGripper('/fake/zx', max_position_raw=9000)
        with patch('time.monotonic', side_effect=[10., 11.]), \
                patch.object(client, 'read_registers', return_value=(0, 0)) as read, \
                patch.object(client, 'write_register') as write, \
                patch.object(client, 'write_registers') as writes:
            with self.assertRaises(ZXGripperInputExpired):
                client.set_position_raw(1, 5000, motion_deadline_s=10.5)
        self.assertEqual(read.call_count, 1)
        write.assert_not_called(); writes.assert_not_called()

    def test_cancel_after_parameter_write_prevents_enable_and_trigger(self):
        fake = FakeSerial(); fake.alarms = (0, 0); stop = threading.Event()
        original_write = fake.write
        def write_then_cancel(packet):
            result = original_write(packet)
            if packet[1] == 16: stop.set()
            return result
        fake.write = write_then_cancel
        client = ZXGripper('/fake/zx', max_position_raw=9000)
        with patch('serial.Serial', return_value=fake):
            client.connect()
            with self.assertRaises(ZXGripperInputExpired):
                client.set_position_raw(1, 5000, motion_cancel_event=stop)
            client.disconnect()
        self.assertFalse(any(p[1:6] == bytes.fromhex('06 01 08 00 01') for p in fake.writes))
        self.assertFalse(any(p[1:4] == bytes.fromhex('06 01 00') for p in fake.writes))


if __name__ == '__main__':
    unittest.main()
