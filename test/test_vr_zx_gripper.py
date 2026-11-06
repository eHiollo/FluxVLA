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
from fluxvla.engines.operators.zx_gripper import ZXGripperState, ZXGripper, ZXGripperInputExpired
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


class FakeZX:
    def __init__(self, position=4500):
        self.position = position
        self.targets = []
        self.connected = False
        self.max_position_raw = 9000
        self.alarm = False
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
                              0x15, int(self.alarm), 0, True, 0, 0, time.monotonic(), 'idle')

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

    def test_alarm_worker_failure_reports_error_without_opening_gripper(self):
        c = self.channel(); fake = FakeZX(); fake.alarm = True; c.client = fake
        with self.assertRaisesRegex(RuntimeError, 'alarm'):
            c.connect()
        c.close()
        self.assertEqual(fake.targets, [])
        self.assertFalse(fake.connected)


class DualZXRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.logs = tempfile.TemporaryDirectory()
        self.addCleanup(self.logs.cleanup)

    def test_config_defaults_and_opt_in_preview_never_open_usb(self):
        cfg = configuration()
        self.assertFalse(cfg.zx_grippers.enabled)
        cfg.zx_grippers.enabled = True
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

    def test_partial_gripper_startup_failure_closes_every_open_channel(self):
        cfg = configuration(); cfg.zx_grippers.enabled = True
        cfg.vr_only = False; cfg.operator.allow_motion = True
        cfg.diagnostics_path = str(Path(self.logs.name) / 'teleop.jsonl')
        rt = build_runtime(cfg); rt.robot = DualRobot()
        rt.teleoperator.input = HandInput(); rt.vr = rt.teleoperator.input
        for channel in rt._zx_channels.values(): channel.client = FakeZX()
        rt._zx_channels['right'].client.alarm = True
        with self.assertRaisesRegex(RuntimeError, 'alarm'):
            rt.connect()
        self.assertTrue(all(not c.client.connected for c in rt._zx_channels.values()))
        self.assertIn('close', rt.robot.events)

    def test_active_follower_is_stopped_when_a_gripper_fails(self):
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
        with self.assertRaises(RuntimeError): rt.step()
        self.assertIn('stop', rt.robot.events)
        self.assertIn('wait_stopped', rt.robot.events)
        self.assertTrue(all(c._stop.is_set() for c in rt._zx_channels.values()))

    def test_gripper_failure_and_arm_fault_cancel_both_channels(self):
        for origin in ('gripper', 'arm'):
            rt = self.live()
            rt.vr.rows = {side: hand(1) for side in ('left', 'right')}
            if origin == 'gripper':
                rt._zx_channels['left'].client.alarm = True
                wait_for(lambda: rt._zx_channels['left'].snapshot()['error'] is not None)
            else:
                rt.robot.frame_fault = True
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
