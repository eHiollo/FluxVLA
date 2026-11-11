"""Dual VR tests: synthetic feedback and local loopback only, never real devices."""
import copy
import json
import struct
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from mmengine import Config

from fluxvla.engines.operators.vr_operator import VROperator
from fluxvla.engines.operators.a10_operator import SDKToolCumulativeAdapter
from fluxvla.engines.operators.vr.control import VRPoseController, ReferenceConfig, follower_relative
from fluxvla.engines.operators.vr.monitor import VRMonitorBridge
from fluxvla.engines.operators.vr.input import VRInput
from fluxvla.engines.operators.vr.runtime import build_runtime, DualVRTeleopRuntime, inspect_controller
from test_a10_operator import FakeController, DUAL_ARMS, dual_motion
from test_vr_reference_runtime import Clock

ROOT = Path(__file__).resolve().parents[1]
IDENTITY = np.array([0., 0., 0., 0., 0., 0., 1.])


def configuration(*, identity_bindings=False):
    cfg = Config.fromfile(str(ROOT / 'configs/a10/a10_dual_arm_vr_teleop.py')).teleop
    if identity_bindings:
        # Lifecycle fixtures name arms after their hands; site mapping is tested separately.
        cfg.input.hand_to_arm = dict(left='left', right='right')
        cfg.input.mapping_by_hand = {}
    return cfg


def hand(seq, pressed=False, x=0., angle=0., age=0., trigger=0.):
    return {'xlevr.enabled': pressed, 'xlevr.grip_active': pressed,
            'xlevr.buttons': {'squeeze': pressed}, 'xlevr.trigger': trigger,
            'xlevr.target_position': [0., x, 0.],
            'xlevr.orientation_quat': Rotation.from_euler('z', angle).as_quat(),
            'xlevr.sample_sequence': seq, 'xlevr.sample_receive_time_s': time.monotonic(),
            'xlevr.verification_tracking': True,
            'xlevr.sample_age_s': age, 'xlevr.thumbstick': {}, 'xlevr.reset_arm': False}


class HandInput:
    def __init__(self):
        self.is_connected = False
        self.rows = None
        self.hud = []

    def send_hud(self, payload):
        self.hud.append(copy.deepcopy(payload))

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_hand_actions(self):
        return copy.deepcopy(self.rows)

    def get_vr_events(self):
        return {}


class DualRobot:
    """Explicit perfect synthetic follower; not evidence of physical response."""
    allow_motion = True
    is_multi_arm = True
    command_mode = 'cartesian'
    feedback_position_unit = 'mm'
    arm_names = ('left', 'right')
    follower_stop_pending = False

    def __init__(self):
        self.events, self.targets = [], []
        self.poses = {'left': np.r_[[.1, .2, .3], Rotation.from_euler('z', .4).as_quat()],
                      'right': np.r_[[.4, -.2, .3], Rotation.from_euler('x', -.3).as_quat()]}
        self.anchors = None
        self.frame_fault = False
        self.missing = False
        self.stale_arm = False
        self.follow = True
        self.stop_timeout = False

    def connect(self):
        self.events.append('connect')

    def get_frame(self):
        stamp = time.monotonic()
        arms = {name: dict(tcp_pq_m=value.copy(), received_monotonic_s=stamp)
                for name, value in self.poses.items()}
        if self.missing:
            arms.pop('right')
        if self.stale_arm:
            arms['right']['received_monotonic_s'] -= 1.
        return dict(arms=arms, has_error=self.frame_fault, received_monotonic_s=stamp,
                    follower_active=self.follower_stop_pending,
                    follower_mode='Cart' if self.follower_stop_pending else None)

    def set_tool_coordinate(self):
        self.events.append('tool')

    def start_follower(self, mode):
        self.events.append('start')
        self.follower_stop_pending = True
        self.anchors = copy.deepcopy(self.poses)

    def send_eepose(self, targets):
        assert set(targets) == set(self.arm_names)
        self.targets.append(copy.deepcopy(targets))
        if self.follow:
            for name, relative in targets.items():
                # Convert the relative quaternion directly, not via Euler shortcuts.
                anchor = self.anchors[name]
                rotation = Rotation.from_quat(anchor[3:])
                self.poses[name] = np.r_[anchor[:3] + rotation.apply(relative[:3]),
                                        (rotation * Rotation.from_quat(relative[3:])).as_quat()]

    def stop_follower(self):
        self.events.append('stop')
        self.follower_stop_pending = False

    def wait_stopped(self):
        self.events.append('wait_stopped')
        if self.stop_timeout:
            raise TimeoutError('synthetic stop unconfirmed')

    def close(self):
        self.events.append('close')


class DualSDKRobot(DualRobot):
    """Ideal pose feedback for lifecycle tests; real codec is tested on loopback."""
    cartesian_wire_format = 'pe321'

    def __init__(self):
        super().__init__()
        self.adapters = None
        self.sdk_packets = []
        self.fail_send = self.fail_begin = False

    def begin_sdk_tool_trajectory(self, poses):
        self.events.append('begin_sdk')
        if self.fail_begin:
            raise RuntimeError('synthetic adapter initialization failed')
        self.adapters = {name: SDKToolCumulativeAdapter(value) for name, value in poses.items()}

    def send_sdk_tool_pose(self, targets, **limits):
        if self.fail_send:
            raise OSError('synthetic UDP failure')
        candidates = {name: self.adapters[name].propose(target, **limits)
                      for name, target in targets.items()}
        for name, (target, cumulative) in candidates.items():
            self.adapters[name].commit(target, cumulative)
        self.sdk_packets.append({name: pe.copy() for name, (_, pe) in candidates.items()})
        self.targets.append({name: follower_relative(self.anchors[name], target)
                             for name, target in targets.items()})
        if self.follow:
            self.poses = copy.deepcopy(targets)
        return dict(arms={name: dict(planned_pose_m=target.tolist(), cumulative_pe321=pe.tolist())
                          for name, (target, pe) in candidates.items()},
                    local_send_success=True, execution_confirmed=False)


class DualInputTest(unittest.TestCase):
    def test_site_profile_uses_shared_translation_and_rotation_for_both_hands(self):
        vr = build_runtime(configuration()).teleoperator
        vr.input = HandInput()
        clock = Clock()
        clock.value = 100.
        # Expected tool-local translations with identical hand poses; not site calibration.
        gestures = (
            ([0., 0., -.003], [0., 0., .0027], [0., 0., .0027]),  # forward
            ([0., .003, 0.], [.0027, 0., 0.], [.0027, 0., 0.]),  # up
            ([.003, 0., 0.], [0., .0027, 0.], [0., .0027, 0.]),  # sideways
        )
        with patch('time.monotonic', clock.now):
            for displacement, expected_left, expected_right in gestures:
                with self.subTest(displacement=displacement):
                    vr.reset_reference()
                    vr.input.rows = {side: hand(1, True) for side in ('left', 'right')}
                    vr.get_action()
                    clock.advance()
                    vr.input.rows = {side: hand(2, True) for side in ('left', 'right')}
                    for raw in vr.input.rows.values():
                        raw['xlevr.target_position'] = displacement
                        raw['xlevr.orientation_quat'] = Rotation.from_rotvec([.01, .02, -.03]).as_quat()
                    actions = vr.get_action()
                    left = actions[vr.hand_to_arm['left']]
                    right = actions[vr.hand_to_arm['right']]
                    np.testing.assert_allclose(left['_xlevr.anchor_command']['offset'][:3], expected_left, atol=1e-12)
                    np.testing.assert_allclose(right['_xlevr.anchor_command']['offset'][:3], expected_right, atol=1e-12)
                    np.testing.assert_allclose([left[f'ee.delta_{a}'] for a in 'xyz'], expected_left, atol=1e-12)
                    np.testing.assert_allclose([right[f'ee.delta_{a}'] for a in 'xyz'], expected_right, atol=1e-12)
                    np.testing.assert_allclose(left['_xlevr.anchor_command']['offset'][3:],
                                               right['_xlevr.anchor_command']['offset'][3:], atol=1e-12)
                    np.testing.assert_allclose([left[f'ee.delta_r{a}'] for a in 'xyz'],
                                               [right[f'ee.delta_r{a}'] for a in 'xyz'], atol=1e-12)
        self.assertIsNone(vr.mappers['left'].position_axis_remap)
        self.assertIsNone(vr.mappers['right'].position_axis_remap)

    def test_invalid_per_hand_settings_are_rejected_before_connecting(self):
        for overrides in (
            {'middle': {}}, {'left': {'position_axis_typo': []}},
            {'left': {'position_control_mode': 'legacy_frame_delta'}},
            {'left': {'require_squeeze_to_move': False}},
            {'left': {'position_axis_remap': [[2., 0., 0.], [0., 1., 0.], [0., 0., 1.]]}},
            {'left': {'position_axis_remap': [[0., 1.], [1., 0.]]}},
            {'left': {'position_axis_remap': [[float('nan'), 0., 0.], [0., 1., 0.], [0., 0., 1.]]}},
        ):
            cfg = configuration()
            cfg.input.mapping_by_hand = overrides
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                build_runtime(cfg)

    def test_snapshot_is_detached_and_partial_left_pose_keeps_its_age(self):
        monitor = VRMonitorBridge('.', {})
        for side in ('left', 'right'):
            monitor._record_goal(SimpleNamespace(arm=side, target_position=[1., 2., 3.],
                metadata={'orientation_quat': [0, 0, 0, 1], 'grip_active': True,
                          'buttons': {'squeeze': True}}),
                receive_monotonic_s=10.)
        monitor._record_goal(SimpleNamespace(arm='left', target_position=None,
                            metadata={'grip_active': False}), receive_monotonic_s=11.)
        snapshot = monitor.get_hand_snapshot()
        self.assertEqual(snapshot['left'].metadata['pose_receive_monotonic_s'], 10.)
        self.assertEqual(snapshot['left'].target_position, [1., 2., 3.])
        snapshot['left'].metadata['grip_active'] = True
        self.assertFalse(monitor.get_hand_snapshot()['left'].metadata['grip_active'])
        vr = VRInput(SimpleNamespace(fine_trigger_threshold=.5, require_squeeze_to_move=True))
        vr._connected, vr._vr_monitor = True, monitor
        vr._vr_thread = SimpleNamespace(is_alive=lambda: True)
        with patch('time.monotonic', return_value=11.):
            rows = vr.get_hand_actions()
        self.assertEqual(rows['left']['xlevr.sample_age_s'], 1.)
        self.assertFalse(rows['left']['xlevr.grip_active'])
        self.assertFalse(rows['left']['xlevr.enabled'])
        self.assertTrue(rows['right']['xlevr.grip_active'])

    def test_front_trigger_alone_does_not_engage_and_events_are_disabled(self):
        runtime = build_runtime(configuration())
        vr = runtime.teleoperator
        self.assertFalse(vr.input.config.enable_left_events)
        vr.input = HandInput()
        vr.input.rows = {side: hand(1, trigger=1.) for side in ('left', 'right')}
        action = vr.get_action()
        self.assertTrue(all(not a['_xlevr.anchor_command']['active'] for a in action.values()))
        self.assertTrue(all(a['gripper.pos'] == 0 for a in action.values()))
        self.assertIsNot(vr.mappers['left'], vr.mappers['right'])
        self.assertIsNone(runtime.robot)

    def test_config_rejects_wrong_bindings_modes_and_single_topology(self):
        for override in ('bindings', 'mode', 'mapping', 'topology', 'acceleration'):
            cfg = configuration()
            if override == 'bindings':
                cfg.input.hand_to_arm.right = cfg.input.hand_to_arm.left
            elif override == 'mode':
                cfg.control_mode = 'delta'
            elif override == 'mapping':
                cfg.pose_mapping = 'calibrated_absolute'
            elif override == 'topology':
                cfg.operator.arms.pop('right')
                cfg.operator.cartesian_target_order = ['left']
            else:
                cfg.reference_options = dict(limit_acceleration=False)
            with self.subTest(override=override), self.assertRaises(ValueError):
                build_runtime(cfg)


class DualRuntimeTest(unittest.TestCase):
    robot_type = DualRobot
    cartesian_adapter = 'legacy_anchor'

    def setUp(self):
        self.clock = Clock()
        self.clock.value = 100.
        self.clock_patch = patch('time.monotonic', self.clock.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        cfg = configuration(identity_bindings=True)
        self.vr = VROperator(**{k: v for k, v in cfg.input.items() if k != 'type'})
        self.vr.input = HandInput()
        self.robot = self.robot_type()
        self.runtime = DualVRTeleopRuntime(self.vr, self.robot, execute=True,
            control_mode='reference', cartesian_adapter=self.cartesian_adapter,
            max_position_step_m=.005, max_rotation_step_rad=.015)
        self.runtime.connect()
        self.addCleanup(self.runtime.close)
        self.seq = 0

    def tick(self, left=True, right=True, xleft=0., xright=0., **kwargs):
        self.clock.advance()
        self.seq += 1
        self.vr.input.rows = {'left': hand(self.seq, left, xleft, **kwargs),
                              'right': hand(self.seq, right, xright)}
        return self.runtime.step()

    def move(self, count=16, left=True, right=True):
        for i in range(count):
            self.tick(left, right, .0003 * i, .0002 * i)

    def test_release_one_arm_keeps_other_moving_and_whole_packets(self):
        released = 'left'
        self.move()
        other = 'right' if released == 'left' else 'left'
        initial_other = self.robot.poses[other].copy()
        old_reference = self.runtime._arms[released]['control'].anchor.target.copy()
        initial_velocity = self.runtime._arms[released]['control'].linear_velocity.copy()
        stopped_target = None
        for i in range(30):
            self.tick(left=released != 'left', right=released != 'right',
                      xleft=.005 + .0002 * i, xright=.005 + .0002 * i)
            state = self.runtime._arms[released]
            if state['state'] == 'holding' and stopped_target is None:
                stopped_target = self.robot.targets[-1][released]
            if stopped_target is not None:
                np.testing.assert_allclose(self.robot.targets[-1][released], stopped_target, atol=1e-12)
        self.assertEqual(self.robot.events.count('stop'), 0)
        self.assertEqual(self.runtime._arms[other]['state'], 'tracking')
        self.assertGreater(np.linalg.norm(self.robot.poses[other][:3] - initial_other[:3]), .001)
        self.assertIsNone(self.runtime._arms[released]['control'].anchor.target)
        self.assertGreater(np.linalg.norm(initial_velocity), 0.)
        self.assertTrue(all(set(t) == {'left', 'right'} for t in self.robot.targets))
        self.assertIsNotNone(stopped_target)
        self.assertIsNotNone(old_reference)

    def test_right_release_while_left_continues(self):
        self.move()
        initial = self.robot.poses['left'].copy()
        for i in range(22):
            self.tick(True, False, .005 + .0002 * i, .005)
        self.assertEqual(self.runtime._arms['right']['state'], 'holding')
        self.assertEqual(self.runtime._arms['left']['state'], 'tracking')
        self.assertGreater(np.linalg.norm(self.robot.poses['left'][:3] - initial[:3]), .001)
        self.assertNotIn('stop', self.robot.events)

    def test_late_engagement_and_reclutch_preserve_other_and_startup_anchors(self):
        self.move(left=False)
        np.testing.assert_allclose(self.robot.poses['left'], self.robot.anchors['left'])
        for i in range(12):
            self.tick(True, True, .006 + i * .0003, .004 + i * .0002)
        self.assertEqual(self.runtime._arms['left']['state'], 'tracking')
        startup = self.runtime._arms['left']['control'].startup_anchor.copy()
        for i in range(20):
            self.tick(False, True, .01, .006 + i * .0002)
        other_key = self.runtime._arms['right']['key']
        for i in range(6):
            self.tick(True, True, .1 + i * .0001, .010 + i * .0002)
        np.testing.assert_array_equal(self.runtime._arms['left']['control'].startup_anchor, startup)
        self.assertEqual(self.runtime._arms['right']['key'], other_key)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertNotIn('stop', self.robot.events)
        for previous, current in zip(self.robot.targets, self.robot.targets[1:]):
            self.assertLessEqual(np.linalg.norm(np.array(current['left'][:3]) - previous['left'][:3]), .005 + 1e-12)

    def test_both_release_stop_immediately_without_chasing_residual(self):
        self.move()
        self.robot.follow = False
        self.robot.poses['left'][0] += .0075
        self.robot.poses['left'][3:] = (Rotation.from_quat(self.robot.poses['left'][3:])
                                       * Rotation.from_euler('z', .057)).as_quat()
        count = len(self.robot.targets)
        self.tick(False, False)
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events[-2:], ['stop', 'wait_stopped'])
        self.assertEqual(self.runtime.last_reference_result['state'], 'stopped')
        for _ in range(25):
            self.tick(False, False)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)
        restart_actual = copy.deepcopy(self.robot.poses)
        for _ in range(5):
            self.tick(True, True)
        self.assertEqual(self.robot.events.count('start'), 2)
        for name, actual in restart_actual.items():
            np.testing.assert_allclose(self.runtime._arms[name]['control'].startup_anchor, actual)

    def test_press_during_deceleration_resumes_without_an_extra_release(self):
        self.move()
        self.tick(False, True, .005, .005)
        for i in range(20):
            self.tick(True, True, .1, .005 + .0001 * i)
        self.assertEqual(self.runtime._arms['left']['state'], 'tracking')
        self.assertEqual(self.runtime._arms['right']['state'], 'tracking')
        self.assertFalse(self.runtime.last_reference_result['arms']['left']['reengage_requires_release'])
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertNotIn('stop', self.robot.events)
        for previous, current in zip(self.robot.targets, self.robot.targets[1:]):
            for name in ('left', 'right'):
                self.assertLessEqual(np.linalg.norm(current[name][:3] - previous[name][:3]), .12 / 30 + 1e-10)

    def test_stale_active_hand_stops_group_no_partial_send_and_no_auto_restart(self):
        self.move()
        count = len(self.robot.targets)
        self.tick(age=.3)
        self.assertEqual(self.runtime.input_pause_status['reason'], 'input_stale')
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('stop'), 1)
        for _ in range(4):
            self.tick()
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.runtime.last_reference_result['state'], 'input_paused')

    def wait_input_ready(self):
        for _ in range(20): self.tick(False, False)
        self.assertEqual(self.runtime.input_pause_status['state'], 'ready')

    def test_position_spike_pauses_held_grips_never_restart_and_reclutch_anchors_actual_pose(self):
        self.move()
        sent = len(self.robot.targets)
        self.tick(xleft=.4)
        pause = self.runtime.input_pause_status
        self.assertEqual(pause['reason'], 'spike_pending')
        self.assertTrue(pause['stopped'])
        self.assertGreater(pause['vr_input_speed_m_s'], 2.)
        self.assertEqual(len(self.robot.targets), sent)
        self.assertEqual(self.robot.events.count('wait_stopped'), 1)
        for _ in range(25): self.tick(xleft=.4)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.runtime.input_pause_status['state'], 'release_required')
        self.wait_input_ready()
        self.assertEqual(self.vr.input.hud[-1]['phase'], 'input_ready')
        for _ in range(5): self.tick(False, False)
        self.assertEqual(self.robot.events.count('start'), 1)
        actual = copy.deepcopy(self.robot.poses)
        self.tick(left=True, right=False)  # New edge clears latch, sends no robot target.
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertEqual(len(self.robot.targets), sent)
        self.tick(left=True, right=False)  # Mapper arming sample.
        self.tick(left=True, right=False)  # Fresh active input may now start.
        self.assertEqual(self.robot.events.count('start'), 2)
        for name in actual:
            np.testing.assert_allclose(self.robot.anchors[name], actual[name])
        self.assertEqual(self.runtime.last_reference_result['state'], 'await_fresh_sample')

    def test_tracking_loss_requires_new_tracked_samples_and_lost_tracking_resets_ready(self):
        self.move()
        self.vr.input.rows['left']['xlevr.verification_tracking'] = False
        self.runtime.step()
        self.assertEqual(self.runtime.input_pause_status['reason'], 'tracking_lost')
        for _ in range(20):
            self.clock.advance(); self.seq += 1
            self.vr.input.rows = {side: hand(self.seq) for side in ('left', 'right')}
            self.vr.input.rows['left']['xlevr.verification_tracking'] = False
            self.runtime.step()
        self.assertEqual(self.runtime.input_pause_status['state'], 'release_required')
        self.wait_input_ready()
        self.vr.input.rows['left']['xlevr.verification_tracking'] = False
        self.runtime.step()
        self.assertEqual(self.runtime.input_pause_status['state'], 'release_required')
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_trigger_stick_or_frozen_input_cannot_pass_recovery_gate(self):
        self.move(); self.tick(age=.3)
        for _ in range(20): self.tick(False, False, trigger=.5)
        self.assertEqual(self.runtime.input_pause_status['state'], 'release_required')
        self.tick(False, False)
        for _ in range(20):
            self.clock.advance(); self.seq += 1
            self.vr.input.rows = {side: hand(self.seq) for side in ('left', 'right')}
            self.vr.input.rows['right']['xlevr.thumbstick'] = {'x': .5}
            self.runtime.step()
        self.assertEqual(self.runtime.input_pause_status['state'], 'release_required')
        self.tick(False, False)
        for _ in range(20):
            self.clock.advance()
            self.runtime.step()  # No new sequence or receive timestamp.
        self.assertNotEqual(self.runtime.input_pause_status['state'], 'ready')
        self.wait_input_ready()

    def test_read_only_observation_never_claims_it_stopped_the_robot(self):
        self.runtime.execute = False
        self.runtime.observe_only = True
        with self.assertRaisesRegex(RuntimeError, 'input_stale'):
            self.tick(age=.3)
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertNotIn('start', self.robot.events)
        self.assertNotIn('stop', self.robot.events)

    def test_second_position_jump_resets_stability_and_controller_fault_while_paused_is_fatal(self):
        self.move(); self.tick(age=.3)
        for _ in range(10): self.tick(False, False)
        self.tick(False, False, xleft=.4)
        self.assertEqual(self.runtime.input_pause_status['state'], 'release_required')
        self.robot.frame_fault = True
        with self.assertRaisesRegex(RuntimeError, 'fault'):
            self.tick(False, False)
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_missing_feedback_stops_group_no_partial_packet(self):
        self.move()
        count = len(self.robot.targets)
        self.robot.missing = True
        with self.assertRaisesRegex(RuntimeError, 'complete configured arm group'):
            self.tick()
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_stale_second_arm_feedback_stops_before_any_packet(self):
        self.move()
        count = len(self.robot.targets)
        self.robot.stale_arm = True
        with self.assertRaisesRegex(RuntimeError, 'stale for right'):
            self.tick()
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_stop_unconfirmed_is_latched_even_after_fresh_release(self):
        self.move()
        self.robot.stop_timeout = True
        with self.assertRaisesRegex(TimeoutError, 'stop unconfirmed'):
            self.tick(age=.3)
        self.robot.stop_timeout = False
        with self.assertRaisesRegex(RuntimeError, 'stop is unconfirmed'):
            self.tick(False, False)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_release_settle_timeout_does_not_silently_leave_other_arm_running(self):
        self.move()
        self.robot.follow = False
        # Continued motion within the hard cap must still time out.
        with self.assertRaisesRegex(RuntimeError, 'release_settle_timeout'):
            for i in range(70):
                self.robot.poses['left'][0] += .001 if i % 2 else -.001
                self.tick(False, True, 0., .004)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_stable_residual_hold_and_reclutch_do_not_rewind_or_catch_up(self):
        self.move()
        self.robot.follow = False
        self.robot.poses['left'][0] += .0075
        self.robot.poses['left'][3:] = (Rotation.from_quat(self.robot.poses['left'][3:])
                                       * Rotation.from_euler('z', .057)).as_quat()
        for _ in range(75):  # Longer than the old two-second precision timeout.
            self.tick(False, True, 0., .004)
        self.assertEqual(self.runtime._arms['left']['state'], 'holding')
        self.assertNotIn('stop', self.robot.events)
        before = self.runtime._arms['left']['control'].command_pose.copy()
        cumulative = (self.robot.sdk_packets[-1]['left'].copy()
                      if hasattr(self.robot, 'sdk_packets') else None)
        other_key = self.runtime._arms['right']['key']
        actual = self.robot.poses['left'].copy()
        for _ in range(15):
            self.tick(True, True, .1, .004)  # New grip position, held stationary.
            np.testing.assert_allclose(self.runtime._arms['left']['control'].command_pose,
                                       before, atol=1e-12)
            if cumulative is not None:
                np.testing.assert_allclose(self.robot.sdk_packets[-1]['left'], cumulative, atol=1e-12)
        np.testing.assert_allclose(self.runtime._arms['left']['control'].anchor.robot_anchor, actual)
        self.assertEqual(self.runtime._arms['right']['key'], other_key)
        self.assertEqual(self.robot.events.count('start'), 1)
        for i in range(1, 10):
            self.tick(True, True, .1 + .0002 * i, .004)
        self.assertGreater(np.linalg.norm(self.runtime._arms['left']['control'].command_pose[:3] - before[:3]), 0.)

    def test_both_release_stop_failure_blocks_restart(self):
        self.move()
        self.robot.stop_timeout = True
        count = len(self.robot.targets)
        with self.assertRaisesRegex(TimeoutError, 'stop unconfirmed'):
            self.tick(False, False)
        self.assertEqual(len(self.robot.targets), count)
        self.robot.stop_timeout = False
        with self.assertRaisesRegex(RuntimeError, 'stop is unconfirmed'):
            self.tick(False, False)
        self.assertEqual(self.robot.events.count('start'), 1)

    def test_released_arm_excessive_residual_stops_entire_group(self):
        self.move()
        self.robot.follow = False
        self.robot.poses['left'][0] += .06
        with self.assertRaisesRegex(RuntimeError, 'release_tracking_error'):
            self.tick(False, True, 0., .004)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_vr_only_diagnostics_include_both_hands_without_robot_feedback(self):
        self.runtime.close()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'dual.jsonl'
            self.runtime = DualVRTeleopRuntime(self.vr, control_mode='reference',
                cartesian_adapter=self.cartesian_adapter, diagnostics_path=str(path))
            self.runtime.connect()
            self.move(count=5)
            self.runtime.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(set(rows[-1]['hands']), {'left', 'right'})
        self.assertEqual(set(rows[-1]['mapped_action']), {'left', 'right'})
        self.assertIsNone(rows[-1]['feedback'])
        self.assertIsNone(rows[-1]['sent_relative_target'])
        self.assertIsNone(self.runtime.robot)

    def test_second_arm_bad_target_rejects_entire_cycle(self):
        self.move()
        count = len(self.robot.targets)
        self.runtime._arms['right']['control'].command_pose[0] += .2
        with self.assertRaises(RuntimeError):
            self.tick()
        self.assertEqual(len(self.robot.targets), count)
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_observe_mode_has_no_follower_or_udp_commands(self):
        self.runtime.close()
        self.robot.events.clear()
        self.robot.allow_motion = False
        self.runtime = DualVRTeleopRuntime(self.vr, self.robot, execute=False, observe_only=True,
            control_mode='reference', cartesian_adapter=self.cartesian_adapter,
            max_position_step_m=.005, max_rotation_step_rad=.015)
        self.runtime.connect()
        self.move()
        self.assertEqual(self.robot.targets, [])
        self.assertNotIn('start', self.robot.events)
        self.assertNotIn('stop', self.robot.events)
        self.assertNotIn('begin_sdk', self.robot.events)
        self.assertEqual(self.runtime._arms['left']['state'], 'tracking')


class DualSDKRuntimeTest(DualRuntimeTest):
    """Run every independent-hand lifecycle check through the SDK path too."""
    robot_type = DualSDKRobot
    cartesian_adapter = 'sdk_tool_cumulative'

    def test_config_selects_sdk_and_keeps_dual_tuning_separate(self):
        cfg = configuration()
        runtime = build_runtime(cfg)
        self.assertEqual(runtime.cartesian_adapter, 'sdk_tool_cumulative')
        self.assertIsNone(runtime.robot)
        self.assertFalse(runtime.execute)
        self.assertEqual(cfg.operator.ws_port, 5999)
        self.assertEqual(runtime.reference_control.config.max_angular_accel_rad_s2, 6.)
        self.assertIsNone(runtime.reference_control.config.max_reference_lead_m)
        self.assertIsNone(runtime.reference_control.config.max_reference_lead_rad)
        self.assertFalse(runtime.reference_control.config.enforce_tracking_error_limits)
        self.assertEqual(runtime.reference_control.config.max_angular_speed_rad_s, 1.4)
        self.assertEqual(runtime.reference_control.config.max_angular_brake_rad_s2, 10.)
        self.assertEqual(runtime.reference_control.config.rotation_follow_mode, 'incremental_velocity')
        self.assertFalse(runtime.stop_on_both_release)
        self.assertTrue(runtime.keep_session_on_query_error)

    def test_site_binding_routes_hands_to_expected_ids_and_releases_independently(self):
        self.runtime.close()
        cfg = configuration()
        self.vr = VROperator(**{k: v for k, v in cfg.input.items() if k != 'type'})
        self.vr.input = HandInput()
        self.robot = DualSDKRobot()
        self.runtime = DualVRTeleopRuntime(self.vr, self.robot, execute=True,
            control_mode='reference', cartesian_adapter=self.cartesian_adapter,
            max_position_step_m=.005, max_rotation_step_rad=.015)
        self.runtime.connect()
        self.addCleanup(self.runtime.close)
        left_hand_arm = self.vr.hand_to_arm['left']
        right_hand_arm = self.vr.hand_to_arm['right']
        self.assertEqual(cfg.operator.arms[left_hand_arm].ee_id, 1)
        self.assertEqual(cfg.operator.arms[right_hand_arm].ee_id, 0)
        self.assertEqual(cfg.operator.cartesian_target_order, ['left', 'right'])
        initial = copy.deepcopy(self.robot.poses)
        self.move(left=True, right=False)
        self.assertGreater(np.linalg.norm(self.robot.poses[left_hand_arm][:3]
                                         - initial[left_hand_arm][:3]), .001)
        np.testing.assert_allclose(self.robot.poses[right_hand_arm], initial[right_hand_arm])
        for i in range(40):
            self.tick(left=False, right=True, xright=.0003 * i)
        self.assertEqual(self.runtime._arms[left_hand_arm]['state'], 'holding')
        self.assertEqual(self.runtime._arms[right_hand_arm]['state'], 'tracking')
        self.assertGreater(np.linalg.norm(self.robot.poses[right_hand_arm][:3]
                                         - initial[right_hand_arm][:3]), .001)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertNotIn('stop', self.robot.events)

    def test_sdk_initializes_once_and_reclutch_never_resets_other_arm(self):
        self.test_late_engagement_and_reclutch_preserve_other_and_startup_anchors()
        self.assertEqual(self.robot.events.count('begin_sdk'), 1)
        self.assertIsNone(self.runtime.last_relative_target)
        self.assertTrue(self.runtime.last_sdk_command['local_send_success'])
        self.assertFalse(self.runtime.last_sdk_command['execution_confirmed'])
        self.assertTrue(all(r['relative_target'] is None
                            for r in self.runtime.last_reference_result['arms'].values()))
        self.assertTrue(all(set(p) == {'left', 'right'} for p in self.robot.sdk_packets))

    def test_sdk_initialization_failure_stops_before_first_target(self):
        self.robot.fail_begin = True
        with self.assertRaisesRegex(RuntimeError, 'initialization failed'):
            self.move(count=2)
        self.assertEqual(self.robot.targets, [])
        self.assertEqual(self.robot.events.count('stop'), 1)

    def test_sdk_send_failure_stops_and_requires_new_release(self):
        self.move()
        sent = len(self.robot.sdk_packets)
        self.robot.fail_send = True
        with self.assertRaisesRegex(OSError, 'UDP failure'):
            self.tick(xleft=.006, xright=.004)
        self.assertEqual(len(self.robot.sdk_packets), sent)
        self.assertEqual(self.robot.events.count('stop'), 1)
        self.tick()
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertEqual(self.runtime.last_reference_result['state'], 'release_required')

    def test_sdk_diagnostics_record_wire_rows_without_claiming_arrival(self):
        with tempfile.TemporaryDirectory() as directory:
            from fluxvla.engines.operators.vr.diagnostics import XLeVRDiagnosticsWriter
            path = Path(directory) / 'sdk.jsonl'
            self.runtime._diagnostics = XLeVRDiagnosticsWriter(path)
            self.move(count=8, right=False)
            self.runtime._diagnostics.close()
            self.runtime._diagnostics = None
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        sent = [r for r in rows if r.get('sdk_tool_command')]
        self.assertTrue(sent)
        self.assertEqual(set(sent[-1]['sdk_tool_command']['arms']), {'left', 'right'})
        self.assertIsNone(sent[-1]['sent_relative_target'])
        self.assertFalse(sent[-1]['sdk_tool_command']['execution_confirmed'])
        np.testing.assert_allclose(sent[-1]['sdk_tool_command']['arms']['right']['cumulative_pe321'], 0.)


class ReleaseControlTest(unittest.TestCase):
    def test_braking_bounds_both_velocities_and_discards_old_goal(self):
        control = VRPoseController()
        control.start(IDENTITY)
        actual = IDENTITY.copy()
        for seq in range(20):
            result = control.step(dict(active=True, session_id='test', anchor_id=1,
                sample_sequence=seq, offset=[.03, 0, 0, 0, 0, .08], gripper=0.), actual, .02)
            actual = np.array(result['command_pose'])
        previous_linear, previous_angular = control.linear_velocity.copy(), control.angular_velocity.copy()
        self.assertGreater(np.linalg.norm(previous_linear), 0)
        for _ in range(60):
            previous_pose = control.command_pose.copy()
            result = control.step_released(actual, .02)
            self.assertFalse(result['stop_requested'])
            self.assertLessEqual(np.linalg.norm(control.linear_velocity - previous_linear), .4 * .02 + 1e-12)
            self.assertLessEqual(np.linalg.norm(control.angular_velocity - previous_angular), .96 * .02 + 1e-12)
            self.assertLessEqual(np.linalg.norm(control.command_pose[:3] - previous_pose[:3]), .12 * .02 + 1e-12)
            previous_linear, previous_angular = control.linear_velocity.copy(), control.angular_velocity.copy()
            actual = control.command_pose.copy()
        self.assertEqual(result['state'], 'holding')
        self.assertIsNone(control.anchor.target)
        np.testing.assert_allclose(control.linear_velocity, 0)
        np.testing.assert_allclose(control.angular_velocity, 0)

    def test_release_timeout_and_stale_feedback_request_shared_stop(self):
        control = VRPoseController(ReferenceConfig(release_timeout_s=.15))
        control.start(IDENTITY)
        actual = IDENTITY.copy()
        for i in range(10):
            actual[0] = .01 + .001 * (i % 2)
            result = control.step_released(actual, .02)
        self.assertEqual(result['fault_reason'], 'release_settle_timeout')
        self.assertTrue(result['stop_requested'])
        control.start(IDENTITY)
        result = control.step_released(IDENTITY, .02, feedback_age_s=.3)
        self.assertEqual(result['fault_reason'], 'feedback_stale')

    def test_long_hold_does_not_turn_one_small_feedback_change_into_immediate_timeout(self):
        control = VRPoseController()
        control.start(IDENTITY)
        for _ in range(150):
            control.step_released(IDENTITY, .02)
        changed = IDENTITY.copy()
        changed[0] = .0008
        result = control.step_released(changed, .02)
        self.assertFalse(result['stop_requested'])
        for _ in range(10):
            result = control.step_released(changed, .02)
        self.assertEqual(result['state'], 'holding')


class DualLoopbackTest(unittest.TestCase):
    def test_dual_readonly_inspection_preserves_mapping_and_sends_only_get(self):
        with FakeController() as server:
            server.motion_override = dual_motion()
            report = inspect_controller(dict(host='127.0.0.1',
                status_ws_port=server.server.socket.getsockname()[1], timeout_s=.3,
                arms=DUAL_ARMS, cartesian_target_order=['left', 'right'],
                feedback_position_unit='mm', allow_motion=True), samples=1)
            self.assertTrue(report['query_success'], report['error'])
            self.assertEqual(set(report['samples'][0]['arms']), {'left', 'right'})
            self.assertTrue(all(command == 'get' for command in server.commands))
            self.assertEqual(report['udp_targets_sent'], 0)

    def test_real_ws_udp_whole_group_and_signed_order(self):
        self.check_real_ws_udp('legacy_anchor')

    def test_sdk_real_ws_udp_matches_planned_poses_and_holds_other_arm(self):
        self.check_real_ws_udp('sdk_tool_cumulative')

    def check_real_ws_udp(self, adapter):
        with FakeController() as server:
            server.motion_override = dual_motion()
            robot = server.operator(arms=DUAL_ARMS, cartesian_target_order=['right', 'left'],
                                    cartesian_wire_format='pe321', feedback_position_unit='mm')
            cfg = configuration(identity_bindings=True)
            vr = VROperator(**{k: v for k, v in cfg.input.items() if k != 'type'})
            vr.input = HandInput()
            runtime = DualVRTeleopRuntime(vr, robot, execute=True, control_mode='reference',
                                         cartesian_adapter=adapter,
                                         max_position_step_m=.005, max_rotation_step_rad=.015)
            clock = Clock()
            clock.value = 100.
            try:
                with patch('time.monotonic', clock.now):
                    runtime.connect()
                    packets = []
                    for seq in range(1, 42):
                        clock.advance()
                        vr.input.rows = {'left': hand(seq, True, seq * .0002, -.001 * seq),
                                         'right': hand(seq, False)}
                        runtime.step()
                        if seq >= 2:
                            packet = server.udp.recv(65535)
                            header = struct.unpack('<IIQqqq', packet[:40])
                            self.assertEqual(header, (len(packet) - 40, 0, 0, 0, 0, 0))
                            payload = json.loads(packet[40:])
                            self.assertEqual(payload['type'], '321')
                            self.assertEqual(len(payload['pe']), 2)
                            self.assertTrue(all(len(row) == 6 for row in payload['pe']))
                            np.testing.assert_allclose(payload['pe'][0], np.zeros(6), atol=1e-12)
                            packets.append(payload)
                            if runtime.last_reference_result['state'] == 'await_fresh_sample':
                                starts = {name: np.array(arm['tcp_pq_m'])
                                          for name, arm in runtime.last_frame['arms'].items()}
                    self.assertGreater(abs(packets[-1]['pe'][1][0]), 0.)
                    if adapter == 'sdk_tool_cumulative':
                        from test_a10_sdk_adapter import sdk_integrate
                        for row, name in enumerate(robot.cartesian_target_order):
                            actual = sdk_integrate(starts[name], [np.array(p['pe'][row]) for p in packets])
                            planned = runtime.last_sdk_command['arms'][name]['planned_pose_m']
                            np.testing.assert_allclose(actual[:3], planned[:3], atol=1e-5)
                            self.assertLess((Rotation.from_quat(actual[3:]).inv()
                                             * Rotation.from_quat(planned[3:])).magnitude(), 1e-5)
                        self.assertIsNone(runtime.last_relative_target)
                    self.assertEqual(server.commands.count('follower_cart'), 1)
                    runtime.close()
                    self.assertEqual(server.commands.count('stop_follower'), 1)
            finally:
                runtime.close()


if __name__ == '__main__':
    unittest.main()
