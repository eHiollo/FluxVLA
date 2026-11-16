"""Final side-grip XYZ lock and trigger controls; synthetic input only."""
import copy
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from mmengine import Config
from scipy.spatial.transform import Rotation
from fluxvla.engines.operators.vr_operator import VROperator
from fluxvla.engines.operators.vr.control import ReferenceConfig, VRPoseController
from fluxvla.engines.operators.vr.mapper import XLeVRDeltaEEMapper
import test_vr_dual_runtime as dual
from test_vr_dual_runtime import HandInput, hand

IDENTITY = np.array([.1, .2, .3, 0., 0., 0., 1.])


def sample(seq, angle=0., x=0., *, trigger=0., grip=True):
    row = dual.hand(seq, True, x, angle=angle, trigger=trigger)
    row['xlevr.sample_receive_time_s'] = seq / 90.
    row['xlevr.grip_active'] = grip
    row['xlevr.buttons']['squeeze'] = grip
    return row


def mapper(enabled=True):
    return XLeVRDeltaEEMapper(squeeze_translation_only=enabled, anchor_translation_frame='robot_base')


class TranslationOnlyTest(unittest.TestCase):
    def setUp(self):
        self.mapper = mapper()
        self.controller = VRPoseController(ReferenceConfig(rotation_follow_mode='incremental_velocity',
            max_angular_speed_rad_s=1.4, max_angular_accel_rad_s2=6.,
            max_angular_brake_rad_s2=10., enforce_tracking_error_limits=False),
            pose_mapping='world_translation_local_rotation')
        self.controller.start(IDENTITY)
        self.seq = 0

    def step(self, angle=0., x=0., *, trigger=0., grip=True, actual=None):
        self.seq += 1
        out = self.mapper(sample(self.seq, angle, x, trigger=trigger, grip=grip))
        return self.controller.step(out['_xlevr.anchor_command'],
            self.controller.command_pose.copy() if actual is None else actual, 1./90., continuous_reanchor=True)

    def test_grip_translates_while_wrist_rotation_cannot_change_orientation(self):
        for i in range(1, 30): self.step(.025*i, .0005*i)
        self.assertFalse(self.controller._rotation_enabled)
        self.assertGreater(np.linalg.norm(self.controller.command_pose[:3] - IDENTITY[:3]), .0001)
        np.testing.assert_allclose(self.controller.command_pose[3:], IDENTITY[3:], atol=1e-12)
        np.testing.assert_array_equal(self.controller.angular_velocity, np.zeros(3))

    def test_trigger_has_priority_and_trigger_alone_keeps_rotation(self):
        self.step()
        for i in range(1, 30): self.step(.015*i, trigger=1., grip=True)
        self.assertTrue(self.controller._rotation_enabled)
        self.assertGreater(Rotation.from_quat(self.controller.command_pose[3:]).magnitude(), .001)
        for i in range(30, 50): self.step(.015*i, trigger=1., grip=False)
        self.assertTrue(self.controller._rotation_enabled)
        self.assertFalse(self.controller.fault_reason)

    def test_switch_from_rotation_locks_command_pose_immediately_without_rewind_to_actual(self):
        for i in range(1, 30): self.step(.015*i, trigger=1.)
        self.assertGreater(np.linalg.norm(self.controller.angular_velocity), .05)
        held = self.controller.command_pose[3:].copy()
        self.step(.8, .001, actual=IDENTITY)
        np.testing.assert_allclose(self.controller.command_pose[3:], held, atol=1e-12)
        np.testing.assert_array_equal(self.controller.angular_velocity, np.zeros(3))
        for i in range(1, 20): self.step(.8+.04*i, .001+.0001*i, actual=IDENTITY)
        np.testing.assert_allclose(self.controller.command_pose[3:], held, atol=1e-12)

    def test_unlock_drops_rotation_made_during_lock_then_follows_only_new_motion(self):
        for i in range(1, 20): self.step(.04*i)
        held = self.controller.command_pose[3:].copy()
        self.step(1.1, trigger=1.)
        np.testing.assert_allclose(self.controller.command_pose[3:], held, atol=1e-12)
        for _ in range(5): self.step(1.1, trigger=1.)
        np.testing.assert_allclose(self.controller.command_pose[3:], held, atol=1e-12)
        for i in range(1, 12): self.step(1.1+.015*i, trigger=1.)
        self.assertGreater(Rotation.from_quat(self.controller.command_pose[3:]).magnitude(), .001)
        self.assertLessEqual(np.linalg.norm(self.controller.angular_velocity), 1.4 + 1e-12)

    def test_duplicate_pose_cannot_unlock_using_mutated_trigger(self):
        self.step(.1)
        row = sample(self.seq, .8, trigger=1.)
        command = self.mapper(row)['_xlevr.anchor_command']
        self.assertFalse(command['rotation_enabled'])
        command = copy.deepcopy(command)
        command['rotation_enabled'] = True
        self.controller.step(command, self.controller.command_pose.copy(), 1./90.)
        self.assertFalse(self.controller._rotation_enabled)
        np.testing.assert_array_equal(self.controller.angular_velocity, np.zeros(3))

    def test_anchor_rotation_follow_mode_also_locks_and_resumes_without_jump(self):
        self.controller = VRPoseController(ReferenceConfig(enforce_tracking_error_limits=False),
                                          pose_mapping='world_translation_local_rotation')
        self.controller.start(IDENTITY)
        for i in range(1, 12): self.step(.02*i)
        self.step(.8, trigger=1.)
        np.testing.assert_allclose(self.controller.command_pose[3:], IDENTITY[3:], atol=1e-12)
        for i in range(1, 15): self.step(.8+.02*i, trigger=1.)
        self.assertGreater(Rotation.from_quat(self.controller.command_pose[3:]).magnitude(), .001)

    def test_default_keeps_original_side_grip_rotation_protocol(self):
        old = mapper(False)
        old(sample(1))
        output = old(sample(2, .05))
        self.assertNotIn('rotation_enabled', output['_xlevr.anchor_command'])
        self.assertGreater(np.linalg.norm(output['_xlevr.anchor_command']['offset'][3:]), 0.)

    def test_invalid_option_and_command_mode_are_rejected(self):
        with self.assertRaises(ValueError): XLeVRDeltaEEMapper(squeeze_translation_only='yes')
        command = self.mapper(sample(1))['_xlevr.anchor_command']
        command['rotation_enabled'] = 0
        with self.assertRaises(ValueError): self.controller.validate_command(command, IDENTITY)


class IndependentHandsTest(unittest.TestCase):
    robot_type = dual.DualSDKRobot
    cartesian_adapter = 'sdk_tool_cumulative'
    setUp = dual.DualRuntimeTest.setUp
    tick = dual.DualRuntimeTest.tick

    def test_left_trigger_rotation_and_right_grip_translation_are_independent(self):
        for m in self.vr.mappers.values(): m.squeeze_translation_only = True
        initial = self.robot.poses['right'][3:].copy()
        def hands(i, trigger):
            self.clock.advance()
            self.seq += 1
            self.vr.input.rows = {
                'left': dual.hand(self.seq, True, .0003*i, angle=.02*i, trigger=trigger),
                'right': dual.hand(self.seq, True, .0002*i, angle=-.03*i)}
            self.runtime.step()
        for i in range(1, 20): hands(i, 1.)
        self.assertTrue(self.runtime._arms['left']['control']._rotation_enabled)
        self.assertFalse(self.runtime._arms['right']['control']._rotation_enabled)
        np.testing.assert_allclose(self.robot.poses['right'][3:], initial, atol=1e-12)
        held = self.robot.poses['left'][3:].copy()
        for i in range(20, 30): hands(i, 0.)
        np.testing.assert_allclose(self.robot.poses['left'][3:], held, atol=1e-12)
        self.assertEqual(self.robot.events.count('start'), 1)
        self.assertTrue(all(set(p) == {'left', 'right'} for p in self.robot.sdk_packets))



ROOT = Path(__file__).resolve().parents[1]


PROFILE = 'configs/a10/a10_dual_arm_vr_teleop_world_translation.py'


class DualTriggerInputTest(unittest.TestCase):
    def setUp(self):
        cfg = Config.fromfile(str(ROOT/PROFILE)).teleop
        self.vr = VROperator(**{k: v for k, v in cfg.input.items() if k != 'type'})
        self.vr.input = HandInput()
        # Match the runtime's world-translation mapper selection without
        # constructing or connecting a robot runtime.
        for mapper in self.vr.mappers.values(): mapper.anchor_translation_frame = 'robot_base'
        self.now = 100.
        self.seq = 0
        clock = patch('time.monotonic', lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        self.initial = np.array([.1, .2, .3, 0., 0., 0., 1.])
        self.controllers = {}
        for physical in ('left', 'right'):
            controller = VRPoseController(ReferenceConfig(**dict(cfg.reference_options)),
                                         pose_mapping=cfg.pose_mapping)
            controller.start(self.initial)
            self.controllers[physical] = controller

    def tick(self, *, left_trigger=0., right_trigger=0., left_grip=False, right_grip=False,
             x=0., angle=0., age=0., input_enabled=False):
        self.now += 1./90.
        self.seq += 1
        self.vr.input.rows = {
            'left': hand(self.seq, left_grip, x, angle=angle, trigger=left_trigger, age=age),
            'right': hand(self.seq, right_grip, x, angle=-angle, trigger=right_trigger, age=age)}
        # Deliberately disagree with raw enabled: the dual clutch must use each
        # hand's actual buttons and mapper threshold rather than trust a gate
        # calculated from the first hand's shared VRInput settings.
        for raw in self.vr.input.rows.values(): raw['xlevr.enabled'] = input_enabled
        actions = self.vr.get_action()
        for physical, logical in self.vr.hand_to_arm.items():
            command = actions[logical]['_xlevr.anchor_command']
            controller = self.controllers[physical]
            if command['active']:
                controller.step(command, controller.command_pose.copy(), 1./90., continuous_reanchor=True)
        return actions

    def test_trigger_alone_reaches_real_dual_mapper_and_drives_translation_and_rotation(self):
        for i in range(1, 35):
            actions = self.tick(left_trigger=1., x=.0003*i, angle=.015*i)
        logical = self.vr.hand_to_arm['left']
        command = actions[logical]['_xlevr.anchor_command']
        self.assertTrue(self.vr.last_raw['left']['xlevr.enabled'])
        self.assertFalse(self.vr.last_raw['left']['xlevr.grip_active'])
        self.assertTrue(command['active'])
        self.assertTrue(command['rotation_enabled'])
        self.assertGreater(np.linalg.norm(self.controllers['left'].command_pose[:3]-self.initial[:3]), .0001)
        self.assertGreater(Rotation.from_quat(self.controllers['left'].command_pose[3:]).magnitude(), .001)
        self.assertFalse(actions[self.vr.hand_to_arm['right']]['_xlevr.anchor_command']['active'])

    def test_side_grip_still_translates_with_orientation_locked(self):
        for i in range(1, 30): self.tick(left_grip=True, x=.0003*i, angle=.02*i)
        controller = self.controllers['left']
        self.assertFalse(controller._rotation_enabled)
        self.assertGreater(np.linalg.norm(controller.command_pose[:3]-self.initial[:3]), .0001)
        np.testing.assert_allclose(controller.command_pose[3:], self.initial[3:], atol=1e-12)

    def test_trigger_has_priority_when_both_buttons_are_held(self):
        for i in range(1, 30): self.tick(left_grip=True, left_trigger=1., angle=.02*i)
        self.assertTrue(self.controllers['left']._rotation_enabled)
        self.assertGreater(Rotation.from_quat(self.controllers['left'].command_pose[3:]).magnitude(), .001)

    def test_hands_and_site_swapped_bindings_are_independent(self):
        self.assertEqual(self.vr.hand_to_arm, {'left': 'right', 'right': 'left'})
        for i in range(1, 30):
            actions = self.tick(left_trigger=1., right_grip=True, x=.0003*i, angle=.02*i)
        self.assertTrue(actions['right']['_xlevr.anchor_command']['rotation_enabled'])
        self.assertFalse(actions['left']['_xlevr.anchor_command']['rotation_enabled'])
        self.assertGreater(Rotation.from_quat(self.controllers['left'].command_pose[3:]).magnitude(), .001)
        np.testing.assert_allclose(self.controllers['right'].command_pose[3:], self.initial[3:], atol=1e-12)
        self.assertGreater(np.linalg.norm(self.controllers['right'].command_pose[:3]-self.initial[:3]), .0001)

    def test_each_hand_uses_its_own_trigger_threshold(self):
        self.vr.mappers['left'].fine_trigger_threshold = .8
        self.vr.mappers['right'].fine_trigger_threshold = .3
        actions = self.tick(left_trigger=.7, right_trigger=.4)
        self.assertFalse(self.vr.last_raw['left']['xlevr.enabled'])
        self.assertTrue(self.vr.last_raw['right']['xlevr.enabled'])
        self.assertFalse(actions[self.vr.hand_to_arm['left']]['_xlevr.anchor_command']['active'])
        self.assertTrue(actions[self.vr.hand_to_arm['right']]['_xlevr.anchor_command']['active'])
        actions = self.tick(left_trigger=.8, right_trigger=.29)
        self.assertTrue(actions[self.vr.hand_to_arm['left']]['_xlevr.anchor_command']['active'])
        self.assertFalse(actions[self.vr.hand_to_arm['right']]['_xlevr.anchor_command']['active'])

    def test_trigger_does_not_bypass_stale_pose_guard(self):
        actions = self.tick(left_trigger=1., age=1.)
        command = actions[self.vr.hand_to_arm['left']]['_xlevr.anchor_command']
        self.assertFalse(command['active'])
        np.testing.assert_array_equal(self.controllers['left'].command_pose, self.initial)

    def test_invalid_trigger_does_not_engage_without_side_grip(self):
        for value in (float('nan'), float('inf'), 'invalid'):
            with self.subTest(trigger=value):
                actions = self.tick(left_trigger=value)
                self.assertFalse(self.vr.last_raw['left']['xlevr.enabled'])
                self.assertFalse(actions[self.vr.hand_to_arm['left']]['_xlevr.anchor_command']['active'])

    def test_original_profile_still_requires_side_grip_for_dual_takeover(self):
        for mapper in self.vr.mappers.values(): mapper.squeeze_translation_only = False
        for _ in range(3): actions = self.tick(left_trigger=1., right_trigger=1., input_enabled=True)
        self.assertTrue(all(not raw['xlevr.enabled'] for raw in self.vr.last_raw.values()))
        self.assertTrue(all(not action['_xlevr.anchor_command']['active'] for action in actions.values()))
