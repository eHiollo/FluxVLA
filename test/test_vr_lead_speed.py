"""Reference lead and one-run speed override: offline / loopback only."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from mmengine import Config

from fluxvla.engines.operators.vr.control import ReferenceConfig, ReferenceGovernor, VRPoseController
from fluxvla.engines.operators.vr.runtime import build_runtime
from test_a10_operator import FakeController
from test_vr_reference_runtime import runtime, sample
from test_vr_control import command, IDENTITY


class LeadTest(unittest.TestCase):
    def test_replay_skips_speed_events_and_accepts_lead_options(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'input.jsonl'
            source.write_text('\n'.join(json.dumps(r) for r in (
                dict(event='controller_speed_override', controller_speed_override={'state': 'confirmed'}),
                dict(mapped_action={'_xlevr.anchor_command': command()}, sample_age_s=0., sample_receive_time_s=1.),
                dict(mapped_action={'_xlevr.anchor_command': command(2)}, sample_age_s=0., sample_receive_time_s=1.033))))
            result = subprocess.run([sys.executable, 'fluxvla/engines/operators/vr/control.py',
                                     '--input', str(source), '--feedback-source', 'synthetic-hold',
                                     '--max-reference-lead-m', '.02', '--max-reference-lead-rad', '.12'],
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)['frames'], 2)

    def test_stalled_robot_stays_bounded_and_reverse_can_return(self):
        cfg = ReferenceConfig(max_reference_lead_m=.02, max_reference_lead_rad=.12)
        gov = ReferenceGovernor(cfg)
        gov.engage(IDENTITY)
        target = np.r_[[1., 0., 0.], Rotation.from_euler('z', 1.).as_quat()]
        for _ in range(200):
            before = gov.reference.copy()
            self.assertEqual(gov.step(target, IDENTITY, 1/30), 'tracking')
            self.assertLessEqual(np.linalg.norm(gov.reference[:3]), .02 + 1e-10)
            self.assertLessEqual(Rotation.from_quat(gov.reference[3:]).magnitude(), .12 + 1e-10)
            self.assertLessEqual(np.linalg.norm(gov.reference[:3]-before[:3]), .12/30 + 1e-10)
        before = gov.reference.copy()
        gov.step(IDENTITY, IDENTITY, 1/30)
        self.assertLess(gov.reference[0], before[0])
        self.assertEqual(gov.position_progress_scale, 1.)
        self.assertEqual(gov.rotation_progress_scale, 1.)

    def test_slow_robot_has_no_fault_and_command_acceleration_stays_continuous(self):
        ctrl = VRPoseController(ReferenceConfig(max_reference_lead_m=.02, max_reference_lead_rad=.12))
        ctrl.start(IDENTITY)
        actual = IDENTITY.copy()
        ctrl.step(command(), actual, 1/30)
        velocity = np.zeros(3)
        for i in range(1, 300):
            actual[0] += np.clip(ctrl.command_pose[0]-actual[0], -.02/30, .02/30)
            result = ctrl.step(command(i+1, offset=(.3, 0, 0, 0, 0, 0)), actual, 1/30)
            self.assertFalse(result['stop_requested'])
            self.assertLessEqual(np.linalg.norm(ctrl.governor.reference[:3]-actual[:3]), .02+1e-10)
            self.assertLessEqual(np.linalg.norm(ctrl.linear_velocity-velocity), .4/30+1e-10)
            self.assertLess(result['command_tracking_error_m'], .05)
            velocity = ctrl.linear_velocity.copy()
        self.assertGreater(actual[0], .1)

    def test_feedback_move_outside_budget_does_not_snap_reference(self):
        gov = ReferenceGovernor(ReferenceConfig(max_reference_lead_m=.02))
        gov.engage(IDENTITY)
        actual = IDENTITY.copy(); actual[0] = -.03
        target = IDENTITY.copy(); target[0] = .2
        gov.step(target, actual, 1/30)
        np.testing.assert_allclose(gov.reference, IDENTITY)
        gov.step(actual, actual, 1/30)
        self.assertAlmostEqual(gov.reference[0], -.004)
        actual[0] = -.2
        for _ in range(4):
            gov.step(target, actual, 1/30)
        self.assertEqual(gov.state, 'fault')

    def test_invalid_lead_and_stale_feedback_still_fail(self):
        for value in (-1, 0, True, float('nan'), .05):
            with self.assertRaises(ValueError):
                ReferenceConfig(max_reference_lead_m=value)
        ctrl = VRPoseController(ReferenceConfig(max_reference_lead_m=.02))
        ctrl.start(IDENTITY)
        result = ctrl.step(command(), IDENTITY, 1/30, feedback_age_s=.3)
        self.assertEqual(result['fault_reason'], 'feedback_stale')

    def test_command_inertia_still_trips_existing_hard_guard(self):
        ctrl = VRPoseController(ReferenceConfig(max_reference_lead_m=.02))
        ctrl.start(IDENTITY)
        ctrl.step(command(), IDENTITY, 1/30)
        # Command can carry velocity even when reference is waiting. Do not
        # falsely treat the soft reference bound as a bound on the final command.
        ctrl.command_pose[0] = .049
        ctrl.linear_velocity[0] = .12
        result = ctrl.step(command(2, offset=(.2, 0, 0, 0, 0, 0)), IDENTITY, 1/30)
        self.assertEqual(result['fault_reason'], 'command_tracking_error')
        self.assertGreater(result['command_tracking_error_m'], .05)
        self.assertIsNone(result['relative_target'])


class SpeedTest(unittest.TestCase):
    def test_manual_auto_protocol_and_motion_gate(self):
        with FakeController() as server:
            robot = server.operator()
            self.assertEqual(robot.get_speed_override(), dict(mode='Manual', percent=10))
            robot.set_speed_override(25, expected_mode='Manual')
            self.assertIn('set_jog_vel --vel_percent=25', server.commands)
            server.op_mode = 'Auto'
            robot.set_speed_override(30, expected_mode='Auto')
            self.assertIn('set_pgm_vel --vel_percent=30', server.commands)
            for field, value in [('active', True), ('motion', 'Running'), ('op_mode', 'Manual')]:
                old = getattr(server, field); setattr(server, field, value)
                before = sum(c.startswith('set_') for c in server.commands)
                with self.assertRaises(RuntimeError):
                    robot.set_speed_override(40, expected_mode='Auto')
                self.assertEqual(sum(c.startswith('set_') for c in server.commands), before)
                setattr(server, field, old)
            robot.allow_motion = False
            with self.assertRaises(PermissionError):
                robot.set_speed_override(40, expected_mode='Auto')

    def test_ack_without_readback_and_rejection_prevent_start(self):
        for reject in (True, False):
            with self.subTest(reject=reject), FakeController() as server:
                server.reject_speed = reject
                server.speed_change = False
                robot = server.operator(feedback_position_unit='mm', timeout_s=.08)
                rt, _ = runtime([sample(1)], robot=robot, controller_speed_percent=25)
                with self.assertRaises(Exception):
                    rt.run(max_steps=1)
                self.assertNotIn('follower_cart', server.commands)
                self.assertFalse(robot.is_connected)

    def test_runtime_sets_once_restores_and_records_exit(self):
        with FakeController() as server, tempfile.TemporaryDirectory() as directory:
            robot = server.operator(feedback_position_unit='mm')
            path = Path(directory)/'diagnostics.jsonl'
            rt, _ = runtime([], robot=robot, controller_speed_percent=25, diagnostics_path=path)
            rt.connect()
            self.assertEqual(server.jog_vel, 25)
            rt.close(); rt.close()
            self.assertEqual(server.jog_vel, 10)
            self.assertEqual(server.commands.count('set_jog_vel --vel_percent=10'), 1)
            events = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(events[-1]['controller_speed_override']['state'], 'restored')

    def test_failed_stop_or_disconnect_does_not_restore_blindly(self):
        for disconnected in (False, True):
            with self.subTest(disconnected=disconnected), FakeController() as server:
                robot = server.operator(feedback_position_unit='mm')
                rt, _ = runtime([], robot=robot, controller_speed_percent=25)
                rt.connect()
                if disconnected:
                    robot.close()
                else:
                    server.active = True; server.motion = 'Running'
                rt.close()
                self.assertEqual(server.jog_vel, 25)
                self.assertEqual(rt.speed_status['state'], 'restore_unconfirmed')
                self.assertNotIn('set_jog_vel --vel_percent=10', server.commands)

    def test_external_speed_change_is_preserved(self):
        with FakeController() as server:
            robot = server.operator(feedback_position_unit='mm')
            rt, _ = runtime([], robot=robot, controller_speed_percent=25)
            rt.connect()
            server.jog_vel = 15
            rt.close()
            self.assertEqual(server.jog_vel, 15)
            self.assertEqual(rt.speed_status['state'], 'restore_unconfirmed')

    def test_missing_feedback_or_fault_blocks_percentage_write(self):
        with FakeController() as server:
            robot = server.operator()
            server.motion_override = {'jog_vel': None}
            with self.assertRaises(Exception):
                robot.get_speed_override()
            server.motion_override = None
            server.driver_fault = True
            with self.assertRaises(RuntimeError):
                robot.set_speed_override(25, expected_mode='Manual')
            self.assertFalse(any(c.startswith('set_') for c in server.commands))

    def test_observe_and_preview_do_not_write(self):
        rt, robot = runtime([], observe=True, controller_speed_percent=25)
        with patch.object(robot, 'get_speed_override', create=True) as getter:
            rt.connect(); rt.close()
            getter.assert_not_called()
        cfg = Config.fromfile('configs/a10/a10_vr_teleop.py').teleop
        cfg.controller_speed_percent = 25
        rt = build_runtime(cfg)
        self.assertFalse(rt._connected)
        self.assertIsNone(rt.robot)
        for value in (0, 101, True, 10.5):
            cfg.controller_speed_percent = value
            with self.assertRaises(ValueError):
                build_runtime(cfg)


if __name__ == '__main__':
    unittest.main()
