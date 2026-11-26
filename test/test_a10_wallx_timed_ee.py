"""TimedEe Wall-X lifecycle and frame checks; no robot/cloud connections."""
import copy
import importlib.util
import json
from pathlib import Path
import runpy
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("_wallx_timed_execution_test", ROOT / "fluxvla/projects/a10/wallx_execution.py")
ex = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ex)


class Clock:
    def __init__(self):
        self.now = 1.

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0., seconds)


class TimedWallXTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.geometry = ex.g.A10WallXGeometry(ROOT / "configs/a10/a10_wallx_geometry.json")
        self.contract = ex.simulation_contract()
        self.settings = {}
        for filename in ("a10_wallx_inference.py", "a10_wallx_deploy.py", "a10_wallx_timed_ee.py"):
            self.settings.update(runpy.run_path(str(ROOT / "configs/a10" / filename))["wallx"])
        self.settings["print_progress"] = False
        self.devices = ex.SimTimedEEDevices(self.geometry, self.clock)
        self.policy = ex.SimPolicy()
        self.log = Mock()
        timer = patch.object(ex.bridge, "time", self.clock)
        timer.start()
        self.addCleanup(timer.stop)

    def run_loop(self, cycles=2, read_only=False):
        return ex.run(self.devices, self.policy, self.geometry, self.contract,
                      self.settings, self.log, cycles=cycles, read_only=read_only, clock=self.clock)

    def test_chunks_share_one_task_with_only_last_target_final(self):
        original = self.policy.infer
        def infer(payload):
            if self.policy.requests:
                self.assertTrue(self.devices.active)
                self.assertFalse(self.devices.finished)
            result = original(payload)
            result["actions"]["left_ee"]["gripper"][:] = .75
            result["actions"]["right_ee"]["gripper"][:] = .25
            return result
        self.policy.infer = infer
        self.run_loop(cycles=3)
        self.assertEqual((self.devices.starts, self.devices.stops, self.devices.sent), (1, 1, 18))
        self.assertEqual(self.devices.final_flags, [False] * 17 + [True])
        self.assertTrue(self.devices.finished)
        self.assertEqual(self.devices.grippers, dict(left=6750, right=2250))
        self.assertAlmostEqual(self.devices.poses["left"][0], .418)
        plans = [call.kwargs for call in self.log.call_args_list if call.args[0] == "plan"]
        self.assertTrue(all(len(p["frames"]) == 6 and p["source_rows"] == list(range(6)) for p in plans))
        self.assertEqual(sum(c.args[0] == "arrival" for c in self.log.call_args_list), 1)

    def test_exactly_30hz_once_per_row_and_no_catchup_after_inference_gap(self):
        sends = []
        original_send, original_infer = self.devices.send_ee, self.policy.infer
        def send(frame, **kwargs):
            sends.append(self.clock.monotonic())
            return original_send(frame, **kwargs)
        def infer(payload):
            self.clock.sleep(.5)
            return original_infer(payload)
        self.devices.send_ee, self.policy.infer = send, infer
        self.run_loop()
        deltas = np.diff(sends)
        self.assertTrue(np.all(deltas >= 1. / 30. - 1e-12))
        np.testing.assert_allclose(deltas[:5], 1. / 30., atol=1e-12)
        self.assertGreaterEqual(deltas[5], .5 - 1e-12)

    def test_send_loop_does_not_query_feedback_or_cameras(self):
        original_send = self.devices.send_ee
        def send(frame, **kwargs):
            with patch.object(self.devices, "sample", side_effect=AssertionError("sample during send")), \
                 patch.object(self.devices, "check_ee", side_effect=AssertionError("GET during send")):
                return original_send(frame, **kwargs)
        self.devices.send_ee = send
        self.run_loop()

    def test_bad_last_waypoint_rejected_before_task_start(self):
        original = self.policy.infer
        def infer(payload):
            result = original(payload)
            result["actions"]["left_ee"]["position"][5, 0] += 1.
            return result
        self.policy.infer = infer
        with self.assertRaisesRegex(ValueError, "step exceeds"):
            self.run_loop()
        self.assertEqual((self.devices.starts, self.devices.sent), (0, 0))
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_no_follower_trial_or_implicit_interpolation(self):
        for change in (dict(interpolation_steps=3), dict(adaptive_interpolation=True),
                       dict(stop_between_chunks=True), dict(timed_ee_hz=90.)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                ex.validate_settings(dict(self.settings, **change))
        with self.assertRaisesRegex(ValueError, "does not use follower"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, read_only=False, limited_trial=True, clock=self.clock)

    def test_read_only_never_opens_a_task_or_finishes(self):
        self.run_loop(read_only=True)
        self.assertEqual((self.devices.starts, self.devices.sent), (0, 0))
        self.assertFalse(self.devices.finished)

    def test_live_send_order_units_and_grippers_without_any_get(self):
        device = object.__new__(ex.LiveTimedEEDevices)
        device.lock, device.cancel = threading.Lock(), threading.Event()
        device.gripper_pause_required, device.error = False, None
        device.cfg = dict(bridge=self.settings)
        device.geometry, device.robot = self.geometry, Mock()
        device.feedback = {s: (4500, self.clock.monotonic(), True) for s in ("left", "right")}
        device._robot_state = Mock(side_effect=AssertionError("GET during send"))
        device.sample = Mock(side_effect=AssertionError("camera during send"))
        frame = dict(controller=dict(left=np.array([.1, .2, .3, 0., 0., 0., 1.]),
                                     right=np.array([.4, .5, .6, 0., 0., 0., 1.])),
                     grasp=copy.deepcopy(self.devices.poses), closures=dict(left=.75, right=.25))
        with patch.object(ex.time, "monotonic", self.clock.monotonic):
            device.send_ee(frame, final=True)
        args, kwargs = device.robot.send_timed_ee_target.call_args
        np.testing.assert_array_equal(args[0][0], frame["controller"]["right"])
        np.testing.assert_array_equal(args[0][1], frame["controller"]["left"])
        self.assertEqual(kwargs, dict(final=True))
        self.assertEqual((device.target[0][6], device.target[0][13]), (.75, .25))

    def test_ui_tool1_observation_sends_tool1_without_295mm_offset(self):
        sample = self.devices.sample()
        context = sample["robot_reply"]["ret_context"]
        context["robot_msg"]["tool"] = [1, 1]
        for side in ("left", "right"):
            p = self.geometry.controller_pose_from_grasp(side, self.devices.poses[side], 1)
            p[:3] *= 1000
            context["motion_msg"]["pq"][self.geometry.config["arms"][side]["ee_id"]] = p.tolist()
        payload, observed = ex.observation(sample, self.geometry, self.contract, self.settings)
        chunk = ex.decode_actions(self.policy.infer(payload), self.policy.metadata, self.contract, 6)
        frames = ex.validate_timed_ee_chunk(chunk, observed, observed, sample, self.geometry, self.settings)
        for side in ("left", "right"):
            expected = self.geometry.controller_pose_from_grasp(side, frames[0]["grasp"][side], 1)
            wrong = self.geometry.controller_pose_from_grasp(side, frames[0]["grasp"][side], 0)
            np.testing.assert_allclose(frames[0]["controller"][side], expected, atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(expected[:3] - wrong[:3]), .295)

    def test_real_model_tool0_to_sdk_tool1_applies_offset_once_for_either_ui_tool(self):
        contract = ex.ModelContract(json.loads((ROOT / 'configs/a10/a10_wallx_forward_experiment.json').read_text()))
        for selected_tool in (0, 1):
            with self.subTest(ui_tool=selected_tool):
                sample = self.devices.sample()
                context = sample['robot_reply']['ret_context']
                context['robot_msg']['tool'] = [selected_tool, selected_tool]
                for side in ('left', 'right'):
                    pose = self.geometry.controller_pose_from_grasp(side, self.devices.poses[side], selected_tool)
                    pose[:3] *= 1000.
                    context['motion_msg']['pq'][self.geometry.config['arms'][side]['ee_id']] = pose.tolist()
                payload, observed = ex.observation(sample, self.geometry, contract, self.settings)
                response = ex.SimPolicy().infer(payload)
                chunk = ex.decode_actions(response, ex.SimPolicy.metadata, contract, 6)
                frames = ex.validate_timed_ee_chunk(chunk, observed, observed, sample, self.geometry, self.settings)
                for side in ('left', 'right'):
                    self.assertAlmostEqual(payload['observation']['state'][side + '_ee']['position'][2], .795)
                    tcp = frames[0]['controller'][side].copy(); tcp[:3] *= 1000.
                    recovered = self.geometry.grasp_in_rig(side, tcp, 1)
                    expected = self.devices.poses[side].copy(); expected[0] += .001
                    np.testing.assert_allclose(recovered[:3], expected[:3], atol=1e-12)
                    self.assertLess(ex.distance(recovered, expected)[1], 1e-12)

    def test_sdk_selecting_ui_tool1_at_start_does_not_discard_valid_tool1_targets(self):
        sample = self.devices.sample
        def selected_sample():
            result = sample()
            if self.devices.active:
                context = result['robot_reply']['ret_context']
                context['robot_msg']['tool'] = [1, 1]
                for side in ('left', 'right'):
                    pose = self.geometry.controller_pose_from_grasp(side, self.devices.poses[side], 1)
                    pose[:3] *= 1000.
                    context['motion_msg']['pq'][self.geometry.config['arms'][side]['ee_id']] = pose.tolist()
            return result
        self.devices.sample = selected_sample
        self.run_loop()
        self.assertEqual((self.devices.starts, self.devices.sent, self.devices.stops), (1, 12, 1))

    def test_finish_failure_still_closes_devices_and_policy(self):
        self.devices.finish_ee = Mock(side_effect=TimeoutError("command convergence timeout"))
        with self.assertRaisesRegex(TimeoutError, "command convergence"):
            self.run_loop()
        self.assertTrue(self.devices.closed and self.policy.closed)
        self.assertEqual(self.devices.stops, 1)

    def test_real_motion_keeps_verified_mapping_gate(self):
        self.devices.simulated = False
        with self.assertRaisesRegex(ValueError, "verified model"):
            self.run_loop()
        self.assertEqual((self.policy.requests, self.devices.starts, self.devices.sent), (0, 0, 0))

    def test_live_start_does_not_change_tool_or_start_follower(self):
        device = object.__new__(ex.LiveTimedEEDevices)
        device.motion = True
        device.lock, device.cancel = threading.Lock(), threading.Event()
        device.gripper_pause_required, device.error = False, None
        device.cfg, device.robot, device.log = dict(bridge=self.settings), Mock(), Mock()
        device.robot.start_timed_ee.return_value = dict(stream_id=9,
            command_pose_m_quat=[[.1, .2, .3, 0., 0., 0., 1.]] * 2)
        result = device.start()
        self.assertEqual(result["stream_id"], 9)
        device.robot.start_follower.assert_not_called()
        device.robot.ensure_tool_coordinate.assert_not_called()
        device.robot.begin_sdk_tool_trajectory.assert_not_called()
        self.assertTrue(device.attempted)

    def test_rejected_live_start_does_not_mask_primary_error_during_stop(self):
        device = object.__new__(ex.LiveTimedEEDevices)
        device.motion = True
        device.lock, device.cancel = threading.Lock(), threading.Event()
        device.gripper_pause_required, device.error = False, None
        device.cfg, device.robot, device.log = dict(bridge=self.settings), Mock(), Mock()
        device.write_locks = {}; device.robot.timed_ee_start_attempted = False
        device.robot.start_timed_ee.side_effect = RuntimeError('primary rejection')
        with self.assertRaisesRegex(RuntimeError, 'primary rejection'): device.start()
        self.assertFalse(device.attempted)
        self.assertTrue(device.stop())
        device.robot.stop_timed_ee.assert_not_called()

    def test_cancel_during_pacing_sleep_sends_no_extra_point(self):
        cancel = threading.Event()
        original_sleep = self.clock.sleep
        def sleep(seconds):
            original_sleep(seconds)
            if seconds > 0 and self.devices.sent == 1:
                cancel.set()
        self.clock.sleep = sleep
        with self.assertRaisesRegex(InterruptedError, "waiting to send"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, read_only=False, clock=self.clock, cancel=cancel)
        self.assertEqual(self.devices.sent, 1)
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_startup_motion_discards_already_planned_chunk(self):
        original = self.devices.start
        def start():
            original()
            self.devices.poses["left"][0] += .03
        self.devices.start = start
        with self.assertRaisesRegex(ValueError, "before timed_ee first send"):
            self.run_loop()
        self.assertEqual((self.devices.starts, self.devices.sent, self.devices.stops), (1, 0, 1))

    def test_get_tcp_arrival_does_not_claim_measured_fk(self):
        result = self.run_loop()
        arrivals = [c.kwargs for c in self.log.call_args_list if c.args[0] == "arrival"]
        self.assertFalse(result["measured_ee_accuracy_verified"])
        self.assertFalse(arrivals[0]["tcp_pose_measured_fk_verified"])
        self.assertEqual(arrivals[0]["confirmation_scope"], "GET_tcp_pose_only")

    def test_fast_v2_arrival_band_applies_at_tool1_not_flange(self):
        targets = {s: p.copy() for s, p in self.devices.poses.items()}
        for side in ("left", "right"):
            tcp = self.geometry.controller_pose_from_grasp(side, targets[side], 1)
            tcp[0] += .0018
            rotation = Rotation.from_rotvec([0., np.deg2rad(.18), 0.]) * Rotation.from_quat(tcp[3:])
            tcp[3:] = rotation.as_quat()
            tcp[:3] *= 1000.
            self.devices.poses[side] = self.geometry.grasp_in_rig(side, tcp, 1)
        self.devices.active = True
        # Flange residual exceeds 2 mm even though the declared tool1 band passes.
        self.assertTrue(any(ex.distance(self.geometry.controller_pose_from_grasp(s, targets[s], 0),
                                       self.geometry.controller_pose_from_grasp(s, self.devices.poses[s], 0))[0] > .002 for s in targets))
        result = ex.wait_arrival(self.devices, targets, self.settings, self.clock, tcp_tool=1)
        self.assertEqual(result["errors_frame"], "tool1_wobj0")
        for error in result["errors"].values():
            self.assertAlmostEqual(error["position_m"], .0018)
            self.assertAlmostEqual(error["rotation_rad"], np.deg2rad(.18))


if __name__ == "__main__":
    unittest.main()
