"""EE production-loop lifecycle, interpolation and fault handling without hardware."""
import copy
import importlib.util
import json
from pathlib import Path
import runpy
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("_wallx_execution_test", ROOT / "fluxvla/projects/a10/wallx_execution.py")
ex = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ex)
SETTINGS = runpy.run_path(str(ROOT / "configs/a10/a10_wallx_inference.py"))["wallx"]


class Clock:
    def __init__(self):
        self.now = 1.

    def monotonic(self):
        return self.now

    def sleep(self, delay):
        self.now += max(0., delay)


class EELoopTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.geometry = ex.g.A10WallXGeometry(ROOT / "configs/a10/a10_wallx_geometry.json")
        self.contract = ex.simulation_contract()
        self.devices = ex.SimEEDevices(self.geometry, self.clock)
        self.policy = ex.SimPolicy()
        self.settings = copy.deepcopy(SETTINGS)
        self.log = Mock()
        self.timer = patch.object(ex.bridge, "time", self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def run_loop(self, *, read_only=False, cycles=2, cancel=None):
        return ex.run(self.devices, self.policy, self.geometry, self.contract,
                      self.settings, self.log, cycles=cycles, read_only=read_only,
                      clock=self.clock, cancel=cancel)

    def test_two_cycles_reobserve_and_use_one_continuous_ee_session(self):
        result = self.run_loop()
        self.assertTrue(result["completed"])
        self.assertEqual((self.devices.starts, self.devices.stops, self.devices.sent), (1, 1, 36))
        self.assertEqual(self.policy.requests, 2)
        self.assertAlmostEqual(self.devices.poses["left"][0], .412)
        self.assertEqual(self.devices.grippers, dict(left=4500, right=4500))
        self.assertTrue(self.devices.closed and self.policy.closed)
        plans = [c.kwargs for c in self.log.call_args_list if c.args[0] == "plan"]
        self.assertEqual(plans[0]["source_rows"], [0, 1, 2, 3, 4, 5])

    def test_read_only_never_starts_sends_or_moves(self):
        self.run_loop(read_only=True)
        self.assertEqual((self.devices.starts, self.devices.sent), (0, 0))
        self.assertEqual(self.policy.requests, 2)
        self.assertAlmostEqual(self.devices.poses["left"][0], .4)

    def alter_response(self, change):
        original = self.policy.infer
        def infer(payload):
            result = original(payload)
            change(result)
            return result
        self.policy.infer = infer

    def test_late_bad_row_rejects_entire_chunk_before_start(self):
        self.alter_response(lambda r: r["actions"]["right_ee"]["position"].__setitem__((5, 0), .9))
        with self.assertRaisesRegex(ValueError, "step exceeds"):
            self.run_loop()
        self.assertEqual((self.devices.starts, self.devices.sent), (0, 0))
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_nan_or_infinite_prediction_never_executes(self):
        self.alter_response(lambda r: r["actions"]["left_ee"]["rotation_rpy"].__setitem__((5, 2), float("nan")))
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.run_loop()
        self.assertEqual(self.devices.sent, 0)

    def test_public_first_future_row_is_never_dropped_as_internal_rtc_anchor(self):
        self.alter_response(lambda r: r["actions"]["left_ee"]["position"].__setitem__((0, 0), 100.))
        with self.assertRaisesRegex(ValueError, "step exceeds"):
            self.run_loop(cycles=1)
        self.assertEqual(self.devices.sent, 0)

    def test_discard_prediction_if_robot_moves_during_inference(self):
        original = self.policy.infer
        def infer(payload):
            self.clock.sleep(1.)
            self.devices.poses["left"][0] += .03
            return original(payload)
        self.policy.infer = infer
        with self.assertRaisesRegex(ValueError, "moved too far"):
            self.run_loop()
        self.assertEqual(self.devices.starts, 0)

    def test_cancellation_during_inference_does_not_start_follower(self):
        cancel = threading.Event()
        original = self.policy.infer
        def infer(payload):
            cancel.set()
            return original(payload)
        self.policy.infer = infer
        with self.assertRaises(InterruptedError):
            self.run_loop(cancel=cancel)
        self.assertEqual(self.devices.starts, 0)
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_send_failure_stops_and_never_replays_uncertain_frame(self):
        original = self.devices.send_ee
        def send(frame):
            if self.devices.sent == 2:
                raise OSError("uncertain UDP send")
            return original(frame)
        self.devices.send_ee = send
        with self.assertRaises(OSError):
            self.run_loop()
        self.assertEqual((self.devices.sent, self.devices.stops, self.policy.requests), (2, 1, 1))

    def test_late_feedback_aborts_without_catchup_send(self):
        original = self.devices.check_ee
        def check():
            self.clock.sleep(.2)
            return original()
        self.devices.check_ee = check
        with self.assertRaisesRegex(TimeoutError, "deadline missed"):
            self.run_loop()
        self.assertEqual((self.devices.sent, self.devices.stops), (0, 1))

    def test_arrival_timeout_prevents_next_prediction(self):
        original = self.devices.check_ee
        def check():
            result = original()
            if self.devices.sent == 18:
                result["arms"]["left_ee"]["grasp_rig_m_xyzw"][0] += .02
            return result
        self.devices.check_ee = check
        self.settings["arrival_timeout_s"] = .1
        with self.assertRaisesRegex(TimeoutError, "arrival not confirmed"):
            self.run_loop()
        self.assertEqual(self.policy.requests, 1)
        self.assertEqual(self.devices.stops, 1)

    def test_actual_sends_keep_minimum_period(self):
        stamps = []
        original = self.devices.send_ee
        def send(frame):
            stamps.append(self.clock.monotonic())
            return original(frame)
        self.devices.send_ee = send
        self.run_loop(cycles=1)
        self.assertTrue(np.all(np.diff(stamps) >= .0337 / 3 - 1e-12))

    def test_invalid_response_valid_rows_and_unverified_motion_fail_closed(self):
        self.alter_response(lambda response: response.update(valid_rows=0))
        with self.assertRaisesRegex(ValueError, "valid_rows"):
            self.run_loop()
        self.assertEqual(self.devices.sent, 0)
        self.devices = ex.SimEEDevices(self.geometry, self.clock)
        self.devices.simulated = False
        self.policy = ex.SimPolicy()
        with self.assertRaisesRegex(ValueError, "verified model"):
            self.run_loop()
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_response_valid_rows_selects_first_future_segment(self):
        self.alter_response(lambda response: response.update(valid_rows=31))
        self.run_loop(cycles=1)
        plan = next(c.kwargs for c in self.log.call_args_list if c.args[0] == "plan")
        self.assertEqual(plan["source_rows"], [0, 1, 2, 3, 4, 5])
        self.assertAlmostEqual(self.devices.poses["left"][0], .406)

    def test_explicit_service_error_with_actions_never_executes(self):
        self.alter_response(lambda response: response.update(status="overloaded"))
        with self.assertRaisesRegex(ValueError, "rejected"):
            self.run_loop()
        self.assertEqual(self.devices.starts, 0)

    def test_no_numeric_normalization_and_explicit_inverted_gripper(self):
        config = copy.deepcopy(self.contract.config)
        config["model_frame_from_rig"]["left"] = [.1, -.2, .3, *Rotation.from_euler("z", .4).as_quat()]
        config["model_ee_in_grasp_m_xyzw"]["left"] = [0., .02, -.295, *Rotation.from_euler("x", .3).as_quat()]
        config["model_gripper_from_closure"]["left"] = [1.5, 0.]
        contract = ex.ModelContract(config)
        pose = np.r_[.4, .2, .5, Rotation.from_euler("xyz", [.3, .4, -.5]).as_quat()]
        roundtrip = contract.from_model("left", contract.to_model("left", pose))
        np.testing.assert_allclose(ex.g.pose_matrix(roundtrip), ex.g.pose_matrix(pose), atol=1e-12)
        self.assertEqual(contract.encode_gripper("left", .2), 1.2)
        self.assertAlmostEqual(contract.decode_gripper("left", 1.2), .2)

    def test_table_plane_and_sdk_substeps_checked_before_any_send(self):
        for mutate in (lambda: self.settings.update(min_grasp_height_above_table_m=.31),
                       lambda: self.settings.update(max_control_step_m=.0001)):
            with self.subTest(mutation=mutate):
                self.settings = copy.deepcopy(SETTINGS)
                self.devices = ex.SimEEDevices(self.geometry, self.clock)
                self.policy = ex.SimPolicy()
                mutate()
                with self.assertRaises(ValueError):
                    self.run_loop()
                self.assertEqual(self.devices.starts, 0)

    def test_live_adapter_uses_cart_tool_and_preserves_physical_aliases(self):
        device = object.__new__(ex.LiveEEDevices)
        device.geometry, device.cfg = self.geometry, dict(bridge=self.settings)
        device.motion, device.attempted = True, False
        device.lock, device.cancel = threading.Lock(), threading.Event()
        device.cancel.set()
        device.gripper_pause_required, device.error = False, None
        device.feedback = {s: (4500, self.clock.monotonic(), True) for s in ("left", "right")}
        device._speed_restore, device._checked_state = None, None
        device._signature, device._previous_sent = None, None
        sample = self.devices.sample()
        sample["robot_reply"]["ret_context"]["motion_msg"]["jog_coordinate"] = "Tool"
        state = SimpleNamespace(raw=sample["robot_reply"], has_error=False,
            follower_active=True, follower_mode="Cart", received_monotonic_s=self.clock.monotonic(),
            arms={"left": SimpleNamespace(ee_id=0, actual_joints_rad=np.zeros(6)),
                  "right": SimpleNamespace(ee_id=1, actual_joints_rad=np.zeros(6))})
        device.robot = Mock()
        device.robot.get_state.return_value = state
        device.cameras = Mock()
        device.cameras.snapshot.return_value = {n: dict(image=image, received_monotonic_s=self.clock.monotonic())
                                                for n, image in sample["images"].items()}
        device.start()
        device.robot.ensure_tool_coordinate.assert_called_once()
        device.robot.start_follower.assert_called_once_with("cartesian")
        startup = device.robot.begin_sdk_tool_trajectory.call_args.args[0]
        original = self.geometry.observation_geometry(sample["robot_reply"])
        np.testing.assert_allclose(startup["right"], original["arms"]["left_ee"]["controller_tcp_base_m_xyzw"])
        with patch.object(ex, "time", self.clock):
            device.check_ee()
            frame = dict(controller={s: np.array(original["arms"][s + "_ee"]["controller_tcp_base_m_xyzw"])
                                     for s in ("left", "right")},
                         grasp=ex.current_grasps(original), closures=dict(left=.25, right=.75))
            device.send_ee(frame)
        sent = device.robot.send_sdk_tool_pose.call_args.args[0]
        np.testing.assert_allclose(sent["right"], frame["controller"]["left"])
        self.assertEqual((device.target[0][6], device.target[0][13]), (.25, .75))
        device.robot.send_joints.assert_not_called()
        device.robot.ensure_motor_acceleration.assert_not_called()

    def test_primary_failure_and_stop_failure_are_both_preserved(self):
        self.policy.infer = Mock(side_effect=RuntimeError("inference failed"))
        self.devices.close = Mock(side_effect=RuntimeError("stop unconfirmed"))
        with self.assertRaisesRegex(RuntimeError, "inference failed.*stop unconfirmed"):
            self.run_loop()
        self.assertTrue(self.policy.closed)

    def test_native_experiment_sends_each_correct_a10_base_and_tool0(self):
        config = json.loads((ROOT / "configs/a10/a10_wallx_native_experiment.json").read_text())
        contract = ex.ModelContract(config)
        sample = self.devices.sample()
        payload, geom = ex.observation(sample, self.geometry, contract, self.settings)
        for side in ("left", "right"):
            model_pose = payload["observation"]["state"][side + "_ee"]
            original = geom["arms"][side + "_ee"]["controller_tcp_base_m_xyzw"]
            np.testing.assert_allclose(ex.g.pose_matrix(ex.g.rpy_pose(model_pose["position"], model_pose["rotation_rpy"])),
                                       ex.g.pose_matrix(original), atol=1e-12)
            self.assertAlmostEqual(model_pose["gripper"], .763)
        self.assertFalse(contract.verified)


if __name__ == "__main__":
    unittest.main()
