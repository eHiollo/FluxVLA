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
        frames_per_cycle = self.settings["execute_horizon"] * self.settings["interpolation_steps"]
        self.assertEqual((self.devices.starts, self.devices.stops, self.devices.sent), (1, 1, 2 * frames_per_cycle))
        self.assertEqual(self.policy.requests, 2)
        self.assertAlmostEqual(self.devices.poses["left"][0], .412)
        self.assertEqual(self.devices.grippers, dict(left=4500, right=4500))
        self.assertTrue(self.devices.closed and self.policy.closed)
        plans = [c.kwargs for c in self.log.call_args_list if c.args[0] == "plan"]
        self.assertEqual(plans[0]["source_rows"], [0, 1, 2, 3, 4, 5])

    def test_deployment_executes_full_chunks_and_grippers_beyond_trial_limits(self):
        self.settings.update(runpy.run_path(str(ROOT / 'configs/a10/a10_wallx_deploy.py'))['wallx'])
        self.settings['print_progress'] = False
        self.contract.verified = True
        self.devices.simulated = False
        original = self.policy.infer
        def infer(payload):
            self.assertFalse(self.devices.active)
            response = original(payload)
            response['actions']['left_ee']['gripper'][:] = .75
            response['actions']['right_ee']['gripper'][:] = .25
            return response
        self.policy.infer = infer
        result = self.run_loop(cycles=7)
        self.assertTrue(result['completed'])
        self.assertFalse(result['limited_trial'] or result['trial_step'])
        self.assertEqual((self.policy.requests, self.devices.starts, self.devices.stops), (7, 7, 7))
        self.assertAlmostEqual(self.devices.poses['left'][0], .442)
        self.assertEqual(self.devices.grippers, dict(left=6750, right=2250))
        plans = [c.kwargs for c in self.log.call_args_list if c.args[0] == 'plan']
        self.assertTrue(all(p['source_rows'] == list(range(6)) for p in plans))
        self.assertFalse(any(c.args[0] == 'trial_step' for c in self.log.call_args_list))

    def test_deployment_retains_mapping_gate_before_inference(self):
        self.settings.update(runpy.run_path(str(ROOT / 'configs/a10/a10_wallx_deploy.py'))['wallx'])
        self.devices.simulated = False
        with self.assertRaisesRegex(ValueError, 'verified model'):
            self.run_loop(cycles=100)
        self.assertEqual((self.policy.requests, self.devices.starts), (0, 0))
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_deployment_unconfirmed_chunk_stop_blocks_next_request(self):
        self.settings.update(runpy.run_path(str(ROOT / 'configs/a10/a10_wallx_deploy.py'))['wallx'])
        self.settings['print_progress'] = False
        self.devices.stop = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, 'Chunk stop unconfirmed'):
            self.run_loop(cycles=2)
        self.assertEqual(self.policy.requests, 1)
        self.assertTrue(self.policy.closed)

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
            if self.devices.sent == self.settings["execute_horizon"] * self.settings["interpolation_steps"]:
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
        period = max(self.settings["action_dt"] / self.settings["interpolation_steps"],
                     1. / self.settings["max_control_hz"])
        self.assertTrue(np.all(np.diff(stamps) >= period - 1e-12))

    def test_adaptive_interpolation_preserves_every_target_and_gripper_endpoint(self):
        self.settings.update(adaptive_interpolation=True, max_interpolation_steps=600,
                             max_tcp_speed_m_s=.05, max_tcp_rotation_rad_s=.2)
        self.alter_response(lambda r: r["actions"]["left_ee"]["position"].__setitem__((0, 0), .419))
        self.alter_response(lambda r: r["actions"]["left_ee"]["gripper"].__setitem__(slice(None), .75))
        sample = self.devices.sample()
        geom = self.geometry.observation_geometry(sample["robot_reply"])
        payload, _ = ex.observation(sample, self.geometry, self.contract, self.settings)
        response = self.policy.infer(payload)
        chunk = ex.decode_actions(response, self.policy.metadata, self.contract, 6)
        frames = ex.validate_and_interpolate(chunk, geom, geom, sample, self.geometry, self.settings)
        ends = [f for f in frames if f["row_endpoint"]]
        self.assertGreater(len(frames), 36)
        self.assertEqual([f["source_row"] for f in ends], list(range(6)))
        for row, frame in enumerate(ends):
            for side in ("left", "right"):
                self.assertLess(ex.distance(frame["grasp"][side], chunk["poses"][side][row])[0], 1e-12)
                self.assertAlmostEqual(frame["closures"][side], chunk["grippers"][side][row])
        previous = {s: geom["arms"][s + "_ee"]["controller_tcp_base_m_xyzw"] for s in ("left", "right")}
        for frame in frames:
            for side in previous:
                p, r = ex.distance(previous[side], frame["controller"][side])
                self.assertLessEqual(p, self.settings["max_tcp_speed_m_s"] * ex.control_period(self.settings) + 1e-12)
                self.assertLessEqual(r, self.settings["max_tcp_rotation_rad_s"] * ex.control_period(self.settings) + 1e-12)
                previous[side] = frame["controller"][side]

    def test_adaptive_interpolation_refuses_unbounded_frame_count_before_start(self):
        self.settings.update(adaptive_interpolation=True, max_interpolation_steps=6,
                             max_tcp_speed_m_s=.0001, max_tcp_rotation_rad_s=.2)
        with self.assertRaisesRegex(ValueError, "interpolation exceeds"):
            self.run_loop(cycles=1)
        self.assertEqual(self.devices.starts, 0)
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_arrival_log_includes_measured_final_geometry_and_errors(self):
        self.run_loop(cycles=1)
        arrival = next(c.kwargs for c in self.log.call_args_list if c.args[0] == "arrival")
        self.assertAlmostEqual(arrival["geometry"]["arms"]["left_ee"]["grasp_rig_m_xyzw"][0], .406)
        self.assertAlmostEqual(arrival["errors"]["left"]["position_m"], 0.)

    def test_extra_substeps_slow_execution_without_exceeding_vr_rate(self):
        self.settings.update(interpolation_steps=12, max_control_hz=90.)
        stamps = []
        original = self.devices.send_ee
        def send(frame):
            stamps.append(self.clock.monotonic())
            return original(frame)
        self.devices.send_ee = send
        self.run_loop(cycles=1)
        self.assertEqual(len(stamps), 72)
        self.assertTrue(np.all(np.diff(stamps) >= 1. / 90. - 1e-12))
        self.assertGreater(stamps[-1] - stamps[0], 2 * self.settings["execute_horizon"] * self.settings["action_dt"])

    def test_cloud_coordinate_or_anchor_warning_rejects_even_small_actions(self):
        for code in ("state_coordinate_origin_ood", "action_step0_anchor_drift"):
            with self.subTest(code=code):
                self.devices = ex.SimEEDevices(self.geometry, self.clock)
                self.policy = ex.SimPolicy()
                self.alter_response(lambda r: r.update(input_warnings=[dict(code=code)]))
                with self.assertRaisesRegex(ValueError, "unsafe coordinate/anchor"):
                    self.run_loop(cycles=1)
                self.assertEqual(self.devices.starts, 0)
                self.assertTrue(self.devices.closed and self.policy.closed)

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

    def test_explicit_trial_keeps_unverified_status_and_gripper_positions(self):
        self.devices.simulated = False
        self.alter_response(lambda r: r["actions"]["left_ee"]["gripper"].__setitem__(slice(None), .9))
        result = ex.run(self.devices, self.policy, self.geometry, self.contract,
                        self.settings, self.log, cycles=1, read_only=False,
                        limited_trial=True, clock=self.clock)
        self.assertTrue(result["limited_trial"])
        self.assertFalse(self.contract.verified)
        self.assertEqual((self.policy.requests, self.devices.sent), (1, 30))
        self.assertEqual(self.devices.grippers, dict(left=4500, right=4500))
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_explicit_trial_step_bounds_both_arms_and_retains_relative_fraction(self):
        config = ex.trial_settings(self.settings)
        observed = self.geometry.observation_geometry(self.devices.sample()["robot_reply"])
        starts = ex.current_grasps(observed)
        chunk = dict(source_rows=[0], poses={}, grippers={s: np.array([.5]) for s in starts})
        for side, mm, degrees in (("left", 28., 6.), ("right", 26., 2.8)):
            pose = starts[side].copy()
            pose[0] += mm / 1000.
            pose[3:] = (Rotation.from_quat(pose[3:]) * Rotation.from_euler("z", degrees, degrees=True)).as_quat()
            chunk["poses"][side] = np.array([pose])
        bounded, details = ex.trial_step_chunk(chunk, observed, config)
        self.assertAlmostEqual(details["fraction"], 1./6.)
        for side in starts:
            p, r = ex.distance(starts[side], bounded["poses"][side][0])
            self.assertLessEqual(p, .005 + 1e-12)
            self.assertLessEqual(r, np.deg2rad(1) + 1e-12)
        self.assertGreater(ex.distance(starts["left"], chunk["poses"]["left"][0])[0], .015)

    def test_trial_step_still_rejects_large_raw_prediction_and_closes_resources(self):
        self.alter_response(lambda r: r["actions"]["left_ee"]["position"].__setitem__((0, 0), .46))
        with self.assertRaisesRegex(ValueError, "raw prediction too large"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, cycles=1, read_only=False, limited_trial=True, trial_step=True, clock=self.clock)
        self.assertEqual(self.devices.starts, 0)
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_trial_step_actual_loop_holds_gripper_and_confirms_arrival(self):
        self.devices.simulated = False
        self.alter_response(lambda r: r["actions"]["left_ee"]["position"].__setitem__((0, 0), .428))
        result = ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                        self.log, cycles=1, read_only=False, limited_trial=True, trial_step=True, clock=self.clock)
        self.assertTrue(result["trial_step"])
        self.assertAlmostEqual(self.devices.poses["left"][0], .405)
        self.assertEqual(self.devices.grippers, dict(left=4500, right=4500))
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_five_trial_steps_reobserve_after_stop_and_hold_grippers(self):
        original = self.policy.infer
        def infer(payload):
            self.assertFalse(self.devices.active)
            response = original(payload)
            response["actions"]["left_ee"]["position"][0, 0] += .027
            return response
        self.policy.infer = infer
        result = ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                        self.log, cycles=5, read_only=False, limited_trial=True, trial_step=True, clock=self.clock)
        self.assertTrue(result["completed"])
        self.assertEqual((self.policy.requests, self.devices.starts, self.devices.stops, self.devices.sent),
                         (5, 5, 5, 150))
        self.assertAlmostEqual(self.devices.poses["left"][0], .425)
        self.assertEqual(self.devices.grippers, dict(left=4500, right=4500))
        budget = [c.kwargs for c in self.log.call_args_list if c.args[0] == "trial_session_budget"]
        self.assertAlmostEqual(budget[-1]["travelled"]["left"][0], .025)

    def test_trial_session_path_budget_rejects_before_exceeding_total(self):
        sample = self.devices.sample()
        current = self.geometry.observation_geometry(sample["robot_reply"])
        origin = ex.current_grasps(current)
        frame = copy.deepcopy(origin)
        frame["left"][0] += .002
        with self.assertRaisesRegex(ValueError, "session budget"):
            ex.trial_session_budget([dict(grasp=frame)], current, origin,
                                    dict(left=[.024, 0.], right=[0., 0.]))

    def test_trial_step_failed_stop_prevents_second_prediction(self):
        self.devices.stop = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, "stop unconfirmed"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, cycles=2, read_only=False, limited_trial=True, trial_step=True, clock=self.clock)
        self.assertEqual(self.policy.requests, 1)
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_trial_step_cycle_limit_fails_before_cloud_or_motion(self):
        with self.assertRaisesRegex(ValueError, "1..5 cycles"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, cycles=6, read_only=False, limited_trial=True, trial_step=True, clock=self.clock)
        self.assertEqual((self.policy.requests, self.devices.starts), (0, 0))
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_five_mm_trial_cannot_report_arrival_from_unmoved_feedback(self):
        self.settings["arrival_timeout_s"] = .15
        self.alter_response(lambda r: r["actions"]["left_ee"]["position"].__setitem__((0, 0), .428))
        original = self.devices.check_ee
        def check():
            geom = original()
            if self.devices.sent == 30:
                geom["arms"]["left_ee"]["grasp_rig_m_xyzw"][0] = .4
            return geom
        self.devices.check_ee = check
        with self.assertRaisesRegex(TimeoutError, "arrival not confirmed"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, cycles=1, read_only=False, limited_trial=True, trial_step=True, clock=self.clock)
        self.assertTrue(self.devices.closed and self.policy.closed)

    def test_trial_rejects_large_target_and_multiple_cycles_before_start(self):
        self.alter_response(lambda r: r["actions"]["left_ee"]["position"].__setitem__((0, 0), .416))
        with self.assertRaisesRegex(ValueError, "step exceeds"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, cycles=1, read_only=False, limited_trial=True, clock=self.clock)
        self.assertEqual(self.devices.starts, 0)
        self.devices, self.policy = ex.SimEEDevices(self.geometry, self.clock), ex.SimPolicy()
        with self.assertRaisesRegex(ValueError, "exactly one"):
            ex.run(self.devices, self.policy, self.geometry, self.contract, self.settings,
                   self.log, cycles=2, read_only=False, limited_trial=True, clock=self.clock)
        self.assertEqual(self.policy.requests, 0)
        self.assertEqual(self.devices.starts, 0)

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

    def test_old_relay_tool_permission_explains_console_action_without_starting(self):
        device = object.__new__(ex.LiveEEDevices)
        device.motion, device.cfg = True, dict(bridge=self.settings)
        device.lock, device.gripper_pause_required = threading.Lock(), False
        device._speed_restore, device.attempted = None, False
        device.robot = Mock()
        device.robot.ensure_tool_coordinate.side_effect = PermissionError(
            'Arbitrary controller commands require the console role')
        with self.assertRaisesRegex(PermissionError, 'set_jog_coordinate --tool'):
            device.start()
        device.robot.start_follower.assert_not_called()
        device.robot.send_sdk_tool_pose.assert_not_called()
        self.assertFalse(device.attempted)

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
