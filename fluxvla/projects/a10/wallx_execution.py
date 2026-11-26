"""Observe/infer/execute loop for dual A10 EE actions, without local model imports."""
from __future__ import annotations

from copy import deepcopy
import importlib.util
import io
import math
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[3]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


g = load("_a10_wallx_geometry", Path(__file__).with_name("wallx.py"))
bridge = load("_a10_wallx_bridge", Path(__file__).with_name("inference.py"))


def distance(a, b):
    return (float(np.linalg.norm(np.asarray(a[:3]) - b[:3])),
            float((Rotation.from_quat(a[3:]).inv() * Rotation.from_quat(b[3:])).magnitude()))


def finite_positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")


def validate_settings(settings):
    mode = settings.get("ee_action_mode", "absolute")
    if mode not in ("absolute", "delta"):
        raise ValueError("ee_action_mode must be absolute or delta")
    if mode == "delta":
        for key in ("delta_translation_scale", "delta_rotation_scale"):
            value = settings.get(key, 1.)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
    backend = settings.get("backend", "follower")
    if backend not in ("follower", "timed_ee"):
        raise ValueError("Wall-X backend must be follower or timed_ee")
    if backend == "timed_ee":
        if settings.get("stop_between_chunks", False):
            raise ValueError("timed_ee requires stop_between_chunks=False")
        if settings.get("adaptive_interpolation", False) or settings["interpolation_steps"] != 1:
            raise ValueError("timed_ee sends each original row once; disable interpolation")
        if not math.isclose(settings.get("timed_ee_hz", 30.), 30., abs_tol=1e-9):
            raise ValueError("timed_ee requires 30 Hz")
        if abs(settings["action_dt"] - 1. / 30.) > .002:
            raise ValueError("Model row duration is incompatible with timed_ee 30 Hz")
    stable = settings["arrival_stable_s"]
    if not (backend == "timed_ee" and type(stable) in (int, float) and stable == 0):
        finite_positive(stable, "arrival_stable_s")
    finite_positive(settings.get("max_control_hz", 90.), "max_control_hz")
    if "adaptive_interpolation" in settings and type(settings["adaptive_interpolation"]) is not bool:
        raise ValueError("adaptive_interpolation must be boolean")
    for key in ("stop_between_chunks", "print_progress"):
        if key in settings and type(settings[key]) is not bool:
            raise ValueError(f"{key} must be boolean")
    if settings.get("adaptive_interpolation", False):
        finite_positive(settings["max_tcp_speed_m_s"], "max_tcp_speed_m_s")
        finite_positive(settings["max_tcp_rotation_rad_s"], "max_tcp_rotation_rad_s")
        if type(settings["max_interpolation_steps"]) is not int or settings["max_interpolation_steps"] < settings["interpolation_steps"]:
            raise ValueError("max_interpolation_steps must be an integer >= interpolation_steps")
    for key in ("execute_horizon", "interpolation_steps", "feedback_every_n_steps"):
        if type(settings[key]) is not int or settings[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if settings["execute_horizon"] > 32:
        raise ValueError("execute_horizon must be <=32 public future rows")
    for key in ("action_dt", "max_ee_step_m", "max_ee_step_rad", "max_control_step_m",
                "max_control_step_rad", "max_chunk_displacement_m", "max_chunk_rotation_rad",
                "max_observation_drift_m", "max_observation_drift_rad", "max_tracking_error_m",
                "max_tracking_error_rad", "max_send_lag_s", "arrival_timeout_s",
                "arrival_position_tolerance_m", "arrival_rotation_tolerance_rad",
                "max_robot_age_s", "max_camera_age_s", "max_gripper_age_s", "max_skew_s",
                "min_grasp_height_above_table_m", "min_grasp_separation_m"):
        finite_positive(settings[key], key)


class ModelContract:
    """Explicit physical-unit mapping; normalization remains on the API server.

    transforms are model_from_rig (one per physical arm). Full tool transforms map
    model EE origins/axes into the calibrated A10 grasp frame. No identity or gripper
    direction is inferred from a robot/model name.
    """
    def __init__(self, config):
        self.config = deepcopy(config)
        self.embodiment = config.get("embodiment")
        if not isinstance(self.embodiment, str) or not self.embodiment:
            raise ValueError("Configure model_contract.embodiment before cloud inference")
        self.verified = config.get("verified") is True
        self.frames, self.tools, self.grips = {}, {}, {}
        for side in ("left", "right"):
            self.frames[side] = g.pose_matrix(config["model_frame_from_rig"][side])
            self.tools[side] = g.pose_matrix(config["model_ee_in_grasp_m_xyzw"][side])
            endpoints = g.vector(config["model_gripper_from_closure"][side], 2, "model open/closed endpoints")
            if endpoints[0] == endpoints[1]:
                raise ValueError("Model gripper open and closed endpoints must differ")
            self.grips[side] = endpoints

    def to_model(self, side, grasp):
        return g.matrix_pose(self.frames[side] @ g.pose_matrix(grasp) @ self.tools[side])

    def from_model(self, side, pose):
        return g.matrix_pose(np.linalg.inv(self.frames[side]) @ g.pose_matrix(pose)
                             @ np.linalg.inv(self.tools[side]))

    def encode_gripper(self, side, closure):
        if not math.isfinite(closure) or not 0 <= closure <= 1:
            raise ValueError("A10 closure must be within 0..1")
        opened, closed = self.grips[side]
        return float(opened + closure * (closed - opened))

    def decode_gripper(self, side, value):
        opened, closed = self.grips[side]
        closure = float((value - opened) / (closed - opened))
        if not math.isfinite(closure) or not 0 <= closure <= 1:
            raise ValueError("Predicted model gripper is outside the configured open/closed endpoints")
        return closure


def observation(sample, geometry, contract, settings):
    bridge.observation(sample, settings)  # Existing real timestamps, camera and gripper freshness checks.
    geom = geometry.observation_geometry(sample["robot_reply"])
    states = {}
    for side in ("left", "right"):
        pose = contract.to_model(side, geom["arms"][side + "_ee"]["grasp_rig_m_xyzw"])
        states[side + "_ee"] = dict(**g.public_pose(pose), gripper=contract.encode_gripper(
            side, geometry.closure_fraction(sample["grippers"][side])))
    names = dict(front="base_0_rgb", left_wrist="left_wrist_0_rgb", right_wrist="right_wrist_0_rgb")
    images = {}
    for public, local in names.items():
        buf = io.BytesIO()
        Image.fromarray(sample["images"][local]).save(buf, format="JPEG", quality=90)
        images[public] = buf.getvalue()
    return dict(embodiment=contract.embodiment, normalization="provided", instruction=settings["prompt"],
                observation=dict(images=images, state=states)), geom


def future_rows(metadata, horizon, response=None):
    n = metadata.get("action_horizon")
    if n != 32:
        raise ValueError("Wall-X EE loop requires a 32-row action horizon")
    # Public-v2 responses begin with a FUTURE action. The handshake also exposes
    # internal RTC metadata whose row-zero anchor is NOT part of this wire schema.
    # Match the official action_public.decode_response: response.valid_rows,
    # then response.rtc.valid_rows, otherwise all returned rows; never drop row 0.
    response = response or {}
    valid = response.get("valid_rows")
    if valid is None and isinstance(response.get("rtc"), dict):
        valid = response["rtc"].get("valid_rows")
    if valid is None:
        valid = n
    if type(valid) is not int or not 1 <= valid <= n:
        raise ValueError("Invalid public-v2 response valid_rows")
    if horizon > valid:
        raise ValueError("execute_horizon exceeds valid future rows")
    return range(horizon)


def decode_actions(response, metadata, contract, horizon):
    if response.get("error") or ("status" in response and response["status"] != "ok"):
        raise ValueError("Wall-X service rejected the inference request")
    # Production returns these even when the wire response itself is successful.
    unsafe = {"state_coordinate_origin_ood", "action_step0_anchor_drift"}
    for warning in response.get("input_warnings", []):
        if isinstance(warning, dict) and warning.get("code") in unsafe:
            raise ValueError(f"Wall-X unsafe coordinate/anchor warning: {warning['code']}")
    if response.get("embodiment") != contract.embodiment or set(response.get("actions", {})) != {"left_ee", "right_ee"}:
        raise ValueError("Expected matching embodiment and both EE action blocks")
    indices = future_rows(metadata, horizon, response)
    arms, grips = {}, {}
    for side in ("left", "right"):
        block = response["actions"][side + "_ee"]
        for field, shape in (("position", (32, 3)), ("rotation_rpy", (32, 3)), ("gripper", (32,))):
            array = np.asarray(block[field])
            if array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
                raise ValueError(f"Invalid {side}.{field} shape or nonfinite actions")
        arms[side] = np.array([contract.from_model(side, g.rpy_pose(block["position"][i], block["rotation_rpy"][i]))
                               for i in indices])
        grips[side] = np.array([contract.decode_gripper(side, block["gripper"][i]) for i in indices])
    return dict(poses=arms, grippers=grips, source_rows=list(indices))


def current_grasps(geom):
    return {side: np.array(geom["arms"][side + "_ee"]["grasp_rig_m_xyzw"])
            for side in ("left", "right")}


def reference_actions(chunk, observed, current, contract, settings):
    """Optional relative-only retargeting, not API denormalization.

    Subtract the first FUTURE prediction, not the observed input pose. The
    first future action is deliberately replaced with a hold; this discards
    its intended motion together with any absolute prediction bias. Translation
    deltas and relative rotations are expressed in model world axes at model EE,
    then transformed once to the robot's grasp/TCP by the existing contract.
    """
    if settings.get("ee_action_mode", "absolute") == "absolute":
        return chunk, None
    if not chunk["source_rows"] or chunk["source_rows"][0] != 0:
        raise ValueError("Delta retargeting requires public future row 0 as its explicit reference")
    p_scale = settings.get("delta_translation_scale", 1.)
    r_scale = settings.get("delta_rotation_scale", 1.)
    result = deepcopy(chunk)
    info = dict(mode="delta", reference="first_future_prediction", first_future_motion_discarded=True,
                delta_frame="model_world", pivot="model_ee", translation_scale=p_scale,
                rotation_scale=r_scale, gripper_reference="absolute_model_prediction", arms={})
    for side in ("left", "right"):
        predicted = [contract.to_model(side, p) for p in chunk["poses"][side]]
        first = predicted[0]
        robot = contract.to_model(side, current["arms"][side + "_ee"]["grasp_rig_m_xyzw"])
        input_pose = contract.to_model(side, observed["arms"][side + "_ee"]["grasp_rig_m_xyzw"])
        first_r, robot_r = Rotation.from_quat(first[3:]), Rotation.from_quat(robot[3:])
        for index, pose in enumerate(predicted):
            delta_r = Rotation.from_quat(pose[3:]) * first_r.inv()
            target_r = Rotation.from_rotvec(delta_r.as_rotvec() * r_scale) * robot_r
            target = np.r_[robot[:3] + (pose[:3] - first[:3]) * p_scale, target_r.as_quat()]
            result["poses"][side][index] = contract.from_model(side, target)
        info["arms"][side] = dict(
            raw_first_model_m_xyzw=first, robot_start_model_m_xyzw=robot,
            raw_first_minus_input_m=first[:3] - input_pose[:3],
            raw_first_rotation_from_input_rad=float((first_r * Rotation.from_quat(input_pose[3:]).inv()).magnitude()),
            raw_end_minus_first_m=predicted[-1][:3] - first[:3],
            target_end_grasp_rig_m_xyzw=result["poses"][side][-1])
    return result, info


def tool_signature(geom):
    return tuple((geom["arms"][s + "_ee"]["ee_id"], geom["arms"][s + "_ee"]["active_tool"],
                  geom["arms"][s + "_ee"]["active_wobj"]) for s in ("left", "right"))


def timed_ee_signature(geom):
    # Known UI tools both recover the same grasp; the SDK selects tool1 itself.
    return tuple((geom["arms"][s + "_ee"]["ee_id"], geom["arms"][s + "_ee"]["active_wobj"])
                 for s in ("left", "right"))


def control_period(settings):
    return max(settings["action_dt"] / settings["interpolation_steps"],
               1. / settings.get("max_control_hz", 90.))


def validate_and_interpolate(chunk, observed, current, sample, geometry, settings):
    """Validate the entire selected chunk and every SDK substep before any send."""
    if tool_signature(observed) != tool_signature(current):
        raise ValueError("Tool/wobj binding changed while inference was in flight")
    starts = current_grasps(current)
    for side in starts:
        a = observed["arms"][side + "_ee"]["grasp_rig_m_xyzw"]
        delta = distance(a, starts[side])
        if delta[0] > settings["max_observation_drift_m"] or delta[1] > settings["max_observation_drift_rad"]:
            raise ValueError("Robot moved too far while inference was in flight; discard old prediction")
    previous = {s: p.copy() for s, p in starts.items()}
    previous_grips = {s: geometry.closure_fraction(sample["grippers"][s]) for s in starts}
    previous_controller = {s: np.array(current["arms"][s + "_ee"]["controller_tcp_base_m_xyzw"]) for s in starts}
    frames = []
    for row in range(len(chunk["source_rows"])):
        subdivisions = settings["interpolation_steps"]
        targets = {s: chunk["poses"][s][row] for s in starts}
        for side, target in targets.items():
            delta, total = distance(previous[side], target), distance(starts[side], target)
            if delta[0] > settings["max_ee_step_m"] or delta[1] > settings["max_ee_step_rad"]:
                raise ValueError(
                    f"EE model step exceeds limit: physical {side}, source row {chunk['source_rows'][row]}, "
                    f"{delta[0]*1000:.2f} mm / {math.degrees(delta[1]):.2f} deg; "
                    f"limits {settings['max_ee_step_m']*1000:.2f} mm / "
                    f"{math.degrees(settings['max_ee_step_rad']):.2f} deg; entire chunk rejected")
            if total[0] > settings["max_chunk_displacement_m"] or total[1] > settings["max_chunk_rotation_rad"]:
                raise ValueError("EE chunk displacement exceeds limit; entire chunk rejected")
            if settings.get("adaptive_interpolation", False):
                period = control_period(settings)
                p_limit = min(settings["max_control_step_m"], settings["max_tcp_speed_m_s"] * period)
                r_limit = min(settings["max_control_step_rad"], settings["max_tcp_rotation_rad_s"] * period)
                tool = current["arms"][side + "_ee"]["active_tool"]
                radius = np.linalg.norm(geometry.active_tool_to_grasp(tool)[:3, 3])
                # Bound the tool TCP arc as well as the grasp translation.
                # Extra frames extend duration while preserving each target.
                subdivisions = max(subdivisions, math.ceil((delta[0] + radius * delta[1]) / p_limit),
                                   math.ceil(delta[1] / r_limit))
                if subdivisions > settings["max_interpolation_steps"]:
                    raise ValueError("Required interpolation exceeds configured maximum; entire chunk rejected")
        interpolators = {s: Slerp([0., 1.], Rotation.from_quat([previous[s][3:], targets[s][3:]])) for s in starts}
        for j in range(1, subdivisions + 1):
            alpha = j / subdivisions
            rig = {s: np.r_[previous[s][:3] * (1 - alpha) + targets[s][:3] * alpha,
                             interpolators[s]([alpha]).as_quat()[0]] for s in starts}
            floor = geometry.config["table"]["height_above_base_mount_m"] + settings["min_grasp_height_above_table_m"]
            if any(p[2] < floor for p in rig.values()):
                raise ValueError("Grasp target crosses the table plane; entire chunk rejected")
            if np.linalg.norm(rig["left"][:3] - rig["right"][:3]) < settings["min_grasp_separation_m"]:
                raise ValueError("Grasp separation below limit; entire chunk rejected")
            controller = {}
            closures = {}
            for side, pose in rig.items():
                tool = current["arms"][side + "_ee"]["active_tool"]
                controller[side] = geometry.controller_pose_from_grasp(side, pose, tool)
                delta = distance(previous_controller[side], controller[side])
                p_limit, r_limit = settings["max_control_step_m"], settings["max_control_step_rad"]
                if settings.get("adaptive_interpolation", False):
                    p_limit = min(p_limit, settings["max_tcp_speed_m_s"] * control_period(settings))
                    r_limit = min(r_limit, settings["max_tcp_rotation_rad_s"] * control_period(settings))
                if delta[0] > p_limit + 1e-12 or delta[1] > r_limit + 1e-12:
                    raise ValueError("SDK EE substep exceeds limit; increase interpolation_steps")
                previous_controller[side] = controller[side]
                closures[side] = float(previous_grips[side] * (1 - alpha) + chunk["grippers"][side][row] * alpha)
            frames.append(dict(controller=controller, grasp=rig, closures=closures,
                               source_row=chunk["source_rows"][row], row_endpoint=j == subdivisions))
        previous = targets
        previous_grips = {s: chunk["grippers"][s][row] for s in starts}
    return frames


def validate_timed_ee_chunk(chunk, observed, current, sample, geometry, settings):
    """Validate original future waypoints, then express fixed tool1 in wobj0.

    The controller does per-target IK and joint tracking, not a Cartesian line.
    These checks describe waypoints only; they do not certify the intervening path.
    """
    if timed_ee_signature(observed) != timed_ee_signature(current):
        raise ValueError("Tool/wobj binding changed while inference was in flight")
    starts = current_grasps(current)
    previous = deepcopy(starts)
    for side in starts:
        p, r = distance(observed["arms"][side + "_ee"]["grasp_rig_m_xyzw"], starts[side])
        if p > settings["max_observation_drift_m"] or r > settings["max_observation_drift_rad"]:
            raise ValueError("Robot moved too far while inference was in flight; discard old prediction")
    frames = []
    floor = geometry.config["table"]["height_above_base_mount_m"] + settings["min_grasp_height_above_table_m"]
    for index, source_row in enumerate(chunk["source_rows"]):
        poses = {s: chunk["poses"][s][index].copy() for s in starts}
        for side, pose in poses.items():
            p, r = distance(previous[side], pose)
            if p > settings["max_ee_step_m"] or r > settings["max_ee_step_rad"]:
                raise ValueError("EE model step exceeds limit; entire chunk rejected")
            p, r = distance(starts[side], pose)
            if p > settings["max_chunk_displacement_m"] or r > settings["max_chunk_rotation_rad"]:
                raise ValueError("EE chunk displacement exceeds limit; entire chunk rejected")
            if pose[2] < floor:
                raise ValueError("Grasp target crosses the table plane; entire chunk rejected")
        if np.linalg.norm(poses["left"][:3] - poses["right"][:3]) < settings["min_grasp_separation_m"]:
            raise ValueError("Grasp separation below limit; entire chunk rejected")
        controller = {s: geometry.controller_pose_from_grasp(s, poses[s], 1) for s in poses}
        frames.append(dict(controller=controller, grasp=poses,
                           closures={s: float(chunk["grippers"][s][index]) for s in poses},
                           source_row=source_row, row_endpoint=True))
        previous = poses
    return frames


class LiveEEDevices(bridge.LiveDevices):
    """Reuse dual joint inference's cameras, USB workers, expiry and cleanup."""
    def __init__(self, config, log, geometry, motion=False):
        self.geometry = geometry
        self._signature = None
        self._previous_sent = None
        config = deepcopy(config)
        config["bridge"]["timed_aj"] = dict(enabled=False)
        config["operator"]["motor_acceleration_deg_s2"] = None
        config["operator"]["joint_follower_velocity_percent"] = None
        super().__init__(config, log, motion=motion, command_mode="cartesian")

    def sample(self, *, state=None, include_history=False):
        state = self._robot_state() if state is None else state
        result = super().sample(state=state, include_history=include_history)
        result["robot_reply"] = state.raw
        return result

    def start(self):
        if not self.motion:
            raise RuntimeError("Read-only Wall-X devices cannot start motion")
        with self.lock:
            if self.gripper_pause_required:
                raise bridge.GripperFault("Gripper fault before EE start")
        self._apply_manual_speed()
        # Use the existing, verified VR mode and SDK encoder. Tool index stays unchanged.
        try:
            self.robot.ensure_tool_coordinate()
        except PermissionError as exc:
            if 'Arbitrary controller commands require the console role' not in str(exc):
                raise
            raise PermissionError(
                "Shared service has not loaded inference Tool-coordinate support. "
                "In the A10 console, confirm stopped and run set_jog_coordinate --tool, "
                "then retry; or restart the shared service after all clients exit. "
                "No follower start or motion target was sent.") from exc
        self.attempted = True
        self.robot.start_follower("cartesian")
        state = self._robot_state()
        geom = self.geometry.observation_geometry(state.raw)
        self._signature = tool_signature(geom)
        aliases = {arm.ee_id: name for name, arm in state.arms.items()}
        startup = {aliases[self.geometry.config["arms"][side]["ee_id"]]:
                   np.array(geom["arms"][side + "_ee"]["controller_tcp_base_m_xyzw"])
                   for side in ("left", "right")}
        self.robot.begin_sdk_tool_trajectory(startup)
        self._previous_sent = current_grasps(geom)
        self.cancel.clear()

    def check_ee(self):
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        state = self._robot_state()
        bridge.observation(self.sample(state=state), self.cfg["bridge"])
        geom = self.geometry.observation_geometry(state.raw)
        robot = state.raw["ret_context"]["robot_msg"]
        if (state.follower_active is not True or state.follower_mode != "Cart"
                or robot.get("activate") != "Enabled"
                or state.raw["ret_context"]["motion_msg"].get("jog_coordinate") != "Tool"):
            raise RuntimeError("EE follower is not healthy active Tool/Cart")
        if tool_signature(geom) != self._signature:
            raise RuntimeError("Tool/wobj binding changed during EE execution")
        if self._previous_sent is not None:
            for side, actual in current_grasps(geom).items():
                delta = distance(actual, self._previous_sent[side])
                if delta[0] > self.cfg["bridge"]["max_tracking_error_m"] or delta[1] > self.cfg["bridge"]["max_tracking_error_rad"]:
                    raise RuntimeError("EE actual feedback exceeded tracking error limit")
        self._checked_state = state
        return geom

    def send_ee(self, frame):
        state = self._checked_state
        if state is None or time.monotonic() - state.received_monotonic_s > self.cfg["bridge"]["max_robot_age_s"]:
            self.check_ee()
            state = self._checked_state
        with self.lock:
            if self.cancel.is_set() or self.gripper_pause_required or self.error:
                raise bridge.GripperFault("EE stream cancelled or gripper faulted")
            now = time.monotonic()
            if any(side not in self.feedback or now - self.feedback[side][1] > self.cfg["bridge"]["max_gripper_age_s"]
                   or not self.feedback[side][2] for side in ("left", "right")):
                raise bridge.GripperFault("EE gripper feedback stale/disabled")
            aliases = {arm.ee_id: name for name, arm in state.arms.items()}
            targets = {aliases[self.geometry.config["arms"][side]["ee_id"]]: frame["controller"][side]
                       for side in ("left", "right")}
            timing = self.robot.send_sdk_tool_pose(targets,
                max_position_step_m=self.cfg["bridge"]["max_control_step_m"],
                max_rotation_step_rad=self.cfg["bridge"]["max_control_step_rad"])
            # Existing USB workers consume physical-left/physical-right columns 6/13.
            row = np.zeros(14)
            row[6], row[13] = frame["closures"]["left"], frame["closures"]["right"]
            self.target = (row, time.monotonic() + .1)
            self._previous_sent = {s: p.copy() for s, p in frame["grasp"].items()}
        return timing

    def stop(self):
        result = super().stop()
        self._signature = self._previous_sent = None
        return result


class LiveTimedEEDevices(LiveEEDevices):
    """Opt-in timed_ee stream; never selects or starts the old follower."""

    def start(self):
        if not self.motion:
            raise RuntimeError("Read-only Wall-X devices cannot start motion")
        with self.lock:
            if self.gripper_pause_required or self.error:
                raise bridge.GripperFault("Gripper fault before timed_ee start")
        # The shared transport checks Manual/Enabled/Normal/Stopped and mutual
        # exclusion. The controller selects calibrated tool1/wobj0 via SDK;
        # no UI Tool selection or repeated flange offset is needed here.
        self.attempted = True
        try:
            status = self.robot.start_timed_ee(self.cfg["bridge"].get("timed_ee", {}))
        except BaseException:
            self.attempted = getattr(self.robot, 'timed_ee_start_attempted', True) is not False
            raise
        initial = np.asarray(status["command_pose_m_quat"], dtype=float)
        if initial.shape != (2, 7) or not np.isfinite(initial).all():
            raise RuntimeError("timed_ee start omitted its dual command-FK reference")
        self._timed_stream_id = status["stream_id"]
        self._timed_finished = False
        self._previous_sent = None
        self.cancel.clear()
        self.log("timed_ee_start", status=status, initial_reference_source="command_fk",
                 initial_reference_is_actual_feedback=False)
        return status

    def check_ee(self):
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        state = self._robot_state()
        bridge.observation(self.sample(state=state), self.cfg["bridge"])
        geom = self.geometry.observation_geometry(state.raw)
        info = state.raw["ret_context"]["robot_msg"]
        if (state.has_error or info.get("activate") != "Enabled"
                or info.get("op_mode") != "Manual" or info.get("status") != "Normal"):
            raise RuntimeError("timed_ee GET feedback is not healthy Manual/Enabled/Normal")
        if self.attempted:
            status = self.robot.timed_ee_status()
            if (status.get("stream_id") != self._timed_stream_id or status.get("reason") != 0
                    or status.get("stop_requested") or status.get("state") in (4, 5)):
                raise RuntimeError(f"timed_ee stream interrupted: {status}")
            if (not getattr(self, "_timed_finished", False)
                    and (not status.get("active") or status.get("state") == 6)):
                raise RuntimeError("timed_ee stream ended before final target")
        if self._previous_sent is not None:
            for side, actual in current_grasps(geom).items():
                p, r = distance(actual, self._previous_sent[side])
                if p > self.cfg["bridge"]["max_tracking_error_m"] or r > self.cfg["bridge"]["max_tracking_error_rad"]:
                    raise RuntimeError("timed_ee GET TCP pose exceeded tracking error limit")
        return geom

    def send_ee(self, frame, *, final=False):
        # Deliberately no GET, status query, camera sampling or arrival check.
        with self.lock:
            if self.cancel.is_set() or self.gripper_pause_required or self.error:
                raise bridge.GripperFault("timed_ee stream cancelled or gripper faulted")
            now = time.monotonic()
            if any(s not in self.feedback or now - self.feedback[s][1] > self.cfg["bridge"]["max_gripper_age_s"]
                   or not self.feedback[s][2] for s in ("left", "right")):
                raise bridge.GripperFault("timed_ee gripper feedback stale/disabled")
            ordered = [None, None]
            for side in ("left", "right"):
                ordered[self.geometry.config["arms"][side]["ee_id"]] = frame["controller"][side]
            timing = self.robot.send_timed_ee_target(ordered, final=final)
            row = np.zeros(14)
            row[6], row[13] = frame["closures"]["left"], frame["closures"]["right"]
            self.target = (row, time.monotonic() + .1)
            self._previous_sent = {s: p.copy() for s, p in frame["grasp"].items()}
        return timing

    def finish_ee(self):
        status = self.robot.finish_timed_ee(timeout_s=self.cfg["bridge"]["arrival_timeout_s"])
        self._timed_finished = True
        return status

    def stop(self):
        self.cancel.set()
        with self.lock:
            self.target = None
        for lock in self.write_locks.values():
            with lock:
                pass
        if not self.attempted:
            return True
        status = self.robot.stop_timed_ee()
        if status.get("active") is not False:
            raise RuntimeError("timed_ee stop unconfirmed")
        self.attempted = False
        self._previous_sent = None
        return True


def run_timed_ee(devices, policy, geometry, contract, settings, log, *, cycles,
                 read_only, clock=time, cancel=None):
    """One task for the entire stream; synchronous inference occurs between chunks."""
    started = False
    next_send = None
    period = 1. / 30.
    final_target = None
    for cycle in range(cycles):
        if cancel is not None and cancel.is_set():
            raise InterruptedError("Wall-X timed_ee run cancelled")
        if started:
            devices.check_ee()  # Block boundary only, never in the send loop.
        sample = devices.sample()
        payload, observed = observation(sample, geometry, contract, settings)
        log("observation", cycle=cycle, geometry=observed, grippers=sample["grippers"],
            tcp_pose_source="GET.ret_context.motion_msg.pq",
            tcp_pose_measured_fk_verified=False)
        began = clock.monotonic()
        response = policy.infer(payload)
        log("prediction", cycle=cycle, response=response, elapsed_s=clock.monotonic() - began)
        chunk = decode_actions(response, policy.metadata, contract, settings["execute_horizon"])
        fresh = devices.sample()
        bridge.observation(fresh, settings)
        current = geometry.observation_geometry(fresh["robot_reply"])
        chunk, reference = reference_actions(chunk, observed, current, contract, settings)
        if reference is not None:
            log("action_reference", cycle=cycle, **reference)
        frames = validate_timed_ee_chunk(chunk, observed, current, fresh, geometry, settings)
        log("plan", cycle=cycle, backend="timed_ee", source_rows=chunk["source_rows"],
            frames=frames, read_only=read_only, pose_frame="wobj0", pose_tool="tool1",
            waypoint_checks_only=True, cartesian_line_guaranteed=False)
        if read_only:
            continue
        if cancel is not None and cancel.is_set():
            raise InterruptedError("Wall-X timed_ee run cancelled before sending")
        if not started:
            devices.start()
            started = True
        # Startup can wait for the controller. Recheck its fresh GET pose once
        # outside the timed loop, never substitute the start command-FK pose.
        ready = devices.check_ee()
        if timed_ee_signature(ready) != timed_ee_signature(current):
            raise ValueError("Tool/wobj binding changed before timed_ee first send")
        for side, pose in current_grasps(ready).items():
            p, r = distance(current_grasps(current)[side], pose)
            if p > settings["max_observation_drift_m"] or r > settings["max_observation_drift_rad"]:
                raise ValueError("Robot moved too far before timed_ee first send; discard prediction")
        # Inference may overrun the ordinary cadence. Resume with a new deadline;
        # never replay missed slots or issue a catch-up burst.
        deadline = max(clock.monotonic(), next_send or 0.)
        for index, frame in enumerate(frames):
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X timed_ee run cancelled")
            clock.sleep(max(0., deadline - clock.monotonic()))
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X timed_ee run cancelled while waiting to send")
            if clock.monotonic() - deadline > settings["max_send_lag_s"]:
                raise TimeoutError("timed_ee send deadline missed; no catch-up burst")
            final = cycle == cycles - 1 and index == len(frames) - 1
            timing = devices.send_ee(frame, final=final)
            log("ee_send", cycle=cycle, index=index, source_row=frame["source_row"],
                backend="timed_ee", final=final, timing=timing)
            next_send = max(deadline + period, clock.monotonic() + period)
            deadline = next_send
        final_target = frames[-1]["grasp"]
        if settings.get("print_progress", False):
            print(f"[Wall-X timed_ee {cycle+1}/{cycles}] rows={chunk['source_rows']}, "
                  f"packets={len(frames)}, final={cycle == cycles-1}", flush=True)
    if started:
        # finish checks command FK. Separately check the existing GET TCP pose.
        # This repository does not prove whether GET pq uses actual or command
        # joints; do not report it as independently measured Cartesian accuracy.
        status = devices.finish_ee()
        log("timed_ee_command_finished", status=status, pose_source="command_fk",
            actual_feedback_confirmed=False)
        arrival = wait_arrival(devices, final_target, settings, clock, cancel, tcp_tool=1)
        log("arrival", cycle=cycles-1, confirmed=True,
            confirmation_scope="GET_tcp_pose_only",
            tcp_pose_source="GET.ret_context.motion_msg.pq",
            tcp_pose_measured_fk_verified=False, **arrival)
        if devices.stop() is not True:
            raise RuntimeError("timed_ee final stop unconfirmed")
        log("timed_ee_stop", confirmed=True)
    return dict(cycles=cycles, read_only=read_only, limited_trial=False,
                trial_step=False, backend="timed_ee", completed=True,
                measured_ee_accuracy_verified=False)


def wait_arrival(devices, target, settings, clock=time, cancel=None, *, tcp_tool=None):
    # Compare at the backend's declared TCP, independently of the UI tool.
    if tcp_tool is not None:
        target = {s: devices.geometry.controller_pose_from_grasp(s, p, tcp_tool) for s, p in target.items()}
    deadline = clock.monotonic() + settings["arrival_timeout_s"]
    stable = None
    while clock.monotonic() < deadline:
        if cancel is not None and cancel.is_set():
            raise InterruptedError("Wall-X EE run cancelled")
        geom = devices.check_ee()
        actual = current_grasps(geom)
        if tcp_tool is not None:
            actual = {s: devices.geometry.controller_pose_from_grasp(s, p, tcp_tool) for s, p in actual.items()}
        errors = [distance(actual[s], target[s]) for s in ("left", "right")]
        inside = all(p <= settings["arrival_position_tolerance_m"] and r <= settings["arrival_rotation_tolerance_rad"]
                     for p, r in errors)
        if inside:
            stable = clock.monotonic() if stable is None else stable
            if clock.monotonic() - stable >= settings["arrival_stable_s"]:
                return dict(geometry=geom, errors_frame="grasp_rig" if tcp_tool is None else f"tool{tcp_tool}_wobj0",
                            errors={s: dict(position_m=p, rotation_rad=r)
                            for s, (p, r) in zip(("left", "right"), errors)})
        else:
            stable = None
        clock.sleep(min(.02, max(0., deadline - clock.monotonic())))
    raise TimeoutError("EE target arrival not confirmed")


def trial_settings(settings):
    """Bound an explicitly requested experiment without claiming calibration."""
    result = deepcopy(settings)
    result.update(execute_horizon=1,
                  interpolation_steps=max(30, settings["interpolation_steps"]),
                  max_control_hz=min(30., settings.get("max_control_hz", 90.)))
    for key, cap in dict(max_ee_step_m=.015, max_chunk_displacement_m=.015,
                         max_ee_step_rad=.05, max_chunk_rotation_rad=.05,
                         max_observation_drift_m=.005, max_observation_drift_rad=.03,
                         max_tracking_error_m=.01, max_tracking_error_rad=.1).items():
        result[key] = min(settings[key], cap)
    result["min_grasp_height_above_table_m"] = max(settings["min_grasp_height_above_table_m"], .05)
    # A 5 mm diagnostic move must not pass arrival while still at its start.
    result["arrival_position_tolerance_m"] = min(settings["arrival_position_tolerance_m"], .001)
    result["arrival_rotation_tolerance_rad"] = min(settings["arrival_rotation_tolerance_rad"], .01)
    result["arrival_stable_s"] = max(settings["arrival_stable_s"], .1)
    return result


def validate_trial_cycles(cycles, trial_step):
    if trial_step:
        if type(cycles) is not int or not 1 <= cycles <= 5:
            raise ValueError("Small-step trial permits 1..5 cycles per run")
    elif type(cycles) is not int or cycles != 1:
        raise ValueError("Limited motion trial requires exactly one motion cycle without --trial-step")


def trial_session_budget(frames, current, origin, travelled):
    """Check every planned pose and accumulated path against the launch frame."""
    updated = deepcopy(travelled)
    previous = current_grasps(current)
    for frame in frames:
        for side, target in frame["grasp"].items():
            p, r = distance(previous[side], target)
            updated[side][0] += p
            updated[side][1] += r
            displacement, rotation = distance(origin[side], target)
            if (displacement > .025 + 1e-9 or rotation > math.radians(5.) + 1e-9
                    or updated[side][0] > .025 + 1e-9
                    or updated[side][1] > math.radians(5.) + 1e-9):
                raise ValueError(f"Trial session budget exceeded: physical {side}; "
                                 "maximum 25 mm / 5 degrees from launch and along planned path")
            previous[side] = target
    return updated


def trial_step_chunk(chunk, current, settings):
    """Explicit diagnostic approach, not execution of the full policy target.

    Both arms share a fraction of their first-target segment. Large raw jumps
    still fail; only the new bounded poses reach ordinary trajectory checks.
    """
    if chunk["source_rows"] != [0]:
        raise ValueError("Trial step requires only the first future target")
    starts = current_grasps(current)
    fraction, measured = 1., {}
    position_cap = min(.005, settings["max_ee_step_m"], settings["max_chunk_displacement_m"])
    rotation_cap = min(math.radians(1.), settings["max_ee_step_rad"], settings["max_chunk_rotation_rad"])
    for side, start in starts.items():
        p, r = distance(start, chunk["poses"][side][0])
        measured[side] = dict(position_m=p, rotation_rad=r)
        if p > .05 or r > .15:
            raise ValueError(f"Trial step raw prediction too large: physical {side}, "
                             f"{p*1000:.2f} mm / {math.degrees(r):.2f} deg; no motion")
        if p > 0:
            fraction = min(fraction, position_cap / p)
        if r > 0:
            fraction = min(fraction, rotation_cap / r)
    bounded = deepcopy(chunk)
    for side, start in starts.items():
        target = chunk["poses"][side][0]
        rotation = Slerp([0., 1.], Rotation.from_quat([start[3:], target[3:]]))
        bounded["poses"][side][0] = np.r_[
            start[:3] + fraction * (target[:3] - start[:3]), rotation([fraction]).as_quat()[0]]
    return bounded, dict(fraction=fraction, raw_delta=measured,
                         position_cap_m=position_cap, rotation_cap_rad=rotation_cap,
                         full_model_target_executed=False)


def run(devices, policy, geometry, contract, settings, log, *, cycles=1, read_only=True,
        clock=time, cancel=None, limited_trial=False, trial_step=False):
    started = False
    primary_error = None
    trial_origin = trial_signature = None
    trial_travelled = {s: [0., 0.] for s in ("left", "right")}
    try:
        if settings.get("backend", "follower") == "timed_ee" and (limited_trial or trial_step):
            raise ValueError("timed_ee does not use follower diagnostic trial modes")
        if limited_trial and settings.get("ee_action_mode", "absolute") != "absolute":
            raise ValueError("Delta retargeting uses normal execution, not the follower trial modes")
        if trial_step and not limited_trial:
            raise ValueError("trial_step requires an explicit limited motion trial")
        if limited_trial:
            if read_only:
                raise ValueError("Limited motion trial requires exactly one motion cycle")
            validate_trial_cycles(cycles, trial_step)
            settings = trial_settings(settings)
            # Hardware uses these bounds too; use a private copy rather than
            # mutating the caller's ordinary inference configuration.
            if hasattr(devices, "cfg"):
                devices.cfg = dict(devices.cfg, bridge=settings)
            log("limited_trial", model_contract_verified=contract.verified,
                hold_grippers=True, settings=settings)
        validate_settings(settings)
        if type(cycles) is not int or cycles < 1:
            raise ValueError("cycles must be a positive integer")
        if not math.isclose(policy.metadata.get("row_dt_s", settings["action_dt"]), settings["action_dt"], abs_tol=1e-6):
            raise ValueError("Configured action_dt differs from Wall-X metadata")
        if not read_only and not contract.verified and not limited_trial and not getattr(devices, "simulated", False):
            raise ValueError("Real motion requires a verified model frame/tool/gripper contract")
        log("execution_contract", model_contract_verified=contract.verified,
            limited_trial=limited_trial, trial_step=trial_step,
            execute_horizon=settings["execute_horizon"], grippers_from_model=not limited_trial)
        if settings.get("backend", "follower") == "timed_ee":
            return run_timed_ee(devices, policy, geometry, contract, settings, log,
                                cycles=cycles, read_only=read_only, clock=clock, cancel=cancel)
        for cycle in range(cycles):
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X EE run cancelled")
            sample = devices.sample()
            payload, observed = observation(sample, geometry, contract, settings)
            if trial_step:
                if trial_origin is None:
                    trial_origin = current_grasps(observed)
                    trial_signature = tool_signature(observed)
                elif tool_signature(observed) != trial_signature:
                    raise ValueError("Tool/wobj binding changed between trial steps")
            log("observation", cycle=cycle, geometry=observed, grippers=sample["grippers"])
            began = clock.monotonic()
            response = policy.infer(payload)
            log("prediction", cycle=cycle, response=response, elapsed_s=clock.monotonic() - began)
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X EE run cancelled during inference")
            chunk = decode_actions(response, policy.metadata, contract, settings["execute_horizon"])
            fresh = devices.sample()  # Never reuse the pre-inference feedback to start execution.
            bridge.observation(fresh, settings)
            current = geometry.observation_geometry(fresh["robot_reply"])
            chunk, reference = reference_actions(chunk, observed, current, contract, settings)
            if reference is not None:
                log("action_reference", cycle=cycle, **reference)
            if trial_step:
                chunk, details = trial_step_chunk(chunk, current, settings)
                log("trial_step", cycle=cycle, **details)
            if limited_trial:
                for side in ("left", "right"):
                    chunk["grippers"][side][:] = geometry.closure_fraction(fresh["grippers"][side])
            frames = validate_and_interpolate(chunk, observed, current, fresh, geometry, settings)
            if trial_step:
                trial_travelled = trial_session_budget(frames, current, trial_origin, trial_travelled)
                log("trial_session_budget", cycle=cycle, travelled=trial_travelled,
                    max_position_m=.025, max_rotation_rad=math.radians(5.))
            log("plan", cycle=cycle, source_rows=chunk["source_rows"], frames=frames, read_only=read_only)
            if read_only:
                continue
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X EE run cancelled before follower start")
            if settings.get("print_progress", False):
                starts = current_grasps(current)
                deltas = {s: distance(starts[s], frames[-1]["grasp"][s])[0] * 1000.
                          for s in ("left", "right")}
                print(f"[Wall-X {cycle+1}/{cycles}] rows={chunk['source_rows']} "
                      f"target L/R={deltas['left']:.1f}/{deltas['right']:.1f} mm, "
                      f"gripper closure L/R={frames[-1]['closures']['left']:.3f}/"
                      f"{frames[-1]['closures']['right']:.3f}, packets={len(frames)}", flush=True)
            if not started:
                devices.start()
                started = True
            period = control_period(settings)
            deadline = clock.monotonic()
            for index, frame in enumerate(frames):
                if cancel is not None and cancel.is_set():
                    raise InterruptedError("Wall-X EE run cancelled")
                clock.sleep(max(0., deadline - clock.monotonic()))
                if index % settings["feedback_every_n_steps"] == 0:
                    devices.check_ee()
                if clock.monotonic() - deadline > settings["max_send_lag_s"]:
                    raise TimeoutError("EE send deadline missed; no catch-up burst")
                timing = devices.send_ee(frame)
                log("ee_send", cycle=cycle, index=index, timing=timing)
                deadline = max(deadline + period, clock.monotonic() + period)
            arrival = wait_arrival(devices, frames[-1]["grasp"], settings, clock, cancel)
            log("arrival", cycle=cycle, confirmed=True, **arrival)
            if trial_step or settings.get("stop_between_chunks", False):
                # This diagnostic uses stop-and-wait cycles. Never leave a
                # live follower waiting for the next cloud prediction.
                if devices.stop() is not True:
                    raise RuntimeError("Chunk stop unconfirmed; no next prediction")
                started = False
                log("trial_cycle_stop" if trial_step else "chunk_stop", cycle=cycle, confirmed=True)
            if settings.get("print_progress", False):
                errors = arrival["errors"]
                print(f"[Wall-X {cycle+1}/{cycles}] arrival error L/R="
                      f"{errors['left']['position_m']*1000:.2f}/"
                      f"{errors['right']['position_m']*1000:.2f} mm", flush=True)
        return dict(cycles=cycles, read_only=read_only, limited_trial=limited_trial,
                    trial_step=trial_step, completed=True)
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        # Stop/USB-drain/restore/close uses the same cleanup as joint inference.
        # Close policy even if hardware cleanup reports an uncertain stop.
        errors = []
        try:
            devices.close()
        except BaseException as exc:
            errors.append(f"devices: {exc}")
        try:
            policy.close()
        except BaseException as exc:
            errors.append(f"policy: {exc}")
        if errors:
            message = f"Wall-X cleanup failed: {errors}"
            if primary_error is not None:
                message = f"{type(primary_error).__name__}: {primary_error}; {message}"
            raise RuntimeError(message) from primary_error


def simulation_contract():
    """Synthetic identity contract for offline checks, never a cloud calibration."""
    return ModelContract(dict(embodiment="simulation", verified=False,
        model_frame_from_rig={s: [0., 0., 0., 0., 0., 0., 1.] for s in ("left", "right")},
        model_ee_in_grasp_m_xyzw={s: [0., 0., 0., 0., 0., 0., 1.] for s in ("left", "right")},
        model_gripper_from_closure={s: [0., 1.] for s in ("left", "right")}))


class SimEEDevices:
    """Idealized feedback simulator for exercising the exact production loop."""
    simulated = True

    def __init__(self, geometry, clock=time):
        self.geometry, self.clock = geometry, clock
        down = Rotation.from_euler("x", np.pi).as_quat()
        self.poses = {s: np.r_[.4, y, .5, down] for s, y in (("left", .25), ("right", -.25))}
        self.grippers = dict(left=4500, right=4500)
        self.active = self.closed = False
        self.sent = self.starts = self.stops = 0

    def sample(self):
        now = self.clock.monotonic()
        pq = [None, None]
        for side in ("left", "right"):
            pose = self.geometry.controller_pose_from_grasp(side, self.poses[side], 0)
            pose[:3] *= 1000.
            pq[self.geometry.config["arms"][side]["ee_id"]] = pose.tolist()
        reply = dict(ret_code=0, ret_context=dict(
            robot_msg=dict(tool=[0, 0], wobj=[0, 0], tool_mode=["Tool", "Tool"], cs_error_code=0,
                           activate="Enabled", motion="Stopped"),
            motion_msg=dict(pq=pq), log_msg=dict(has_error=False), driver_msg=dict(driver_err_code=[0] * 12)))
        return dict(joints={0: np.zeros(6), 1: np.zeros(6)}, grippers=dict(self.grippers),
                    gripper_times=dict(left=now, right=now), robot_time=now, has_error=False,
                    camera_times={n: now for n in bridge.CAMERAS},
                    images={n: np.zeros((32, 32, 3), np.uint8) for n in bridge.CAMERAS}, robot_reply=reply)

    def start(self):
        self.starts += 1
        self.active = True

    def check_ee(self):
        if not self.active:
            raise RuntimeError("Simulated EE follower inactive")
        return self.geometry.observation_geometry(self.sample()["robot_reply"])

    def send_ee(self, frame):
        if not self.active:
            raise RuntimeError("Simulated EE follower inactive")
        self.poses = {s: p.copy() for s, p in frame["grasp"].items()}
        self.grippers = {s: int(round(c * 9000)) for s, c in frame["closures"].items()}
        self.sent += 1
        return dict(simulated=True, execution_confirmed=True)

    def stop(self):
        if self.active:
            self.stops += 1
        self.active = False
        return True

    def close(self):
        self.stop()
        self.closed = True


class SimTimedEEDevices(SimEEDevices):
    """Offline ideal feedback, exercising the TimedEe stream lifecycle."""
    def __init__(self, geometry, clock=time):
        super().__init__(geometry, clock)
        self.final_flags = []
        self.finished = False

    def send_ee(self, frame, *, final=False):
        if self.final_flags and self.final_flags[-1]:
            raise RuntimeError("Target after simulated final point")
        self.final_flags.append(final)
        return super().send_ee(frame)

    def finish_ee(self):
        if not self.final_flags or not self.final_flags[-1]:
            raise RuntimeError("Cannot finish simulated TimedEe without final point")
        self.finished = True
        return dict(simulated=True, final_reference_reached=True,
                    ee_pose_source="command_fk")


class SimPolicy:
    metadata = dict(action_horizon=32, valid_rows=31, row_dt_s=.0337,
                    rtc=dict(row_zero="command_at_observation_anchor"))

    def __init__(self):
        self.closed = False
        self.requests = 0

    def infer(self, payload):
        self.requests += 1
        actions = {}
        for side in ("left", "right"):
            block = payload["observation"]["state"][side + "_ee"]
            positions = np.tile(block["position"], (32, 1))
            positions[:, 0] += (np.arange(32) + 1) * .001
            actions[side + "_ee"] = dict(position=positions,
                rotation_rpy=np.tile(block["rotation_rpy"], (32, 1)),
                gripper=np.full(32, block["gripper"]))
        return dict(embodiment=payload["embodiment"], actions=actions)

    def close(self):
        self.closed = True
