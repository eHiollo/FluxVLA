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
    for key in ("execute_horizon", "interpolation_steps", "feedback_every_n_steps"):
        if type(settings[key]) is not int or settings[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if settings["execute_horizon"] > 32:
        raise ValueError("execute_horizon must be <=32 public future rows")
    for key in ("action_dt", "max_ee_step_m", "max_ee_step_rad", "max_control_step_m",
                "max_control_step_rad", "max_chunk_displacement_m", "max_chunk_rotation_rad",
                "max_observation_drift_m", "max_observation_drift_rad", "max_tracking_error_m",
                "max_tracking_error_rad", "max_send_lag_s", "arrival_timeout_s",
                "arrival_position_tolerance_m", "arrival_rotation_tolerance_rad", "arrival_stable_s",
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


def tool_signature(geom):
    return tuple((geom["arms"][s + "_ee"]["ee_id"], geom["arms"][s + "_ee"]["active_tool"],
                  geom["arms"][s + "_ee"]["active_wobj"]) for s in ("left", "right"))


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
    subdivisions = settings["interpolation_steps"]
    for row in range(len(chunk["source_rows"])):
        targets = {s: chunk["poses"][s][row] for s in starts}
        for side, target in targets.items():
            delta, total = distance(previous[side], target), distance(starts[side], target)
            if delta[0] > settings["max_ee_step_m"] or delta[1] > settings["max_ee_step_rad"]:
                raise ValueError("EE model step exceeds limit; entire chunk rejected")
            if total[0] > settings["max_chunk_displacement_m"] or total[1] > settings["max_chunk_rotation_rad"]:
                raise ValueError("EE chunk displacement exceeds limit; entire chunk rejected")
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
                if delta[0] > settings["max_control_step_m"] or delta[1] > settings["max_control_step_rad"]:
                    raise ValueError("SDK EE substep exceeds limit; increase interpolation_steps")
                previous_controller[side] = controller[side]
                closures[side] = float(previous_grips[side] * (1 - alpha) + chunk["grippers"][side][row] * alpha)
            frames.append(dict(controller=controller, grasp=rig, closures=closures))
        previous = targets
        previous_grips = {s: chunk["grippers"][s][row] for s in starts}
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
        self.robot.ensure_tool_coordinate()
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


def wait_arrival(devices, target, settings, clock=time, cancel=None):
    deadline = clock.monotonic() + settings["arrival_timeout_s"]
    stable = None
    while clock.monotonic() < deadline:
        if cancel is not None and cancel.is_set():
            raise InterruptedError("Wall-X EE run cancelled")
        actual = current_grasps(devices.check_ee())
        errors = [distance(actual[s], target[s]) for s in ("left", "right")]
        inside = all(p <= settings["arrival_position_tolerance_m"] and r <= settings["arrival_rotation_tolerance_rad"]
                     for p, r in errors)
        if inside:
            stable = clock.monotonic() if stable is None else stable
            if clock.monotonic() - stable >= settings["arrival_stable_s"]:
                return
        else:
            stable = None
        clock.sleep(min(.02, max(0., deadline - clock.monotonic())))
    raise TimeoutError("EE target arrival not confirmed")


def run(devices, policy, geometry, contract, settings, log, *, cycles=1, read_only=True,
        clock=time, cancel=None):
    started = False
    primary_error = None
    try:
        validate_settings(settings)
        if type(cycles) is not int or cycles < 1:
            raise ValueError("cycles must be a positive integer")
        if not math.isclose(policy.metadata.get("row_dt_s", settings["action_dt"]), settings["action_dt"], abs_tol=1e-6):
            raise ValueError("Configured action_dt differs from Wall-X metadata")
        if not read_only and not contract.verified and not getattr(devices, "simulated", False):
            raise ValueError("Real motion requires a verified model frame/tool/gripper contract")
        for cycle in range(cycles):
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X EE run cancelled")
            sample = devices.sample()
            payload, observed = observation(sample, geometry, contract, settings)
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
            frames = validate_and_interpolate(chunk, observed, current, fresh, geometry, settings)
            log("plan", cycle=cycle, source_rows=chunk["source_rows"], frames=frames, read_only=read_only)
            if read_only:
                continue
            if cancel is not None and cancel.is_set():
                raise InterruptedError("Wall-X EE run cancelled before follower start")
            if not started:
                devices.start()
                started = True
            period = settings["action_dt"] / settings["interpolation_steps"]
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
            wait_arrival(devices, frames[-1]["grasp"], settings, clock, cancel)
            log("arrival", cycle=cycle, confirmed=True)
        return dict(cycles=cycles, read_only=read_only, completed=True)
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

    def close(self):
        if self.active:
            self.stops += 1
        self.active, self.closed = False, True


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
