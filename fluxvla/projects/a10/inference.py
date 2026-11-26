"""Shared synchronous dual A10 inference control. Hardware imports occur only in LiveDevices."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
CAMERAS = {
    "base_0_rgb": "top",
    "left_wrist_0_rgb": "left",
    "right_wrist_0_rgb": "right",
}


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    import sys

    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# Same function as the recorder, without importing FluxVLA's model registry.
pack_dual_state = load_file("_openpi_dual_schema", ROOT / "fluxvla/datasets/a10_recording_schema.py").pack_dual_state


def plain(value):
    if isinstance(value, type):
        return f"{value.__module__}.{value.__qualname__}"
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


class Journal:
    def __init__(self, path, metadata=None):
        from fluxvla.engines.utils.a10_debug_log import A10DebugLog
        self.recorder = A10DebugLog(role='dual_client', directory=Path(path).parent,
                                    image_every_n_cycles=0, queue_size=2048, metadata=metadata,
                                    units=dict(joints='rad', gripper='normalized_closure_0_1',
                                               gripper_raw='0..9000', time='s'))

    def __call__(self, event, **fields):
        self.recorder.record(event, **fields)

    def close(self):
        self.recorder.close()


class RemotePolicy:
    def __init__(self, host, port, timeout_s=120.0, client_root=None):
        from websockets.sync.client import connect

        if client_root:
            codec = Path(client_root) / "openpi_client/msgpack_numpy.py"
            self.codec = load_file("_openpi_wire_codec", codec)
        else:
            from openpi_client import msgpack_numpy

            self.codec = msgpack_numpy
        self.timeout = timeout_s
        self.ws = connect(
            f"ws://{host}:{port}",
            compression=None,
            max_size=None,
            open_timeout=timeout_s,
            # Synchronous server inference can block pong replies during JAX
            # compilation. recv(timeout=self.timeout) bounds response waiting.
            ping_interval=None,
        )
        try:
            self.metadata = self.decode(self.ws.recv(timeout=timeout_s))
        except BaseException:
            self.ws.close()
            raise

    def decode(self, message):
        if isinstance(message, str):
            raise RuntimeError(f"OpenPI server error: {message}")
        return self.codec.unpackb(message)

    def infer(self, observation):
        self.ws.send(self.codec.packb(observation))
        message = self.ws.recv(timeout=self.timeout)
        self.response_received_s = time.monotonic()
        return self.decode(message)

    def close(self):
        self.ws.close()


def observation(sample, settings):
    now = time.monotonic()
    if sample.get("has_error") is not False:
        raise RuntimeError("Faulted/unknown robot feedback")
    times = []
    for stamp, limit in [
        (sample["robot_time"], settings["max_robot_age_s"]),
        *[(s, settings["max_gripper_age_s"]) for s in sample["gripper_times"].values()],
        *[(s, settings["max_camera_age_s"]) for s in sample["camera_times"].values()],
    ]:
        if not math.isfinite(stamp) or not 0 <= now - stamp <= limit:
            raise RuntimeError("Missing/stale feedback")
        times.append(stamp)
    if max(times) - min(times) > settings["max_skew_s"]:
        raise RuntimeError("Feedback receive-time skew exceeded")
    state = pack_dual_state(sample["joints"], sample["grippers"])
    result = {"observation/state": state}
    for name, alias in CAMERAS.items():
        image = np.asarray(sample["images"][name])
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[-1] != 3 or min(image.shape[:2]) < 1:
            raise ValueError(f"{name}: expected nonempty HWC uint8 RGB")
        result[f"observation/images/{alias}"] = image
    if settings.get("prompt") is not None:
        result["prompt"] = settings["prompt"]
    return result


def actions_from_response(response, action_count=30):
    raw = np.asarray(response["actions"])
    if raw.shape != (action_count, 14) or raw.dtype.kind not in "fiu" or not np.isfinite(raw).all():
        raise ValueError(f"Expected finite numeric actions [{action_count},14]")
    actions = np.array(raw, dtype=np.float64, copy=True)
    before = actions[:, [6, 13]].copy()
    actions[:, [6, 13]] = np.clip(before, 0, 1)
    return actions, int(np.count_nonzero(before != actions[:, [6, 13]]))


def check_steps(actions, state, limit):
    indices = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
    delta = np.diff(np.vstack([state, actions])[:, indices], axis=0)
    if np.any(np.abs(delta) > limit):
        raise ValueError("Joint step exceeds max_joint_step_rad; entire chunk rejected")


class SimDevices:
    """Explicit simulator; never imports or opens hardware drivers."""

    def __init__(self, log):
        self.log = log
        self.joints = {1: np.full(6, 0.1), 0: np.full(6, -0.2)}
        self.grippers = {"left": 2250, "right": 6750}
        self.active = False
        self.sent = 0

    def sample(self):
        now = time.monotonic()
        return dict(
            joints=self.joints,
            grippers=self.grippers,
            robot_time=now,
            has_error=False,
            gripper_times=dict(left=now, right=now),
            camera_times={k: now for k in CAMERAS},
            images={k: np.full((224, 224, 3), i * 60, np.uint8) for i, k in enumerate(CAMERAS, 1)},
        )

    def start(self):
        self.active = True

    def check(self):
        if not self.active:
            raise RuntimeError("Simulator follower inactive")

    def send(self, row):
        self.check()
        self.joints = {1: row[:6].copy(), 0: row[7:13].copy()}
        self.grippers = dict(left=int(round(row[6] * 9000)), right=int(round(row[13] * 9000)))
        self.sent += 1
        self.log(
            "simulated_send",
            frame=self.sent,
            joints=self.joints,
            gripper_raw=self.grippers,
        )

    def arrival_feedback(self):
        return dict(joints=np.r_[self.joints[1], self.joints[0]], stamp=time.monotonic(),
                    has_error=False, follower_active=self.active,
                    follower_mode='Axis' if self.active else None,
                    activation='Enabled', motion='Stopped')

    def stop(self):
        self.active = False
        return True

    def close(self):
        pass


class GripperFault(RuntimeError):
    """Recoverable gripper fault; motion must pause until acknowledged."""


def gripper_target_raw(closure, device):
    """Return unbiased and compensated targets; never accumulate on feedback.

    Both onsite grippers close toward 9000. Fade the offset near full open so
    small positive model noise does not become a large closing command.
    """
    model_target = int(round(closure * 9000))
    bias = device.get('closure_bias_raw', 0)
    fade = device.get('closure_bias_fade_raw', 900)
    offset = int(round(bias * min(1., max(0., model_target / fade))))
    return model_target, min(9000, max(0, model_target + offset))


class GripperReversalGuard:
    """Confirm sustained opposite commands without blocking the arm publisher.

    The reference is the last acknowledged command, never measured aperture:
    contact and tracking lag must not manufacture a new motion direction.
    """

    def __init__(self, deadband_raw=0, confirm_s=0.):
        self.deadband_raw, self.confirm_s = deadband_raw, confirm_s
        self.reset()

    def reset(self):
        self.target, self.direction, self.pending_s = None, 0, None

    def allows(self, target, now):
        if self.target is None:
            return True
        delta = target-self.target
        direction = (delta > 0)-(delta < 0)
        if not direction or not self.direction or direction == self.direction:
            self.pending_s = None
            return True
        # Endpoints still require time confirmation, but a tiny residual
        # aperture must never prevent an intentional complete release.
        if abs(delta) < self.deadband_raw and target not in (0, 9000):
            self.pending_s = None
            return False
        if self.pending_s is None:
            self.pending_s = now
        return now-self.pending_s >= self.confirm_s-1e-12

    def acknowledge(self, target):
        if self.target is not None and target != self.target:
            self.direction = 1 if target > self.target else -1
        self.target, self.pending_s = target, None


class LiveDevices:
    def __init__(self, config, log, motion=False, *, command_mode="joint"):
        from fluxvla.engines.operators.a10_operator import A10Operator
        from fluxvla.engines.operators.camera_operator import BufferedCameraOperator
        from fluxvla.engines.operators.zx_gripper import ZXGripper

        self.log, self.motion = log, motion
        self.cfg = config
        settings = dict(config["operator"])
        settings.pop("type", None)
        if command_mode not in ("joint", "cartesian"):
            raise ValueError("command_mode must be joint or cartesian")
        settings.update(command_mode=command_mode, allow_motion=motion, cameras=None)
        settings["shared_control"] = dict(settings["shared_control"], role="inference")
        if settings["shared_control"].get("mode") != "managed":
            raise ValueError("Managed shared follower required")
        self.robot = A10Operator(**settings)
        self.longwam_history = config['bridge'].get('longwam_history', False)
        self.cameras = BufferedCameraOperator(config["cameras"],
                                             history_size=64 if self.longwam_history else 4)
        self.grips = {}
        self.feedback = {}
        self.error = None
        self.gripper_faults = {}
        self.gripper_pause_required = False
        self.lock = threading.Lock()
        self.target = None
        self.cancel = threading.Event()
        self.cancel.set()
        self.quit = threading.Event()
        self.threads = []
        self.write_locks = {side: threading.Lock() for side in ("left", "right")}
        self.attempted = False
        self._checked_state = None
        self._speed_restore = None
        try:
            self.robot.connect()
            if motion and command_mode == "joint" and not self.timed_enabled:
                self.robot.ensure_motor_acceleration()
            self.cameras.start()
            for side, device in config["grippers"].items():
                check = device["check"]
                if (check["open_raw"], check["close_raw"]) != (0, 9000):
                    raise ValueError("Model requires raw 0..9000 closure calibration")
                identity = device["identity"]
                grip = ZXGripper(
                    device["port"],
                    baudrate=device["baudrate"],
                    timeout=device.get("serial_timeout_s", .2),
                    max_position_raw=9000,
                    expected_usb_serial=identity["serial"],
                    expected_usb_interface=identity["usb_interface"],
                    drop_alarm_policy=device.get("drop_alarm_policy", "stop"),
                )
                self.grips[side] = grip
                thread = threading.Thread(target=self._gripper_worker, args=(side, device), daemon=True)
                self.threads.append(thread)
                thread.start()
            deadline = time.monotonic() + 20
            while True:
                try:
                    observation(self.sample(), config["bridge"])
                    break
                except GripperFault:
                    self.wait_for_grippers()
                    deadline = time.monotonic() + 20
                except (RuntimeError, KeyError):
                    if self.error:
                        raise RuntimeError("Gripper worker failed") from self.error
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)
        except BaseException:
            self.close()
            raise

    def _gripper_worker(self, side, device):
        from fluxvla.engines.operators.zx_gripper import (
            ZXGripperCommunicationError, ZXGripperError, ZXGripperInputExpired,
            has_blocking_alarm,
        )

        grip = self.grips[side]
        motion_only = device.get("feedback_mode", "full") == "motion"
        health = None
        health_received_s = None
        last_warning = None
        last_command_id = None
        last_command_target = None
        target_deadband = device.get('target_deadband_raw', 0)
        reversal = GripperReversalGuard(device.get('reversal_deadband_raw', 0),
                                        device.get('reversal_confirm_s', 0.))
        period = 1 / device.get("io_hz", 20.)
        try:
            while not self.quit.is_set():
                started = time.monotonic()
                try:
                    grip.connect()
                    if motion_only and health is not None:
                        state = grip.read_motion_state(device["device_id"])
                        sampled_health = False
                    else:
                        state = grip.read_state(device["device_id"])
                        health, health_received_s = state, time.monotonic()
                        sampled_health = True
                        if state.has_error and has_blocking_alarm(
                            state.alarm_a, state.alarm_b,
                            drop_alarm_policy=device.get("drop_alarm_policy", "stop"),
                        ):
                            raise ZXGripperError(
                                f"{side} gripper alarm A=0x{state.alarm_a:04X} B=0x{state.alarm_b:04X}"
                            )
                        warning = (state.alarm_a, state.alarm_b) if state.has_error else None
                        if warning and warning != last_warning:
                            self.log("gripper_alarm_warning", side=side, alarm_a=warning[0], alarm_b=warning[1])
                            print(f"[A10] {side} gripper warning A=0x{warning[0]:04X} B=0x{warning[1]:04X}; continuing", flush=True)
                        last_warning = warning
                except ZXGripperError as exc:
                    last_command_target = None
                    with self.lock:
                        previous = self.feedback.get(side)
                    # A bad motion sample is dropped, never replaced with a
                    # fresh timestamp or retried. Prolonged loss still pauses.
                    if (motion_only and health is not None
                            and isinstance(exc, ZXGripperCommunicationError)
                            and previous is not None
                            and time.monotonic() - previous[1] <=
                            self.cfg["bridge"].get("max_gripper_age_s", .2)):
                        self.log("gripper_feedback_dropped", side=side, error=str(exc))
                        self.quit.wait(max(0., period - (time.monotonic() - started)))
                        continue
                    self._gripper_fault(side, exc)
                    reversal.reset()
                    health = None
                    grip.disconnect()
                    self.quit.wait(0.2)
                    continue
                received_s = time.monotonic()
                with self.lock:
                    self.gripper_faults.pop(side, None)
                    # In motion-only mode enabled is the startup check. Its
                    # age is recorded separately, never presented as fresh.
                    self.feedback[side] = (state.position_raw, received_s, health.enabled)
                    target = self.target
                    final_target = getattr(self, '_gripper_target_final', False)
                self.log("gripper_feedback", side=side,
                         position_raw=state.position_raw, received_s=received_s,
                         target_raw=getattr(state, "target_raw", None),
                         speed_raw=getattr(state, "speed_raw", None),
                         current_raw=getattr(state, "current_raw", None),
                         enabled=health.enabled if sampled_health else None,
                         health_received_s=health_received_s,
                         status=getattr(state, "status", None),
                         alarm_a=getattr(state, "alarm_a", None),
                         alarm_b=getattr(state, "alarm_b", None))
                if target is not None and not self.cancel.is_set():
                    row, deadline = target
                    index = 6 if side == "left" else 13
                    model_target, requested_target = gripper_target_raw(row[index], device)
                    # Compare with the last acknowledged target, so slow
                    # intended motion accumulates past the deadband. Exact
                    # open/closed endpoints always pass through unchanged.
                    changed = (target_deadband == 0 or last_command_target is None or requested_target in (0, 9000)
                               or abs(requested_target-last_command_target) > target_deadband)
                    fresh = time.monotonic() < deadline
                    allowed = reversal.allows(requested_target, time.monotonic()) if fresh else False
                    # An explicit task endpoint is committed, unlike an
                    # ordinary replaceable forecast. Its TTL may be shorter
                    # than confirmation; issue it once while it is fresh.
                    allowed = allowed or (fresh and final_target)
                    if not fresh:
                        reversal.pending_s = None
                    if fresh and changed and allowed:
                        if not health.enabled:
                            self._gripper_fault(side, "Gripper must already be enabled; no automatic enable")
                            self.quit.wait(0.2)
                            continue
                        with self.write_locks[side]:
                            try:
                                write_started = time.monotonic()
                                options = (dict(compact_trigger=True)
                                           if device.get("compact_trigger", False) else {})
                                command_id = grip.set_position_raw(
                                    device["device_id"],
                                    requested_target,
                                    speed=device["speed"], torque=device["torque"],
                                    feedback=health,
                                    motion_deadline_s=deadline,
                                    motion_cancel_event=self.cancel, **options,
                                )
                                if command_id != last_command_id:
                                    self.log("gripper_command", side=side,
                                             target_raw=requested_target,
                                             model_target_raw=model_target,
                                             applied_bias_raw=requested_target - model_target,
                                             command_id=command_id,
                                             write_started_s=write_started,
                                             acknowledged_s=time.monotonic(),
                                             speed=device["speed"], torque=device["torque"])
                                last_command_id = command_id
                                last_command_target = requested_target
                                reversal.acknowledge(requested_target)
                            except ZXGripperInputExpired as exc:
                                self.log("gripper_target_discarded", side=side,
                                         error=str(exc), cancelled=self.cancel.is_set())
                            except ZXGripperError as exc:
                                self._gripper_fault(side, exc)
                                reversal.reset()
                                health = None
                                grip.disconnect()
                else:
                    last_command_target = None
                    reversal.reset()
                # No catch-up burst after a slow read/write; use latest target.
                self.quit.wait(max(0., period - (time.monotonic() - started)))
        except BaseException as exc:
            self.error = exc
            self.cancel.set()

    def _gripper_fault(self, side, error):
        with self.lock:
            changed = self.gripper_faults.get(side) != str(error)
            self.gripper_faults[side] = str(error)
            self.gripper_pause_required = True
            self.cancel.set()
            self.target = None
        if changed:
            self.log("gripper_fault", side=side, error=str(error))
            print(f"[A10] {side} gripper fault; motion paused: {error}", flush=True)

    def wait_for_grippers(self):
        # Cancel pending targets and confirm the arms stopped before waiting.
        if not self.stop():
            raise RuntimeError("Follower stop unconfirmed")
        self.log("gripper_pause")
        print("[A10] Motion paused. Gripper feedback polling continues.", flush=True)
        while True:
            if self.error:
                raise RuntimeError("Gripper worker failed") from self.error
            with self.lock:
                healthy = not self.gripper_faults and all(
                    side in self.feedback
                    and time.monotonic() - self.feedback[side][1] <= self.cfg["bridge"]["max_gripper_age_s"]
                    and (self.feedback[side][2] or not self.motion)
                    for side in self.grips
                )
            if not healthy:
                time.sleep(0.2)
                continue
            input("[A10] Gripper feedback recovered. Press Enter to retry with a fresh prediction (Ctrl+C exits): ")
            with self.lock:
                # Feedback may fail again while the terminal prompt is open.
                if self.gripper_faults or any(
                    time.monotonic() - self.feedback[side][1] > self.cfg["bridge"]["max_gripper_age_s"]
                    or (self.motion and not self.feedback[side][2])
                    for side in self.grips
                ):
                    continue
                self.gripper_pause_required = False
            self.log("gripper_resume")
            return

    def _robot_state(self):
        started = time.monotonic()
        state = self.robot.get_state()
        callback = getattr(self, 'feedback_callback', None)
        if callback is not None:
            callback(state, started)
        return state

    def sample(self, *, state=None, include_history=False):
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        with self.lock:
            if self.gripper_pause_required:
                raise GripperFault(f"Gripper fault requires acknowledgement: {self.gripper_faults}")
        if state is None:
            state = self._robot_state()
        history = (self.cameras.snapshot_window(state.received_monotonic_s)
                   if include_history and getattr(self, 'longwam_history', False) else None)
        images = ({name: frames[-1] for name, frames in history.items()} if history is not None
                  else self.cameras.snapshot(state.received_monotonic_s))
        with self.lock:
            feedback = dict(self.feedback)
        return dict(
            joints={arm.ee_id: arm.actual_joints_rad for arm in state.arms.values()},
            grippers={side: feedback[side][0] for side in ("left", "right")},
            gripper_times={side: feedback[side][1] for side in ("left", "right")},
            robot_time=state.received_monotonic_s,
            has_error=state.has_error,
            images={name: value["image"] for name, value in images.items()},
            camera_times={name: value["received_monotonic_s"] for name, value in images.items()},
            image_history=history,
        )

    @property
    def timed_enabled(self):
        return (self.cfg['bridge'].get('timed_aj') or {}).get('enabled', False)

    def start(self):
        self._checked_state = None
        if not self.motion:
            raise RuntimeError("Read-only mode")
        with self.lock:
            if self.gripper_pause_required:
                raise GripperFault("Gripper fault before follower start")
        if self.timed_enabled:
            from fluxvla.engines.communication.a10_timed_aj import timed_config
            cfg = timed_config(self.cfg['bridge']['timed_aj'])
            if not cfg['network_ready']:
                raise RuntimeError('timed_aj UDP loopback entry is not remotely reachable; '
                                   'configure NIC listening/forwarding before network_ready=True')
            # Production fixed_A limits are downstream code constants. Neither
            # derive tracking limits from XML nor rewrite site motor settings.
            self._timed_initial_state = self._robot_state()
            self.attempted = True  # Also clean up an uncertain asynchronous start.
            try:
                status = self.robot.start_timed_aj(cfg)
            except PermissionError as exc:
                # These relay claim denials precede any downstream Plan start.
                # Other errors (including lost replies) still require cleanup.
                if str(exc) in (
                    'Another session owns the follower; stop it before starting inference',
                    'Console command is still running',
                    'An unmanaged follower is active; the shared service will not take it over',
                ):
                    self.attempted = False
                    self._timed_initial_state = None
                raise
            self._timed_stream_id, self._timed_index = status['stream_id'], 0
            self._timed_final = False
            self.log('timed_stream_start', status=status, independent_block=False)
        else:
            self._apply_manual_speed()
            self.attempted = True
            self.robot.start_follower("joint")
        with self.lock:
            if self.gripper_pause_required:
                raise GripperFault("Gripper fault during follower start")
            self.cancel.clear()

    def _apply_manual_speed(self):
        percent = self.cfg["bridge"].get("manual_speed_percent")
        if percent is None or self._speed_restore is not None:
            return
        original = self.robot.get_speed_override()
        if original["mode"] != "Manual":
            raise RuntimeError("manual_speed_percent requires Manual mode; mode was not changed")
        state = self.robot.get_state()
        info = state.raw["ret_context"]["robot_msg"]
        if (state.has_error or state.follower_active is not False
                or info.get("activate") != "Enabled" or info.get("motion") != "Stopped"):
            raise RuntimeError("Manual speed change requires healthy Enabled/Stopped feedback and inactive follower")
        # Save before sending: the setting may take effect even if its ACK is lost.
        self._speed_restore = dict(original)
        if original["percent"] != percent:
            self.robot.set_speed_override(percent, expected_mode="Manual",
                                          expected_percent=original["percent"])
        self.log("manual_speed_applied", original=original, percent=percent)
        print(f"[A10] Manual speed {original['percent']:g}% -> {percent:g}%; restore on exit.", flush=True)

    def _restore_manual_speed(self):
        original = self._speed_restore
        if original is None:
            return
        percent = self.cfg["bridge"]["manual_speed_percent"]
        current = self.robot.get_speed_override()
        if current["mode"] != original["mode"] or current["percent"] not in (percent, original["percent"]):
            raise RuntimeError("Controller mode or speed changed externally; preserving site settings")
        if current["percent"] != original["percent"]:
            self.robot.set_speed_override(original["percent"], expected_mode=original["mode"],
                                          expected_percent=current["percent"], allow_cancelled=True)
        self._speed_restore = None
        self.log("manual_speed_restored", **original)
        print(f"[A10] Manual speed restored to {original['percent']:g}%.", flush=True)

    def check(self):
        # A successful check supplies exactly one send. Never reuse a failed
        # check or carry feedback over to the next frame/chunk.
        self._checked_state = None
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        state = self._robot_state()
        observation(self.sample(state=state), self.cfg["bridge"])
        info = state.raw['ret_context']['robot_msg']
        active = (True if self.timed_enabled else
                  state.follower_active is True and state.follower_mode == 'Axis')
        if (state.has_error or not active or info.get('activate') != 'Enabled'
                or (self.timed_enabled and (info.get('op_mode') != 'Manual' or info.get('status') != 'Normal'))):
            raise RuntimeError("Follower feedback fault/inactive")
        if self.timed_enabled:
            from fluxvla.engines.communication.a10_timed_aj import require_stream
            require_stream(self.robot.timed_aj_status(), self._timed_stream_id, accepting=True)
        now = time.monotonic()
        with self.lock:
            feedback = dict(self.feedback)
        if any(now - feedback[side][1] > self.cfg["bridge"]["max_gripper_age_s"] for side in ("left", "right")):
            raise RuntimeError("Stale gripper feedback")
        self._checked_state = state

    def send(self, row, *, final=False):
        # A10Operator verifies current ownership before every UDP target.
        if self.timed_enabled:
            # This path submits targets only. Acquisition/health checks belong
            # to observation and stream startup, never the 30 Hz send loop.
            state = self._timed_initial_state
            if self.error:
                raise RuntimeError('Gripper worker failed') from self.error
        else:
            if self._checked_state is None:
                self.check()
            state, self._checked_state = self._checked_state, None
            observation(self.sample(state=state), self.cfg["bridge"])
        mapping = {arm.ee_id: name for name, arm in state.arms.items()}
        if set(mapping) != {0, 1}:
            raise RuntimeError("Missing dual arm mapping")
        with self.lock:
            if self.gripper_pause_required:
                raise GripperFault("Gripper fault before joint send")
            if self.timed_enabled:
                from fluxvla.engines.communication.a10_timed_aj import ROW_TO_PHYSICAL
                if self._timed_index == 0:
                    arms = {a.ee_id: a for a in state.arms.values()}
                    commands = [arms[i].target_joints_rad for i in (0, 1)]
                    if any(q is None for q in commands):
                        raise RuntimeError('timed_aj requires current command positions for initial continuity')
                    if np.max(np.abs(np.concatenate(commands) - row[ROW_TO_PHYSICAL])) > .035:
                        raise RuntimeError('timed_aj first point exceeds 0.035 rad command continuity; no target sent')
                timing = self.robot.send_timed_joint_target(
                    row[ROW_TO_PHYSICAL], final=final)
                self._timed_index += 1
                self._timed_final = final
            else:
                timing = self.robot.send_joints({mapping[1]: row[:6], mapping[0]: row[7:13]})
            self.target = (row.copy(), time.monotonic() + 0.1)
            self._gripper_target_final = bool(final)
        return timing

    def arrival_feedback(self):
        # No camera snapshot here: the next observation is acquired AFTER arrival.
        if self.error:
            raise RuntimeError('Gripper worker failed') from self.error
        with self.lock:
            if self.gripper_pause_required:
                raise GripperFault('Gripper fault during arrival wait')
        state = self._robot_state()
        arms = {arm.ee_id: arm for arm in state.arms.values()}
        if set(arms) != {0, 1}:
            raise RuntimeError('Missing dual arm arrival feedback')
        info = state.raw['ret_context']['robot_msg']
        result = dict(joints=np.r_[arms[1].actual_joints_rad, arms[0].actual_joints_rad],
                    stamp=state.received_monotonic_s, has_error=state.has_error,
                    follower_active=state.follower_active, follower_mode=state.follower_mode,
                    activation=info.get('activate'), motion=info.get('motion'))
        if self.timed_enabled:
            from fluxvla.engines.communication.a10_timed_aj import require_stream
            status = self.robot.timed_aj_status()
            if self.attempted:
                require_stream(status, self._timed_stream_id)
            elif status['stream_id'] != self._timed_stream_id or status['active']:
                raise RuntimeError('timed_aj restarted externally after stop')
            commands = [arms[i].target_joints_rad for i in (1, 0)]
            if any(q is None for q in commands):
                raise RuntimeError('Missing timed_aj command position feedback')
            result.update(timed_status=status, command_joints=np.concatenate(commands),
                          reference_finished=(status['final_reference_reached'] and status['reason'] == 0
                                              and status['consumed'] == self._timed_index
                                              and self._timed_final),
                          operation_mode=info.get('op_mode'), robot_status=info.get('status'))
        return result

    def stop(self):
        self._checked_state = None
        self.cancel.set()
        with self.lock:
            self.target = None
        # Drain bounded in-flight writes before this cycle can finish/restart.
        for lock in self.write_locks.values():
            with lock:
                pass
        if not self.attempted:
            return True
        if self.timed_enabled:
            result = self.robot.stop_timed_aj()
            confirmed = result.get('active') is False
        else:
            state = self.robot.stop_follower()
            confirmed = state.follower_active is False
        if not confirmed:
            raise RuntimeError("Follower stop unconfirmed")
        self.attempted = False
        if self.timed_enabled:
            self._timed_initial_state = None
            self._timed_index = 0
            self._timed_final = False
        return confirmed

    def close(self):
        errors = []
        stopped = False
        try:
            stopped = self.stop()
            if not stopped:
                raise RuntimeError("Follower stop unconfirmed")
        except BaseException as exc:
            errors.append(exc)
        self.quit.set()
        for thread in self.threads:
            thread.join(2)
            if thread.is_alive():
                errors.append(RuntimeError("Gripper worker did not stop"))
        for grip in self.grips.values():
            try:
                grip.disconnect()
            except Exception as exc:
                errors.append(exc)
        if self._speed_restore is not None:
            if stopped:
                try:
                    self._restore_manual_speed()
                except Exception as exc:
                    self.log("manual_speed_restore_failed", error=str(exc))
                    errors.append(exc)
            else:
                self.log("manual_speed_restore_skipped", reason="Follower stop unconfirmed")
        for item in (self.cameras, self.robot):
            try:
                item.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError(f"Hardware cleanup failed: {errors}")


def run(devices, policy, settings, log, *, cycles=2, read_only=False, clock=time):
    """Compatibility adapter; production and offline runs share the framework loop."""
    from types import SimpleNamespace
    from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner
    runner = A10DualInferenceRunner.from_components(
        devices, policy, settings, log, cycles=cycles, read_only=read_only,
        clock=clock, bridge=SimpleNamespace(**globals()))
    return runner.run()


class ZmqPolicy:
    """Native FluxVLA wire adapter; output is already absolute [T,14]."""

    def __init__(self, host, port, timeout_s=120.0, prompt=None):
        import zmq
        from fluxvla.engines.runners.serving.serializers import MsgSerializer

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, int(timeout_s * 1000))
        self.socket.setsockopt(zmq.SNDTIMEO, int(timeout_s * 1000))
        self.socket.connect(f"tcp://{host}:{port}")
        self.prompt = prompt or "Fold the garment neatly using both arms."
        self.metadata = dict(protocol="fluxvla_zmq", host=host, port=port)
        try:
            self.socket.send(MsgSerializer.to_bytes({'endpoint': 'get_status'}))
            status = MsgSerializer.from_bytes(self.socket.recv())
            if isinstance(status, dict) and isinstance(status.get('model'), dict):
                self.metadata['model'] = status['model']
            if isinstance(status, dict) and isinstance(status.get('rtc'), dict):
                self.metadata['rtc'] = status['rtc']
        except BaseException:
            self.close()
            raise

    def infer(self, observation):
        import io
        from fluxvla.engines.runners.serving.serializers import (
            encode_predict_request,
            decode_predict_response,
        )

        obs = dict(
            qpos=observation["observation/state"],
            task_description=observation.get("prompt", self.prompt),
        )
        obs.update({name: observation[f"observation/images/{alias}"] for name, alias in CAMERAS.items()})
        for key in ('a10_rtc_config', 'a10_rtc_actions', 'a10_rtc_execution'):
            if key in observation:
                obs[key] = observation[key]
        self.socket.send(encode_predict_request(obs, "", fmt="msgpack", compress=False))
        response = decode_predict_response(self.socket.recv(), fmt=0)
        if "error" in response:
            raise RuntimeError(f"FluxVLA server error: {response['error']}")
        actions = np.load(io.BytesIO(response["action_data"]), allow_pickle=False)
        if actions.ndim == 3 and actions.shape[0] == 1:
            actions = actions[0]
        return dict(actions=actions, debug=response.get('debug', {}))

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
