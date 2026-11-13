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
    def __init__(self, path):
        self.stream = Path(path).open("x", encoding="utf-8")

    def __call__(self, event, **fields):
        self.stream.write(
            json.dumps(
                plain(dict(event=event, monotonic_s=time.monotonic(), **fields)),
                allow_nan=False,
            )
            + "\n"
        )
        self.stream.flush()

    def close(self):
        self.stream.close()


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
        return self.decode(self.ws.recv(timeout=self.timeout))

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


def actions_from_response(response):
    raw = np.asarray(response["actions"])
    if raw.shape != (30, 14) or raw.dtype.kind not in "fiu" or not np.isfinite(raw).all():
        raise ValueError("Expected finite numeric actions [30,14]")
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

    def stop(self):
        self.active = False
        return True

    def close(self):
        pass


class LiveDevices:
    def __init__(self, config, log, motion=False):
        from fluxvla.engines.operators.a10_operator import A10Operator
        from fluxvla.engines.operators.camera_operator import BufferedCameraOperator
        from fluxvla.engines.operators.zx_gripper import ZXGripper

        self.log, self.motion = log, motion
        self.cfg = config
        settings = dict(config["operator"])
        settings.pop("type", None)
        settings.update(command_mode="joint", allow_motion=motion, cameras=None)
        settings["shared_control"] = dict(settings["shared_control"], role="inference")
        if settings["shared_control"].get("mode") != "managed":
            raise ValueError("Managed shared follower required")
        self.robot = A10Operator(**settings)
        self.cameras = BufferedCameraOperator(config["cameras"])
        self.grips = {}
        self.feedback = {}
        self.error = None
        self.lock = threading.Lock()
        self.target = None
        self.cancel = threading.Event()
        self.cancel.set()
        self.quit = threading.Event()
        self.threads = []
        self.write_locks = {side: threading.Lock() for side in ("left", "right")}
        self.attempted = False
        try:
            self.robot.connect()
            self.cameras.start()
            for side, device in config["grippers"].items():
                check = device["check"]
                if (check["open_raw"], check["close_raw"]) != (0, 9000):
                    raise ValueError("Model requires raw 0..9000 closure calibration")
                identity = device["identity"]
                grip = ZXGripper(
                    device["port"],
                    baudrate=device["baudrate"],
                    timeout=0.05,
                    max_position_raw=9000,
                    expected_usb_serial=identity["serial"],
                    expected_usb_interface=identity["usb_interface"],
                )
                self.grips[side] = grip
                grip.connect()
                thread = threading.Thread(target=self._gripper_worker, args=(side, device), daemon=True)
                self.threads.append(thread)
                thread.start()
            deadline = time.monotonic() + 5
            while True:
                try:
                    observation(self.sample(), config["bridge"])
                    break
                except (RuntimeError, KeyError):
                    if self.error or time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)
        except BaseException:
            self.close()
            raise

    def _gripper_worker(self, side, device):
        grip = self.grips[side]
        try:
            while not self.quit.is_set():
                started = time.monotonic()
                state = grip.read_state(device["device_id"])
                if state.has_error:
                    raise RuntimeError(f"{side} gripper alarm")
                with self.lock:
                    self.feedback[side] = (
                        state.position_raw,
                        time.monotonic(),
                        state.enabled,
                    )
                    target = self.target
                if target is not None and not self.cancel.is_set():
                    row, deadline = target
                    index = 6 if side == "left" else 13
                    if time.monotonic() < deadline:
                        if not state.enabled:
                            raise RuntimeError("Gripper must already be enabled; no automatic enable")
                        with self.write_locks[side]:
                            grip.set_position_raw(
                                device["device_id"],
                                int(round(row[index] * 9000)),
                                speed=device["speed"],
                                torque=device["torque"],
                                feedback=state,
                                motion_deadline_s=deadline,
                                motion_cancel_event=self.cancel,
                            )
                self.quit.wait(max(0, 0.05 - (time.monotonic() - started)))
        except BaseException as exc:
            self.error = exc
            self.cancel.set()

    def sample(self):
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        state = self.robot.get_state()
        images = self.cameras.snapshot(state.received_monotonic_s)
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
        )

    def start(self):
        if not self.motion:
            raise RuntimeError("Read-only mode")
        self.attempted = True
        self.robot.start_follower("joint")
        self.cancel.clear()

    def check(self):
        if self.error:
            raise RuntimeError("Gripper worker failed") from self.error
        observation(self.sample(), self.cfg["bridge"])
        state = self.robot.get_state()
        if state.has_error or state.follower_active is not True or state.follower_mode != "Axis":
            raise RuntimeError("Follower feedback fault/inactive")
        now = time.monotonic()
        with self.lock:
            feedback = dict(self.feedback)
        if any(now - feedback[side][1] > self.cfg["bridge"]["max_gripper_age_s"] for side in ("left", "right")):
            raise RuntimeError("Stale gripper feedback")

    def send(self, row):
        # A10Operator verifies current ownership before every UDP target.
        mapping = {arm.ee_id: name for name, arm in self.robot.get_state().arms.items()}
        if set(mapping) != {0, 1}:
            raise RuntimeError("Missing dual arm mapping")
        self.robot.send_joints({mapping[1]: row[:6], mapping[0]: row[7:13]})
        with self.lock:
            self.target = (row.copy(), time.monotonic() + 0.1)

    def stop(self):
        self.cancel.set()
        with self.lock:
            self.target = None
        # Drain bounded in-flight writes before this cycle can finish/restart.
        for lock in self.write_locks.values():
            with lock:
                pass
        if not self.attempted:
            return True
        state = self.robot.stop_follower()
        confirmed = state.follower_active is False
        if not confirmed:
            raise RuntimeError("Follower stop unconfirmed")
        self.attempted = False
        return confirmed

    def close(self):
        errors = []
        try:
            self.stop()
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
        for item in (self.cameras, self.robot):
            try:
                item.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError(f"Hardware cleanup failed: {errors}")


def run(devices, policy, settings, log, *, cycles=2, read_only=False, clock=time):
    try:
        for cycle in range(cycles):
            sample = devices.sample()
            obs = observation(sample, settings)
            log(
                "observation",
                cycle=cycle,
                state=obs["observation/state"],
                robot_time=sample["robot_time"],
                camera_times=sample["camera_times"],
                gripper_times=sample["gripper_times"],
            )
            started = clock.monotonic()
            response = policy.infer(obs)
            actions, clipped = actions_from_response(response)
            log(
                "prediction",
                cycle=cycle,
                request_s=clock.monotonic() - started,
                raw_actions=response["actions"],
                converted_actions=actions,
                clipped_grippers=clipped,
            )
            if read_only:
                continue
            # Validate against NEW actual state after network inference, not the old observation.
            current = observation(devices.sample(), settings)["observation/state"]
            check_steps(actions, current, settings["max_joint_step_rad"])
            sent = 0
            try:
                devices.start()
                start = clock.monotonic()
                for row in actions[: settings["execute_horizon"]]:
                    clock.sleep(max(0, start + sent * settings["action_dt"] - clock.monotonic()))
                    if clock.monotonic() - (start + sent * settings["action_dt"]) > settings["max_send_lag_s"]:
                        raise RuntimeError("Joint send deadline missed")
                    devices.check()
                    if clock.monotonic() - (start + sent * settings["action_dt"]) > settings["max_send_lag_s"]:
                        raise RuntimeError("Feedback check exceeded joint send deadline")
                    devices.send(row)
                    sent += 1
                    log("sent", cycle=cycle, frame=sent, target=row)
                clock.sleep(max(0, start + sent * settings["action_dt"] - clock.monotonic()))
            finally:
                try:
                    confirmed = devices.stop()
                except BaseException as exc:
                    log("stop", cycle=cycle, sent=sent, confirmed=False, error=str(exc))
                    raise
                log("stop", cycle=cycle, sent=sent, confirmed=confirmed)
                if not confirmed:
                    raise RuntimeError("Follower stop unconfirmed")
        log("complete", cycles=cycles)
    except BaseException as exc:
        log("error", error=str(exc), type=type(exc).__name__)
        raise
    finally:
        try:
            devices.close()
        finally:
            policy.close()


class ZmqPolicy:
    """Native FluxVLA wire adapter; output is already absolute [T,14]."""

    def __init__(self, host, port, timeout_s=120.0, prompt=None):
        import zmq

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, int(timeout_s * 1000))
        self.socket.setsockopt(zmq.SNDTIMEO, int(timeout_s * 1000))
        self.socket.connect(f"tcp://{host}:{port}")
        self.prompt = prompt or "Fold the garment neatly using both arms."
        self.metadata = dict(protocol="fluxvla_zmq", host=host, port=port)

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
        self.socket.send(encode_predict_request(obs, "", fmt="msgpack", compress=False))
        response = decode_predict_response(self.socket.recv(), fmt=0)
        if "error" in response:
            raise RuntimeError(f"FluxVLA server error: {response['error']}")
        actions = np.load(io.BytesIO(response["action_data"]), allow_pickle=False)
        if actions.ndim == 3 and actions.shape[0] == 1:
            actions = actions[0]
        return dict(actions=actions)

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
