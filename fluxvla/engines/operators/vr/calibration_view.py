"""Read-only calibration inspection. No reference governor or motion lifecycle.

The browser receives immutable snapshots over the existing HTTPS service. It
never joins the VR input WebSocket (disconnecting an observer must not release
the controller). All robot IO uses the monitor-only constructor and GET.
"""
import copy
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .control import CalibratedAnchorState, pose, vector
from .runtime import _calibration_state, _load_pose_calibration, _read_only_robot


def orientation_axes(quaternion):
    """Rows are the tool's +X/+Y/+Z directions expressed in robot base."""
    return Rotation.from_quat(quaternion).apply(np.eye(3)).tolist()


class CalibrationView:
    def __init__(self, vr, robot, calibration, metadata):
        self.vr, self.robot = vr, robot
        self.anchor = CalibratedAnchorState(calibration)
        self.metadata = copy.deepcopy(metadata)
        self._lock = threading.Lock()
        self._sample_time = None
        self._sequence = -1
        self._hand = self._ray = None
        self._latest = dict(valid=False, status='等待 VR 和机器人反馈', actual_pose=None,
                            target_pose=None, transmitted=False, read_only=True)
        # Install before VRInput starts HTTPS. Observer access has no input side effects.
        self.vr.input.calibration_view = self

    def snapshot(self):
        with self._lock:
            result = copy.deepcopy(self._latest)
            sample_time = self._sample_time
        result['metadata'] = copy.deepcopy(self.metadata)
        result['snapshot_age_s'] = None if sample_time is None else max(0., time.monotonic() - sample_time)
        # A blocking/failed GET or a stopped producer must never appear live.
        if sample_time is not None and (result['snapshot_age_s'] + result.get('vr_age_s', 0.) > .25):
            result.update(valid=False, status='验证数据已过期，等待新数据', actual_pose=None,
                          target_pose=None, actual_axes=None, target_axes=None,
                          ray_base=None, displacement_mm=None, rotation_error_deg=None)
        return result

    def _publish(self, values):
        with self._lock:
            self._latest = dict(read_only=True, transmitted=False,
                                captured_at=datetime.now(timezone.utc).isoformat(), **values)
            self._sample_time = time.monotonic()

    def step(self):
        """A failed sample clears the current display; the server stays open."""
        raw = {}
        try:
            action = self.vr.get_action()
            raw = self.vr.last_raw
            actual = _calibration_state(self.robot, .25)
            now = time.monotonic()
            received = raw.get('xlevr.sample_receive_time_s')
            if isinstance(received, bool) or not isinstance(received, (float, int)):
                raise ValueError('尚未收到 VR 姿态')
            age = now - received
            if not math.isfinite(age) or not 0 <= age <= .25:
                raise ValueError('VR 姿态过期，等待新数据')
            if raw.get('xlevr.verification_tracking') is not True:
                raise ValueError('手柄尚未有效追踪；请进入 VR。旧页面需更新后重新核对标定坐标系')
            sequence = raw.get('xlevr.sample_sequence')
            if type(sequence) is not int or sequence < 0:
                raise ValueError('VR 样本序号无效')
            if sequence < self._sequence:
                self._sequence = -1
                raise ValueError('VR 序号重置；请松开侧键重新接管，并检查追踪坐标系是否改变')
            if sequence > self._sequence:
                hand = pose(np.r_[vector(raw.get('xlevr.target_position'), 3, 'hand position'),
                                  vector(raw.get('xlevr.orientation_quat'), 4, 'hand quaternion')])
                ray = raw.get('xlevr.ray_direction_local')
                if ray is not None:
                    ray = vector(ray, 3, 'ray direction')
                    length = np.linalg.norm(ray)
                    if length < 1e-6:
                        raise ValueError('射线方向无效')
                    ray = ray / length
                self._hand, self._ray, self._sequence = hand, ray, sequence
            hand = self._hand
            rotation = (self.anchor.base_from_vr * Rotation.from_quat(hand[3:])
                        * self.anchor.hand_from_tool)
            target = np.r_[actual[:3], rotation.as_quat()]
            command = action.get('_xlevr.anchor_command')
            anchored = False
            if raw.get('xlevr.reset_arm'):
                self.anchor.active = False
                self.vr.reset_reference()
            elif isinstance(command, dict):
                state = self.anchor.update(command, actual)
                if state in ('anchored', 'updated', 'duplicate') and self.anchor.active:
                    target[:3] = self.anchor.target[:3]
                    anchored = True
            if not anchored:
                self.anchor.active = False
            ray_base = None
            if self._ray is not None:
                ray_base = (self.anchor.base_from_vr * Rotation.from_quat(hand[3:])).apply(self._ray).tolist()
            self._publish(dict(
                valid=True, status='只读映射中；按住侧键可检查平移' if not anchored else '只读映射中；平移锚点已记录',
                sample_sequence=sequence, vr_age_s=age,
                actual_pose=actual.tolist(), target_pose=target.tolist(),
                actual_axes=orientation_axes(actual[3:]), target_axes=orientation_axes(target[3:]),
                hand_pose_vr=hand.tolist(), ray_direction_local=None if self._ray is None else self._ray.tolist(),
                ray_base=ray_base, translation_anchored=anchored,
                displacement_mm=((target[:3] - self.anchor.robot_anchor[:3]) * 1000).tolist() if anchored else None,
                rotation_error_deg=float((Rotation.from_quat(actual[3:]).inv() * rotation).magnitude() * 180 / np.pi),
                vr_world_up_base=self.anchor.base_from_vr.apply([0., 1., 0.]).tolist()))
        except Exception as error:
            self.anchor.active = False
            self.vr.reset_reference()
            self._publish(dict(valid=False, status=f'{type(error).__name__}: {error}',
                               actual_pose=None, target_pose=None))
        return self.snapshot()

    def run(self, *, max_steps=None):
        # Verification samples at 10 Hz; no 30/500 Hz control loop is needed.
        if max_steps is not None and (type(max_steps) is not int or max_steps < 1):
            raise ValueError('max_steps must be a positive integer or None')
        try:
            self.robot.connect()
            self.vr.connect()
            network = self.vr.input.config.server_settings
            from .monitor import get_local_ip
            host = network['host_ip']
            if host == '0.0.0.0':
                host = get_local_ip()
            print(f'电脑只读验证页：https://{host}:{network["https_port"]}/calibration.html', flush=True)
            print('机械臂须保持 Stopped 且 follower 未运行；侧键仅记录平移锚点。Ctrl+C 关闭。', flush=True)
            index, previous = 0, None
            while max_steps is None or index < max_steps:
                started = time.monotonic()
                result = self.step()
                if result['status'] != previous:
                    print(result['status'], flush=True)
                    previous = result['status']
                index += 1
                time.sleep(max(0., .1 - (time.monotonic() - started)))
        finally:
            self._publish(dict(valid=False, status='只读验证已关闭', actual_pose=None, target_pose=None))
            try:
                self.vr.close()
            finally:
                self.robot.close()  # Close sockets only; no stop/disable commands.


def build_calibration_view(cfg):
    """Construct only. Preview never connects or modifies the calibration file."""
    from fluxvla.engines.utils.builder import build_operator_from_cfg
    settings = copy.deepcopy(dict(cfg))
    settings['pose_mapping'] = 'calibrated_absolute'
    _load_pose_calibration(settings)
    calibration = settings.get('pose_calibration')
    CalibratedAnchorState(calibration)  # Validate before constructing any services.
    calibration = {name: np.asarray(value, dtype=float).tolist() for name, value in calibration.items()}
    if settings['operator'].get('feedback_position_unit') not in ('mm', 'm'):
        raise ValueError('验证需要明确 teleop.operator.feedback_position_unit=mm 或 m')
    input_cfg = dict(settings['input'])
    input_cfg['enable_left_events'] = False
    vr = build_operator_from_cfg(input_cfg)
    robot = _read_only_robot(settings['operator'])  # Whitelist, forced monitor + no motion.
    path = settings.get('pose_calibration_path')
    inline = (cfg.get('pose_calibration') or {}).get('vr_to_robot_quat_xyzw') is not None
    metadata = dict(controller_host=settings['operator'].get('host', '127.0.0.1'),
                    feedback_position_unit=settings['operator']['feedback_position_unit'],
                    arm=input_cfg.get('arm', 'right'), pose_calibration=calibration,
                    calibration_source='inline' if inline else str(path) if path else None,
                    tool_axes_status='未现场确认：默认 TCP +Z 为前方、+X 为上方；请核对后选择',
                    physical_validation='需人工按现场固定方向核对；界面不自动判定通过',
                    vr_reference_space='local-floor；页面重入/重新居中后应重新核对')
    if not inline and path and Path(path).is_file():
        metadata['calibration_sha256'] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return CalibrationView(vr, robot, calibration, metadata)
