"""A10 Cartesian teleoperation lifecycle and config assembly, without project imports."""

from __future__ import annotations

import math
import time

import numpy as np
from scipy.spatial.transform import Rotation


class VRTeleopRuntime:
    """Run the already-portable XLeVR input against an A10Operator instance.

    ``robot`` may be omitted for VR-only diagnostics. ``execute`` controls
    whether any controller connection or motion command is made. A separate
    GripperOperator can be supplied for thumbstick velocity control.
    """

    def __init__(self, vr, mapper=None, robot=None, *, execute=False, gripper=None,
                 diagnostics_path=None,
                 max_position_step_m=None, max_rotation_step_rad=None,
                 max_feedback_age_s=0.25, gripper_speed_mm_s=30.0,
                 max_position_tracking_error_m=0.05,
                 max_rotation_tracking_error_rad=0.35):
        if type(execute) is not bool:
            raise ValueError('execute must be bool')
        if execute:
            if robot is None or not getattr(robot, 'allow_motion', False):
                raise ValueError('Live VR requires an A10Operator with allow_motion=true')
            if getattr(robot, 'command_mode', None) != 'cartesian':
                raise ValueError('Live VR requires the Cartesian follower')
            if getattr(robot, 'feedback_position_unit', None) not in ('mm', 'm'):
                raise ValueError('Confirm A10 feedback_position_unit as mm or m')
            for name, value in [('max_position_step_m', max_position_step_m),
                                ('max_rotation_step_rad', max_rotation_step_rad)]:
                if value is None or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'Configure a positive finite {name} for live VR')
        if not math.isfinite(max_feedback_age_s) or max_feedback_age_s <= 0:
            raise ValueError('max_feedback_age_s must be positive and finite')
        if not math.isfinite(gripper_speed_mm_s) or gripper_speed_mm_s <= 0:
            raise ValueError('gripper_speed_mm_s must be positive and finite')
        for name, value in [('max_position_tracking_error_m', max_position_tracking_error_m),
                            ('max_rotation_tracking_error_rad', max_rotation_tracking_error_rad)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        self.teleoperator = vr if mapper is None else None
        if self.teleoperator is not None:
            vr, mapper = self.teleoperator.input, self.teleoperator.mapper
        self.vr, self.mapper, self.robot, self.gripper = vr, mapper, robot, gripper
        self.diagnostics_path = diagnostics_path
        self._diagnostics = None
        self.last_frame = self.last_relative_target = None
        self._frame_index = 0
        self.execute = execute
        self.max_position_step_m = max_position_step_m
        self.max_rotation_step_rad = max_rotation_step_rad
        self.max_feedback_age_s = max_feedback_age_s
        self.gripper_speed_mm_s = gripper_speed_mm_s
        self.max_position_tracking_error_m = max_position_tracking_error_m
        self.max_rotation_tracking_error_rad = max_rotation_tracking_error_rad
        self._target_pose = None
        self._follower_anchor_pose = None
        self._engaged = False
        self._started_follower = False
        self._connected = False
        self._last_gripper_time = None

    def connect(self):
        """Start VR input and read-only robot connection; defer follower start."""
        if self._connected:
            raise RuntimeError('VR operator is already connected')
        try:
            self.mapper.reset()
            if self.diagnostics_path:
                from .diagnostics import XLeVRDiagnosticsWriter
                self._diagnostics = XLeVRDiagnosticsWriter(self.diagnostics_path)
            if self.teleoperator is not None:
                self.teleoperator.connect()
            else:
                self.vr.connect()
            if self.execute:
                self.robot.connect()
                self._checked_frame(require_active=False)
                if self.gripper is not None:
                    self.gripper.connect()
            self._connected = True
        except BaseException:
            self.close()
            raise
        return self

    def _checked_frame(self, *, require_active):
        frame = self.robot.get_frame()
        self.last_frame = frame
        if not isinstance(frame, dict) or frame.get('has_error') is not False:
            raise RuntimeError('A10 feedback is missing or reports a fault')
        stamp = frame.get('received_monotonic_s')
        if (stamp is None or not math.isfinite(stamp) or stamp > time.monotonic()
                or time.monotonic() - stamp > self.max_feedback_age_s):
            raise RuntimeError('A10 feedback is stale or has no valid timestamp')
        pose = np.asarray(frame.get('tcp_pq_m'), dtype=float)
        if (pose.shape != (7,) or not np.isfinite(pose).all()
                or not np.isclose(np.linalg.norm(pose[3:]), 1., atol=1e-3)):
            raise RuntimeError('A10 TCP feedback must be a finite metre/xyzw pose')
        if require_active and (frame.get('follower_active') is not True
                               or frame.get('follower_mode') != 'Cart'):
            raise RuntimeError('A10 Cartesian follower is not confirmed active')
        return pose

    def _absolute_target(self, pose, action):
        translation = np.asarray([action[f'ee.delta_{axis}'] for axis in 'xyz'],
                                 dtype=float)
        rotation = np.asarray([action[f'ee.delta_r{axis}'] for axis in 'xyz'],
                              dtype=float)
        if (not np.isfinite(translation).all() or not np.isfinite(rotation).all()
                or np.linalg.norm(translation) > self.max_position_step_m
                or np.linalg.norm(rotation) > self.max_rotation_step_rad):
            raise ValueError('VR delta is nonfinite or exceeds the configured step limit')
        current = Rotation.from_quat(pose[3:])
        target = np.concatenate((pose[:3] + current.apply(translation),
                                 (current * Rotation.from_rotvec(rotation)).as_quat()))
        return target

    def _send_absolute_target(self, target):
        """Express a base-frame TCP target in the fixed follower anchor frame."""
        if self._follower_anchor_pose is None:
            raise RuntimeError('Follower startup anchor is unavailable')
        anchor = self._follower_anchor_pose
        inverse_anchor = Rotation.from_quat(anchor[3:]).inv()
        relative = np.concatenate((inverse_anchor.apply(target[:3] - anchor[:3]),
                                   (inverse_anchor * Rotation.from_quat(target[3:])).as_quat()))
        self.robot.send_eepose({self.robot.arm_name: relative})
        self.last_relative_target = relative.copy()

    def _check_tracking(self, pose, target):
        position_error = np.linalg.norm(target[:3] - pose[:3])
        rotation_error = (Rotation.from_quat(pose[3:]).inv()
                          * Rotation.from_quat(target[3:])).magnitude()
        if (position_error > self.max_position_tracking_error_m
                or rotation_error > self.max_rotation_tracking_error_rad):
            raise RuntimeError('VR target exceeds the configured tracking error limit')

    def _hold(self):
        """Replace queued motion with actual TCP once, then repeat that hold."""
        if self._started_follower:
            pose = self._checked_frame(require_active=True)
            if self._engaged or self._target_pose is None:
                self._target_pose = pose.copy()
            self._check_tracking(pose, self._target_pose)
            self._send_absolute_target(self._target_pose)
        self._engaged = False

    def _send_gripper(self, action):
        if self.gripper is None:
            return
        velocity = float(action.get('gripper.pos', 0.0))
        if not math.isfinite(velocity) or abs(velocity) > 1:
            raise ValueError('VR gripper input must be within [-1, 1]')
        now = time.monotonic()
        elapsed = 0. if self._last_gripper_time is None else min(
            now - self._last_gripper_time, self.max_feedback_age_s)
        self._last_gripper_time = now
        if abs(velocity) < 1e-6 or elapsed <= 0:
            return
        measured = self.gripper.get_position_mm()
        target = min(self.gripper.max_width_mm, max(
            0., measured + velocity * self.gripper_speed_mm_s * elapsed))
        self.gripper.set_position_mm(target)

    def step(self):
        if not self._connected or not self.vr.is_connected:
            raise ConnectionError('VR input is not connected')
        self.last_relative_target = None
        if self.teleoperator is not None:
            action = self.teleoperator.get_action()
            raw = self.teleoperator.last_raw
        else:
            raw = self.vr.get_action()
            action = self.mapper(raw)
        started = time.monotonic()
        error = None
        try:
            return self._step(raw, action)
        except BaseException as exc:
            error = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            if self._diagnostics is not None:
                getter = getattr(self.mapper, 'get_last_diagnostics', None)
                record = dict(getter() or {}) if getter else {}
                record.update(control_frame_index=self._frame_index,
                              raw_input=raw, mapped_action=action,
                              feedback=self.last_frame,
                              sent_relative_target=self.last_relative_target,
                              step_elapsed_s=time.monotonic() - started,
                              error=error, execute=self.execute)
                self._diagnostics.write(record)
            self._frame_index += 1

    def _step(self, raw, action):
        """Accumulate tool deltas; release/reset/stale input replaces motion with hold."""
        if not self._connected or not self.vr.is_connected:
            raise ConnectionError('VR input is not connected')
        if raw.get('xlevr.reset_arm'):
            # LeRobot's reset/home command has no verified A10Operator equivalent.
            self.mapper.reset()
            if self.execute:
                self._hold()
            self._last_gripper_time = None
            return action
        if not self.execute:
            return action
        if not action.get('ee.enabled', False):
            self._hold()
            # Mapper arm guards retain the thumbstick value. Never integrate a
            # stale browser sample into another gripper command.
            age = raw.get('xlevr.sample_age_s')
            if age is None or (math.isfinite(age) and 0 <= age <= self.mapper.stale_timeout_s):
                self._send_gripper(action)
            else:
                self._last_gripper_time = None
            return action
        pose = self._checked_frame(require_active=self._started_follower)
        # Validate before starting or updating the controller.
        target = self._absolute_target(
            self._target_pose if self._engaged else pose, action)
        self._check_tracking(pose, target)
        if not self._started_follower:
            self.robot.set_tool_coordinate()
            self.robot.start_follower('cartesian')
            self._started_follower = True
            pose = self._checked_frame(require_active=True)
            # No UDP target has been sent yet; startup requires Stopped, so
            # the confirmed TCP is the follower's startup anchor.
            self._follower_anchor_pose = pose.copy()
            # WS mode selection can block for seconds. Discard the pre-start
            # sample and re-anchor VR after it, so startup cannot replay motion.
            self.mapper.reset()
            self._target_pose = pose.copy()
            self._engaged = False
            self._send_absolute_target(pose)
            self._last_gripper_time = None
            return action
        self._send_absolute_target(target)
        self._target_pose = target
        self._engaged = True
        self._send_gripper(action)
        return action

    def run(self, *, fps=30, max_steps=None):
        """Poll until left-stick stop, Ctrl+C, or an optional test step limit."""
        if type(fps) is not int or fps <= 0:
            raise ValueError('fps must be a positive integer')
        if max_steps is not None and (type(max_steps) is not int or max_steps <= 0):
            raise ValueError('max_steps must be a positive integer')
        try:
            self.connect()
            count = 0
            while max_steps is None or count < max_steps:
                started = time.monotonic()
                events = (self.teleoperator.get_events() if self.teleoperator is not None
                          else self.vr.get_vr_events())
                if events.get('stop_recording'):
                    break
                action = self.step()
                count += 1
                if not self.execute and count % fps == 0:
                    print({key: action.get(key) for key in (
                        'ee.enabled', 'ee.delta_x', 'ee.delta_y', 'ee.delta_z',
                        'ee.delta_rx', 'ee.delta_ry', 'ee.delta_rz',
                        'gripper.pos')}, flush=True)
                time.sleep(max(0., 1. / fps - (time.monotonic() - started)))
            return count
        except KeyboardInterrupt:
            return 0
        finally:
            self.close()

    def close(self):
        """Request follower stop before closing links; preserve stop errors."""
        try:
            if self.execute and self.robot is not None and (
                    self._started_follower or self.robot.follower_stop_pending):
                self.robot.stop_follower()
        finally:
            self._started_follower = False
            self._connected = False
            self._last_gripper_time = None
            self._target_pose = None
            self._follower_anchor_pose = None
            self._engaged = False
            try:
                if self.gripper is not None:
                    self.gripper.disconnect()
            finally:
                try:
                    if self.robot is not None:
                        self.robot.close()
                finally:
                    try:
                        if self.teleoperator is not None:
                            self.teleoperator.close()
                        else:
                            self.vr.disconnect()
                    finally:
                        if self._diagnostics is not None:
                            self._diagnostics.close()
                            self._diagnostics = None


def build_runtime(cfg):
    """Construct from a native teleop config; no services open until run()."""
    from fluxvla.engines.utils.builder import build_operator_from_cfg
    settings = dict(cfg)
    vr_only = settings.pop('vr_only', True)
    if type(vr_only) is not bool:
        raise ValueError('vr_only must be bool')
    vr = build_operator_from_cfg(settings.pop('input'))
    robot_cfg = dict(settings.pop('operator'))
    settings.pop('fps', None)
    settings.pop('max_steps', None)
    gripper = None
    robot = None
    if not vr_only:
        # Use the current A10 lifecycle/feedback and the shared USB gripper driver.
        # A separate gripper instance permits joystick operation before arm engagement.
        if robot_cfg.get('gripper_transport') == 'usb':
            from ..gripper_operator import GripperOperator
            gripper = GripperOperator(**dict(robot_cfg.pop('gripper')))
            robot_cfg['gripper_transport'] = 'controller'
        robot = build_operator_from_cfg(robot_cfg)
    return VRTeleopRuntime(vr, robot=robot, execute=not vr_only,
                          gripper=gripper, **settings)


def build_vr(cfg):
    """Adapt legacy recording settings to the single native VR implementation."""
    from dataclasses import fields
    from ..vr_operator import VROperator
    from .mapper import XLeVRDeltaEEMapper
    allowed = {f.name for f in fields(XLeVRDeltaEEMapper) if f.init}
    options = cfg['vr']
    operator = VROperator(network=cfg['vr_server'],
                          mapping={k: v for k, v in options.items() if k in allowed},
                          arm=options['arm'],
                          enable_left_events=options['enable_left_events'],
                          thumbstick_event_threshold=options['thumbstick_event_threshold'])
    return operator.input, operator.mapper
