"""A10 Cartesian teleoperation lifecycle and config assembly, without project imports."""

from __future__ import annotations

import math
import logging
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
                 max_rotation_tracking_error_rad=0.35,
                 control_mode='delta', reference_options=None, observe_only=False):
        if type(execute) is not bool:
            raise ValueError('execute must be bool')
        if control_mode not in ('delta', 'reference'):
            raise ValueError('control_mode must be delta or reference')
        if type(observe_only) is not bool:
            raise ValueError('observe_only must be bool')
        if observe_only and (execute or control_mode != 'reference' or robot is None or gripper is not None):
            raise ValueError('observe_only requires reference mode, a read-only robot and no gripper')
        if observe_only and getattr(robot, 'allow_motion', False):
            raise ValueError('observe_only requires allow_motion=false')
        if (execute or observe_only) and getattr(robot, 'feedback_position_unit', None) not in ('mm', 'm'):
            raise ValueError('Confirm A10 feedback_position_unit as mm or m')
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
        self.control_mode, self.observe_only = control_mode, observe_only
        self.reference_control = None
        if control_mode == 'reference':
            from .control import ReferenceConfig, VRPoseController
            if (getattr(mapper, 'position_control_mode', None) != 'safe_frame_delta'
                    or getattr(mapper, 'motion_shaping_mode', None) != 'robot_controller'
                    or not getattr(mapper, 'compute_anchor_shadow', False)):
                raise ValueError('reference mode requires safe_frame_delta, anchor output, and mapper shaping bypass')
            self.reference_control = VRPoseController(ReferenceConfig(**dict(reference_options or {})))
        elif reference_options:
            raise ValueError('reference_options requires control_mode=reference')
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
        self.last_reference_result = None
        self.last_stop_status = None
        self._reference_time = None
        self._input_read_time = None
        self._reference_stop_attempted = False
        self._require_release = False

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
            if self.execute or self.observe_only:
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
        self.last_relative_target = None
        self.last_reference_result = None
        started = time.monotonic()
        raw, action = {}, {}
        error = None
        try:
            if not self._connected or not self.vr.is_connected:
                raise ConnectionError('VR input is not connected')
            if self.teleoperator is not None:
                action = self.teleoperator.get_action()
                raw = self.teleoperator.last_raw
            else:
                raw = self.vr.get_action()
                action = self.mapper(raw)
            self._input_read_time = time.monotonic()
            return self._step(raw, action)
        except BaseException as exc:
            error = f'{type(exc).__name__}: {exc}'
            if self.control_mode == 'reference' and self.execute:
                self._require_release = True
                try:
                    self._stop_reference()
                except Exception as stop_error:
                    error += f'; stop unconfirmed: {stop_error}'
                    raise RuntimeError(error) from exc
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
                              error=error, execute=self.execute,
                              cartesian_wire_format=getattr(self.robot, 'cartesian_wire_format', None),
                              control_mode=self.control_mode, observe_only=self.observe_only,
                              reference_control=self.last_reference_result,
                              stop_status=self.last_stop_status)
                self._diagnostics.write(record)
            self._frame_index += 1

    def _step(self, raw, action):
        """Accumulate tool deltas; release/reset/stale input replaces motion with hold."""
        if not self._connected or not self.vr.is_connected:
            raise ConnectionError('VR input is not connected')
        if self.control_mode == 'reference':
            return self._step_reference(raw, action)
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

    def _stop_reference(self):
        """Stop this session once, then confirm reported Stopped separately."""
        if self.observe_only or not self.execute:
            return
        pending = self._started_follower or self.robot.follower_stop_pending
        if not pending or self._reference_stop_attempted:
            return
        self._reference_stop_attempted = True
        self.last_stop_status = dict(requested=True, mode_inactive=False, reported_stopped=False)
        try:
            self.robot.stop_follower()
            self._started_follower = False
            self.last_stop_status['mode_inactive'] = True
            self.robot.wait_stopped()
            self.last_stop_status['reported_stopped'] = True
            logging.getLogger(__name__).info('参考跟随停止确认：inactive + Stopped')
        except BaseException as exc:
            self.last_stop_status['error'] = f'{type(exc).__name__}: {exc}'
            logging.getLogger(__name__).error(
                '机械臂停止未确认：%s；请在控制器现场确认停止，勿重新启动遥操作。', exc)
            raise
        finally:
            self._reference_time = None
            self._target_pose = self._follower_anchor_pose = None
            self._engaged = False
            self._last_gripper_time = None

    def _step_reference(self, raw, action):
        """Read-only shadow or the explicitly selected new UDP control path."""
        if not self.execute and not self.observe_only:
            return action  # VR-only still has no fabricated robot feedback.
        command = action.get('_xlevr.anchor_command')
        if not isinstance(command, dict):
            raise ValueError('reference mode requires mapper _xlevr.anchor_command')
        active = command.get('active') is True
        if raw.get('xlevr.reset_arm') or action.get('_xlevr.reset_arm'):
            self._stop_reference()
            self.mapper.reset()
            self._require_release = True
            self.last_reference_result = dict(state='release_required', relative_target=None)
            return action
        if self._require_release:
            age = raw.get('xlevr.sample_age_s')
            fresh = (age is not None and math.isfinite(age) and 0 <= age
                     <= min(self.mapper.stale_timeout_s, self.reference_control.config.max_input_age_s))
            if fresh and not active and not raw.get('xlevr.enabled', False):
                self._require_release = False
            self.last_reference_result = dict(state='release_required', relative_target=None)
            return action
        pose = self._checked_frame(require_active=self.execute and self._started_follower)
        raw_age = raw.get('xlevr.sample_age_s')
        input_bad = (raw_age is None or not math.isfinite(raw_age) or raw_age < 0
                     or raw_age + max(0., time.monotonic() - self._input_read_time)
                     > min(self.mapper.stale_timeout_s, self.reference_control.config.max_input_age_s))
        # Mapper deactivation due to stale/invalid input is not a fresh operator
        # release. Otherwise its inactive anchor packet could hide a fault and
        # allow an automatic restart while the hand is still squeezing.
        if self._reference_time is not None and (input_bad or
                (not active and raw.get('xlevr.enabled', False))):
            reason = ('control_gap' if time.monotonic() - self._reference_time > self.reference_control.config.max_dt_s
                      else 'input_stale' if input_bad else 'mapper_input_rejected')
            self.last_reference_result = dict(state='fault', fault_reason=reason,
                                             relative_target=None, transmitted=False)
            raise RuntimeError(f'VR reference fault: {reason}')
        if self._reference_time is None:
            if not active or not action.get('ee.enabled', False):
                self.last_reference_result = dict(state='idle', relative_target=None)
                if self.execute:
                    self._send_fresh_gripper(raw, action)
                return action
            age = raw.get('xlevr.sample_age_s')
            if age is None or not math.isfinite(age) or age < 0:
                raise ValueError('Active VR reference requires a valid input age')
            age += max(0., time.monotonic() - self._input_read_time)
            if age > min(self.mapper.stale_timeout_s, self.reference_control.config.max_input_age_s):
                raise RuntimeError('VR reference input_stale before follower start')
            # Validate all command fields before changing any controller state.
            from .control import AnchorState
            AnchorState().update(command, pose)
            if self.execute:
                self.robot.set_tool_coordinate()
                self._reference_stop_attempted = False
                self.last_stop_status = None
                self.robot.start_follower('cartesian')
                self._started_follower = True
                pose = self._checked_frame(require_active=True)
            self._follower_anchor_pose = pose.copy()
            self.reference_control.start(pose)
            self.mapper.reset()  # Never replay a sample captured before blocking WS setup.
            self._reference_time = time.monotonic()
            self._target_pose = pose.copy()
            self.last_reference_result = dict(state='await_fresh_sample',
                                             relative_target=None if self.observe_only else [0., 0., 0., 0., 0., 0., 1.],
                                             transmitted=False,
                                             startup_anchor_assumed=self.observe_only)
            if self.execute:
                self._send_absolute_target(pose)
                self.last_reference_result['transmitted'] = True
            return action
        # A fresh VR anchor appearing during a movement is not permission to
        # reset command velocity or jump the target to measured feedback.
        key = (command.get('session_id'), command.get('anchor_id'))
        previous = self.reference_control.anchor.key
        if active and previous is not None and key != previous:
            self._stop_reference()
            self._reference_time = None
            self.mapper.reset()
            self._require_release = True
            self.last_reference_result = dict(state='release_required', relative_target=None,
                                             reason='anchor_changed_while_engaged')
            return action
        now = time.monotonic()
        age = raw.get('xlevr.sample_age_s')
        if active and (age is None or not math.isfinite(age) or age < 0):
            raise ValueError('Active VR reference requires a valid input age')
        age = 0. if age is None or not math.isfinite(age) or age < 0 else age
        age += max(0., now - self._input_read_time)
        result = self.reference_control.step(
            command, pose, now - self._reference_time,
            sample_age_s=age, feedback_age_s=now - self.last_frame['received_monotonic_s'],
            has_error=self.last_frame['has_error'])
        result.update(observe_only=self.observe_only, transmitted=False,
                      feedback_clock='client_receive', startup_anchor_assumed=self.observe_only)
        self.last_reference_result = result
        self._reference_time = now
        if result['stop_requested']:
            self._stop_reference()
            self._reference_time = None
            self.mapper.reset()
            if result['state'] == 'fault':
                raise RuntimeError(f"VR reference fault: {result['fault_reason']}")
            return action
        target = np.asarray(result['command_pose'])
        self._check_tracking(pose, target)
        previous_target = self._target_pose
        if self.execute:
            distance = np.linalg.norm(target[:3] - previous_target[:3])
            angle = (Rotation.from_quat(previous_target[3:]).inv() * Rotation.from_quat(target[3:])).magnitude()
            if distance > self.max_position_step_m or angle > self.max_rotation_step_rad:
                raise ValueError('Reference command exceeds the configured step limit')
            self.robot.send_eepose({self.robot.arm_name: result['relative_target']})
            self.last_relative_target = np.array(result['relative_target'])
            result['transmitted'] = True
            self._send_fresh_gripper(raw, action)
        self._target_pose = target.copy()
        self._engaged = True
        return action

    def _send_fresh_gripper(self, raw, action):
        age = raw.get('xlevr.sample_age_s')
        if age is not None and math.isfinite(age) and 0 <= age <= self.mapper.stale_timeout_s:
            self._send_gripper(action)
        else:
            self._last_gripper_time = None

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
                if self.control_mode == 'reference':
                    self._stop_reference()
                else:
                    self.robot.stop_follower()
        finally:
            self._started_follower = False
            self._connected = False
            self._last_gripper_time = None
            self._target_pose = None
            self._follower_anchor_pose = None
            self._engaged = False
            self._reference_time = None
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
    observe_only = settings.get('observe_only', False)
    if observe_only and vr_only:
        raise ValueError('observe_only requires vr_only=false to read real controller feedback')
    if not vr_only:
        # Use the current A10 lifecycle/feedback and the shared USB gripper driver.
        # A separate gripper instance permits joystick operation before arm engagement.
        if observe_only:
            if robot_cfg.get('cameras') or robot_cfg.get('gripper') or robot_cfg.get('gripper_transport') == 'usb':
                raise ValueError('observe_only does not open cameras or gripper devices')
            robot_cfg['allow_motion'] = False
            robot_cfg['ws_port'] = robot_cfg.get('status_ws_port', 5888)
            robot_cfg['status_poll_interval_s'] = None
        elif robot_cfg.get('gripper_transport') == 'usb':
            from ..gripper_operator import GripperOperator
            gripper = GripperOperator(**dict(robot_cfg.pop('gripper')))
            robot_cfg['gripper_transport'] = 'controller'
        robot = build_operator_from_cfg(robot_cfg)
    return VRTeleopRuntime(vr, robot=robot, execute=not vr_only and not observe_only,
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


def inspect_controller(operator_cfg, *, samples=10, interval_s=.1):
    """Read GET only, using the monitor endpoint; never create input/accessories.

    This characterizes replies and client latency, not hidden follower dynamics
    or the controller's actual sampling age. Unknown capabilities stay unknown.
    """
    from ..a10_operator import A10Operator
    if type(samples) is not int or not 1 <= samples <= 100:
        raise ValueError('samples must be an integer within 1..100')
    if not math.isfinite(interval_s) or interval_s <= 0:
        raise ValueError('interval_s must be positive and finite')
    # Only transport/decoding fields enter this instance. Camera/gripper/motion
    # options in another workflow must never leak into a read-only inspection.
    allowed = ('host', 'timeout_s', 'udp_port', 'joint_udp_port',
               'feedback_joint_unit', 'feedback_position_unit')
    options = {key: operator_cfg[key] for key in allowed if key in operator_cfg}
    options.update(ws_port=operator_cfg.get('status_ws_port', 5888), allow_motion=False)
    robot = A10Operator(**options)
    rows = []
    error = None
    attempts = 0
    try:
        robot.connect()
        for index in range(samples):
            started = time.monotonic()
            attempts += 1
            state = robot.get_state()
            elapsed = time.monotonic() - started
            context = state.raw['ret_context']
            rows.append(dict(index=index, query_elapsed_s=elapsed,
                             received_monotonic_s=state.received_monotonic_s,
                             has_error=state.has_error, follower_active=state.follower_active,
                             follower_mode=state.follower_mode,
                             motion=context.get('robot_msg', {}).get('motion'),
                             activation=context.get('robot_msg', {}).get('activate'),
                             jog_coordinate=context.get('motion_msg', {}).get('jog_coordinate'),
                             actual_joints_rad=None if state.actual_joints_rad is None else state.actual_joints_rad.tolist(),
                             tcp_pq_raw=None if state.tcp_pq_raw is None else state.tcp_pq_raw.tolist(),
                             tcp_pq_m=None if state.tcp_pq_m is None else state.tcp_pq_m.tolist()))
            if index + 1 < samples:
                time.sleep(interval_s)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        robot.close()
    return dict(read_only=True, query_success=error is None and len(rows) == samples,
                error=error, get_attempts=attempts, completed_queries=len(rows),
                commands=['get'] if attempts else [], udp_targets_sent=0,
                host=options['host'], monitor_port=options['ws_port'],
                feedback_position_unit=options.get('feedback_position_unit'),
                controller_sample_age_verified=False,
                unknown_capabilities=['cartesian_smoothing', 'target_queue',
                                      'packet_ordering', 'stream_timeout_behavior'],
                max_query_elapsed_s=max((row['query_elapsed_s'] for row in rows), default=None), samples=rows)
