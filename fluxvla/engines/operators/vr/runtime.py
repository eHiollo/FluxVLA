"""A10 Cartesian teleoperation lifecycle and config assembly, without project imports."""

from __future__ import annotations

import math
import logging
import time
import json
import threading
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


class _ZXGripperChannel:
    """One USB owner, bounded target lead, latest input only; no foreground I/O."""

    def __init__(self, side, device, options):
        from ..zx_gripper import ZXGripper, has_blocking_alarm
        self.side = side
        self.options = dict(options)
        self.device_id = device['device_id']
        if type(self.device_id) is not int or not 1 <= self.device_id <= 247:
            raise ValueError('ZX device_id must be within 1..247')
        identity, check = device['identity'], device['check']
        self.low, self.high = check['min_raw'], check['max_raw']
        self.opened, self.closed = check['open_raw'], check['close_raw']
        if (any(type(v) is not int for v in (self.low, self.high, self.opened, self.closed))
                or not 0 <= self.low < self.high <= 0xFFFFFFFF
                or not self.low <= self.opened <= self.high or not self.low <= self.closed <= self.high
                or self.opened == self.closed):
            raise ValueError('Confirm ZX gripper raw range and distinct open_raw/close_raw')
        if not identity.get('serial') or not identity.get('usb_interface'):
            raise ValueError('ZX grippers require USB serial and interface identity')
        self.serial = identity['serial']
        serial_timeout = options.get('serial_timeout_s', .2)
        if (isinstance(serial_timeout, bool) or not isinstance(serial_timeout, (int, float))
                or not math.isfinite(serial_timeout) or not .02 <= serial_timeout <= .2):
            raise ValueError('ZX serial_timeout_s must be within .02..0.2 seconds')
        self.tolerance = check.get('tolerance_raw', 5)
        if type(self.tolerance) is not int or not 0 <= self.tolerance <= self.high - self.low:
            raise ValueError('Invalid ZX raw position tolerance')
        self.speed, self.torque = device['speed'], device['torque']
        if (type(self.speed) is not int or not 1 <= self.speed <= 50 or
                type(self.torque) is not int or not 1 <= self.torque <= 60):
            raise ValueError('ZX teleop requires integer speed within 1..50% and torque within 1..60%')
        self.direction = options['direction'][side]
        if self.direction not in (-1, 1) or isinstance(self.direction, bool):
            raise ValueError('ZX direction must be 1 or -1 for each hand')
        self.drop_alarm_policy = options.get('drop_alarm_policy', 'stop')
        has_blocking_alarm(0, 0, drop_alarm_policy=self.drop_alarm_policy)
        self.client = ZXGripper(device['port'], baudrate=device['baudrate'], timeout=serial_timeout,
                                max_position_raw=self.high, drop_alarm_policy=self.drop_alarm_policy, expected_usb_serial=self.serial,
                                expected_usb_interface=identity['usb_interface'])
        self._lock = threading.Lock()
        self._stop, self._ready = threading.Event(), threading.Event()
        self._motion_cancel = threading.Event()
        self._reset_requested = threading.Event()
        self._disable_requested = threading.Event()
        self._thread = None
        self._input = (0., 0.)
        self._target = None
        self._last_tick = None
        self._need_center = True
        self._active = False
        self._last_sent = None
        self._last_link_warning = float('-inf')
        self._state = dict(phase='not_started', serial=self.serial, actual_raw=None,
                           target_raw=None, last_sent_raw=None, error=None, communication_error=None, feedback_monotonic_s=None)

    def connect(self):
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError('ZX channel cannot restart; create a new teleop session')
        self._thread = threading.Thread(target=self._run, name='zx-gripper-' + self.side, daemon=True)
        self._thread.start()
        if not self._ready.wait(3.):
            self.request_stop()
            raise TimeoutError(f'ZX {self.side} startup did not complete')
        self.check_health()
        if self.snapshot().get('error'):
            logging.getLogger(__name__).warning('ZX %s 启动时不可用；保留VR界面并禁止该夹爪目标：%s',
                                                 self.side, self.snapshot()['error'])

    def submit(self, axis, deadline):
        if not math.isfinite(axis) or abs(axis) > 1 or not math.isfinite(deadline):
            raise ValueError('ZX joystick must be finite within [-1, 1], with a finite deadline')
        with self._lock:
            if not self._stop.is_set():
                self._input = (float(axis), float(deadline))
                if abs(axis) <= self.options['deadzone'] and deadline > time.monotonic():
                    self._motion_cancel.clear()  # A fresh neutral sample rearms USB commands.

    def pause(self):
        self._motion_cancel.set()  # Also cancels a target waiting inside the serial driver.
        with self._lock:
            self._input = (0., 0.)
            self._need_center, self._active = True, False

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def check_health(self):
        state = self.snapshot()
        if state['error']:
            return False  # A failed accessory must not tear down the recording UI.
        if state.get('communication_error'):
            return False
        stamp = state['feedback_monotonic_s']
        if stamp is None or not 0 <= time.monotonic() - stamp <= self.options['feedback_timeout_s']:
            self._communication_failed('feedback is stale')
            return False
        return True

    def request_alarm_action(self, action):
        """Explicit operator request; serial writes stay on the owning worker."""
        if (action not in ('reset', 'disable') or self._stop.is_set()
                or self._thread is None or not self._thread.is_alive()):
            raise RuntimeError('Invalid action or gripper worker is not running')
        self.pause()
        (self._reset_requested if action == 'reset' else self._disable_requested).set()

    def _communication_failed(self, error):
        # Only this gripper pauses. Never discard CRC or fabricate feedback,
        # release holding torque, replay a write, or stop the arm VR loop.
        self.pause()
        self._last_sent = None
        with self._lock:
            self._state.update(phase='communication_wait', communication_error=str(error),
                               last_io_error=str(error), last_sent_raw=None,
                               crc_errors=getattr(self.client, 'crc_error_count', 0),
                               crc_read_retries=getattr(self.client, 'crc_read_retry_count', 0),
                               last_crc_error=getattr(self.client, 'last_crc_error', self._state.get('last_crc_error')))
        now = time.monotonic()
        if now - self._last_link_warning >= 5.:
            self._last_link_warning = now
            logging.getLogger(__name__).warning(
                'ZX %s 通信异常，仅暂停此夹爪，机械臂遥操作继续；反馈恢复后摇杆回中再操作：%s',
                self.side, error)

    def request_stop(self):
        self._stop.set()
        self._motion_cancel.set()
        with self._lock:
            self._input = (0., 0.)

    def close(self):
        self.request_stop()
        if self._thread is not None:
            self._thread.join(2.)
            if self._thread.is_alive():
                raise RuntimeError(f'ZX {self.side} serial worker did not finish; close unconfirmed')

    def _choose_target(self, actual, now, axis, deadline):
        dt = 0. if self._last_tick is None else max(0., min(now - self._last_tick, self.options['input_timeout_s']))
        self._last_tick = now
        if now >= deadline:
            self._need_center, self._active = True, False
            self._target = float(actual)
            return None, 'input_stale'
        if abs(axis) <= self.options['deadzone']:
            self._need_center, self._active = False, False
            self._target = float(actual)
            return None, 'centered'
        if self._need_center:
            return None, 'neutral_required'
        if not self._active:
            self._target, self._active = float(actual), True
            return None, 'engaging'
        sign = 1 if self.opened > self.closed else -1
        desired = self._target + axis * self.direction * sign * self.options['target_rate_raw_s'] * dt
        lead = self.options['max_lead_raw']
        self._target = max(self.low, min(self.high, max(actual - lead, min(actual + lead, desired))))
        return int(round(self._target)), 'tracking'

    def _read(self):
        before = time.monotonic()
        state = self.client.read_state(self.device_id)
        now = time.monotonic()
        if now - before > self.options['feedback_timeout_s']:
            from ..zx_gripper import ZXGripperTimeout
            raise ZXGripperTimeout('serial feedback read took too long')
        if not max(0, self.low - self.tolerance) <= state.position_raw <= self.high + self.tolerance:
            raise RuntimeError('actual position exceeds the configured raw range')
        warning = ('drop detected' if self.drop_alarm_policy == 'warn' and state.alarm_b & 0x1000 else None)
        previous_warning = self.snapshot().get('warning')
        if warning and not previous_warning:
            logging.getLogger(__name__).warning('ZX %s 掉落报警仅提示，继续接受摇杆目标；原始报警仍记录。', self.side)
        with self._lock:
            self._state.update(drop_alarm_policy=self.drop_alarm_policy, warning=warning, actual_raw=state.position_raw, feedback_monotonic_s=now,
                               enabled=state.enabled, alarm_a=state.alarm_a, alarm_b=state.alarm_b, communication_error=None,
                               status=state.status, speed_raw=state.speed_raw, current_raw=state.current_raw,
                               completion=state.completion,
                               crc_errors=getattr(self.client, 'crc_error_count', 0),
                               crc_read_retries=getattr(self.client, 'crc_read_retry_count', 0),
                               last_crc_error=getattr(self.client, 'last_crc_error', self._state.get('last_crc_error')))
        return state, now

    def _run(self):
        from ..zx_gripper import ZXGripperInputExpired, ZXGripperCommunicationError, has_blocking_alarm
        try:
            with self.client:
                next_tick = time.monotonic()
                while not self._stop.wait(max(0., next_tick - time.monotonic())):
                    next_tick = time.monotonic() + 1. / self.options['io_hz']
                    if self._disable_requested.is_set() or self._reset_requested.is_set():
                        disable = self._disable_requested.is_set()
                        request = self._disable_requested if disable else self._reset_requested
                        request.clear()
                        try:
                            if disable:
                                self.client.disable(self.device_id)
                            else:
                                self.client.reset_alarms(self.device_id)
                                with self._lock:
                                    self._state.update(alarm_latched=False, error=None, reset_error=None)
                            self._target = self._last_sent = None
                            self.pause()
                        except Exception as exc:
                            with self._lock:
                                self._state.update(reset_error=str(exc))
                            logging.getLogger(__name__).error('ZX %s 人工%s未完成：%s',
                                self.side, '掉使能' if disable else '清报警', exc)
                    try:
                        state, now = self._read()
                    except ZXGripperCommunicationError as exc:
                        self._communication_failed(exc)
                        self._ready.set()
                        continue  # Poll feedback only; no uncertain write is retried.
                    blocking_alarm = has_blocking_alarm(state.alarm_a, state.alarm_b,
                                                        drop_alarm_policy=self.drop_alarm_policy)
                    if blocking_alarm or self.snapshot().get('alarm_latched'):
                        first = not self.snapshot().get('alarm_latched')
                        self.pause()
                        with self._lock:
                            self._state.update(phase='alarm_latched', alarm_latched=True,
                                error=(f'gripper alarm 0x{state.alarm_a:04X}/0x{state.alarm_b:04X}'
                                       if blocking_alarm else self._state['error']))
                        self._ready.set()
                        if first:
                            logging.getLogger(__name__).error('ZX %s 硬件报警，停止新目标并保持只读查询：%s',
                                self.side, self.snapshot()['error'])
                        continue  # Never auto-clear, disable, or replay an old position.
                    if self._target is None:
                        self._target = float(np.clip(state.position_raw, self.low, self.high))
                    self._ready.set()
                    if self._stop.is_set():
                        break
                    with self._lock:
                        axis, deadline = self._input
                    target, phase = self._choose_target(state.position_raw, now, axis, deadline)
                    if target is not None and target != self._last_sent:
                        try:
                            self.client.set_position_raw(self.device_id, target, speed=self.speed, torque=self.torque,
                                                         motion_deadline_s=deadline, motion_cancel_event=self._motion_cancel, feedback=state)
                            self._last_sent = target
                        except ZXGripperCommunicationError as exc:
                            self._communication_failed(exc)
                            self._target = float(state.position_raw)
                            continue
                        except ZXGripperInputExpired:
                            self._need_center, self._active = True, False
                            phase = 'input_expired_during_io'
                    with self._lock:
                        self._state.update(phase=phase, target_raw=self._target, last_sent_raw=self._last_sent,
                                           input_axis=axis, input_deadline_s=deadline)
        except Exception as exc:
            with self._lock:
                self._state.update(phase='failed', error=f'{type(exc).__name__}: {exc}')
            self._stop.set()
        finally:
            self._ready.set()


class _VRFeedbackStale(RuntimeError):
    """Measured robot feedback aged out; not a controller fault or bad packet."""


class _VRInputPause(RuntimeError):
    """Only known input/timing guards may enter recoverable pause."""
    def __init__(self, arm, reason, *, details=None):
        self.arm, self.reason = arm, reason
        self.details = dict(details or {})
        super().__init__(f'Dual VR {arm}: {reason}')


class VRTeleopRuntime:
    """Run the already-portable XLeVR input against an A10Operator instance.

    ``robot`` may be omitted for VR-only diagnostics. ``execute`` controls
    whether any controller connection or motion command is made. A separate
    GripperOperator can be supplied for thumbstick velocity control.
    """

    def __init__(self, vr, mapper=None, robot=None, *, execute=False, gripper=None,
                 diagnostics_path=None,
                 max_position_step_m=None, max_rotation_step_rad=None,
                 max_feedback_age_s=0.25, gripper_speed_mm_s=30.0, gripper_direction=1,
                 max_position_tracking_error_m=0.05,
                 max_rotation_tracking_error_rad=0.35,
                 control_mode='delta', reference_options=None, observe_only=False,
                 pose_mapping='anchor_relative', pose_calibration=None,
                 controller_speed_percent=None, cartesian_adapter='legacy_anchor',
                 record_controller=False, keep_session_on_query_error=False, confirm_stop=True,
                 resilient_takeover=False):
        if cartesian_adapter not in ('legacy_anchor', 'sdk_tool_cumulative'):
            raise ValueError('Unknown cartesian_adapter')
        if cartesian_adapter == 'sdk_tool_cumulative':
            if control_mode != 'reference':
                raise ValueError('SDK tool adapter requires reference mode')
            if robot is not None and getattr(robot, 'cartesian_wire_format', None) != 'pe321':
                raise ValueError('SDK tool adapter requires cartesian_wire_format=pe321')
        if type(record_controller) is not bool:
            raise ValueError('record_controller must be bool')
        if record_controller and (robot is None or not (execute or observe_only) or not diagnostics_path):
            raise ValueError('record_controller requires live/observe feedback and a diagnostics_path')
        self.record_controller = record_controller
        if type(confirm_stop) is not bool:
            raise ValueError('confirm_stop must be bool')
        self.confirm_stop = confirm_stop
        if type(resilient_takeover) is not bool:
            raise ValueError('resilient_takeover must be bool')
        self.resilient_takeover = resilient_takeover
        self._takeover_wait_reason = None
        if type(keep_session_on_query_error) is not bool:
            raise ValueError('keep_session_on_query_error must be bool')
        self.keep_session_on_query_error = keep_session_on_query_error
        self.communication_pause = None
        self.shared_control_pause = None
        self.last_tracking_warning = None
        self._tracking_warning_logged = False
        self.controller_recording = dict(state='disabled' if not record_controller else 'pending')
        self._recording_next_heartbeat = None
        self._recording_last_dropped = 0
        self.cartesian_adapter = cartesian_adapter
        self.last_sdk_command = None
        if controller_speed_percent is not None and (
                type(controller_speed_percent) is not int or not 1 <= controller_speed_percent <= 100):
            raise ValueError('controller_speed_percent must be None or an integer within 1..100')
        if type(execute) is not bool:
            raise ValueError('execute must be bool')
        if control_mode not in ('delta', 'reference'):
            raise ValueError('control_mode must be delta or reference')
        if pose_mapping not in ('anchor_relative', 'calibrated_absolute', 'world_translation_local_rotation'):
            raise ValueError(f'Unknown pose_mapping: {pose_mapping}')
        if pose_mapping != 'anchor_relative' and control_mode != 'reference':
            raise ValueError('Selected pose_mapping requires control_mode=reference')
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
            if control_mode == 'delta':
                for name, value in [('max_position_step_m', max_position_step_m),
                                    ('max_rotation_step_rad', max_rotation_step_rad)]:
                    if value is None or not math.isfinite(value) or value <= 0:
                        raise ValueError(f'Configure a positive finite {name} for live delta VR')
        if not math.isfinite(max_feedback_age_s) or max_feedback_age_s <= 0:
            raise ValueError('max_feedback_age_s must be positive and finite')
        if not math.isfinite(gripper_speed_mm_s) or gripper_speed_mm_s <= 0:
            raise ValueError('gripper_speed_mm_s must be positive and finite')
        if type(gripper_direction) not in (int, float) or gripper_direction not in (-1, 1):
            raise ValueError('gripper_direction must be 1 or -1')
        for name, value in [('max_position_tracking_error_m', max_position_tracking_error_m),
                            ('max_rotation_tracking_error_rad', max_rotation_tracking_error_rad)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        self.teleoperator = vr if mapper is None else None
        if self.teleoperator is not None:
            vr, mapper = self.teleoperator.input, self.teleoperator.mapper
        self.vr, self.mapper, self.robot, self.gripper = vr, mapper, robot, gripper
        self.control_mode, self.observe_only = control_mode, observe_only
        self.pose_mapping = pose_mapping
        self.reference_control = None
        if control_mode == 'reference':
            from .control import ReferenceConfig, VRPoseController
            if (getattr(mapper, 'position_control_mode', None) != 'safe_frame_delta'
                    or getattr(mapper, 'motion_shaping_mode', None) != 'robot_controller'
                    or not getattr(mapper, 'compute_anchor_shadow', False)):
                raise ValueError('reference mode requires safe_frame_delta, anchor output, and mapper shaping bypass')
            mapper.anchor_translation_frame = ('robot_base'
                if pose_mapping == 'world_translation_local_rotation' else 'tool_anchor')
            self.reference_control = VRPoseController(
                ReferenceConfig(**dict(reference_options or {})),
                pose_mapping=pose_mapping, pose_calibration=pose_calibration)
        elif reference_options:
            raise ValueError('reference_options requires control_mode=reference')
        self.diagnostics_path = diagnostics_path
        self._diagnostics = None
        self.last_frame = self.last_relative_target = None
        self._frame_index = 0
        self.last_io_timing = {}
        self.execute = execute
        self.max_position_step_m = max_position_step_m
        self.max_rotation_step_rad = max_rotation_step_rad
        self.max_feedback_age_s = max_feedback_age_s
        self.gripper_speed_mm_s = gripper_speed_mm_s
        self.gripper_direction = gripper_direction
        self.last_gripper_command = None
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
        self.controller_speed_percent = controller_speed_percent
        self._speed_restore = None
        self.speed_status = dict(requested_percent=controller_speed_percent, state='unchanged')

    def _recording_event(self):
        if self._diagnostics is not None:
            self._diagnostics.write(dict(event='controller_recording',
                                         controller_recording=self.controller_recording))

    def _timed_call(self, stage, function, *args, **kwargs):
        """Measure blocking work without adding requests or changing deadlines."""
        started = time.monotonic()
        try:
            return function(*args, **kwargs)
        finally:
            self.last_io_timing[stage] = self.last_io_timing.get(stage, 0.) + time.monotonic() - started

    def _controller_recording_request(self, *, start=False):
        before = dict(client_send_monotonic_ns=time.monotonic_ns(), client_send_wall_ns=time.time_ns())
        try:
            status = (self.robot.start_runtime_recording() if start
                      else self.robot.heartbeat_runtime_recording())
            timing = dict(before, client_receive_monotonic_ns=time.monotonic_ns(),
                          client_receive_wall_ns=time.time_ns())
            self.controller_recording['last_reply'] = status
            old_session = self.controller_recording.get('session_id')
            # A shorter server timeout cannot be serviced at the one-second cadence.
            if status['heartbeat_timeout_ms'] <= 2000:
                raise RuntimeError('Controller recording heartbeat timeout must exceed 2000 ms')
            if (not status['recording'] or not status['connected'] or status['closing'] or status['error']
                    or (not start and status['session_id'] != old_session)):
                raise RuntimeError(f'Controller recording interrupted: {status}')
            self.controller_recording.update(state='recording', session_id=status['session_id'],
                                             file=status['file'], status=status, timing=timing)
            self._recording_next_heartbeat = time.monotonic() + 1.
            if start:
                logging.getLogger(__name__).info('控制器高频记录已开始：session=%s，file=%s',
                                                 status['session_id'], status['file'])
            if status['dropped'] > self._recording_last_dropped:
                logging.getLogger(__name__).warning('控制器高频记录丢样：%s', status['dropped'])
            self._recording_last_dropped = status['dropped']
        except BaseException as exc:
            self.controller_recording.update(state='failed', error=f'{type(exc).__name__}: {exc}', timing=before)
            logging.getLogger(__name__).error('控制器高频记录未确认：%s', exc)
            raise
        finally:
            self._recording_event()

    def _poll_controller_recording(self):
        if self.record_controller and self._recording_next_heartbeat is not None:
            if time.monotonic() >= self._recording_next_heartbeat:
                self._controller_recording_request()

    def _speed_event(self):
        logging.getLogger(__name__).info('控制器倍率：%s', self.speed_status)
        if self._diagnostics is not None:
            self._diagnostics.write(dict(event='controller_speed_override',
                                         controller_speed_override=self.speed_status))

    def _apply_speed_override(self):
        if self.controller_speed_percent is None:
            return
        if not self.execute:
            self.speed_status['state'] = 'ignored_read_only'
            self._speed_event()
            return
        try:
            original = self.robot.get_speed_override()
            if original['percent'] == self.controller_speed_percent:
                self.speed_status.update(original=original, state='confirmed',
                                         confirmed=dict(original), already_set=True)
                self._speed_event()
                return
            self.speed_status.update(original=original, state='setting')
            # A timed-out write may already have applied: retain restoration intent.
            if original['percent'] != self.controller_speed_percent:
                self._speed_restore = original
            confirmed = self.robot.set_speed_override(
                self.controller_speed_percent, expected_mode=original['mode'])
            self.speed_status.update(state='confirmed', confirmed=confirmed)
        except BaseException as exc:
            self.speed_status.update(state='set_failed', error=str(exc))
            self._speed_event()
            raise
        self._speed_event()

    def _restore_speed_override(self):
        original, self._speed_restore = self._speed_restore, None
        if original is None:
            return
        if (not self.confirm_stop and self.last_stop_status
                and not self.last_stop_status.get('reported_stopped')):
            self.speed_status['state'] = 'restore_skipped_stop_unconfirmed'
            self._speed_event()
            return
        try:
            # Never reconnect or overwrite a different mode/operator's new setting.
            if not self.robot.is_connected:
                raise RuntimeError('连接已断开，未恢复倍率')
            current = self.robot.get_speed_override()
            if current['mode'] != original['mode'] or current['percent'] not in (
                    self.controller_speed_percent, original['percent']):
                raise RuntimeError('控制器模式或倍率已被改变，未覆盖现场设置')
            self.robot.set_speed_override(original['percent'], expected_mode=original['mode'])
            self.speed_status['state'] = 'restored'
        except Exception as exc:
            self.speed_status.update(state='restore_unconfirmed', restore_error=str(exc))
            logging.getLogger(__name__).error('倍率恢复未确认：%s', exc)
        self._speed_event()

    def _cancel_accessories(self):
        pass

    def connect(self):
        """Connect input/feedback, optionally set speed, and defer follower start."""
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
                if self.record_controller:
                    self._controller_recording_request(start=True)
                self._checked_frame(require_active=False)
            lease = self.robot.get_shared_control_status() if (
                self.execute and getattr(self.robot, 'shared_control', None)) else None
            if lease is not None and not lease['owns'] and not lease['available']:
                self._pause_shared_control()
            else:
                self._apply_speed_override()
            if self.execute or self.observe_only:
                if self.gripper is not None:
                    self.gripper.connect()
            self._connected = True
        except BaseException:
            self.close()
            raise
        return self

    def _pause_shared_control(self):
        """The relay already stopped the old owner; never stop the new one."""
        if self.shared_control_pause is None:
            self._cancel_accessories()
            recording = getattr(self, 'recording', None)
            if recording is not None and recording.state == 'recording':
                recording.fail('Console took control; episode interrupted')
            self.robot.get_shared_control_status()
            self._speed_restore = None  # Console now owns settings; old VR cleanup must not overwrite them.
            self.shared_control_pause = dict(state='console_control')
            self._started_follower = False
            self._reference_time = None
            if hasattr(self, '_dual_time'):
                self._dual_time = None
            self._follower_anchor_pose = None
            if self.teleoperator is not None:
                self.teleoperator.reset_reference()
            self._shared_control_hud(paused=True)
            logging.getLogger(__name__).info('控制台已接管：VR界面保留；控制台停止后，松开侧键再按可重新接管。')
        self.last_reference_result = dict(state='console_control', arms={}, transmitted=False)

    def _shared_control_hud(self, *, paused):
        if self.teleoperator is not None:
            try:
                self.teleoperator.send_feedback(dict(type='hud',
                    phase='console_control' if paused else 'idle', error='',
                    hint='CONSOLE HAS CONTROL' if paused else 'VR READY - PRESS SIDE GRIP',
                    controls=dict(visible=False, ready=not paused)))
            except Exception:
                logging.getLogger(__name__).warning('控制权状态提示发送失败', exc_info=True)

    def _step_shared_control(self, raw):
        if not self.execute or not getattr(self.robot, 'shared_control', None):
            return False
        lease = self.robot.get_shared_control_status()
        if (not lease.get('preempted') and (lease['owns'] or lease['available'])
                and self.shared_control_pause is None):
            return False
        self._pause_shared_control()
        samples = list(raw.values()) if getattr(self.teleoperator, 'is_dual', False) else [raw]
        released = bool(samples) and all(
            not (sample.get('xlevr.enabled') or sample.get('xlevr.grip_active')
                 or sample.get('xlevr.buttons', {}).get('squeeze')) for sample in samples)
        # An unused, untracked hand must not prevent a fresh release of the
        # active hand from returning control. Active-hand validation is unchanged.
        fresh_release = released and any(
            isinstance(sample.get('xlevr.sample_age_s'), (int, float))
            and 0 <= sample['xlevr.sample_age_s'] <= self.max_feedback_age_s
            for sample in samples)
        if lease['available'] and fresh_release:
            self.robot.acknowledge_shared_takeover()
            self.shared_control_pause = None
            self._require_release = False
            if hasattr(self, '_dual_require_release'):
                self._dual_require_release = False
            self._reference_stop_attempted = False
            self.last_reference_result = dict(state='idle', arms={}, transmitted=False)
            self._shared_control_hud(paused=False)
            logging.getLogger(__name__).info('VR控制权可用：下一次按侧键时重新取实际位姿，不重发旧目标。')
        return True

    def _checked_frame(self, *, require_active):
        frame = self._timed_call('robot_feedback_s', self.robot.get_frame)
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
        if self.cartesian_adapter == 'sdk_tool_cumulative':
            self.last_sdk_command = self.robot.send_sdk_tool_pose(
                target, max_position_step_m=None, max_rotation_step_rad=None)
            self.last_relative_target = None  # Cumulative 321 is not a relative pose.
        else:
            self.robot.send_eepose({self.robot.arm_name: relative})
            self.last_relative_target = relative.copy()

    def _check_tracking(self, pose, target):
        position_error = np.linalg.norm(target[:3] - pose[:3])
        rotation_error = (Rotation.from_quat(pose[3:]).inv()
                          * Rotation.from_quat(target[3:])).magnitude()
        if (position_error > self.max_position_tracking_error_m
                or rotation_error > self.max_rotation_tracking_error_rad):
            if self.reference_control is not None and not self.reference_control.config.enforce_tracking_error_limits:
                self.last_tracking_warning = dict(position_error_m=float(position_error),
                                                  rotation_error_rad=float(rotation_error))
                if not self._tracking_warning_logged:
                    logging.getLogger(__name__).info(
                        '跟踪误差参考（不限制、不锁定）：位置%.4fm，姿态%.4frad',
                        position_error, rotation_error)
                    self._tracking_warning_logged = True
                return
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

    def _send_gripper(self, action, *, input_deadline=None):
        if self.gripper is None:
            return
        velocity = float(action.get('gripper.pos', 0.0)) * self.gripper_direction
        if not math.isfinite(velocity) or abs(velocity) > 1:
            raise ValueError('VR gripper input must be within [-1, 1]')
        now = time.monotonic()
        elapsed = 0. if self._last_gripper_time is None else min(
            now - self._last_gripper_time, self.max_feedback_age_s)
        self._last_gripper_time = now
        if abs(velocity) < 1e-6 or elapsed <= 0:
            return
        started = time.monotonic()
        measured = self.gripper.get_position_mm()
        if input_deadline is not None and time.monotonic() > input_deadline:
            self._last_gripper_time = None
            self.last_gripper_command = dict(measured_mm=measured, transmitted=False,
                                            reason='input_expired_during_read',
                                            io_elapsed_s=time.monotonic() - started)
            return
        target = min(self.gripper.max_width_mm, max(
            0., measured + velocity * self.gripper_speed_mm_s * elapsed))
        self.gripper.set_position_mm(target)
        self.last_gripper_command = dict(measured_mm=measured, target_mm=target,
                                        transmitted=True, arrival_confirmed=False,
                                        io_elapsed_s=time.monotonic() - started)

    def step(self):
        self.last_sdk_command = None
        self.last_relative_target = None
        self.last_reference_result = None
        self.last_tracking_warning = None
        started = time.monotonic()
        started_wall_ns = time.time_ns()
        self.last_io_timing = {}
        self.last_gripper_command = None
        recording = getattr(self, 'recording', None)
        raw, action = {}, {}
        error = None
        try:
            if not self._connected or not self.vr.is_connected:
                raise ConnectionError('VR input is not connected')
            if self.communication_pause is None and (recording is None or not recording.blocks_motion):
                self._timed_call('controller_recording_s', self._poll_controller_recording)
            if self.teleoperator is not None:
                action = self.teleoperator.get_action()
                raw = self.teleoperator.last_raw
            else:
                raw = self.vr.get_action()
                action = self.mapper(raw)
            self._input_read_time = time.monotonic()
            if self.communication_pause is not None:
                # Keep VR/HUD and diagnostics alive, but never replay commands,
                # poll the closed socket, or restart with keys still held.
                self.last_reference_result = dict(state='communication_paused', arms={},
                                                 transmitted=False)
                return action
            latency = getattr(self.vr, 'latency', None)
            if latency is not None:
                latency.consume(raw, self._input_read_time)
            if self._timed_call('shared_control_s', self._step_shared_control, raw):
                return action
            blocked = recording.before_step(raw, action) if recording is not None else False
            result = action if blocked else self._step(raw, action)
            if recording is not None: recording.after_step()
            return result
        except BaseException as exc:
            from ...communication.a10_shared_connection import ControlTransferredError
            transferred = isinstance(exc, ControlTransferredError)
            if (not transferred and isinstance(exc, Exception) and self.execute
                    and getattr(self.robot, 'shared_control', None) and self.robot.is_connected):
                try:
                    transferred = self.robot.get_shared_control_status().get('preempted', False)
                except Exception:
                    pass  # Preserve the original transport/controller failure.
            if transferred:
                if recording is not None and recording.state == 'recording':
                    recording.fail('Console took control; episode interrupted')
                self._pause_shared_control()
                return action
            if recording is not None and recording.state == 'recording':
                recording.fail(f'Control interrupted: {exc}')
            self._cancel_accessories()
            error = f'{type(exc).__name__}: {exc}'
            failure = getattr(self.robot, 'last_communication_failure', None)
            if (self.keep_session_on_query_error and isinstance(exc, Exception)
                    and failure is not None and failure.get('failed_wall_ns', 0) >= started_wall_ns
                    and failure.get('command') in ('get', 'feedback_snapshot', 'record_heartbeat')):
                if recording is not None:
                    recording.reset_cancel.set()
                self.communication_pause = dict(reason=error, restart_required=True,
                                                stop_confirmed=False)
                try:
                    if self.execute:
                        self._stop_reference()
                    self.communication_pause['stop_confirmed'] = bool(
                        self.last_stop_status and self.last_stop_status.get('reported_stopped'))
                except Exception as stop_error:
                    self.communication_pause['stop_error'] = str(stop_error)
                self._recording_next_heartbeat = None
                self.last_reference_result = dict(state='communication_paused', arms={},
                                                 transmitted=False)
                logging.getLogger(__name__).error(
                    '控制器查询失败，VR界面保留，已暂停新目标；停止确认=%s。'
                    '%s',
                    self.communication_pause['stop_confirmed'],
                    '请在现场确认停止，Ctrl+C退出后重新连接。')
                if self.teleoperator is not None:
                    try:
                        self.teleoperator.send_feedback(dict(
                            type='hud', phase='communication_paused',
                            hint='CONTROLLER QUERY FAILED - MOTION PAUSED',
                            error='Confirm robot stopped onsite. Ctrl+C to exit and reconnect.'))
                    except Exception:
                        logging.getLogger(__name__).warning('通信暂停提示发送失败', exc_info=True)
                return action
            if recording is not None and recording.blocks_motion:
                recording.reset_cancel.set()
                if recording.reset_thread is not None:
                    self.robot.cancel_current_command()
            elif self.control_mode == 'reference' and self.execute:
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
                if getattr(self.teleoperator, 'is_dual', False):
                    record = dict(hands={hand: mapper.get_last_diagnostics() for hand, mapper
                                         in self.teleoperator.mappers.items()})
                record.update(control_frame_index=self._frame_index,
                              raw_input=raw, mapped_action=action,
                              feedback=self.last_frame,
                              sent_relative_target=self.last_relative_target,
                              step_elapsed_s=time.monotonic() - started,
                              requested_control_hz=getattr(self, '_requested_control_hz', None),
                              requested_feedback_hz=getattr(self, 'feedback_poll_hz', None),
                              io_timing_s=dict(self.last_io_timing),
                              error=error, execute=self.execute,
                              cartesian_adapter=self.cartesian_adapter,
                              sdk_tool_command=self.last_sdk_command,
                              gripper_command=self.last_gripper_command,
                              cartesian_wire_format=getattr(self.robot, 'cartesian_wire_format', None),
                              control_mode=self.control_mode, observe_only=self.observe_only,
                              pose_mapping=self.pose_mapping,
                              pose_calibration=(None if self.reference_control is None else
                                                self.reference_control.pose_calibration),
                              reference_control=self.last_reference_result,
                              controller_speed_override=self.speed_status,
                              controller_recording=self.controller_recording,
                              input_pause=getattr(self, "input_pause_status", None),
                              dataset_recording=recording.status() if recording is not None else None,
                              client_monotonic_ns=time.monotonic_ns(), client_wall_ns=time.time_ns(),
                              stop_status=self.last_stop_status)
                if self.last_tracking_warning is not None:
                    record['tracking_limit_warning'] = dict(self.last_tracking_warning)
                if self.communication_pause is not None:
                    record['communication_pause'] = dict(self.communication_pause)
                if self.shared_control_pause is not None:
                    record['shared_control_pause'] = dict(self.shared_control_pause)
                failure = getattr(self.robot, 'last_communication_failure', None)
                if failure is not None:
                    record['communication_failure'] = failure
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
            self._send_fresh_gripper(raw, action, allow_missing_age=True)
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
        self._send_fresh_gripper(raw, action, allow_missing_age=True)
        return action

    def _stop_reference(self):
        """Stop targets once; optionally wait for native inactive/Stopped feedback."""
        if self.observe_only or not self.execute:
            return
        pending = self._started_follower or self.robot.follower_stop_pending
        if not pending or self._reference_stop_attempted:
            return
        self._reference_stop_attempted = True
        self.last_stop_status = dict(requested=True, mode_inactive=False, reported_stopped=False)
        try:
            if self.confirm_stop:
                self._timed_call('stop_request_s', self.robot.stop_follower)
                self._started_follower = False
                self.last_stop_status['mode_inactive'] = True
                self._timed_call('stop_confirm_s', self.robot.wait_stopped)
                self.last_stop_status['reported_stopped'] = True
                logging.getLogger(__name__).info('参考跟随停止确认：inactive + Stopped')
            else:
                self._timed_call('stop_request_s', self.robot.request_stop_follower)
                self._started_follower = False
                self.last_stop_status['state'] = 'stop_requested'
                logging.getLogger(__name__).info('已发送停止跟随请求，不等待执行结果或停止回读')
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
            self.last_sdk_command = None

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
            self.reference_control.validate_command(command, pose)
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
                if self.cartesian_adapter == 'sdk_tool_cumulative':
                    self.robot.begin_sdk_tool_trajectory(pose)
                self._send_absolute_target(pose)
                if self.cartesian_adapter == 'sdk_tool_cumulative':
                    self.last_reference_result['relative_target'] = None
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
        if self.execute:
            # ReferenceConfig constrains speed and acceleration using actual dt.
            # A longer valid tick must not trip an unrelated fixed-step bound.
            self._send_absolute_target(target)
            if self.cartesian_adapter == 'sdk_tool_cumulative':
                result['relative_target'] = None
            result['transmitted'] = True
            self._send_fresh_gripper(raw, action)
        self._target_pose = target.copy()
        self._engaged = True
        return action

    def _send_fresh_gripper(self, raw, action, *, allow_missing_age=False):
        age = raw.get('xlevr.sample_age_s')
        if age is None and allow_missing_age:
            age = 0.  # Compatibility with the delta input contract.
        budget = self.mapper.stale_timeout_s
        if self.reference_control is not None:
            budget = min(budget, self.reference_control.config.max_input_age_s)
        elapsed = 0. if self._input_read_time is None else max(0., time.monotonic() - self._input_read_time)
        if age is not None and math.isfinite(age) and 0 <= age and age + elapsed <= budget:
            self._send_gripper(action, input_deadline=time.monotonic() + budget - age - elapsed)
        else:
            self._last_gripper_time = None

    def run(self, *, fps=30, max_steps=None):
        """Poll until left-stick stop, Ctrl+C, or an optional test step limit."""
        if type(fps) is not int or fps <= 0:
            raise ValueError('fps must be a positive integer')
        if max_steps is not None and (type(max_steps) is not int or max_steps <= 0):
            raise ValueError('max_steps must be a positive integer')
        self._requested_control_hz = fps
        try:
            self.connect()
            recording = getattr(self, 'recording', None)
            if recording is not None: recording.connect()
            next_tick = time.monotonic()
            count = 0
            while max_steps is None or count < max_steps:
                started = time.monotonic()
                events = (self.teleoperator.get_events() if self.teleoperator is not None
                          else self.vr.get_vr_events())
                if events.get('stop_recording'):
                    break
                action = self.step()
                count += 1
                if recording is not None and recording.quit_requested: break
                if recording is None and not self.execute and count % fps == 0:
                    if getattr(self.teleoperator, 'is_dual', False):
                        print({arm: {'enabled': value.get('ee.enabled'),
                                     'anchor': value.get('_xlevr.anchor_command')}
                               for arm, value in action.items()}, flush=True)
                    else:
                        print({key: action.get(key) for key in (
                            'ee.enabled', 'ee.delta_x', 'ee.delta_y', 'ee.delta_z',
                            'ee.delta_rx', 'ee.delta_ry', 'ee.delta_rz',
                            'gripper.pos')}, flush=True)
                if recording is None:
                    time.sleep(max(0., 1. / fps - (time.monotonic() - started)))
                else:
                    next_tick += 1. / fps
                    if next_tick < time.monotonic(): next_tick = time.monotonic()
                    time.sleep(max(0., next_tick-time.monotonic()))
            return count
        except KeyboardInterrupt:
            return 0
        finally:
            recording = getattr(self, 'recording', None)
            try:
                if recording is not None: recording.before_close()
            finally:
                try: self.close()
                finally:
                    if recording is not None: recording.close()

    def close(self):
        """Request follower stop before closing links; preserve stop errors."""
        try:
            if self.execute and self.robot is not None and (
                    self._started_follower or self.robot.follower_stop_pending):
                if self.control_mode == 'reference':
                    self._stop_reference()
                else:
                    self.robot.stop_follower()
                    if self._speed_restore is not None:
                        self.robot.wait_stopped()
        finally:
            self._restore_speed_override()
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
                        try:
                            self.robot.close()  # Disconnect ends the remote recording; no record_stop exists.
                        finally:
                            if self.record_controller and self.controller_recording['state'] not in (
                                    'pending', 'disconnected_save_unconfirmed'):
                                self._recording_next_heartbeat = None
                                self.controller_recording.update(state='disconnected_save_unconfirmed')
                                self._recording_event()
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


class DualVRTeleopRuntime(VRTeleopRuntime):
    """Two independent clutches/controllers, one whole-group transport/lifecycle.

    Optional ZX grippers own independent USB workers. No dual pose calibration, IK or collision avoidance. A normal per-arm
    release shapes a bounded command stop; faults stop the entire group.
    """

    def __init__(self, vr, robot=None, *, gripper=None, zx_grippers=None,
                 input_recovery_stable_s=.5, stop_on_both_release=True, feedback_poll_hz=None,
                 continuous_feedback=True, **settings):
        from .control import VRPoseController
        if (not getattr(vr, 'is_dual', False) or settings.get('control_mode') != 'reference'
                or settings.get('pose_mapping', 'anchor_relative') not in (
                    'anchor_relative', 'world_translation_local_rotation')
                or gripper is not None):
            raise ValueError('Dual VR requires both hands, reference and a relative pose mapping; use zx_grippers for dual USB devices')
        self.arm_names = tuple(vr.hand_to_arm.values())
        if robot is not None and (not getattr(robot, 'is_multi_arm', False)
                                  or set(robot.arm_names) != set(self.arm_names)):
            raise ValueError('VR bindings must match the complete dual A10 arm group')
        super().__init__(vr, robot=robot, gripper=None, **settings)
        if type(stop_on_both_release) is not bool:
            raise ValueError('stop_on_both_release must be bool')
        self.stop_on_both_release = stop_on_both_release
        if type(continuous_feedback) is not bool:
            raise ValueError('continuous_feedback must be bool')
        self.continuous_feedback = continuous_feedback
        self._sent_reference_poses = None
        self._last_target_send_s = None
        if not continuous_feedback:
            cfg = self.reference_control.config
            if (self.observe_only or self.cartesian_adapter != 'sdk_tool_cumulative'
                    or cfg.enforce_tracking_error_limits or cfg.max_reference_lead_m is not None
                    or cfg.max_reference_lead_rad is not None):
                raise ValueError('Command-only control requires SDK cumulative mode without measured tracking/lead limits')
        if feedback_poll_hz is not None and (isinstance(feedback_poll_hz, bool)
                or not isinstance(feedback_poll_hz, (float, int))
                or not math.isfinite(feedback_poll_hz) or not 1 <= feedback_poll_hz <= 90):
            raise ValueError('feedback_poll_hz must be None or within 1..90 Hz')
        self.feedback_poll_hz = feedback_poll_hz
        self._feedback_polling = False
        if not self.reference_control.config.limit_acceleration:
            raise ValueError('Dual release requires acceleration limiting')
        for mapper in vr.mappers.values():
            mapper.anchor_translation_frame = self.mapper.anchor_translation_frame
        self._arms = {arm: dict(hand=hand, mapper=vr.mappers[hand],
                               control=VRPoseController(self.reference_control.config,
                                                       pose_mapping=self.pose_mapping),
                               state='holding', key=None, target=None)
                      for hand, arm in vr.hand_to_arm.items()}
        self._dual_time = None
        self._dual_require_release = False
        self._dual_stop_failed = False
        if (isinstance(input_recovery_stable_s, bool) or not isinstance(input_recovery_stable_s, (int, float))
                or not math.isfinite(input_recovery_stable_s) or not .5 <= input_recovery_stable_s <= 5):
            raise ValueError('input_recovery_stable_s must be within .5..5 seconds')
        self.input_recovery_stable_s = input_recovery_stable_s
        self._input_pause = None
        self._pause_samples = {}
        self._pause_feedback_stamp = None
        self._feedback_resume_after_s = None
        self._pause_stable_since = None
        self._pause_hud_signature = None
        self._zx_channels = {}
        config = dict(zx_grippers or {})
        enabled = config.pop('enabled', False)
        devices = config.pop('devices', {})
        if type(enabled) is not bool:
            raise ValueError('zx_grippers.enabled must be bool')
        if enabled:
            defaults = dict(target_rate_raw_s=1800., deadzone=.1, direction=dict(left=1, right=1),
                            max_lead_raw=300., io_hz=20., input_timeout_s=.25, feedback_timeout_s=.5,
                            serial_timeout_s=.2, drop_alarm_policy='stop')
            if set(config) - set(defaults):
                raise ValueError('Unknown ZX teleop settings')
            options = dict(defaults, **config)
            for key in ('target_rate_raw_s', 'max_lead_raw', 'io_hz', 'input_timeout_s', 'feedback_timeout_s'):
                if isinstance(options[key], bool) or not isinstance(options[key], (int, float)) or not math.isfinite(options[key]) or options[key] <= 0:
                    raise ValueError(f'ZX {key} must be positive and finite')
            if (isinstance(options['deadzone'], bool) or not isinstance(options['deadzone'], (int, float))
                    or not 0 <= options['deadzone'] < 1 or options['io_hz'] > 50):
                raise ValueError('ZX deadzone must be within [0,1) and io_hz <= 50')
            if set(devices) != {'left', 'right'} or set(options['direction']) != {'left', 'right'}:
                raise ValueError('ZX devices/direction require both physical left and right hands')
            if (devices['left']['identity']['serial'] == devices['right']['identity']['serial'] or
                    devices['left']['port'] == devices['right']['port']):
                raise ValueError('ZX left/right must have distinct USB serials and ports')
            channels = {side: _ZXGripperChannel(side, dict(devices[side]), options)
                        for side in ('left', 'right')}
            self._zx_channels = channels if self.execute else {}
        self.zx_grippers_enabled = enabled

    def connect(self):
        try:
            super().connect()
            # A command-only session subscribes to measured feedback only for recording.
            if (self.robot is not None and self.feedback_poll_hz is not None
                    and (self.continuous_feedback or getattr(self, 'recording', None) is not None)):
                self.robot.start_feedback_polling(self.feedback_poll_hz)
                self._feedback_polling = self.continuous_feedback
            if not self.continuous_feedback:
                logging.getLogger(__name__).info(
                    '遥操作仅发送增量目标：不读取持续反馈；位置使用本地零点，朝向取接管反馈。录制单独读取实际反馈。')
            config = self.reference_control.config
            logging.getLogger(__name__).info(
                '双臂旋转模式=%s，角速度=%srad/s，加速度=%srad/s²，制动=%srad/s²；'
                '双手松开=%s，查询失败=%s，跟踪误差阈值=%s',
                config.rotation_follow_mode, config.max_angular_speed_rad_s,
                config.max_angular_accel_rad_s2,
                config.max_angular_brake_rad_s2 or config.max_angular_accel_rad_s2,
                (('停止并确认' if self.confirm_stop else '仅发送停止请求')
                 if self.stop_on_both_release else '减速保持'),
                '保留界面/暂停新目标' if self.keep_session_on_query_error else '退出',
                '超差停止' if config.enforce_tracking_error_limits else '仅记录')
            if any(mapper.squeeze_translation_only for mapper in self.teleoperator.mappers.values()):
                logging.getLogger(__name__).info(
                    '侧键=仅XYZ平移/姿态锁定；前扳机=平移+局部旋转/精调；同时按下以前扳机为准。')
            for channel in self._zx_channels.values():
                channel.connect()
            faults = self._gripper_faults()
            if faults:
                logging.getLogger(__name__).warning('夹爪未就绪：%s；VR界面保留，禁止故障目标。', faults)
            elif self._zx_channels:
                logging.getLogger(__name__).info(
                    '两只知行夹爪已连接：左右摇杆X分别控制，向右张开、向左闭合；请先将摇杆回中。')
        except BaseException:
            self.close()
            raise
        return self

    def _cancel_accessories(self):
        for channel in getattr(self, '_zx_channels', {}).values():
            channel.request_stop()

    def close(self):
        self._cancel_accessories()
        try:
            super().close()
        finally:
            errors = []
            for channel in getattr(self, '_zx_channels', {}).values():
                try:
                    channel.close()
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                raise RuntimeError('; '.join(errors))

    def _step(self, raw, action):
        if self._zx_channels:
            self.last_gripper_command = {side: channel.snapshot() for side, channel in self._zx_channels.items()}
            health = {side: channel.check_health() for side, channel in self._zx_channels.items()}
        try:
            faults = self._gripper_faults()
            recording = getattr(self, 'recording', None)
            if recording is not None:
                recording.gripper_faulted(faults)
            result = self._step_arms(raw, action)
        except _VRFeedbackStale as exc:
            if not self.execute:
                raise
            self._enter_input_pause(_VRInputPause(self.arm_names[0], 'feedback_stale',
                details=dict(feedback_error=str(exc))))
            result = action
        except _VRInputPause as exc:
            if not self.execute:
                raise  # Read-only observation cannot promise a confirmed physical stop.
            self._enter_input_pause(exc)
            result = action
        for side, channel in self._zx_channels.items():
            sample = raw[side]
            age = sample.get('xlevr.sample_age_s', -1.)
            elapsed = max(0., time.monotonic() - self._input_read_time)
            budget = channel.options['input_timeout_s']
            valid = (health[side] and self._input_pause is None and isinstance(age, (int, float))
                     and math.isfinite(age) and 0 <= age and age + elapsed <= budget)
            value = action[self.teleoperator.hand_to_arm[side]].get('gripper.pos', 0.)
            channel.submit(float(value) if valid else 0., time.monotonic() + budget - age - elapsed if valid else 0.)
        return result

    def _gripper_faults(self):
        faults = {}
        for side, channel in self._zx_channels.items():
            state = channel.snapshot()
            if state.get('error'):
                faults[side] = state['error']
        return faults

    def prepare_gripper_maintenance(self):
        """Explicit maintenance only: stop arms, confirm, retain VR input state."""
        if not self.execute:
            raise RuntimeError('Gripper maintenance requires a live robot connection')
        self._stop_reference()
        self.robot.wait_stopped()  # Fresh confirmation, including before any follower has started.
        self._dual_time = None
        self._dual_require_release = True
        self.teleoperator.reset_reference()


    @property
    def input_pause_status(self):
        return dict(self._input_pause) if self._input_pause is not None else None

    def input_pause_hud(self):
        status = self.input_pause_status
        if status is None:
            return None
        ready = status['state'] == 'ready'
        feedback_wait = status['state'] == 'feedback_wait'
        stop_failed = status['state'] == 'stop_unconfirmed'
        prompt = ('PRESS A SIDE GRIP TO RESUME' if ready else
                  'RELEASE BOTH GRIPS/TRIGGERS + CENTER STICKS; WAIT FOR TRACKING')
        if stop_failed:
            prompt = 'check controller; restart session'
        return dict(type='hud', phase='input_ready' if ready else 'input_paused',
                    hint=('stop unconfirmed' if stop_failed else
                          'robot feedback delayed' if feedback_wait else
                          'robot feedback restored; release grips' if status['reason'] == 'feedback_stale' and not ready else
                          'robot ready; press side grip' if status['reason'] == 'feedback_stale' else
                          'CONTROL LOOP DELAY - RELEASE GRIPS AND REARM' if status['reason'] == 'control_gap' else
                          'VR INPUT STREAM LOST' if status['reason'] == 'vr_stream_stale' else
                          f"{status['hand'].upper()} HAND: {status['reason']}"), error='',
                    pending=None, input_pause=status,
                    controls=dict(visible=False, ready=ready, prompt=prompt, directions={}))

    def resume_recording_input(self):
        """An explicitly queued recording retry can rearm a stopped, stable input.

        This is called after the current pause step checked real robot feedback
        and neutral controls. Recording may resume; motion still needs a new grip.
        """
        status = self._input_pause
        if status is None:
            return True
        if (status.get('state') != 'ready' or status.get('stopped') is not True
                or self._dual_stop_failed or self._started_follower):
            return False
        frame = self.last_frame or {}
        stamp = frame.get('received_monotonic_s')
        if self.execute and (frame.get('has_error') is not False
                or frame.get('follower_active') is not False
                or not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                or not 0 <= time.monotonic()-stamp <= self.max_feedback_age_s):
            return False
        self._input_pause = None
        self._pause_stable_since = None
        self._pause_samples.clear()
        self._pause_feedback_stamp = None
        self._feedback_resume_after_s = None
        self._dual_time = None
        self._dual_require_release = True
        self.teleoperator.reset_reference()
        self.last_reference_result = dict(state='input_resumed', arms={}, transmitted=False)
        logging.getLogger(__name__).info('录制重试：输入已稳定且停止已确认；重新按侧键才接管机械臂。')
        return True

    def _pause_result(self):
        self.last_reference_result = dict(state='input_paused', arms={}, transmitted=False,
                                         input_pause=self.input_pause_status)
        if getattr(self, 'recording', None) is None:
            payload = self.input_pause_hud()
            signature = (payload['phase'], payload['hint'])
            if signature != self._pause_hud_signature:
                try:
                    self.teleoperator.send_feedback(payload)
                    self._pause_hud_signature = signature
                except Exception:
                    logging.getLogger(__name__).warning('暂停提示发送失败', exc_info=True)

    def _enter_input_pause(self, error):
        diagnostics = self._arms[error.arm]['mapper'].get_last_diagnostics() or {}
        for channel in self._zx_channels.values():
            channel.pause()
        self._input_pause = dict(state='release_required', arm=error.arm,
                                 hand=self._arms[error.arm]['hand'], reason=error.reason,
                                 stopped=False, stable_s=0.)
        self._input_pause.update(error.details)
        self._pause_samples.clear()
        self._pause_stable_since = None
        self._pause_feedback_stamp = None
        self._feedback_resume_after_s = None
        self._pause_hud_signature = None
        recording = getattr(self, 'recording', None)
        if recording is not None:
            recording.input_paused(str(error))
        if self.execute:
            try:
                self._stop_reference()
                # Also confirm an idle/pre-start fault; no session-owned follower may exist yet.
                if self.confirm_stop and (not self.last_stop_status or self.last_stop_status.get('reported_stopped') is not True):
                    self.robot.wait_stopped()
                # Stop confirmation queries fresh state. The memory cache may
                # still contain a pre-stop active frame; wait for its next update.
                self._feedback_resume_after_s = time.monotonic()
            except Exception as exc:
                self._dual_stop_failed = True
                if error.reason == 'feedback_stale':
                    self._input_pause.update(state='stop_unconfirmed', stopped=False,
                                             stop_error=str(exc))
                    self._dual_time = None
                    self._dual_require_release = True
                    self.teleoperator.reset_reference()
                    self._pause_result()
                    logging.getLogger(__name__).error(
                        '反馈过期，停止未确认；保留头显并锁定本会话，请现场确认：%s', exc)
                    return
                raise
        self._input_pause.update({key: diagnostics.get(key) for key in ('vr_input_speed_m_s', 'sample_dt_s')})
        self._input_pause['stopped'] = not self.execute or self.confirm_stop
        if error.reason == 'feedback_stale':
            self._input_pause['state'] = 'feedback_wait'
        self._dual_time = None
        self._dual_require_release = True
        self.teleoperator.reset_reference()
        self._pause_result()
        logging.getLogger(__name__).warning(
            'VR已停发目标并锁定接管：%s；双手松开并回中，输入稳定后重新按侧键接管。诊断=%s',
            error, error.details)

    def _pause_input_stable(self, raw, now):
        """New paired poses, fresh tracking and the existing position-speed guard."""
        valid, updated = True, True
        for arm, context in self._arms.items():
            side = context['hand']
            sample = raw[side]
            age, stamp = sample.get('xlevr.sample_age_s'), sample.get('xlevr.sample_receive_time_s')
            sequence = sample.get('xlevr.sample_sequence')
            limit = min(context['mapper'].stale_timeout_s, context['control'].config.max_input_age_s)
            try:
                position = np.asarray(sample.get('xlevr.target_position'), dtype=float)
                quat = np.asarray(sample.get('xlevr.orientation_quat'), dtype=float)
                good = (sample.get('xlevr.verification_tracking') is True and type(sequence) is int
                        and sequence >= 0 and isinstance(age, (int, float)) and math.isfinite(age)
                        and 0 <= age + max(0., now-self._input_read_time) <= limit
                        and isinstance(stamp, (int, float)) and math.isfinite(stamp) and 0 <= now-stamp <= limit
                        and position.shape == (3,) and np.isfinite(position).all()
                        and quat.shape == (4,) and np.isfinite(quat).all() and np.linalg.norm(quat) > 1e-8)
            except (TypeError, ValueError):
                good = False
            if not good:
                valid = False
                self._pause_samples.pop(side, None)
                continue
            previous = self._pause_samples.get(side)
            if previous is None:
                valid = False  # Establish a baseline before starting the stability timer.
            elif sequence == previous[0]:
                updated = False  # Re-reading one pose never completes the stability gate.
            else:
                dt = stamp-previous[1]
                if sequence < previous[0] or not 0 < dt <= limit or (
                        np.linalg.norm(position-previous[2])/dt > context['mapper'].max_vr_speed_m_s):
                    valid = False
            if previous is None or sequence != previous[0]:
                self._pause_samples[side] = (sequence, stamp, position.copy())
        return valid, updated

    def _step_input_pause(self, raw):
        if self._dual_stop_failed:
            if self._input_pause['reason'] != 'feedback_stale':
                raise RuntimeError('Dual follower stop is unconfirmed; do not restart this session')
            self._input_pause.update(state='stop_unconfirmed', stable_s=0.)
            self._pause_result()
            return
        now = time.monotonic()
        feedback_recovery = False
        feedback_updated = True
        if self.continuous_feedback:
            try:
                self._checked_frame(require_active=False)  # Real faults retain their existing handling.
            except _VRFeedbackStale:
                self._wait_for_feedback()
                return
            stamp = self.last_frame['received_monotonic_s']
            if self._feedback_resume_after_s is not None and stamp <= self._feedback_resume_after_s:
                self._wait_for_feedback()
                return
            feedback_recovery = self._input_pause['reason'] == 'feedback_stale'
            if feedback_recovery and (self.last_frame.get('follower_active') is not False
                    or self.last_frame.get('motion') != 'Stopped'
                    or self.last_frame.get('activation') != 'Enabled'):
                self._wait_for_feedback()
                return
            if self.execute and not self.confirm_stop:
                # No extra stop query: the existing feedback cache may update later.
                if (self.last_frame.get('follower_active') is not False
                        or self.last_frame.get('motion') != 'Stopped'):
                    self._wait_for_feedback()
                    return
                self._input_pause['stopped'] = True
            if self.execute and self.last_frame.get('follower_active') is not False:
                raise RuntimeError('Follower restarted while VR input is paused')
            feedback_updated = self._pause_feedback_stamp is None or stamp > self._pause_feedback_stamp
            if self._pause_feedback_stamp is not None and (
                    stamp < self._pause_feedback_stamp or stamp-self._pause_feedback_stamp > self.max_feedback_age_s):
                self._pause_stable_since = None
                self._input_pause.update(state='stabilizing', stable_s=0.)
            if feedback_updated:
                self._pause_feedback_stamp = stamp
        valid, updated = self._pause_input_stable(raw, now)
        grips, neutral = [], True
        for sample in raw.values():
            grip = bool(sample.get('xlevr.grip_active') or sample.get('xlevr.buttons', {}).get('squeeze'))
            grips.append(grip)
            trigger = sample.get('xlevr.trigger')
            stick = sample.get('xlevr.thumbstick', {})
            values = [trigger, stick.get('x', 0.), stick.get('y', 0.)]
            neutral = neutral and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= .1 for v in values)
        if self._input_pause['state'] == 'ready' and valid and updated and neutral and any(grips):
            self._input_pause = None
            self._pause_stable_since = None
            self._pause_samples.clear()
            self._pause_feedback_stamp = None
            self._feedback_resume_after_s = None
            self._dual_require_release = False
            self.teleoperator.reset_reference()  # Next sample uses current hand + actual robot anchors.
            self.last_reference_result = dict(state='input_resumed', arms={}, transmitted=False)
            if getattr(self, 'recording', None) is None:
                try:
                    self.teleoperator.send_feedback(dict(type='hud', phase='idle', hint='INPUT RESUMED', controls={}))
                except Exception:
                    logging.getLogger(__name__).warning('恢复提示发送失败', exc_info=True)
            logging.getLogger(__name__).info('VR输入恢复：已重新按侧键，下一帧重新建立锚点。')
            return
        if not valid or not neutral or any(grips):
            self._pause_stable_since = None
            self._input_pause.update(state='release_required', stable_s=0.)
        elif updated and (not feedback_recovery or feedback_updated):
            if self._pause_stable_since is None:
                self._pause_stable_since = now
            elapsed = now-self._pause_stable_since
            self._input_pause.update(state='ready' if elapsed >= self.input_recovery_stable_s else 'stabilizing',
                                     stable_s=round(elapsed, 3))
        self._pause_result()

    def _wait_for_feedback(self):
        """No GET/stop retries or target replay while a stopped session is locked."""
        self._pause_stable_since = None
        self._pause_feedback_stamp = None
        self._input_pause.update(state='feedback_wait', stable_s=0.)
        self._pause_result()

    def _checked_frame(self, *, require_active, force_refresh=False):
        from .control import pose
        if (not self.continuous_feedback and not force_refresh and self._dual_time is not None):
            if self._sent_reference_poses is None:
                raise RuntimeError('No successfully sent reference is available')
            # These are local references, never labelled/published as measured feedback.
            return {name: value.copy() for name, value in self._sent_reference_poses.items()}
        read = (self.robot.get_cached_frame if self._feedback_polling and not force_refresh
                else self.robot.get_frame)
        frame = self._timed_call('robot_feedback_s', read)
        self.last_frame = frame
        now = time.monotonic()
        if not isinstance(frame, dict) or frame.get('has_error') is not False:
            raise RuntimeError('Dual A10 feedback is missing or reports a fault')
        stamp = frame.get('received_monotonic_s')
        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                or not math.isfinite(stamp) or not 0 <= stamp <= now):
            raise RuntimeError('Dual A10 feedback timestamp is invalid')
        if now - stamp > self.max_feedback_age_s:
            raise _VRFeedbackStale('Dual A10 feedback is stale')
        if require_active and (frame.get('follower_active') is not True
                               or frame.get('follower_mode') != 'Cart'):
            raise RuntimeError('Dual Cartesian follower is not confirmed active')
        arms = frame.get('arms')
        if not isinstance(arms, dict) or set(arms) != set(self.arm_names):
            raise RuntimeError('Dual A10 feedback must contain the complete configured arm group')
        poses = {}
        for name in self.arm_names:
            arm_stamp = arms[name].get('received_monotonic_s')
            if (isinstance(arm_stamp, bool) or not isinstance(arm_stamp, (int, float))
                    or not math.isfinite(arm_stamp) or not 0 <= arm_stamp <= now):
                raise RuntimeError(f'Dual A10 feedback timestamp is invalid for {name}')
            if now - arm_stamp > self.max_feedback_age_s:
                raise _VRFeedbackStale(f'Dual A10 feedback is stale for {name}')
            poses[name] = pose(arms[name].get('tcp_pq_m'))
        return poses

    def _stop_reference(self):
        try:
            super()._stop_reference()
        except BaseException:
            self._dual_stop_failed = True
            raise
        finally:
            self._dual_time = None
            self._sent_reference_poses = None
            self._last_target_send_s = None
            self._dual_require_release = True
            self.teleoperator.reset_reference()

    def _send_dual_targets(self, targets):
        """Send both planned poses together; SDK state survives per-hand re-clutch."""
        if self.cartesian_adapter == 'sdk_tool_cumulative':
            self.last_sdk_command = self._timed_call('target_send_s', self.robot.send_sdk_tool_pose,
                targets, max_position_step_m=None, max_rotation_step_rad=None)
            self.last_relative_target = None
        else:
            from .control import follower_relative
            relative = {name: follower_relative(self._arms[name]['control'].startup_anchor, target)
                        for name, target in targets.items()}
            self._timed_call('target_send_s', self.robot.send_eepose, relative)
            self.last_relative_target = relative
        if not self.continuous_feedback:
            self._sent_reference_poses = {name: value.copy() for name, value in targets.items()}
            self._last_target_send_s = time.monotonic()
            self.last_reference_result.update(reference_source='last_sent_target', reference_origin='local_zero')
        self.last_reference_result['transmitted'] = True

    def _input_state(self, raw, action):
        requested, fresh = {}, {}
        for arm, context in self._arms.items():
            hand_raw = raw[context['hand']]
            age = hand_raw.get('xlevr.sample_age_s')
            requested[arm] = bool(hand_raw.get('xlevr.enabled'))
            fresh[arm] = (isinstance(age, (int, float)) and math.isfinite(age)
                          and 0 <= age + max(0., time.monotonic() - self._input_read_time)
                          <= min(context['mapper'].stale_timeout_s,
                                 context['control'].config.max_input_age_s))
        # A lost browser stream must stop HOLD packets too. Previously only a
        # pressed/tracking arm checked age, so old nonzero targets were repeated
        # indefinitely after both hands had already released.
        if self._dual_time is not None and not any(fresh.values()):
            raise _VRInputPause(self.arm_names[0], 'vr_stream_stale')
        for arm, context in self._arms.items():
            hand_raw = raw[context['hand']]
            if hand_raw.get('xlevr.reset_arm') or action[arm].get('_xlevr.reset_arm'):
                raise RuntimeError('Dual VR live reset is unsupported; release the side keys')
            if requested[arm] or (self._dual_time is not None and context['state'] == 'tracking'):
                if not fresh[arm]:
                    raise _VRInputPause(arm, 'input_stale')
                if hand_raw.get('xlevr.verification_tracking') is False:
                    raise _VRInputPause(arm, 'tracking_lost')
                command = action[arm].get('_xlevr.anchor_command', {})
                if requested[arm] and command.get('active') is not True:
                    reason = (context['mapper'].get_last_diagnostics() or {}).get('guard_reason')
                    if reason in ('spike_pending', 'spike_reanchor', 'stale_age', 'sample_gap',
                                  'invalid_sample', 'sequence_rollback'):
                        raise _VRInputPause(arm, reason)
                    raise RuntimeError(f'Dual VR {arm}: mapper_input_rejected ({reason})')
        return requested, fresh

    def _wait_for_takeover(self, reason):
        """Temporarily not ready is an idle UI state, not an exception or retry."""
        self._dual_require_release = True
        self.teleoperator.reset_reference()
        self.last_reference_result = dict(state='await_controller_ready', reason=reason,
                                         arms={}, transmitted=False)
        if reason != self._takeover_wait_reason:
            self._takeover_wait_reason = reason
            logging.getLogger(__name__).warning('暂不可接管：%s；松开侧键，状态恢复后重新按下。', reason)
            try:
                self.teleoperator.send_feedback(dict(type='hud', phase='input_paused',
                    hint='CONTROLLER NOT READY - RELEASE GRIPS AND RETRY', error=reason))
            except Exception:
                logging.getLogger(__name__).warning('接管状态提示发送失败', exc_info=True)

    def _step_arms(self, raw, action):
        if not self.execute and not self.observe_only:
            return action  # VR-only maps both hands, never fabricates feedback.
        if self._input_pause is not None:
            self._step_input_pause(raw)
            return action
        if self._dual_stop_failed:
            raise RuntimeError('Dual follower stop is unconfirmed; do not restart this session')
        if set(raw) != {'left', 'right'} or set(action) != set(self.arm_names):
            raise ValueError('Dual VR requires complete hand input and arm actions')
        if self._dual_require_release:
            # Latched after a shared stop/fault: no automatic restart with held keys.
            released = all(not raw[c['hand']].get('xlevr.enabled') and
                           isinstance(raw[c['hand']].get('xlevr.sample_age_s'), (float, int)) and
                           0 <= raw[c['hand']]['xlevr.sample_age_s'] <= c['mapper'].stale_timeout_s
                           for c in self._arms.values())
            if released:
                self._dual_require_release = False
                self.teleoperator.reset_reference()
            self.last_reference_result = dict(state='release_required', arms={})
            return action
        requested, fresh = self._input_state(raw, action)
        if (not self.continuous_feedback and self._dual_time is None
                and not any(requested[n] and action[n].get('ee.enabled') for n in self.arm_names)):
            self.last_reference_result = dict(state='idle', arms={}, transmitted=False)
            return action
        poses = self._checked_frame(require_active=self.execute and self._started_follower)
        if self._dual_time is None:
            starters = [name for name in self.arm_names if requested[name]
                        and action[name].get('ee.enabled')]
            if not starters:
                self.last_reference_result = dict(state='idle', arms={})
                return action
            if self.execute and self.resilient_takeover:
                frame = self.last_frame
                if (frame.get('follower_active') is not False or frame.get('motion') != 'Stopped'
                        or frame.get('activation') != 'Enabled'):
                    self._wait_for_takeover(
                        f"activate={frame.get('activation')}, motion={frame.get('motion')}, follower={frame.get('follower_active')}")
                    return action
            for name in starters:
                self._arms[name]['control'].validate_command(
                    action[name]['_xlevr.anchor_command'], poses[name])
            if self.execute:
                self._reference_stop_attempted = False
                self.last_stop_status = None
                if not self._started_follower:
                    from ..a10_operator import A10NotReadyError
                    try:
                        if self.resilient_takeover:
                            self.robot.ensure_tool_coordinate()
                        else:
                            self.robot.set_tool_coordinate()
                        self.robot.start_follower('cartesian')
                    except A10NotReadyError as exc:
                        if not self.resilient_takeover or self.robot.follower_stop_pending:
                            raise
                        self._wait_for_takeover(str(exc))
                        return action
                self._takeover_wait_reason = None
                self._started_follower = True
                poses = self._checked_frame(require_active=True, force_refresh=True)
            if not self.continuous_feedback:
                # The absolute position cancels in the cumulative SDK adapter.
                # Keep the measured orientation for base-to-tool conversion only.
                poses = {name: np.r_[np.zeros(3), value[3:]] for name, value in poses.items()}
            targets = {}
            for name, context in self._arms.items():
                context.update(state='holding', key=None, target=poses[name].copy())
                context['control'].start(poses[name])
                targets[name] = poses[name].copy()
            self.teleoperator.reset_reference()  # Discard input predating blocking WS setup.
            self._dual_time = time.monotonic()
            self.last_reference_result = dict(state='await_fresh_sample', arms={}, transmitted=False)
            if self.execute:
                if self.cartesian_adapter == 'sdk_tool_cumulative':
                    self.robot.begin_sdk_tool_trajectory(poses)
                self._send_dual_targets(targets)
            return action
        # End the native follower when both grips release. A repeating HOLD is
        # not a verified controller stop; per-arm release below keeps the other arm active.
        if self.stop_on_both_release and not any(requested.values()) and any(fresh.values()):
            self.last_reference_result = dict(state='stopping', arms={}, transmitted=False,
                                             reason='both_hands_released')
            if self.execute:
                self._stop_reference()
                # Both grips were freshly released before this normal stop.
                # New fresh presses may re-clutch after confirmation; faults
                # retain their separate release/recovery path.
                self._dual_require_release = False
            else:
                self._dual_time = None
                self._dual_require_release = True
                self.teleoperator.reset_reference()
            self.last_reference_result['state'] = 'stopped' if self.confirm_stop else 'stop_requested'
            return action
        now = time.monotonic()
        dt = now - self._dual_time
        if dt > self.reference_control.config.max_dt_s:
            # A delayed GET/relay transaction must not advance either arm's
            # integrator with a large dt, nor terminate the recording UI.
            # Discard this input, confirm native stop and require fresh re-clutch.
            raise _VRInputPause(self.arm_names[0], 'control_gap', details=dict(
                control_gap_s=dt, max_control_gap_s=self.reference_control.config.max_dt_s,
                io_timing_s=dict(self.last_io_timing)))
        feedback_age = now - self.last_frame['received_monotonic_s'] if self.continuous_feedback else 0.
        targets, results = {}, {}
        for name, context in self._arms.items():
            control = context['control']
            command = action[name]['_xlevr.anchor_command']
            active = requested[name] and command.get('active') is True and action[name].get('ee.enabled')
            key = (command.get('session_id'), command.get('anchor_id'))
            if context['state'] == 'tracking' and requested[name]:
                if key != context['key']:
                    raise RuntimeError(f'Dual VR {name}: anchor_changed_while_engaged')
            if context['state'] == 'tracking' and not requested[name]:
                context['state'] = 'releasing'
            # A fresh clutch press can resume even during frontend braking.
            # continuous_reanchor preserves the command pose and velocity;
            # input/transport fault recovery is handled separately above.
            if active:
                age = raw[context['hand']]['xlevr.sample_age_s'] + max(0., now - self._input_read_time)
                result = control.step(command, poses[name], dt, sample_age_s=age,
                                      feedback_age_s=feedback_age, continuous_reanchor=True)
                context.update(state='tracking', key=key)
            else:
                result = control.step_released(poses[name], dt, feedback_age_s=feedback_age,
                    feedback_time_s=self.last_frame['received_monotonic_s'] if self._feedback_polling else None)
                if context['state'] == 'releasing' and result['state'] == 'holding':
                    context['mapper'].reset()  # Re-clutch only this hand; preserve the other.
                if context['state'] == 'releasing':
                    context['state'] = result['state']
            results[name] = result
            result.update(hand=context['hand'], reengage_requires_release=False)
            self.last_reference_result = dict(state='running', arms=results, transmitted=False)
            if result['stop_requested']:
                # The controller also ages duplicate input internally. It may
                # cross its input budget AFTER the pre-GET freshness check.
                # Apply the same release/rearm path as an early stale check.
                if result['fault_reason'] in ('input_stale', 'control_gap'):
                    raise _VRInputPause(name, result['fault_reason'])
                if result['fault_reason'] == 'feedback_stale':
                    raise _VRFeedbackStale(f'Dual A10 feedback is stale for {name}')
                raise RuntimeError(f"Dual VR {name}: {result['fault_reason']}")
            target = np.asarray(result['command_pose'])
            self._check_tracking(poses[name], target)
            targets[name] = target
            if self.cartesian_adapter == 'sdk_tool_cumulative':
                result['relative_target'] = None  # Wire rows are cumulative 321, not relative poses.
            context['target'] = target.copy()
        # Validate BOTH arms first: never transmit a partially valid group.
        # After the explicitly configured soft release, a stationary endpoint
        # need not be resent. The native session stays owned until fault/exit.
        unchanged_hold = (not self.continuous_feedback and not any(requested.values())
            and self._sent_reference_poses is not None
            and all(np.linalg.norm(c['control'].linear_velocity) < 1e-10
                    and np.linalg.norm(c['control'].angular_velocity) < 1e-10
                    and np.allclose(targets[name], self._sent_reference_poses[name], rtol=0., atol=1e-12)
                    for name, c in self._arms.items()))
        if self.execute and not unchanged_hold:
            self._send_dual_targets(targets)
        elif unchanged_hold:
            self.last_reference_result.update(reference_source='last_sent_target', reference_origin='local_zero',
                                             hold_transmission_suppressed=True)
        if not any(requested.values()):
            self.last_reference_result['state'] = (
                'holding' if all(r['state'] == 'holding' for r in results.values()) else 'releasing')
        self._dual_time = now
        return action


def _read_only_robot(operator_cfg):
    """Whitelist status transport only; never inherit accessories or polling."""
    from ..a10_operator import A10Operator
    allowed = ('host', 'timeout_s', 'udp_port', 'joint_udp_port',
               'feedback_joint_unit', 'feedback_position_unit', 'arms', 'cartesian_target_order')
    options = {key: operator_cfg[key] for key in allowed if key in operator_cfg}
    options.update(ws_port=operator_cfg.get('status_ws_port', 5888), allow_motion=False)
    return A10Operator(**options)


def _calibration_state(robot, max_age_s):
    """GET only, with positive stop confirmation and transport freshness."""
    from .control import pose
    state = robot.get_state()
    age = time.monotonic() - state.received_monotonic_s
    if not math.isfinite(age) or not 0. <= age <= max_age_s or state.has_error is not False:
        raise ValueError('机械臂反馈过期或有故障，不能标定')
    motion = state.raw['ret_context'].get('robot_msg', {}).get('motion')
    if state.follower_active is not False or motion != 'Stopped':
        raise ValueError('标定要求跟随器未运行且机械臂状态为 Stopped；请先在现场停止')
    if state.tcp_pq_m is None:
        raise ValueError('缺少末端实际姿态，请确认 feedback_position_unit 为 mm 或 m')
    return pose(state.tcp_pq_m)


def collect_pose_calibration(vr_input, robot, hand_to_tool, *, duration_s=1., fps=30.,
                             timeout_s=10., max_age_s=.25):
    """Collect unique raw VR poses paired with read-only stopped feedback."""
    from .control import pose, calibrate_reference_pose
    for name, value in [('duration_s', duration_s), ('fps', fps),
                        ('timeout_s', timeout_s), ('max_age_s', max_age_s)]:
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if timeout_s <= duration_s:
        raise ValueError('timeout_s must exceed duration_s')
    hands, robots, sequences, stamps = [], [], [], []
    began = time.monotonic()
    while time.monotonic() - began < timeout_s:
        raw = vr_input.get_action()
        now = time.monotonic()
        seq = raw.get('xlevr.sample_sequence')
        stamp = raw.get('xlevr.sample_receive_time_s')
        age = raw.get('xlevr.sample_age_s')
        if (not raw.get('xlevr.enabled') or type(seq) is not int or seq < 0
                or not isinstance(stamp, (float, int)) or not math.isfinite(stamp)
                or not isinstance(age, (float, int)) or not math.isfinite(age)
                or not 0. <= age <= max_age_s or not 0. <= now - stamp <= max_age_s):
            raise ValueError('没有新鲜有效的 VR 姿态；请连接浏览器、按住右手握键并保持静止后重试')
        if sequences and seq < sequences[-1]:
            raise ValueError('VR 序号回退或会话重置，请重新标定')
        if not sequences or seq > sequences[-1]:
            if stamps and stamp <= stamps[-1]:
                raise ValueError('VR 新帧时间没有递增，请重新标定')
            hand = pose(np.r_[raw.get('xlevr.target_position'), raw.get('xlevr.orientation_quat')])
            actual = _calibration_state(robot, max_age_s)
            # GET may block. A pre-GET fresh hand must not become an old pair.
            if time.monotonic() - stamp > max_age_s or time.monotonic() - now + age > max_age_s:
                raise ValueError('读取机械臂反馈耗时过长，VR 姿态已过期，请重试')
            hands.append(hand)
            robots.append(actual)
            sequences.append(seq)
            stamps.append(stamp)
            if len(hands) >= 10 and stamps[-1] - stamps[0] >= duration_s:
                report = calibrate_reference_pose(hands, robots, hand_to_tool)
                report.update(sample_sequences=[sequences[0], sequences[-1]],
                              sample_duration_s=stamps[-1] - stamps[0],
                              controller_sample_age_verified=False)
                return report
        time.sleep(1. / fps)
    raise TimeoutError('未采到足够多的新鲜 VR 帧；原标定未改动，请检查输入后重试')


def calibrate_pose(cfg, *, output_path, countdown=5, duration_s=1., prompt=None, preview=False):
    """Interactive stationary calibration. Creates no motion-capable operator."""
    from datetime import datetime, timezone
    import os
    import shutil
    import tempfile
    from fluxvla.engines.utils.builder import build_operator_from_cfg
    from .control import CalibratedAnchorState
    output_path = Path(output_path).expanduser().resolve()
    if output_path.suffix != '.json':
        raise ValueError('标定输出必须为 .json 文件，不能覆盖 Python 配置或其他文件')
    if type(countdown) is not int or not 0 <= countdown <= 30:
        raise ValueError('countdown must be an integer within 0..30')
    if not math.isfinite(duration_s) or not 1. <= duration_s <= 5.:
        raise ValueError('duration_s must be within 1..5 seconds')
    operator_cfg = cfg['operator']
    if operator_cfg.get('feedback_position_unit') not in ('mm', 'm'):
        raise ValueError('标定需要明确 teleop.operator.feedback_position_unit=mm 或 m')
    hand_to_tool = (cfg.get('pose_calibration') or {}).get('hand_to_tool_quat_xyzw')
    CalibratedAnchorState(dict(vr_to_robot_quat_xyzw=[0., 0., 0., 1.],
                              hand_to_tool_quat_xyzw=hand_to_tool))
    robot = _read_only_robot(operator_cfg)
    if preview:
        build_operator_from_cfg(cfg['input'])
        print(f'只读姿态标定预览：监视端口 {operator_cfg.get("status_ws_port", 5888)}，'
              f'输出 {output_path}；未启动服务或连接设备。')
        return None
    vr = None
    try:
        robot.connect()
        _calibration_state(robot, .25)
        vr = build_operator_from_cfg(cfg['input'])
        vr.connect()
        print('只读姿态标定：机械臂保持静止；手柄的指向和翻转角度都应与期望夹爪朝向对应。', flush=True)
        print(f'VR 浏览器连接后，准备好按住右手握键；倒计时结束后保持静止约 {duration_s:g} 秒。', flush=True)
        (input if prompt is None else prompt)('按回车开始倒计时（Ctrl+C 取消）：')
        for remaining in range(countdown, 0, -1):
            print(f'{remaining}…', flush=True)
            time.sleep(1.)
        print('正在采样，请保持静止并按住握键…', flush=True)
        report = collect_pose_calibration(vr.input, robot, hand_to_tool, duration_s=duration_s)
        report.update(schema_version=1, kind='vr_pose_calibration', read_only=True,
                      created_utc=datetime.now(timezone.utc).isoformat(),
                      controller_host=operator_cfg['host'],
                      feedback_position_unit=operator_cfg['feedback_position_unit'],
                      vr_arm=cfg['input'].get('arm', 'right'),
                      reference_alignment_operator_confirmed=True)
    finally:
        try:
            if vr is not None:
                vr.close()
        finally:
            robot.close()
    # Save only after successful capture AND cleanup; failed runs keep old data.
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=output_path.parent,
                                         suffix='.tmp', delete=False) as stream:
            temporary_path = Path(stream.name)
            json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
        if output_path.exists():
            shutil.copyfile(output_path, str(output_path) + '.bak')
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
    print(f'标定已保存：{output_path}（{report["quality"]["sample_count"]} 对姿态）', flush=True)
    print('这只完成参数拟合；请先用 --verify-calibration 核对现场方向，不能仅凭采样误差认定标定正确。', flush=True)
    return report


def _load_pose_calibration(settings):
    """Auto-load site data only for absolute mapping with no explicit base rotation."""
    path = settings.pop('pose_calibration_path', None)
    calibration = dict(settings.get('pose_calibration') or {})
    if (settings.get('pose_mapping') != 'calibrated_absolute'
            or calibration.get('vr_to_robot_quat_xyzw') is not None or not path):
        return
    path = Path(path).expanduser()
    if not path.exists():
        return  # Existing missing-calibration validation supplies the error.
    report = json.loads(path.read_text(encoding='utf-8'))
    if (not isinstance(report, dict) or report.get('schema_version') != 1
            or report.get('kind') != 'vr_pose_calibration' or report.get('read_only') is not True):
        raise ValueError('标定文件格式不正确，请重新运行 --calibrate-pose')
    if (report.get('controller_host') != settings['operator']['host']
            or report.get('vr_arm') != settings['input'].get('arm', 'right')):
        raise ValueError('标定文件的控制器地址或手柄侧别与当前配置不同，请重新标定')
    unit = settings['operator'].get('feedback_position_unit')
    if unit is not None and unit != report.get('feedback_position_unit'):
        raise ValueError('标定文件与当前配置的反馈位置单位不同，请核对')
    from .control import CalibratedAnchorState
    loaded = report.get('pose_calibration')
    anchor = CalibratedAnchorState(loaded)
    expected = calibration.get('hand_to_tool_quat_xyzw')
    if expected is not None and (anchor.hand_from_tool.inv() * Rotation.from_quat(expected)).magnitude() > 1e-8:
        raise ValueError('当前手柄工具轴约定与标定文件不同，请重新标定')
    settings['pose_calibration'] = dict(loaded)
    print(f'已加载姿态标定：{path}', flush=True)


def build_runtime(cfg):
    """Construct from a native teleop config; no services open until run()."""
    from fluxvla.engines.utils.builder import build_operator_from_cfg
    settings = dict(cfg)
    _load_pose_calibration(settings)
    vr_only = settings.pop('vr_only', True)
    if type(vr_only) is not bool:
        raise ValueError('vr_only must be bool')
    vr = build_operator_from_cfg(settings.pop('input'))
    robot_cfg = dict(settings.pop('operator'))
    settings.pop('fps', None)
    settings.pop('max_steps', None)
    if settings.get('control_mode', 'delta') != 'reference':
        # Shared config also supports delta; its reference settings stay dormant.
        settings.pop('reference_options', None)
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
    if getattr(vr, 'is_dual', False):
        from ..a10_operator import A10Operator
        # Validate device topology even for VR-only/preview; do not connect it.
        topology = robot or A10Operator(**{k: v for k, v in robot_cfg.items() if k != 'type'})
        if not topology.is_multi_arm or set(topology.arm_names) != set(vr.hand_to_arm.values()):
            raise ValueError('Dual VR bindings must match the dual operator configuration')
    elif robot is not None and getattr(robot, 'is_multi_arm', False):
        raise ValueError('Dual A10 requires a both-hands VR configuration')
    runtime_class = DualVRTeleopRuntime if getattr(vr, 'is_dual', False) else VRTeleopRuntime
    return runtime_class(vr, robot=robot, execute=not vr_only and not observe_only,
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
    if type(samples) is not int or not 1 <= samples <= 100:
        raise ValueError('samples must be an integer within 1..100')
    if not math.isfinite(interval_s) or interval_s <= 0:
        raise ValueError('interval_s must be positive and finite')
    # Only transport/decoding fields enter this instance. Camera/gripper/motion
    # options in another workflow must never leak into a read-only inspection.
    robot = _read_only_robot(operator_cfg)
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
            if robot.is_multi_arm:
                rows[-1]['arms'] = {name: dict(ee_id=arm.ee_id,
                    received_monotonic_s=arm.received_monotonic_s,
                    actual_joints_rad=None if arm.actual_joints_rad is None else arm.actual_joints_rad.tolist(),
                    tcp_pq_m=None if arm.tcp_pq_m is None else arm.tcp_pq_m.tolist())
                    for name, arm in state.arms.items()}
            if index + 1 < samples:
                time.sleep(interval_s)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        robot.close()
    return dict(read_only=True, query_success=error is None and len(rows) == samples,
                error=error, get_attempts=attempts, completed_queries=len(rows),
                commands=['get'] if attempts else [], udp_targets_sent=0,
                host=operator_cfg['host'], monitor_port=operator_cfg.get('status_ws_port', 5888),
                feedback_position_unit=operator_cfg.get('feedback_position_unit'),
                controller_sample_age_verified=False,
                unknown_capabilities=['cartesian_smoothing', 'target_queue',
                                      'packet_ordering', 'stream_timeout_behavior'],
                max_query_elapsed_s=max((row['query_elapsed_s'] for row in rows), default=None), samples=rows)
