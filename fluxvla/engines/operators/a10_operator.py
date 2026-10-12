"""A10 single-arm operator for the KAANH WebSocket / UDP follower protocol.

Wire layout follows the user-supplied robot.7z reference. Controller-side
planning stays on the controller. This module does not open connections at
import/construction, automatically enable motors, interpret VR increments, or invent a
seventh joint. A Cartesian target is an origin-relative pose, not joint state.
"""

from __future__ import annotations

import json
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Mapping

import numpy as np

from ..utils.root import OPERATORS
from .base_operator import BaseOperator
from .camera_operator import CameraOperator
from .gripper_operator import GripperOperator

__all__ = ['A10Operator', 'A10State', 'A10ProtocolError', 'A10ControllerError']

_HEADER = struct.Struct('<IIQqqq')
_MAX_PAYLOAD = 1024 * 1024
_FOLLOWER_MODES = {'cartesian': ('follower_cart', 'Cart'),
                   'joint': ('follower_aj', 'Axis')}


class A10ProtocolError(ValueError):
    """A malformed or incompatible controller message."""


class A10ControllerError(RuntimeError):
    """The controller rejected a command or reported a fault."""


def _pack(payload: bytes, *, command: bool) -> bytes:
    if len(payload) > _MAX_PAYLOAD:
        raise A10ProtocolError('Payload exceeds the protocol size limit')
    fields = (1, 0x1000, 0xA1B2C3D4, 0, 0) if command else (0, 0, 0, 0, 0)
    return _HEADER.pack(len(payload), *fields) + payload


def _unpack(packet: bytes) -> bytes:
    if not isinstance(packet, bytes) or len(packet) < _HEADER.size:
        raise A10ProtocolError('Expected a binary frame with a 40-byte header')
    size, *_ = _HEADER.unpack_from(packet)
    if size > _MAX_PAYLOAD or size != len(packet) - _HEADER.size:
        raise A10ProtocolError('Header payload length does not match frame')
    # Response IDs/reserved fields have no documented echo contract.
    return packet[_HEADER.size:]


def _reject_constant(value):
    raise A10ProtocolError(f'Non-finite JSON number: {value}')


def _json_object(payload):
    try:
        value = json.loads(payload, parse_constant=_reject_constant)
    except (ValueError, UnicodeDecodeError) as exc:
        raise A10ProtocolError('Invalid controller JSON') from exc
    if not isinstance(value, dict):
        raise A10ProtocolError('Controller reply must be a JSON object')
    return value


def _decode_reply(packet):
    value = _json_object(_unpack(packet))
    code = value.get('ret_code')
    if type(code) is not int:
        raise A10ProtocolError('Reply must contain integer ret_code')
    if code != 0:
        raise A10ControllerError(f'Controller ret_code={code}: {value.get("ret_msg", "")}')
    context = value.get('ret_context')
    if isinstance(context, str) and context.strip():
        value['ret_context'] = _json_object(context)
    return value


def _vector(value, size, field):
    if (not isinstance(value, (list, tuple, np.ndarray))
            or np.asarray(value).shape != (size,)):
        raise A10ProtocolError(f'{field} must have exactly {size} values')
    if any(isinstance(x, (bool, np.bool_)) or not isinstance(
            x, (int, float, np.integer, np.floating)) for x in value):
        raise A10ProtocolError(f'{field} must contain numbers, not strings/bools')
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise A10ProtocolError(f'{field} must contain finite values')
    return result.copy()


def _single_group(value, size, field):
    """Accept flat or one nested group; never silently truncate another robot."""
    if value is None:
        return None
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], list):
        value = value[0]
    return _vector(value, size, field)


def _pose(value):
    result = _vector(value, 7, 'relative_pose_m_xyzw')
    if not math.isclose(float(np.linalg.norm(result[3:])), 1.0, abs_tol=1e-3):
        raise A10ProtocolError('Quaternion xyzw must have unit length')
    return result


def _optional_bool(value, field):
    if value is not None and type(value) is not bool:
        raise A10ProtocolError(f'{field} must be a boolean')
    return value


@dataclass(frozen=True)
class A10State:
    """Parsed feedback. None means unavailable, never a synthetic zero.

    received_monotonic_s is client receive time, not controller sample time.
    tcp_pq_raw retains the controller's unconfirmed pose semantics/units.
    """

    actual_joints_rad: np.ndarray | None
    target_joints_rad: np.ndarray | None
    tcp_pq_raw: np.ndarray | None
    tcp_pq_m: np.ndarray | None
    gripper_mm: float | None
    follower_active: bool | None
    follower_mode: str | None
    has_error: bool
    received_monotonic_s: float
    raw: dict

    @property
    def qpos(self):
        if self.actual_joints_rad is None or self.gripper_mm is None:
            return None
        return np.concatenate((self.actual_joints_rad, [self.gripper_mm]))


def _parse_state(reply, *, joint_unit, position_unit, gripper_feedback_path):
    context = reply.get('ret_context')
    if not isinstance(context, dict) or not isinstance(context.get('motion_msg'), dict):
        raise A10ProtocolError('Status reply is missing motion_msg')
    motion = context['motion_msg']
    groups = {}
    for key in ('follower_msg', 'robot_msg', 'driver_msg', 'log_msg'):
        groups[key] = context.get(key, {})
        if not isinstance(groups[key], dict):
            raise A10ProtocolError(f'{key} must be an object')
    follower, robot, driver, log = (groups[k] for k in groups)
    actual = _single_group(motion.get('actual_pos'), 6, 'actual_pos')
    target = _single_group(motion.get('motor_pos'), 6, 'motor_pos')
    if joint_unit == 'deg':
        actual = None if actual is None else np.deg2rad(actual)
        target = None if target is None else np.deg2rad(target)
    pq = motion.get('pq')
    # The template allows [model TCP, model axis pose] even for a single arm.
    # Joint groups above still have to describe exactly one six-axis arm.
    if isinstance(pq, list) and len(pq) == 2 and all(isinstance(x, list) for x in pq):
        _vector(pq[1], 7, 'auxiliary axis pq')
        pq = pq[0]
    pose = _single_group(pq, 7, 'pq')
    converted = None
    if pose is not None:
        _pose(pose)
        if position_unit is not None:
            converted = pose.copy()
            if position_unit == 'mm':
                converted[:3] /= 1000.0
    gripper = None
    if gripper_feedback_path is not None:
        value = context
        for key in gripper_feedback_path:
            value = value.get(key) if isinstance(value, dict) else None
        if value is not None:
            gripper = float(_vector([value], 1, 'gripper feedback in mm')[0])
            if not 0 <= gripper <= 100:
                raise A10ProtocolError('A10 gripper feedback must be within 0..100 mm')
    codes = driver.get('driver_err_code', [])
    if not isinstance(codes, list) or any(type(x) is not int for x in codes):
        raise A10ProtocolError('driver_err_code must be an integer list')
    has_error = bool(_optional_bool(log.get('has_error'), 'has_error')) or any(codes)
    for group in (robot, log):
        for key in ('cs_error_code', 'error_code', 'err_code'):
            if key in group:
                if type(group[key]) is not int:
                    raise A10ProtocolError(f'{key} must be an integer')
                has_error |= group[key] != 0
    mode = follower.get('follower_mode')
    if mode is not None and not isinstance(mode, str):
        raise A10ProtocolError('follower_mode must be a string')
    return A10State(actual, target, pose, converted, gripper,
                    _optional_bool(follower.get('running_state'), 'running_state'),
                    mode, bool(has_error), time.monotonic(), reply)


class _FollowerConnection:
    """Serialized WS transactions plus a separate UDP socket; no robot logic."""

    def __init__(self, host, ws_port, udp_port, timeout_s, joint_udp_port=9999):
        self.host, self.ws_port, self.udp_port = host, ws_port, udp_port
        self.timeout_s = timeout_s
        self.joint_udp_port = joint_udp_port
        self.ws = self.udp = self.joint_udp = None

    def connect(self):
        from websockets.sync.client import connect
        try:
            self.ws = connect(f'ws://{self.host}:{self.ws_port}', proxy=None,
                              # KAANH command replies are our health check. Do
                              # not require an independently unverified Pong
                              # implementation during an idle terminal session.
                              ping_interval=None,
                              open_timeout=self.timeout_s, close_timeout=self.timeout_s,
                              max_size=_MAX_PAYLOAD + _HEADER.size)
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.settimeout(self.timeout_s)
            self.udp.connect((self.host, self.udp_port))
            self.joint_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.joint_udp.settimeout(self.timeout_s)
            self.joint_udp.connect((self.host, self.joint_udp_port))
        except Exception:
            self.close()
            raise

    def send_command(self, command):
        if self.ws is None:
            raise ConnectionError('A10 operator is not connected')
        try:
            self.ws.send(_pack(command.encode('utf-8'), command=True))
        except Exception:
            self.close()
            raise

    def request(self, command, *, status=False, timeout_s=None):
        deadline = time.monotonic() + (timeout_s or self.timeout_s)
        try:
            self.send_command(command)
            for _ in range(32):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('A10 reply deadline exceeded')
                reply = _decode_reply(self.ws.recv(timeout=remaining))
                context = reply.get('ret_context')
                is_status = isinstance(context, dict) and 'motion_msg' in context
                if status and is_status:
                    return reply
                if not status and not is_status:
                    return reply
                if status and (context not in ('', None, {}) or reply.get('ret_msg', '')):
                    raise A10ProtocolError('Unexpected response while waiting for status')
                # Bounded skipping of delayed empty ACKs, or old status replies.
            raise A10ProtocolError('Too many unrelated replies')
        except Exception:
            # An uncertain transaction cannot be reused for a later command.
            self.close()
            raise

    def close(self):
        ws, udp, joint_udp = self.ws, self.udp, self.joint_udp
        self.ws = self.udp = self.joint_udp = None
        try:
            if ws is not None:
                ws.close()
        finally:
            try:
                if udp is not None:
                    udp.close()
            finally:
                if joint_udp is not None:
                    joint_udp.close()


@OPERATORS.register_module()
class A10Operator(BaseOperator):
    """Six-axis A10 + gripper, using the reference KAANH follower protocol.

    ``send_eepose({'right': pose})`` expects origin-relative xyz metres + xyzw.
    ``send_joints({'right': joints})`` expects six absolute joint angles in
    radians. They are converted to degrees in UDP ``jp --motor_pos=[...]``;
    the gripper remains a separate command. Only the selected follower mode
    may receive targets, after this session's start is confirmed in feedback.
    It does not convert absolute poses or per-frame VR deltas. Joint feedback
    defaults to degrees on the wire per the template; set feedback_joint_unit
    to the verified controller convention. Position units are unknown by
    default. get_state() remains controller-only. get_frame() can compose
    cameras and an explicitly selected USB GripperOperator. Configure
    gripper_feedback_path only for a verified controller millimetre field.

    Motion commands require allow_motion=True. close() closes network resources
    only; call stop_follower() explicitly to request and confirm mode exit. Mode
    exit is not a physical emergency stop. No automatic motor enable or login.
    """

    capabilities = frozenset({'relative_cartesian_follower', 'absolute_joint_follower',
                              'gripper_position'})
    command_mode = 'cartesian'

    def __init__(self, host, ws_port=5999, udp_port=9998, timeout_s=2.0,
                 allow_motion=False, arm_name='right', feedback_joint_unit='deg',
                 feedback_position_unit=None, gripper_feedback_path=None,
                 joint_udp_port=9999, command_mode='cartesian',
                 cameras=None, gripper=None, gripper_transport='controller',
                 trajectory_feedback_interval_s=0.1,
                 trajectory_feedback_timeout_s=0.015,
                 trajectory_max_lateness_s=None):
        super().__init__(sync_warning_enabled=False)
        if not isinstance(host, str) or not host or any(c in host for c in '/?#@'):
            raise ValueError('host must be a controller hostname or IP')
        if any(type(p) is not int or not 1 <= p <= 65535
               for p in (ws_port, udp_port, joint_udp_port)):
            raise ValueError('Ports must be integers within 1..65535')
        if joint_udp_port == udp_port:
            raise ValueError('Cartesian and joint UDP ports must differ')
        if command_mode not in _FOLLOWER_MODES:
            raise ValueError('command_mode must be cartesian or joint')
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('timeout_s must be positive and finite')
        if type(allow_motion) is not bool or not isinstance(arm_name, str) or not arm_name:
            raise ValueError('allow_motion must be bool and arm_name must be nonempty')
        if feedback_joint_unit not in ('deg', 'rad'):
            raise ValueError('feedback_joint_unit must be deg or rad')
        if feedback_position_unit not in (None, 'mm', 'm'):
            raise ValueError('feedback_position_unit must be None, mm or m')
        if gripper_feedback_path is not None and (
                not isinstance(gripper_feedback_path, (list, tuple))
                or not gripper_feedback_path
                or any(not isinstance(k, str) or not k for k in gripper_feedback_path)):
            raise ValueError('gripper_feedback_path must be a nonempty list of keys')
        if gripper_transport not in ('controller', 'usb'):
            raise ValueError('gripper_transport must be controller or usb')
        if gripper_transport == 'usb' and not isinstance(gripper, Mapping):
            raise ValueError('USB gripper requires explicit gripper settings')
        if gripper_transport == 'controller' and gripper is not None:
            raise ValueError('USB settings require gripper_transport="usb"')
        reserved = {'qpos', 'joints', 'gripper_mm', 'tcp_pq_m', 'follower_active',
                    'follower_mode', 'has_error', 'received_monotonic_s',
                    'gripper_received_monotonic_s', 'camera_received_monotonic_s'}
        if cameras is not None and (not isinstance(cameras, dict)
                or any(not isinstance(k, str) or not k or k in reserved for k in cameras)):
            raise ValueError('Camera names must be nonempty and not overwrite robot fields')
        self.camera_operator = None if cameras is None else CameraOperator(cameras)
        self.gripper_transport = gripper_transport
        self.gripper_operator = (GripperOperator(**dict(gripper))
                                 if gripper_transport == 'usb' else None)
        self.gripper_max_width_mm = (self.gripper_operator.max_width_mm
                                    if self.gripper_operator is not None else 100.)
        for name, value in [('trajectory_feedback_interval_s', trajectory_feedback_interval_s),
                            ('trajectory_feedback_timeout_s', trajectory_feedback_timeout_s),
                            ('trajectory_max_lateness_s', trajectory_max_lateness_s)]:
            if value is not None and (isinstance(value, bool)
                                      or not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be positive and finite')
        self.trajectory_feedback_interval_s = float(trajectory_feedback_interval_s)
        self.trajectory_feedback_timeout_s = float(trajectory_feedback_timeout_s)
        self.trajectory_max_lateness_s = trajectory_max_lateness_s
        self._trajectory_lock = threading.Lock()
        self._trajectory_owner = None
        self.last_trajectory = None
        self.allow_motion, self.arm_name = allow_motion, arm_name
        self.feedback_joint_unit = feedback_joint_unit
        self.feedback_position_unit = feedback_position_unit
        self.gripper_feedback_path = gripper_feedback_path
        self.timeout_s = float(timeout_s)
        self.command_mode = command_mode
        self._connection = _FollowerConnection(host, ws_port, udp_port, self.timeout_s,
                                               joint_udp_port)
        self._io_lock = threading.RLock()
        self._confirmed_active = False
        self._started_here = False
        self._start_confirmed = False
        self._follower_mode = None

    @property
    def is_connected(self):
        return (self._connection.ws is not None and self._connection.udp is not None
                and self._connection.joint_udp is not None)

    @property
    def follower_stop_pending(self):
        """This session sent a start that hasn't been confirmed inactive yet."""
        return self._started_here

    def connect(self):
        with self._io_lock:
            if not self.is_connected:
                self._confirmed_active = self._started_here = False
                self._start_confirmed = False
                self._follower_mode = None
                try:
                    self._connection.connect()
                    if self.gripper_operator is not None:
                        self.gripper_operator.connect()
                except BaseException:
                    self.close()
                    raise

    def close(self):
        self._traj_stop_event.set()
        with self._io_lock:
            self._confirmed_active = self._started_here = False
            self._start_confirmed = False
            self._follower_mode = None
            errors = []
            closers = [self._connection.close]
            if self.camera_operator is not None:
                closers.append(self.camera_operator.close)
            if self.gripper_operator is not None:
                closers.append(self.gripper_operator.disconnect)
            for closer in closers:
                try:
                    closer()
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise RuntimeError(f'A10 resource cleanup failed: {errors}') from errors[0]

    def _require_motion(self):
        if not self.allow_motion:
            raise PermissionError('A10 allow_motion is false')
        if not self.is_connected:
            raise ConnectionError('A10 operator is not connected')

    def get_state(self, *, timeout_s=None):
        with self._io_lock:
            try:
                reply = self._connection.request('get', status=True, timeout_s=timeout_s)
                state = _parse_state(reply, joint_unit=self.feedback_joint_unit,
                                     position_unit=self.feedback_position_unit,
                                     gripper_feedback_path=self.gripper_feedback_path)
                self._confirmed_active = (self._started_here
                                          and self._start_confirmed
                                          and state.follower_active is True
                                          and state.follower_mode == self._follower_mode
                                          and state.raw['ret_context'].get('robot_msg', {}).get('activate') == 'Enabled'
                                          and not state.has_error)
                return state
            except Exception:
                self._confirmed_active = False
                self._connection.close()
                raise

    def get_frame(self, slop=0.7):
        """Actual robot + optional USB gripper + RGB frames, with receive times.

        These sequential reads are not synchronized sensor samples. Missing
        configured hardware raises; targets are never substituted for feedback.
        get_state() remains the unmodified controller-only status interface.
        """
        del slop
        with self._io_lock:
            try:
                state = self.get_state()
                width = state.gripper_mm
                frame = dict(joints=state.actual_joints_rad, tcp_pq_m=state.tcp_pq_m,
                             follower_active=state.follower_active,
                             follower_mode=state.follower_mode, has_error=state.has_error,
                             received_monotonic_s=state.received_monotonic_s)
                if self.gripper_operator is not None:
                    grip = self.gripper_operator.read_state()
                    width = grip.position_mm
                    frame['gripper_received_monotonic_s'] = grip.timestamp
                frame['gripper_mm'] = width
                frame['qpos'] = (None if state.actual_joints_rad is None or width is None
                                 else np.concatenate((state.actual_joints_rad, [width])))
                if self.camera_operator is not None:
                    frame.update(self.camera_operator.get_frame())
                return frame
            except BaseException:
                self.close()
                raise

    def _wait_mode(self, active):
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
            if state.follower_active is active and (not active or state.follower_mode == self._follower_mode):
                if active and state.has_error:
                    raise A10ControllerError('Controller reports a fault')
                if active and state.raw['ret_context'].get('robot_msg', {}).get('activate') != 'Enabled':
                    raise A10ControllerError('Follower running but motion permission is not confirmed')
                if active:
                    self._start_confirmed = self._confirmed_active = True
                return state
            if state.has_error and active:
                raise A10ControllerError('Controller reports a fault')
            time.sleep(min(.02, max(0., deadline - time.monotonic())))
        self._confirmed_active = False
        raise TimeoutError('Follower mode change was not confirmed')

    def enable(self):
        """Request manual_en in Manual mode and confirm Enabled feedback."""
        return self._set_manual_enable(True)

    def disable(self):
        """Request manual_ds in Manual mode; this is not an emergency stop."""
        return self._set_manual_enable(False)

    def _set_manual_enable(self, enabled):
        with self._io_lock:
            self._require_motion()
            state = self.get_state()
            info = state.raw['ret_context'].get('robot_msg', {})
            if info.get('op_mode') != 'Manual':
                raise RuntimeError('Manual enable/disable requires Manual mode; no automatic mode switch')
            if enabled:
                if state.has_error:
                    raise A10ControllerError('Controller reports a fault')
                if info.get('motion') != 'Stopped' or state.follower_active is not False:
                    raise RuntimeError('Enable requires Stopped feedback and inactive follower')
            self._confirmed_active = False
            self._connection.request('manual_en' if enabled else 'manual_ds')
            expected = 'Enabled' if enabled else 'Disabled'
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
                info = state.raw['ret_context'].get('robot_msg', {})
                if info.get('op_mode') != 'Manual':
                    raise RuntimeError('Controller left Manual mode during enable/disable')
                if enabled and state.has_error:
                    raise A10ControllerError('Controller reports a fault')
                if info.get('activate') == expected:
                    return state
                time.sleep(min(.02, max(0., deadline - time.monotonic())))
            raise TimeoutError(f'{expected} feedback was not confirmed')

    def set_tool_coordinate(self):
        """Explicitly select Tool jog coordinates and verify feedback; no target sent."""
        return self.set_jog_coordinate('Tool')

    def set_jog_coordinate(self, coordinate):
        """Explicitly select Axis or Tool jog coordinates and verify feedback."""
        if coordinate not in ('Axis', 'Tool'):
            raise ValueError('coordinate must be Axis or Tool')
        with self._io_lock:
            self._require_motion()
            state = self.get_state()
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is not False:
                raise RuntimeError('Stop the follower before changing jog coordinates')
            if state.raw['ret_context'].get('robot_msg', {}).get('motion') != 'Stopped':
                raise RuntimeError('Coordinate change requires confirmed Stopped feedback')
            self._connection.request(f'set_jog_coordinate --{coordinate.lower()}')
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
                if state.has_error or state.follower_active is not False:
                    raise A10ControllerError('Controller fault or follower state changed during coordinate selection')
                if state.raw['ret_context']['motion_msg'].get('jog_coordinate') == coordinate:
                    return state
                time.sleep(min(.02, max(0., deadline - time.monotonic())))
            raise TimeoutError(f'{coordinate} jog coordinate was not confirmed')

    def start_follower(self, mode=None):
        """Start cartesian (default) or joint follower; never send an initial target."""
        mode = self.command_mode if mode is None else mode
        if mode not in _FOLLOWER_MODES:
            raise ValueError('mode must be cartesian or joint')
        command, feedback_mode = _FOLLOWER_MODES[mode]
        with self._io_lock:
            self._require_motion()
            if self._started_here:
                raise RuntimeError('This session has a pending follower; stop_follower before switching modes')
            state = self.get_state()
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is not False:
                raise RuntimeError('Follower is active or its state is unknown; refusing takeover')
            if state.actual_joints_rad is None or (mode == 'cartesian' and state.tcp_pq_raw is None):
                raise A10ProtocolError('Follower start requires joint and TCP feedback')
            info = state.raw['ret_context'].get('robot_msg', {})
            if info.get('activate') != 'Enabled' or info.get('motion') != 'Stopped':
                raise RuntimeError('Follower start requires Enabled and Stopped feedback')
            coordinate = state.raw['ret_context']['motion_msg'].get('jog_coordinate')
            if mode == 'cartesian' and coordinate not in ('Base', 'Tool', 'Wobj'):
                raise RuntimeError(
                    f'Cartesian follower requires Cartesian jog coordinates, got {coordinate!r}; '
                    'explicitly call set_tool_coordinate() before starting')
            # Record the attempt before send: Ctrl+C can arrive after bytes
            # leave the socket but before send() returns to Python.
            self._started_here = True
            self._start_confirmed = False
            self._confirmed_active = False
            self._follower_mode = feedback_mode
            self.command_mode = mode
            try:
                self._connection.send_command(command)
                return self._wait_mode(True)
            except A10ControllerError:
                # A rejection may precede the outstanding get reply. Never reuse
                # that queue or retry the start: inspect state on a fresh socket.
                # Only confirmed inactivity clears this attempt's cleanup flag.
                if not self.is_connected:
                    try:
                        self._connection.connect()
                        recovered = self.get_state()
                        if recovered.follower_active is False:
                            self._started_here = False
                            self._follower_mode = None
                    except Exception:
                        self._connection.close()
                self._confirmed_active = False
                raise

    def stop_follower(self):
        self._traj_stop_event.set()
        with self._io_lock:
            self._require_motion()
            if not self._started_here:
                raise RuntimeError('This connection did not start the follower')
            self._confirmed_active = False
            self._start_confirmed = False
            self._connection.send_command('stop_follower')
            state = self._wait_mode(False)
            self._started_here = False
            self._follower_mode = None
            return state

    def send_eepose(self, arm_targets):
        """Send one origin-relative pose in metres/xyzw for the configured arm."""
        if not isinstance(arm_targets, Mapping) or set(arm_targets) != {self.arm_name}:
            raise ValueError(f'Expected exactly one arm: {self.arm_name}')
        pose = _pose(arm_targets[self.arm_name])
        payload = json.dumps({'type': 'pq', 'pq': [pose.tolist()]},
                             separators=(',', ':'), allow_nan=False).encode('utf-8')
        self._send_follower_target(_pack(payload, command=False), 'Cart')

    def _send_follower_target(self, packet, mode):
        with self._io_lock:
            self._require_target_owner()
            self._require_motion()
            if (not self._started_here or not self._confirmed_active
                    or self._follower_mode != mode):
                raise RuntimeError(f'{mode} follower start has not been confirmed')
            udp = self._connection.udp if mode == 'Cart' else self._connection.joint_udp
            try:
                if udp.send(packet) != len(packet):
                    raise ConnectionError('Incomplete UDP datagram send')
            except OSError:
                self._confirmed_active = False
                raise
        # UDP has no application ACK. This return does not mean execution.

    def send_gripper(self, gripper_targets, wait=False):
        """Explicit USB or controller route; only USB can wait for actual arrival."""
        if not isinstance(gripper_targets, Mapping) or set(gripper_targets) != {self.arm_name}:
            raise ValueError(f'Expected exactly one gripper: {self.arm_name}')
        position = float(_vector([gripper_targets[self.arm_name]], 1, 'gripper mm')[0])
        if not 0 <= position <= self.gripper_max_width_mm:
            raise ValueError(f'Gripper target must be within 0..{self.gripper_max_width_mm} mm')
        if wait and self.gripper_transport != 'usb':
            raise NotImplementedError('Controller gripper arrival feedback is not verified')
        return self._send_gripper_target(position, wait=wait)

    def _send_gripper_target(self, position, *, wait=False, check_state=True):
        with self._io_lock:
            self._require_target_owner()
            self._require_motion()
            if check_state and self.get_state().has_error:
                raise A10ControllerError('Controller reports a fault')
            if self.gripper_operator is not None:
                return self.gripper_operator.set_position_mm(position, wait=wait)
            return self._connection.request(f'g_move --pos={position:.12g}')

    def _require_target_owner(self):
        if self._trajectory_owner is not None:
            if self._trajectory_owner != threading.get_ident():
                raise RuntimeError('A trajectory already owns the target stream')
            if self._traj_stop_event.is_set():
                raise InterruptedError('A10 trajectory was cancelled')

    def _check_trajectory_feedback(self, timeout_s, *, check_gripper):
        state = self.get_state(timeout_s=timeout_s)
        if state.has_error:
            raise A10ControllerError('Controller fault during trajectory')
        if not self._confirmed_active or self._follower_mode != 'Axis':
            raise RuntimeError('Joint follower permission lost during trajectory')
        if state.actual_joints_rad is None:
            raise A10ProtocolError('Trajectory requires actual joint feedback')
        if check_gripper:
            self.gripper_operator.read_state(timeout=timeout_s)

    def execute_trajectory(self, arm_trajectories, gripper_trajectories=None,
                           head_trajectory=None, dt=0.1, async_exec=False):
        """Send absolute joint rad / gripper mm rows on a monotonic clock.

        Synchronous only. Validate the WHOLE chunk before any target. First
        send is immediate after bounded preflight checks; return after T*dt
        seconds means sending ended, not physical arrival or UDP receipt.
        Periodic status/servo checks interrupt a faulted stream. Overdue slots
        abort, with no retry, interpolation, catch-up burst or ROS dependency.
        Caller owns follower start/stop (the runner stops in its finally).
        """
        if async_exec is not False:
            raise NotImplementedError('A10 trajectories support async_exec=False only')
        if head_trajectory is not None:
            raise ValueError('A10 single arm has no head trajectory')
        if isinstance(dt, bool) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        if not isinstance(arm_trajectories, Mapping) or set(arm_trajectories) != {self.arm_name}:
            raise ValueError(f'Expected exactly one arm: {self.arm_name}')
        raw = np.asarray(arm_trajectories[self.arm_name])
        if raw.ndim != 2 or raw.shape[1] != 6 or not len(raw) or raw.dtype.kind not in 'iuf':
            raise ValueError('Arm trajectory must be numeric nonempty [T,6] radians')
        joints = np.array(raw, dtype=np.float64, copy=True)
        with np.errstate(over='ignore'):
            if not np.isfinite(joints).all() or not np.isfinite(np.rad2deg(joints)).all():
                raise ValueError('Arm trajectory must remain finite after degree conversion')
        if not math.isfinite(len(joints) * dt):
            raise ValueError('Trajectory duration must be finite')
        widths = None
        if gripper_trajectories is not None and not (isinstance(gripper_trajectories, Mapping) and not gripper_trajectories):
            if (not isinstance(gripper_trajectories, Mapping)
                    or set(gripper_trajectories) != {self.arm_name}):
                raise ValueError(f'Expected exactly one gripper: {self.arm_name}')
            raw = np.asarray(gripper_trajectories[self.arm_name])
            if raw.shape != (len(joints),) or raw.dtype.kind not in 'iuf':
                raise ValueError('Gripper trajectory must be numeric [T] matching arm length')
            widths = np.array(raw, dtype=np.float64, copy=True)
            if (not np.isfinite(widths).all() or np.any(widths < 0)
                    or np.any(widths > self.gripper_max_width_mm)):
                raise ValueError('Gripper trajectory exceeds the configured working range')
            if self.gripper_transport != 'usb':
                raise NotImplementedError('Timed gripper trajectories require the USB driver')
        lateness = dt * .5 if self.trajectory_max_lateness_s is None else self.trajectory_max_lateness_s
        if lateness >= dt:
            raise ValueError('trajectory_max_lateness_s must be smaller than dt')
        if not self._trajectory_lock.acquire(blocking=False):
            raise RuntimeError('A trajectory is already running')
        sent = 0
        started = None
        completed = False
        try:
            with self._io_lock:
                self._require_motion()
                if (not self._started_here or not self._start_confirmed
                        or not self._confirmed_active or self._follower_mode != 'Axis'):
                    raise RuntimeError('Confirmed owned joint follower required for trajectory')
                self._trajectory_owner = threading.get_ident()
                self._traj_stop_event.clear()
                feedback_timeout = min(self.trajectory_feedback_timeout_s, dt / 2)
                self._check_trajectory_feedback(feedback_timeout, check_gripper=widths is not None)
            started = last_feedback = time.monotonic()
            for index, target in enumerate(joints):
                deadline = started + index * dt
                self._traj_stop_event.wait(max(0., deadline - time.monotonic()))
                with self._io_lock:
                    self._require_target_owner()
                    now = time.monotonic()
                    if now - last_feedback >= self.trajectory_feedback_interval_s:
                        self._check_trajectory_feedback(feedback_timeout, check_gripper=widths is not None)
                        last_feedback = time.monotonic()
                    if time.monotonic() - deadline > lateness:
                        raise TimeoutError(f'Trajectory slot {index} missed; no catch-up sends')
                    self.send_joints({self.arm_name: target})
                    if widths is not None:
                        self._send_gripper_target(float(widths[index]), check_state=False)
                    sent += 1
                    if time.monotonic() > deadline + dt:
                        raise TimeoutError(f'Trajectory slot {index} exceeded its time budget')
            self._traj_stop_event.wait(max(0., started + len(joints) * dt - time.monotonic()))
            with self._io_lock:
                self._require_target_owner()
                # A final check also catches faults in shorter-than-monitor chunks.
                self._check_trajectory_feedback(feedback_timeout, check_gripper=widths is not None)
            completed = True
        except BaseException:
            with self._io_lock:
                self._confirmed_active = self._start_confirmed = False
                self._traj_stop_event.set()
            raise
        finally:
            with self._io_lock:
                self.last_trajectory = dict(selected=len(joints), sent=sent,
                    completed=completed, physical_arrival_confirmed=False,
                    elapsed_s=None if started is None else time.monotonic() - started)
                self._trajectory_owner = None
            self._trajectory_lock.release()

    def stop_trajectory(self):
        """Cancel pending sends and explicitly stop this session's follower."""
        self._traj_stop_event.set()
        if self.follower_stop_pending:
            return self.stop_follower()
        return None

    def is_trajectory_running(self):
        """Whether this operator is currently scheduling a chunk, not follower mode."""
        return self._trajectory_owner is not None

    def send_joints(self, arm_targets):
        """Send six absolute joint radians via joint follower UDP (wire degrees).

        No gripper, implicit hold, interpolation, retry, or arrival ACK. Client
        targets must match controller joint order; controller plans motion.
        """
        if not isinstance(arm_targets, Mapping) or set(arm_targets) != {self.arm_name}:
            raise ValueError(f'Expected exactly one arm: {self.arm_name}')
        radians = _vector(arm_targets[self.arm_name], 6, 'joint targets in radians')
        with np.errstate(over='ignore'):
            degrees = np.rad2deg(radians)
        if not np.isfinite(degrees).all():
            raise A10ProtocolError('Joint conversion to degrees must remain finite')
        values = ','.join(format(float(v), '.17g') for v in degrees)
        payload = f'jp --motor_pos=[{values}]'.encode('utf-8')
        self._send_follower_target(_pack(payload, command=False), 'Axis')

    def gohome(self):
        raise NotImplementedError('No verified home command for this follower adapter')
