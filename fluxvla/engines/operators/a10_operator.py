"""A10 single-arm operator for the KAANH WebSocket / UDP follower protocol.

Wire layout follows the user-supplied robot.7z reference. Controller-side
planning stays on the controller. This module does not open connections at
import/construction, enable motors, interpret VR increments, or invent a
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

__all__ = ['A10Operator', 'A10State', 'A10ProtocolError', 'A10ControllerError']

_HEADER = struct.Struct('<IIQqqq')
_MAX_PAYLOAD = 1024 * 1024


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

    def __init__(self, host, ws_port, udp_port, timeout_s):
        self.host, self.ws_port, self.udp_port = host, ws_port, udp_port
        self.timeout_s = timeout_s
        self.ws = self.udp = None

    def connect(self):
        from websockets.sync.client import connect
        try:
            self.ws = connect(f'ws://{self.host}:{self.ws_port}', proxy=None,
                              open_timeout=self.timeout_s, close_timeout=self.timeout_s,
                              max_size=_MAX_PAYLOAD + _HEADER.size)
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.settimeout(self.timeout_s)
            self.udp.connect((self.host, self.udp_port))
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
        ws, udp = self.ws, self.udp
        self.ws = self.udp = None
        try:
            if ws is not None:
                ws.close()
        finally:
            if udp is not None:
                udp.close()


@OPERATORS.register_module()
class A10Operator(BaseOperator):
    """Six-axis A10 + gripper, using the reference KAANH follower protocol.

    ``send_eepose({'right': pose})`` expects origin-relative xyz metres + xyzw.
    It does not convert absolute poses or per-frame VR deltas. Joint feedback
    defaults to degrees on the wire per the template; set feedback_joint_unit
    to the verified controller convention. Position units are unknown by
    default. Configure gripper_feedback_path relative to ret_context only
    after confirming a real numeric millimetre feedback field.

    Motion commands require allow_motion=True. close() closes network resources
    only; call stop_follower() explicitly to request and confirm mode exit. Mode
    exit is not a physical emergency stop. No automatic motor enable or login.
    """

    capabilities = frozenset({'relative_cartesian_follower', 'gripper_position'})
    command_mode = 'cartesian'

    def __init__(self, host, ws_port=5999, udp_port=9998, timeout_s=2.0,
                 allow_motion=False, arm_name='right', feedback_joint_unit='deg',
                 feedback_position_unit=None, gripper_feedback_path=None):
        super().__init__(sync_warning_enabled=False)
        if not isinstance(host, str) or not host or any(c in host for c in '/?#@'):
            raise ValueError('host must be a controller hostname or IP')
        if any(type(p) is not int or not 1 <= p <= 65535 for p in (ws_port, udp_port)):
            raise ValueError('Ports must be integers within 1..65535')
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
        self.allow_motion, self.arm_name = allow_motion, arm_name
        self.feedback_joint_unit = feedback_joint_unit
        self.feedback_position_unit = feedback_position_unit
        self.gripper_feedback_path = gripper_feedback_path
        self.timeout_s = float(timeout_s)
        self._connection = _FollowerConnection(host, ws_port, udp_port, self.timeout_s)
        self._io_lock = threading.RLock()
        self._confirmed_active = False
        self._started_here = False

    @property
    def is_connected(self):
        return self._connection.ws is not None and self._connection.udp is not None

    def connect(self):
        with self._io_lock:
            if not self.is_connected:
                self._confirmed_active = self._started_here = False
                self._connection.connect()

    def close(self):
        with self._io_lock:
            self._confirmed_active = self._started_here = False
            self._connection.close()

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
                self._confirmed_active = (state.follower_active is True
                                          and state.follower_mode == 'Cart'
                                          and not state.has_error)
                return state
            except Exception:
                self._confirmed_active = False
                self._connection.close()
                raise

    def get_frame(self, slop=0.7):
        """Return robot feedback only; this operator does not acquire cameras."""
        del slop
        state = self.get_state()
        return dict(qpos=state.qpos, joints=state.actual_joints_rad,
                    gripper_mm=state.gripper_mm, tcp_pq_m=state.tcp_pq_m,
                    follower_active=state.follower_active, follower_mode=state.follower_mode,
                    has_error=state.has_error, received_monotonic_s=state.received_monotonic_s)

    def _wait_mode(self, active):
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is active and (not active or state.follower_mode == 'Cart'):
                return state
            time.sleep(min(.02, max(0., deadline - time.monotonic())))
        self._confirmed_active = False
        raise TimeoutError('Follower mode change was not confirmed')

    def set_tool_coordinate(self):
        """Explicitly select Tool jog coordinates and verify feedback; no target sent."""
        with self._io_lock:
            self._require_motion()
            state = self.get_state()
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is not False:
                raise RuntimeError('Stop the follower before changing jog coordinates')
            if state.raw['ret_context'].get('robot_msg', {}).get('motion') != 'Stopped':
                raise RuntimeError('Coordinate change requires confirmed Stopped feedback')
            self._connection.request('set_jog_coordinate --tool')
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                state = self.get_state(timeout_s=max(.001, deadline - time.monotonic()))
                if state.has_error or state.follower_active is not False:
                    raise A10ControllerError('Controller fault or follower state changed during coordinate selection')
                if state.raw['ret_context']['motion_msg'].get('jog_coordinate') == 'Tool':
                    return state
                time.sleep(min(.02, max(0., deadline - time.monotonic())))
            raise TimeoutError('Tool jog coordinate was not confirmed')

    def start_follower(self):
        with self._io_lock:
            self._require_motion()
            state = self.get_state()
            if state.has_error:
                raise A10ControllerError('Controller reports a fault')
            if state.follower_active is not False:
                raise RuntimeError('Follower is active or its state is unknown; refusing takeover')
            if state.actual_joints_rad is None or state.tcp_pq_raw is None:
                raise A10ProtocolError('Follower start requires joint and TCP feedback')
            coordinate = state.raw['ret_context']['motion_msg'].get('jog_coordinate')
            if coordinate not in ('Base', 'Tool', 'Wobj'):
                raise RuntimeError(
                    f'Cartesian follower requires Cartesian jog coordinates, got {coordinate!r}; '
                    'explicitly call set_tool_coordinate() before starting')
            self._connection.send_command('follower_cart')
            self._started_here = True
            try:
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
                    except Exception:
                        self._connection.close()
                self._confirmed_active = False
                raise

    def stop_follower(self):
        with self._io_lock:
            self._require_motion()
            if not self._started_here:
                raise RuntimeError('This connection did not start the follower')
            self._confirmed_active = False
            self._connection.send_command('stop_follower')
            state = self._wait_mode(False)
            self._started_here = False
            return state

    def send_eepose(self, arm_targets):
        """Send one origin-relative pose in metres/xyzw for the configured arm."""
        if not isinstance(arm_targets, Mapping) or set(arm_targets) != {self.arm_name}:
            raise ValueError(f'Expected exactly one arm: {self.arm_name}')
        pose = _pose(arm_targets[self.arm_name])
        payload = json.dumps({'type': 'pq', 'pq': [pose.tolist()]},
                             separators=(',', ':'), allow_nan=False).encode('utf-8')
        with self._io_lock:
            self._require_motion()
            if not self._started_here or not self._confirmed_active:
                raise RuntimeError('Cartesian follower start has not been confirmed')
            packet = _pack(payload, command=False)
            try:
                if self._connection.udp.send(packet) != len(packet):
                    raise ConnectionError('Incomplete UDP datagram send')
            except OSError:
                self._confirmed_active = False
                raise
        # UDP has no application ACK. This return does not mean execution.

    def send_gripper(self, gripper_targets, wait=False):
        """Request g_move in millimetres; ACK does not confirm position reached."""
        if wait:
            raise NotImplementedError('Gripper arrival feedback is not yet verified')
        if not isinstance(gripper_targets, Mapping) or set(gripper_targets) != {self.arm_name}:
            raise ValueError(f'Expected exactly one gripper: {self.arm_name}')
        position = float(_vector([gripper_targets[self.arm_name]], 1, 'gripper mm')[0])
        if not 0 <= position <= 100:
            raise ValueError('Gripper target must be within 0..100 mm')
        with self._io_lock:
            self._require_motion()
            if self.get_state().has_error:
                raise A10ControllerError('Controller reports a fault')
            return self._connection.request(f'g_move --pos={position:.12g}')

    def execute_trajectory(self, *args, **kwargs):
        raise NotImplementedError('Use send_eepose; trajectory timing belongs to the controller')

    def stop_trajectory(self):
        return self.stop_follower()

    def is_trajectory_running(self):
        return self.get_state().follower_active

    def send_joints(self, arm_targets):
        raise NotImplementedError('Joint follower protocol is outside this Cartesian adapter')

    def gohome(self):
        raise NotImplementedError('No verified home command for this follower adapter')
