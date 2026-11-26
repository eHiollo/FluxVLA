"""TimedEe transport; absolute tool1/wobj0 poses, physical ee0 then ee1.

The relay supplies its existing serialized command connection. Only the long
Plan response uses a separate socket, as in the upstream client's start_plan.
Completion here describes command FK, never measured robot feedback.
"""
import json
import math
import socket
import struct
import threading
import time

import numpy as np

from .a10_timed_aj import timed_config as _joint_transport_config


def timed_config(value):
    """Transport options only; TimedEe's fixed limits belong to the controller."""
    try:
        return _joint_transport_config(value)
    except ValueError as exc:
        raise ValueError(str(exc).replace('timed_aj', 'timed_ee')) from exc


def validated_poses(value):
    pose = np.asarray(value)
    if (pose.shape != (2, 7) or pose.dtype.kind not in 'iuf'
            or not np.isfinite(pose).all()):
        raise ValueError('Expected two finite [x_m,y_m,z_m,qx,qy,qz,qw] poses, ee0 then ee1')
    pose = pose.astype(float)
    norm2 = np.sum(pose[:, 3:] ** 2, axis=1)
    if np.any(np.abs(norm2 - 1.) > .002):
        raise ValueError('TimedEe quaternion squared norm error must be <=0.002')
    # Match upstream encode_ee_target: normalize only after the near-unit check,
    # preserving quaternion sign (q and -q describe the same rotation).
    pose[:, 3:] /= np.sqrt(norm2)[:, None]
    return pose


def encode_target(stream_id, index, pose_m_quat, final=False):
    if (type(stream_id) is not int or not 0 < stream_id < 2**64
            or type(index) is not int or not 0 <= index < 2**64):
        raise ValueError('Invalid timed stream ID or index')
    if type(final) is not bool:
        raise ValueError('final must be bool')
    poses = validated_poses(pose_m_quat)
    return json.dumps(dict(stream_id=stream_id, index=index,
                           pose_m_quat=poses.tolist(), frame='wobj0', tool='tool1', final=final),
                      separators=(',', ':'), allow_nan=False).encode('utf-8')


def validate_status_header(reply):
    if (not isinstance(reply, dict) or type(reply.get('ret_code')) is not int
            or reply['ret_code'] != 0):
        raise RuntimeError(f'Invalid tee_status reply: {reply}')
    if (type(reply.get('active')) is not bool or type(reply.get('stream_id')) is not int
            or not 0 <= reply['stream_id'] < 2**64):
        raise ValueError('tee_status requires bool active and an unsigned stream_id')
    return reply


def validate_status(reply):
    validate_status_header(reply)
    identity = dict(implementation='joint_tracking.TimedEe',
                    control_space='ee', tracking_profile='capture_fast',
                    pose_convention='xyz_m_quaternion_xyzw', frame='wobj0', tool='tool1',
                    path_policy='per_target_ik_then_capture_fast_joint_tracking',
                    ee_pose_source='command_fk',
                    gap_policy='hold_and_resume')
    for key, expected in identity.items():
        if reply.get(key) != expected:
            raise RuntimeError(f'tee_status requires {key}={expected!r}; received {reply.get(key)!r} '
                               f'(active={reply.get("active")}, stream_id={reply.get("stream_id")})')
    # Idle tee_status retains the previous run, including a sample/file source.
    # That history is not an active UDP task. Never accept a non-UDP active run.
    source = reply.get('input_source')
    if not isinstance(source, str) or not source or (reply.get('active') is True and source != 'udp'):
        raise RuntimeError('Active tee_status requires input_source=udp')
    if reply.get('cartesian_line_guaranteed') is not False:
        raise RuntimeError('TimedEe must declare cartesian_line_guaranteed=false')
    params = reply.get('parameters')
    policy = reply.get('ee_endpoint_policy')
    if policy not in ('exact_ik_goal', 'bounded_cartesian_capture'):
        raise RuntimeError('Unsupported TimedEe endpoint policy')
    tolerance = params.get('endpoint_tolerance_deg') if isinstance(params, dict) else None
    if (type(tolerance) not in (int, float) or not math.isfinite(tolerance)
            or not 0 <= tolerance <= (.3 if policy == 'bounded_cartesian_capture' else 0)):
        raise RuntimeError('Unexpected TimedEe endpoint joint tolerance')
    bounded = policy == 'bounded_cartesian_capture'
    for key, expected in (('endpoint_position_tolerance_m', .002 if bounded else .001),
                          ('endpoint_orientation_tolerance_rad', math.radians(.2 if bounded else .1))):
        val = reply.get(key)
        if type(val) not in (int, float) or not math.isclose(val, expected, rel_tol=1e-9):
            raise RuntimeError(f'Unexpected tee_status.{key}')
    if bounded or 'endpoint_capture_complete' in reply:
        if type(reply.get('endpoint_capture_complete')) is not bool:
            raise ValueError('tee_status.endpoint_capture_complete must be bool')
    for key in ('active', 'ready', 'accepting', 'stop_requested', 'waiting_for_input',
                'hold_transition', 'end_received', 'final_reference_reached'):
        if type(reply.get(key)) is not bool:
            raise ValueError(f'tee_status.{key} must be bool')
    for key in ('stream_id', 'received', 'consumed', 'queue_depth', 'state', 'reason'):
        if type(reply.get(key)) is not int or reply[key] < 0:
            raise ValueError(f'tee_status.{key} must be a nonnegative integer')
    if reply['stream_id'] >= 2**64 or reply['state'] > 6 or reply['reason'] > 11:
        raise ValueError('Invalid tee_status range')
    return reply


def require_stream(reply, stream_id, *, accepting=False):
    validate_status(reply)
    if (reply.get('input_source') != 'udp'
            or reply['stream_id'] != stream_id or not reply['active'] or reply['reason'] != 0
            or reply['state'] in (4, 5, 6) or reply['stop_requested']
            or (accepting and not (reply['ready'] and reply['accepting']))):
        raise RuntimeError(f'timed_ee stream interrupted/not ready: {reply}')
    return reply


def _finite_flat(reply, key, sizes):
    value = np.asarray(reply.get(key))
    if (value.dtype.kind not in 'iuf' or value.size not in sizes
            or not np.isfinite(value).all()):
        raise ValueError(f'Invalid tee_status.{key}')
    return value.astype(float).ravel()


def endpoint_reached(reply, expected_points):
    """One complete command-side endpoint sample; no extra settling delay.

    Fast-v2 requires capture completion plus its declared Cartesian band and
    joint candidate tolerance. Historical exact_ik_goal retains a numerical
    1e-6 rad comparison. Neither establishes actual motor arrival.
    """
    validate_status(reply)
    command = _finite_flat(reply, 'command_joint_rad', (12,))
    goal = _finite_flat(reply, 'joint_goal_rad', (12,))
    speed = _finite_flat(reply, 'command_joint_velocity_rad_s', (12,))
    position_error = _finite_flat(reply, 'endpoint_position_error_m', (1, 2))
    orientation_error = _finite_flat(reply, 'endpoint_orientation_error_rad', (1, 2))
    if np.any(position_error < 0) or np.any(orientation_error < 0):
        raise ValueError('TimedEe endpoint residuals must be nonnegative')
    bounded = reply['ee_endpoint_policy'] == 'bounded_cartesian_capture'
    joint_tolerance = math.radians(reply['parameters']['endpoint_tolerance_deg']) + 1e-9 if bounded else 1e-6
    return bool(reply['active'] and not reply['reason'] and not reply['stop_requested']
                and reply['state'] not in (4, 5, 6)
                and reply.get('endpoint_capture_complete', not bounded)
                and reply['end_received'] and reply['final_reference_reached']
                and reply['received'] == expected_points and reply['consumed'] == expected_points
                and np.max(np.abs(command - goal)) <= joint_tolerance
                and np.max(np.abs(speed)) <= math.radians(.5)
                and np.max(position_error) <= reply['endpoint_position_tolerance_m']
                and np.max(orientation_error) <= reply['endpoint_orientation_tolerance_rad'])


class TimedEeTransport:
    """Relay-owned transport. Never resend a point or stop another stream.

    Caller schedules each new point once at 30 Hz, without catch-up bursts.
    send() performs no status queries, camera reads, sleeping or arrival gates.
    Ordinary chunks do not restart the Plan or mark final.
    """
    def __init__(self, command, robot_state, host, command_port, status_port, timeout_s):
        self.command, self.robot_state = command, robot_state
        self.host, self.command_port, self.status_port = host, command_port, status_port
        self.timeout_s = timeout_s
        self.plan_ws = self.udp = self.thread = None
        self.plan_error = self.stream_id = self.last_status = None
        self.index, self.ended = 0, False
        self.started_s = None
        self.start_attempted = False
        self._starting_after_id = None

    def status(self):
        result = validate_status_header(self.command('tee_status', raw=True))
        # The shared executor can report idle joint/test history before the
        # first EE task. It grants no target permission and is never cached as
        # an accepting EE task. All active/owned tasks retain strict checks.
        if (self.start_attempted and self.stream_id is None and self._starting_after_id is not None
                and result['stream_id'] > self._starting_after_id
                and result.get('implementation') == 'joint_tracking.TimedEe'
                and result.get('control_space') == 'ee'):
            self.stream_id = result['stream_id']
        if not result['active'] and self.stream_id is None:
            self.last_status = None
            return result
        result = validate_status(result)
        self.last_status = result
        return result

    def start(self, options=None):
        from websockets.sync.client import connect
        self.start_attempted = False
        cfg = timed_config(options)
        if not cfg['network_ready']:
            raise RuntimeError('timed_ee UDP network route must be configured; set network_ready=True')
        if cfg['plan_ws_port'] in (self.command_port, self.status_port):
            raise ValueError('timed_ee requires a dedicated Plan WebSocket port')
        status = self.status()
        if status['active']:
            raise PermissionError('An unmanaged timed stream is active')
        info = self.robot_state()['ret_context']['robot_msg']
        if tuple(info.get(k) for k in ('op_mode', 'activate', 'status', 'motion')) != (
                'Manual', 'Enabled', 'Normal', 'Stopped'):
            raise RuntimeError('timed_ee start requires Manual/Enabled/Normal/Stopped')
        previous_id = status['stream_id']
        self.close()
        self.plan_ws = connect(f'ws://{self.host}:{cfg["plan_ws_port"]}', proxy=None,
                               ping_interval=None, open_timeout=self.timeout_s,
                               close_timeout=self.timeout_s)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(self.timeout_s)
        self.udp.connect((cfg['udp_host'] or self.host, cfg['udp_port']))
        self.index, self.ended, self.plan_error = 0, False, None
        self._starting_after_id = previous_id
        self.started_s = time.monotonic()
        self.start_attempted = True
        # Match upstream Client.start_plan's seq=1, zero reserved fields and
        # NUL-terminated command; do not change the legacy follower/AJ codec.
        body = b'timed_ee\0'
        self.plan_ws.send(struct.pack('<IIQqqq', len(body), 1, 0, 0, 0, 0) + body)
        self.thread = threading.Thread(target=self._read_plan, args=(self.plan_ws,),
                                       name='a10-timed-ee-plan-reply', daemon=True)
        self.thread.start()
        deadline = self.started_s + self.timeout_s
        while time.monotonic() < deadline:
            self._check_plan()
            status = self.status()
            if status['stream_id'] > previous_id:
                self.stream_id = status['stream_id']
                if status['reason'] or status['state'] in (4, 5, 6):
                    raise RuntimeError(f'timed_ee start aborted: {status}')
                if status['active'] and status['ready'] and status['accepting']:
                    validated_poses(status.get('command_pose_m_quat'))
                    return require_stream(status, self.stream_id, accepting=True)
            time.sleep(.01)
        raise TimeoutError('timed_ee did not become ready within the command timeout')

    def _read_plan(self, ws):
        from ..operators.a10_operator import _unpack, _json_object
        try:
            reply = _json_object(_unpack(ws.recv()).rstrip(b'\0'))
            if reply.get('ret_code') != 0:
                self.plan_error = RuntimeError(f'timed_ee Plan failed: {reply}')
        except Exception as exc:
            self.plan_error = exc

    def _check_plan(self):
        if self.plan_error is not None:
            raise RuntimeError('timed_ee Plan transport failed') from self.plan_error

    def send(self, pose_m_quat, final=False):
        self._check_plan()
        if self.last_status is None:
            raise RuntimeError('Start and confirm timed_ee before sending')
        require_stream(self.last_status, self.stream_id, accepting=True)
        if self.ended or self.udp is None:
            raise RuntimeError('timed_ee final already sent or transport closed')
        index = self.index
        body = encode_target(self.stream_id, index, pose_m_quat, final)
        started = time.monotonic()
        sent = self.udp.send(body)
        finished = time.monotonic()
        if sent != len(body):
            raise ConnectionError('Incomplete timed EE UDP send; target will not be retried')
        self.index += 1
        self.ended = final
        return dict(stream_id=self.stream_id, index=index, final=final, bytes_sent=sent,
                    udp_send_started_s=started, udp_send_finished_s=finished)

    def finish(self, timeout_s=None):
        """Read-only wait for final command convergence; does not stop the Plan."""
        if self.stream_id is None or not self.ended:
            raise RuntimeError('Mark the final EE point before waiting for completion')
        # Capture immutable identity: a concurrent stop/start cannot make this
        # waiter accept a later task's endpoint or release its ownership.
        stream_id, expected_points = self.stream_id, self.index
        timeout = self.timeout_s if timeout_s is None else timeout_s
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('TimedEe completion timeout must be finite and positive')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_plan()
            status = self.status()
            if (status['stream_id'] != stream_id or not status['active'] or status['reason']
                    or status['stop_requested'] or status['state'] in (4, 5, 6)):
                raise RuntimeError(f'timed_ee stream interrupted: {status}')
            if endpoint_reached(status, expected_points):
                return status
            time.sleep(.01)
        raise TimeoutError('TimedEe command endpoint did not settle within completion timeout')

    wait_finished = finish

    def stop(self):
        stream_id = self.stream_id
        # Releasing an identified own task needs its identity and stopped
        # state, not agreement with a possibly incompatible endpoint profile.
        # This never permits sending to an incompatible profile.
        def stop_status():
            result = validate_status_header(self.command('tee_status', raw=True))
            if result['stream_id'] != stream_id:
                raise RuntimeError('timed_ee stream changed; no foreign stream stopped')
            if result['active'] and (result.get('implementation') != 'joint_tracking.TimedEe'
                                     or result.get('control_space') != 'ee'):
                raise RuntimeError('Active task is not the owned EE task; no foreign task stopped')
            return result
        status = stop_status()
        if stream_id is None or status['stream_id'] != stream_id:
            raise RuntimeError('timed_ee stream changed or start uncertain; no foreign stream stopped')
        if not status['active']:
            info = self.robot_state()['ret_context']['robot_msg']
            if info.get('motion') == 'Stopped':
                self.close()
                return status
        reply = self.command('tee_stop', raw=True)
        if reply.get('ret_code') != 0:
            raise RuntimeError(f'tee_stop rejected: {reply}')
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            status = stop_status()
            if status['stream_id'] != stream_id:
                raise RuntimeError('timed EE stream changed during stop')
            info = self.robot_state()['ret_context']['robot_msg']
            if not status['active'] and info.get('motion') == 'Stopped':
                self.close()
                return status
            time.sleep(.02)
        raise TimeoutError('tee_stop ACK received but inactive/Stopped not confirmed')

    def close(self):
        """Close transports only; caller must confirm stop before lease release."""
        if self.plan_ws is not None:
            self.plan_ws.close()
            self.plan_ws = None
        if self.thread is not None:
            self.thread.join(timeout=self.timeout_s)
            self.thread = None
        if self.udp is not None:
            self.udp.close()
            self.udp = None
        self.stream_id = None
        self.index, self.ended = 0, False
        self.last_status = self.plan_error = None
        self._starting_after_id = None
