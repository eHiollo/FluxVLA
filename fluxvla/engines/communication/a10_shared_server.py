"""Shared robot transport, serialized commands and exclusive target ownership."""
import base64
import copy
import errno
import json
import logging
import math
import socket
import socketserver
import threading
import time

from websockets.exceptions import ConnectionClosed

from ..operators.a10_operator import _FollowerConnection, _parse_state, _set_controller_speed
from .a10_shared_connection import ControlTransferredError

logger = logging.getLogger(__name__)


def _copy_feedback(value):
    """Copy decoded JSON containers without deepcopy's per-number bookkeeping.

    Controller feedback consists of dicts/lists and immutable JSON scalars.
    Keep a fallback for non-JSON values supplied by in-process callers so the
    cache continues to own an independent copy in that case too.
    """
    kind = type(value)
    if kind is dict:
        return {key: _copy_feedback(item) for key, item in value.items()}
    if kind is list:
        return [_copy_feedback(item) for item in value]
    if kind in (str, int, float, bool, type(None)):
        return value
    return copy.deepcopy(value)


def _controller_state(reply):
    """Parse global flags only; clients validate their own joint/TCP layout.

    A relay must not apply the single-arm six-joint parser to dual-arm rows.
    Fault, activation, mode, speed and coordinate fields retain full validation.
    """
    context = reply.get('ret_context', {})
    motion = context.get('motion_msg', {})
    global_motion = {k: v for k, v in motion.items() if k not in ('actual_pos', 'motor_pos', 'pq')}
    return _parse_state(dict(reply, ret_context=dict(context, motion_msg=global_motion)),
                        joint_unit='deg', position_unit=None, gripper_feedback_path=None)


class SharedControl:
    def __init__(self, host, ws_port=5999, status_ws_port=5888,
                 udp_port=9998, joint_udp_port=9999, timeout_s=5.):
        self.host, self.ws_port = host, ws_port
        self.status_ws_port = status_ws_port
        self.udp_port, self.joint_udp_port = udp_port, joint_udp_port
        self.timeout_s = timeout_s
        self.control = _FollowerConnection(host, ws_port, udp_port, timeout_s, joint_udp_port)
        self.status = (self.control if status_ws_port == ws_port else
                       _FollowerConnection(host, status_ws_port, udp_port, timeout_s, joint_udp_port))
        self._control_lock = threading.RLock()
        # UDP/ownership changes are atomic without waiting for read-only GET.
        self._target_lock = threading.RLock()
        self._status_lock = self._control_lock if self.status is self.control else threading.RLock()
        self.owner = None
        self.revoked = set()
        self._feedback_independent = set()
        self.preempted = set()
        self.recording_owner = None
        self.console_operation = None
        self.console_operation_depth = 0
        self.stopped = set()
        self.stop_requests = set()
        self._speed_restore_allowed = set()
        self.recorder = None
        self._snapshot_lock = threading.Lock()
        self._snapshot = None
        self._snapshot_sequence = 0
        self._poll_lock = threading.Lock()
        self._poll_rates = {}
        self._poll_stop = threading.Event()
        self._poll_wakeup = threading.Event()
        self._poll_thread = None
        self._poll_error = None
        self._poll_count = 0
        self._poll_started = None
        self._poll_last_duration_s = None

    def _remember_state(self, reply):
        # This is the actual robot receive time. Reading the cache must never
        # refresh it, including while a native stop holds the transaction lock.
        stamp = time.monotonic()
        cached_reply = _copy_feedback(reply)
        with self._snapshot_lock:
            self._snapshot_sequence += 1
            self._snapshot = dict(reply=cached_reply, received_monotonic_s=stamp,
                                  sequence=self._snapshot_sequence)
        return reply

    def feedback_snapshot(self):
        with self._snapshot_lock:
            # Published snapshots are never mutated in place. Copy outside the
            # mutex so a reader doesn't delay publication of newer feedback.
            published = self._snapshot
        snapshot = _copy_feedback(published)
        with self._poll_lock:
            error = self._poll_error
            if snapshot is not None:
                snapshot['polling'] = self._poll_metrics_locked()
        if error is not None:
            raise error  # Known fault is not hidden by a still-young cached pose.
        return snapshot

    def _poll_metrics_locked(self):
        elapsed = 0. if self._poll_started is None else time.monotonic() - self._poll_started
        return dict(requested_hz=max(self._poll_rates.values(), default=0.),
                    count=self._poll_count, elapsed_s=elapsed,
                    achieved_hz=self._poll_count / elapsed if elapsed > 0 else 0.,
                    last_query_s=self._poll_last_duration_s,
                    error=None if self._poll_error is None else str(self._poll_error))

    def configure_feedback_poll(self, session_id, hz):
        if hz is not None and (isinstance(hz, bool) or not isinstance(hz, (int, float))
                               or not math.isfinite(hz) or not 0 < hz <= 100):
            raise ValueError('feedback poll hz must be within (0, 100] or None')
        with self._poll_lock:
            if hz is None:
                self._poll_rates.pop(session_id, None)
            else:
                if self._poll_stop.is_set():
                    raise RuntimeError('Shared feedback service is closing')
                # An explicit subscription restart may follow a fresh successful
                # GET. Keep ownership revoked, but let read-only polling run again.
                self._poll_error = None
                if not self._poll_rates:
                    self._poll_count = 0
                    self._poll_started = time.monotonic()
                self._poll_rates[session_id] = float(hz)
                if self._poll_thread is None:
                    self._poll_thread = threading.Thread(target=self._poll_feedback,
                        name='a10-shared-feedback', daemon=True)
                    self._poll_thread.start()
            result = self._poll_metrics_locked()
        self._poll_wakeup.set()
        return result

    def _poll_feedback(self):
        while not self._poll_stop.is_set():
            with self._poll_lock:
                hz = max(self._poll_rates.values(), default=0.)
                failed = self._poll_error is not None
            if not hz or failed:
                self._poll_wakeup.wait(.1)
                self._poll_wakeup.clear()
                continue
            started = time.monotonic()
            # Never hold target mutex during GET. Command transactions still
            # serialize with GET on the same real WebSocket.
            if not self._status_lock.acquire(timeout=.02):
                continue
            failure = None
            try:
                reply = self.read_state()
                state = _controller_state(reply)
                if state.has_error:
                    from ..operators.a10_operator import A10ControllerError
                    raise A10ControllerError('Controller feedback reports a fault')
                with self._poll_lock:
                    self._poll_count += 1
                    self._poll_last_duration_s = time.monotonic() - started
            except Exception as exc:
                failure = exc
                with self._poll_lock:
                    self._poll_error = exc
            finally:
                self._status_lock.release()
            if failure is not None:
                # Commands may hold target_lock while querying a separate status
                # socket. Release status_lock before invalidating ownership.
                with self._target_lock:
                    if self.owner is not None and self.owner not in self._feedback_independent:
                        self.revoked.add(self.owner)
                self.note('feedback_poll_failed', error=str(failure))
            # Rate means query start period, not GET latency plus a fixed sleep.
            # Overruns discard missed ticks; no burst/catch-up requests.
            self._poll_wakeup.wait(max(0., 1. / hz - (time.monotonic() - started)))
            self._poll_wakeup.clear()

    def stop_feedback_polling(self):
        self._poll_stop.set()
        self._poll_wakeup.set()
        thread = self._poll_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(self.timeout_s + .5)
            if thread.is_alive():
                raise RuntimeError('Shared feedback poller did not stop')

    def connect(self):
        try:
            self.control.connect()
            if self.status is not self.control:
                self.status.connect()
            self.read_state()
        except BaseException:
            self.control.close()
            self.status.close()
            raise

    def note(self, event, **data):
        logger.info('A10 shared control: %s %s', event, data)
        if self.recorder is not None:
            self.recorder.record(event, **data)

    def _probe_control(self):
        """Validate the real socket, not just the local RPC listener.

        Only GET is repeated automatically, and only if no motion owner exists.
        Once a command is attempted its result stays uncertain; never replay it.
        Reopening a closed transport for GET revokes any old target owner;
        motion still requires an explicit stop and a new takeover.
        Call with _control_lock held.
        """
        if self.control.ws is None:
            if self.owner is not None:
                self.revoked.add(self.owner)
                self.note('target_stream_revoked', session_id=self.owner, reason='transport_reconnect')
            self.control.connect()
        try:
            return self._remember_state(self.control.request('get', status=True))
        except (OSError, ConnectionClosed) as exc:
            if self.owner is not None:
                self.revoked.add(self.owner)
                raise
            self.note('idle_transport_reconnect', reason=str(exc))
            self.control.close()  # Isolate old or delayed replies before the GET retry.
            self.control.connect()
            return self._remember_state(self.control.request('get', status=True))

    def read_state(self):
        with self._status_lock:
            if self.status is self.control:
                return self._probe_control()
            if self.status.ws is None:
                self.status.connect()  # Read-only recovery; never repeats control commands.
            return self._remember_state(self.status.request('get', status=True))

    def mode_command(self, command, active, *, stop_motion=False):
        if self.control.ws is None:
            self.control.connect()
        if active and command.split(maxsplit=1)[0] in ('follower_aj', 'follower_aj_tg'):
            self.control.request('start_follower')
        self.control.send_command(command)
        if stop_motion:
            self.control.send_command('manual_stop')
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            reply = self.control.request('get', status=True,
                                         timeout_s=max(.001, deadline - time.monotonic()))
            self._remember_state(reply)
            state = _controller_state(reply)
            mode = 'Axis' if command.split(maxsplit=1)[0] in ('follower_aj', 'follower_aj_tg') else 'Cart'
            if (state.follower_active is active and (not active or state.follower_mode == mode)
                    and (active or state.raw['ret_context'].get('robot_msg', {}).get('motion') == 'Stopped')
                    and (not stop_motion or (
                        not state.has_error and
                        state.raw['ret_context'].get('robot_msg', {}).get('motion') == 'Stopped'))):
                return reply
            time.sleep(.01)
        raise TimeoutError('Shared service could not confirm follower mode change')

    def stop_owner(self, reason):
        owner = self.owner
        if owner is None:
            return
        self.revoked.add(owner)
        if reason.startswith('console'):
            self.preempted.add(owner)
        state = _controller_state(self.read_state())
        if (state.follower_active is not False
                or state.raw['ret_context'].get('robot_msg', {}).get('motion') != 'Stopped'):
            self.mode_command('stop_follower', False)
        self.owner = None
        self.stopped.add(owner)
        if reason == 'explicit_stop':
            self._speed_restore_allowed.add(owner)
        else:
            self._speed_restore_allowed.discard(owner)
        self.note('owner_stopped', session_id=owner, reason=reason, confirmed=True)

    def claim(self, session_id, role, *, require_feedback=True):
        if type(require_feedback) is not bool:
            raise ValueError('require_feedback must be bool')
        if self.console_operation not in (None, session_id):
            error = ControlTransferredError if role == 'teleop' else PermissionError
            raise error('Console command is still running')
        if self.owner == session_id:
            if session_id in self.revoked:
                raise RuntimeError('Previous stop is not confirmed; stop the follower before reacquiring')
            return
        if self.owner is not None and self.owner != session_id:
            if role != 'console':
                error = ControlTransferredError if role == 'teleop' else PermissionError
                raise error('Another session owns the follower; stop it before starting inference')
            self.stop_owner('console_takeover')
        if self.owner is None:
            state = _controller_state(self.read_state())
            if state.follower_active is not False:
                raise PermissionError('An unmanaged follower is active; the shared service will not take it over')
        self.revoked.discard(session_id)
        self.preempted.discard(session_id)
        self.stopped.discard(session_id)
        self.stop_requests.discard(session_id)
        self._speed_restore_allowed.clear()
        self.owner = session_id
        if require_feedback:
            self._feedback_independent.discard(session_id)
        else:
            self._feedback_independent.add(session_id)
        self.note('owner_acquired', session_id=session_id, role=role)

    def request_stop(self, session_id):
        """No controller result/state reads. Revoke targets before serializing WS send."""
        with self._target_lock:
            if session_id in self.stop_requests and self.owner != session_id:
                return dict(requested=True, confirmed=False)
            if self.owner != session_id:
                raise PermissionError('This session does not own the follower')
            self.revoked.add(session_id)
        with self._control_lock, self._target_lock:
            if self.owner != session_id:
                raise ControlTransferredError('Control changed; no stop sent to the new owner')
            # Do not reconnect or replay on a failed send.
            self.control.send_command('stop_follower')
            self.owner = None
            self.stop_requests.add(session_id)
            self._speed_restore_allowed.discard(session_id)
            self.note('owner_stop_requested', session_id=session_id, confirmed=False)
        return dict(requested=True, confirmed=False)

    def lease_status(self, session_id):
        return dict(owns=self.owner == session_id, stopped_by_service=session_id in self.stopped,
                    preempted=session_id in self.preempted,
                    available=self.owner is None and self.console_operation is None)

    def handle(self, request):
        session_id, role = request['session_id'], request['role']
        if role not in ('console', 'inference', 'teleop'):
            raise ValueError('Shared client role must be console, inference or teleop')
        op = request['op']
        if op == 'stop_request':
            return self.request_stop(session_id)
        if op == 'feedback_poll':
            return self.configure_feedback_poll(session_id, request.get('hz'))
        if op == 'feedback_snapshot':
            # Cache-only read: no robot request, target ownership or control lock.
            return self.feedback_snapshot()
        if op == 'hello':
            expected = dict(robot_host=self.host, ws_port=(self.status_ws_port if request.get('status_channel') else self.ws_port),
                            udp_port=self.udp_port, joint_udp_port=self.joint_udp_port)
            if any(request.get(k) != value for k, value in expected.items()):
                raise ValueError(f'Shared service robot endpoints differ: {expected}')
            # Managed services can outlive controller restarts or idle sockets.
            # VR must validate feedback before record_start. Console hello
            # stays local: a reconnect for explicit stop must be able to
            # interrupt a long command before waiting for the control lock.
            if role != 'console' and not request.get('status_channel'):
                with self._control_lock:
                    self._probe_control()
            return expected
        if op == 'request' and request['command'].strip() == 'get':
            return self.read_state()
        if op == 'request' and request['command'].strip() == 'read_motor --all':
            # Explicit read-only configuration query. Never claim/revoke a
            # target lease or invalidate an active follower's permissions.
            with self._control_lock:
                return self.control.request('read_motor --all', raw=True,
                                            timeout_s=request.get('timeout_s'))
        if op == 'lease' and request['operation'] == 'status':
            return self.lease_status(session_id)
        if op == 'stop':
            if role != 'console':
                raise PermissionError('Only the console may issue a global stop')
            owner = self.owner
            if owner is not None:
                self.revoked.add(owner)
                if owner != session_id:
                    self.preempted.add(owner)
            # A native command may be holding the control lock while awaiting
            # a long reply. Isolate that socket before obtaining the lock.
            self.control.interrupt()
            with self._control_lock, self._target_lock:
                self._speed_restore_allowed.clear()
                reply = self.mode_command('stop_follower', False, stop_motion=True)
                if owner is not None:
                    self.stopped.add(owner)
                self.owner = None
                self.note('console_stop', session_id=session_id, confirmed=True)
                return reply
        if op == 'target':
            while not self._target_lock.acquire(timeout=.02):
                if self.owner != session_id or session_id in self.revoked:
                    raise ControlTransferredError('Control was transferred to the console; no target sent')
            try:
                with self._poll_lock:
                    feedback_error = self._poll_error
                if feedback_error is not None and session_id not in self._feedback_independent:
                    raise feedback_error
                if self.owner != session_id or session_id in self.revoked:
                    raise ControlTransferredError('Control was transferred to the console; no target sent')
                packet = base64.b64decode(request['packet'], validate=True)
                if len(packet) > 65507:
                    raise ValueError('Datagram is too large')
                mode = request['mode']
                if mode not in ('Cart', 'Axis'):
                    raise ValueError('Invalid target mode')
                udp = self.control.joint_udp if mode == 'Axis' else self.control.udp
                if udp is None:
                    raise ConnectionError('Robot control connection unavailable; reconnect explicitly')
                started = time.monotonic()
                sent = udp.send(packet)
                finished = time.monotonic()
                if request.get('include_send_timing', False):
                    # Host socket submission, not controller reception/execution.
                    return dict(bytes_sent=sent, udp_send_started_s=started,
                                udp_send_finished_s=finished)
                return sent
            finally:
                self._target_lock.release()
        with self._control_lock, self._target_lock:
            if op == 'motor_acceleration':
                if request.get('status_channel'):
                    raise PermissionError('Status-only connections cannot configure motor acceleration')
                from ..operators.kaanh_commands.motor_acceleration import ensure_motor_acceleration, acceleration_matches
                from ..operators.kaanh_commands.set_acc import positive_acceleration
                service = self
                class MotorAccess:
                    allow_motion = True
                    shared_control = None
                    command_timeout_s = service.timeout_s

                    def get_state(self):
                        return _controller_state(service.control.request('get', status=True))

                    def read_motor_config(self):
                        reply = service.control.request('read_motor --all', raw=True)
                        if reply.get('ret_code') != 0:
                            raise RuntimeError(f'Motor configuration read failed: {reply}')
                        data = reply.get('ret_context')
                        return json.loads(data) if isinstance(data, str) else data

                    def begin_shared_console_operation(self):
                        if service.owner is not None or service.console_operation not in (None, session_id):
                            raise PermissionError('Motor configuration requires no motion owner or other console operation')

                    def end_shared_console_operation(self):
                        pass

                    def send_controller_command(self, command):
                        if not command.startswith('write_motor --id='):
                            raise PermissionError('Only prepared motor acceleration writes are permitted')
                        return service.control.request(command, raw=True)

                started = time.monotonic()
                acceleration = positive_acceleration(request['acceleration'])
                if acceleration_matches(MotorAccess().read_motor_config(), acceleration):
                    result = dict(changed=False, max_acc_deg_s2=acceleration, min_acc_deg_s2=-acceleration)
                else:
                    if self.owner is not None or self.console_operation not in (None, session_id):
                        raise PermissionError('Motor acceleration write requires no follower owner or other console operation')
                    result = ensure_motor_acceleration(MotorAccess(), acceleration,
                        output=lambda text: self.note('motor_acceleration_detail', message=text))
                result['preflight_elapsed_s'] = time.monotonic() - started
                self.note('motor_acceleration_preflight', session_id=session_id, role=role, **result)
                return result
            if op == 'speed_override':
                if self.owner is not None or (session_id in self.revoked
                        and session_id not in self._speed_restore_allowed):
                    raise PermissionError('Speed change requires no follower owner and a valid session')
                if self.control.ws is None:
                    raise ConnectionError('Robot control connection unavailable; reconnect explicitly')
                def get_state(*, timeout_s):
                    reply = self.control.request('get', status=True, timeout_s=timeout_s)
                    return _controller_state(reply)
                result = _set_controller_speed(self.control, get_state,
                    request['percent'], request['expected_mode'], self.timeout_s,
                    request.get('expected_percent'))
                self.note('controller_speed_override', session_id=session_id, confirmed=result)
                return result
            if op == 'lease':
                if request['operation'] in ('begin_console', 'begin_idle_console'):
                    if role != 'console' or self.console_operation not in (None, session_id):
                        raise PermissionError('Another console command is still running')
                    if request['operation'] == 'begin_idle_console' and self.owner is not None:
                        raise PermissionError('电机加速度切换要求共享 follower 无控制者')
                    if self.owner != session_id:
                        self.stop_owner('console_command_begin')
                    self.console_operation = session_id
                    self.console_operation_depth += 1
                    if request['operation'] == 'begin_idle_console':
                        return dict(self.lease_status(session_id), idle_console_reserved=True)
                elif request['operation'] == 'end_console':
                    if self.console_operation == session_id:
                        self.console_operation_depth -= 1
                        if self.console_operation_depth == 0:
                            self.console_operation = None
                elif request['operation'] == 'acquire':
                    self.claim(session_id, role, require_feedback=request.get('require_feedback', True))
                elif request['operation'] == 'acknowledge':
                    if (self.console_operation is not None or (self.owner is not None
                            and self.owner != session_id and self.owner not in self.revoked)):
                        raise PermissionError('Another session still owns control')
                    state = _controller_state(self.read_state())
                    if (state.follower_active is not False or state.has_error
                            or state.raw['ret_context'].get('robot_msg', {}).get('motion') != 'Stopped'):
                        raise PermissionError('Follower stop is not confirmed')
                    if self.owner is not None:
                        # A failed transport can leave a revoked, now-inactive
                        # lease behind. Release metadata only; no stop is replayed.
                        self.stopped.add(self.owner)
                        self.owner = None
                    self.preempted.discard(session_id)
                    self.revoked.discard(session_id)
                elif request['operation'] == 'prepare_command':
                    if role != 'console':
                        raise PermissionError('Only the console may preempt for a control command')
                    if self.console_operation not in (None, session_id):
                        raise PermissionError('Another console command is still running')
                    self.stop_owner('console_command_preflight')
                    # A newly requested console command may set its temporary speed
                    # after the old follower was stopped; target ownership stays empty.
                    self.revoked.discard(session_id)
                return self.lease_status(session_id)
            if op not in ('command', 'request'):
                raise ValueError(f'Unknown shared operation: {op}')
            command = request['command']
            name = command.split(maxsplit=1)[0]
            if name in ('follower_aj', 'follower_aj_tg', 'follower_cart'):
                self.claim(session_id, role)
                return self.mode_command(command, True)
            if name == 'stop_follower':
                if role != 'console' and session_id in self.stopped and self.owner != session_id:
                    return self.read_state()  # Old cleanup must not stop the new owner's follower.
                if role != 'console' and self.owner != session_id:
                    raise PermissionError('This session does not own the follower')
                self.stop_owner('explicit_stop')
                return self.read_state()
            if name in ('record_start', 'record_status', 'record_heartbeat'):
                if name == 'record_start':
                    if self.recording_owner not in (None, session_id):
                        raise PermissionError('Another session owns controller recording')
                elif name == 'record_heartbeat' and self.recording_owner != session_id:
                    raise PermissionError('This session does not own controller recording')
                reply = self.control.request(command)
                if name == 'record_start':
                    self.recording_owner = session_id
                return reply
            if role == 'teleop' and command == 'set_jog_coordinate --tool':
                if (self.owner not in (None, session_id) or session_id in self.preempted
                        or self.console_operation not in (None, session_id)):
                    raise ControlTransferredError('Console owns control; release VR grips before resuming')
                return self.control.request(command)
            if role != 'console':
                raise PermissionError('Arbitrary controller commands require the console role')
            if self.console_operation not in (None, session_id):
                raise PermissionError('Another console command is still running')
            self._speed_restore_allowed.clear()
            if self.owner is not None:
                self.stop_owner('console_command:' + name)
            if self.control.ws is None:
                self.control.connect()
            self.note('controller_command', session_id=session_id, command=command)
            return self.control.request(command, raw=request.get('raw', False),
                                        status=request.get('status', False),
                                        timeout_s=request.get('timeout_s', self.timeout_s))

    def disconnected(self, session_id):
        self.configure_feedback_poll(session_id, None)
        with self._control_lock, self._target_lock:
            self._feedback_independent.discard(session_id)
            if self.console_operation == session_id:
                self.console_operation = None
                self.console_operation_depth = 0
            if self.recording_owner == session_id:
                self.recording_owner = None
            if self.owner == session_id:
                try:
                    self.stop_owner('client_disconnected')
                except Exception as exc:
                    self.note('owner_stop_unconfirmed', session_id=session_id, error=repr(exc))

    def close(self):
        self.stop_feedback_polling()
        with self._control_lock, self._target_lock:
            try:
                self.stop_owner('service_shutdown')
            finally:
                self.control.close()
                self.status.close()
                if self.recorder is not None:
                    self.recorder.close()


class _Handler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup()
        with self.server.clients_lock:
            self.server.clients += 1

    def finish(self):
        try:
            super().finish()
        finally:
            with self.server.clients_lock:
                self.server.clients -= 1
                self.server.last_client_time = time.monotonic()

    def handle(self):
        session_id = None
        status_channel = True
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:
                line = self.rfile.readline(1 << 20)
                if not line:
                    return
                try:
                    request = json.loads(line)
                    if session_id is None:
                        if request.get('op') != 'hello':
                            raise ValueError('Shared connection must start with hello')
                        session_id = request['session_id']
                        status_channel = bool(request.get('status_channel'))
                        role = request['role']
                    elif request.get('session_id') != session_id or request.get('role') != role:
                        raise ValueError('Shared session identity cannot change')
                    if status_channel and request.get('op') not in ('hello', 'feedback_snapshot') and not (request.get('op') == 'request' and request.get('command', '').strip() == 'get'):
                        raise PermissionError('Status channel only accepts get/feedback_snapshot queries')
                    result = self.server.robot.handle(request)
                    response = dict(result=result)
                except Exception as exc:
                    self.server.robot.note('request_error', session_id=session_id,
                                           error_type=type(exc).__name__, error=str(exc))
                    response = dict(error=str(exc), error_type=type(exc).__name__)
                self.wfile.write(json.dumps(response, ensure_ascii=False).encode() + b'\n')
                self.wfile.flush()
        finally:
            if session_id is not None and not status_channel:
                self.server.robot.disconnected(session_id)


class SharedControlServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, robot, *, idle_timeout_s=None):
        if address[0] not in ('127.0.0.1', 'localhost'):
            raise ValueError('Shared control service must bind to loopback')
        self.robot = robot
        self.clients_lock = threading.Lock()
        self.clients = 0
        self.last_client_time = time.monotonic()
        self.idle_timeout_s = idle_timeout_s
        self._idle_shutdown = False
        super().__init__(address, _Handler)

    def service_actions(self):
        with self.clients_lock:
            expired = (self.idle_timeout_s is not None and self.clients == 0
                       and time.monotonic() - self.last_client_time >= self.idle_timeout_s)
        if expired and not self._idle_shutdown:
            self._idle_shutdown = True
            threading.Thread(target=self.shutdown, daemon=True).start()


class ConsoleSharedControl:
    """Start a local service on demand, or reuse one already listening.

    Only the console which starts the service owns its lifetime. Keeping that
    console open keeps the shared robot connections available for inference.
    """

    def __init__(self, address, *, output=print, **robot_options):
        self.address, self.robot_options = address, robot_options
        self.output = output
        self.server = self.thread = None

    def start(self):
        if self.server is not None:
            return
        robot = SharedControl(**self.robot_options)
        try:
            server = SharedControlServer(self.address, robot)
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE:
                # The client's hello validates endpoints and the protocol.
                # Do not open a second pair of robot WebSockets.
                return
            raise
        try:
            robot.connect()
            from fluxvla.engines.utils.a10_debug_log import create_a10_debug_log
            robot.recorder = create_a10_debug_log(
                dict(enabled=True), role='shared_control', metadata=self.robot_options)
            thread = threading.Thread(target=server.serve_forever,
                                      kwargs=dict(poll_interval=.1),
                                      name='a10-shared-control', daemon=True)
            thread.start()
        except BaseException:
            server.server_close()
            robot.close()
            raise
        self.server, self.thread = server, thread
        self.output(f'共享连接已自动启动：{self.address[0]}:{self.address[1]}；'
                    '可直接启动推理，请保持此控制台打开。')

    def close(self):
        server, self.server = self.server, None
        if server is not None:
            server.shutdown()
            self.thread.join(timeout=2.)
            server.server_close()
            server.robot.close()
