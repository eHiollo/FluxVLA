"""One robot connection pair, serialized commands and exclusive target ownership."""
import base64
import errno
import json
import logging
import socket
import socketserver
import threading
import time

from ..operators.a10_operator import _FollowerConnection, _parse_state

logger = logging.getLogger(__name__)


class SharedControl:
    def __init__(self, host, ws_port=5999, status_ws_port=5888,
                 udp_port=9998, joint_udp_port=9999, timeout_s=5.):
        self.host, self.ws_port = host, ws_port
        self.status_ws_port = status_ws_port
        self.udp_port, self.joint_udp_port = udp_port, joint_udp_port
        self.timeout_s = timeout_s
        self.control = _FollowerConnection(host, ws_port, udp_port, timeout_s, joint_udp_port)
        self.status = _FollowerConnection(host, status_ws_port, udp_port, timeout_s, joint_udp_port)
        self._control_lock, self._status_lock = threading.RLock(), threading.RLock()
        self.owner = None
        self.revoked = set()
        self.stopped = set()
        self.recorder = None

    def connect(self):
        try:
            self.control.connect()
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

    def read_state(self):
        with self._status_lock:
            if self.status.ws is None:
                self.status.connect()  # Read-only recovery; never repeats control commands.
            return self.status.request('get', status=True)

    def mode_command(self, command, active):
        if self.control.ws is None:
            self.control.connect()
        self.control.send_command(command)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            reply = self.control.request('get', status=True,
                                         timeout_s=max(.001, deadline - time.monotonic()))
            state = _parse_state(reply, joint_unit='deg', position_unit=None, gripper_feedback_path=None)
            mode = 'Axis' if command.split(maxsplit=1)[0] == 'follower_aj' else 'Cart'
            if state.follower_active is active and (not active or state.follower_mode == mode):
                return reply
            time.sleep(.01)
        raise TimeoutError('Shared service could not confirm follower mode change')

    def stop_owner(self, reason):
        owner = self.owner
        if owner is None:
            return
        self.revoked.add(owner)
        state = _parse_state(self.read_state(), joint_unit='deg', position_unit=None, gripper_feedback_path=None)
        if state.follower_active is not False:
            self.mode_command('stop_follower', False)
        self.owner = None
        self.stopped.add(owner)
        self.note('owner_stopped', session_id=owner, reason=reason, confirmed=True)

    def claim(self, session_id, role):
        if self.owner == session_id:
            if session_id in self.revoked:
                raise RuntimeError('Previous stop is not confirmed; stop the follower before reacquiring')
            return
        if self.owner is not None and self.owner != session_id:
            if role != 'console':
                raise PermissionError('Another session owns the follower; stop it before starting inference')
            self.stop_owner('console_takeover')
        if self.owner is None:
            state = _parse_state(self.read_state(), joint_unit='deg', position_unit=None, gripper_feedback_path=None)
            if state.follower_active is not False:
                raise PermissionError('An unmanaged follower is active; the shared service will not take it over')
        self.revoked.discard(session_id)
        self.stopped.discard(session_id)
        self.owner = session_id
        self.note('owner_acquired', session_id=session_id, role=role)

    def handle(self, request):
        session_id, role = request['session_id'], request['role']
        if role not in ('console', 'inference'):
            raise ValueError('Shared client role must be console or inference')
        op = request['op']
        if op == 'hello':
            expected = dict(robot_host=self.host, ws_port=(self.status_ws_port if request.get('status_channel') else self.ws_port),
                            udp_port=self.udp_port, joint_udp_port=self.joint_udp_port)
            if any(request.get(k) != value for k, value in expected.items()):
                raise ValueError(f'Shared service robot endpoints differ: {expected}')
            return expected
        if op == 'request' and request['command'].strip() == 'get':
            return self.read_state()
        if op == 'lease' and request['operation'] == 'status':
            return dict(owns=self.owner == session_id, stopped_by_service=session_id in self.stopped)
        if op == 'target':
            while not self._control_lock.acquire(timeout=.02):
                if self.owner != session_id or session_id in self.revoked:
                    raise InterruptedError('Control was transferred to the console; no joint target sent')
            try:
                if self.owner != session_id or session_id in self.revoked:
                    raise InterruptedError('Control was transferred to the console; no joint target sent')
                packet = base64.b64decode(request['packet'], validate=True)
                if len(packet) > 65507:
                    raise ValueError('Datagram is too large')
                mode = request['mode']
                if mode not in ('Cart', 'Axis'):
                    raise ValueError('Invalid target mode')
                udp = self.control.joint_udp if mode == 'Axis' else self.control.udp
                if udp is None:
                    raise ConnectionError('Robot control connection unavailable; reconnect explicitly')
                return udp.send(packet)
            finally:
                self._control_lock.release()
        with self._control_lock:
            if op == 'lease':
                if request['operation'] == 'acquire':
                    self.claim(session_id, role)
                elif request['operation'] == 'prepare_command':
                    if role != 'console':
                        raise PermissionError('Only the console may preempt for a control command')
                    self.stop_owner('console_command_preflight')
                return dict(owns=self.owner == session_id, stopped_by_service=session_id in self.stopped)
            if op not in ('command', 'request'):
                raise ValueError(f'Unknown shared operation: {op}')
            command = request['command']
            name = command.split(maxsplit=1)[0]
            if name in ('follower_aj', 'follower_cart'):
                self.claim(session_id, role)
                return self.mode_command(command, True)
            if name == 'stop_follower':
                if role != 'console' and session_id in self.stopped and self.owner != session_id:
                    return self.read_state()  # Old cleanup must not stop the new owner's follower.
                if role != 'console' and self.owner != session_id:
                    raise PermissionError('This session does not own the follower')
                self.stop_owner('explicit_stop')
                return self.read_state()
            if role != 'console':
                raise PermissionError('Arbitrary controller commands require the console role')
            if self.owner is not None:
                self.stop_owner('console_command:' + name)
            if self.control.ws is None:
                self.control.connect()
            self.note('controller_command', session_id=session_id, command=command)
            return self.control.request(command, raw=request.get('raw', False),
                                        status=request.get('status', False),
                                        timeout_s=request.get('timeout_s', self.timeout_s))

    def disconnected(self, session_id):
        with self._control_lock:
            if self.owner == session_id:
                try:
                    self.stop_owner('client_disconnected')
                except Exception as exc:
                    self.note('owner_stop_unconfirmed', session_id=session_id, error=repr(exc))

    def close(self):
        with self._control_lock:
            try:
                self.stop_owner('service_shutdown')
            finally:
                self.control.close()
                self.status.close()
                if self.recorder is not None:
                    self.recorder.close()


class _Handler(socketserver.StreamRequestHandler):
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
                    if status_channel and request.get('op') != 'hello' and not (request.get('op') == 'request' and request.get('command', '').strip() == 'get'):
                        raise PermissionError('Status channel only accepts get queries')
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

    def __init__(self, address, robot):
        if address[0] not in ('127.0.0.1', 'localhost'):
            raise ValueError('Shared control service must bind to loopback')
        self.robot = robot
        super().__init__(address, _Handler)


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
