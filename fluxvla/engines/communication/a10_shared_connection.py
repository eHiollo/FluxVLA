"""Loopback RPC adapter for one shared KAANH control/status connection pair."""
import base64
import json
import socket
import subprocess
import sys
import time
from pathlib import Path
import threading
import uuid


class ControlTransferredError(InterruptedError):
    """The console revoked this client; old targets must not be replayed."""


class SharedServiceUnavailable(ConnectionRefusedError):
    """The local service refused the initial TCP connection, before any RPC."""


def _start_managed_service(connection):
    """Start the existing relay entrypoint, independently of either UI process.

    The child reserves the local port before connecting to the robot, so a
    concurrent launcher cannot open a second robot connection.
    """
    address = (connection.shared.get('host', '127.0.0.1'), connection.shared.get('port', 5870))
    if address[0] not in ('127.0.0.1', 'localhost'):
        raise ValueError('Managed shared control must use loopback')
    root = Path(__file__).resolve().parents[3]
    log = Path(connection.shared.get('log_path', root / 'work_dirs/a10/shared_control/service.log'))
    log.parent.mkdir(parents=True, exist_ok=True)
    args = [sys.executable, str(root / 'scripts/a10_shared_control.py'),
            '--host', connection.host, '--ws-port', str(connection.ws_port),
            '--status-ws-port', str(connection.shared.get('status_ws_port', connection.ws_port)),
            '--udp-port', str(connection.udp_port), '--joint-udp-port', str(connection.joint_udp_port),
            '--timeout', str(connection.timeout_s), '--port', str(address[1]),
            '--idle-timeout', str(connection.shared.get('idle_timeout_s', 5.))]
    with log.open('ab') as output:
        process = subprocess.Popen(args, cwd=root, stdin=subprocess.DEVNULL, stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    threading.Thread(target=process.wait, name='a10-relay-child', daemon=True).start()
    deadline = time.monotonic() + connection.shared.get('startup_timeout_s', 15.)
    while time.monotonic() < deadline:
        try:
            return socket.create_connection(address, min(.3, connection.timeout_s))
        except ConnectionRefusedError:
            if process.poll() is not None:
                # A competing launcher may have won the local bind. Recheck
                # once before reporting failure, never fall back to direct.
                try:
                    return socket.create_connection(address, .3)
                except ConnectionRefusedError:
                    raise ConnectionError(f'A10 shared service failed to start; see {log}') from None
            time.sleep(.05)
    raise TimeoutError(f'A10 shared service startup timed out; see {log}')


class _Datagram:
    def __init__(self, connection, mode):
        self.connection, self.mode = connection, mode
        self.last_send_timing = None

    def send(self, packet):
        self.last_send_timing = None
        result = self.connection.call('target', mode=self.mode, include_send_timing=True,
                                      packet=base64.b64encode(packet).decode('ascii'))
        # Older running relays return only the byte count. Do not mistake the
        # client RPC completion time for the UDP syscall time in that case.
        if isinstance(result, dict):
            self.last_send_timing = {key: result[key] for key in (
                'udp_send_started_s', 'udp_send_finished_s')}
            return result['bytes_sent']
        return result


class SharedFollowerConnection:
    def __init__(self, host, ws_port, udp_port, timeout_s, joint_udp_port=9999,
                 *, shared, session_id=None, status_channel=False):
        self.host, self.ws_port, self.udp_port = host, ws_port, udp_port
        self.timeout_s, self.joint_udp_port = timeout_s, joint_udp_port
        self.shared = dict(shared)
        self.session_id = session_id or uuid.uuid4().hex
        self.status_channel = status_channel
        self.ws = self.udp = self.joint_udp = None
        self._sock = self._reader = None
        self._lock = threading.RLock()
        self._request_pending = threading.Event()
        self.command_cancelled = None
        self.last_request_failure = None

    def connect(self):
        if not self.status_channel:
            # A late disconnect callback from the old socket must not revoke
            # this newly connected control session.
            self.session_id = uuid.uuid4().hex
        self.last_request_failure = None
        try:
            address = (self.shared.get('host', '127.0.0.1'), self.shared.get('port', 5870))
            try:
                self._sock = socket.create_connection(address, self.timeout_s)
            except ConnectionRefusedError as exc:
                if self.shared.get('mode') == 'managed':
                    self._sock = _start_managed_service(self)
                else:
                    raise SharedServiceUnavailable(
                        f'A10 shared service {address[0]}:{address[1]} is not listening; '
                        'start python scripts/a10_control.py shell first '
                        '(the console starts the shared service automatically)') from exc
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._reader = self._sock.makefile('rb')
            self.ws = self._sock
            self.call('hello', robot_host=self.host, ws_port=self.ws_port,
                      joint_udp_port=self.joint_udp_port, udp_port=self.udp_port,
                      timeout_s=max(self.timeout_s, self.shared.get('startup_timeout_s', 15.))
                      if self.shared.get('mode') == 'managed' else None)
            self.udp, self.joint_udp = _Datagram(self, 'Cart'), _Datagram(self, 'Axis')
        except BaseException:
            self.close()
            raise

    def call(self, op, *, timeout_s=None, allow_cancelled=False, **data):
        with self._lock:
            if (not allow_cancelled and self.command_cancelled is not None
                    and self.command_cancelled.is_set() and op in ('timed_start', 'timed_target')):
                raise InterruptedError('当前命令已取消，未发送 timed_aj 控制命令')
            if (not allow_cancelled and self.command_cancelled is not None and self.command_cancelled.is_set()
                    and op in ('command', 'request')
                    and data.get('command', '').split(maxsplit=1)[0] not in ('get', 'read_motor', 'stop_follower', 'manual_stop')):
                raise InterruptedError('当前命令已取消，未发送共享控制命令')
            if self._sock is None:
                raise ConnectionError('Shared A10 connection is closed')
            request = dict(op=op, session_id=self.session_id,
                           role=self.shared.get('role', 'inference'),
                           status_channel=self.status_channel, **data)
            request['timeout_s'] = self.timeout_s if timeout_s is None else timeout_s
            self._request_pending.set()
            try:
                self._sock.settimeout(request['timeout_s'] + 1.)
                self._sock.sendall(json.dumps(request, ensure_ascii=False).encode() + b'\n')
                line = self._reader.readline(1 << 20)
                if not line:
                    raise ConnectionError('Shared A10 service disconnected')
                response = json.loads(line)
            except (OSError, ValueError) as exc:
                self._note_failure(op, data, exc)
                self.close()
                raise
            finally:
                self._request_pending.clear()
            if 'error' in response:
                self._note_failure(op, data, response['error'])
                from ..operators.a10_operator import A10ControllerError, A10ProtocolError
                kinds = dict(ControlTransferredError=ControlTransferredError,
                             InterruptedError=InterruptedError, PermissionError=PermissionError,
                             TimeoutError=TimeoutError, ConnectionError=ConnectionError,
                             A10ControllerError=A10ControllerError, A10ProtocolError=A10ProtocolError)
                raise kinds.get(response.get('error_type'), RuntimeError)(response['error'])
            return response.get('result')

    def _note_failure(self, op, data, error):
        self.last_request_failure = dict(
            command=data.get('command', op), failed_wall_ns=time.time_ns(),
            error=str(error), transport='shared', host=self.host, port=self.ws_port,
            shared_host=self.shared.get('host', '127.0.0.1'),
            shared_port=self.shared.get('port', 5870), events=[])

    def interrupt(self):
        sock = self._sock
        if self._request_pending.is_set() and sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def request(self, command, *, status=False, timeout_s=None, raw=False, allow_cancelled=False):
        return self.call('request', command=command, status=status, raw=raw, timeout_s=timeout_s,
                         allow_cancelled=allow_cancelled)

    def send_command(self, command):
        self.call('command', command=command)

    def lease(self, operation):
        return self.call('lease', operation=operation,
                         require_feedback=self.shared.get('require_feedback', True))

    def close(self):
        sock, reader = self._sock, self._reader
        self._sock = self._reader = self.ws = self.udp = self.joint_udp = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        if reader is not None:
            reader.close()
