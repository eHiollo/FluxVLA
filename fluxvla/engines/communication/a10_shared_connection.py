"""Loopback RPC adapter for one shared KAANH control/status connection pair."""
import base64
import json
import socket
import threading
import uuid


class SharedServiceUnavailable(ConnectionRefusedError):
    """The local service refused the initial TCP connection, before any RPC."""


class _Datagram:
    def __init__(self, connection, mode):
        self.connection, self.mode = connection, mode

    def send(self, packet):
        return self.connection.call('target', mode=self.mode,
                                    packet=base64.b64encode(packet).decode('ascii'))


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

    def connect(self):
        try:
            address = (self.shared.get('host', '127.0.0.1'), self.shared.get('port', 5870))
            try:
                self._sock = socket.create_connection(address, self.timeout_s)
            except ConnectionRefusedError as exc:
                raise SharedServiceUnavailable(
                    f'A10 shared service {address[0]}:{address[1]} is not listening; '
                    'start python scripts/a10_control.py shell first '
                    '(the console starts the shared service automatically)') from exc
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._reader = self._sock.makefile('rb')
            self.ws = self._sock
            self.call('hello', robot_host=self.host, ws_port=self.ws_port,
                      joint_udp_port=self.joint_udp_port, udp_port=self.udp_port)
            self.udp, self.joint_udp = _Datagram(self, 'Cart'), _Datagram(self, 'Axis')
        except BaseException:
            self.close()
            raise

    def call(self, op, *, timeout_s=None, **data):
        with self._lock:
            if self._sock is None:
                raise ConnectionError('Shared A10 connection is closed')
            request = dict(op=op, session_id=self.session_id,
                           role=self.shared.get('role', 'inference'),
                           status_channel=self.status_channel, **data)
            request['timeout_s'] = self.timeout_s if timeout_s is None else timeout_s
            try:
                self._sock.settimeout(request['timeout_s'] + 1.)
                self._sock.sendall(json.dumps(request, ensure_ascii=False).encode() + b'\n')
                line = self._reader.readline(1 << 20)
                if not line:
                    raise ConnectionError('Shared A10 service disconnected')
                response = json.loads(line)
            except (OSError, ValueError):
                self.close()
                raise
            if 'error' in response:
                from ..operators.a10_operator import A10ControllerError, A10ProtocolError
                kinds = dict(InterruptedError=InterruptedError, PermissionError=PermissionError,
                             TimeoutError=TimeoutError, ConnectionError=ConnectionError,
                             A10ControllerError=A10ControllerError, A10ProtocolError=A10ProtocolError)
                raise kinds.get(response.get('error_type'), RuntimeError)(response['error'])
            return response.get('result')

    def request(self, command, *, status=False, timeout_s=None, raw=False):
        return self.call('request', command=command, status=status, raw=raw, timeout_s=timeout_s)

    def send_command(self, command):
        self.call('command', command=command)

    def lease(self, operation):
        return self.call('lease', operation=operation)

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
