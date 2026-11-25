"""timed_aj wire protocol. The shared relay owns all three transports.

Commands reuse the serialized command WebSocket. Long Plan replies have their
own socket; targets are plain JSON UDP, in physical ee0 then ee1 order.
"""
import json
import socket
import threading
import time

import numpy as np


ROW_TO_PHYSICAL = [7, 8, 9, 10, 11, 12, 0, 1, 2, 3, 4, 5]


def encode_target(stream_id, index, position_rad, final=False):
    if (type(stream_id) is not int or not 0 < stream_id < 2**64
            or type(index) is not int or not 0 <= index < 2**64):
        raise ValueError('Invalid timed stream ID or index')
    if type(final) is not bool:
        raise ValueError('final must be bool')
    q = np.asarray(position_rad)
    if q.shape != (12,) or q.dtype.kind not in 'iuf' or not np.isfinite(q).all():
        raise ValueError('Expected twelve finite absolute radians, physical ee0 then ee1')
    return json.dumps(dict(stream_id=stream_id, index=index,
                           position_rad=q.astype(float).tolist(), final=final),
                      separators=(',', ':'), allow_nan=False).encode('utf-8')


def validate_status(reply):
    if (type(reply.get('ret_code')) is not int or reply['ret_code'] != 0
            or reply.get('implementation') != 'joint_tracking.TimedAj'
            or reply.get('input_source') != 'udp'):
        raise RuntimeError(f'Invalid taj_status implementation/source: {reply}')
    limits = reply.get('motion_limits')
    if (reply.get('parameter_policy') != 'fixed_A' or not isinstance(limits, dict)
            or limits.get('source') != 'fixed_A'):
        raise RuntimeError('timed_aj requires the updated fixed_A parameter/limit policy')
    if reply.get('gap_policy') != 'hold_and_resume':
        raise RuntimeError('timed_aj requires the hold_and_resume protocol')
    for key in ('active', 'ready', 'accepting', 'stop_requested', 'waiting_for_input',
                'hold_transition', 'end_received', 'final_reference_reached'):
        if type(reply.get(key)) is not bool:
            raise ValueError(f'taj_status.{key} must be bool')
    for key in ('stream_id', 'received', 'consumed', 'queue_depth', 'state', 'reason'):
        if type(reply.get(key)) is not int or reply[key] < 0:
            raise ValueError(f'taj_status.{key} must be a nonnegative integer')
    if reply['stream_id'] >= 2**64 or reply['state'] > 6 or reply['reason'] > 8:
        raise ValueError('Invalid taj_status range')
    return reply


def require_stream(reply, stream_id, *, accepting=False):
    validate_status(reply)
    if (reply['stream_id'] != stream_id or not reply['active'] or reply['reason'] != 0
            or reply['state'] in (4, 5, 6) or reply['stop_requested']
            or (accepting and not (reply['ready'] and reply['accepting']))):
        raise RuntimeError(f'timed_aj stream interrupted/not ready: {reply}')
    return reply


def timed_config(value):
    """Only transport settings: tracking parameters remain downstream defaults A."""
    cfg = dict(enabled=False, udp_host=None, udp_port=10000, plan_ws_port=5888,
               network_ready=False)
    value = dict(value or {})
    if value.keys() - cfg.keys():
        raise ValueError(f'Unknown timed_aj options: {sorted(value.keys() - cfg.keys())}')
    cfg.update(value)
    for key in ('enabled', 'network_ready'):
        if type(cfg[key]) is not bool:
            raise ValueError(f'timed_aj.{key} must be bool')
    for key in ('udp_port', 'plan_ws_port'):
        if type(cfg[key]) is not int or not 0 < cfg[key] <= 65535:
            raise ValueError(f'Invalid timed_aj.{key}')
    if cfg['udp_host'] is not None and (not isinstance(cfg['udp_host'], str) or not cfg['udp_host'].strip()):
        raise ValueError('Invalid timed_aj.udp_host')
    return cfg


class TimedAjTransport:
    """Called under the relay locks. Never retries a start, target or stop.

    Keep ownership after an uncertain start/stop; only confirmed inactive and
    GET Stopped can release it. A dedicated reader drains delayed Plan replies.
    """
    def __init__(self, command, robot_state, host, command_port, status_port, timeout_s):
        self.command, self.robot_state = command, robot_state
        self.host, self.command_port, self.status_port = host, command_port, status_port
        self.timeout_s = timeout_s
        self.plan_ws = self.udp = None
        self.thread = None
        self.plan_error = None
        self.stream_id = None
        self.index = 0
        self.ended = False
        self.started_s = None
        self.start_attempted = False
        self.last_status = None

    def status(self):
        result = validate_status(self.command('taj_status', raw=True))
        self.last_status = result
        return result

    def start(self, options):
        from websockets.sync.client import connect
        from ..operators.a10_operator import _pack
        self.start_attempted = False
        cfg = timed_config(options)
        if not cfg['network_ready']:
            raise RuntimeError('timed_aj UDP currently listens on controller loopback only; '
                               'configure controller NIC listening or forwarding, then set network_ready=True')
        if cfg['plan_ws_port'] in (self.command_port, self.status_port):
            raise ValueError('timed_aj requires a dedicated Plan WebSocket port')
        status = self.status()
        if status['active']:
            raise PermissionError('An unmanaged timed stream is active')
        info = self.robot_state()['ret_context']['robot_msg']
        if tuple(info.get(k) for k in ('op_mode', 'activate', 'status', 'motion')) != (
                'Manual', 'Enabled', 'Normal', 'Stopped'):
            raise RuntimeError('timed_aj start requires Manual/Enabled/Normal/Stopped')
        previous_id = status['stream_id']
        self.close()
        self.plan_ws = connect(f'ws://{self.host}:{cfg["plan_ws_port"]}', proxy=None,
                               ping_interval=None, open_timeout=self.timeout_s,
                               close_timeout=self.timeout_s)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(self.timeout_s)
        self.udp.connect((cfg['udp_host'] or self.host, cfg['udp_port']))
        self.index, self.ended, self.plan_error = 0, False, None
        self.started_s = time.monotonic()
        # Send before starting the receiver: never wait for this long Plan ACK.
        self.start_attempted = True
        self.plan_ws.send(_pack(b'timed_aj', command=True))
        self.thread = threading.Thread(target=self._read_plan, args=(self.plan_ws,),
                                       name='a10-timed-plan-reply', daemon=True)
        self.thread.start()
        deadline = self.started_s + self.timeout_s
        while time.monotonic() < deadline:
            self._check_plan()
            status = self.status()
            if status['stream_id'] > previous_id:
                # Controller allocates the actual ID; never predict N+1.
                self.stream_id = status['stream_id']
                if status['reason'] or status['state'] in (4, 5, 6):
                    raise RuntimeError(f'timed_aj start aborted: {status}')
                if status['active'] and status['ready'] and status['accepting']:
                    return require_stream(status, self.stream_id, accepting=True)
            time.sleep(.01)
        raise TimeoutError('timed_aj did not become ready within the command timeout')

    def _read_plan(self, ws):
        from ..operators.a10_operator import _decode_reply
        try:
            reply = _decode_reply(ws.recv(), raw=True)
            if reply.get('ret_code') != 0:
                self.plan_error = RuntimeError(f'timed_aj Plan failed: {reply}')
        except Exception as exc:
            self.plan_error = exc

    def _check_plan(self):
        if self.plan_error is not None:
            raise RuntimeError('timed_aj Plan transport failed') from self.plan_error

    def send(self, position_rad, final=False):
        self._check_plan()
        stream_id, index = self.stream_id, self.index
        # No GET/taj_status round trip in the sending loop. Check the last known
        # status locally; downstream owns underrun detection and braking.
        if self.last_status is None:
            raise RuntimeError('Start and confirm timed_aj before sending')
        if (self.last_status['stream_id'] != stream_id or self.last_status['reason']
                or not self.last_status['active'] or not self.last_status['accepting']):
            raise RuntimeError(f'timed_aj stream interrupted: {self.last_status}')
        if self.ended or self.udp is None:
            raise RuntimeError('timed_aj final already sent or transport closed')
        body = encode_target(stream_id, index, position_rad, final)
        started = time.monotonic()
        sent = self.udp.send(body)
        finished = time.monotonic()
        if sent != len(body):
            raise ConnectionError('Incomplete timed UDP send; target will not be retried')
        self.index += 1
        self.ended = final
        return dict(stream_id=stream_id, index=index, final=final, bytes_sent=sent,
                    udp_send_started_s=started, udp_send_finished_s=finished)

    def stop(self):
        status = self.status()
        if self.stream_id is None or status['stream_id'] != self.stream_id:
            raise RuntimeError('timed_aj stream changed or start uncertain; no foreign stream stopped')
        # ACK means requested, never completed. Keep the lease on every failure.
        reply = self.command('taj_stop', raw=True)
        if reply.get('ret_code') != 0:
            raise RuntimeError(f'taj_stop rejected: {reply}')
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            status = self.status()
            if status['stream_id'] != self.stream_id:
                raise RuntimeError('timed stream changed during stop')
            info = self.robot_state()['ret_context']['robot_msg']
            if not status['active'] and info.get('motion') == 'Stopped':
                self.close()
                return status
            time.sleep(.02)
        raise TimeoutError('taj_stop ACK received but inactive/Stopped not confirmed')

    def close(self):
        if self.plan_ws is not None:
            self.plan_ws.close()
            self.plan_ws = None
        if self.thread is not None:
            self.thread.join(timeout=self.timeout_s)
            self.thread = None
        if self.udp is not None:
            self.udp.close()
            self.udp = None
        # Never carry a final flag, pending point counter or cached acceptance
        # state into another task. The closed UDP socket owns no retry queue.
        self.stream_id = None
        self.index = 0
        self.ended = False
        self.last_status = None
        self.plan_error = None
