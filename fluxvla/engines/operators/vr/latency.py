"""Opt-in application WebSocket timing. Never estimates unsynchronised one-way latency."""
from collections import defaultdict, deque
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time
import uuid

from .diagnostics import XLeVRDiagnosticsWriter


class VRLatencyRecorder:
    def __init__(self, root="work_dirs/a10/vr/latency", *, context=None):
        self.path = Path(root) / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + uuid.uuid4().hex[:6])
        self.writer = XLeVRDiagnosticsWriter(self.path / "events.jsonl")
        self.lock = threading.RLock()
        self.closed = False
        self.connections = {}
        self.metrics = defaultdict(lambda: deque(maxlen=100000))
        self.totals = defaultdict(lambda: dict(count=0, max=0., over_100_ms=0, over_250_ms=0))
        self.last_consumed = {}
        self.started = time.monotonic()
        self.context = dict(context or {})
        self.event("run_start", context=self.context, clock="server monotonic seconds; browser performance milliseconds",
                   scope="application RTT includes scheduling; no one-way network estimate")
        print(f"VR延迟诊断目录：{self.path.resolve()}", flush=True)

    def event(self, kind, **fields):
        with self.lock:
            if not self.closed:
                self.writer.write(dict(event=kind, server_s=time.monotonic(), wall_time_ns=time.time_ns(), **fields))

    def metric(self, name, value):
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            return
        self.metrics[name].append(value)
        total = self.totals[name]
        total['count'] += 1
        total['max'] = max(total['max'], value)
        total['over_100_ms'] += value > 100
        total['over_250_ms'] += value > 250

    def connect(self, address):
        with self.lock:
            key = uuid.uuid4().hex
            self.connections[key] = dict(last_rx=None, last_tx=None, last_seq=None, probes={}, packets=0)
            self.event("connect", connection=key, address=str(address))
            return key

    def disconnect(self, key):
        with self.lock:
            state = self.connections[key]
            self.event("disconnect", connection=key, pending_probes=len(state['probes']), packets=state['packets'])
            state['probes'].clear()

    def packet(self, key, data, received):
        """Only combined pose packets carry traces; button/release/probe packets do not."""
        trace = data.get('_latency')
        if not isinstance(trace, dict):
            return None
        seq, sent = trace.get('seq'), trace.get('sent_ms')
        if type(seq) is not int or not isinstance(sent, (int, float)) or not math.isfinite(sent):
            return None
        with self.lock:
            state = self.connections[key]
            rx_gap = None if state['last_rx'] is None else (received - state['last_rx']) * 1000
            tx_gap = None if state['last_tx'] is None else sent - state['last_tx']
            self.metric('receive_interval_ms', rx_gap)
            self.metric('send_interval_ms', tx_gap)
            self.event('packet', connection=key, seq=seq, received_s=received, browser_sent_ms=sent,
                       receive_interval_ms=rx_gap, send_interval_ms=tx_gap,
                       sequence_gap=None if state['last_seq'] is None else seq - state['last_seq'] - 1,
                       buffered_bytes=trace.get('buffered_bytes'), page_visibility=trace.get('page_visibility'),
                       xr_visibility=trace.get('xr_visibility'), tracking=trace.get('tracking'))
            state.update(last_rx=received, last_tx=sent, last_seq=seq, packets=state['packets'] + 1)
            return dict(connection=key, seq=seq, received_s=received)

    def probe(self, key, data, received):
        """Route diagnostics separately from all robot input."""
        with self.lock:
            state = self.connections[key]
            pending = state['probes']
            if data.get('type') == 'latency_ping':
                token = data.get('id')
                if type(token) is not int:
                    return None
                # Bound unresolved probes, including a browser that stopped replying.
                if len(pending) >= 8:
                    expired = next(iter(pending))
                    pending.pop(expired)
                    self.event('probe_unresolved', connection=key, id=expired)
                sent = time.monotonic()
                pending[token] = (received, sent)
                age = None if state['last_rx'] is None else (sent - state['last_rx']) * 1000
                self.event('probe_received', connection=key, id=token, pose_age_ms=age,
                           browser_status=data.get('status'))
                return dict(type='latency_pong', id=token, pose_age_ms=age)
            if data.get('type') == 'latency_result':
                timing = pending.pop(data.get('id'), None)
                rtt = data.get('rtt_ms')
                if timing is None or not isinstance(rtt, (int, float)) or not math.isfinite(rtt) or rtt < 0:
                    return None
                processing = (timing[1] - timing[0]) * 1000
                self.metric('application_rtt_ms', rtt)
                self.event('probe_result', connection=key, id=data['id'], application_rtt_ms=rtt,
                           server_processing_ms=processing)
            return None

    def consume(self, raw, now):
        hands = raw if 'left' in raw or 'right' in raw else {'single': raw}
        with self.lock:
            for hand, action in hands.items():
                trace = action.get('xlevr.latency')
                if not trace:
                    continue
                key = (trace['connection'], hand)
                fresh = self.last_consumed.get(key) != trace['seq']
                age = max(0., (now - trace['received_s']) * 1000)
                self.metric('consumed_age_ms', age)
                if fresh:
                    self.metric('first_consume_delay_ms', age)
                self.last_consumed[key] = trace['seq']
                self.event('consume', hand=hand, **trace, consumed_s=now, age_ms=age, first_read=fresh)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.event('run_end')
            self.closed = True
            metrics = {}
            for name, values in self.metrics.items():
                ordered = sorted(values)
                def percentile(p):
                    return ordered[max(0, math.ceil(len(ordered) * p) - 1)]
                metrics[name] = dict(self.totals[name], retained_samples=len(ordered),
                                     median=percentile(.5), p95=percentile(.95), p99=percentile(.99))
        self.writer.close()
        summary = dict(context=self.context, duration_s=time.monotonic() - self.started, connections=len(self.connections),
                       metrics=metrics, percentile_scope='last 100000 samples per metric; count/max/gaps are all-run',
                       records_written=self.writer.records_written, dropped_records=self.writer.dropped_records,
                       writer_error=str(self.writer.error) if self.writer.error else None,
                       scope='Application RTT, not pure network latency. No cross-device timestamp subtraction. '
                             'Sequence gaps are not UDP loss. Consumer metrics are per hand. '
                             'Summary requires graceful shutdown; events survive abnormal termination up to last flush.')
        (self.path / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(f"VR延迟诊断已保存：{self.path.resolve() / 'summary.json'}", flush=True)
