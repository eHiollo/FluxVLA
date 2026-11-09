"""Timing diagnostics tested without a VR headset or robot."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from fluxvla.engines.operators.vr.latency import VRLatencyRecorder
from fluxvla.engines.operators.vr.ws_server import VRWebSocketServer
from fluxvla.engines.operators.vr.server_config import XLeVRConfig
from fluxvla.engines.operators.vr.monitor import VRMonitorBridge
from fluxvla.engines.operators.vr.input import VRInput


class LatencyTests(unittest.TestCase):
    def test_clocks_repeated_reads_reconnect_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = VRLatencyRecorder(tmp)
            key = rec.connect('test')
            trace = rec.packet(key, {'_latency': {'seq': 1, 'sent_ms': 9000000}}, 10.)
            raw = {'left': {'xlevr.latency': trace}, 'right': {'xlevr.latency': trace}}
            rec.consume(raw, 10.010)
            rec.consume(raw, 10.040)
            rec.packet(key, {'_latency': {'seq': 2, 'sent_ms': 9000300}}, 10.31)
            with patch('fluxvla.engines.operators.vr.latency.time.monotonic', return_value=11.002):
                pong = rec.probe(key, {'type': 'latency_ping', 'id': 1}, 11.)
            self.assertEqual(pong['id'], 1)
            rec.probe(key, {'type': 'latency_result', 'id': 1, 'rtt_ms': 12.}, 11.1)
            rec.probe(key, {'type': 'latency_result', 'id': 1, 'rtt_ms': 999.}, 11.2)
            rec.disconnect(key)
            key2 = rec.connect('same browser reconnect')
            rec.packet(key2, {'_latency': {'seq': 1, 'sent_ms': 3}}, 12.)
            rec.disconnect(key2)
            rec.close(); rec.close()
            report = json.loads((rec.path / 'summary.json').read_text())
            m = report['metrics']
            self.assertEqual(m['send_interval_ms']['count'], 1)
            self.assertEqual(m['send_interval_ms']['median'], 300)
            self.assertEqual(m['receive_interval_ms']['over_250_ms'], 1)
            self.assertEqual(m['first_consume_delay_ms']['count'], 2)
            self.assertAlmostEqual(m['first_consume_delay_ms']['median'], 10.)
            self.assertEqual(m['consumed_age_ms']['count'], 4)
            self.assertEqual(m['application_rtt_ms']['count'], 1)
            self.assertEqual(m['application_rtt_ms']['median'], 12.)
            self.assertEqual(report['connections'], 2)
            self.assertEqual(report['dropped_records'], 0)

    def test_probe_does_not_enter_control_queue_and_trace_survives_snapshot(self):
        async def run(tmp):
            rec = VRLatencyRecorder(tmp)
            queue = asyncio.Queue()
            server = VRWebSocketServer(queue, XLeVRConfig(host_ip='127.0.0.1', https_port=8443, websocket_port=8442, certfile='unused', keyfile='unused'))
            server.latency = rec
            hand = dict(position=dict(x=1,y=2,z=3), quaternion=dict(x=0,y=0,z=0,w=1),
                        rotation=dict(x=0,y=0,z=0), buttons={}, gripActive=False)
            packet = dict(leftController=hand, rightController=hand,
                          _latency=dict(seq=1,sent_ms=100.))
            class Socket:
                remote_address = ('fake', 0)
                def __init__(self): self.sent = []
                async def send(self, text): self.sent.append(json.loads(text))
                def __aiter__(self):
                    async def messages():
                        yield json.dumps({'type':'latency_ping','id':1})
                        self.assert_empty()
                        yield json.dumps(packet)
                        yield json.dumps({'type':'latency_result','id':1,'rtt_ms':5.})
                    return messages()
                def assert_empty(self):
                    if not queue.empty(): raise AssertionError('probe became a control goal')
            socket = Socket()
            await server.websocket_handler(socket)
            self.assertEqual([v['type'] for v in socket.sent], ['latency_config','latency_pong'])
            bridge = VRMonitorBridge('.', {})
            traces = []
            while not queue.empty():
                goal = queue.get_nowait()
                bridge._record_goal(goal)
                if goal.target_position is not None and goal.metadata.get('latency_trace'):
                    traces.append(goal.metadata['latency_trace'])
            self.assertGreaterEqual(len(traces), 2)
            self.assertEqual(traces[0]['seq'], 1)
            self.assertEqual(traces[0]['connection'], traces[1]['connection'])
            snap = bridge.get_hand_snapshot()
            self.assertEqual(snap['left'].metadata['latency_trace']['seq'], 1)
            vr = VRInput(SimpleNamespace(fine_trigger_threshold=.5, require_squeeze_to_move=True))
            raw = {hand: vr._goal_action(goal) for hand, goal in snap.items()}
            self.assertEqual(raw['left']['xlevr.latency']['seq'], 1)
            rec.consume(raw, traces[0]['received_s'] + .01)
            self.assertEqual(rec.totals['first_consume_delay_ms']['count'], 2)
            rec.close()
        with tempfile.TemporaryDirectory() as tmp: asyncio.run(run(tmp))

    def test_old_client_without_trace_is_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = VRLatencyRecorder(tmp); key = rec.connect('old')
            self.assertIsNone(rec.packet(key, {'leftController':{}}, 1.))
            self.assertIsNone(rec.packet(key, {'_latency':{'seq':1,'sent_ms':float('nan')}}, 1.))
            rec.consume({'xlevr.enabled':False}, 1.)
            rec.close()
            self.assertEqual(json.loads((rec.path/'summary.json').read_text())['metrics'], {})


if __name__ == '__main__': unittest.main()
