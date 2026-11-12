"""Standalone network diagnosis: synthetic evidence and local loopback only."""
import asyncio
import copy
import json
from pathlib import Path
import socket
import struct
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts.diagnose_vr_network import (
    ROOT, PacketCapture, capture_packets, diagnose, main, make_runtime,
    receiver_heartbeat, save_report, tracked,
)
from fluxvla.engines.operators.vr.latency import VRLatencyRecorder

PEER = '192.0.2.2'


def evidence():
    events = [dict(event='connect', connection='a', address=f"('{PEER}', 50001)"),
        dict(event='packet', connection='a', seq=1, server_s=10., wall_time_ns=10_000_000_000,
             receive_interval_ms=None, send_interval_ms=None),
        dict(event='packet', connection='a', seq=2, server_s=10.4, wall_time_ns=10_400_000_000,
             receive_interval_ms=400., send_interval_ms=11., xr_visibility='visible',
             tracking=dict(left=True, right=True))]
    for i in range(8):
        stamp = 10.05+i*.05
        events.append(dict(event='receiver_heartbeat', scheduled_s=stamp-.001,
                           observed_s=stamp, lag_ms=1., wall_time_ns=int(stamp*1e9)))
    def packet(t):
        return dict(wall_s=t, src=PEER, sport=50001, dport=8442, flow='0', seq=1,
                    length=100, retransmission=False, out_of_order=False,
                    lost_segment=False, zero_window=False)
    return events, [packet(10.), packet(10.4)], packet


class CorrelationTests(unittest.TestCase):
    def test_no_ingress_while_receiver_runs_is_not_called_python_computation(self):
        events, packets, _ = evidence()
        result = diagnose(events, packets, PEER, 8442, True)
        self.assertEqual(result['anomalies'][0]['cause'], 'before_host_ingress')

    def test_blocked_receiver_with_continuous_ingress_is_detected(self):
        events, packets, packet = evidence()
        events = [e for e in events if e['event'] != 'receiver_heartbeat']
        # Heartbeat executes later than the packet callback, but covers the same stall.
        events.append(dict(event='receiver_heartbeat', scheduled_s=10.04,
            observed_s=10.65, lag_ms=610., wall_time_ns=10_650_000_000))
        packets.extend(packet(10.+i*.011) for i in range(1,36))
        self.assertEqual(diagnose(events, packets, PEER,8442,True)['anomalies'][0]['cause'],
                         'receiver_loop_delayed')

    def test_tcp_gap_not_confused_with_event_loop_stall(self):
        events, packets, _ = evidence(); packets[-1]['retransmission'] = True
        self.assertEqual(diagnose(events, packets, PEER,8442,True)['anomalies'][0]['cause'],
                         'tcp_delivery_issue')

    def test_server_zero_window_counts_even_when_both_sides_use_loopback_ip(self):
        events, packets, packet = evidence()
        events[0]['address'] = "('127.0.0.1', 50001)"
        for p in packets: p['src'] = '127.0.0.1'
        window = packet(10.1)
        window.update(src='127.0.0.1',sport=8442,dport=50001,length=0,zero_window=True)
        packets.append(window)
        self.assertEqual(diagnose(events,packets,'127.0.0.1',8442,True)['anomalies'][0]['cause'],
                         'tcp_delivery_issue')

    def test_browser_stop_is_separate_from_network(self):
        events, packets, _ = evidence(); events[2]['send_interval_ms'] = 399.
        self.assertEqual(diagnose(events, packets, PEER,8442,True)['anomalies'][0]['cause'],
                         'browser_send_pause')

    def test_packets_received_with_normal_heartbeat_require_socket_delivery_check(self):
        events, packets, packet = evidence(); packets.append(packet(10.15))
        self.assertEqual(diagnose(events, packets, PEER,8442,True)['anomalies'][0]['cause'],
                         'socket_delivery_or_processing')

    def test_capture_loss_missing_heartbeat_wrong_flow_or_missing_capture_never_blames_network(self):
        for mode in ('drops','heartbeat','wrong_flow','empty'):
            events, packets, _ = evidence()
            if mode == 'wrong_flow':
                for p in packets: p['sport'] = 50002
            if mode == 'empty': packets = []
            if mode == 'heartbeat': events = [e for e in events if e['event'] != 'receiver_heartbeat']
            report = diagnose(events, packets, PEER,8442,True,capture_ok=mode!='drops')
            self.assertEqual(report['anomalies'][0]['cause'],'insufficient_evidence',mode)

    def test_measurement_window_and_peer_are_enforced(self):
        events, packets, _ = evidence()
        self.assertEqual(diagnose(events,packets,PEER,8442,True,[11.,12.])['anomalies'],[])
        self.assertEqual(diagnose(events,packets,'192.0.2.3',8442,True)['matched_pose_packets'],0)

    def test_system_clock_step_does_not_create_false_network_attribution(self):
        events, packets, _ = evidence(); events[2]['wall_time_ns'] += 1_000_000_000
        self.assertEqual(diagnose(events,packets,PEER,8442,True)['anomalies'][0]['cause'],
                         'insufficient_evidence')

    def test_mixed_tcp_and_receiver_delay_remains_uncertain(self):
        events, packets, _ = evidence(); packets[-1]['lost_segment'] = True
        events.append(dict(event='receiver_heartbeat', scheduled_s=10.05, observed_s=10.4,
                           lag_ms=350., wall_time_ns=10_400_000_000))
        self.assertEqual(diagnose(events,packets,PEER,8442,True)['anomalies'][0]['cause'],
                         'mixed_or_uncertain')


class StandaloneTests(unittest.TestCase):
    def test_live_default_config_is_forced_to_input_only_without_instantiating_hardware(self):
        with tempfile.TemporaryDirectory() as tmp, patch(
                'fluxvla.engines.operators.a10_operator.A10Operator.connect') as connect:
            runtime = make_runtime(ROOT/'configs/a10/a10_dual_arm_vr_teleop.py',
                                   ROOT/'configs/a10/a10_dual_arm_record.py',Path(tmp))
            self.assertIsNone(runtime.robot)
            self.assertFalse(runtime.execute)
            self.assertFalse(runtime.zx_grippers_enabled)
            self.assertFalse(runtime.record_controller)
            self.assertTrue(runtime.recording.button_check)
            self.assertFalse(runtime.recording.options['terminal'])
            self.assertIsNone(runtime.recording.writer)
            connect.assert_not_called()

    def test_heartbeat_detects_a_real_event_loop_block(self):
        async def run(tmp):
            recorder = VRLatencyRecorder(tmp)
            monitor = SimpleNamespace(command_queue=asyncio.Queue())
            stop = threading.Event()
            task = asyncio.create_task(receiver_heartbeat(monitor,recorder,stop,interval=.01))
            await asyncio.sleep(.025)
            time.sleep(.15)  # Intentionally block this exact receiving event loop.
            await asyncio.sleep(.025)
            stop.set(); await task
            recorder.close()
            summary = json.loads((recorder.path/'summary.json').read_text())
            self.assertGreater(summary['metrics']['receiver_loop_lag_ms']['max'],100.)
            self.assertEqual(summary['metrics']['receiver_loop_lag_ms']['over_100_ms'],1)
        with tempfile.TemporaryDirectory() as tmp: asyncio.run(run(tmp))

    def test_capture_start_failure_closes_owned_stream(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = PacketCapture(Path(tmp),PEER,8442,'lo')
            process = Mock(); process.poll.return_value = 1
            with patch('scripts.diagnose_vr_network.subprocess.Popen',return_value=process):
                with self.assertRaisesRegex(RuntimeError,'启动失败'): capture.start()
                capture.stop()
            self.assertTrue(capture.stream.closed)

    def test_invalid_ip_and_duration_fail_before_opening_any_service(self):
        for args in (['--vr-ip','bad'],['--vr-ip',PEER,'--duration','nan'],
                     ['--vr-ip',PEER,'--duration','0']):
            with patch('scripts.diagnose_vr_network.run') as run:
                self.assertEqual(main(args),1)
                run.assert_not_called()

    def test_tcp_parser_uses_real_pcap_header_sequence_and_flags(self):
        # Two copies of the same IPv4/TCP segment: tshark should flag retransmission.
        payload=b'x'*20
        tcp=struct.pack('!HHIIBBHHH',50001,8442,100,1,5<<4,0x18,65535,0,0)+payload
        ip=struct.pack('!BBHHHBBH4s4s',0x45,0,20+len(tcp),1,0,64,6,0,
                       socket.inet_aton(PEER),socket.inet_aton('192.0.2.1'))
        frame=b'\x00'*12+b'\x08\x00'+ip+tcp
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);pcap=directory/'test.pcap'
            with pcap.open('wb') as out:
                out.write(struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,160,1))
                for sec,usec in ((10,0),(10,400000)):
                    out.write(struct.pack('<IIII',sec,usec,len(frame),len(frame)));out.write(frame)
            rows=capture_packets(pcap,PEER,8442,directory/'packets.tsv')
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[0]['length'],20)
            self.assertTrue(rows[1]['retransmission'])

    def test_missing_capture_statistics_are_reported_as_incomplete(self):
        events, _, _ = evidence()
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
            (d/'tcpdump.log').write_text('capture failed')
            report=save_report(d,PEER,8442,[9.,11.],None,'no capture')
            self.assertFalse(report['capture_ok'])
            self.assertEqual(report['anomalies'][0]['cause'],'insufficient_evidence')
            self.assertIn('不完整',(d/'report.md').read_text())

    def test_dropped_diagnostic_events_prevent_false_network_attribution(self):
        for dropped in (0,1):
            events, packets, _ = evidence()
            with tempfile.TemporaryDirectory() as tmp:
                d=Path(tmp);(d/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
                (d/'tcpdump.log').write_text('2 packets captured\n0 packets dropped by kernel\n')
                (d/'summary.json').write_text(json.dumps(dict(dropped_records=dropped,writer_error=None)))
                with patch('scripts.diagnose_vr_network.capture_packets',return_value=packets):
                    report=save_report(d,PEER,8442,[9.,11.],None,None)
                self.assertEqual(report['capture_ok'],dropped==0)
                self.assertEqual(report['anomalies'][0]['cause'],
                                 'before_host_ingress' if dropped==0 else 'insufficient_evidence')


if __name__ == '__main__': unittest.main()
