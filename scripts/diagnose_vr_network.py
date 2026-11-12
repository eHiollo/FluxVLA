"""Standalone VR-only socket/heartbeat/packet-capture diagnosis; no robot I/O."""
import argparse
import asyncio
import ast
from datetime import datetime
import ipaddress
import json
import logging
import math
import os
from pathlib import Path
import pwd
import shutil
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
ROOT = Path(__file__).resolve().parents[1]


def route_to(peer):
    result = subprocess.run(['ip', '-j', 'route', 'get', peer], check=True,
                            text=True, capture_output=True)
    return json.loads(result.stdout)[0]


class PacketCapture:
    """Only this process group's filtered capture receives elevated privileges."""
    def __init__(self, directory, peer, port, interface):
        self.path = directory / 'network.pcap'
        self.log = directory / 'tcpdump.log'
        self.peer, self.port, self.interface = peer, port, interface
        self.process = None
        self.stream = None

    def start(self):
        tool = shutil.which('tcpdump')
        if not tool:
            raise RuntimeError('缺少tcpdump，请安装：sudo apt-get install tcpdump tshark')
        username = pwd.getpwuid(os.getuid()).pw_name
        command = [tool, '-i', self.interface, '-nn', '-s', '160', '-U', '-Z', username,
                   '-w', str(self.path), f'host {self.peer} and tcp port {self.port}']
        if os.geteuid() != 0: command = ['sudo', '-n', *command]
        self.stream = self.log.open('w')
        self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                        stderr=self.stream, start_new_session=True)
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('抓包启动失败，见tcpdump.log；需要时先在终端运行sudo -v')
            if 'listening on' in self.log.read_text(): return
            time.sleep(.05)
        raise RuntimeError('抓包未就绪，见tcpdump.log')

    def stop(self):
        try:
            if self.process is None or self.process.poll() is not None: return
            for name, timeout in (('INT', 5), ('TERM', 3), ('KILL', 3)):
                if self.process.poll() is not None: break
                if os.geteuid() == 0:
                    os.killpg(self.process.pid, getattr(signal, 'SIG'+name))
                else:
                    subprocess.run(['sudo', '-n', 'kill', '-'+name, '--', '-'+str(self.process.pid)],
                                   check=True, capture_output=True)
                try: self.process.wait(timeout=timeout)
                except subprocess.TimeoutExpired: continue
            if self.process.poll() is None:
                raise RuntimeError(f'抓包进程组{self.process.pid}未退出，请检查')
        finally:
            if self.stream is not None: self.stream.close()


async def receiver_heartbeat(monitor, recorder, stop, interval=.05):
    """Runs on the ACTUAL VR WebSocket event loop, never a separate timer thread."""
    deadline = time.monotonic() + interval
    while not stop.is_set():
        await asyncio.sleep(max(0., deadline-time.monotonic()))
        now = time.monotonic()
        if stop.is_set(): break
        lag = max(0., now-deadline)*1000
        with recorder.lock:
            recorder.metric('receiver_loop_lag_ms', lag)
            recorder.event('receiver_heartbeat', scheduled_s=deadline, observed_s=now,
                           lag_ms=lag, queue_depth=monitor.command_queue.qsize())
        # Do not fabricate catch-up heartbeats after a blocked event loop.
        deadline = now + interval


def make_runtime(config, record_config, output, ws_port=None, https_port=None):
    from mmengine import Config
    from fluxvla.engines.operators.vr.runtime import build_runtime
    from fluxvla.engines.operators.vr.recording import RecordingSession
    cfg = Config.fromfile(str(config))
    cfg.teleop.update(vr_only=True, observe_only=False, record_controller=False,
                     diagnostics_path=None)
    cfg.teleop.operator.allow_motion = False
    cfg.teleop.zx_grippers.enabled = False
    if ws_port is not None: cfg.teleop.input.network.websocket_port = ws_port
    if https_port is not None: cfg.teleop.input.network.https_port = https_port
    runtime = build_runtime(cfg.teleop)
    if runtime.execute or runtime.robot is not None or runtime._zx_channels:
        raise RuntimeError('诊断入口拒绝创建机器人或夹爪连接')
    runtime.vr.measure_latency = True
    runtime.vr.latency_root = str(output)
    runtime.vr.latency_context = dict(standalone=True, button_check=True,
        execute=False, recording=False, fps=30, hardware_io=False)
    record = Config.fromfile(str(record_config)).to_dict()
    record['capture']['terminal'] = False
    runtime.recording = RecordingSession(runtime, record, button_check=True)
    runtime.vr.recording_enabled = True
    # Keep the HUD active without printing every 2Hz button-check refresh.
    logging.getLogger('fluxvla.engines.operators.vr.recording').setLevel(logging.WARNING)
    return runtime


def tracked(raw):
    return set(raw) == {'left', 'right'} and all(
        h.get('xlevr.verification_tracking') is True
        and isinstance(h.get('xlevr.sample_age_s'), (int, float))
        and 0 <= h['xlevr.sample_age_s'] <= .25 for h in raw.values())


def capture_packets(pcap, peer, port, output):
    fields = ['frame.time_epoch', 'ip.src', 'tcp.srcport', 'tcp.dstport', 'tcp.stream',
              'tcp.seq_raw', 'tcp.len', 'tcp.analysis.retransmission',
              'tcp.analysis.fast_retransmission', 'tcp.analysis.out_of_order',
              'tcp.analysis.lost_segment', 'tcp.analysis.zero_window']
    command = ['tshark', '-n', '-r', str(pcap), '-Y', f'ip.addr == {peer} && tcp.port == {port}',
               '-T', 'fields', '-E', 'separator=/t', '-E', 'occurrence=f']
    for field in fields: command.extend(['-e', field])
    with output.open('w') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.PIPE, text=True)
    if result.returncode:
        raise RuntimeError('tshark解析失败：'+result.stderr.strip()[:250])
    rows = []
    for line in output.read_text().splitlines():
        values = line.split('\t')
        if len(values) != len(fields): raise ValueError('抓包解析字段不完整')
        stamp, src, sport, dport, flow, seq, length, *flags = values
        if not stamp or not flow: continue
        rows.append(dict(wall_s=float(stamp), src=src, sport=int(sport), dport=int(dport),
            flow=flow, seq=int(seq), length=int(length),
            retransmission=bool(flags[0] or flags[1]), out_of_order=bool(flags[2]),
            lost_segment=bool(flags[3]), zero_window=bool(flags[4])))
    return rows


def diagnose(events, packets, peer, port, heartbeat_available, measurement=None, capture_ok=True):
    """Evidence classes describe stages; they never claim proven Wi-Fi interference."""
    peers = {}
    previous = {}
    for event in events:
        if event['event'] == 'connect':
            try: peers[event['connection']] = ast.literal_eval(event['address'])[:2]
            except (ValueError, SyntaxError, TypeError): pass
    heartbeats = [e for e in events if e['event'] == 'receiver_heartbeat']
    measurements = [e for e in events if e['event'] == 'packet'
        and peers.get(e.get('connection'), (None,))[0] == peer]
    anomalies = []
    for event in measurements:
        key = event['connection']; old = previous.get(key); previous[key] = event
        rx = event.get('receive_interval_ms')
        if old is None or rx is None or rx <= 250: continue
        end = event['wall_time_ns']/1e9; start = end-rx/1000
        if measurement is not None and not measurement[0] <= end <= measurement[1]: continue
        tx = event.get('send_interval_ms')
        expected_first = event['server_s']-rx/1000
        hb = [h for h in heartbeats if h['scheduled_s'] <= event['server_s']
              and h['observed_s'] >= expected_first]
        hb_lag = max((h['lag_ms'] for h in hb), default=None)
        # Match the exact client TCP connection, not an unrelated browser/ACK stream.
        source_port = peers[key][1]
        incoming = [p for p in packets if p['src'] == peer and p['sport'] == source_port
                    and p['dport'] == port and p['length'] > 0
                    and start-.02 <= p['wall_s'] <= end+.02]
        local = [p for p in packets if start-.02 <= p['wall_s'] <= end+.12
                 and ((p['src'] == peer and p['sport'] == source_port and p['dport'] == port)
                      or (p['sport'] == port and p['dport'] == source_port))]
        # TCP flags are clues, not direct packet-loss counters (capture/offload can affect them).
        tcp_issue = any(p['retransmission'] or p['lost_segment'] or p['out_of_order']
                        or p['zero_window'] for p in local)
        hb_overlap = any(h['scheduled_s'] <= event['server_s']
                         and h['observed_s'] >= expected_first and h['lag_ms'] >= 100 for h in hb)
        prearrival = [p for p in incoming if start+.02 < p['wall_s'] < end-.05]
        connection_captured = any(p['src'] == peer and p['sport'] == source_port
                                  and p['dport'] == port for p in packets)
        clocks_consistent = abs((event['wall_time_ns']-old['wall_time_ns'])/1e6-rx) < 50
        if tx is None or not clocks_consistent:
            cause = 'insufficient_evidence'
        elif tx > 250:
            cause = 'browser_send_pause'
        elif not heartbeat_available or hb_lag is None or not capture_ok or not connection_captured:
            cause = 'insufficient_evidence'
        elif hb_overlap and (tcp_issue or not prearrival):
            cause = 'mixed_or_uncertain'
        elif hb_overlap:
            cause = 'receiver_loop_delayed'
        elif tcp_issue:
            cause = 'tcp_delivery_issue'
        elif not prearrival:
            cause = 'before_host_ingress'
        else:
            cause = 'socket_delivery_or_processing'
        anomalies.append(dict(time=datetime.fromtimestamp(end).isoformat(timespec='milliseconds'),
            connection=key, seq=event['seq'], cause=cause, receive_gap_ms=rx, send_gap_ms=tx,
            receiver_loop_lag_ms=hb_lag, ingress_payload_packets=len(incoming),
            payload_during_wait=len(prearrival), tcp_issue_observed=tcp_issue,
            xr_visibility=event.get('xr_visibility'), tracking=event.get('tracking')))
    return dict(anomalies=anomalies, matched_pose_packets=len(measurements),
        ingress_packets=len(packets), heartbeat_samples=len(heartbeats),
        capture_ok=capture_ok, measurement_window_wall_s=measurement,
        scope='接收空档不是单向网络延迟。抓包入口不是Wi-Fi空口；TCP标志不是丢包计数。'
              '无异常不证明长期稳定；未匹配到指定头显时不能作出结论。')


def save_report(directory, peer, port, measurement, run_error, capture_error):
    events = [json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
    packets = []
    if not capture_error:
        try: packets = capture_packets(directory/'network.pcap', peer, port, directory/'network.tsv')
        except Exception as exc: capture_error = str(exc)
    # Nonzero capture drops/early exit can create false "nothing reached the NIC" conclusions.
    log = (directory/'tcpdump.log').read_text() if (directory/'tcpdump.log').exists() else ''
    import re
    drops = re.search(r'(\d+) packets dropped by kernel', log)
    summary = json.loads((directory/'summary.json').read_text()) if (directory/'summary.json').exists() else {}
    events_complete = (summary.get('dropped_records') == 0 and summary.get('writer_error') is None)
    capture_ok = not capture_error and drops is not None and int(drops[1]) == 0 and events_complete
    report = diagnose(events, packets, peer, port, True, measurement, capture_ok=capture_ok)
    report.update(peer=peer, ws_port=port, run_error=run_error, capture_error=capture_error,
                  capture_dropped_packets=int(drops[1]) if drops else None,
                  diagnostic_events_complete=events_complete,
                  measured_duration_s=None if measurement is None else measurement[1]-measurement[0])
    (directory/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    labels = dict(browser_send_pause='浏览器停止发位姿', receiver_loop_delayed='接收事件循环延迟',
        tcp_delivery_issue='发现TCP交付异常线索', before_host_ingress='停顿在Thor抓包入口之前',
        socket_delivery_or_processing='数据已到Thor，检查TCP/TLS/WebSocket交付',
        mixed_or_uncertain='链路与接收循环可能同时异常，暂不能归因', insufficient_evidence='证据不完整')
    lines = ['# VR两分钟独立诊断', '', f'头显：{peer}，端口：{port}；不连接机器人、相机或夹爪。', '',
        f'匹配位姿包：{report["matched_pose_packets"]}；心跳：{report["heartbeat_samples"]}；'
        f'抓包有效：{capture_ok}；超过250ms的收包空档：{len(report["anomalies"])}。', '']
    if measurement is not None:
        lines += [f'从双手有效追踪开始，实际测试{report["measured_duration_s"]:.2f}秒。', '']
    if run_error or capture_error: lines += [f'本次不完整：{run_error or capture_error}', '']
    if measurement is None: lines += ['未进入双手有效追踪，未完成正式测试。', '']
    if not report['matched_pose_packets']: lines += ['未匹配到指定头显的带时戳位姿；不能判断停顿来源。', '']
    for item in report['anomalies']:
        lines.append(f'- {item["time"]}：发包间隔{item["send_gap_ms"]}ms，'
                     f'收包空档{item["receive_gap_ms"]:.1f}ms；{labels[item["cause"]]}。')
    lines += ['', report['scope'], '', '完整明细见report.json；events.jsonl含心跳和应用时序，network.pcap保留网络证据。']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')
    return report


def run(args):
    if not shutil.which('tcpdump') or not shutil.which('tshark'):
        raise RuntimeError('先安装抓包工具：sudo apt-get install tcpdump tshark')
    route = route_to(args.vr_ip)
    interface = args.interface or route['dev']
    runtime = make_runtime(args.config, args.record_config, args.output, args.ws_port, args.https_port)
    stop = threading.Event(); heartbeat = None; capture = None; directory = None
    measurement = None; error = None; capture_error = None
    try:
        runtime.connect(); runtime.recording.connect()
        recorder = runtime.vr.latency; directory = recorder.path
        monitor = runtime.vr._vr_monitor
        capture = PacketCapture(directory, args.vr_ip, monitor.config.websocket_port, interface)
        capture.start()
        heartbeat = asyncio.run_coroutine_threadsafe(receiver_heartbeat(monitor, recorder, stop), monitor._loop)
        recorder.event('diagnostic_start', peer=args.vr_ip, interface=interface,
                       duration_s=args.duration, heartbeat_interval_s=.05)
        local_ip = route.get('prefsrc') or route.get('src')
        print(f'独立VR诊断：等待双手有效追踪，随后计时{args.duration:g}秒。请刷新头显网页进入VR。\n'
              f'头显打开：https://{local_ip}:{monitor.config.https_port}\n'
              f'仅抓取{args.vr_ip}:{monitor.config.websocket_port}；目录：{directory.resolve()}', flush=True)
        deadline = time.monotonic()+args.wait_timeout
        started = None; next_tick = time.monotonic(); last_status = 0.
        while True:
            if capture.process.poll() is not None:
                capture_error = '抓包进程提前退出，不能据此认定网络未到包'
                raise RuntimeError(capture_error)
            if heartbeat.done(): heartbeat.result(); raise RuntimeError('接收心跳任务提前退出')
            runtime.step(); now = time.monotonic()
            if started is None:
                if tracked(runtime.teleoperator.last_raw):
                    started = now; measurement = [time.time(), None]
                    recorder.event('measurement_start', duration_s=args.duration)
                    print(f'双手追踪正常，开始{args.duration:g}秒测试；可操作侧键和A菜单。', flush=True)
                elif now >= deadline:
                    raise TimeoutError('等待双手有效追踪超时；请核对头显网址、IP并进入VR')
            elif now-started >= args.duration: break
            if started is not None and now-last_status >= 15:
                print(f'剩余{max(0., args.duration-(now-started)):.0f}秒', flush=True); last_status = now
            next_tick = max(next_tick+1/30, time.monotonic())
            time.sleep(max(0., next_tick-time.monotonic()))
    except KeyboardInterrupt:
        error = '用户提前结束；保留已记录证据'
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        if measurement is not None:
            measurement[1] = time.time()
            runtime.vr.latency.event('measurement_end')
        stop.set()
        if heartbeat is not None:
            try: heartbeat.result(timeout=2)
            except Exception as exc:
                heartbeat.cancel(); error = error or f'心跳未正常结束：{exc}'
        if capture is not None:
            try: capture.stop()
            except Exception as exc: capture_error = str(exc)
        try: runtime.recording.before_close()
        finally:
            try: runtime.close()
            finally: runtime.recording.close()
    if directory is None: raise RuntimeError(error or 'VR未启动')
    report = save_report(directory, args.vr_ip, runtime.vr._vr_monitor.config.websocket_port,
                         measurement, error, capture_error)
    print(f'诊断结束：{directory.resolve()/"report.md"}', flush=True)
    if error: print(error, file=sys.stderr)
    return 1 if error or capture_error or not report['capture_ok'] or not report['matched_pose_packets'] else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vr-ip', required=True, help='头显IPv4地址；不是机器人控制器IP')
    parser.add_argument('--duration', type=float, default=120., help='追踪正常后测量秒数，默认120')
    parser.add_argument('--wait-timeout', type=float, default=90., help='等待头显追踪的最长秒数')
    parser.add_argument('--config', type=Path, default=ROOT/'configs/a10/a10_dual_arm_vr_teleop.py')
    parser.add_argument('--record-config', type=Path, default=ROOT/'configs/a10/a10_dual_arm_record.py')
    parser.add_argument('--output', type=Path, default=ROOT/'work_dirs/a10/vr/network_checks')
    parser.add_argument('--interface', help='默认按头显IP自动选择Thor网卡')
    parser.add_argument('--ws-port', type=int, help='可选测试端口，默认沿用配置')
    parser.add_argument('--https-port', type=int, help='可选测试端口，默认沿用配置')
    args = parser.parse_args(argv)
    try:
        args.vr_ip = str(ipaddress.IPv4Address(args.vr_ip))
        for name in ('duration', 'wait_timeout'):
            value = getattr(args, name)
            if not math.isfinite(value) or not 1 <= value <= 600: raise ValueError(f'{name}须为1～600秒')
        for port in (args.ws_port, args.https_port):
            if port is not None and not 1 <= port <= 65535: raise ValueError('端口须为1～65535')
        args.output = args.output.expanduser().resolve()
        return run(args)
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'诊断未完成：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
