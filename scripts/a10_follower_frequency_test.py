#!/usr/bin/env python3
"""Compare A10 joint follower response to the same step at different send rates."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def degrees(values):
    return None if values is None else [math.degrees(float(x)) for x in values]


class StateSampler:
    """Independent shared client: state queries never serialize with target sends."""
    def __init__(self, robot, log, hz):
        self.robot, self.log, self.hz = robot, log, hz
        self.lock = threading.Lock()
        self.state = self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='follower-test-state', daemon=True)

    def _run(self):
        deadline = time.monotonic()
        try:
            while not self.stop.is_set():
                started = time.monotonic()
                state = self.robot.get_state()
                self.log.record('state', query_started_s=started,
                                received_monotonic_s=state.received_monotonic_s,
                                actual_deg=degrees(state.actual_joints_rad),
                                controller_target_deg=degrees(state.target_joints_rad), raw=state.raw)
                with self.lock:
                    self.state = state
                deadline = max(deadline + 1 / self.hz, time.monotonic())
                self.stop.wait(max(0., deadline - time.monotonic()))
        except BaseException as exc:
            with self.lock:
                self.error = exc

    def latest(self):
        with self.lock:
            if self.error:
                raise ConnectionError(f'State sampler failed: {self.error}') from self.error
            state = self.state
        if state is None or time.monotonic() - state.received_monotonic_s > .3:
            raise TimeoutError('No robot state newer than 0.3 seconds')
        if state.actual_joints_rad is None or state.has_error:
            raise RuntimeError('Missing actual joints or controller fault')
        info = state.raw['ret_context'].get('robot_msg', {})
        if info.get('activate') != 'Enabled' or state.follower_active is not True or state.follower_mode != 'Axis':
            raise RuntimeError('Enabled joint follower permission lost')
        return state

    def start(self):
        self.thread.start()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            with self.lock:
                ready = self.state is not None or self.error is not None
            if ready:
                self.latest()
                return
            time.sleep(.005)
        raise TimeoutError('State sampler did not become ready')

    def close(self):
        self.stop.set()
        self.thread.join(timeout=6)


def run_phase(robot, sampler, log, *, phase, hz, target_deg, origin_deg, args):
    """One constant absolute target; repeat until measured settling or timeout."""
    start = time.monotonic()
    deadline = start
    count = skipped = 0
    sent_times, samples = [], []
    previous = stable_since = reached90 = None
    first_send = None
    start_q = degrees(sampler.latest().actual_joints_rad)
    displacement = target_deg[args.joint - 1] - start_q[args.joint - 1]
    log.record('phase_start', phase=phase, hz=hz, target_deg=target_deg, start_deg=start_q)
    while True:
        now = time.monotonic()
        if now - start > args.timeout:
            raise TimeoutError(f'{phase}: did not settle within {args.timeout:g}s; test stopped')
        state = sampler.latest()
        q = degrees(state.actual_joints_rad)
        bounds = [2.] * 6
        bounds[args.joint - 1] += abs(args.delta_deg)
        if any(abs(x - o) > bound for x, o, bound in zip(q, origin_deg, bounds)):
            raise RuntimeError('Actual joints left the test excursion envelope')
        if log.dropped or log.write_errors:
            raise RuntimeError('Measurement log lost events; test stopped')
        stamp = state.received_monotonic_s
        if first_send is not None and stamp > first_send and (previous is None or stamp > previous[0]):
            error = max(abs(x - y) for x, y in zip(q, target_deg))
            speed = None if previous is None else max(abs(x - y) / (stamp - previous[0]) for x, y in zip(q, previous[1]))
            if abs(displacement) > args.tolerance_deg:
                progress = (q[args.joint - 1] - start_q[args.joint - 1]) / displacement
                if reached90 is None and progress >= .9:
                    reached90 = stamp - first_send
            samples.append((stamp, q))
            if error <= args.tolerance_deg and speed is not None and speed <= .5:
                if stable_since is None:
                    stable_since = stamp
                if stamp - stable_since >= args.settle_s:
                    break
            else:
                stable_since = None
            previous = stamp, q
        if now < deadline:
            time.sleep(min(deadline - now, .005))
            continue
        started = time.monotonic()
        try:
            robot.send_joints({'right': [math.radians(x) for x in target_deg]})
        except BaseException as exc:
            log.record('send_error', phase=phase, target_deg=target_deg,
                       send_started_s=started, error=repr(exc))
            raise
        finished = time.monotonic()
        if first_send is None:
            first_send = started
        sent_times.append(started)
        log.record('send', phase=phase, hz=hz, index=count, target_deg=target_deg,
                   send_started_s=started, send_finished_s=finished,
                   scheduled_s=deadline, late_s=started - deadline)
        count += 1
        deadline += 1 / hz
        # Late slots are skipped, never sent as a catch-up burst.
        if deadline < finished:
            missed = math.floor((finished - deadline) * hz) + 1
            skipped += missed
            deadline += missed / hz
    j = args.joint - 1
    speeds = [abs(b[1][j] - a[1][j]) / (b[0] - a[0]) for a, b in zip(samples, samples[1:])]
    result = dict(phase=phase, requested_hz=hz, sends=count, skipped_slots=skipped,
                  measured_send_hz=(count - 1) / (sent_times[-1] - sent_times[0]) if count > 1 else None,
                  time_to_90pct_s=reached90, settled_s=stamp - first_send,
                  first_in_tolerance_s=stable_since - first_send,
                  peak_sampled_speed_deg_s=max(speeds, default=0.),
                  final_max_error_deg=error, start_deg=start_q, target_deg=target_deg)
    log.record('phase_result', **result)
    return result


def export_csv(directory):
    """Full raw state stays in JSONL; CSV makes targets and measurements easy to plot."""
    last_target = [None] * 6
    with (directory / 'sends.csv').open('w') as send_file, (directory / 'states.csv').open('w') as state_file:
        sends, states = csv.writer(send_file), csv.writer(state_file)
        joints = [f'q{i}_deg' for i in range(1, 7)]
        sends.writerow(['send_started_s', 'send_finished_s', 'phase', 'requested_hz', 'late_s', *joints])
        states.writerow(['received_s', 'query_started_s', *['actual_' + x for x in joints],
                         *['controller_target_' + x for x in joints], *['last_sent_' + x for x in joints]])
        # Async state replies can enqueue ahead of a send which is still in flight.
        with (directory / 'events.jsonl').open() as stream:
            rows = [json.loads(line) for line in stream]
        rows.sort(key=lambda row: row.get('send_started_s', row.get('received_monotonic_s', row['monotonic_s'])))
        for row in rows:
            if row['event'] == 'send':
                last_target = row['target_deg']
                sends.writerow([row['send_started_s'], row['send_finished_s'], row['phase'], row['hz'], row['late_s'], *last_target])
            elif row['event'] == 'state':
                states.writerow([row['received_monotonic_s'], row['query_started_s'],
                                 *(row['actual_deg'] or [None] * 6),
                                 *(row['controller_target_deg'] or [None] * 6), *last_target])


def main(argv=None):
    parser = argparse.ArgumentParser(description='相同关节位移，比较不同发送频率的 follower 响应；会实际运动并回到起点')
    parser.add_argument('--rates', nargs='+', type=float, default=[30., 60., 120.])
    parser.add_argument('--joint', type=int, choices=range(1, 7), default=4)
    parser.add_argument('--delta-deg', type=float, default=5., help='相对起点的测试角度，默认 +5 度')
    parser.add_argument('--rounds', type=int, default=1)
    parser.add_argument('--sample-hz', type=float, default=50.)
    parser.add_argument('--timeout', type=float, default=8., help='每次去/回运动到位超时秒数')
    parser.add_argument('--tolerance-deg', type=float, default=.2)
    parser.add_argument('--settle-s', type=float, default=.3)
    parser.add_argument('--host', default='192.168.0.125')
    parser.add_argument('--ws-port', type=int, default=5999)
    parser.add_argument('--status-ws-port', type=int, default=5888)
    parser.add_argument('--udp-port', type=int, default=9998)
    parser.add_argument('--joint-udp-port', type=int, default=9999)
    parser.add_argument('--shared-port', type=int, default=5870)
    parser.add_argument('--log-dir', default='work_dirs/a10/follower_tests')
    args = parser.parse_args(argv)
    if not all(math.isfinite(x) and 1 <= x <= 240 for x in args.rates):
        parser.error('rates must be within 1..240 Hz')
    if not math.isfinite(args.delta_deg) or not .5 <= abs(args.delta_deg) <= 10:
        parser.error('delta-deg magnitude must be within 0.5..10 degrees')
    if args.rounds < 1 or not all(math.isfinite(x) and x > 0 for x in
            (args.sample_hz, args.timeout, args.tolerance_deg, args.settle_s)):
        parser.error('rounds and timing/tolerance options must be positive')
    if args.sample_hz > 100 or args.tolerance_deg >= abs(args.delta_deg) / 2:
        parser.error('sample-hz must be <=100; tolerance-deg must be less than half the displacement')

    from fluxvla.engines.operators.a10_operator import A10Operator
    from fluxvla.engines.communication.a10_shared_server import ConsoleSharedControl
    from fluxvla.engines.utils.a10_debug_log import A10DebugLog
    service = ConsoleSharedControl(('127.0.0.1', args.shared_port),
        host=args.host, ws_port=args.ws_port, status_ws_port=args.status_ws_port,
        udp_port=args.udp_port, joint_udp_port=args.joint_udp_port)
    options = dict(host=args.host, ws_port=args.ws_port, joint_udp_port=args.joint_udp_port,
                   udp_port=args.udp_port, command_mode='joint',
                   shared_control=dict(host='127.0.0.1', port=args.shared_port, role='inference'))
    robot = A10Operator(**options, allow_motion=True)
    reader = A10Operator(**options, allow_motion=False, timeout_s=.5)
    log = A10DebugLog(role='follower_frequency', root_dir=args.log_dir,
                      image_every_n_cycles=0, queue_size=2048, metadata=vars(args))
    sampler = None
    results = []
    error = None
    code = 0
    try:
        print(f'日志：{log.path}', flush=True)
        service.start()
        robot.connect()
        baseline = robot.get_state()
        if baseline.actual_joints_rad is None:
            raise RuntimeError('Actual joint feedback is required')
        origin = degrees(baseline.actual_joints_rad)
        target = origin.copy()
        target[args.joint - 1] += args.delta_deg
        print(f'第{args.joint}关节 {origin[args.joint-1]:.3f}° → {target[args.joint-1]:.3f}°；'
              f'依次测试 {args.rates} Hz，每次以60Hz返回同一起点。其他关节保持起始目标；不操作夹爪。', flush=True)
        robot.start_follower()  # Refuses an existing owner; never enables motors.
        reader.connect()
        sampler = StateSampler(reader, log, args.sample_hz)
        sampler.start()
        for repeat in range(args.rounds):
            for rate in args.rates:
                run_phase(robot, sampler, log, phase=f'{repeat+1}_{rate:g}Hz_baseline', hz=60.,
                          target_deg=origin, origin_deg=origin, args=args)
                result = run_phase(robot, sampler, log, phase=f'{repeat+1}_{rate:g}Hz_out', hz=rate,
                                   target_deg=target, origin_deg=origin, args=args)
                results.append(result)
                t90 = ('未达到' if result['time_to_90pct_s'] is None
                       else f'{result["time_to_90pct_s"]:.3f}s')
                print(f'{rate:g}Hz：实发 {result["measured_send_hz"]:.1f}Hz，'
                      f'90%位移 {t90}，稳定到位 {result["settled_s"]:.3f}s，'
                      f'采样峰值速度 {result["peak_sampled_speed_deg_s"]:.2f}°/s', flush=True)
                run_phase(robot, sampler, log, phase=f'{repeat+1}_{rate:g}Hz_return', hz=60.,
                          target_deg=origin, origin_deg=origin, args=args)
    except BaseException as exc:
        error = repr(exc)
        code = 130 if isinstance(exc, KeyboardInterrupt) else 1
        log.record('test_error', error=error)
        print(f'测试结束：{error}；不自动重发或回位。', file=sys.stderr)
    finally:
        try:
            if robot.follower_stop_pending:
                state = robot.stop_follower()
                confirmed = state.follower_active is False or robot.last_stop_confirmed
                log.record('follower_stop', confirmed=confirmed, raw=state.raw)
        except BaseException as exc:
            code = 1
            error = f'{error or ""}; stop unconfirmed: {exc!r}'
            log.record('follower_stop', confirmed=False, error=repr(exc))
            print(f'未确认 follower 停止：{exc}', file=sys.stderr)
        finally:
            if sampler is not None:
                sampler.close()
            for closer in (reader.close, robot.close, service.close):
                try:
                    closer()
                except BaseException as exc:
                    code = 1
                    log.record('cleanup_error', error=repr(exc))
            log.close()
            report = dict(results=results, error=error, dropped_events=log.dropped,
                          write_errors=log.write_errors,
                          comparison_valid=(code == 0 and not log.dropped and not log.write_errors),
                          state_time_semantics='local reply receipt time; no controller sample timestamp',
                          udp_semantics='local send success does not acknowledge receipt or arrival')
            (log.path / 'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            export_csv(log.path)
            print(f'已保存 sends.csv、states.csv、events.jsonl（完整state）、comparison.json：{log.path}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
