"""只读测量控制器 GET 频率和缓存读取耗时；不使能、不启动跟随、不发目标。"""
import argparse
import copy
import json
import sys
import time
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def summary(values):
    if not values:
        return None
    ordered = sorted(values)
    return dict(mean=sum(values) / len(values), p95=ordered[int((len(values)-1)*.95)], max=max(values))


def measure(robot, rate, duration):
    robot.start_feedback_polling(rate)
    stamps, durations, query_durations = {}, [], []
    first = robot.get_cached_frame()['controller_feedback_sequence']
    started = time.monotonic()
    next_tick = started
    last = None
    try:
        while time.monotonic() - started < duration:
            tick = time.monotonic()
            frame = robot.get_cached_frame()
            durations.append((time.monotonic() - tick)*1000)
            sequence = frame['controller_feedback_sequence']
            if sequence != first and sequence not in stamps:
                stamps[sequence] = frame['received_monotonic_s']
                query = frame.get('feedback_polling', {}).get('last_query_s')
                if query is not None:
                    query_durations.append(query*1000)
            last = frame
            next_tick += 1/90
            if next_tick < time.monotonic():
                next_tick = time.monotonic()
            time.sleep(max(0., next_tick-time.monotonic()))
    finally:
        robot.stop_feedback_polling()
    values = sorted(stamps.values())
    return dict(requested_feedback_hz=rate, duration_s=time.monotonic()-started,
        cache_reads=len(durations), unique_feedback=len(values),
        observed_feedback_hz=(len(values)-1)/(values[-1]-values[0]) if len(values)>1 else 0.,
        cache_call_ms=summary(durations), query_ms=summary(query_durations),
        receive_interval_ms=summary([(b-a)*1000 for a,b in zip(values, values[1:])]),
        poller={} if last is None else last.get('feedback_polling', {}))


@contextmanager
def simulated(delay_ms):
    # Reuse the independent protocol fixture; all endpoints are loopback.
    sys.path.insert(0, str(ROOT/'test'))
    from test_a10_feedback_polling import FeedbackPollingTest
    from test_a10_operator import status_reply
    fixture = FeedbackPollingTest()
    fixture.setUp()
    def reply():
        time.sleep(delay_ms/1000)
        return status_reply()
    fixture.fake.command_replies['get'] = reply
    try:
        yield fixture.client
    finally:
        fixture.tearDown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='configs/a10/a10_dual_arm_operator.py')
    parser.add_argument('--rates', type=float, nargs='+', default=[30, 60])
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--synthetic', action='store_true', help='仅本地假控制器')
    parser.add_argument('--get-delay-ms', type=float, default=15.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 0 < args.duration <= 60 or any(not 0 < r <= 90 for r in args.rates):
        parser.error('duration须在0..60秒，rates须在0..90 Hz')
    if args.synthetic:
        with simulated(args.get_delay_ms) as robot:
            results = [measure(robot, rate, args.duration) for rate in args.rates]
    else:
        from mmengine import Config
        from fluxvla.engines.operators.a10_operator import A10Operator
        cfg = Config.fromfile(args.config)
        options = copy.deepcopy(dict(cfg.teleop.operator if 'teleop' in cfg else cfg.operator))
        options.pop('type', None)
        options.update(allow_motion=False, cameras=None, gripper=None)
        robot = A10Operator(**options)
        try:
            robot.connect()
            results = [measure(robot, rate, args.duration) for rate in args.rates]
        finally:
            robot.close()
    report = dict(mode='synthetic' if args.synthetic else 'read_only_controller', results=results)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text+'\n')
    print(text)


if __name__ == '__main__':
    main()
