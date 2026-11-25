#!/usr/bin/env python3
"""Start controller state recording via the shared relay, or measure host clock offset."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def record(args):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from mmengine import Config
    from fluxvla.engines.communication.a10_shared_connection import SharedFollowerConnection

    cfg = Config.fromfile(str(args.config)).operator
    # Reuse the running relay and its controller WebSocket.
    shared = dict(cfg.shared_control, mode='external', role='teleop')
    connection = SharedFollowerConnection(
        cfg.host, cfg.ws_port, cfg.udp_port, cfg.timeout_s, cfg.joint_udp_port, shared=shared)
    status = {}
    try:
        connection.connect()
        status = connection.request('record_status')['ret_context']
        if status['recording'] or status['closing']:
            raise RuntimeError('Controller recorder already has a recording or is closing')
        status = connection.request('record_start')['ret_context']
        if not status['recording'] or status['error']:
            raise RuntimeError(f'Controller recorder failed to start: {status}')
        print(json.dumps(status, ensure_ascii=False), flush=True)
        interval = min(1., status['heartbeat_timeout_ms'] / 3000)
        deadline = time.monotonic() + args.seconds if args.seconds is not None else float('inf')
        while time.monotonic() < deadline:
            time.sleep(min(interval, max(0, deadline - time.monotonic())))
            status = connection.request('record_heartbeat')['ret_context']
            if not status['recording'] or status['error']:
                raise RuntimeError(f'Controller recorder stopped: {status}')
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()
    if status.get('file'):
        print(f"Controller file: {status['file']} (allow heartbeat timeout for final flush)")


def clock(args):
    remote = ('python3 -u -c "import sys,time; print(\'ready\'); '
              '[print(time.time_ns()) for line in sys.stdin]"')
    # One established SSH session excludes handshake time from offset samples.
    with subprocess.Popen(['ssh', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=3',
                           '-o', 'ServerAliveCountMax=2', '-T', args.host, remote],
                          text=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE) as proc:
        if proc.stdout.readline().strip() != 'ready':
            raise RuntimeError('SSH clock helper did not start')
        samples = []
        for _ in range(args.samples):
            start = time.time_ns()
            proc.stdin.write('\n')
            proc.stdin.flush()
            server = int(proc.stdout.readline())
            end = time.time_ns()
            samples.append((server - (start + end) // 2, end - start))
            time.sleep(.02)
        proc.stdin.close()
        if proc.wait(timeout=5):
            raise RuntimeError('SSH clock helper failed')
    offset, rtt = min(samples, key=lambda sample: sample[1])
    print(json.dumps(dict(controller_offset_ms=offset/1e6, clock_error_ms=rtt/2e6,
                          rtt_ms=rtt/1e6, thor_epoch_s=time.time()), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    recording = sub.add_parser('record', help='Record only, using a running local shared relay')
    recording.add_argument('--config', type=Path, default=Path('configs/a10/a10_dual_arm_operator.py'))
    recording.add_argument('--seconds', type=float, help='Omit to record until Ctrl+C')
    measure = sub.add_parser('clock', help='Measure remote-minus-Thor wall clock via SSH')
    measure.add_argument('host', help='SSH login, e.g. kaanh@192.168.0.150')
    measure.add_argument('--samples', type=int, default=12)
    args = parser.parse_args()
    if args.command == 'record':
        if args.seconds is not None and not 0 < args.seconds <= 3600:
            parser.error('seconds must be within (0, 3600]')
        record(args)
    else:
        if not 1 <= args.samples <= 100:
            parser.error('samples must be within 1–100')
        clock(args)


if __name__ == '__main__':
    main()
