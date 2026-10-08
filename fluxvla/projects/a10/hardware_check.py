"""Bounded camera capture and read-only A10 protocol checks for onsite setup."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from .cameras import build_cameras
from .config import DEFAULT_CONFIG, DEFAULT_SITE, fingerprint, load_project
from .transport import A10TcpClient


def device_path(device) -> Path:
    value = str(device)
    return Path(f'/dev/video{value}' if value.isdigit() else value)


def camera_inventory() -> list[dict]:
    aliases = [*Path('/dev/v4l/by-id').glob('*'),
               *Path('/dev/v4l/by-path').glob('*')]
    result = []
    for path in sorted(Path('/dev').glob('video*')):
        name_path = Path('/sys/class/video4linux') / path.name / 'name'
        result.append({'device': str(path),
                       'name': name_path.read_text().strip() if name_path.exists() else None,
                       'aliases': [str(alias) for alias in aliases
                                   if alias.resolve() == path.resolve()]})
    return result


def check_cameras(cfg: dict, output: Path, frames: int) -> dict:
    settings = cfg['cameras']
    paths = {key: device_path(item['device']) for key, item in settings.items()}
    missing = [str(path) for path in paths.values() if not path.exists()]
    result = {'inventory': camera_inventory(), 'configured_devices': {
        key: str(path) for key, path in paths.items()}}
    if missing:
        return dict(result, status='FAIL', error=f'Camera devices missing: {missing}')
    if len({path.resolve() for path in paths.values()}) != len(paths):
        return dict(result, status='FAIL', error='Both camera roles resolve to the same device')
    cameras = build_cameras(cfg)
    observations, completion_times = {}, []
    try:
        # The two devices remain open together, as they do during recording.
        # Exclude initial open/warmup from the throughput measurement.
        for index in range(frames + 2):
            for name, camera in cameras.items():
                rgb = camera.read_rgb()
                expected = (settings[name]['height'], settings[name]['width'], 3)
                if rgb.shape != expected or rgb.dtype != np.uint8:
                    raise ValueError(f'{name}: expected uint8 {expected}, got {rgb.dtype} {rgb.shape}')
                observations[name] = rgb
            if index >= 2:
                completion_times.append(time.monotonic())
        elapsed = completion_times[-1] - completion_times[0]
        fps = (frames - 1) / elapsed if elapsed > 0 else 0.0
        snapshots = {}
        for name, rgb in observations.items():
            target = output / f'{name}.png'
            if not cv2.imwrite(str(target), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise OSError(f'Cannot save camera snapshot: {target}')
            snapshots[name] = {'path': str(target), 'shape': list(rgb.shape),
                               'rotate_180': settings[name]['rotate_180']}
        return dict(result, status='PASS', sampled_pairs=frames,
                    paired_capture_fps=fps, target_fps=cfg['dataset']['fps'],
                    meets_90_percent_target=fps >= cfg['dataset']['fps'] * .9,
                    snapshots=snapshots,
                    note='Capture throughput only; excludes robot/VR/writer cost. '
                         'Camera identity, focus and rotation require visual confirmation; '
                         'this does not establish sustained sensor frame rate.')
    finally:
        for camera in cameras.values():
            camera.close()


def check_robot(cfg: dict) -> dict:
    robot = cfg['robot']
    if robot['backend'] != 'A10LegacyFollower':
        raise ValueError('Read-only protocol probe currently supports A10LegacyFollower only')
    client = A10TcpClient(robot['host'], robot['port'], timeout_s=3.0)
    try:
        before = client.status()
        if before['remaining'] != 0 or before.get('idle') is not True:
            return {'status': 'FAIL', 'error': 'Controller is busy; retry after existing operation',
                    'policy_status': before}
        state = client.state()
        after = client.status()
        if after['remaining'] != 0 or after.get('idle') is not True:
            return {'status': 'FAIL', 'error': 'Controller became busy during state query',
                    'policy_status': after}
        return {'status': 'PASS', 'endpoint': f'{robot["host"]}:{robot["port"]}',
                'state': state.tolist(), 'policy_status': after,
                'gripper_in_physical_range': bool(0 <= state[6] <= 100),
                'note': 'Protocol and numeric state only; freshness, units, pose and '
                        'physical limits require onsite verification.'}
    finally:
        client.close()


def _write(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def run_worker(cfg: dict, kind: str, output: Path, frames: int, timeout: float) -> dict:
    # A subprocess lets us terminate a camera driver stuck inside native read().
    command = [cfg['paths']['fluxvla_python'], '-m', __package__ + '.hardware_check',
               '--worker', kind, '--output', str(output), '--frames', str(frames)]
    with (output / f'{kind}.log').open('w', encoding='utf-8') as log:
        try:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                     timeout=timeout, check=False)
        except subprocess.TimeoutExpired:
            return {'status': 'FAIL', 'error': f'{kind} timed out after {timeout}s; worker terminated'}
    result_file = output / f'{kind}.json'
    if process.returncode != 0 or not result_file.is_file():
        return {'status': 'FAIL', 'error': f'{kind} worker failed; see {kind}.log'}
    return json.loads(result_file.read_text(encoding='utf-8'))


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--cameras-only', action='store_true')
    group.add_argument('--robot-only', action='store_true')
    parser.add_argument('--frames', type=int, default=60)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--worker', choices=('cameras', 'robot'), help=argparse.SUPPRESS)
    parser.add_argument('--output', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 2 <= args.frames <= 600 or not 1 <= args.timeout <= 300:
        parser.error('frames must be within [2, 600], timeout within [1, 300] seconds')
    cfg = load_project(os.environ.get('FLUXVLA_A10_CONFIG', DEFAULT_CONFIG),
                       os.environ.get('FLUXVLA_A10_SITE', DEFAULT_SITE))
    output = Path(args.output) if args.output else (
        Path(cfg['paths']['outputs']) / 'hardware_checks' /
        datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=True)
    if args.worker:
        try:
            result = (check_cameras(cfg, output, args.frames) if args.worker == 'cameras'
                      else check_robot(cfg))
        except Exception as exc:
            result = {'status': 'FAIL', 'error': f'{type(exc).__name__}: {exc}'}
        _write(output / f'{args.worker}.json', result)
        return 0
    components = ['cameras'] if args.cameras_only else ['robot'] if args.robot_only else ['cameras', 'robot']
    checks = {kind: run_worker(cfg, kind, output, args.frames, args.timeout)
              for kind in components}
    passed = all(item['status'] == 'PASS' for item in checks.values())
    report = {'created_at': datetime.now().astimezone().isoformat(),
              'status': 'PASS' if passed else 'FAIL', 'config_sha256': fingerprint(cfg),
              'checks': checks, 'robot_motion_commands_sent': False,
              'onsite_acceptance': 'PENDING',
              'note': 'Selected automated checks only; VR and human visual/physical checks remain pending.'}
    _write(output / 'report.json', report)
    for kind, item in checks.items():
        print(f'{kind}: {item["status"]} {item.get("error", "")}', flush=True)
    print(f'现场检查报告：{output / "report.json"}', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
