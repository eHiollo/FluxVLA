"""FluxVLA ZMQ predictions to A10 TCP batch control."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

from .config import DEFAULT_CONFIG, DEFAULT_SITE, load_project
from .transport import PolicyClient
from .cameras import build_cameras
from .follower import build_follower


def checked_actions(raw: np.ndarray, state: np.ndarray, *,
                    max_chunk: int, max_step: float,
                    gripper_range: list[float]) -> np.ndarray:
    values = np.asarray(raw, dtype=np.float32)
    if values.ndim == 3 and values.shape[0] == 1:
        values = values[0]
    if values.ndim != 2 or values.shape[1] != 7:
        raise ValueError(f'Policy action must be [T,7], got {values.shape}')
    if not 1 <= len(values) <= max_chunk or not np.isfinite(values).all():
        raise ValueError('Policy action chunk length or values are invalid')
    previous = np.concatenate([state[None, :6], values[:-1, :6]], axis=0)
    if np.any(np.abs(values[:, :6] - previous) > max_step):
        raise ValueError('Policy joint step exceeds configured site limit')
    if np.any(values[:, 6] < gripper_range[0]) or np.any(
            values[:, 6] > gripper_range[1]):
        raise ValueError('Policy gripper target exceeds configured site range')
    return np.ascontiguousarray(values)


def _cameras(cfg: dict):
    return build_cameras(cfg)


def run(cfg: dict, cycles: int) -> int:
    robot_cfg = cfg['robot']
    if not robot_cfg['allow_motion']:
        raise ValueError('robot.allow_motion is false')
    policy = PolicyClient(cfg['inference']['host'], cfg['inference']['port'],
                          cfg['inference']['timeout_s'])
    robot = None
    cameras = {}
    run_dir = os.environ.get('FLUXVLA_A10_RUN_DIR')
    log_path = Path(run_dir) / 'cycles.jsonl' if run_dir else None
    log_file = log_path.open('a', encoding='utf-8') if log_path else None
    try:
        policy.ping()
        cameras = _cameras(cfg)
        robot = build_follower(cfg)
        if 'joint_batch' not in robot.capabilities:
            raise ValueError('Current A10 PI0.5 policy requires joint_batch capability')
        for cycle in range(cycles or 2**63):
            started = time.monotonic()
            status = robot.status()
            if status['remaining'] != 0:
                raise RuntimeError('A10 controller is busy; no new batch sent')
            state = robot.state()
            observation = {
                'qpos': state,
                'base_0_rgb': cameras['base_0_rgb'].read_rgb(),
                'right_wrist_0_rgb': cameras['right_wrist_0_rgb'].read_rgb(),
                'task_description': cfg['model']['prompt'],
            }
            actions = checked_actions(
                policy.predict(observation), state,
                max_chunk=cfg['inference']['max_action_chunk'],
                max_step=robot_cfg['max_joint_step_rad'],
                gripper_range=robot_cfg['gripper_mm_range'])
            robot.send_batch(actions)
            robot.wait_idle(robot_cfg['idle_poll_s'],
                            robot_cfg['idle_timeout_s'])
            time.sleep(robot_cfg['post_idle_settle_s'])
            if log_file:
                log_file.write(json.dumps({
                    'cycle': cycle + 1, 'state': state.tolist(),
                    'actions': len(actions),
                    'first_action': actions[0].tolist(),
                    'elapsed_s': time.monotonic() - started}) + '\n')
                log_file.flush()
            print(f'A10 cycle {cycle + 1}: {len(actions)} actions finished',
                  flush=True)
    finally:
        if robot is not None:
            robot.close()
        for camera in cameras.values():
            camera.close()
        policy.close()
        if log_file:
            log_file.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--cycles', type=int, default=0)
    args = parser.parse_args()
    cfg = load_project(os.environ.get('FLUXVLA_A10_CONFIG', DEFAULT_CONFIG),
                       os.environ.get('FLUXVLA_A10_SITE', DEFAULT_SITE))
    if args.cycles < 0:
        raise ValueError('--cycles must be nonnegative')
    return run(cfg, args.cycles)


if __name__ == '__main__':
    raise SystemExit(main())
