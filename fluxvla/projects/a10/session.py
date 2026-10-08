"""Native VR teleoperation and recording orchestration inside the platform."""

from __future__ import annotations

import json
import queue
import signal
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .cameras import build_cameras
from .follower import build_follower
from .recording import ACTION_NAMES, EpisodeWriter
from .vr.diagnostics import XLeVRDiagnosticsWriter
from .vr.runtime import build_vr


def send_vr_action(follower, action):
    if action.get('_xlevr.reset_arm'):
        follower.reset()
        return
    anchor = action.get('_xlevr.anchor_command')
    if anchor is not None:
        follower.send_anchor_diagnostics(anchor)
    arm = [float(action.get(key, 0)) for key in ACTION_NAMES[1:7]]
    if not action.get('ee.enabled', False):
        arm = [0.] * 6
    follower.send_ee_delta(arm + [float(action['gripper.pos'])])


def _keyboard(commands):
    while True:
        try:
            text = input().strip().lower()
        except EOFError:
            return
        command = {'': 'next', 'n': 'next', 'r': 'rerecord', 's': 'stop'}.get(text)
        if command:
            commands.put(command)


def run_session(cfg, *, recording=False, root=None, task=None, episodes=1,
                resume=False, vr_only=False):
    if not vr_only and not cfg['robot']['allow_motion']:
        raise ValueError('robot.allow_motion=false; verify site settings first')
    if recording and (Path(root) / 'SIMULATED_DATA.json').exists():
        raise ValueError('Record real demonstrations into a separate real dataset')
    vr, mapper = build_vr(cfg)
    follower, cameras, writer, diagnostics = None, {}, None, None
    commands = queue.SimpleQueue()
    old_signals = {}
    if recording:
        writer = EpisodeWriter(root, cfg['cameras'], cfg['dataset']['fps'], resume=resume)
        threading.Thread(target=_keyboard, args=(commands,), daemon=True).start()
        for signum, command in [(signal.SIGUSR1, 'next'), (signal.SIGUSR2, 'rerecord'),
                                (signal.SIGTERM, 'stop')]:
            old_signals[signum] = signal.signal(signum, lambda *_args, c=command: commands.put(c))
        print('先准备场景。Enter/n 或左摇杆右：开始/保存；r 或左：重录；s 或下：停止。')
    completed, phase, tick = 0, 'ready', 0
    phase_start = time.monotonic()
    try:
        if not vr_only:
            follower = build_follower(cfg)
            if 'ee_delta' not in follower.capabilities:
                raise ValueError('Current A10 VR mapper requires ee_delta capability')
        if recording:
            cameras = build_cameras(cfg)
            if cfg['vr']['record_vr_diagnostics']:
                diagnostics = XLeVRDiagnosticsWriter(
                    Path(root) / 'meta/xlevr_diagnostics' /
                    f'session_{datetime.now():%Y%m%d_%H%M%S}.jsonl',
                    queue_size=cfg['vr']['diagnostics_queue_size'],
                    flush_every=cfg['vr']['diagnostics_flush_every'])
        vr.connect()
        while not recording or completed < episodes:
            started = time.monotonic()
            raw = vr.get_action()
            action = mapper(raw)
            events = vr.get_vr_events()
            if events['stop_recording']:
                commands.put('stop')
            elif events['rerecord_episode']:
                commands.put('rerecord')
            elif events['exit_early']:
                commands.put('next')
            if recording:
                limit = cfg['dataset']['episode_time_s' if phase == 'recording' else 'reset_time_s']
                if limit and time.monotonic() - phase_start >= limit:
                    commands.put('next')
                if not commands.empty():
                    command = commands.get()
                    if command == 'stop':
                        break
                    if command == 'rerecord':
                        if writer.active is not None:
                            writer.discard()
                        phase = 'ready'
                        mapper.reset()
                    elif command == 'next':
                        if phase == 'ready':
                            writer.start(task)
                            phase = 'recording'
                        elif len(writer.rows) >= 2:
                            effective_fps = len(writer.rows) / max(.001, time.monotonic() - phase_start)
                            print(f'本集实际采集速度约 {effective_fps:.1f} fps（配置 {cfg["dataset"]["fps"]} fps）', flush=True)
                            print(f'已保存 episode {writer.save()}', flush=True)
                            completed += 1
                            phase = 'ready'
                        else:
                            print('至少需要两帧才能保存，继续录制。', flush=True)
                    phase_start = time.monotonic()
                if completed >= episodes:
                    break
            state = follower.state() if follower is not None else None
            images = ({key: camera.read_rgb() for key, camera in cameras.items()}
                      if recording and phase == 'recording' else {})
            if follower is not None:
                send_vr_action(follower, action)
            if recording and phase == 'recording' and not action.get('_xlevr.reset_arm'):
                writer.append(state, [float(action.get(key, 0)) for key in ACTION_NAMES], images)
            if diagnostics:
                record = mapper.get_last_diagnostics() or {}
                record.update(phase=phase, elapsed_s=time.monotonic() - started,
                              capture_monotonic_s=time.monotonic())
                diagnostics.write(record)
            if tick % cfg['vr']['control_fps'] == 0:
                vr.send_hud({'type': 'hud', 'phase': phase,
                             'recorded_episodes': completed})
                if vr_only:
                    print(json.dumps({key: action.get(key) for key in ACTION_NAMES}), flush=True)
            tick += 1
            time.sleep(max(0., 1. / cfg['vr']['control_fps'] - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None and writer.active is not None:
            writer.discard()
        if diagnostics is not None:
            diagnostics.close()
        for camera in cameras.values():
            camera.close()
        if follower is not None:
            follower.close()
        if vr.is_connected:
            vr.disconnect()
        for signum, previous in old_signals.items():
            signal.signal(signum, previous)
    return completed


def simulate_recording(cfg, root, task, episodes, frames, resume=False):
    """Exercise the real encoder/writer using clearly marked synthetic input."""
    if frames < 2:
        raise ValueError('Simulation requires at least two frames')
    if (Path(root) / 'meta/info.json').exists() and not (Path(root) / 'SIMULATED_DATA.json').exists():
        raise ValueError('Do not append simulation frames to real demonstrations')
    writer = EpisodeWriter(root, cfg['cameras'], cfg['dataset']['fps'], resume=resume)
    Path(root, 'SIMULATED_DATA.json').write_text(json.dumps(
        {'simulation': True, 'purpose': 'pipeline validation; not robot demonstrations'}))
    try:
        for episode in range(episodes):
            writer.start(task)
            for frame in range(frames):
                state = np.array([frame * .001] * 6 + [50.], dtype=np.float32)
                action = np.array([1.] + [.0001] * 6 + [0.], dtype=np.float32)
                images = {key: np.full((settings['height'], settings['width'], 3),
                                       (frame * 20 + camera * 80) % 256,
                                       dtype=np.uint8)
                          for camera, (key, settings) in enumerate(cfg['cameras'].items())}
                writer.append(state, action, images)
            writer.save()
    finally:
        if writer.active is not None:
            writer.discard()
