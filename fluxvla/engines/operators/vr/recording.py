"""Dual VR dataset collection. No second robot/serial owner, no policy dependency.

Control-thread work: bounded snapshots, validation and nonblocking frame enqueue.
Camera reads, file finalization and m_init run outside that thread. Robot motion
and dataset state machines are distinct; a failed episode cannot be published.
"""
from __future__ import annotations

import copy
import json
import logging
import math
import queue
import select
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from fluxvla.datasets.a10_recording_schema import get_schema, pack_dual_state
from fluxvla.datasets.episode_writer import EpisodeWriter
from ..camera_operator import BufferedCameraOperator
from .vr_events import RecordingButtons

LOG = logging.getLogger(__name__)


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [plain(v) for v in value]
    return value


def fresh(stamp, now, limit, label):
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not 0 <= now-stamp <= limit:
        raise ValueError(f'{label}: feedback missing/stale')


class RecordingSession:
    def __init__(self, runtime, config, *, button_check=False):
        self.runtime = runtime
        self.record = copy.deepcopy(dict(config['record']))
        self.cameras_config = copy.deepcopy(dict(config['cameras']))
        self.options = dict(config['capture'])
        self.task = self.options['task']
        if not isinstance(self.task, str) or not self.task.strip():
            raise ValueError('capture.task must be nonempty text')
        for key in ('max_camera_age_s', 'max_robot_age_s', 'max_gripper_age_s', 'max_skew_s',
                    'max_sample_gap_s', 'max_clock_drift_s', 'confirmation_timeout_s', 'hud_hz'):
            value = self.options[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'capture.{key} must be finite and positive')
        if self.options['hud_hz'] > 10:
            raise ValueError('HUD rate must be <= 10Hz')
        self.buttons = RecordingButtons(**dict(self.options['buttons']))
        self.button_check = button_check
        if not getattr(runtime.teleoperator, 'is_dual', False):
            raise ValueError('Dataset recording/button check requires both hands')
        if self.record.get('schema') != 'a10_dual_v1' or self.record.get('fps') != 30:
            raise ValueError('Live recording requires a10_dual_v1 at 30Hz')
        if set(self.cameras_config) != set(get_schema('a10_dual_v1').cameras):
            raise ValueError('Three physical cameras required')
        if self.record.get('synthetic'):
            raise ValueError('Live entry cannot write into a synthetic dataset')
        if button_check:
            if runtime.robot is not None or runtime.execute:
                raise ValueError('Button check must not construct a robot connection')
        else:
            if not runtime.execute or runtime.observe_only or not runtime.robot.allow_motion:
                raise ValueError('Recording requires live dual teleop, not VR-only/observe')
            if runtime.control_mode != 'reference' or runtime.robot.camera_operator is not None:
                raise ValueError('Recording needs reference mode and separately owned cameras')
            if set(runtime._zx_channels) != {'left', 'right'}:
                raise ValueError('Enable both ZX grippers before dataset recording')
            for side, channel in runtime._zx_channels.items():
                identity = self.record['device_metadata']['grippers'][side]
                if (channel.serial != identity['identity']['serial'] or channel.device_id != identity['device_id']
                        or (channel.low, channel.high, channel.opened, channel.closed) != (0, 9000, 0, 9000)):
                    raise ValueError(f'{side}: gripper identity/range differs from recording contract')
            for name, camera in self.cameras_config.items():
                contract = self.record['cameras'][name]
                if any(camera.get(k) != contract.get(k) for k in ('width', 'height', 'identity', 'rotate_180')):
                    raise ValueError(f'{name}: camera identity/rotation differs from recording contract')
        self.writer = None; self.cameras = None; self.pool = None; self.future = None
        self.future_operation = None
        self.state = 'button_check' if button_check else 'idle'
        self.error = ''; self.notice = ''; self.health = 'Waiting for samples'
        self.frames = 0; self.elapsed = 0.; self._first_sample = self._last_sample = None
        self._camera_sequences = {}; self.repeated_images = 0
        self.ready_sample = None; self.raw = {}; self._last_hud = 0.
        self._hud_signature = None; self._saved_until = 0.; self._saved_episode = None
        self.pending = None; self.pending_at = 0.; self.stopping = False
        self.commands = queue.Queue(maxsize=16)
        self.quit_requested = False; self._closing = threading.Event()
        self.reset_cancel = threading.Event(); self.reset_thread = None
        self.reset_result = queue.Queue(maxsize=1)
        self.reset_failed = False; self._closed = False
        self.input_fresh = False
        self._input_pause_invalid = None

    @property
    def blocks_motion(self):
        return self.reset_thread is not None or self.reset_failed

    def connect(self):
        if not self.button_check:
            # Reuse the existing read-only inventory matcher; never update config or open serial.
            from scripts.a10_devices import scan_devices, match_devices, require_matched
            rows = match_devices({'cameras': self.cameras_config}, scan_devices())
            require_matched(rows, 'cameras')
            self.writer = EpisodeWriter(**self.record)
            self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='episode-finalize')
            self.cameras = BufferedCameraOperator(self.cameras_config)
            self.cameras.start()
        if self.options.get('terminal', True):
            threading.Thread(target=self._terminal, name='record-input', daemon=True).start()
        LOG.info('录制入口：右手功能键 %s + 左摇杆；start/save/discard/status/reset/stop/quit/help。'
                 '保存不是停止机械臂；复位需两手释放并再次确认。', self.buttons.button.upper())
        if self.button_check:
            LOG.info('只读按键检查：不连接机器人、夹爪或相机；按键结果在头显及终端显示。')

    def _terminal(self):
        while not self._closing.is_set():
            try:
                readable, _, _ = select.select([sys.stdin], [], [], .2)
                if not readable: continue
                text = sys.stdin.readline()
                if not text:
                    self.commands.put_nowait('quit'); return
                self.commands.put_nowait(text.strip().lower())
            except queue.Full:
                LOG.warning('录制指令队列已满，请等待当前操作完成')
            except (OSError, ValueError):
                return

    def before_step(self, raw, action):
        self.raw = raw
        event = self.buttons.update(raw)
        self.input_fresh = event['fresh'] and not event.get('restarted', False)
        if not self.input_fresh:
            self.pending = None
            if self.reset_thread is not None: self.reset_cancel.set()
            if self.state == 'recording': self.fail('VR input stale; episode is incomplete')
        if self.pending and time.monotonic()-self.pending_at > self.options['confirmation_timeout_s']:
            self.pending = None; self.notice = 'Confirmation expired'
        # No automatic gripper target on entering/leaving command mode. Existing
        # driver decelerates/holds according to its normal zero-axis behavior.
        if event['block_left'] or self.blocks_motion:
            for side in (('left', 'right') if self.blocks_motion else ('left',)):
                mapped = action[self.runtime.teleoperator.hand_to_arm[side]]
                mapped['gripper.pos'] = 0.
                mapped['_xlevr.anchor_command']['gripper'] = 0.
        if self.button_check:
            if event['command']: self.notice = 'BUTTON EVENT: ' + event['command']
        elif event['command']:
            try: self.commands.put_nowait(event['command'])
            except queue.Full: self.notice = 'Command queue full'
        if self.blocks_motion:
            for channel in self.runtime._zx_channels.values():
                channel.check_health()
                channel.submit(0., time.monotonic()+.1)
        return self.blocks_motion or self.button_check

    def _assemble(self, now):
        if getattr(self.runtime, 'input_pause_status', None) is not None:
            raise ValueError('VR input paused; release controls and re-engage first')
        if not self.input_fresh: raise ValueError('Waiting for fresh paired VR input')
        frame = self.runtime.last_frame
        if not frame or frame.get('has_error') is not False:
            raise ValueError('Robot feedback missing/faulted')
        stamp = frame.get('received_monotonic_s')
        fresh(stamp, time.monotonic(), self.options['max_robot_age_s'], 'robot')
        arms = frame.get('arms', {})
        if len(arms) != 2 or {a.get('ee_id') for a in arms.values()} != {0, 1}:
            raise ValueError('Expected actual feedback for both ee_id 0 and 1')
        joints = {}; times = [stamp]; grippers = {}
        for arm in arms.values():
            fresh(arm.get('received_monotonic_s'), time.monotonic(), self.options['max_robot_age_s'], 'arm')
            times.append(arm['received_monotonic_s']); joints[arm['ee_id']] = arm.get('joints')
        grip_details = {}
        for side, channel in self.runtime._zx_channels.items():
            grip = channel.snapshot(); t = grip.get('feedback_monotonic_s')
            fresh(t, time.monotonic(), self.options['max_gripper_age_s'], side+' gripper')
            if grip.get('error') or grip.get('alarm_a') or grip.get('alarm_b') or grip.get('enabled') is not True:
                raise ValueError(f'{side} gripper not healthy/enabled')
            times.append(t); grippers[side] = grip.get('actual_raw'); grip_details[side] = grip
        cameras = self.cameras.snapshot(stamp)
        for name, sample in cameras.items():
            fresh(sample['received_monotonic_s'], time.monotonic(), self.options['max_camera_age_s'], name)
            times.append(sample['received_monotonic_s'])
            settings = self.record['cameras'][name]
            if sample['image'].shape != (settings['height'], settings['width'], 3):
                raise ValueError(f'{name}: camera resolution mismatch')
        if max(times)-min(times) > self.options['max_skew_s']:
            raise ValueError('Camera/robot/gripper receive-time skew exceeded')
        state = pack_dual_state(joints, grippers)
        detail = dict(robot_received_monotonic_s=stamp,
                      arm_received_monotonic_s={a['ee_id']:a['received_monotonic_s'] for a in arms.values()},
                      grippers=grip_details,
                      cameras={n:{k:v for k,v in c.items() if k!='image'} for n,c in cameras.items()},
                      control_frame_index=self.runtime._frame_index,
                      raw_input=self.raw, sdk_tool_command=self.runtime.last_sdk_command,
                      reference_control=self.runtime.last_reference_result)
        return state, {n:c['image'] for n,c in cameras.items()}, plain(detail), cameras

    def after_step(self):
        now = time.monotonic()
        self._poll_future()
        self._poll_reset()
        self.ready_sample = None
        if not self.button_check and not self.blocks_motion:
            try:
                self.ready_sample = self._assemble(now)
                self.health = 'Cameras 3/3 | Arms OK | Grippers OK'
            except Exception as exc:
                self.health = str(exc)
                if self.state == 'recording': self.fail(str(exc))
        for _ in range(16):
            try: command = self.commands.get_nowait()
            except queue.Empty: break
            self.command(command)
        now = time.monotonic()
        if self.state == 'recording' and self.ready_sample is not None:
            state, images, detail, cameras = self.ready_sample
            try:
                if self._last_sample is not None:
                    if now-self._last_sample > self.options['max_sample_gap_s']:
                        raise ValueError('Recording sample gap exceeded; refusing fake continuous 30Hz')
                    drift = now-self._first_sample-self.frames/30
                    if abs(drift) > self.options['max_clock_drift_s']:
                        raise ValueError('Recording cadence drift exceeded nominal 30Hz')
                if self._first_sample is None: self._first_sample = now
                for name, c in cameras.items():
                    if c['sequence'] == self._camera_sequences.get(name): self.repeated_images += 1
                self._camera_sequences = {n:c['sequence'] for n,c in cameras.items()}
                detail['repeated_image_count'] = self.repeated_images
                self.writer.append(state, state.copy(), images, capture_monotonic_s=now, diagnostics=detail)
                self.frames += 1; self._last_sample = now; self.elapsed = now-self._first_sample
                if self.frames >= self.record.get('max_episode_frames',18000): self.command('save')
            except Exception as exc:
                self.fail(str(exc))
        self._hud(now)

    def input_paused(self, message):
        self.pending = None
        self.ready_sample = None
        if self.state == 'recording' or (self.future is not None and self.future_operation == 'starting'):
            self._input_pause_invalid = message
            self.fail(message)

    def fail(self, message):
        self.error = message
        if self.state == 'recording': self.state = 'failed'
        LOG.error('录制本集失效（未发布）：%s', message)

    def _work(self, operation, fn, *args):
        self.future_operation = operation
        self.future = self.pool.submit(fn, *args)
        self.state = operation

    def _poll_future(self):
        if self.future is None or not self.future.done(): return
        operation = self.future_operation
        try:
            value = self.future.result()
            if operation == 'starting':
                self.state = 'recording'; self.frames = 0; self.elapsed = 0.
                self._first_sample = self._last_sample = None
                self._camera_sequences = {}; self.repeated_images = 0
                if self._input_pause_invalid:
                    self.state = 'failed'
            else:
                self.state = 'idle'; self.stopping = False
                self.notice = f'Episode {value+1} saved' if operation == 'saving' else 'Episode discarded; ready'
            if operation == 'saving':
                self._saved_until = time.monotonic() + 1.5
                self._saved_episode = value + 1
            self.error = self._input_pause_invalid if operation == 'starting' and self._input_pause_invalid else ''
            if operation != 'starting': self._input_pause_invalid = None
            LOG.info('录制：%s %s', self.state, self.notice)
        except Exception as exc:
            self.state = 'failed'; self.error = str(exc)
            LOG.error('录制%s失败，保留未完成文件：%s', operation, exc)
        finally:
            self.future = None

    def _released(self):
        if set(self.raw) != {'left', 'right'}: return False
        for hand in self.raw.values():
            age = hand.get('xlevr.sample_age_s', -1.)
            if (not isinstance(age,(float,int)) or not math.isfinite(age) or not 0 <= age <= self.buttons.max_age_s
                    or hand.get('xlevr.grip_active') or hand.get('xlevr.buttons',{}).get('squeeze')
                    or hand.get('xlevr.trigger',1.) > .1): return False
        return True

    def command(self, command):
        if command == 'quit':
            self.quit_requested = True; self.reset_cancel.set(); return
        if command == 'status':
            LOG.info('录制状态：%s', json.dumps(self.status(), ensure_ascii=False)); return
        if command == 'help':
            LOG.info('start/save/discard/status/reset/stop/quit；右手功能键+左摇杆右/左/下/上。discard/reset需再次确认。'); return
        if not command: return
        if self.button_check:
            self.notice = f'CHECK ONLY: {command}'; return
        if self.blocks_motion:
            if command in ('down','stop'):
                self.reset_cancel.set(); self.notice = 'Cancelling reset; waiting for stop confirmation'
            else: self.notice = 'Reset owns motion; wait or use DOWN/stop to cancel'
            return
        if self.future is not None:
            self.notice = 'Disk operation in progress; wait'; return
        command = {'right':'save' if self.state in ('recording','review','failed') else 'start',
                   'left':'discard', 'up':'reset', 'down':'stop'}.get(command,command)
        if self.pending and command != self.pending: self.pending = None
        if command in ('start', 'reset') and getattr(self.runtime, 'input_pause_status', None) is not None:
            self.notice = 'VR input paused: release controls, wait, then press a side grip'; return
        if command == 'start':
            if self.state != 'idle': self.notice = 'Save or discard current episode first'; return
            if self.ready_sample is None: self.notice = 'Not ready: '+self.health; return
            self._input_pause_invalid = None
            self.notice = 'Starting episode'
            self._work('starting', self.writer.start, self.task)
        elif command == 'save':
            if self.state not in ('recording','review') or self.frames < 2:
                self.notice = 'Need >=2 valid frames; invalid episode must be discarded'; return
            self._work('saving', self.writer.save)
        elif command == 'stop':
            if self.state == 'recording': self.state = 'review'
            self.stopping = True
            self.notice = 'Capture stopped: RIGHT save / LEFT discard' if self.state in ('review','failed') else 'Capture idle'
        elif command in ('discard','reset'):
            if command == 'discard' and self.state not in ('recording','review','failed'):
                self.notice = 'No unfinished episode'; return
            if command == 'reset' and (self.state != 'idle' or not self._released()):
                self.notice = 'Save/discard first; release BOTH grips and triggers before reset'; return
            if self.pending != command:
                self.pending = command; self.pending_at = time.monotonic()
                self.notice = f'CONFIRM {command.upper()}: recenter then repeat direction'; return
            self.pending = None
            if command == 'discard': self._work('discarding', self.writer.discard)
            else: self._start_reset()
        else:
            self.notice = 'Unknown command; use help'

    def _start_reset(self):
        # Called after a successful control step, so no Cartesian send can race
        # the worker. Future control steps skip robot I/O until it joins.
        self.reset_cancel.clear(); self.state = 'resetting'; self.notice = 'Resetting arms; DOWN cancels'
        self.health = 'Capture paused; robot owned by m_init'
        for channel in self.runtime._zx_channels.values(): channel.submit(0., time.monotonic()+.1)
        self.reset_thread = threading.Thread(target=self._reset_worker, name='record-m-init', daemon=True)
        self.reset_thread.start()

    def _reset_worker(self):
        robot = self.runtime.robot
        error = None
        try:
            from ..kaanh_commands.m_init import move_to_init
            if robot.follower_stop_pending: robot.stop_follower()
            robot.wait_stopped()
            if self.reset_cancel.is_set(): raise InterruptedError('Reset cancelled before motion')
            move_to_init(robot, cancel_event=self.reset_cancel)
            robot.wait_stopped()
        except Exception as exc:
            error = str(exc)
            try: robot.stop_all_actions()
            except Exception as stop_error: error += f'; STOP UNCONFIRMED: {stop_error}'
        finally:
            self.reset_result.put(error)

    def _poll_reset(self):
        if self.reset_thread is None or self.reset_thread.is_alive(): return
        self.reset_thread.join(); self.reset_thread = None
        error = self.reset_result.get_nowait()
        # All mapper/reference state changes happen back on the control thread.
        r = self.runtime
        r._started_follower = False; r._dual_time = None; r._dual_require_release = True
        r._reference_stop_attempted = False
        r.teleoperator.reset_reference()
        self.buttons.reset(); self.pending = None
        if error:
            self.reset_failed = True; self.state = 'reset_failed'; self.error = error
            self.notice = 'Reset failed/cancelled; restart session after checking controller'
        else:
            self.state = 'idle'; self.notice = 'Reset complete; release grips before taking control'
        LOG.info('复位结果：%s %s', self.state, self.error)

    def status(self):
        return dict(state=self.state, frames=self.frames, elapsed_s=round(self.elapsed,2),
                    episode=(self.writer.info['total_episodes']+1 if self.writer else 0),
                    pending=self.pending, health=self.health, error=self.error, hint=self.notice,
                    task=self.task, repeated_images=self.repeated_images,
                    input_pause=getattr(self.runtime, 'input_pause_status', None),
                    effective_fps=round((self.frames-1)/self.elapsed,2) if self.frames>1 and self.elapsed>0 else None)

    def _hud_controls(self):
        """Presentation of the existing command state machine, never executes commands."""
        pressed = bool(self.raw.get('right', {}).get('xlevr.buttons', {}).get(self.buttons.button))
        directions = {key: dict(label='UNAVAILABLE', enabled=False)
                      for key in ('up', 'down', 'left', 'right')}
        def show(key, label, enabled=True):
            directions[key] = dict(label=label, enabled=enabled)
        if self.button_check:
            for key, label in dict(up='RESET', down='STOP CAPTURE', left='DISCARD / REDO', right='START / SAVE').items():
                show(key, label)
        elif self.blocks_motion:
            if self.reset_thread is not None: show('down', 'CANCEL RESET')
        elif self.future is None:
            if self.state == 'idle':
                paused = getattr(self.runtime, 'input_pause_status', None) is not None
                show('right', 'REARM VR FIRST' if paused else ('START' if self.ready_sample is not None else 'WAIT FOR DEVICES'),
                     self.ready_sample is not None and not paused)
                can_reset = self._released() and getattr(self.runtime, 'input_pause_status', None) is None
                show('up', 'RESET' if can_reset else 'RELEASE BOTH GRIPS', can_reset)
            elif self.state in ('recording', 'review', 'failed'):
                show('left', 'DISCARD / REDO')
                show('up', 'SAVE / DISCARD FIRST', False)
                if self.state != 'failed':
                    show('right', 'SAVE' if self.frames >= 2 else 'NEED 2 FRAMES', self.frames >= 2)
                if self.state == 'recording': show('down', 'STOP CAPTURE')
        if self.pending:
            key = 'up' if self.pending == 'reset' else 'left'
            if directions[key]['enabled']: show(key, 'CONFIRM ' + self.pending.upper())
        if not self.input_fresh:
            prompt = 'VR INPUT LOST - RELEASE CONTROLS'
        elif not self.buttons.released:
            prompt = 'RELEASE RIGHT ' + self.buttons.button.upper() + ' + CENTER LEFT STICK'
        elif (self.buttons.mode and not self.buttons.armed) or (not pressed and self.buttons.block_left):
            prompt = 'CENTER LEFT STICK'
        elif pressed:
            prompt = 'RIGHT ' + self.buttons.button.upper() + ' HELD - LEFT STICK MENU'
        else:
            prompt = 'HOLD RIGHT ' + self.buttons.button.upper() + ' FOR LEFT STICK MENU'
        return dict(visible=pressed and self.input_fresh, prompt=prompt,
                    ready=self.input_fresh and self.buttons.mode and self.buttons.armed,
                    directions=directions)

    def _hud(self, now):
        status = self.status()
        controls = self._hud_controls()
        phase = 'saved' if (self.state == 'idle' and now < self._saved_until
                            and self.pending is None and not self.error) else self.state
        status.update(type='hud', phase=phase, controls=controls)
        if phase == 'saved': status['episode'] = self._saved_episode
        # Hide the old success notice after the short nonblocking acknowledgement.
        if phase == 'idle' and self.notice.endswith(' saved'):
            status['hint'] = 'Ready for next episode'
        if self.button_check:
            status['hint'] = self.notice or 'Button check only - no device motion'
        if getattr(self.runtime, 'input_pause_status', None) is not None:
            pause = self.runtime.input_pause_hud()
            status.update({key: value for key, value in pause.items() if key != 'controls'})
            if self.state == 'failed': status['hint'] += ' / INVALID EPISODE: LEFT TO DISCARD'
            controls['prompt'] = pause['controls']['prompt']
            phase = status['phase']
        # Clock/frame counts stay rate limited; input gates and state transitions do not.
        signature = (phase, self.pending, self.error, status['hint'],
                     json.dumps(controls, sort_keys=True))
        if signature == self._hud_signature and now-self._last_hud < 1/self.options['hud_hz']:
            return
        self._last_hud = now; self._hud_signature = signature
        if self.button_check:
            LOG.info('按键检查：%s；buttons=%s', self.notice,
                     {side:hand.get('xlevr.buttons', {}) for side,hand in self.raw.items()})
        # Existing websocket sender queues asynchronously; no waiting on the control thread.
        try: self.runtime.teleoperator.send_feedback(status)
        except Exception: LOG.warning('HUD unavailable', exc_info=True)

    def before_close(self):
        self._closing.set(); self.reset_cancel.set()
        if self.reset_thread is not None:
            # Cancel without waiting for the robot IO lock; worker performs stop.
            self.runtime.robot.cancel_current_command()
            self.reset_thread.join(10.)
            if self.reset_thread.is_alive():
                self.runtime.robot.stop_all_actions()
                self.reset_thread.join(3.)
                if self.reset_thread.is_alive():
                    raise RuntimeError('Reset worker did not stop; confirm robot stop at controller')
            self._poll_reset()

    def close(self):
        if self._closed: return
        self._closed = True; self._closing.set()
        # Invoked AFTER robot cleanup. Closing never silently saves/discards a
        # partial episode. An already requested save is allowed to finish.
        try:
            if self.pool: self.pool.shutdown(wait=True)
            self._poll_future()
            if self.writer:
                self.writer.close()
                if self.writer.active is not None:
                    LOG.warning('本集尚未发布，保留于 %s/.recording/active；下次需检查并显式恢复。', self.record['root'])
        finally:
            if self.cameras: self.cameras.close()
