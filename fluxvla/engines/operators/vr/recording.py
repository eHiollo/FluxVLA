"""Dual VR dataset collection. No second robot/serial owner, no policy dependency.

Control-thread work: button/state transitions and cached health only.
30 Hz sampling, image enqueue, camera reads, file finalization and m_init run
outside the 90 Hz motion thread. Robot motion
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
from ..zx_gripper import has_blocking_alarm
from .vr_events import RecordingButtons

LOG = logging.getLogger(__name__)


class _SampleNotReady(ValueError):
    """A missing/late measurement; distinct from malformed data or device alarms."""


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {str(k): plain(v) for k, v in list(value.items())}
    if isinstance(value, (list, tuple)): return [plain(v) for v in value]
    return value


def fresh(stamp, now, limit, label):
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not 0 <= now-stamp <= limit:
        raise _SampleNotReady(f'{label}: feedback missing/stale')


class RecordingSession:
    def __init__(self, runtime, config, *, button_check=False):
        self.runtime = runtime
        self.record = copy.deepcopy(dict(config['record']))
        self.backend = self.record.pop('backend', 'legacy')
        if self.backend not in ('legacy', 'lerobot'):
            raise ValueError('record.backend must be legacy or lerobot')
        self.cameras_config = copy.deepcopy(dict(config['cameras']))
        self.options = dict(config['capture'])
        self.workflow = self.options.get('workflow', 'legacy')
        if self.workflow not in ('legacy', 'lerobot'):
            raise ValueError('capture.workflow must be legacy or lerobot')
        # Same event contract and stage order as lerobot_record.record().
        self.events = dict(exit_early=False, rerecord_episode=False, stop_recording=False)
        self._auto_start = False
        self._reset_return_state = 'idle'
        self.gripper_faults = {}
        self.continue_on_spike = self.options.get('continue_on_spike', False)
        if type(self.continue_on_spike) is not bool:
            raise ValueError('capture.continue_on_spike must be boolean')
        self.continue_on_vr_pause = self.options.get('continue_on_vr_pause', False)
        if type(self.continue_on_vr_pause) is not bool:
            raise ValueError('capture.continue_on_vr_pause must be boolean')
        self.sampling_mode = self.options.get('sampling_mode', 'background_30hz')
        if self.sampling_mode not in ('control_loop', 'background_30hz'):
            raise ValueError('capture.sampling_mode must be control_loop or background_30hz')
        self._missing_since = None
        self.skipped_samples = 0
        self.sample_gaps = 0
        self.task = self.options['task']
        if not isinstance(self.task, str) or not self.task.strip():
            raise ValueError('capture.task must be nonempty text')
        for key in ('max_camera_age_s', 'max_robot_age_s', 'max_gripper_age_s', 'max_skew_s',
                    'max_sample_gap_s', 'max_clock_drift_s', 'confirmation_timeout_s', 'hud_hz'):
            value = self.options[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'capture.{key} must be finite and positive')
        self.max_missing_feedback_s = self.options.get('max_missing_feedback_s', 1.)
        if (isinstance(self.max_missing_feedback_s, bool)
                or not isinstance(self.max_missing_feedback_s, (float, int))
                or not math.isfinite(self.max_missing_feedback_s) or self.max_missing_feedback_s <= 0):
            raise ValueError('capture.max_missing_feedback_s must be finite and positive')
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
                if any(camera.get(k) != contract.get(k) for k in ('width', 'height', 'fps', 'identity', 'rotate_180')):
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
        self._sample_lock = threading.RLock()
        self._sample_stop = threading.Event()
        self._sample_thread = None
        self._background_sampling = False
        self._feedback_reader = None
        self._append_inflight = False

    @property
    def blocks_motion(self):
        return self.reset_thread is not None or self.reset_failed

    @property
    def capturing(self):
        """Motion ownership must not end an unfinished training episode."""
        return self.state == 'recording' or (
            self.state == 'resetting' and self._reset_return_state == 'recording'
            and not self.error and not self.reset_cancel.is_set())

    @property
    def can_sample(self):
        return not self.blocks_motion or self.capturing

    def _recordable_input_pause(self, raw=None, now=None):
        """Input/control timing pauses are independent of measured data validity.

        Motion remains locked by runtime; this never rearms or sends a target.
        Legacy storage and real controller/reset failures retain their guards.
        """
        if (self.backend != 'lerobot' or self.blocks_motion
                or getattr(self.runtime, '_dual_stop_failed', False)
                or getattr(self.runtime, 'communication_pause', None)):
            return False
        pause = getattr(self.runtime, 'input_pause_status', None)
        if isinstance(pause, dict):
            if pause.get('state') == 'stop_unconfirmed':
                return False
            reason = pause.get('reason')
            if reason in ('spike_pending', 'spike_reanchor'):
                return self.continue_on_spike
            return self.continue_on_vr_pause and reason in (
                'vr_stream_stale', 'input_stale', 'stale_age', 'sample_gap',
                'tracking_lost', 'invalid_sample', 'sequence_rollback', 'control_gap')
        # before_step and the background sampler can see stale input BEFORE
        # runtime publishes its pause. Do not invalidate the episode in that gap.
        return self.continue_on_vr_pause and self._vr_input_unavailable(raw, now)

    def _vr_input_unavailable(self, raw=None, now=None):
        raw = self.raw if raw is None else raw
        now = time.monotonic() if now is None else now
        if not isinstance(raw, dict) or set(raw) != {'left', 'right'}:
            return True
        for hand in raw.values():
            if not isinstance(hand, dict):
                return True
            stamp = hand.get('xlevr.sample_receive_time_s')
            age = now-stamp if isinstance(stamp, (int, float)) else hand.get('xlevr.sample_age_s')
            if (isinstance(age, bool) or not isinstance(age, (int, float))
                    or not math.isfinite(age) or not 0 <= age <= self.buttons.max_age_s):
                return True
            if hand.get('xlevr.enabled') and hand.get('xlevr.verification_tracking') is not True:
                return True
        return False

    def connect(self):
        if not self.button_check:
            # Reuse the existing read-only inventory matcher; never update config or open serial.
            from scripts.a10_devices import scan_devices, match_devices, require_matched
            rows = match_devices({'cameras': self.cameras_config}, scan_devices())
            require_matched(rows, 'cameras')
            writer_class = EpisodeWriter
            if self.backend == 'lerobot':
                from fluxvla.datasets.lerobot_episode_writer import LeRobotEpisodeWriter
                writer_class = LeRobotEpisodeWriter
            self.writer = writer_class(**self.record)
            self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='episode-finalize')
            self.cameras = BufferedCameraOperator(self.cameras_config)
            self.cameras.start()
            self._feedback_reader = self.runtime.robot.open_feedback_reader()
            if self.sampling_mode == 'background_30hz':
                self._start_sampling()
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
        with self._sample_lock:
            return self._before_step(raw, action)

    def _before_step(self, raw, action):
        self.raw = raw
        event = self.buttons.update(raw)
        self.input_fresh = event['fresh'] and not event.get('restarted', False)
        if not self.input_fresh:
            self.pending = None
            if self.reset_thread is not None: self.reset_cancel.set()
            if (self.capturing or (self.reset_thread is not None and self._reset_return_state == 'recording')) and not self._recordable_input_pause():
                self.fail('VR input stale; episode is incomplete')
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

    def _assemble(self, now, *, frame=None, raw=None):
        input_pause = self._recordable_input_pause(raw, now)
        if getattr(self.runtime, 'input_pause_status', None) is not None and not input_pause:
            raise ValueError('VR input paused; release controls and re-engage first')
        if raw is None:
            if not self.input_fresh and self.backend != 'lerobot':
                raise ValueError('Waiting for fresh paired VR input')
            raw = self.raw
        elif not input_pause:
            if set(raw) != {'left', 'right'}:
                raise ValueError('Waiting for fresh paired VR input')
            for hand in raw.values():
                fresh(hand.get('xlevr.sample_receive_time_s'), now,
                      self.buttons.max_age_s, 'VR input')
                if hand.get('xlevr.enabled') and hand.get('xlevr.verification_tracking') is not True:
                    raise ValueError('VR control requested without tracking')
        if (getattr(self.runtime, '_dual_stop_failed', False)
                or getattr(self.runtime, 'communication_pause', None)):
            raise ValueError('Robot stop unconfirmed or controller communication failed')
        if frame is None: frame = self.runtime.last_frame
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
            # A single missed reply pauses this gripper's writes, but the last
            # measured position remains usable within the SAME age/skew bounds.
            # Preserve its timestamp and communication_error in diagnostics.
            if (grip.get('error') or has_blocking_alarm(grip.get('alarm_a'), grip.get('alarm_b'),
                    drop_alarm_policy=grip.get('drop_alarm_policy', 'stop'))
                    or (self.backend != 'lerobot' and grip.get('enabled') is not True)):
                raise ValueError(f'{side} gripper not healthy/enabled: '
                                 f'{grip.get("error") or "check alarms/enable"}; clear before recording')
            times.append(t); grippers[side] = grip.get('actual_raw'); grip_details[side] = grip
        cameras = self.cameras.snapshot(stamp)
        for name, sample in cameras.items():
            fresh(sample['received_monotonic_s'], time.monotonic(), self.options['max_camera_age_s'], name)
            times.append(sample['received_monotonic_s'])
            settings = self.record['cameras'][name]
            if sample['image'].shape != (settings['height'], settings['width'], 3):
                raise ValueError(f'{name}: camera resolution mismatch')
        if max(times)-min(times) > self.options['max_skew_s']:
            raise _SampleNotReady('Camera/robot/gripper receive-time skew exceeded')
        state = pack_dual_state(joints, grippers)
        detail = dict(robot_received_monotonic_s=stamp,
                      controller_feedback_sequence=frame.get('controller_feedback_sequence'),
                      arm_received_monotonic_s={a['ee_id']:a['received_monotonic_s'] for a in arms.values()},
                      grippers=grip_details,
                      cameras={n:{k:v for k,v in c.items() if k!='image'} for n,c in cameras.items()},
                      control_frame_index=self.runtime._frame_index,
                      raw_input=raw, sdk_tool_command=self.runtime.last_sdk_command,
                      reference_control=self.runtime.last_reference_result)
        detail['control_phase'] = ('m_init' if self.reset_thread is not None else
                                   'input_pause' if input_pause else 'teleop')
        detail['input_pause'] = copy.deepcopy(getattr(self.runtime, 'input_pause_status', None))
        if input_pause and detail['input_pause'] is None:
            detail['input_pause'] = dict(reason='vr_input_unavailable', state='pending_runtime_guard')
        if self.reset_thread is not None or input_pause:
            # Old Cartesian targets aren't commands for reset or locked-input phases.
            detail['sdk_tool_command'] = None
            detail['reference_control'] = None
        return state, {n:c['image'] for n,c in cameras.items()}, plain(detail), cameras

    def _prepare_sample(self, now, *, frame=None, raw=None):
        self.ready_sample = None
        if not self.button_check and self.can_sample:
            try:
                self.ready_sample = self._assemble(now, frame=frame, raw=raw)
                self.health = 'Cameras 3/3 | Arms OK | Grippers OK'
                self._missing_since = None
            except Exception as exc:
                self.health = str(exc)
                if self.capturing:
                    if self.backend == 'lerobot' and isinstance(exc, _SampleNotReady):
                        # Like the old recorder, do not invent a frame or reject
                        # an entire episode for one slow acquisition. The true
                        # gap remains in diagnostics; persistent loss is fatal.
                        self.skipped_samples += 1
                        if self._missing_since is None: self._missing_since = now
                        if now-self._missing_since >= self.max_missing_feedback_s:
                            self.fail(f'Measurements unavailable for >={self.max_missing_feedback_s:g}s: {exc}')
                    else:
                        self.fail(str(exc))

    def _append_sample(self, now):
        # Native add_frame copies images/enqueues PNGs and may stall on IO. Keep
        # it outside the state lock so 90 Hz control still sees every button.
        with self._sample_lock:
            if not self.capturing or self.ready_sample is None or self._append_inflight:
                return
            state, images, detail, cameras = self.ready_sample
            try:
                if self._last_sample is not None:
                    if now-self._last_sample > self.options['max_sample_gap_s']:
                        if self.backend != 'lerobot':
                            raise ValueError('Recording sample gap exceeded; refusing fake continuous 30Hz')
                        self.sample_gaps += 1
                    drift = now-self._first_sample-self.frames/30
                    if self.backend != 'lerobot' and abs(drift) > self.options['max_clock_drift_s']:
                        raise ValueError('Recording cadence drift exceeded nominal 30Hz')
                if self._first_sample is None: self._first_sample = now
                for name, c in cameras.items():
                    if c['sequence'] == self._camera_sequences.get(name): self.repeated_images += 1
                self._camera_sequences = {n:c['sequence'] for n,c in cameras.items()}
                detail['repeated_image_count'] = self.repeated_images
                detail.update(capture_gap_s=None if self._last_sample is None else now-self._last_sample,
                              skipped_samples=self.skipped_samples, sample_gaps=self.sample_gaps,
                              sampling_mode=self.sampling_mode, nominal_capture_hz=self.record['fps'])
                self._append_inflight = True
            except Exception as exc:
                self.fail(str(exc))
                return
        write_error = None
        try:
            self.writer.append(state, state.copy(), images, capture_monotonic_s=now, diagnostics=detail)
        except Exception as exc:
            write_error = exc
        with self._sample_lock:
            self._append_inflight = False
            if write_error is not None:
                self.fail(str(write_error))
                return
            self.frames += 1; self._last_sample = now; self.elapsed = now-self._first_sample
            # A fault/quit may have arrived while this actual frame was written.
            # Never turn that failure into a saveable review state.
            if self.capturing and self.frames >= self.record.get('max_episode_frames',18000):
                self.state = 'review'
                if self.reset_thread is not None: self._reset_return_state = 'review'
                if self.workflow == 'lerobot':
                    self.events['exit_early'] = True
                    self.notice = 'FRAME LIMIT: RESET SCENE; RIGHT = SAVE + NEXT'
                else:
                    self.commands.put_nowait('save')

    def _start_sampling(self):
        self._background_sampling = True
        self._sample_thread = threading.Thread(target=self._sample_loop,
                                               name='a10-record-sampler', daemon=True)
        self._sample_thread.start()

    def _sample_loop(self):
        next_tick = time.monotonic()
        while not self._sample_stop.wait(max(0., next_tick-time.monotonic())):
            try:
                # Independent cache read: never waits for the motion owner or
                # issues a second robot GET. Raw VR read has no mapper effects.
                frame = self.runtime.robot.get_feedback_snapshot(self._feedback_reader)
                raw = self.runtime.teleoperator.input.get_hand_actions()
                with self._sample_lock:
                    if self._sample_stop.is_set(): break
                    self._prepare_sample(time.monotonic(), frame=frame, raw=raw)
                self._append_sample(time.monotonic())
            except Exception as exc:
                with self._sample_lock:
                    self.ready_sample = None; self.health = str(exc)
                    if self.capturing: self.fail(str(exc))
            # Never manufacture catch-up frames after a delayed acquisition.
            next_tick = max(next_tick + 1./self.record['fps'], time.monotonic())

    def _stop_sampling(self):
        self._sample_stop.set()
        if self._sample_thread is not None:
            self._sample_thread.join(2.)
            if self._sample_thread.is_alive() and self._feedback_reader is not None:
                self._feedback_reader.interrupt()
                self._sample_thread.join(2.)
            if self._sample_thread.is_alive():
                raise RuntimeError('Recording sampler did not stop')
            self._sample_thread = None

    def after_step(self):
        with self._sample_lock:
            self._poll_future()
            self._poll_reset()
            if (self.workflow == 'lerobot' and self._auto_start and self.state == 'idle'
                    and self.future is None and self.input_fresh and not self.blocks_motion
                    and getattr(self.runtime, 'input_pause_status', None) is not None):
                # A confirmed redo/start remains queued through an input pause.
                # Runtime only releases it after fresh neutral input + real stop;
                # this acknowledgement never starts a follower or sends a target.
                resume = getattr(self.runtime, 'resume_recording_input', None)
                if callable(resume): resume()
            if not self._background_sampling:
                frame = None
                if self.backend == 'lerobot' and not self.button_check and self.can_sample:
                    # The stop path receives real feedback internally. Read its
                    # cache, rather than the pre-stop frame or another GET.
                    frame = self.runtime.robot.get_feedback_snapshot(self._feedback_reader)
                self._prepare_sample(time.monotonic(), frame=frame)
            # Deferred disk transitions are requeued once per tick; quit/reset
            # cancellation must still be handled during a slow add_frame.
            for _ in range(min(16, self.commands.qsize())):
                try: command = self.commands.get_nowait()
                except queue.Empty: break
                self.command(command)
            if (self._auto_start and self.future is None and self.state == 'idle'
                    and self.ready_sample is not None and self.input_fresh and not self.blocks_motion
                    and getattr(self.runtime, 'input_pause_status', None) is None
                    and time.monotonic() >= self._saved_until):
                self._auto_start = False
                self.command('start')
            if not self._background_sampling:
                self._append_sample(time.monotonic())
        now = time.monotonic()
        self._hud(now)

    def input_paused(self, message):
        with self._sample_lock:
            self.pending = None
            self.ready_sample = None
            if self._recordable_input_pause():
                self.notice = 'input paused; recording' if self.capturing else 'input paused; release grips'
                LOG.warning('VR输入暂停：保留本集并继续采集有效实测数据，运动保护由runtime处理：%s', message)
                return
            if self.capturing or (self.future is not None and self.future_operation == 'starting'):
                self._input_pause_invalid = message
                self.fail(message)

    def gripper_faulted(self, faults):
        """Accessories gate training samples, never arm/VR control ownership."""
        with self._sample_lock:
            self.gripper_faults = dict(faults)
            if not faults:
                return
            message = '; '.join(f'{side} gripper: {error}' for side, error in faults.items())
            if self.capturing or (
                    self.future is not None and self.future_operation == 'starting'
                    and not self._input_pause_invalid):
                self._input_pause_invalid = message
                self.fail('Training episode interrupted: ' + message)

    def fail(self, message):
        with self._sample_lock:
            was_capturing = self.capturing
            self.error = message
            self.notice = ''
            if was_capturing or (self.reset_thread is not None and self._reset_return_state == 'recording'):
                self.state = 'failed'
                if self.reset_thread is not None: self.reset_cancel.set()
        LOG.error('录制本集失效（未发布）：%s', message)

    def _work(self, operation, fn, *args):
        self.future_operation = operation
        self.state = operation
        self.future = self.pool.submit(fn, *args)

    def _poll_future(self):
        with self._sample_lock:
            self._poll_future_locked()

    def _poll_future_locked(self):
        if self.future is None or not self.future.done(): return
        operation = self.future_operation
        try:
            value = self.future.result()
            if operation == 'starting':
                self.state = 'recording'; self.frames = 0; self.elapsed = 0.
                self._first_sample = self._last_sample = None
                self._camera_sequences = {}; self.repeated_images = 0
                self.skipped_samples = self.sample_gaps = 0
                self._missing_since = None
                if self.workflow == 'lerobot':
                    self.notice = 'RIGHT = FINISH EP / LEFT = REDO / UP = RESET / DOWN = STOP'
                    if self.events['stop_recording']:
                        self.state = 'review'
                        self.quit_requested = True
                        self.notice = 'Stopped before the first frame; buffer preserved'
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
            if self.workflow == 'lerobot' and operation != 'starting':
                if self.events['stop_recording']:
                    self.quit_requested = True
                    self.notice = 'Recording stopped'
                else:
                    self._auto_start = True
                    self.notice = (f'Episode {value+1} saved; NEXT EP STARTS AUTOMATICALLY'
                                   if operation == 'saving' else 'Re-recording; waiting for fresh measurements')
                self.events.update(exit_early=False, rerecord_episode=False)
            LOG.info('录制：%s %s', self.state, self.notice)
        except Exception as exc:
            self.state = 'failed'; self.error = str(exc)
            LOG.error('录制%s失败：%s；未保存的当前集在丢弃/退出时清理', operation, exc)
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
        with self._sample_lock:
            return self._command(command)

    def _command(self, command):
        if command == 'quit':
            self.quit_requested = True; self.reset_cancel.set(); return
        if command == 'status':
            LOG.info('录制状态：%s', json.dumps(self.status(), ensure_ascii=False)); return
        if command == 'help':
            LOG.info('start/save/discard/status/reset/stop/quit；右A+左摇杆右/左/下/上。'
                     'gripper-reset left/right，gripper-disable left/right为人工夹爪维护。'); return
        if not command: return
        if self.button_check:
            self.notice = f'CHECK ONLY: {command}'; return
        if self.blocks_motion:
            if command in ('down','stop'):
                self.reset_cancel.set(); self.notice = 'Cancelling reset; waiting for stop confirmation'
            else: self.notice = 'Reset owns motion; wait or use DOWN/stop to cancel'
            return
        if self._append_inflight:
            try: self.commands.put_nowait(command)
            except queue.Full: self.notice = 'Command queue full; wait for frame writer'
            return
        if self.future is not None:
            if self.workflow == 'lerobot' and command in ('down', 'stop'):
                self.events['stop_recording'] = True
                self._auto_start = False
                self.notice = 'STOP requested; waiting for current disk operation'
                return
            self.notice = 'Disk operation in progress; wait'; return
        if command.split()[0] in ('gripper-reset', 'gripper-disable'):
            parts = command.split()
            if (len(parts) != 2 or parts[1] not in ('left', 'right')
                    or not self._released()):
                self.notice = 'Release BOTH grips/triggers. Use gripper-reset left/right'; return
            action = 'reset' if parts[0] == 'gripper-reset' else 'disable'
            if action == 'disable':
                LOG.warning('人工掉使能将释放%s夹爪夹持力；此指令不用于自动故障恢复。', parts[1])
            try:
                self.runtime.prepare_gripper_maintenance()
                self.runtime._zx_channels[parts[1]].request_alarm_action(action)
            except Exception as exc:
                self.notice = f'Gripper action unavailable: {exc}; check device or restart session'
                LOG.warning('%s', self.notice)
                return
            self.notice = f'{parts[1]} gripper {action} requested; check feedback and center stick'
            LOG.info('人工夹爪操作：%s', self.notice)
            return
        if self.workflow == 'lerobot':
            return self._lerobot_command(command)
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

    def _lerobot_command(self, command):
        """Old initial reset -> recording -> reset/rerecord -> save -> next loop.

        RIGHT ends a stage; LEFT marks re-record, never silently saves it.
        Keep A gating and the existing m_init stop/confirmation boundary.
        """
        LOG.info('录制指令：%s；阶段=%s，设备=%s', command, self.state, self.health)
        if command == 'down': command = 'stop'
        if command == 'up': command = 'reset'
        if command in ('left', 'discard'):
            self.pending = None
            if self.state == 'idle' and self._auto_start:
                self.notice = 'Retry queued; release grips and center sticks'
                return
            if self.state not in ('recording', 'review', 'failed'):
                self.notice = 'No current episode. RIGHT = START'; return
            self.events.update(exit_early=True, rerecord_episode=True)
            if self.state == 'recording': self.state = 'review'
            self.notice = 'RERECORD: RESET SCENE; RIGHT = DISCARD + START AGAIN'
            return
        if command == 'right' and self.state == 'recording':
            self.events['exit_early'] = True
            self.state = 'review'; self.pending = None
            self.notice = 'RESET SCENE; RIGHT = SAVE + NEXT; LEFT = REDO'
            return
        if command == 'right':
            command = 'start' if self.state == 'idle' else 'save'
        if command == 'start':
            self.pending = None
            if self.state != 'idle':
                self.notice = 'Finish/reset the current episode first'; return
            if not self.input_fresh:
                self.notice = 'Waiting for fresh VR input'; return
            if getattr(self.runtime, 'input_pause_status', None) is not None:
                self._auto_start = True
                self.notice = 'Retry queued; release grips and center sticks'; return
            if self.ready_sample is None:
                self.notice = 'Cannot start: ' + self.health
                LOG.warning('录制未开始：%s', self.health)
                return
            self.events.update(exit_early=False, rerecord_episode=False, stop_recording=False)
            self._auto_start = False
            self._input_pause_invalid = None
            self.notice = 'Starting episode'
            self._work('starting', self.writer.start, self.task)
        elif command == 'save':
            self.pending = None
            if self.events['rerecord_episode'] and self.state in ('review', 'failed'):
                self._work('discarding', self.writer.discard)
            elif self.state in ('recording', 'review') and not self.error and self.frames >= 2:
                self._work('saving', self.writer.save)
            else:
                self.notice = 'Need valid frames. LEFT = REDO, then RIGHT = START AGAIN'
        elif command == 'stop':
            self._auto_start = False
            self.events.update(stop_recording=True, exit_early=True)
            if self.writer.active is not None:
                self._work('discarding', self.writer.discard)
            else:
                self.quit_requested = True
        elif command == 'reset':
            self._auto_start = False
            if (self.state not in ('idle', 'recording', 'review') or not self._released()
                    or getattr(self.runtime, 'input_pause_status', None) is not None):
                self.notice = 'Release BOTH grips/triggers and resolve faults before m_init'; return
            self.pending = None
            self._reset_return_state = self.state
            self._start_reset()
        else:
            self.notice = 'Unknown command; use help'

    def _start_reset(self):
        # Called after a successful control step, so no Cartesian send can race
        # the worker. VR motion I/O stays blocked; sampling reads measured caches.
        self.reset_cancel.clear(); self.state = 'resetting'; self.notice = 'Resetting arms; DOWN cancels'
        if self.capturing:
            self.notice = 'resetting; capture active'
        else:
            self.health = 'Capture paused; robot owned by m_init'
        for channel in self.runtime._zx_channels.values(): channel.submit(0., time.monotonic()+.1)
        self.reset_thread = threading.Thread(target=self._reset_worker, name='record-m-init', daemon=True)
        self.reset_thread.start()

    def _reset_worker(self):
        robot = self.runtime.robot
        error = None
        try:
            from ..kaanh_commands.m_init import move_to_init
            if self.reset_cancel.is_set(): raise InterruptedError('Reset cancelled before motion')
            move_to_init(robot, cancel_event=self.reset_cancel)
            # Console m_init restores its original follower. Recording instead
            # hands control back through a new VR anchor/session; stop the
            # restored follower before clearing the old local reference state.
            if robot.follower_stop_pending:
                robot.stop_follower()
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
        # Cancellation/data failure must not be overwritten by a worker that
        # happened to finish successfully at the same time.
        error = error or self.error or ('Reset cancelled' if self.reset_cancel.is_set() else None)
        # All mapper/reference state changes happen back on the control thread.
        r = self.runtime
        r._started_follower = not error and r.robot.follower_stop_pending
        r._dual_time = None; r._dual_require_release = True
        r._reference_stop_attempted = False
        r.teleoperator.reset_reference()
        if self.workflow == 'lerobot' and not error:
            # Old LeRobot allows UP -> recenter -> RIGHT while A remains held.
            # Disarm the direction edge, not the already verified A session.
            self.buttons.armed = False
        else:
            self.buttons.reset()
        self.pending = None
        if error:
            self.reset_failed = True; self.state = 'reset_failed'; self.error = error
            self.notice = 'Reset failed/cancelled; restart session after checking controller'
        else:
            self.state = self._reset_return_state if self.workflow == 'lerobot' else 'idle'
            self.notice = ('reset complete; capture active' if self.state == 'recording' else
                           'Reset complete; RIGHT = START' if self.state == 'idle' else
                           'Reset complete; RIGHT = SAVE + NEXT / START AGAIN')
        LOG.info('复位结果：%s %s', self.state, self.error)

    def status(self):
        with self._sample_lock:
            return dict(state=self.state, recording_state=self.state, frames=self.frames, elapsed_s=round(self.elapsed,2),
                        capture_active=self.capturing,
                        sampling_mode='background_30hz' if self._background_sampling else 'control_loop',
                        episode=(self.writer.info['total_episodes']+1 if self.writer else 0),
                        pending=self.pending, health=self.health, error=self.error, hint=self.notice,
                        task=self.task, repeated_images=self.repeated_images,
                        writer_backend=self.backend, skipped_samples=self.skipped_samples,
                        workflow=self.workflow, recording_events=dict(self.events),
                        gripper_faults=dict(self.gripper_faults),
                        gripper_warnings={side: snap['warning']
                            for side, channel in getattr(self.runtime, '_zx_channels', {}).items()
                            if (snap := channel.snapshot()).get('warning')},
                        sample_gaps=self.sample_gaps,
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
        if self.workflow == 'lerobot' and not self.button_check and self.future is None and not self.blocks_motion:
            directions = {key: dict(label='UNAVAILABLE', enabled=False) for key in directions}
            paused = getattr(self.runtime, 'input_pause_status', None) is not None
            if self.state == 'idle':
                show('right', 'QUEUE RETRY' if paused else 'START',
                     self.input_fresh if paused else self.ready_sample is not None)
                if self._auto_start:
                    show('left', 'RETRY QUEUED')
            elif self.state == 'recording':
                show('right', 'FINISH EP / RESET SCENE')
                show('left', 'REDO / RESET SCENE')
            elif self.state in ('review', 'failed'):
                show('left', 'REDO THIS EP')
                redo = self.events['rerecord_episode']
                show('right', 'DISCARD + START AGAIN' if redo else 'SAVE + NEXT',
                     redo or (self.state == 'review' and not self.error and self.frames >= 2))
            if self.state in ('idle', 'recording', 'review'):
                show('up', 'RESET', self._released() and not paused)
            show('down', 'DISCARD + EXIT')
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
        if self.workflow == 'lerobot' and phase in ('idle', 'review'):
            from .hud import hud_phase_from_record
            phase = hud_phase_from_record('reset', self.events['rerecord_episode'])
        status.update(type='hud', phase=phase, controls=controls)
        if phase == 'saved': status['episode'] = self._saved_episode
        # Hide the old success notice after the short nonblocking acknowledgement.
        if phase == 'idle' and self.notice.endswith(' saved'):
            status['hint'] = 'Ready for next episode'
        if self.workflow == 'lerobot':
            from .hud import build_hud_message
            old = build_hud_message(phase=phase, hint=self.notice or None)
            status['hint'] = old.get('hint', '')
            if self.state == 'idle' and self.ready_sample is None:
                status['hint'] += '\nWAIT FOR DEVICES: ' + self.health
        if self.button_check:
            status['hint'] = self.notice or 'Button check only - no device motion'
        elif self.gripper_faults:
            warning = '; '.join(f'{side.upper()} GRIPPER LOCKED: {error}'
                                for side, error in self.gripper_faults.items())
            status['hint'] = warning + '\nARMS AVAILABLE; CLEAR ALARM BEFORE RECORDING\n' + status['hint']
        if getattr(self.runtime, 'input_pause_status', None) is not None:
            pause = self.runtime.input_pause_hud()
            status.update({key: value for key, value in pause.items() if key != 'controls'})
            if self.state == 'failed':
                status['error'] = self.error
                status['hint'] += (' / INVALID EP: LEFT = REDO, RIGHT = CONFIRM'
                                   if self.workflow == 'lerobot' else ' / INVALID EPISODE: LEFT TO DISCARD')
            controls['prompt'] = pause['controls']['prompt']
            if self._auto_start and self.state == 'idle':
                # Do not replace the pending recording retry with "press grip".
                # Centering the menu stick is enough; moving an arm is optional.
                controls['prompt'] = 'release grips; center sticks'
                status['hint'] = 'retry pending'
            if self.capturing and self._recordable_input_pause():
                status['hint'] = 'input paused; recording'
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
        self._stop_sampling()
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
        self._stop_sampling()
        # Invoked AFTER robot cleanup. Native recording discards unsaved input;
        # an already explicitly requested save is allowed to finish first.
        try:
            if self.pool: self.pool.shutdown(wait=True)
            self._poll_future()
            if self.writer:
                self.writer.close()
                if self.writer.active is not None:
                    LOG.warning('本集尚未发布，保留于 %s/.recording/active；下次需检查并显式恢复。', self.record['root'])
        finally:
            if self.cameras: self.cameras.close()
            if self._feedback_reader: self._feedback_reader.close()
