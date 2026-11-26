"""FluxVLA operator contract for the existing dual A10 hardware adapter.

All pacing is here, not in the runner. Construction is hardware-free.
"""

from contextlib import contextmanager
import logging
import math
import threading
import time

import numpy as np

from ..utils.root import OPERATORS
from ..utils.a10_execution_trace import JOINTS, TailExecutionTrace
from ..utils.action_fusion import ExponentialActionFusion
from .base_operator import BaseOperator


@OPERATORS.register_module()
class A10DualOperator(BaseOperator):
    def __init__(self, config=None, settings=None, mode='preview', *, devices=None,
                 log=None, clock=None, bridge=None):
        super().__init__()
        if bridge is None:
            from fluxvla.projects.a10 import inference as bridge
        self.bridge = bridge
        self.config, self.settings, self.mode = config or {}, settings or {}, mode
        self.devices, self.log = devices, log or (lambda *a, **k: None)
        self.clock = clock or time
        self.command_mode = 'joint'
        self.cycle = 0
        self.attempted = self.closed = False
        self.trace = TailExecutionTrace(self.log)
        self.last_frame = None
        self._arrival_pending = False
        self._arrival_confirmed_s = None
        self.last_target = self.last_target_step = None
        self._last_packet_send_s = None
        self._last_packet_clock = None
        self.stream_final = False
        self.action_fusion = ExponentialActionFusion(self.settings.get('action_fusion'))
        from ..utils.chunk_execution import StreamReferenceClock
        self.reference_clock = StreamReferenceClock(self.settings.get('action_dt', 1/30))
        self._stream_mailbox_lock = threading.Lock()
        self._stream_pending = None
        self._stream_accepting = False
        from ..utils.a10_follower_tail import tail_config
        self.tail_config = tail_config(self.settings.get('follower_tail'))
        self._lifecycle_lock = threading.RLock()
        if devices is not None:
            self._attach_feedback()

    @property
    def timed_enabled(self):
        # simulate remains the existing hardware-free model loop simulator.
        return self.mode == 'motion' and (self.settings.get('timed_aj') or {}).get('enabled', False)

    def _attach_feedback(self):
        self.devices.feedback_callback = self._on_feedback

    def connect(self):
        if self.devices is None:
            config = dict(self.config, bridge=self.settings)
            self.devices = (self.bridge.SimDevices(self.log) if self.mode == 'simulate'
                            else self.bridge.LiveDevices(config, self.log, motion=self.mode == 'motion'))
            self._attach_feedback()

    def _on_feedback(self, state, query_started_s):
        # Match the trained physical ee_id mapping, independent of operator aliases.
        arms = {arm.ee_id: arm for arm in state.arms.values()}
        actual = np.concatenate([arms[i].actual_joints_rad for i in (1, 0)])
        targets = [getattr(arms[i], 'target_joints_rad', None) for i in (1, 0)]
        target = np.concatenate(targets) if all(q is not None for q in targets) else None
        self.trace.observe(actual, state.received_monotonic_s,
                           query_started_s=query_started_s, controller_target=target)

    def get_frame(self, slop=None, *, include_history=True):
        self.raise_trajectory_error()
        if self._arrival_pending:
            raise RuntimeError('Previous target arrival unconfirmed; observation blocked')
        from .camera_operator import CameraHistoryNotReady
        sample_options = ({'include_history': True}
                          if include_history and self.settings.get('longwam_history') else {})
        deadline = self.clock.monotonic() + 10.
        while True:
            try:
                sample = self.devices.sample(**sample_options)
                break
            except CameraHistoryNotReady:
                if self._traj_stop_event.is_set() or self.clock.monotonic() >= deadline:
                    raise TimeoutError('LongWAM camera history warmup timed out')
                self.clock.sleep(1 / 30)
        # Buffered cameras may still contain a frame from before confirmation.
        deadline = self.clock.monotonic() + 2.
        while self._arrival_confirmed_s is not None and (
                sample['robot_time'] < self._arrival_confirmed_s
                or any(t < self._arrival_confirmed_s for t in sample['camera_times'].values())):
            if self._traj_stop_event.is_set() or self.clock.monotonic() >= deadline:
                raise TimeoutError('No fresh post-arrival observation')
            self.clock.sleep(1 / self.tail_config['poll_hz'])
            sample = self.devices.sample(**sample_options)
        obs = self.bridge.observation(sample, self.settings)
        frame = dict(qpos=obs['observation/state'].copy(), has_error=False,
                     received_monotonic_s=sample['robot_time'],
                     robot_time=sample['robot_time'], camera_times=sample['camera_times'],
                     gripper_times=sample['gripper_times'], **sample['images'])
        if sample.get('image_history') is not None:
            frame['image_history'] = sample['image_history']
        self.last_frame = frame
        return frame

    def validate_actions(self, actions):
        current = self.get_frame(include_history=False)['qpos']
        self.bridge.check_steps(actions, current, self.settings['max_joint_step_rad'])
        return current

    def start_follower(self):
        if not self.attempted:
            self.attempted = True  # A failed start may already have taken effect.
            try:
                self.devices.start()
            except PermissionError:
                if getattr(self.devices, 'attempted', None) is False:
                    self.attempted = False
                raise

    def execute_trajectory(self, arm_trajectories, gripper_trajectories=None,
                           head_trajectory=None, dt=1 / 30, async_exec=False,
                           reference_time=None):
        if type(async_exec) is not bool or head_trajectory is not None:
            raise ValueError('Dual A10 requires boolean async_exec and has no head')
        if async_exec and self.tail_config['enabled']:
            raise ValueError('Asynchronous publishing cannot use per-block arrival/stop')
        if reference_time is not None and (
                not async_exec or not self.action_fusion.config['enabled']
                or not math.isfinite(reference_time)):
            raise ValueError('reference_time requires asynchronous action fusion and a finite timestamp')
        if set(arm_trajectories) != {'left', 'right'} or set(gripper_trajectories or {}) != {'left', 'right'}:
            raise ValueError('Both physical arms and normalized gripper trajectories are required')
        n = self._validate_trajectory_lengths(arm_trajectories, gripper_trajectories)
        for name in ('left', 'right'):
            arm = np.asarray(arm_trajectories[name])
            grip = np.asarray(gripper_trajectories[name])
            if (arm.shape != (n, 6) or arm.dtype.kind not in 'fiu'
                    or grip.shape != (n,) or grip.dtype.kind not in 'fiu'):
                raise ValueError('Each arm must be numeric [T,6] with a numeric [T] gripper')
        rows = np.column_stack([arm_trajectories['left'], gripper_trajectories['left'],
                                arm_trajectories['right'], gripper_trajectories['right']])
        if rows.shape != (n, 14) or not n or not np.isfinite(rows).all():
            raise ValueError('Expected finite dual trajectory [T,14]')
        if not math.isfinite(dt) or dt <= 0 or dt != self.settings['action_dt']:
            raise ValueError('Trajectory dt must match the configured model action_dt')
        if np.any((rows[:, [6, 13]] < 0) | (rows[:, [6, 13]] > 1)):
            raise ValueError('Dual gripper closure must be within 0..1')
        rows = rows.copy()  # The caller may reuse its action array during inference.
        cycle, final = self.cycle, self.stream_final

        def publish(stop_event):
            self._trajectory_running = True
            try:
                if async_exec:
                    origin = self.last_target if self.last_target is not None else self.last_frame['qpos']
                    published, raw, start = rows, None, None
                    continuous = (self.action_fusion.config['enabled']
                                  and self.action_fusion.config['continuous_clock'])
                    if self.action_fusion.config['enabled']:
                        # The previous writer has joined. Align only once, to
                        # the next permitted wire slot, before fusing forecasts.
                        from ..utils.trajectory_utils import resample_remaining
                        start = self.clock.monotonic()
                        if continuous:
                            start = self.reference_clock.next_slot(start)
                        elif self._last_packet_send_s is not None:
                            start = max(start, self._last_packet_send_s + dt)
                        raw = (resample_remaining(rows, max(0., (start-reference_time)/dt))
                               if reference_time is not None else rows)
                        if not len(raw):
                            raise RuntimeError('Prediction expired before fusion; no target sent')
                        self.bridge.check_steps(raw, origin, self.settings['max_joint_step_rad'])
                        published, info = self.action_fusion.fuse(raw, start_s=start, dt=dt)
                        self.log('action_fusion', cycle=cycle, reference_time=reference_time,
                                 raw_targets=raw, fused_targets=published, **info)
                    self.bridge.check_steps(published, origin, self.settings['max_joint_step_rad'])
                    self._send_rows(published, dt, stop_event=stop_event, cycle=cycle,
                                    final=final, start_s=start, raw_rows=raw, continuous_clock=continuous)
                else:
                    self._execute_rows(rows, dt, stop_event=stop_event, cycle=cycle, final=final)
            finally:
                self._trajectory_running = False

        with self._trajectory_submission_lock:
            if self.closed:
                raise RuntimeError('A10 operator is closed')
            if (async_exec and self.action_fusion.config['enabled']
                    and self.action_fusion.config['continuous_clock']
                    and self.action_fusion.config['persistent_sender']):
                return self._submit_stream_plan(dict(rows=rows, cycle=cycle, final=final,
                                                     reference_time=reference_time), dt)
            return self._submit_trajectory(publish, async_exec=async_exec)

    def _submit_stream_plan(self, packet, dt):
        """One latest prediction mailbox; a replacement never stops the writer."""
        self.raise_trajectory_error()
        with self._stream_mailbox_lock:
            if self._stream_accepting:
                self._stream_pending = packet
                return
        # A drained publisher may still be leaving its finalizer. Join it
        # without holding the mailbox lock needed by that finalizer.
        self._cancel_trajectory_worker()
        self.raise_trajectory_error()
        with self._stream_mailbox_lock:
            self._stream_pending = packet
            self._stream_accepting = True

        def stream(stop_event):
            self._trajectory_running = True
            try:
                self.start_follower()
                start = self.reference_clock.next_slot(self.clock.monotonic())
                self.log('stream_publisher_start', thread_id=threading.get_ident(), started_s=start)
                self._send_rows(np.empty((0, 14)), dt, stop_event=stop_event,
                                start_s=start, continuous_clock=True, stream_updates=True)
            finally:
                with self._stream_mailbox_lock:
                    self._stream_accepting = False
                self._trajectory_running = False

        return self._submit_trajectory(stream, async_exec=True)

    def _take_stream_plan(self, start, dt):
        with self._stream_mailbox_lock:
            packet, self._stream_pending = self._stream_pending, None
        if packet is None:
            return None
        from ..utils.trajectory_utils import resample_remaining
        preparing = self.clock.monotonic()
        reference = packet['reference_time']
        raw = (resample_remaining(packet['rows'], max(0., (start-reference)/dt))
               if reference is not None else packet['rows'])
        if not len(raw):
            raise RuntimeError('Prediction expired before stream adoption; no target sent')
        origin = self.last_target if self.last_target is not None else self.last_frame['qpos']
        self.bridge.check_steps(raw, origin, self.settings['max_joint_step_rad'])
        plan, info = self.action_fusion.fuse(raw, start_s=start, dt=dt)
        self.bridge.check_steps(plan, origin, self.settings['max_joint_step_rad'])
        self.log('action_fusion', cycle=packet['cycle'], reference_time=reference,
                 raw_targets=raw, fused_targets=plan,
                 preparation_s=self.clock.monotonic()-preparing, **info)
        return plan, raw, packet['cycle'], packet['final']

    def _execute_rows(self, rows, dt, *, stop_event=None, cycle=None, final=None):
        if self.timed_enabled:
            return self._send_rows(rows, dt, stop_event=stop_event, cycle=cycle, final=final)
        if self.tail_config['enabled']:
            return self._execute_follower_tail(rows, dt)
        return self._send_rows(rows, dt, stop_event=stop_event, cycle=cycle, final=final)

    def _send_rows(self, rows, dt, *, stop_event=None, cycle=None, final=None,
                   start_s=None, raw_rows=None, continuous_clock=False, stream_updates=False):
        n = len(rows)
        if stop_event is None:
            stop_event = self._traj_stop_event
            stop_event.clear()
        cycle = self.cycle if cycle is None else cycle
        final = self.stream_final if final is None else final
        self.start_follower()
        period = dt
        sent = 0
        # Legacy blocks restart local pacing after a gap. Continuous-clock
        # async blocks arrive with a slot on the stream's original grid.
        start = self.clock.monotonic() if start_s is None else start_s
        if start_s is None and self._last_packet_send_s is not None:
            start = max(start, self._last_packet_send_s + period)
        if n:
            self.log('execution_start', cycle=cycle, started_s=start, selected=n)
        index = 0
        while True:
            if stop_event.is_set():
                raise InterruptedError('Dual A10 trajectory cancelled')
            checking = self.clock.monotonic()
            deadline = start + index * period
            not_before = deadline
            if continuous_clock and self._last_packet_send_s is not None:
                # A stalled send must not be followed by a catch-up burst.
                # Leave at least half a period; expired slots below are skipped.
                not_before = max(not_before, self._last_packet_send_s+period/2)
            self._sleep_publishing(max(0., not_before - self.clock.monotonic()), stop_event)
            if stop_event.is_set():
                raise InterruptedError('Dual A10 trajectory cancelled')
            lag_limit = self.settings['max_send_lag_s']
            if self.clock.monotonic() - deadline > lag_limit:
                raise RuntimeError('Joint send deadline missed')
            if stream_updates:
                replacement = self._take_stream_plan(deadline, dt)
                if replacement is not None:
                    if n:
                        self.log('execution_end', cycle=cycle, sent=sent,
                                 finished_s=self.clock.monotonic(), replaced=True,
                                 physical_arrival_confirmed=False)
                    rows, raw_rows, cycle, final = replacement
                    n, start, index, sent = len(rows), deadline, 0, 0
                    self.log('execution_start', cycle=cycle, started_s=start, selected=n,
                             persistent_sender=True)
                if index >= n:
                    # Do not lose an update arriving while the last plan is
                    # draining. Either keep this worker or atomically close
                    # admission so the submitter starts its successor.
                    with self._stream_mailbox_lock:
                        if self._stream_pending is not None:
                            continue
                        self._stream_accepting = False
                    break
            if continuous_clock:
                advanced = min(n-1, max(index, math.floor((self.clock.monotonic()-start)/period+1e-8)))
                if advanced > index:
                    # Skip expired reference slots, never burst or slow the
                    # model clock to drain them. Keep wire indices consecutive.
                    self.log('reference_slots_skipped', cycle=cycle, first_frame=index+1,
                             next_frame=advanced+1, skipped=advanced-index)
                    origin = self.last_target if self.last_target is not None else self.last_frame['qpos']
                    self.bridge.check_steps(rows[advanced:advanced+1], origin,
                                            self.settings['max_joint_step_rad'])
                    index = advanced
                    deadline = start+index*period
            if stop_event.is_set():
                raise InterruptedError('Dual A10 trajectory cancelled')
            if not self.timed_enabled:
                checking = self.clock.monotonic()
                self.devices.check()
            checked = self.clock.monotonic()
            if self.timed_enabled:
                checking = checked  # There was no feedback query in this slot.
            lag = checked - deadline
            if lag > lag_limit:
                self.log('send_deadline_missed', cycle=cycle, frame=index + 1,
                         phase='feedback_check', check_s=checked - checking, lag_s=lag)
                raise RuntimeError('Feedback check exceeded joint send deadline')
            row = rows[index]
            send_started = self.clock.monotonic()
            timing = (self.devices.send(row, final=final and index == n - 1) if self.timed_enabled
                      else self.devices.send(row))
            finished = self.clock.monotonic()
            wire_time = timing.get('udp_send_started_s') if isinstance(timing, dict) else None
            self._last_packet_send_s = finished if wire_time is None else wire_time
            self._last_packet_clock = ('send_finished_fallback' if wire_time is None else 'host_udp_send_started')
            if continuous_clock:
                self.reference_clock.record(deadline)
            if sent == 0:
                if self.last_target is not None:
                    jump = row[JOINTS] - self.last_target[JOINTS]
                    self.log('chunk_boundary', cycle=cycle,
                             previous_target=self.last_target, next_target=row,
                             target_jump_rad=jump,
                             reverses_previous_target_direction=(None if self.last_target_step is None
                                 else jump * self.last_target_step < 0),
                             next_send_started_s=send_started)
                self.trace.finish('next_chunk', send_started)
            self.last_target_step = (None if self.last_target is None
                                     else row[JOINTS] - self.last_target[JOINTS])
            self.last_target = row.copy()
            sent += 1
            self.log('sent', cycle=cycle, frame=index+1, target=row, scheduled_s=deadline,
                     **({'unfused_target': raw_rows[index]} if raw_rows is not None else {}),
                     continuous_clock=continuous_clock,
                     persistent_sender=stream_updates,
                     send_started_s=send_started, send_finished_s=finished,
                     check_s=checked - checking, send_s=finished - send_started,
                     check_lag_s=lag, send_hz=1/dt,
                     point_role=('policy' if self.timed_enabled or self.tail_config['enabled'] else None),
                     policy_frame=(index+1 if self.timed_enabled or self.tail_config['enabled'] else None),
                     repeat_index=(1 if self.timed_enabled or self.tail_config['enabled'] else None),
                     stream_id=timing.get('stream_id') if isinstance(timing, dict) else None,
                     stream_index=timing.get('index') if isinstance(timing, dict) else None,
                     final=timing.get('final') if isinstance(timing, dict) else None,
                     udp_send_started_s=timing.get('udp_send_started_s') if isinstance(timing, dict) else None,
                     udp_send_finished_s=timing.get('udp_send_finished_s') if isinstance(timing, dict) else None)
            if self.timed_enabled and self._last_packet_send_s - deadline > lag_limit:
                raise RuntimeError('timed_aj UDP submission exceeded send deadline; no retry')
            done = index >= n-1
            if done:
                self.trace.arm(cycle, row, finished)
                if not stream_updates or final:
                    if stream_updates:
                        with self._stream_mailbox_lock:
                            self._stream_accepting = False
                    break
            index += 1
        self._sleep_publishing(max(0., start + n * dt - self.clock.monotonic()), stop_event)
        self.log('execution_end', cycle=cycle, sent=sent,
                 finished_s=self.clock.monotonic(), physical_arrival_confirmed=False)

    def _sleep_publishing(self, duration, stop_event):
        if self.clock is time:
            stop_event.wait(duration)
        else:
            self.clock.sleep(duration)

    def finish_timed_stream(self):
        """Only the true end of a run drains final before releasing the Plan."""
        self.wait_trajectory()
        if not self.timed_enabled or not self.attempted or not self.stream_final:
            return
        self._arrival_pending = True
        self._wait_tail_arrival(self.last_target)
        self.stop_trajectory('stream_finished')
        self._arrival_pending = False

    def _execute_follower_tail(self, rows, dt):
        if rows.shape != (20, 14) or not np.isfinite(rows).all():
            raise ValueError('Follower arrival mode requires twenty finite [14] targets')
        if self.mode == 'motion' and not self.timed_enabled and self.config['operator'].get('joint_follower_filter') is not False:
            raise ValueError('Follower arrival mode requires joint_follower_filter=False')
        packets = rows.copy()
        origin = self.last_frame['qpos']
        # Validate the full policy block before motion.
        self.bridge.check_steps(packets, origin, self.settings['max_joint_step_rad'])
        if self.mode == 'motion':
            motors = self.devices.robot.read_motor_config()
            cfg = self.config['operator']
            ids = [cfg['arms'][name]['ee_id'] for name in cfg['joint_target_order']]
            order = [j for ee in (1, 0) for j in range(ids.index(ee) * 6, ids.index(ee) * 6 + 6)]
            bounds = np.array([[motors[j][k] for j in order] for k in ('min_pos', 'max_pos')])
            if (bounds.shape != (2, 12) or not np.isfinite(bounds).all()
                    or np.any(bounds[0] >= bounds[1])):
                raise ValueError('Invalid follower joint limits')
            targets = np.rad2deg(packets[:, JOINTS])
            if np.any(targets < bounds[0]) or np.any(targets > bounds[1]):
                raise ValueError('Follower target exceeds motor position limits; chunk rejected')
        self.log('follower_tail', cycle=self.cycle, policy_points=20, total_packets=len(packets),
                 point_repeat=1, policy_hz=1/dt, packet_hz=1/dt, final_target=rows[-1])
        self._arrival_pending = True
        self._send_rows(packets, dt)
        # Keep follower running until it has consumed the tail and reached it.
        # Stopping earlier can freeze its target at an older queued waypoint.
        self._wait_tail_arrival(rows[-1])
        confirmed = self.devices.stop()
        self.log('stop', cycle=self.cycle, reason='follower_tail_end', confirmed=confirmed)
        if not confirmed:
            raise RuntimeError('Follower stop unconfirmed after tail')
        self.attempted = False
        arrived = self._confirm_tail_stopped(rows[-1])
        # Require new robot and camera samples even when the stopped pose missed the target.
        self._arrival_confirmed_s = self.clock.monotonic()
        self._arrival_pending = False
        self.trace.finish('arrival_confirmed' if arrived else 'post_stop_position_error', self._arrival_confirmed_s)
        self.log('chunk_complete', cycle=self.cycle, sent=len(packets), physical_arrival_confirmed=arrived)

    def _wait_tail_arrival(self, target):
        from ..utils.a10_follower_tail import ArrivalMonitor
        cfg = self.tail_config
        started = self.clock.monotonic()
        monitor = ArrivalMonitor(target[JOINTS], started, cfg)
        command_cfg = dict(cfg, position_tolerance_deg=.3, speed_tolerance_deg_s=.5,
                           stable_duration_s=.04)
        command_monitor = ArrivalMonitor(target[JOINTS], started, command_cfg)
        first_position_stamp = None
        self.log('arrival_wait_start', cycle=self.cycle, target=target, phase='before_stop', **cfg)
        try:
            while self.clock.monotonic() - started < cfg['arrival_timeout_s']:
                if self._traj_stop_event.is_set():
                    raise InterruptedError('Arrival wait cancelled')
                feedback = self.devices.arrival_feedback()
                running = (feedback.get('operation_mode') == 'Manual' and feedback.get('robot_status') == 'Normal'
                           if self.timed_enabled else feedback['follower_active'] is True
                           and feedback.get('follower_mode') == 'Axis')
                if (feedback['has_error'] is not False or not running or feedback['activation'] != 'Enabled'):
                    if self.timed_enabled:
                        raise RuntimeError('Robot health or timed stream invalid during final drain')
                    raise RuntimeError('Robot fault or active Axis follower lost during arrival wait')
                arrived, error, speed = monitor.observe(
                    feedback['joints'], feedback['stamp'], self.clock.monotonic())
                if self.timed_enabled:
                    if not feedback['reference_finished']:
                        monitor = ArrivalMonitor(target[JOINTS], started, cfg)
                        command_monitor = ArrivalMonitor(target[JOINTS], started, command_cfg)
                        arrived = False
                    else:
                        commands_arrived, command_error, command_speed = command_monitor.observe(
                            feedback['command_joints'], feedback['stamp'], self.clock.monotonic())
                        self.log('timed_completion_gate', cycle=self.cycle,
                                 status=feedback['timed_status'], command_error_deg=command_error,
                                 command_speed_deg_s=command_speed, actual_arrived=arrived,
                                 command_arrived=commands_arrived)
                        arrived = arrived and commands_arrived
                self.log('arrival_feedback', cycle=self.cycle, received_monotonic_s=feedback['stamp'],
                         actual_joints_rad=feedback['joints'], max_error_deg=error,
                         max_speed_deg_s=speed, motion=feedback['motion'], phase='before_stop',
                         follower_active=feedback['follower_active'], follower_mode=feedback['follower_mode'])
                if error is not None and error <= cfg['position_tolerance_deg'] and first_position_stamp is None:
                    first_position_stamp = feedback['stamp']
                    self.log('arrival_position_reached', cycle=self.cycle,
                             received_monotonic_s=first_position_stamp, max_error_deg=error,
                             last_packet_sent_s=self._last_packet_send_s,
                             after_last_packet_s=first_position_stamp-self._last_packet_send_s,
                             send_clock=self._last_packet_clock,
                             position_tolerance_deg=cfg['position_tolerance_deg'])
                if arrived:
                    self.log('target_arrival_confirmed', cycle=self.cycle,
                             elapsed_s=self.clock.monotonic() - started, max_error_deg=error,
                             max_speed_deg_s=speed, physical_arrival_confirmed=True,
                             follower_active=feedback['follower_active'],
                             last_packet_sent_s=self._last_packet_send_s,
                             after_last_packet_s=feedback['stamp']-self._last_packet_send_s,
                             extra_wait_after_position_s=feedback['stamp']-first_position_stamp)
                    return
                self.clock.sleep(1 / cfg['poll_hz'])
            raise TimeoutError('Final policy target arrival not confirmed; next observation blocked')
        except BaseException as exc:
            self.log('arrival_failed', cycle=self.cycle, error=str(exc))
            raise

    def _confirm_tail_stopped(self, target):
        """Require a confirmed stop and report whether the stopped pose is in tolerance."""
        cfg = self.tail_config
        started = self.clock.monotonic()
        while self.clock.monotonic() - started < cfg['arrival_timeout_s']:
            if self._traj_stop_event.is_set():
                raise InterruptedError('Arrival stop confirmation cancelled')
            # Query immediately; only wait another period if the sample is not usable.
            feedback = self.devices.arrival_feedback()
            inactive = (feedback['timed_status']['active'] is False if self.timed_enabled
                        else feedback['follower_active'] is False)
            if (feedback['has_error'] is not False or not inactive
                    or feedback['activation'] != 'Enabled'):
                raise RuntimeError('Robot fault or follower reactivated after arrival stop')
            stamp, now = feedback['stamp'], self.clock.monotonic()
            actual = np.asarray(feedback['joints'], dtype=float)
            if (actual.shape != (12,) or not np.isfinite(actual).all() or not np.isfinite(stamp)
                    or not 0 <= now-stamp <= cfg['max_feedback_gap_s']):
                raise RuntimeError('Invalid/stale post-stop arrival feedback')
            if stamp <= started:
                self.clock.sleep(1 / cfg['poll_hz'])
                continue
            error = float(np.rad2deg(np.abs(actual-target[JOINTS])).max())
            self.log('arrival_feedback', cycle=self.cycle, received_monotonic_s=stamp,
                     actual_joints_rad=actual, max_error_deg=error, max_speed_deg_s=None,
                     motion=feedback['motion'], phase='after_stop', follower_active=False)
            if feedback['motion'] == 'Stopped':
                tolerance = cfg['position_tolerance_deg']
                arrived = error <= tolerance
                self.log('arrival_confirmed' if arrived else 'post_stop_position_error',
                         cycle=self.cycle, elapsed_s=now-started, max_error_deg=error,
                         position_tolerance_deg=tolerance, physical_arrival_confirmed=arrived,
                         follower_active=False, phase='after_stop',
                         after_last_packet_s=stamp-self._last_packet_send_s)
                if not arrived:
                    if self.timed_enabled:
                        raise RuntimeError('Actual endpoint moved outside tolerance after timed_aj stop')
                    logging.getLogger(__name__).warning(
                        'A10 推理第 %d 轮停稳后最大关节误差 %.3f°，超过 %.3f°，按实测位姿继续推理。',
                        self.cycle + 1, error, tolerance)
                return arrived
            self.clock.sleep(1 / cfg['poll_hz'])
        raise TimeoutError('Post-stop Stopped state not confirmed; next observation blocked')





    @contextmanager
    def inference_feedback(self):
        """Optional measurement on the shared state connection; never sends targets."""
        enabled = self.settings.get('trace_inference_feedback', False)
        reader = getattr(self.devices, '_robot_state', None)
        if not enabled or reader is None:
            yield
            return
        stop, failures = threading.Event(), []

        def sample():
            try:
                while not stop.is_set():
                    started = time.monotonic()
                    reader()
                    # Query starts target 100Hz. Discard missed sampling slots;
                    # never queue GETs or add a full period after a slow reply.
                    stop.wait(max(0., .01 - (time.monotonic() - started)))
            except Exception as exc:
                failures.append(exc)
                stop.set()

        worker = threading.Thread(target=sample, name='a10-inference-measurement', daemon=True)
        worker.start()
        try:
            yield
        finally:
            stop.set()
            worker.join(self.devices.robot.timeout_s + 2.)
            if worker.is_alive():
                raise TimeoutError('Diagnostic state sampler did not finish; no new targets permitted')
        if failures:
            raise ConnectionError('Diagnostic feedback query failed') from failures[0]

    def stop_trajectory(self, reason='run_exit'):
        self._traj_stop_event.set()
        with self._trajectory_submission_lock:
            self._cancel_trajectory_worker()
            with self._lifecycle_lock:
                result = self._stop_trajectory(reason)
                self._trajectory_error = None
                return result

    def _on_trajectory_error(self, error):
        # The worker must halt hardware even while the model RPC is blocked.
        # Do not acquire the submission lock: another thread may be joining us.
        with self._lifecycle_lock:
            try:
                self._stop_trajectory('publisher_error')
            except BaseException as stop_error:
                self.log('publisher_stop_failed', error=str(error), stop_error=str(stop_error))

    def _stop_trajectory(self, reason):
        self._traj_stop_event.set()
        with self._stream_mailbox_lock:
            # Discard unpublished work even if the physical stop cannot be
            # confirmed. A failed stop must never leave a replayable mailbox.
            self._stream_pending = None
            self._stream_accepting = False
        self.trace.finish(reason, self.clock.monotonic())
        if self.attempted:
            try:
                confirmed = self.devices.stop()
            except BaseException as exc:
                self.log('stop', cycle=self.cycle, reason=reason, confirmed=False, error=str(exc))
                raise
            self.log('stop', cycle=self.cycle, reason=reason, confirmed=confirmed)
            if not confirmed:
                raise RuntimeError('Follower stop unconfirmed')
            self.attempted = False
            if self.timed_enabled:
                self.last_target = self.last_target_step = None
        self.action_fusion.reset()
        self.reference_clock.reset()

    def is_trajectory_running(self):
        return getattr(self, '_trajectory_running', False)

    def close(self):
        self._traj_stop_event.set()
        with self._trajectory_submission_lock:
            if self.closed:
                return
            self.closed = True
            try:
                self.stop_trajectory()
            finally:
                if self.devices is not None:
                    self.devices.close()
