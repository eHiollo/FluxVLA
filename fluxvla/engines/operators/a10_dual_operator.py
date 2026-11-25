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
        self._trajectory_lock = threading.Lock()
        self.cycle = 0
        self.attempted = self.closed = False
        self.trace = TailExecutionTrace(self.log)
        self.last_frame = None
        self._arrival_pending = False
        self._arrival_confirmed_s = None
        self.last_target = self.last_target_step = None
        self._last_packet_send_s = None
        self._last_packet_clock = None
        from ..utils.a10_follower_tail import tail_config
        self.tail_config = tail_config(self.settings.get('follower_tail'))
        self._lifecycle_lock = threading.RLock()
        if devices is not None:
            self._attach_feedback()

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
            self.devices.start()

    def execute_trajectory(self, arm_trajectories, gripper_trajectories=None,
                           head_trajectory=None, dt=1 / 30, async_exec=False):
        if async_exec is not False or head_trajectory is not None:
            raise ValueError('Dual A10 baseline requires synchronous arm execution without a head')
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
        if not self._trajectory_lock.acquire(blocking=False):
            raise RuntimeError('A dual A10 trajectory is already running')
        self._trajectory_running = True
        try:
            self._execute_rows(rows, dt)
        finally:
            self._trajectory_running = False
            self._trajectory_lock.release()

    def _execute_rows(self, rows, dt):
        if self.tail_config['enabled']:
            return self._execute_follower_tail(rows, dt)
        return self._send_rows(rows, dt)

    def _send_rows(self, rows, dt):
        n = len(rows)
        repeat = 2
        self._traj_stop_event.clear()
        self.start_follower()
        period = dt
        sent = 0
        start = self.clock.monotonic()
        self.log('execution_start', cycle=self.cycle, started_s=start, selected=n)
        slot = 0
        while True:
            if self._traj_stop_event.is_set():
                raise InterruptedError('Dual A10 trajectory cancelled')
            deadline = start + slot * period
            self.clock.sleep(max(0., deadline - self.clock.monotonic()))
            if self.clock.monotonic() - deadline > self.settings['max_send_lag_s']:
                raise RuntimeError('Joint send deadline missed')
            checking = self.clock.monotonic()
            self.devices.check()
            checked = self.clock.monotonic()
            lag = checked - deadline
            if lag > self.settings['max_send_lag_s']:
                self.log('send_deadline_missed', cycle=self.cycle, frame=sent + 1,
                         phase='feedback_check', check_s=checked - checking, lag_s=lag)
                raise RuntimeError('Feedback check exceeded joint send deadline')
            row = rows[sent]
            send_started = self.clock.monotonic()
            timing = self.devices.send(row)
            finished = self.clock.monotonic()
            wire_time = timing.get('udp_send_started_s') if isinstance(timing, dict) else None
            self._last_packet_send_s = finished if wire_time is None else wire_time
            self._last_packet_clock = ('send_finished_fallback' if wire_time is None else 'host_udp_send_started')
            if sent == 0:
                if self.last_target is not None:
                    jump = row[JOINTS] - self.last_target[JOINTS]
                    self.log('chunk_boundary', cycle=self.cycle,
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
            self.log('sent', cycle=self.cycle, frame=sent, target=row, scheduled_s=deadline,
                     send_started_s=send_started, send_finished_s=finished,
                     check_s=checked - checking, send_s=finished - send_started,
                     check_lag_s=lag, send_hz=1/dt,
                     point_role=('policy' if self.tail_config['enabled'] else None),
                     policy_frame=(min(20, (sent-1)//repeat+1)
                                   if self.tail_config['enabled'] else None),
                     repeat_index=((sent-1)%repeat+1
                                   if self.tail_config['enabled'] else None),
                     udp_send_started_s=timing.get('udp_send_started_s') if isinstance(timing, dict) else None,
                     udp_send_finished_s=timing.get('udp_send_finished_s') if isinstance(timing, dict) else None)
            done = sent >= n
            if done:
                self.trace.arm(self.cycle, row, finished)
                break
            slot = sent
        self.clock.sleep(max(0., start + n * dt - self.clock.monotonic()))
        self.log('execution_end', cycle=self.cycle, sent=sent,
                 finished_s=self.clock.monotonic(), physical_arrival_confirmed=False)

    def _execute_follower_tail(self, rows, dt):
        if rows.shape != (20, 14) or not np.isfinite(rows).all():
            raise ValueError('Follower arrival mode requires twenty finite [14] targets')
        if self.mode == 'motion' and self.config['operator'].get('joint_follower_filter') is not False:
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
        repeat = 2
        packets = np.repeat(packets, repeat, axis=0)
        self.log('follower_tail', cycle=self.cycle, policy_points=20, total_packets=len(packets),
                 point_repeat=repeat, policy_hz=1/dt, packet_hz=repeat/dt, final_target=rows[-1])
        self._arrival_pending = True
        self._send_rows(packets, dt/repeat)
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
        first_position_stamp = None
        self.log('arrival_wait_start', cycle=self.cycle, target=target, phase='before_stop', **cfg)
        try:
            while self.clock.monotonic() - started < cfg['arrival_timeout_s']:
                if self._traj_stop_event.is_set():
                    raise InterruptedError('Arrival wait cancelled')
                feedback = self.devices.arrival_feedback()
                if (feedback['has_error'] is not False or feedback['follower_active'] is not True
                        or feedback.get('follower_mode') != 'Axis' or feedback['activation'] != 'Enabled'):
                    raise RuntimeError('Robot fault or active Axis follower lost during arrival wait')
                arrived, error, speed = monitor.observe(
                    feedback['joints'], feedback['stamp'], self.clock.monotonic())
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
                             max_speed_deg_s=speed, physical_arrival_confirmed=True, follower_active=True,
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
            if (feedback['has_error'] is not False or feedback['follower_active'] is not False
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
                    logging.getLogger(__name__).warning(
                        'A10 推理第 %d 轮停稳后最大关节误差 %.3f°，超过 %.3f°，按实测位姿继续推理。',
                        self.cycle + 1, error, tolerance)
                return arrived
            self.clock.sleep(1 / cfg['poll_hz'])
        raise TimeoutError('Post-stop Stopped state not confirmed; next observation blocked')





    @contextmanager
    def inference_feedback(self):
        """Optional measurement only: no reads overlap the target sending loop."""
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
        with self._lifecycle_lock:
            return self._stop_trajectory(reason)

    def _stop_trajectory(self, reason):
        self._traj_stop_event.set()
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

    def is_trajectory_running(self):
        return getattr(self, '_trajectory_running', False)

    def close(self):
        with self._lifecycle_lock:
            if self.closed:
                return
            self.closed = True
            try:
                self.stop_trajectory()
            finally:
                if self.devices is not None:
                    self.devices.close()
