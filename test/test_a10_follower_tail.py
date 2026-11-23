"""Follower tail execution and arrival-before-stop gates; no real hardware."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from test_a10_dual_inference import b, Clock, Device, Policy, SETTINGS, ROOT
from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner
from fluxvla.engines.utils.a10_follower_tail import (
    ArrivalMonitor, JOINTS, tail_config)


class TailMathTests(unittest.TestCase):


    def test_bad_config_rejected(self):
        for cfg in (dict(enabled='yes'), dict(poll_hz=0), dict(stable_duration_s=-1),
                    dict(arrival_timeout_s=float('nan')), dict(window=9),
                    dict(point_repeat=True), dict(point_repeat=3), dict(point_repeat=2),
                    dict(point_stride=True), dict(point_stride=3), dict(point_stride=2),
                    dict( point_stride=2, point_repeat=2)):
            with self.assertRaises(ValueError):
                tail_config(cfg)


class ArrivalTests(unittest.TestCase):
    def monitor(self):
        return ArrivalMonitor(np.zeros(12), 1., tail_config())

    def test_position_alone_does_not_confirm_moving_robot(self):
        m = self.monitor()
        for i in range(20):
            q = np.zeros(12); q[0] = np.deg2rad(.1 if i % 2 else -.1)
            self.assertFalse(m.observe(q, 1. + i/30, 1. + i/30)[0])

    def test_requires_stable_actual_joints_after_stop(self):
        m = self.monitor()
        for i in range(10):
            self.assertFalse(m.observe(np.zeros(12), 1. + i/30, 1. + i/30)[0])
        self.assertFalse(m.observe(np.zeros(12), 1.5, 1.5)[0])  # long gap resets
        for i in range(1, 12):
            result = m.observe(np.zeros(12), 1.5 + i/30, 1.5 + i/30)
        self.assertTrue(result[0])

    def test_position_only_confirms_first_fresh_sample_without_speed_or_dwell(self):
        cfg=tail_config(dict(speed_tolerance_deg_s=None,stable_duration_s=0))
        monitor=ArrivalMonitor(np.zeros(12),1.,cfg)
        arrived,error,speed=monitor.observe(np.full(12,np.deg2rad(.1)),1.,1.)
        self.assertTrue(arrived)
        self.assertIsNone(speed)
        self.assertAlmostEqual(error,.1)
        # A fast-moving sample within position tolerance also needs no dwell.
        self.assertTrue(monitor.observe(np.zeros(12),1.01,1.01)[0])

    def test_zero_dwell_can_still_keep_optional_speed_limit(self):
        monitor=ArrivalMonitor(np.zeros(12),1.,tail_config(dict(stable_duration_s=0)))
        self.assertFalse(monitor.observe(np.zeros(12),1.,1.)[0])
        self.assertTrue(monitor.observe(np.zeros(12),1.03,1.03)[0])

    def test_duplicate_stale_backwards_and_before_stop_samples(self):
        m = self.monitor()
        m.observe(np.zeros(12), 1., 1.)
        for _ in range(20):
            self.assertFalse(m.observe(np.zeros(12), 1., 1.01)[0])
        for stamp, now in ((1., 1.3), (.9, .9), (float('nan'), 1.)):
            with self.assertRaises(RuntimeError):
                m.observe(np.zeros(12), stamp, now)
        m.observe(np.zeros(12), 1.05, 1.05)
        with self.assertRaisesRegex(RuntimeError, 'backwards'):
            m.observe(np.zeros(12), 1.04, 1.05)


class TailDevice(Device):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.packets, self.observations, self.stops_at = [], [], []
        self.arrival_delay = .2
        self.fail_arrival = False
        self.stale_arrival = False
        self.feedback_calls = 0
        self.last_send_at = None
        self.pre_stop_samples = []
        self.post_stop_samples = []

    def sample(self):
        sample = super().sample()
        self.observations.append(self.clock.monotonic())
        return sample

    def send(self, row):
        self.packets.append(row.copy())
        self.last_send_at = self.clock.monotonic()
        super().send(row)

    def stop(self):
        self.stops_at.append(self.clock.monotonic())
        return super().stop()

    def arrival_feedback(self):
        self.feedback_calls += 1
        result = super().arrival_feedback()
        result['motion'] = 'Running' if self.active else 'Stopped'
        if self.fail_arrival or self.clock.now - self.last_send_at < self.arrival_delay:
            result['joints'] += .01
        if self.stale_arrival:
            result['stamp'] -= 1
        (self.pre_stop_samples if self.active else self.post_stop_samples).append(
            (self.clock.monotonic(),result['joints'].copy()))
        return result


class TailExecutionTests(unittest.TestCase):
    def setup_runner(self, **overrides):
        clock = Clock()
        d = TailDevice(clock)
        settings = dict(SETTINGS, execute_horizon=20, follower_tail=dict(enabled=True, arrival_timeout_s=1.5))
        settings.update(overrides)
        runner = A10DualInferenceRunner.from_components(
            d, Policy(), settings, Mock(), cycles=2, clock=clock,
            bridge=SimpleNamespace(**vars(b)))
        return clock, d, runner

    def test_two_chunks_send_exactly_40_each_and_observe_only_after_arrival(self):
        clock, d, r = self.setup_runner()
        with patch.object(b, 'time', clock):
            r.run()
        self.assertEqual(len(d.packets), 80)
        self.assertEqual(len(d.stops_at), 2)
        self.assertGreaterEqual(d.feedback_calls, 30)
        for c in (0, 1):
            np.testing.assert_array_equal(d.packets[c*40+39], d.packets[c*40+38])
        # Two observations per cycle (policy input and pre-send check), none while waiting.
        self.assertEqual(len(d.observations), 4)
        self.assertGreater(d.observations[2], d.stops_at[0])
        events = [c.args[0] for c in r.log.call_args_list]
        self.assertEqual(events.count('arrival_confirmed'), 2)
        self.assertEqual(events.count('hold_sent'), 0)
        self.assertFalse(r.ros_operator._arrival_pending)
        confirmed = [c for c in r.log.call_args_list if c.args[0]=='target_arrival_confirmed']
        self.assertEqual(len(confirmed),2)
        for c in confirmed:
            self.assertGreaterEqual(c.kwargs['elapsed_s'],.3)
        events=[c.args[0] for c in r.log.call_args_list]
        for cycle in (0,1):
            subset=[c.args[0] for c in r.log.call_args_list if c.kwargs.get('cycle')==cycle]
            self.assertLess(subset.index('target_arrival_confirmed'),subset.index('stop'))
            self.assertLess(subset.index('stop'),subset.index('arrival_confirmed'))
        # No extra 0.3s stability dwell is imposed after the stop confirmation.
        self.assertLess(d.observations[2]-d.stops_at[0],.1)

    def test_unfiltered_mode_sends_only_policy_points_then_waits_for_arrival(self):
        clock, d, r = self.setup_runner(follower_tail=dict(
            enabled=True,  arrival_timeout_s=1.5))
        with patch.object(b, 'time', clock):
            r.run()
        self.assertEqual(len(d.packets), 80)
        self.assertEqual(len(d.stops_at), 2)
        self.assertEqual(len(d.observations), 4)
        self.assertGreater(d.observations[2], d.stops_at[0])
        for call in r.log.call_args_list:
            if call.args[0] == 'sent':
                self.assertEqual(call.kwargs['point_role'], 'policy')
            if call.args[0] == 'follower_tail':
                self.assertEqual(call.kwargs['total_packets'], 40)
                self.assertNotIn('compensation', call.kwargs)
        self.assertFalse(r.ros_operator._arrival_pending)

    def test_twice_repeated_policy_keeps_duration_and_arrival_before_stop_gate(self):
        clock, d, r = self.setup_runner(follower_tail=dict(
            enabled=True,  arrival_timeout_s=1.5))
        with patch.object(b, 'time', clock):
            r.run()
        self.assertEqual(len(d.packets), 80)
        self.assertEqual(len(d.stops_at), 2)
        for i in range(0, 80, 2):
            np.testing.assert_array_equal(d.packets[i], d.packets[i+1])
        calls = r.log.call_args_list
        starts = [x.kwargs['started_s'] for x in calls if x.args[0]=='execution_start']
        ends = [x.kwargs['finished_s'] for x in calls if x.args[0]=='execution_end']
        for start, end in zip(starts, ends):
            self.assertAlmostEqual(end-start, 20/30)
        sent = [x.kwargs for x in calls if x.args[0]=='sent']
        self.assertEqual([x['policy_frame'] for x in sent[:40]], [i for i in range(1,21) for _ in range(2)])
        self.assertEqual([x['repeat_index'] for x in sent[:40]], [1,2]*20)
        self.assertGreater(d.observations[2],d.stops_at[0])

    def test_unfiltered_timeout_still_blocks_next_observation(self):
        clock, d, r = self.setup_runner(follower_tail=dict(
            enabled=True,  arrival_timeout_s=1.5))
        d.fail_arrival = True
        with patch.object(b, 'time', clock), self.assertRaises(TimeoutError):
            r.run()
        self.assertEqual(len(d.packets), 40)
        self.assertEqual(len(d.observations), 2)
        with self.assertRaisesRegex(RuntimeError, 'observation blocked'):
            r.ros_operator.get_frame()

    def test_timeout_blocks_next_observation_and_prediction(self):
        clock, d, r = self.setup_runner()
        d.fail_arrival = True
        with patch.object(b, 'time', clock), self.assertRaisesRegex(TimeoutError, 'next observation blocked'):
            r.run()
        self.assertEqual(len(d.packets), 40)
        self.assertEqual(len(d.observations), 2)
        with self.assertRaisesRegex(RuntimeError, 'observation blocked'):
            r.ros_operator.get_frame()
        self.assertTrue(d.closed)
        self.assertEqual(len(d.stops_at),1)
        self.assertGreaterEqual(d.stops_at[0],20/30+1.5)
        self.assertFalse(d.active)

    def test_stale_arrival_fails_closed_without_a_new_observation(self):
        clock, d, r = self.setup_runner()
        d.stale_arrival = True
        with patch.object(b, 'time', clock), self.assertRaisesRegex(RuntimeError, 'stale arrival'):
            r.run()
        self.assertEqual(len(d.observations), 2)
        self.assertEqual(d.sent, 40)


    def test_motor_bounds_rejected_before_start_and_preserve_physical_mapping(self):
        clock, d, r = self.setup_runner()
        op = r.ros_operator
        op.mode = 'motion'
        op.config = dict(operator=dict(joint_follower_filter=False, joint_target_order=['zero', 'one'],
                                      arms=dict(zero=dict(ee_id=0), one=dict(ee_id=1))))
        d.robot = Mock()
        motors = [dict(min_pos=-180., max_pos=180.) for _ in range(12)]
        # Physical policy left is ee_id=1, controller motor indices 6..11.
        motors[6] = dict(min_pos=-1., max_pos=1.)
        d.robot.read_motor_config.return_value = motors
        with patch.object(b, 'time', clock), self.assertRaisesRegex(ValueError, 'position limits'):
            r.run()
        self.assertEqual(d.sent, 0)
        d.robot.start_follower.assert_not_called()

    def test_unconfirmed_stop_keeps_observation_blocked(self):
        clock, d, r = self.setup_runner()
        d.stop = Mock(return_value=False)
        with patch.object(b, 'time', clock), self.assertRaisesRegex(RuntimeError, 'stop unconfirmed'):
            r._run_action_chunk('fold')
        self.assertEqual(d.sent, 40)
        with self.assertRaisesRegex(RuntimeError, 'observation blocked'):
            r.ros_operator.get_frame()
        d.stop = lambda: True
        r.cleanup()

    def test_stop_during_wait_cancels_without_new_observation(self):
        clock, d, r = self.setup_runner()
        original = d.arrival_feedback
        def feedback():
            r.ros_operator._traj_stop_event.set()
            return original()
        d.arrival_feedback = feedback
        with patch.object(b, 'time', clock), self.assertRaisesRegex(InterruptedError, 'cancelled'):
            r.run()
        self.assertEqual(d.sent, 40)
        self.assertEqual(len(d.observations), 2)

    def test_post_arrival_observation_rejects_old_buffered_camera_frames(self):
        clock, d, r = self.setup_runner()
        with patch.object(b, 'time', clock):
            r._run_action_chunk('fold')
            confirmed = r.ros_operator._arrival_confirmed_s
            sample = d.sample
            snapshots = []
            def delayed_camera():
                data = sample()
                snapshots.append(clock.now)
                data['camera_times']['base_0_rgb'] = confirmed - .01 if len(snapshots) <= 2 else clock.now
                return data
            d.sample = delayed_camera
            frame = r.ros_operator.get_frame()
            self.assertEqual(len(snapshots), 3)
            self.assertGreaterEqual(frame['camera_times']['base_0_rgb'], confirmed)
        r.cleanup()

    def test_stop_is_never_called_until_active_target_is_stable(self):
        clock,d,r=self.setup_runner()
        original_stop=d.stop
        def stop_after_arrival():
            # During active wait the fake reports motion=Running. Stability,
            # rather than that flag, must allow the normal stop to happen.
            self.assertGreater(len(d.pre_stop_samples),10)
            last_time,last_q=d.pre_stop_samples[-1]
            np.testing.assert_allclose(last_q,r.ros_operator.last_target[JOINTS])
            self.assertGreaterEqual(last_time-d.last_send_at,d.arrival_delay+.3)
            return original_stop()
        d.stop=stop_after_arrival
        with patch.object(b,'time',clock):
            r._run_action_chunk('fold')
        self.assertEqual(len(d.stops_at),1)
        self.assertGreater(r.ros_operator._arrival_confirmed_s,d.stops_at[0])
        r.cleanup()

    def test_position_only_stops_without_extra_stability_wait_and_logs_latency(self):
        clock,d,r=self.setup_runner(follower_tail=dict(enabled=True,
            position_tolerance_deg=.5,speed_tolerance_deg_s=None,
            stable_duration_s=0,arrival_timeout_s=1.5))
        with patch.object(b,'time',clock):
            r._run_action_chunk('fold')
        self.assertEqual(len(d.packets),40)
        self.assertGreaterEqual(d.stops_at[0]-d.last_send_at,d.arrival_delay)
        self.assertLessEqual(d.stops_at[0]-d.last_send_at,d.arrival_delay+2/30)
        records=[call for call in r.log.call_args_list if call.args[0]=='target_arrival_confirmed']
        self.assertEqual(len(records),1)
        self.assertEqual(records[0].kwargs['extra_wait_after_position_s'],0)
        self.assertGreaterEqual(records[0].kwargs['after_last_packet_s'],d.arrival_delay)
        self.assertFalse(r.ros_operator._arrival_pending)
        r.cleanup()

    def test_axis_mode_loss_stops_during_cleanup_and_blocks_observation(self):
        clock,d,r=self.setup_runner()
        original=d.arrival_feedback
        def wrong_mode():
            feedback=original()
            feedback['follower_mode']='Cart'
            return feedback
        d.arrival_feedback=wrong_mode
        with patch.object(b,'time',clock),self.assertRaisesRegex(RuntimeError,'Axis follower lost'):
            r.run()
        self.assertEqual(len(d.observations),2)
        self.assertEqual(len(d.stops_at),1)
        self.assertFalse(d.active)
        self.assertTrue(r.ros_operator._arrival_pending)

    def test_post_stop_drift_warns_and_next_chunk_uses_measured_pose(self):
        clock, d, r = self.setup_runner()
        original_stop = d.stop

        def stop_with_drift():
            confirmed = original_stop()
            d.joints[1][0] += .01
            return confirmed

        d.stop = stop_with_drift
        with patch.object(b, 'time', clock), self.assertLogs(level='WARNING'):
            r.run()
        self.assertEqual(len(d.packets), 80)
        self.assertEqual(len(d.stops_at), 2)
        self.assertEqual(len(d.observations), 4)
        self.assertGreater(d.observations[2], d.stops_at[0])
        self.assertAlmostEqual(d.packets[40][0], d.packets[39][0] + .01)
        warnings = [c.kwargs for c in r.log.call_args_list if c.args[0] == 'post_stop_position_error']
        self.assertEqual(len(warnings), 2)
        self.assertGreater(warnings[0]['max_error_deg'], warnings[0]['position_tolerance_deg'])
        for result in [c.kwargs for c in r.log.call_args_list if c.args[0] == 'chunk_complete']:
            self.assertFalse(result['physical_arrival_confirmed'])
        self.assertFalse(r.ros_operator._arrival_pending)

    def test_post_stop_motion_still_blocks_next_observation(self):
        clock, d, r = self.setup_runner()
        original_feedback = d.arrival_feedback

        def still_moving():
            feedback = original_feedback()
            feedback['motion'] = 'Running'
            return feedback

        d.arrival_feedback = still_moving
        with patch.object(b, 'time', clock), self.assertRaisesRegex(TimeoutError, 'Stopped state not confirmed'):
            r.run()
        self.assertEqual(len(d.packets), 40)
        self.assertEqual(len(d.observations), 2)
        self.assertTrue(r.ros_operator._arrival_pending)

    def test_post_stop_cached_sample_does_not_release_observation(self):
        clock,d,r=self.setup_runner()
        original=d.arrival_feedback
        cached=0
        def stop_cache():
            nonlocal cached
            feedback=original()
            if not d.active:
                cached+=1
                if cached<=2:
                    feedback['stamp']=d.stops_at[-1]
            return feedback
        d.arrival_feedback=stop_cache
        with patch.object(b,'time',clock):
            r._run_action_chunk('fold')
        self.assertEqual(cached,3)
        self.assertGreaterEqual(r.ros_operator._arrival_confirmed_s,d.stops_at[-1]+2/30-1e-9)
        r.cleanup()

    def test_conflicting_modes_and_wrong_horizon_rejected(self):
        for settings in (dict(execute_horizon=10),):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                _, _, r = self.setup_runner(**settings)
                r.run_setup()

    def test_config_disables_filter_and_compensation(self):
        from mmengine import Config
        cfg = Config.fromfile(ROOT / 'configs/a10/a10_dual_openpi_remote.py')
        self.assertNotIn('trajectory_planner', cfg.inference)
        self.assertTrue(cfg.inference.follower_tail.enabled)
        self.assertNotIn('mean_compensation', cfg.inference.follower_tail)
        self.assertNotIn('point_repeat', cfg.inference.follower_tail)
        self.assertNotIn('point_stride', cfg.inference.follower_tail)
        self.assertEqual(cfg.inference.mode, 'motion')
        self.assertEqual(cfg.inference.cycles, 1_000_000)
        self.assertEqual(cfg.inference.remote_inference.timeout_s, 3600)
        self.assertEqual(cfg.operator.joint_follower_velocity_percent, 12)
        self.assertEqual(cfg.operator.motor_acceleration_deg_s2, 375)
        self.assertEqual(cfg.grippers.left.drop_alarm_policy, 'warn')
        self.assertEqual(cfg.grippers.right.drop_alarm_policy, 'warn')
        self.assertEqual(cfg.inference.follower_tail.stable_duration_s, 0)
        self.assertIsNone(cfg.inference.follower_tail.speed_tolerance_deg_s)
        self.assertIs(cfg.operator.joint_follower_filter, False)
        self.assertEqual(cfg.inference.execute_horizon, 20)


if __name__ == '__main__':
    unittest.main()
