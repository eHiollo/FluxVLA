"""Same-time forecast fusion and integration with the real background writer."""
import contextlib
import io
import time
import unittest
from unittest.mock import Mock

import numpy as np

from fluxvla.engines.utils.action_fusion import ExponentialActionFusion, fusion_config
from fluxvla.engines.runners.a10_inference_runner import A10DualInferenceRunner
from test_a10_async_execution import RecordingDevice, operator, state
from test_a10_dual_inference import SETTINGS


class FusionTests(unittest.TestCase):
    def test_invalid_continuous_clock_config_rejected(self):
        with self.assertRaisesRegex(ValueError, 'continuous_clock'):
            fusion_config(dict(continuous_clock=1))

    def test_gripper_fusion_joins_same_time_apertures_and_leaves_arms_identical(self):
        config = dict(enabled=True, method='velocity', alpha=.25,
                      handover_steps=6, tail_steps=12)
        baseline = ExponentialActionFusion(config)
        corrected = ExponentialActionFusion(dict(config, gripper_alpha=.35))
        for step in range(8):
            now = step*.25
            raw = np.tile((np.arange(30)/30+now)[:, None]*.03, (1, 14))
            raw[:, 6] = .1 if step%2 else .9
            raw[:, 13] = 1-raw[:, 6]
            old = corrected.plan.copy() if corrected.plan is not None else None
            old_start = corrected.start_s
            before, _ = baseline.fuse(raw, start_s=now, dt=1/30)
            result, info = corrected.fuse(raw, start_s=now, dt=1/30)
            np.testing.assert_array_equal(result[:, corrected.joint_columns],
                                          before[:, corrected.joint_columns])
            self.assertTrue(np.all((result[:, [6, 13]] >= 0) & (result[:, [6, 13]] <= 1)))
            np.testing.assert_array_equal(result[-1, [6, 13]], raw[-1, [6, 13]])
            if old is not None:
                np.testing.assert_allclose(result[0, [6, 13]],
                    corrected._sample(old, (now-old_start)*30)[[6, 13]])
                self.assertTrue(info['gripper_fusion_applied'])
        corrected.reset()
        fresh, info = corrected.fuse(raw, start_s=10., dt=1/30)
        np.testing.assert_array_equal(fresh, raw)
        self.assertFalse(info['gripper_fusion_applied'])
        fresh, info = corrected.fuse(raw[:2], start_s=12., dt=1/30)
        np.testing.assert_array_equal(fresh, raw[:2])
        self.assertTrue(info['history_expired'])

    def test_invalid_gripper_fusion_weights_rejected(self):
        for alpha in (0, 1.1, True, '0.3', float('nan')):
            with self.subTest(alpha=alpha), self.assertRaisesRegex(ValueError, 'gripper_alpha'):
                fusion_config(dict(gripper_alpha=alpha))

    def fuser(self, alpha=.25, tail=4):
        return ExponentialActionFusion(dict(enabled=True, alpha=alpha, tail_steps=tail))

    def velocity_fuser(self):
        return ExponentialActionFusion(dict(enabled=True, method='velocity', alpha=.25,
                                            handover_steps=6, tail_steps=12))

    def test_velocity_fusion_same_linear_motion_is_unchanged_at_fractional_times(self):
        f = self.velocity_fuser()
        t = np.arange(30)/30
        first = np.tile(t[:, None]*.3, (1, 14))
        f.fuse(first, start_s=0., dt=1/30)
        latest = np.tile((t+.237)[:, None]*.3, (1, 14))
        latest[:, [6, 13]] = .7
        out, _ = f.fuse(latest, start_s=.237, dt=1/30)
        np.testing.assert_allclose(out, latest, atol=1e-12)

    def test_velocity_fusion_avoids_offset_induced_reversal_without_losing_endpoint(self):
        t = np.arange(23)/30
        # Steady state has about 23 remaining points after inference; the
        # next update overlaps about 14, so the old 12-point tail starts early.
        old = np.tile((np.arange(23)/30*.5)[:, None], (1, 14))
        new = np.tile(((t+.27)*.5-.16)[:, None], (1, 14))
        new[:, [6, 13]] = .7
        position = ExponentialActionFusion(dict(enabled=True, alpha=.25, tail_steps=12, handover_steps=6))
        velocity = self.velocity_fuser()
        for f in (position, velocity):
            f.fuse(old, start_s=0., dt=1/30)
        before, _ = position.fuse(new, start_s=.27, dt=1/30)
        after, info = velocity.fuse(new, start_s=.27, dt=1/30)
        before_v, after_v = np.diff(before[:8, 0])*30, np.diff(after[:8, 0])*30
        self.assertLess(before_v.min(), 0.)  # Position cross-fade reverses two forward forecasts.
        self.assertGreater(after_v.min(), 0.)
        self.assertAlmostEqual(after[0, 0], .135)
        np.testing.assert_array_equal(after[-1], new[-1])
        self.assertLessEqual(info['max_position_correction_velocity_rad_s'], np.deg2rad(30))

    def test_position_correction_state_continues_across_revisions_and_resets_on_expiry(self):
        f = self.velocity_fuser()
        old = np.zeros((30, 14)); new = old+.01
        f.fuse(old, start_s=0., dt=1/30)
        f.fuse(new, start_s=.25, dt=1/30)
        expected = f._sample(f.correction_velocity, .25*30)
        self.assertGreater(np.linalg.norm(expected), 0)
        f.fuse(new, start_s=.5, dt=1/30)
        np.testing.assert_allclose(f.correction_velocity[0], expected)
        actual, info = f.fuse(new, start_s=3., dt=1/30)
        self.assertTrue(info['history_expired'])
        np.testing.assert_array_equal(actual, new)
        np.testing.assert_array_equal(f.correction_velocity, 0.)
        f.reset()
        self.assertIsNone(f.motion_velocity)
        self.assertIsNone(f.correction_velocity)

    def test_late_new_block_joins_the_applied_endpoint_recovery_velocity(self):
        f = self.velocity_fuser()
        old = np.tile((np.arange(30)/30*.3)[:, None], (1, 14))
        f.fuse(old, start_s=0., dt=1/30)
        f.fuse(old+.03, start_s=.25, dt=1/30)
        x=(1.02-f.start_s)*30
        expected_q=f._sample(f.plan[:, f.joint_columns], x)
        expected_v=f._sample(np.gradient(f.plan[:, f.joint_columns], 1/30, axis=0), x)
        result, info=f.fuse(old+.08, start_s=1.02, dt=1/30)
        self.assertTrue(info['late_endpoint_join'])
        np.testing.assert_allclose(result[0, f.joint_columns], expected_q)
        np.testing.assert_allclose(f.motion_velocity[0]+f.correction_velocity[0], expected_v)

    def test_thousand_reference_points_do_not_accumulate_drift_and_grippers_are_latest(self):
        f = self.velocity_fuser()
        result = []
        end = 999/30
        for frame in range(0, 1000, 8):
            t = frame/30 + np.arange(30)/30
            raw = np.zeros((30, 14))
            raw[:, 0] = .05*np.sin(.5*np.minimum(t, end))
            raw[:, [6, 13]] = (frame//8 % 2)
            plan, _ = f.fuse(raw, start_s=frame/30, dt=1/30)
            result.extend(plan[:min(8, 1000-frame)])
            np.testing.assert_array_equal(plan[:, [6, 13]], raw[:, [6, 13]])
            self.assertEqual(f.plan.shape, (30, 14))
        result = np.asarray(result)
        self.assertEqual(len(result), 1000)
        expected = .05*np.sin(.5*np.arange(1000)/30)
        self.assertLess(np.max(abs(result[:, 0]-expected)), .001)
        self.assertAlmostEqual(f.plan[-1, 0], .05*np.sin(.5*end))

    def test_velocity_mode_short_and_stationary_plans_retain_exact_latest_endpoint(self):
        for n in (1, 2, 3, 30):
            with self.subTest(n=n):
                f = self.velocity_fuser()
                f.fuse(np.zeros((30, 14)), start_s=0., dt=1/30)
                raw = np.full((n, 14), .01)
                actual, _ = f.fuse(raw, start_s=.25, dt=1/30)
                self.assertTrue(np.isfinite(actual).all())
                np.testing.assert_array_equal(actual[-1], raw[-1])
                if n == 30:
                    self.assertLess(actual[:, 0].max()-.01, np.deg2rad(.15))

    def test_repeated_biased_motion_forecasts_cannot_accumulate_position_drift(self):
        f = self.velocity_fuser()
        result = []
        for frame in range(0, 1000, 8):
            # Each independent forecast suggests forward velocity but its
            # absolute positions restart at the same place. Integrating only
            # velocity would drift by several radians over this sequence.
            raw = np.zeros((30, 14))
            raw[:, 0] = .02+.1*np.arange(30)/30
            plan, _ = f.fuse(raw, start_s=frame/30, dt=1/30)
            result.extend(plan[:min(8, 1000-frame), 0])
        self.assertLess(max(abs(np.asarray(result))), .12)
        self.assertLess(abs(result[-1]), .08)

    def test_identical_future_motion_has_no_filter_lag_at_fractional_offset(self):
        f = self.fuser()
        t = np.arange(30)*.1
        first = np.tile(t[:, None], (1, 14))
        f.fuse(first, start_s=0., dt=.1)
        latest = np.tile((t+.235)[:, None], (1, 14))
        actual, info = f.fuse(latest, start_s=.235, dt=.1)
        np.testing.assert_allclose(actual, latest, atol=1e-14)
        self.assertTrue(info['applied'])
        np.testing.assert_allclose(first[:, 0], t)  # Never mutate model output.

    def test_revision_weights_decay_recursively_and_grippers_stay_latest(self):
        f = self.fuser()
        f.fuse(np.zeros((20, 14)), start_s=0., dt=.1)
        new = np.ones((20, 14))
        second, _ = f.fuse(new, start_s=.2, dt=.1)
        np.testing.assert_allclose(second[0, [0, 7]], .25)
        np.testing.assert_array_equal(second[:, [6, 13]], 1.)
        third, _ = f.fuse(new, start_s=.3, dt=.1)
        np.testing.assert_allclose(third[0, [0, 7]], 1-.75**2)
        self.assertEqual(f.plan.shape, (20, 14))

    def test_old_forecast_releases_before_its_endpoint_and_new_final_is_exact(self):
        f = self.fuser(alpha=.1)
        f.fuse(np.zeros((10, 14)), start_s=0., dt=.1)
        actual, _ = f.fuse(np.ones((10, 14)), start_s=.2, dt=.1)
        self.assertAlmostEqual(actual[0, 0], .1)
        self.assertTrue(np.all(np.diff(actual[:, 0]) >= 0))
        np.testing.assert_array_equal(actual[7:, 0], 1.)
        np.testing.assert_array_equal(actual[-1], np.ones(14))

    def test_expired_history_and_stop_reset_never_replay_stale_targets(self):
        f = self.fuser()
        f.fuse(np.zeros((3, 14)), start_s=0., dt=.1)
        new = np.ones((10, 14))
        actual, info = f.fuse(new, start_s=1., dt=.1)
        np.testing.assert_array_equal(actual, new)
        self.assertTrue(info['history_expired'])
        f.reset()
        actual, info = f.fuse(new*2, start_s=0., dt=.1)
        np.testing.assert_array_equal(actual, new*2)
        self.assertFalse(info['applied'])

    def test_one_remaining_overlap_sample_cannot_change_the_latest_endpoint(self):
        f = self.fuser()
        f.fuse(np.zeros((3, 14)), start_s=0., dt=.1)
        actual, _ = f.fuse(np.ones((1, 14)), start_s=.2, dt=.1)
        np.testing.assert_array_equal(actual, np.ones((1, 14)))

    def test_disabled_baseline_and_bad_parameters(self):
        f = ExponentialActionFusion()
        f.fuse(np.zeros((10, 14)), start_s=0., dt=.1)
        actual, _ = f.fuse(np.ones((10, 14)), start_s=.1, dt=.1)
        np.testing.assert_array_equal(actual, 1.)
        for value in (dict(alpha=0), dict(alpha=float('nan')), dict(alpha=True),
                      dict(alpha=1.1), dict(tail_steps=1), dict(tail_steps=2.5),
                      dict(handover_steps=1), dict(handover_steps=True), dict(handover_steps=31),
                      dict(method='bad'), dict(position_gain=0), dict(correction_tau_s=float('nan')),
                      dict(correction_max_deg_s=1), dict(endpoint_steps=2), dict(correction_fraction=True),
                      dict(enabled=1), dict(unknown=0)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                fusion_config(value)

    def test_handover_starts_on_the_old_moving_forecast_and_admits_new_revision_gradually(self):
        f = ExponentialActionFusion(dict(enabled=True, alpha=.25, tail_steps=8, handover_steps=6))
        old = np.tile(np.arange(30)[:, None]*.01, (1, 14))
        f.fuse(old, start_s=0., dt=.1)
        latest = np.tile((np.arange(30)*.01+.02+.4)[:, None], (1, 14))
        actual, _ = f.fuse(latest, start_s=.2, dt=.1)
        # First future point continues the old moving plan (not a stationary
        # hold); full alpha applies by the sixth point, before the tail taper.
        self.assertAlmostEqual(actual[0, 0], .02)
        self.assertAlmostEqual(actual[5, 0], .07+.25*.4)
        self.assertTrue(np.all(np.diff(actual[:6, 0]) > 0))
        np.testing.assert_array_equal(actual[:, [6, 13]], latest[:, [6, 13]])
        np.testing.assert_array_equal(actual[-1], latest[-1])

    def test_model_guidance_and_output_fusion_require_explicit_selection(self):
        with self.assertRaisesRegex(ValueError, 'separately'):
            A10DualInferenceRunner(
                dict(protocol='openpi_ws', server_port=8001), async_execution=True,
                execute_horizon=0, rtc_config=dict(enabled=True, method='guidance',
                                                  prefix_len=12, decay_end=18),
                action_fusion=dict(enabled=True))

    def test_public_runner_passes_raw_predictions_and_their_origins_to_fusion(self):
        class Policy:
            def infer(self, obs):
                time.sleep(.025)
                actions = np.tile(obs['observation/state'], (30, 1))
                actions[:, 0] += .03
                return dict(actions=actions)

            def close(self):
                pass

        d, log = RecordingDevice(), Mock()
        runner = A10DualInferenceRunner.from_components(
            d, Policy(), dict(SETTINGS, action_dt=.01, async_execution=True,
                             execute_horizon=0, action_fusion=dict(enabled=True, method='velocity', alpha=.25)),
            log, cycles=3)
        with contextlib.redirect_stdout(io.StringIO()):
            runner.run()
        fusion = [c.kwargs for c in log.call_args_list if c.args[0] == 'action_fusion']
        schedule = [c.kwargs for c in log.call_args_list if c.args[0] == 'chunk_schedule']
        self.assertEqual(len(fusion), 3)
        self.assertTrue(all(e['method'] == 'velocity' for e in fusion))
        self.assertFalse(fusion[0]['applied'])
        self.assertTrue(all(e['applied'] for e in fusion[1:]))
        for f, s in zip(fusion, schedule):
            self.assertEqual(f['reference_time'], s['action_timestamp'])
            self.assertGreaterEqual(f['start_s'], f['reference_time'])
            self.assertLessEqual(len(f['raw_targets']), 30)
        self.assertTrue(d.closed)
        self.assertIsNone(runner.ros_operator.action_fusion.plan)

    def test_real_publisher_fuses_at_next_slot_preserves_final_and_resets_on_stop(self):
        d = RecordingDevice()
        op = operator(d, timed=True)
        op.action_fusion = self.fuser()
        old = np.tile(state(d), (30, 1))

        def submit(rows, reference):
            op.execute_trajectory(dict(left=rows[:, :6], right=rows[:, 7:13]),
                                  dict(left=rows[:, 6], right=rows[:, 13]),
                                  dt=.01, async_exec=True, reference_time=reference)

        try:
            submit(old, time.monotonic())
            self.assertTrue(d.started.wait(1))
            time.sleep(.025)
            op.cycle = 1
            op.stream_final = True
            new = old[:7].copy(); new[:, 0] += .04; new[:, 6] = .75
            submit(new, time.monotonic())
            op.wait_trajectory(1)
            sent = [c.kwargs for c in op.log.call_args_list if c.args[0] == 'sent']
            fused = [e for e in sent if e['cycle'] == 1]
            self.assertAlmostEqual(fused[0]['target'][0]-old[0, 0], .01)
            self.assertAlmostEqual(fused[0]['unfused_target'][0]-old[0, 0], .04)
            self.assertTrue(all(e['target'][6] == .75 for e in fused))
            np.testing.assert_allclose(fused[-1]['target'], new[-1])
            self.assertEqual([e['stream_index'] for e in sent], list(range(len(sent))))
            self.assertEqual([e['final'] for e in sent], [False]*(len(sent)-1)+[True])
            self.assertTrue(np.all(np.diff([r[0] for r in d.records]) >= .008))
            op.stop_trajectory('pause')
            self.assertIsNone(op.action_fusion.plan)
        finally:
            op.close()

    def test_timed_velocity_publisher_keeps_stream_indices_final_and_gripper_targets(self):
        d = RecordingDevice()
        op = operator(d, timed=True)
        op.action_fusion = self.velocity_fuser()
        old = np.tile(state(d), (30, 1))

        def submit(rows):
            op.execute_trajectory(dict(left=rows[:, :6], right=rows[:, 7:13]),
                                  dict(left=rows[:, 6], right=rows[:, 13]),
                                  dt=.01, async_exec=True, reference_time=time.monotonic())
        try:
            submit(old)
            self.assertTrue(d.started.wait(1))
            time.sleep(.025)
            op.cycle = 1
            op.stream_final = True
            new = old[:12].copy(); new[:, 0] += .04; new[:, 6] = .8
            submit(new)
            op.wait_trajectory(1)
            sent = [c.kwargs for c in op.log.call_args_list if c.args[0] == 'sent']
            replaced = [e for e in sent if e['cycle'] == 1]
            self.assertAlmostEqual(replaced[0]['target'][0], old[0, 0])
            self.assertTrue(all(e['target'][6] == new[0, 6] for e in replaced))
            np.testing.assert_allclose(replaced[-1]['target'], new[-1])
            self.assertEqual([e['stream_index'] for e in sent], list(range(len(sent))))
            self.assertEqual([e['final'] for e in sent], [False]*(len(sent)-1)+[True])
            self.assertEqual(d.stops, 0)
            op.stop_trajectory('pause')
            self.assertIsNone(op.action_fusion.motion_velocity)
            self.assertIsNone(op.action_fusion.correction_velocity)
        finally:
            op.close()


if __name__ == '__main__':
    unittest.main()
