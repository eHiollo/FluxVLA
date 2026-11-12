"""Replay old LeRobot stage order with synthetic devices and a real writer."""
import time
import threading
import unittest
from unittest.mock import Mock, patch

from test_vr_recording import SessionTests


class LeRobotWorkflowTests(SessionTests):
    def setUp(self):
        super().setUp()
        self.session.workflow = 'lerobot'
        self.session.backend = 'lerobot'
        self.runtime.robot.get_feedback_snapshot.side_effect = lambda _: self.runtime.last_frame
        self.runtime.prepare_gripper_maintenance = Mock()
        self.runtime.input_pause_hud = lambda: dict(type='hud', phase='input_paused',
            hint='CHECK GRIPPER', controls=dict(prompt='CHECK GRIPPER'))

    def right(self):
        self.update(modifier=True)
        self.update(modifier=True, x=1.)

    def left(self):
        self.update(modifier=True)
        self.update(modifier=True, x=-1.)

    def automatic_next(self):
        self.session._saved_until = 0.
        self.update()
        self.settle()
        self.update(); self.update()

    def test_original_initial_reset_record_scene_reset_save_and_automatic_next(self):
        self.session._hud(time.monotonic())
        self.assertEqual(self.runtime.teleoperator.send_feedback.call_args.args[0]['phase'], 'reset')
        self.right(); self.settle(); self.update(); self.update()
        self.assertEqual(self.session.state, 'recording')
        self.right()
        count = self.session.frames
        self.assertEqual(self.session.state, 'review')
        self.assertEqual(self.writer.info['total_episodes'], 0)  # Old reset stage precedes save.
        self.update(); self.update()
        self.assertEqual(self.session.frames, count)
        self.right(); self.settle()
        self.assertEqual(self.writer.info['total_episodes'], 1)
        self.automatic_next()
        self.assertEqual(self.session.state, 'recording')
        self.assertEqual(self.writer.info['total_episodes'], 1)

    def test_left_selects_rerecord_then_right_discards_and_restarts_same_episode(self):
        self.start()
        self.left()
        self.assertTrue(self.session.events['rerecord_episode'])
        self.assertEqual(self.session.state, 'review')
        self.assertIsNone(self.session.future)  # Preserve until the scene is ready.
        self.assertEqual(self.writer.info['total_episodes'], 0)
        self.session._hud(time.monotonic())
        self.assertEqual(self.runtime.teleoperator.send_feedback.call_args.args[0]['phase'], 'rerecord')
        self.right(); self.settle(); self.automatic_next()
        self.assertEqual(self.session.state, 'recording')
        self.assertEqual(self.writer.info['total_episodes'], 0)
        self.session.command('stop'); self.settle()
        self.assertTrue(self.session.quit_requested)
        self.assertEqual(self.writer.info['total_episodes'], 1)

    def reset_arms(self):
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init') as move:
            self.session.command('up'); self.session.command('up')
            self.session.reset_thread.join(1.)
            self.session._poll_reset()
            move.assert_called_once()

    def test_m_init_then_recenter_and_right_can_start_without_releasing_held_A(self):
        self.update(modifier=True)
        self.update(modifier=True, y=-1.)
        self.update(modifier=True)
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init'):
            self.update(modifier=True, y=-1.)
            self.session.reset_thread.join(1.)
            self.session._poll_reset()
        self.assertEqual(self.session.state, 'idle')
        self.update(modifier=True)  # A never released; only stick recentered.
        self.update(modifier=True, x=1.)
        self.settle()
        self.assertEqual(self.session.state, 'recording', self.session.notice)

    def test_reset_during_episode_keeps_recording_until_finish(self):
        self.start()
        count = self.session.frames
        self.reset_arms()
        self.assertEqual(self.session.state, 'recording')
        self.assertEqual(self.session.frames, count)
        self.assertIsNotNone(self.writer.active)
        self.update()
        self.assertGreater(self.session.frames, count)
        self.right()
        self.assertEqual(self.session.state, 'review')
        self.right(); self.settle()
        self.assertEqual(self.writer.info['total_episodes'], 1)

    def test_reset_records_measured_motion_and_images_in_same_episode(self):
        import pyarrow.parquet as pq
        self.session.sampling_mode = 'control_loop'
        self.start()
        entered, release = threading.Event(), threading.Event()
        def moving(*args, **kwargs):
            entered.set(); release.wait(3.)
        # Use the real writer and changing read-only feedback, while the normal
        # runtime frame stays at the pre-reset position.
        frame = self.runtime.last_frame
        measured = dict(frame, arms={n: dict(a, joints=[.7 if a['ee_id'] == 1 else .8]*6)
                                     for n, a in frame['arms'].items()})
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init', side_effect=moving):
            self.session.command('up')
            self.assertEqual(self.session.state, 'recording')
            self.assertTrue(self.session.capturing)  # Confirmation does not finish the episode.
            self.session.command('up'); self.assertTrue(entered.wait(1.))
            try:
                self.assertTrue(self.session.blocks_motion)
                self.assertTrue(self.session.capturing)
                self.runtime.robot.get_feedback_snapshot.side_effect = lambda _: measured
                count = self.session.frames
                self.update(); self.update()
                self.assertEqual(self.session.frames, count+2)
                self.session._hud(time.monotonic())
                hud = self.runtime.teleoperator.send_feedback.call_args.args[0]
                self.assertEqual(hud['phase'], 'resetting')
                self.assertTrue(hud['capture_active'])
                self.assertIn('capture active', hud['hint'])
                self.assertFalse(hud['controls']['directions']['right']['enabled'])
                self.runtime.robot.get_state.assert_not_called()
                self.runtime.robot.send_joints.assert_not_called()
            finally:
                release.set(); self.session.reset_thread.join(2.)
            self.session._poll_reset()
        self.assertEqual(self.session.state, 'recording')
        self.right(); self.right(); self.settle()
        self.assertEqual(self.writer.info['total_episodes'], 1)
        table = pq.read_table(next(self.root.glob('data/**/*.parquet'))).to_pydict()
        self.assertTrue(any(abs(row[0]-.7)<1e-5 and abs(row[7]-.8)<1e-5
                            for row in table['observation.state']))
        self.assertEqual(len(list(self.root.glob('videos/**/*.mp4'))), 3)

    def test_reset_after_finish_or_before_start_does_not_capture(self):
        for initial in ('idle', 'review'):
            with self.subTest(initial=initial):
                if initial == 'review':
                    self.update(); self.start(); self.right()
                    self.assertEqual(self.session.state, 'review')
                entered, release = threading.Event(), threading.Event()
                def moving(*args, **kwargs):
                    entered.set(); release.wait(3.)
                with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init', side_effect=moving):
                    count = self.session.frames
                    self.session.command('up'); self.session.command('up')
                    self.assertTrue(entered.wait(1.))
                    try:
                        self.update()
                        self.assertFalse(self.session.capturing)
                        self.assertEqual(self.session.frames, count)
                    finally:
                        release.set(); self.session.reset_thread.join(2.)
                    self.session._poll_reset()
                self.assertEqual(self.session.state, initial)

    def test_reset_data_fault_cancels_motion_and_never_resumes_or_saves_episode(self):
        self.start()
        entered = threading.Event()
        def moving(robot, *, cancel_event):
            entered.set(); cancel_event.wait(3.)
            # Even a successful worker return cannot erase a sampling error.
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init', side_effect=moving):
            self.session.command('up'); self.session.command('up')
            self.assertTrue(entered.wait(1.))
            count = self.session.frames
            self.runtime.robot.get_feedback_snapshot.side_effect = lambda _: dict(self.runtime.last_frame, has_error=True)
            self.update()
            self.assertTrue(self.session.reset_cancel.is_set())
            self.session.reset_thread.join(2.); self.session._poll_reset()
        self.assertEqual(self.session.state, 'reset_failed')
        self.assertEqual(self.session.frames, count)
        self.assertIn('faulted', self.session.error)
        self.session.command('save')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_reset_cancel_invalidates_recording_even_if_worker_returns_success(self):
        self.start()
        def moving(robot, *, cancel_event): cancel_event.wait(3.)
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init', side_effect=moving):
            self.session.command('up'); self.session.command('up')
            self.session.command('down')
            self.assertFalse(self.session.capturing)
            self.session.reset_thread.join(2.); self.session._poll_reset()
        self.assertEqual(self.session.state, 'reset_failed')
        self.assertIn('cancelled', self.session.error)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_reset_rejected_or_confirmation_expired_does_not_finish_episode(self):
        self.start(); self.update(grip=True)
        self.session.command('up')
        self.assertEqual(self.session.state, 'recording')
        self.assertIsNone(self.session.pending)
        self.update(); self.session.command('up')
        self.session.pending_at -= 20.
        self.update()
        self.assertIsNone(self.session.pending)
        self.assertEqual(self.session.state, 'recording')

    def test_disabled_gripper_with_real_feedback_can_record_without_automatic_enable(self):
        for channel in self.channels.values():
            read = channel.snapshot.side_effect
            channel.snapshot.side_effect = lambda source=read: dict(source(), enabled=False)
        self.update(); self.start()
        self.assertEqual(self.session.state, 'recording', self.session.health)
        for channel in self.channels.values():
            channel.submit.assert_not_called()
        self.session.command('stop'); self.settle()
        self.assertEqual(self.writer.info['total_episodes'], 1)

    def test_manual_gripper_reset_requires_release_and_verified_stop(self):
        self.runtime.prepare_gripper_maintenance.side_effect = TimeoutError('stop unconfirmed')
        self.session.command('gripper-reset left')
        self.channels['left'].request_alarm_action.assert_not_called()
        self.runtime.prepare_gripper_maintenance.side_effect = None
        self.session.command('gripper-reset left')
        self.channels['left'].request_alarm_action.assert_called_once_with('reset')
        self.channels['left'].request_alarm_action.reset_mock()
        self.update(grip=True)
        self.session.command('gripper-reset left')
        self.channels['left'].request_alarm_action.assert_not_called()

    def test_gripper_alarm_blocks_training_only_and_starts_after_clearing_without_VR_rearm(self):
        read = self.channels['left'].snapshot.side_effect
        self.channels['left'].snapshot.side_effect = lambda: dict(read(), error='alarm 0x1000', alarm_b=4096)
        self.session.gripper_faulted({'left': 'alarm 0x1000'})
        self.update(); self.session.command('start')
        self.assertEqual(self.session.state, 'idle')
        self.assertIsNone(self.session.future)
        self.assertFalse(self.session.blocks_motion)
        self.assertEqual(self.writer.info['total_frames'], 0)
        self.assertIsNone(getattr(self.runtime, 'input_pause_status', None))
        self.session._hud(time.monotonic())
        self.assertIn('ARMS AVAILABLE', self.runtime.teleoperator.send_feedback.call_args.args[0]['hint'])
        self.channels['left'].snapshot.side_effect = read
        self.session.gripper_faulted({})
        self.update(); self.start()
        self.assertEqual(self.session.state, 'recording')
        self.session.gripper_faulted({'left': 'alarm 0x1000'})
        self.assertEqual(self.session.state, 'failed')
        self.assertFalse(self.session.blocks_motion)
        self.session.command('save')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_interrupted_episode_cannot_be_saved_but_can_follow_old_rerecord_flow(self):
        self.start()
        self.session.fail('gripper alarm 0x1000')
        self.runtime.input_pause_status = dict(stopped=True, reason='gripper_alarm')
        self.session.command('right')
        self.assertIsNone(self.session.future)
        self.session.command('left'); self.session.command('right'); self.settle()
        self.update()
        self.assertEqual(self.writer.info['total_episodes'], 0)
        self.assertTrue(self.session._auto_start)
        self.assertIsNone(self.session.future)  # No automatic restart before resolving the hardware fault.
        self.runtime.input_pause_status = None
        self.automatic_next()
        self.assertEqual(self.session.state, 'recording')

    def test_stop_during_async_start_never_enters_recording_or_publishes_empty_episode(self):
        entered, release = threading.Event(), threading.Event()
        start = self.writer.start
        def delayed_start(task):
            entered.set(); release.wait(2.); return start(task)
        with patch.object(self.writer, 'start', delayed_start):
            self.session.command('start')
            self.assertTrue(entered.wait(1.))
            self.session.command('stop')
            release.set(); self.settle()
        self.assertTrue(self.session.quit_requested)
        self.assertNotEqual(self.session.state, 'recording')
        self.assertEqual(self.session.frames, 0)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_redo_after_stale_input_waits_for_stability_then_restarts_same_episode(self):
        self.start()
        self.session.fail('VR input stale; episode is incomplete')
        self.runtime.input_pause_status = dict(state='release_required', stopped=True)
        def resume():
            if self.runtime.input_pause_status['state'] != 'ready': return False
            self.runtime.input_pause_status = None
            return True
        self.runtime.resume_recording_input = Mock(side_effect=resume)
        menu = self.session._hud_controls()['directions']
        self.assertTrue(menu['left']['enabled'])
        self.session.command('left'); self.session.command('right'); self.settle()
        self.update()
        self.assertEqual(self.session.state, 'idle')
        self.assertTrue(self.session._auto_start)
        menu = self.session._hud_controls()['directions']
        self.assertTrue(menu['left']['enabled'])
        self.assertEqual(menu['left']['label'], 'RETRY QUEUED')
        self.session._hud(time.monotonic())
        hud = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual(hud['controls']['prompt'], 'release grips; center sticks')
        self.assertFalse(self.session.quit_requested)
        self.runtime.input_pause_status['state'] = 'ready'
        self.update(); self.settle(); self.update()
        self.assertEqual(self.session.state, 'recording', self.session.health)
        self.assertIsNone(self.runtime.input_pause_status)
        self.assertEqual(self.writer.info['total_episodes'], 0)
        self.assertFalse(self.session.quit_requested)

    def test_idle_start_queues_retry_without_saving_or_unlocking_unstable_input(self):
        self.runtime.input_pause_status = dict(state='release_required', stopped=True)
        self.runtime.resume_recording_input = Mock(return_value=False)
        self.session.command('right'); self.update()
        self.assertTrue(self.session._auto_start)
        self.assertEqual(self.session.state, 'idle')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_frames'], 0)
        self.session.command('left')
        self.assertTrue(self.session._auto_start)
        self.session.command('stop')
        self.assertTrue(self.session.quit_requested)
        self.assertFalse(self.session._auto_start)


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(LeRobotWorkflowTests(name) for name in LeRobotWorkflowTests.__dict__
                              if name.startswith('test_'))
