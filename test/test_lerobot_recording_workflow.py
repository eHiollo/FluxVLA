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

    def test_reset_during_episode_keeps_buffer_and_returns_to_scene_reset(self):
        self.start()
        count = self.session.frames
        self.reset_arms()
        self.assertEqual(self.session.state, 'review')
        self.assertEqual(self.session.frames, count)
        self.assertIsNotNone(self.writer.active)
        self.right(); self.settle()
        self.assertEqual(self.writer.info['total_episodes'], 1)

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


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(LeRobotWorkflowTests(name) for name in LeRobotWorkflowTests.__dict__
                              if name.startswith('test_'))
