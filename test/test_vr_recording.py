"""Synthetic recording/VR lifecycle tests; never opens real device endpoints."""
import copy
import json
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor, Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from mmengine import Config

from fluxvla.datasets.episode_writer import EpisodeWriter
from fluxvla.datasets.a10_recording_io import validate_dataset
from fluxvla.engines.operators.camera_operator import BufferedCameraOperator
from fluxvla.engines.operators.vr.recording import RecordingSession
from fluxvla.engines.operators.vr.vr_events import RecordingButtons
from fluxvla.engines.operators.vr.runtime import build_runtime
from fluxvla.engines.operators.kaanh_commands.m_init import move_to_init, configure_init_motion

ROOT = Path(__file__).resolve().parents[1]


def hands(seq, *, modifier=False, x=0., y=0., age=0., grip=False):
    return {side: {'xlevr.sample_age_s':age, 'xlevr.sample_sequence':seq,
                  'xlevr.thumbstick':{'x':x if side=='left' else .6, 'y':y if side=='left' else 0.},
                  'xlevr.buttons':{'a':modifier if side=='right' else False,'squeeze':grip},
                  'xlevr.grip_active':grip, 'xlevr.trigger':0.}
            for side in ('left','right')}


def actions():
    return {side:{'gripper.pos':.6, '_xlevr.anchor_command':{'gripper':.6}} for side in ('left','right')}


class ButtonTests(unittest.TestCase):
    def test_gradual_push_and_dominant_diagonal_emit_once_per_neutral(self):
        b = RecordingButtons()
        b.update(hands(1)); b.update(hands(2, modifier=True))
        for seq, x in enumerate((-.205, -.4, -.69), 3):
            self.assertIsNone(b.update(hands(seq, modifier=True, x=x))['command'])
            self.assertTrue(b.armed)
        self.assertEqual(b.update(hands(6, modifier=True, x=-.93, y=.37))['command'], 'left')
        self.assertIsNone(b.update(hands(7, modifier=True, x=-.95, y=.35))['command'])
        b.update(hands(8, modifier=True))
        self.assertEqual(b.update(hands(9, modifier=True, x=-.8, y=.59))['command'], 'left')
        b.update(hands(10, modifier=True))
        self.assertEqual(b.update(hands(11, modifier=True, x=.35, y=-.9))['command'], 'up')

    def test_neutral_gates_edges_diagonals_and_gripper_restore(self):
        b=RecordingButtons()
        self.assertTrue(b.update(hands(1,modifier=True,x=1))['block_left'])
        self.assertIsNone(b.update(hands(2,modifier=True))['command'])
        self.assertFalse(b.update(hands(3))['block_left'])
        self.assertIsNone(b.update(hands(4,modifier=True,x=1))['command'])
        b.update(hands(5,modifier=True))
        self.assertEqual(b.update(hands(6,modifier=True,x=1))['command'],'right')
        self.assertIsNone(b.update(hands(7,modifier=True,x=1))['command'])
        b.update(hands(8,modifier=True))
        self.assertIsNone(b.update(hands(9,modifier=True,x=1,y=1))['command'])
        self.assertIsNone(b.update(hands(10,modifier=True,y=1))['command'])
        b.update(hands(11,modifier=True))
        self.assertEqual(b.update(hands(12,modifier=True,y=-1))['command'],'up')
        self.assertTrue(b.update(hands(13,x=1))['block_left'])
        self.assertFalse(b.update(hands(14))['block_left'])

    def test_stale_duplicate_and_reconnect_cannot_replay(self):
        b=RecordingButtons(); b.update(hands(1)); b.update(hands(2,modifier=True))
        self.assertEqual(b.update(hands(3,modifier=True,x=-1))['command'],'left')
        b.update(hands(3,modifier=True))
        self.assertIsNone(b.update(hands(3,modifier=True,x=-1))['command'])
        self.assertFalse(b.update(hands(4,age=1))['fresh'])
        self.assertIsNone(b.update(hands(5,modifier=True,x=-1))['command'])
        b.update(hands(6)); b.update(hands(7,modifier=True))
        result=b.update(hands(0,modifier=True,x=-1))
        self.assertTrue(result['restarted']); self.assertIsNone(result['command'])


class CameraTests(unittest.TestCase):
    def test_bounded_background_snapshot_and_cleanup(self):
        closed=[]
        class Camera:
            def __init__(self, settings): self.name=settings['device']
            def read_rgb(self):
                time.sleep(.003); return np.zeros((16,16,3),dtype=np.uint8)
            def close(self): closed.append(self.name)
        cameras=BufferedCameraOperator({'a':{'device':'a'},'b':{'device':'b'}},camera_factory=Camera)
        self.assertEqual(closed,[]); cameras.start()
        try:
            deadline=time.monotonic()+1
            while min(map(len,cameras.histories.values()))<4 and time.monotonic()<deadline: time.sleep(.01)
            frames=cameras.snapshot(time.monotonic())
            self.assertEqual(set(frames),{'a','b'})
            self.assertTrue(all(len(v)<=4 for v in cameras.histories.values()))
            self.assertFalse(frames['a']['image'].flags.writeable)
        finally: cameras.close()
        self.assertEqual(set(closed),{'a','b'})

    def test_camera_fault_visible_and_snapshot_does_not_wait(self):
        entered=threading.Event(); release=threading.Event()
        class Camera:
            def __init__(self, _): pass
            def read_rgb(self): entered.set(); release.wait(2); raise OSError('unplugged')
            def close(self): pass
        cameras=BufferedCameraOperator({'a':{'device':'a'}},camera_factory=Camera); cameras.start()
        try:
            self.assertTrue(entered.wait(1))
            with self.assertRaisesRegex(RuntimeError,'Waiting'): cameras.snapshot(time.monotonic())
            release.set(); cameras._threads[0].join(1)
            with self.assertRaisesRegex(RuntimeError,'unplugged'): cameras.snapshot(time.monotonic())
        finally: release.set(); cameras.close()


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.root=Path(self.temp.name)/'data'
        cfg=Config.fromfile(str(ROOT/'configs/a10/a10_dual_arm_record.py')).to_dict()
        cfg['record']['root']=str(self.root)
        # These cases exercise the historical strict writer. Native LeRobot
        # lifecycle/quality behavior has its own integration tests.
        cfg['record'].update(backend='legacy')
        cfg['record'].pop('image_writer_threads', None)
        cfg['capture']['sampling_mode']='background_30hz'
        cfg['capture']['workflow']='legacy'
        cfg['capture'].update(terminal=False, max_clock_drift_s=10., max_sample_gap_s=10.)
        for group in ('record',''):
            cams=cfg['record']['cameras'] if group else cfg['cameras']
            for c in cams.values(): c.update(width=16,height=16)
        self.channels={}
        for side in ('left','right'):
            meta=cfg['record']['device_metadata']['grippers'][side]
            channel=Mock(serial=meta['identity']['serial'],device_id=meta['device_id'],
                         low=0,high=9000,opened=0,closed=9000)
            channel.snapshot.side_effect=lambda: dict(actual_raw=4500,feedback_monotonic_s=time.monotonic(),
                                                     error=None,enabled=True,alarm_a=0,alarm_b=0)
            self.channels[side]=channel
        self.robot=Mock(allow_motion=True,camera_operator=None,follower_stop_pending=False)
        self.runtime=SimpleNamespace(teleoperator=Mock(is_dual=True,hand_to_arm={'left':'right','right':'left'}),
            execute=True,observe_only=False,control_mode='reference',robot=self.robot,
            _zx_channels=self.channels,_frame_index=0,last_frame=None,last_sdk_command=None,last_reference_result=None)
        self.session=RecordingSession(self.runtime,cfg)
        # Storage is explicitly synthetic in tests; bypass live startup only here.
        self.writer=EpisodeWriter(self.root,cfg['record']['cameras'],schema='a10_dual_v1',synthetic=True)
        self.session.writer=self.writer
        self.session.pool=ThreadPoolExecutor(max_workers=1)
        self.session.cameras=Mock()
        self.seq=0
        def camera_snapshot(t):
            return {n:dict(image=np.zeros((16,16,3),np.uint8),received_monotonic_s=time.monotonic(),sequence=self.seq)
                    for n in cfg['cameras']}
        self.session.cameras.snapshot.side_effect=camera_snapshot
        self.update()

    def tearDown(self):
        self.session.before_close(); self.session.close(); self.temp.cleanup()

    def update(self, **kwargs):
        self.seq+=1; now=time.monotonic()
        self.runtime.last_frame=dict(has_error=False,received_monotonic_s=now,arms={
            'left':dict(ee_id=0,joints=[.2]*6,received_monotonic_s=now),
            'right':dict(ee_id=1,joints=[.1]*6,received_monotonic_s=now)})
        self.session.before_step(hands(self.seq,**kwargs),actions())
        self.session.after_step()

    def settle(self):
        if self.session.future: self.session.future.result(timeout=3)
        self.session._poll_future()

    def start(self):
        self.session.command('start'); self.settle(); self.update(); self.update()

    def prepare_spike_recording(self):
        self.session.backend = 'lerobot'
        self.session.continue_on_spike = True
        self.robot.get_feedback_snapshot.side_effect = lambda _: self.runtime.last_frame
        self.runtime.input_pause_hud = lambda: dict(type='hud', phase='input_paused', error='',
            hint='left hand: spike_pending', input_pause=self.runtime.input_pause_status,
            controls=dict(prompt='release grips; center sticks'))
        self.start()
        self.runtime.input_pause_status = dict(state='release_required', reason='spike_pending', stopped=False)
        self.session.input_paused('Dual VR right: spike_pending')

    def test_spike_pause_keeps_same_episode_measured_frames_and_diagnostics(self):
        self.prepare_spike_recording()
        session = self.session
        frames = session.frames
        self.update()
        self.assertEqual(session.state, 'recording', session.error)
        self.assertGreater(session.frames, frames)
        # A stale hand while motion is locked is not a stale robot measurement.
        self.update(age=.4)
        self.assertEqual(session.state, 'recording', session.error)
        # The optional background sampler obeys the same measured-data rule.
        sample = session._assemble(time.monotonic(), raw=hands(self.seq, age=.4))
        self.assertEqual(sample[2]['control_phase'], 'input_pause')
        self.assertIsNone(sample[2]['sdk_tool_command'])
        session.options['max_sample_gap_s'] = .05
        time.sleep(.08)  # Simulate waiting for stop; no invented catch-up frames.
        self.update()
        self.assertGreaterEqual(session.sample_gaps, 1)
        self.runtime.input_pause_status['stopped'] = True
        self.update()
        session._hud(time.monotonic())
        hud = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual(hud['hint'], 'input paused; recording')
        self.assertTrue(hud['capture_active'])
        session.command('reset')
        self.assertIsNone(session.reset_thread)
        self.assertIsNone(session.pending)
        # Recovery is supplied by runtime only after release/stability/new press.
        self.runtime.input_pause_status = None
        self.update()
        self.assertEqual(session.state, 'recording')
        session.command('save'); self.settle(); session.close()
        self.assertEqual(self.writer.info['total_episodes'], 1)
        rows = [json.loads(line) for line in next((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')).read_text().splitlines()]
        paused = [row for row in rows if row['details']['control_phase'] == 'input_pause']
        self.assertTrue(paused)
        self.assertTrue(any(row['details']['control_phase'] == 'teleop' for row in rows))
        self.assertEqual(paused[0]['details']['input_pause']['reason'], 'spike_pending')
        self.assertEqual(paused[0]['details']['grippers']['left']['actual_raw'], 4500)
        self.assertGreaterEqual(rows[-1]['details']['capture_gap_s'], 0.)

    def test_spike_pause_does_not_allow_camera_or_robot_fault_or_other_pause(self):
        self.prepare_spike_recording()
        self.session.continue_on_vr_pause = False  # Historical spike-only policy.
        for change in ('camera', 'robot', 'stop_failed', 'communication', 'stale_pause', 'policy_off'):
            with self.subTest(change=change):
                self.runtime._dual_stop_failed = change == 'stop_failed'
                self.runtime.communication_pause = {'reason': 'get failed'} if change == 'communication' else None
                self.runtime.input_pause_status['reason'] = 'input_stale' if change == 'stale_pause' else 'spike_pending'
                self.session.continue_on_spike = change != 'policy_off'
                self.runtime.last_frame['has_error'] = change == 'robot'
                if change == 'camera':
                    with patch.object(self.session.cameras, 'snapshot', side_effect=RuntimeError('camera failed')):
                        with self.assertRaisesRegex(RuntimeError, 'camera failed'): self.session._assemble(time.monotonic())
                else:
                    with self.assertRaises(ValueError): self.session._assemble(time.monotonic())
        self.session.continue_on_spike = True
        self.runtime.input_pause_status['reason'] = 'spike_pending'
        self.runtime._dual_stop_failed = False
        self.runtime.communication_pause = None
        self.session.gripper_faulted({'left': 'overcurrent'})
        self.assertEqual(self.session.state, 'failed')

    def test_stream_pause_keeps_same_episode_and_saves_measured_frames(self):
        self.prepare_spike_recording()
        self.runtime.input_pause_status = dict(reason='vr_stream_stale', state='release_required', stopped=True)
        self.session.input_paused('Dual VR right: vr_stream_stale')
        count = self.session.frames
        self.update(age=1.9)
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertGreater(self.session.frames, count)
        sample = self.session._assemble(time.monotonic(), raw=hands(100, age=1.9))
        self.assertEqual(sample[2]['input_pause']['reason'], 'vr_stream_stale')
        self.assertIsNone(sample[2]['sdk_tool_command'])
        self.assertIsNone(sample[2]['reference_control'])
        self.assertEqual(sample[0].shape, (14,))
        hud = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertTrue(hud['capture_active'])
        self.assertEqual(hud['recording_state'], 'recording')
        self.assertEqual(hud['hint'], 'input paused; recording')
        self.robot.start_follower.assert_not_called()
        self.runtime.input_pause_status = None
        self.update()
        self.assertEqual(self.session.state, 'recording')
        self.assertEqual(self.session.status()['episode'], 1)
        self.session.command('save'); self.settle(); self.session.close()
        self.assertEqual(self.writer.info['total_episodes'], 1)
        rows = [json.loads(line) for line in next((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')).read_text().splitlines()]
        self.assertTrue(any((r['details']['input_pause'] or {}).get('reason') == 'vr_stream_stale' for r in rows))

    def test_stale_input_before_runtime_guard_does_not_fail_background_sample(self):
        self.prepare_spike_recording()
        self.runtime.input_pause_status = None
        self.update(age=.4)  # before_step detects stale before runtime sets pause.
        self.assertEqual(self.session.state, 'recording', self.session.error)
        raw = hands(100)
        for hand in raw.values(): hand['xlevr.sample_receive_time_s'] = time.monotonic()-1.9
        sample = self.session._assemble(time.monotonic(), raw=raw)
        self.assertEqual(sample[2]['control_phase'], 'input_pause')
        self.assertEqual(sample[2]['input_pause']['state'], 'pending_runtime_guard')
        self.assertIsNone(sample[2]['sdk_tool_command'])

    def test_stream_pause_policy_off_and_unsafe_pause_still_fail(self):
        self.prepare_spike_recording()
        for reason in ('feedback_stale', 'control_gap', 'unknown'):
            self.runtime.input_pause_status = dict(reason=reason, stopped=True)
            with self.subTest(reason=reason), self.assertRaises(ValueError):
                self.session._assemble(time.monotonic(), raw=hands(1, age=1))
        self.runtime.input_pause_status = dict(reason='vr_stream_stale', stopped=True)
        self.session.continue_on_vr_pause = False
        self.session.input_paused('VR stream stale')
        self.assertEqual(self.session.state, 'failed')
        self.assertFalse(self.session.capturing)

    def test_stream_pause_rejects_real_sensor_faults_and_unconfirmed_stop(self):
        self.prepare_spike_recording()
        self.runtime.input_pause_status = dict(reason='vr_stream_stale', stopped=True)
        for issue in ('robot', 'camera', 'gripper', 'stop', 'communication'):
            with self.subTest(issue=issue):
                self.runtime._dual_stop_failed = issue == 'stop'
                self.runtime.communication_pause = {'reason': 'failed'} if issue == 'communication' else None
                self.runtime.last_frame['has_error'] = issue == 'robot'
                with patch.object(self.session.cameras, 'snapshot', side_effect=(RuntimeError('camera fault') if issue == 'camera' else self.session.cameras.snapshot.side_effect)), \
                     patch.object(self.channels['left'], 'snapshot', side_effect=lambda: dict(
                         actual_raw=4500, feedback_monotonic_s=time.monotonic(), error='gripper fault' if issue == 'gripper' else None,
                         enabled=True, alarm_a=0, alarm_b=0)):
                    with self.assertRaises((ValueError, RuntimeError)):
                        self.session._assemble(time.monotonic(), raw=hands(1, age=1))

    def test_failed_recording_hud_cannot_be_overwritten_by_runtime_pause(self):
        self.prepare_spike_recording()
        self.session.fail('camera disconnected')
        self.session._hud(time.monotonic())
        hud = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual(hud['recording_state'], 'failed')
        self.assertEqual(hud['error'], 'camera disconnected')
        self.assertFalse(hud['capture_active'])
        self.assertNotIn('input paused; recording', hud['hint'])

    def test_vr_pause_does_not_start_idle_or_allow_reset(self):
        self.session.backend = 'lerobot'
        self.runtime.input_pause_status = dict(reason='vr_stream_stale', stopped=True)
        self.runtime.input_pause_hud = lambda: dict(phase='input_paused', hint='vr signal delayed',
            controls=dict(prompt='release grips; wait for signal'))
        self.session.input_paused('VR stream stale')
        self.update(age=1)
        self.session.command('start')
        self.assertEqual(self.session.state, 'idle')
        self.assertIsNone(self.session.future)
        self.session.command('reset')
        self.assertIsNone(self.session.reset_thread)
        self.assertIsNone(self.session.pending)

    def test_background_30hz_keeps_writing_during_stream_pause(self):
        self.prepare_spike_recording()
        self.runtime.input_pause_status = dict(reason='vr_stream_stale', stopped=True)
        raw = hands(100, age=1.9)
        for hand in raw.values(): hand['xlevr.sample_receive_time_s'] = time.monotonic()-1.9
        self.runtime.teleoperator.input.get_hand_actions.return_value = raw
        def feedback(_):
            frame = copy.deepcopy(self.runtime.last_frame)
            frame['received_monotonic_s'] = time.monotonic()
            for arm in frame['arms'].values():
                arm['received_monotonic_s'] = frame['received_monotonic_s']
            return frame
        self.robot.get_feedback_snapshot.side_effect = feedback
        before = self.session.frames
        self.session._start_sampling()
        try:
            deadline = time.monotonic()+1.
            while self.session.frames < before+4 and time.monotonic() < deadline:
                time.sleep(.01)
        finally:
            self.session._stop_sampling()
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertGreaterEqual(self.session.frames, before+4)
        self.assertEqual(self.session.status()['episode'], 1)
        self.assertEqual(self.session.record['fps'], 30)
        self.robot.start_follower.assert_not_called()

    def test_stream_pause_still_skips_stale_measurements_and_fails_persistent_loss(self):
        self.prepare_spike_recording()
        self.runtime.input_pause_status = dict(reason='vr_stream_stale', stopped=True)
        frame = copy.deepcopy(self.runtime.last_frame)
        frame['received_monotonic_s'] = time.monotonic()-1.
        before = self.session.frames
        self.session._prepare_sample(time.monotonic(), frame=frame, raw=hands(100, age=1))
        self.assertIsNone(self.session.ready_sample)
        self.assertEqual(self.session.state, 'recording')
        self.assertGreater(self.session.skipped_samples, 0)
        self.session._append_sample(time.monotonic())
        self.assertEqual(self.session.frames, before)
        self.session._missing_since = time.monotonic()-2.
        self.session._prepare_sample(time.monotonic(), frame=frame, raw=hands(100, age=1))
        self.assertEqual(self.session.state, 'failed')
        self.session.command('save')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_drop_warning_records_actual_samples_and_preserves_alarm_in_saved_sidecar(self):
        source = self.channels['right'].snapshot.side_effect
        self.channels['right'].snapshot.side_effect = lambda: dict(
            source(), alarm_b=0x1000, drop_alarm_policy='warn', warning='drop detected')
        self.update(); self.start()
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertEqual(self.session.status()['gripper_warnings'], {'right': 'drop detected'})
        self.session._hud(time.monotonic())
        self.assertEqual(self.runtime.teleoperator.send_feedback.call_args.args[0]['gripper_warnings'],
                         {'right': 'drop detected'})
        self.session.command('save'); self.settle(); self.session.close()
        files = list((self.root/'meta/xlevr_diagnostics').rglob('*.jsonl'))
        self.assertTrue(files)
        records = [json.loads(line) for line in files[0].read_text().splitlines()]
        self.assertTrue(records)
        for row in records:
            grip = row['details']['grippers']['right']
            self.assertEqual(grip['alarm_b'], 0x1000)
            self.assertEqual(grip['warning'], 'drop detected')
            self.assertEqual(grip['actual_raw'], 4500)

    def test_hud_modifier_and_recenter_update_without_waiting_for_timer(self):
        session = self.session
        send = self.runtime.teleoperator.send_feedback
        session._hud(100.)
        send.reset_mock()
        session._hud(100.01)
        send.assert_not_called()
        self.seq += 1
        session.before_step(hands(self.seq, modifier=True, x=.9), actions())
        session._hud(100.02)
        controls = send.call_args.args[0]['controls']
        self.assertTrue(controls['visible'])
        self.assertFalse(controls['ready'])
        self.assertEqual(controls['prompt'], 'CENTER LEFT STICK')
        self.seq += 1
        session.before_step(hands(self.seq, modifier=True), actions())
        session._hud(100.03)
        self.assertTrue(send.call_args.args[0]['controls']['ready'])
        self.seq += 1
        session.before_step(hands(self.seq), actions())
        session._hud(100.04)
        self.assertFalse(send.call_args.args[0]['controls']['visible'])

    def test_hud_directions_match_episode_state_and_confirmation(self):
        session = self.session
        self.assertEqual(session._hud_controls()['directions']['right']['label'], 'START')
        self.start()
        menu = session._hud_controls()['directions']
        self.assertEqual(menu['right']['label'], 'SAVE')
        self.assertEqual(menu['down']['label'], 'STOP CAPTURE')
        self.assertFalse(menu['up']['enabled'])
        session.command('discard')
        self.assertEqual(session._hud_controls()['directions']['left']['label'], 'CONFIRM DISCARD')
        session._hud(time.monotonic())
        self.assertEqual(self.runtime.teleoperator.send_feedback.call_args.args[0]['pending'], 'discard')
        session.command('stop')
        self.assertEqual(session.state, 'review')
        self.assertFalse(session._hud_controls()['directions']['down']['enabled'])
        session.fail('camera unplugged')
        session.state = 'failed'
        self.assertFalse(session._hud_controls()['directions']['right']['enabled'])

    def test_hud_reset_and_disk_busy_offer_only_valid_controls(self):
        session = self.session
        session.command('reset')
        self.assertEqual(session._hud_controls()['directions']['up']['label'], 'CONFIRM RESET')
        session.pending = None
        session.reset_thread = object()
        try:
            directions = session._hud_controls()['directions']
            self.assertEqual([k for k, v in directions.items() if v['enabled']], ['down'])
            self.assertEqual(directions['down']['label'], 'CANCEL RESET')
        finally: session.reset_thread = None
        session.future = object()
        try: self.assertFalse(any(v['enabled'] for v in session._hud_controls()['directions'].values()))
        finally: session.future = None

    def test_hud_save_success_expires_without_blocking_or_auto_start(self):
        self.start(); self.session.command('save'); self.settle()
        session = self.session
        session._hud(session._saved_until - .1)
        payload = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual((payload['phase'], payload['episode']), ('saved', 1))
        session._hud(session._saved_until + .01)
        payload = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual((payload['phase'], payload['episode']), ('idle', 2))
        self.assertEqual(payload['hint'], 'Ready for next episode')
        self.assertEqual(session.state, 'idle')
        self.assertIsNone(session.future)

    def test_button_check_shows_menu_without_debug_dump_or_device_commands(self):
        session = self.session
        session.button_check = True; session.state = 'button_check'
        self.update(modifier=True)
        self.update(modifier=True, x=1.)
        payload = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual(payload['hint'], 'BUTTON EVENT: right')
        self.assertTrue(payload['controls']['visible'])
        self.assertEqual(payload['controls']['directions']['up']['label'], 'RESET')
        self.assertNotIn('pressed=', payload['hint'])
        self.assertEqual(session.frames, 0)
        self.assertIsNone(session.future)
        self.robot.start_follower.assert_not_called()
        self.update(age=1.)
        payload = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertFalse(payload['controls']['visible'])
        self.assertIn('INPUT LOST', payload['controls']['prompt'])

    def test_record_save_two_episodes_discard_and_same_frame_values(self):
        self.start(); self.session.command('save'); self.settle()
        self.assertEqual(self.session.state,'idle')
        self.start(); self.session.command('discard')
        self.assertEqual(self.session.pending,'discard')
        self.session.command('discard'); self.settle()
        self.assertEqual(self.writer.info['total_episodes'],1)
        self.start(); self.session.command('save'); self.settle()
        self.session.close()
        report=validate_dataset(self.root)
        self.assertEqual(report['episodes'],2)
        import pyarrow.parquet as pq
        row=pq.read_table(self.root/'data/chunk-000/file-000.parquet').to_pylist()[0]
        np.testing.assert_allclose(row['observation.state'],[.1]*6+[.5]+[.2]*6+[.5])
        self.assertEqual(row['action'],row['observation.state'])

    def test_stop_is_review_and_resume_requires_new_episode(self):
        self.start(); count=self.session.frames
        self.session.command('stop'); self.update()
        self.assertEqual(self.session.state,'review'); self.assertEqual(count,self.session.frames)
        self.session.command('start'); self.assertEqual(self.session.state,'review')
        self.session.command('right'); self.settle(); self.assertEqual(self.session.state,'idle')
        self.robot.stop_follower.assert_not_called()

    def _background_inputs(self, *, frozen_robot=False, untracked_idle=False):
        stamp = time.monotonic()
        def measured(_reader):
            now = stamp if frozen_robot else time.monotonic()
            return dict(has_error=False, received_monotonic_s=now, arms={
                'left': dict(ee_id=0, joints=[.2]*6, received_monotonic_s=now),
                'right': dict(ee_id=1, joints=[.1]*6, received_monotonic_s=now)})
        self.robot.get_feedback_snapshot.side_effect = measured
        def controllers():
            self.seq += 1
            raw = hands(self.seq)
            for value in raw.values():
                value.update({'xlevr.sample_receive_time_s': time.monotonic(),
                              'xlevr.verification_tracking': not untracked_idle})
            return raw
        self.runtime.teleoperator.input.get_hand_actions.side_effect = controllers
        self.session.options.update(max_sample_gap_s=.15, max_robot_age_s=.15,
                                    max_gripper_age_s=.20, max_clock_drift_s=.30)
        self.session._start_sampling()
        deadline = time.monotonic()+1
        while self.session.ready_sample is None and time.monotonic() < deadline: time.sleep(.005)
        self.assertIsNotNone(self.session.ready_sample)

    def test_90hz_controls_and_independent_30hz_samples_with_slow_feedback(self):
        self.session.backend = 'lerobot'
        self.session.sampling_mode = 'background_30hz'
        self._background_inputs()
        stop = threading.Event()
        cache = [self.robot.get_feedback_snapshot(None)]
        requests = []
        def poll_feedback():
            due = time.monotonic()
            while not stop.wait(max(0., due-time.monotonic())):
                time.sleep(.020)  # GET itself exceeds the 11.1ms control budget.
                now = time.monotonic(); requests.append(now)
                cache[0] = dict(has_error=False, received_monotonic_s=now,
                    controller_feedback_sequence=len(requests), arms={
                        'left': dict(ee_id=0, joints=[.2]*6, received_monotonic_s=now),
                        'right': dict(ee_id=1, joints=[.1]*6, received_monotonic_s=now)})
                due = max(due+1/30, time.monotonic())
        self.robot.get_feedback_snapshot.side_effect = lambda _: cache[0]
        self.robot.get_frame.side_effect = AssertionError('Control/recording must not issue GET')
        worker = threading.Thread(target=poll_feedback, daemon=True); worker.start()
        try:
            self.session.command('start'); self.settle()
            begin = next_tick = time.monotonic(); calls = []
            first_request = len(requests)
            while time.monotonic()-begin < 2.:
                self.seq += 1
                raw = hands(self.seq)
                self.session.before_step(raw, actions())
                self.session.after_step()
                calls.append(time.monotonic())
                next_tick += 1/90
                time.sleep(max(0., next_tick-time.monotonic()))
            controls_duration = time.monotonic()-begin
            feedback_count = len(requests)-first_request
            self.assertEqual(self.session.state, 'recording', self.session.error)
            self.assertGreater(len(calls), 150)
            self.assertLess(len(calls), 200)
            self.assertGreater(self.session.frames, 45)
            self.assertLess(self.session.frames, 75)
            self.assertGreater(feedback_count, 45)
            self.assertLess(feedback_count, 75)
            self.session.command('save')
            deadline = time.monotonic()+3
            while self.session.state != 'idle' and time.monotonic()<deadline:
                self.session.after_step(); time.sleep(.005)
            self.assertEqual(self.session.state, 'idle', self.session.error)
            self.session._stop_sampling(); self.session.close()
        finally:
            stop.set(); worker.join(1.)
        rows = [json.loads(line) for line in next((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')).read_text().splitlines()]
        duration = rows[-1]['capture_monotonic_s']-rows[0]['capture_monotonic_s']
        measured_hz = (len(rows)-1)/duration
        self.assertGreater(measured_hz, 25.)
        self.assertLess(measured_hz, 35.)
        self.assertTrue(all(row['details']['nominal_capture_hz'] == 30 for row in rows))
        self.assertTrue(all(row['details']['robot_received_monotonic_s'] <= row['capture_monotonic_s'] for row in rows))
        self.assertTrue(any(row['details']['controller_feedback_sequence'] for row in rows))
        print(f'RATE TEST: controls={len(calls)/controls_duration:.2f}Hz capture={measured_hz:.2f}Hz feedback={feedback_count/controls_duration:.2f}Hz')

    def test_slow_add_frame_does_not_block_control_or_lose_finish_button(self):
        self.session.backend = 'lerobot'; self.session.workflow = 'lerobot'
        self._background_inputs()
        entered = threading.Event(); release = threading.Event()
        append = self.writer.append
        def slow_append(*args, **kwargs):
            entered.set(); release.wait(2.)
            return append(*args, **kwargs)
        with patch.object(self.writer, 'append', side_effect=slow_append):
            self.session.command('start'); self.settle()
            self.assertTrue(entered.wait(1.))
            worst = 0.
            # Real modifier/neutral/direction sequence, while append stays blocked.
            for kw in ({}, {'modifier': True}, {'modifier': True, 'x': 1.}, {}):
                self.seq += 1; before = time.monotonic()
                self.session.before_step(hands(self.seq, **kw), actions())
                self.session.after_step()
                worst = max(worst, time.monotonic()-before)
            self.assertLess(worst, .05)
            self.assertFalse(self.session.commands.empty())
            self.assertEqual(self.session.state, 'recording')
            release.set()
            deadline = time.monotonic()+1
            while self.session.state != 'review' and time.monotonic()<deadline:
                self.session.after_step(); time.sleep(.005)
            self.assertEqual(self.session.state, 'review')
            self.assertGreaterEqual(self.session.frames, 1)
        self.session._stop_sampling()

    def test_inflight_append_never_delays_reset_cancel_or_overwrites_fault(self):
        self.session.backend = 'lerobot'; self.session.workflow = 'lerobot'
        self._background_inputs()
        entered = threading.Event(); release = threading.Event()
        append = self.writer.append
        def slow_append(*args, **kwargs):
            entered.set(); release.wait(2.)
            return append(*args, **kwargs)
        with patch.object(self.writer, 'append', side_effect=slow_append):
            self.session.command('start'); self.settle()
            self.assertTrue(entered.wait(1.))
            # An existing m_init remains exclusive; cancellation must not wait
            # for disk IO. Mimic the actual resetting state without motion.
            self.session.state = 'resetting'
            self.session._reset_return_state = 'recording'
            self.session.reset_thread = Mock(is_alive=lambda: True)
            self.session.commands.put_nowait('down')
            self.session.after_step()
            self.assertTrue(self.session.reset_cancel.is_set())
            self.session.reset_thread = None
            self.session.state = 'recording'
            self.session.record['max_episode_frames'] = 1
            self.session.fail('camera failed during append')
            release.set()
            deadline = time.monotonic()+1
            while self.session._append_inflight and time.monotonic()<deadline: time.sleep(.005)
            self.assertEqual(self.session.state, 'failed')
            self.assertIn('camera failed', self.session.error)
        self.session._stop_sampling()

    def test_background_sampler_keeps_real_frames_during_300ms_main_thread_stop(self):
        self._background_inputs()
        self.session.command('start'); self.settle()
        time.sleep(.1)
        first = self.session.frames
        # Simulate native stop/query waiting on the control thread. The cached
        # measured feedback source keeps updating, as it does during relay stop polls.
        with patch.object(self.robot, 'get_frame', side_effect=AssertionError('New robot GET')):
            time.sleep(.3)
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertGreaterEqual(self.session.frames-first, 7)
        self.session.command('save'); self.settle()
        self.session._stop_sampling()
        self.session.close()
        report = validate_dataset(self.root)
        self.assertEqual(report['episodes'], 1)
        self.assertGreater(report['frames'], 8)

    def test_background_sampler_never_refreshes_a_frozen_robot_timestamp(self):
        self._background_inputs(frozen_robot=True)
        self.session.command('start'); self.settle()
        deadline = time.monotonic()+1
        while self.session.state != 'failed' and time.monotonic() < deadline: time.sleep(.005)
        self.assertEqual(self.session.state, 'failed')
        self.assertIn('feedback missing/stale', self.session.error)
        count = self.session.frames
        time.sleep(.08)
        self.assertEqual(self.session.frames, count)
        self.session.command('save')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_resting_untracked_hands_do_not_invalidate_actual_robot_recording(self):
        self._background_inputs(untracked_idle=True)
        self.session.command('start'); self.settle()
        time.sleep(.25)
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertGreaterEqual(self.session.frames, 6)

    def test_short_gripper_reply_gap_preserves_original_timestamp_and_recovers(self):
        self._background_inputs()
        source = self.channels['left'].snapshot.side_effect
        stamp = time.monotonic()
        last = source()
        def delayed_reply():
            if time.monotonic()-stamp < .1:
                return dict(last, communication_error='temporary missing reply')
            return source()
        self.channels['left'].snapshot.side_effect = delayed_reply
        self.session.command('start'); self.settle()
        time.sleep(.18)
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertGreaterEqual(self.session.frames, 4)
        self.session.command('save'); self.settle(); self.session.close()
        files = list((self.root/'meta/xlevr_diagnostics').rglob('*.jsonl'))
        self.assertEqual(len(files), 1)
        rows = [json.loads(line) for line in files[0].read_text().splitlines()]
        stamps = [row['details']['grippers']['left']['feedback_monotonic_s'] for row in rows]
        self.assertIn(last['feedback_monotonic_s'], stamps)
        self.assertTrue(any(value > last['feedback_monotonic_s'] for value in stamps))

    def test_shared_native_stop_350ms_records_continuously_and_saves_valid_episode(self):
        from test_a10_vr_shared_control import DualSharedIntegrationTest
        from test_a10_operator import status_reply, dual_motion
        relay = DualSharedIntegrationTest()
        relay.setUp()
        self.addCleanup(relay.doCleanups)
        robot = relay.client('teleop')
        self.runtime.robot = robot
        self.session._feedback_reader = robot.open_feedback_reader()
        robot.start_follower('cartesian')
        def controllers():
            self.seq += 1
            raw = hands(self.seq)
            for value in raw.values():
                value.update({'xlevr.sample_receive_time_s': time.monotonic(),
                              'xlevr.verification_tracking': True})
            return raw
        self.runtime.teleoperator.input.get_hand_actions.side_effect = controllers
        self.session.options.update(max_sample_gap_s=.15, max_robot_age_s=.15,
                                    max_gripper_age_s=.20, max_clock_drift_s=.30)
        self.session._start_sampling()
        deadline = time.monotonic()+1
        while self.session.ready_sample is None and time.monotonic()<deadline: time.sleep(.005)
        self.assertIsNotNone(self.session.ready_sample)
        self.session.command('start'); self.settle()
        stopped_after = time.monotonic()+.35
        def feedback():
            reply = status_reply(active=relay.controller.active, mode=relay.controller.mode)
            reply['ret_context']['motion_msg'] = dual_motion()
            reply['ret_context']['robot_msg']['motion'] = 'Running' if time.monotonic()<stopped_after else 'Stopped'
            return reply
        relay.controller.command_replies['get'] = feedback
        robot.stop_follower()  # Real RPC, one robot socket, many in-stop GET replies.
        self.assertEqual(self.session.state, 'recording', self.session.error)
        self.assertGreaterEqual(self.session.frames, 8)
        self.assertEqual(relay.controller.max_connections, 1)
        self.session.command('save'); self.settle()
        self.session.close()
        self.assertEqual(validate_dataset(self.root)['episodes'], 1)

    def test_input_pause_invalidates_recording_blocks_start_reset_and_keeps_discard_available(self):
        self.start()
        count = self.session.frames
        self.runtime.input_pause_status = dict(state='release_required', reason='spike_pending')
        self.runtime.input_pause_hud = lambda: dict(type='hud', phase='input_paused', error='',
            hint='LEFT HAND: spike_pending', pending=None,
            controls=dict(prompt='RELEASE BOTH GRIPS/TRIGGERS + CENTER STICKS'))
        self.session.input_paused('Dual VR right: spike_pending')
        self.update()
        self.assertEqual(self.session.state, 'failed')
        self.assertEqual(self.session.frames, count)
        self.session.command('save')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'], 0)
        payload = self.runtime.teleoperator.send_feedback.call_args.args[0]
        self.assertEqual(payload['phase'], 'input_paused')
        self.assertIn('INVALID EPISODE', payload['hint'])
        self.assertIn('RELEASE BOTH', payload['controls']['prompt'])
        self.session.command('discard'); self.session.command('discard'); self.settle()
        self.assertEqual(self.session.state, 'idle')
        self.session.command('start'); self.assertIsNone(self.session.future)
        self.session.command('reset'); self.assertIsNone(self.session.pending)
        self.assertFalse(self.session._hud_controls()['directions']['up']['enabled'])
        self.runtime.input_pause_status = None
        self.update(); self.start()
        self.assertEqual(self.session.state, 'recording')

    def test_input_pause_during_async_start_cannot_publish_a_continuous_episode(self):
        self.writer.start(self.session.task)
        future = Future()
        self.session.future, self.session.future_operation = future, 'starting'
        self.session.state = 'starting'
        self.session.input_paused('VR input fault while starting')
        future.set_result(None)
        self.session._poll_future()
        self.assertEqual(self.session.state, 'failed')
        self.assertEqual(self.session.frames, 0)
        self.assertIn('input fault', self.session.error)
        self.session.command('save')
        self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'], 0)

    def test_camera_stale_fault_prevents_publication(self):
        self.start()
        self.session.cameras.snapshot.side_effect=RuntimeError('left wrist unplugged')
        self.update(); self.assertEqual(self.session.state,'failed')
        self.session.command('save'); self.assertIsNone(self.session.future)
        self.assertEqual(self.writer.info['total_episodes'],0)
        self.session.command('discard'); self.session.command('discard'); self.settle()
        self.assertEqual(self.session.state,'idle')

    def test_failed_episode_discards_after_two_real_gradual_left_gestures(self):
        self.start()
        self.runtime.input_pause_status = dict(state='release_required', reason='control_gap')
        self.runtime.input_pause_hud = lambda: dict(type='hud', phase='input_paused', error='',
            hint='CONTROL LOOP DELAY', pending=None,
            controls=dict(prompt='RELEASE BOTH GRIPS/TRIGGERS + CENTER STICKS'))
        self.session.input_paused('Control interrupted: control_gap')
        self.update(modifier=True)
        self.update(modifier=True, x=-.205)
        self.update(modifier=True, x=-.93, y=.37)
        self.assertEqual(self.session.pending, 'discard')
        self.assertEqual(self.session.state, 'failed')
        self.assertEqual(self.writer.info['total_episodes'], 0)
        self.update(modifier=True)  # Recenter, then repeat within confirmation time.
        self.update(modifier=True, x=-.25)
        self.update(modifier=True, x=-.9, y=.4)
        self.settle()
        self.assertEqual(self.session.state, 'idle')
        self.assertIsNone(self.writer.active)
        self.assertEqual(self.writer.info['total_episodes'], 0)
        self.session.command('start')
        self.assertIsNone(self.session.future)  # Discarding data does not rearm motion.
        self.runtime.input_pause_status = None
        self.update(modifier=True)
        self.update(modifier=True, x=.9)
        self.settle()
        self.assertEqual(self.session.state, 'recording')

    def test_gap_and_skew_are_not_silently_accepted(self):
        self.start(); self.session._last_sample-=100
        self.update(); self.assertEqual(self.session.state,'failed')
        self.session.command('discard'); self.session.command('discard'); self.settle()
        self.channels['left'].snapshot.side_effect=lambda: dict(actual_raw=4000,feedback_monotonic_s=time.monotonic()-1,
                                                               enabled=True,error=None,alarm_a=0,alarm_b=0)
        self.update(); self.session.command('start')
        self.assertEqual(self.session.state,'idle')

    def test_save_is_asynchronous_and_start_is_rejected_while_saving(self):
        self.start(); entered=threading.Event(); release=threading.Event(); save=self.writer.save
        def slow(): entered.set(); release.wait(2); return save()
        with patch.object(self.writer,'save',slow):
            self.session.command('save'); self.assertTrue(entered.wait(1))
            try:
                self.update(); self.session.command('start')
                self.assertEqual(self.session.state,'saving')
            finally: release.set(); self.settle()
        self.assertEqual(self.session.state,'idle')

    def test_gripper_gate_only_changes_physical_left(self):
        self.session.before_step(hands(10),actions())
        action=actions(); self.session.before_step(hands(11,modifier=True,x=1),action)
        self.assertEqual(action['right']['gripper.pos'],0.)  # physical left = internal right
        self.assertEqual(action['left']['gripper.pos'],.6)
        action=actions(); self.session.before_step(hands(12,x=1),action)
        self.assertEqual(action['right']['gripper.pos'],0.)
        action=actions(); self.session.before_step(hands(13),action)
        self.assertEqual(action['right']['gripper.pos'],.6)

    def test_confirmation_expiry_disconnect_and_motion_guard(self):
        self.start(); self.session.command('discard'); self.session.pending_at-=20
        self.update(); self.assertIsNone(self.session.pending)
        self.session.command('discard'); self.update(age=1)
        self.assertIsNone(self.session.pending); self.assertEqual(self.session.state,'failed')
        self.session.command('reset'); self.assertIsNone(self.session.reset_thread)

    def test_reset_requires_release_and_second_confirmation_is_exclusive(self):
        self.update(grip=True); self.session.command('reset'); self.assertIsNone(self.session.pending)
        self.update(); self.session.command('reset'); self.assertEqual(self.session.pending,'reset')
        entered=threading.Event(); release=threading.Event()
        def init(robot, cancel_event):
            entered.set(); release.wait(2)
            if cancel_event.is_set(): raise InterruptedError('cancelled')
            return dict(completed=True)
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init',init):
            self.session.command('reset'); self.assertTrue(entered.wait(1))
            try:
                action=actions()
                self.assertTrue(self.session.before_step(hands(20),action))
                self.assertTrue(all(a['gripper.pos']==0 for a in action.values()))
                self.session.command('start'); self.assertEqual(self.session.state,'resetting')
            finally: release.set(); self.session.reset_thread.join(2)
            self.session._poll_reset()
        self.assertEqual(self.session.state,'idle')
        self.assertTrue(self.runtime._dual_require_release)
        self.robot.wait_stopped.assert_called()

    def test_reset_cancel_on_stale_input_latches_failure(self):
        entered=threading.Event()
        def init(robot,cancel_event):
            entered.set(); cancel_event.wait(2); raise InterruptedError('cancelled')
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init',init):
            self.session.command('reset'); self.session.command('reset')
            self.assertTrue(entered.wait(1)); self.session.before_step(hands(30,age=1),actions())
            self.session.reset_thread.join(2); self.session._poll_reset()
        self.assertTrue(self.session.blocks_motion)
        self.robot.stop_all_actions.assert_called_once()

    def test_real_runtime_hooks_capture_and_block_robot_during_reset(self):
        from test_vr_dual_runtime import configuration, DualRobot, HandInput, hand
        from fluxvla.engines.operators.vr_operator import VROperator
        from fluxvla.engines.operators.vr.runtime import DualVRTeleopRuntime
        class Robot(DualRobot):
            camera_operator = None
            def get_frame(self):
                frame = super().get_frame()
                for name, arm in frame['arms'].items():
                    arm.update(ee_id=0 if name=='left' else 1, joints=[.2 if name=='left' else .1]*6)
                return frame
        cfg=configuration()
        vr=VROperator(**{k:v for k,v in cfg.input.items() if k!='type'})
        vr.input=HandInput(); vr.send_feedback=Mock()
        robot=Robot()
        rt=DualVRTeleopRuntime(vr,robot,execute=True,control_mode='reference',
                              max_position_step_m=.005,max_rotation_step_rad=.015)
        for channel in self.channels.values(): channel.options={'input_timeout_s':.25}
        rt._zx_channels=self.channels
        rt.recording=self.session; self.session.runtime=rt; self.session._last_hud=0.
        rt.connect()
        try:
            def tick(seq):
                vr.input.rows={side:hand(seq,False) for side in ('left','right')}
                rt.step()
            tick(1)
            self.session.commands.put('start'); tick(2); self.settle()
            tick(3); tick(4)
            self.assertEqual(self.session.frames,2)
            self.session.commands.put('save'); tick(5); self.settle()
            self.assertEqual(self.writer.info['total_episodes'],1)
            self.session.reset_failed=True
            with patch.object(robot,'get_frame',side_effect=AssertionError('robot IO while reset owns it')):
                tick(6)
            self.assertGreaterEqual(vr.send_feedback.call_count,1)
        finally:
            rt.close()

    def test_quit_retains_unfinished_episode(self):
        self.start(); self.session.command('quit'); self.session.close()
        self.assertTrue(self.session.quit_requested)
        self.assertTrue((self.root/'.recording/active').exists())
        self.assertFalse((self.root/'data').exists())


class InitCancelTest(unittest.TestCase):
    def test_external_cancel_is_not_cleared_during_prepare(self):
        event=threading.Event()
        robot=Mock(command_mode='cartesian',follower_stop_pending=False)
        robot._init_motion_lock=threading.Lock()
        robot._traj_stop_event=threading.Event()
        robot._prepare_init_motion.side_effect=event.set
        robot._read_init_joints.return_value={'arm':np.zeros(6)}
        configure_init_motion(robot,joint_dofs={'arm':6},init_position={'arm':[0]*6},init_motion={})
        with self.assertRaises(InterruptedError): move_to_init(robot,cancel_event=event)
        robot.start_follower.assert_not_called(); robot.send_joints.assert_not_called()
        self.assertFalse(robot._init_motion_lock.locked())


class EntryTest(unittest.TestCase):
    def test_recording_cadence_is_independent_only_in_background_mode(self):
        from scripts.teleoperate import main
        args=['--config', str(ROOT/'configs/a10/a10_dual_arm_vr_teleop.py'),
              '--record-config', str(ROOT/'configs/a10/a10_dual_arm_record.py'), '--preview']
        with patch('fluxvla.engines.operators.vr.recording.RecordingSession.connect', side_effect=AssertionError('IO')):
            self.assertEqual(main(args+['--cfg-options', 'teleop.fps=90']), 0)
            with self.assertRaises(SystemExit):
                main(args+['--record-options', 'capture.sampling_mode=control_loop',
                           '--cfg-options', 'teleop.fps=90'])
            self.assertEqual(main(args+['--record-options', 'capture.sampling_mode=control_loop',
                                       '--cfg-options', 'teleop.fps=30']), 0)
            with self.assertRaises(SystemExit):
                main(args+['--record-options', 'record.fps=90'])
        from mmengine import Config
        config = Config.fromfile(str(ROOT/'configs/a10/a10_dual_arm_record.py'))
        self.assertTrue(all(c.fps == 30 for c in config.record.cameras.values()))

    def test_button_preview_is_robot_free_and_live_requires_grippers(self):
        from scripts.teleoperate import main
        args=['--config',str(ROOT/'configs/a10/a10_dual_arm_vr_teleop.py'),
              '--record-config',str(ROOT/'configs/a10/a10_dual_arm_record.py'),'--preview']
        with patch('fluxvla.engines.operators.vr.recording.RecordingSession.connect',side_effect=AssertionError('IO')):
            self.assertEqual(main(args+['--check-record-buttons']),0)
            self.assertEqual(main(args),0)  # Live + grippers is now the explicit site default.
            self.assertEqual(main(args+['--cfg-options','teleop.vr_only=False','teleop.observe_only=False',
                                       'teleop.operator.allow_motion=True','teleop.zx_grippers.enabled=True']),0)


if __name__=='__main__': unittest.main()
