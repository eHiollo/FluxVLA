"""Actual native LeRobot IO with synthetic RGB/feedback only; no device access."""
import json
import multiprocessing
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from fluxvla.datasets.a10_recording_schema import get_schema
from fluxvla.datasets.a10_recording_io import export_openpi_dataset, validate_dataset
from fluxvla.datasets.lerobot_episode_writer import LeRobotEpisodeWriter
from test_vr_recording import SessionTests
from test_vr_dual_runtime import DualRuntimeTest


_previous_start_method = None


def setUpModule():
    global _previous_start_method
    _previous_start_method = multiprocessing.get_start_method(allow_none=True)
    multiprocessing.set_start_method('spawn', force=True)


def tearDownModule():
    multiprocessing.set_start_method(_previous_start_method, force=True)


CAMERAS = {n: dict(width=64, height=64) for n in get_schema('a10_dual_v1').cameras}


class NativeWriterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'native'

    def writer(self, **kwargs):
        writer = LeRobotEpisodeWriter(self.root, CAMERAS, synthetic=True, **kwargs)
        self.addCleanup(writer.close)
        return writer

    def frames(self, writer, count=4):
        for i in range(count):
            state = np.array([.1]*6+[i/10]+[-.2]*6+[.9-i/10], dtype=np.float32)
            images = {name: np.full((64,64,3), 30+60*c+i, np.uint8)
                      for c, name in enumerate(CAMERAS)}
            writer.append(state, state.copy(), images, capture_monotonic_s=100+i/30,
                          diagnostics={'actual_feedback': True})

    def test_native_save_discard_two_tasks_resume_and_openpi_export(self):
        writer = self.writer()
        # Verify a real installed library, not a facade over the old custom writer.
        self.assertEqual(writer.dataset.__class__.__module__, 'lerobot.datasets.lerobot_dataset')
        writer.start('discarded'); self.frames(writer); writer.discard()
        self.assertEqual(writer.info['total_episodes'], 0)
        for ep in range(2):
            writer.start(f'task {ep}'); self.frames(writer)
            self.assertEqual(writer.save(), ep)
        writer.close()
        first_video = next((self.root/'videos').rglob('*.mp4'))
        before = first_video.read_bytes()
        writer = self.writer(resume=True)
        writer.start('third'); self.frames(writer)
        self.assertEqual(writer.save(), 2)
        writer.close()
        self.assertEqual(before, first_video.read_bytes())
        report = validate_dataset(self.root)
        self.assertEqual((report['episodes'], report['frames']), (3, 12))
        sidecar = self.root/'meta/xlevr_diagnostics/episode_000002.jsonl'
        rows = [json.loads(line) for line in sidecar.read_text().splitlines()]
        self.assertEqual(len(rows), 4)
        self.assertTrue(rows[0]['details']['actual_feedback'])
        out = Path(self.temp.name)/'openpi'
        export_openpi_dataset(self.root, out)
        self.assertEqual(validate_dataset(out)['frames'], 12)
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        ds = LeRobotDataset('local/a10_dual_record', root=self.root, video_backend='pyav')
        try:
            self.assertEqual(ds.num_episodes, 3)
            np.testing.assert_allclose(ds.hf_dataset[0]['observation.state'], [.1]*6+[0]+[-.2]*6+[.9])
        finally:
            ds.finalize()

    def test_recording_entrypoint_selects_spawn_before_runtime_construction(self):
        from scripts.teleoperate import main
        root = Path(__file__).resolve().parents[1]
        args = ['--config', str(root/'configs/a10/a10_dual_vr.py'),
                '--record-config', str(root/'configs/a10/a10_dual_arm_record.py')]
        class ConstructionReached(Exception): pass
        for flags, configure in (([], True), (['--preview'], False), (['--check-record-buttons'], False)):
            events = []
            def construct(_):
                events.append('runtime'); raise ConstructionReached()
            with self.subTest(flags=flags), \
                    patch('multiprocessing.set_start_method', side_effect=lambda *a, **kw: events.append((a, kw))), \
                    patch('scripts.teleoperate.build_runtime', side_effect=construct):
                with self.assertRaises(ConstructionReached): main(args+flags)
            self.assertEqual(events, [(('spawn',), {'force': True}), 'runtime'] if configure else ['runtime'])

    def test_native_dataset_and_image_writer_are_reused_until_final_close(self):
        writer = self.writer()
        dataset, image_writer = writer.dataset, writer.dataset.image_writer
        with patch.object(dataset, 'finalize', wraps=dataset.finalize) as finalize, \
             patch.object(dataset, 'stop_image_writer', wraps=dataset.stop_image_writer) as stop_images, \
             patch.object(dataset, 'save_episode', wraps=dataset.save_episode) as save:
            for episode in range(3):
                writer.start(f'task {episode}'); self.frames(writer)
                self.assertEqual(writer.save(), episode)
                self.assertIs(writer.dataset, dataset)
                self.assertIs(writer.dataset.image_writer, image_writer)
            self.assertEqual(save.call_args_list, [unittest.mock.call()]*3)
            self.assertEqual(finalize.call_count, 3); stop_images.assert_not_called()
            writer.close(); writer.close()
            self.assertEqual(finalize.call_count, 4); stop_images.assert_called_once()
        report = validate_dataset(self.root)
        self.assertEqual((report['episodes'], report['frames']), (3, 12))

    def test_unfinished_episode_is_discarded_on_close_without_recover_option(self):
        writer = self.writer()
        writer.start('unfinished'); self.frames(writer); writer.close()
        self.assertFalse((self.root/'.recording/active').exists())
        writer = self.writer(resume=True)
        writer.start('after recovery'); self.frames(writer); writer.save(); writer.close()
        self.assertEqual(validate_dataset(self.root)['episodes'], 1)

    def test_failed_save_is_discarded_and_previous_saved_episode_remains_readable(self):
        writer = self.writer()
        writer.start('saved'); self.frames(writer); writer.save()
        saved_files = {str(p): p.read_bytes() for p in (self.root/'videos').rglob('*.mp4')}
        writer.start('interrupted save'); self.frames(writer)
        native_save = writer.dataset.save_episode
        def incomplete():
            native_save()  # Simulate a failure even after native metadata changed.
            raise OSError('disk failed')
        with patch.object(writer.dataset, 'save_episode', side_effect=incomplete):
            with self.assertRaisesRegex(OSError, 'disk failed'): writer.save()
        writer.discard()
        writer.close()
        self.assertFalse((self.root/'.recording/active').exists())
        self.assertEqual(validate_dataset(self.root)['episodes'], 1)
        self.assertEqual(saved_files, {str(p):p.read_bytes() for p in (self.root/'videos').rglob('*.mp4')})
        writer = self.writer(resume=True)
        writer.start('new'); self.frames(writer); writer.save(); writer.close()
        self.assertEqual(validate_dataset(self.root)['episodes'], 2)

    def test_startup_rolls_back_interrupted_save_before_accepting_new_episode(self):
        writer = self.writer()
        writer.start('saved'); self.frames(writer); writer.save()
        writer.start('interrupted'); self.frames(writer)
        with patch.object(writer.dataset, 'save_episode', side_effect=OSError('interrupted')):
            with self.assertRaises(OSError): writer.save()
        # Emulate process loss: release resources without running adapter cleanup.
        writer.dataset.stop_image_writer(); writer.dataset.finalize()
        writer._sidecar.close(); writer._lock.close(); writer._closed = True
        orphan = self.root/'videos/partial.mp4'; orphan.write_bytes(b'incomplete')
        writer = self.writer(resume=True)
        self.assertFalse(orphan.exists())
        self.assertFalse((self.root/'.recording/active').exists())
        writer.close()
        self.assertEqual(validate_dataset(self.root)['episodes'], 1)

    def test_old_backend_directory_wrong_identity_and_second_writer_are_rejected(self):
        writer = self.writer()
        with self.assertRaisesRegex(RuntimeError, 'already in use'): self.writer(resume=True)
        writer.close()
        with self.assertRaisesRegex(ValueError, 'same backend/contract'):
            self.writer(resume=True, device_metadata={'unexpected': True})
        contract = self.root/'meta/recording_contract.json'
        content = json.loads(contract.read_text()); content.pop('writer_backend')
        contract.write_text(json.dumps(content))
        with self.assertRaisesRegex(ValueError, 'NEW root'): self.writer(resume=True)

    def test_invalid_action_cannot_be_saved(self):
        writer = self.writer(); writer.start('bad frame')
        with self.assertRaisesRegex(ValueError, 'same-frame'):
            writer.append(np.zeros(14), np.ones(14), {})
        with self.assertRaisesRegex(RuntimeError, 'discard a failed episode'): writer.save()
        writer.discard()


class NativeSessionTests(SessionTests):
    save_timeout_s = 60

    def test_native_control_gap_keeps_same_measured_episode(self):
        self.use_native_writer()
        SessionTests.test_control_gap_pauses_motion_but_keeps_same_measured_episode(self)
        report = validate_dataset(self.root)
        self.assertEqual(report['episodes'], 1)

    def settle(self):
        if self.session.future:
            self.session.future.result(timeout=60)
        self.session._poll_future()

    def test_native_90hz_controls_preserve_30hz_dataset_with_slow_feedback(self):
        self.use_native_writer()
        SessionTests.test_90hz_controls_and_independent_30hz_samples_with_slow_feedback(self)
        self.session.close()  # Native Parquet footers are committed at session end.
        import pyarrow.parquet as pq
        table = pq.read_table(next((self.root/'data').rglob('*.parquet')))
        timestamps = table.column('timestamp').to_numpy()
        np.testing.assert_allclose(np.diff(timestamps), 1/30, atol=1e-6)
        rows = [json.loads(line) for line in next((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')).read_text().splitlines()]
        self.assertEqual(len(timestamps), len(rows))
        self.assertAlmostEqual(float(timestamps[-1]),
                               rows[-1]['capture_monotonic_s']-rows[0]['capture_monotonic_s'], delta=.10)

    def test_native_same_episode_survives_spike_pause_and_keeps_diagnostics(self):
        self.use_native_writer()
        SessionTests.test_spike_pause_keeps_same_episode_measured_frames_and_diagnostics(self)

    def test_native_drop_warning_records_and_preserves_alarm(self):
        self.use_native_writer()
        SessionTests.test_drop_warning_records_actual_samples_and_preserves_alarm_in_saved_sidecar(self)

    # Inherit the existing UI, m_init and lifecycle regressions below only via
    # targeted load_tests. New cases use its synthetic device setup.
    def use_native_writer(self):
        session = self.session
        self.writer.close()
        self.root = Path(self.temp.name)/'real-native'
        self.writer = LeRobotEpisodeWriter(self.root, CAMERAS, synthetic=True)
        session.writer = self.writer
        session.backend = 'lerobot'
        session.record.update(root=str(self.root), cameras=CAMERAS)
        session.cameras_config = CAMERAS
        self.runtime.robot.get_feedback_snapshot.side_effect = lambda _: self.runtime.last_frame
        session.cameras.snapshot.side_effect = lambda _: {
            n: dict(image=np.zeros((64,64,3), np.uint8), received_monotonic_s=time.monotonic(), sequence=self.seq)
            for n in CAMERAS}

    def test_native_writer_through_recording_ui_save_discard_and_next_episode(self):
        self.use_native_writer()
        session = self.session
        self.update(); self.start()
        session.command('discard'); session.command('discard'); self.settle()
        self.assertEqual(session.state, 'idle')
        for _ in range(2):
            self.update(); self.start()
            session.command('save'); self.settle()
            self.assertEqual(session.state, 'idle', session.error)
        session.close()
        self.assertEqual(validate_dataset(self.root)['episodes'], 2)

    def test_old_stage_workflow_with_actual_native_save_clear_and_next_episode(self):
        self.use_native_writer()
        session = self.session; session.workflow = 'lerobot'
        self.update(); self.start()
        session.command('right')
        self.assertEqual(session.state, 'review')
        self.assertEqual(self.writer.info['total_episodes'], 0)
        session.command('right'); self.settle()
        self.assertEqual(self.writer.info['total_episodes'], 1)
        session._saved_until = 0.
        self.update(); self.settle(); self.update(); self.update()
        self.assertEqual(session.state, 'recording')
        session.command('left'); session.command('right'); self.settle()
        self.update(); self.settle(); self.update(); self.update()
        self.assertEqual(self.writer.info['total_episodes'], 1)  # Re-record keeps the same episode index.
        session.command('stop'); self.settle(); session.close()
        self.assertTrue(session.quit_requested)
        self.assertEqual(validate_dataset(self.root)['episodes'], 1)

    def test_native_episode_contains_m_init_measurements_and_three_videos(self):
        import pyarrow.parquet as pq
        self.use_native_writer()
        session = self.session; session.workflow = 'lerobot'
        self.update(); self.start()
        release = threading.Event()
        def moving(*args, **kwargs): release.wait(3.)
        with patch('fluxvla.engines.operators.kaanh_commands.m_init.move_to_init', side_effect=moving):
            session.command('up'); session.command('up')
            try:
                self.assertTrue(session.capturing)
                self._background_inputs()  # Exercise the independent sampler too.
                initial = session.frames
                deadline = time.monotonic()+1.
                while session.frames < initial+3 and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertGreaterEqual(session.frames, initial+3)
                session._stop_sampling(); session._background_sampling = False
            finally:
                release.set(); session.reset_thread.join(2.)
            session._poll_reset()
        self.assertEqual(session.state, 'recording', session.error)
        self.update()
        session.command('right'); session.command('right'); self.settle(); session.close()
        self.assertEqual(self.writer.info['total_episodes'], 1)
        rows = [json.loads(l) for l in next((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')).read_text().splitlines()]
        self.assertGreaterEqual(sum(row['details']['control_phase']=='m_init' for row in rows), 3)
        phases = [row['details']['control_phase'] for row in rows]
        self.assertEqual((phases[0], phases[-1]), ('teleop', 'teleop'))
        self.assertEqual(pq.read_table(next(self.root.glob('data/**/*.parquet'))).num_rows, len(rows))
        self.assertEqual(len(list(self.root.glob('videos/**/*.mp4'))), 3)

    def test_control_loop_adds_one_frame_after_slow_stop_without_filling_gap(self):
        session = self.session
        session.backend = 'lerobot'
        self.runtime.robot.get_feedback_snapshot.side_effect = lambda _: self.runtime.last_frame
        self.start()
        count = session.frames
        session.options.update(max_sample_gap_s=.15, max_clock_drift_s=.3)
        session._last_sample -= .4
        session._first_sample -= .4
        self.update()
        self.assertEqual(session.state, 'recording', session.error)
        self.assertEqual(session.frames, count+1)
        self.assertEqual(session.sample_gaps, 1)
        session.command('save'); self.settle(); session.close()
        rows = [json.loads(l) for l in next((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')).read_text().splitlines()]
        self.assertGreater(rows[-1]['details']['capture_gap_s'], .4)

    def test_transient_measurement_age_skips_and_persistent_loss_invalidates(self):
        session = self.session
        session.backend = 'lerobot'
        self.runtime.robot.get_feedback_snapshot.side_effect = lambda _: self.runtime.last_frame
        self.start()
        count = session.frames
        session.cameras.snapshot.side_effect = lambda _: {
            n: dict(image=np.zeros((16,16,3), np.uint8), received_monotonic_s=time.monotonic()-1., sequence=1)
            for n in session.cameras_config}
        self.update()
        self.assertEqual((session.state, session.frames), ('recording', count))
        self.assertEqual(session.skipped_samples, 1)
        session._missing_since -= 1.1
        self.update()
        self.assertEqual(session.state, 'failed')
        self.assertIn('Measurements unavailable', session.error)


class LateInputPauseTests(DualRuntimeTest):
    def test_controller_internal_duplicate_age_uses_existing_pause_and_rearm(self):
        self.move()
        # Keep a duplicate sample that passes the early raw-age check; the
        # controller's own accumulated age expires later, as in the real log.
        sent_before = len(self.robot.targets)
        for _ in range(10):
            self.clock.value += .04
            self.runtime.step()
            if self.runtime.input_pause_status is not None:
                break
        self.assertEqual(self.runtime.input_pause_status['reason'], 'input_stale')
        self.assertTrue(self.runtime.input_pause_status['stopped'])
        self.assertTrue(self.vr.is_connected)
        self.assertEqual(self.robot.events.count('stop'), 1)
        sent_at_pause = len(self.robot.targets)
        self.assertGreaterEqual(sent_at_pause, sent_before)
        self.tick(True, True)
        self.assertEqual(len(self.robot.targets), sent_at_pause)
        self.wait_input_ready()
        for _ in range(3): self.tick(True, True)
        self.assertEqual(self.robot.events.count('start'), 2)


def load_tests(loader, tests, pattern):
    suite = loader.loadTestsFromTestCase(NativeWriterTests)
    for cls in (NativeSessionTests, LateInputPauseTests):
        for name in cls.__dict__:
            if name.startswith('test_'):
                suite.addTest(cls(name))
    return suite
