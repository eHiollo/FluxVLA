"""Export regressions using small synthetic episodes; no hardware or network."""
import fcntl
import json
import contextlib
import io
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import av

from fluxvla.datasets import a10_dataset_export as exporter
from fluxvla.datasets.a10_recording_io import export_openpi_dataset, validate_dataset
from test_a10_recording import make_fixture, CAMERAS


class DatasetExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.source = self.base / 'source'
        self.output = self.base / 'output'
        self.stage = self.base / '.output.export'
        make_fixture(self.source, frames=4)

    def interrupt(self):
        def progress(event):
            if event['phase'] == 'video_done' and event['completed'] == 1:
                raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            export_openpi_dataset(self.source, self.output, progress=progress)
        self.assertFalse(self.output.exists())
        self.assertTrue((self.stage / '.export-state.json').is_file())

    def test_auto_copies_independent_clips_byte_for_byte_and_writes_v21_metadata(self):
        events = []
        info_path = self.source / 'meta/info.json'
        source_info = json.loads(info_path.read_text())
        source_info['features']['action']['fps'] = 30
        info_path.write_text(json.dumps(source_info))
        with patch.object(exporter, '_transcode', side_effect=AssertionError('unnecessary transcode')):
            result = export_openpi_dataset(self.source, self.output, progress=events.append)
        self.assertEqual(result['copied'], 4)
        self.assertEqual(result['transcoded'], 0)
        self.assertEqual(result['frames'], 8)
        self.assertTrue(result['videos_checked'])
        info = json.loads((self.output / 'meta/info.json').read_text())
        self.assertNotIn('data_files_size_in_mb', info)
        self.assertNotIn('video_files_size_in_mb', info)
        self.assertNotIn('fps', info['features']['action'])
        self.assertEqual(info['total_chunks'], 1)
        self.assertEqual(info['total_videos'], 4)
        for episode in range(2):
            for old, new in [('base_0_rgb', 'top'), ('right_wrist_0_rgb', 'right')]:
                original = self.source / f'videos/observation.images.{old}/chunk-000/file-{episode:03d}.mp4'
                actual = self.output / f'videos/chunk-000/observation.images.{new}/episode_{episode:06d}.mp4'
                self.assertEqual(original.read_bytes(), actual.read_bytes())
        self.assertIn('validate_output', {event['phase'] for event in events})
        self.assertEqual(events[-1]['phase'], 'complete')
        self.assertFalse(self.stage.exists())
        self.assertFalse((self.output / '.export-state.json').exists())

    def test_force_h264_runs_encoder(self):
        with patch.object(exporter, '_transcode', wraps=exporter._transcode) as transcode:
            result = export_openpi_dataset(self.source, self.output, video_mode='h264')
        self.assertEqual(transcode.call_count, 4)
        self.assertEqual(result['transcoded'], 4)
        self.assertEqual(result['copied'], 0)
        for path in (self.output / 'videos').rglob('*.mp4'):
            with av.open(str(path)) as video:
                self.assertEqual(video.streams.video[0].codec_context.name, 'h264')

    def test_resume_reuses_finished_files_after_interrupt(self):
        self.interrupt()
        completed = json.loads((self.stage / '.export-state.json').read_text())['completed']
        video = next(path for path in completed if path.startswith('videos/'))
        before = (self.stage / video).stat().st_mtime_ns
        result = export_openpi_dataset(self.source, self.output, resume=True)
        self.assertEqual(result['resumed'], 1)
        self.assertEqual((self.output / video).stat().st_mtime_ns, before)
        self.assertEqual(validate_dataset(self.output)['frames'], 8)

    def test_resume_regenerates_corrupted_file_even_when_size_is_unchanged(self):
        self.interrupt()
        video = next((self.stage / 'videos').rglob('*.mp4'))
        rel = video.relative_to(self.stage)
        original = video.read_bytes()
        video.write_bytes(b'x' + original[1:])
        result = export_openpi_dataset(self.source, self.output, resume=True)
        self.assertEqual(result['resumed'], 0)
        self.assertEqual((self.output / rel).read_bytes(), original)

    def test_resume_rejects_changed_source_or_options(self):
        self.interrupt()
        with self.assertRaisesRegex(ValueError, '转换参数'):
            export_openpi_dataset(self.source, self.output, video_mode='h264', resume=True)
        info = self.source / 'meta/info.json'
        info.write_text(info.read_text() + '\n')
        with self.assertRaisesRegex(ValueError, '源文件'):
            export_openpi_dataset(self.source, self.output, resume=True)
        self.assertFalse(self.output.exists())

    def test_existing_staging_requires_explicit_resume(self):
        self.interrupt()
        with self.assertRaisesRegex(FileExistsError, '--resume'):
            export_openpi_dataset(self.source, self.output)

    def test_unknown_staging_directory_is_not_modified(self):
        self.stage.mkdir()
        marker = self.stage / 'important'
        marker.write_text('preserve')
        with self.assertRaisesRegex(ValueError, '检查点'):
            export_openpi_dataset(self.source, self.output, resume=True)
        self.assertEqual([p.name for p in self.stage.iterdir()], ['important'])

    def test_version_is_checked_before_video_decode_or_output_write(self):
        info = self.source / 'meta/info.json'
        value = json.loads(info.read_text()); value['codebase_version'] = 'v2.1'
        info.write_text(json.dumps(value))
        with patch.object(exporter, 'validate_dataset') as validate:
            with self.assertRaisesRegex(ValueError, '当前标记为 v2.1'):
                export_openpi_dataset(self.source, self.output)
        validate.assert_not_called()
        self.assertFalse(self.stage.exists())

    def test_mixed_v21_files_are_rejected_without_modifying_source(self):
        extra = self.source / 'data/chunk-000/episode_000000.parquet'
        shutil.copy2(self.source / 'data/chunk-000/file-000.parquet', extra)
        original = extra.read_bytes()
        with self.assertRaisesRegex(ValueError, '混合格式'):
            export_openpi_dataset(self.source, self.output)
        self.assertEqual(extra.read_bytes(), original)
        self.assertFalse(self.stage.exists())

    def test_staging_cannot_alias_source(self):
        # destination '.source.export' would put its staging folder at source.
        moved = self.base / '.output.export'
        self.source.rename(moved)
        with self.assertRaisesRegex(ValueError, '暂存目录'):
            export_openpi_dataset(moved, self.output, resume=True)
        self.assertFalse((moved / '.export.lock').exists())

    def test_publish_does_not_replace_even_an_empty_existing_directory(self):
        self.stage.mkdir()
        (self.stage / 'marker').write_text('export')
        self.output.mkdir()
        with self.assertRaises(FileExistsError):
            exporter._publish(self.stage, self.output)
        self.assertEqual(list(self.output.iterdir()), [])
        self.assertEqual((self.stage / 'marker').read_text(), 'export')

    def test_output_validation_failure_keeps_checkpoint_and_never_publishes(self):
        validate = exporter.validate_dataset
        def fail_output(root, **kwargs):
            if root != self.source:
                raise ValueError('test invalid video')
            return validate(root, **kwargs)
        with patch.object(exporter, 'validate_dataset', side_effect=fail_output):
            with self.assertRaisesRegex(ValueError, 'invalid video'):
                export_openpi_dataset(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertTrue((self.stage / '.export-state.json').is_file())
        # Once verification succeeds, the complete staged files can be resumed.
        self.assertEqual(export_openpi_dataset(self.source, self.output, resume=True)['resumed'], 4)

    def test_cli_interrupt_returns_130_and_explains_resume(self):
        from scripts import export_a10_openpi_dataset as cli
        from fluxvla.datasets import a10_recording_io
        output, errors = io.StringIO(), io.StringIO()
        with patch.object(a10_recording_io, 'export_openpi_dataset', side_effect=KeyboardInterrupt), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            code = cli.main(['--source', str(self.source), '--output', str(self.output)])
        self.assertEqual(code, 130)
        self.assertIn('--resume', errors.getvalue())
        self.assertNotIn('Traceback', errors.getvalue())

    def test_concurrent_resume_is_rejected(self):
        self.interrupt()
        with (self.stage / '.export.lock').open('a+') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError, '已有导出任务'):
                export_openpi_dataset(self.source, self.output, resume=True)


if __name__ == '__main__':
    unittest.main()
