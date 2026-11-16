"""Immutable historical diagnostics are retained without repeated backup copies."""
import json
import unittest
from unittest.mock import patch

import test_lerobot_recording as native
from fluxvla.datasets.a10_recording_io import validate_dataset

setUpModule = native.setUpModule
tearDownModule = native.tearDownModule


class MetadataBackupTests(native.NativeWriterTests):
    def history(self, writer):
        writer.start('saved'); self.frames(writer, count=2); writer.save()
        paths = {p: p.read_bytes() for p in (self.root/'meta/xlevr_diagnostics').glob('*.jsonl')}
        writer.start('next'); self.frames(writer, count=2)
        return paths

    def check_snapshot(self, writer, paths):
        baseline = writer.active/'baseline_meta'
        self.assertFalse((baseline/'xlevr_diagnostics').exists())
        for name in ('info.json', 'stats.json', 'tasks.parquet'):
            self.assertTrue((baseline/name).is_file(), name)
        self.assertTrue(list((baseline/'episodes').rglob('*.parquet')))
        files = set(json.loads((writer.active/'baseline.json').read_text())['files'])
        for path in paths:
            self.assertIn(str(path.relative_to(self.root)), files)

    def test_save_excludes_old_diagnostics_and_keeps_both_episodes(self):
        writer = self.writer(); paths = self.history(writer)
        save = writer.dataset.save_episode
        def checked_save():
            self.check_snapshot(writer, paths)
            return save()
        with patch.object(writer.dataset, 'save_episode', side_effect=checked_save): writer.save()
        writer.close()
        for path, content in paths.items(): self.assertEqual(path.read_bytes(), content)
        self.assertEqual(validate_dataset(self.root)['episodes'], 2)

    def test_failure_restores_metadata_without_copying_or_deleting_old_diagnostics(self):
        writer = self.writer(); paths = self.history(writer)
        info = (self.root/'meta/info.json').read_bytes()
        save = writer.dataset.save_episode
        def interrupted():
            self.check_snapshot(writer, paths)
            save()  # Fail after the native metadata has actually changed.
            raise OSError('simulated disk failure')
        with patch.object(writer.dataset, 'save_episode', side_effect=interrupted):
            with self.assertRaises(OSError): writer.save()
        # Emulate process exit without adapter rollback; startup must handle it.
        writer.dataset.stop_image_writer(); writer.dataset.finalize()
        writer._sidecar.close(); writer._lock.close(); writer._closed = True
        writer = self.writer(resume=True); writer.close()
        self.assertEqual((self.root/'meta/info.json').read_bytes(), info)
        for path, content in paths.items(): self.assertEqual(path.read_bytes(), content)
        self.assertEqual(set((self.root/'meta/xlevr_diagnostics').glob('*.jsonl')), set(paths))
        self.assertEqual(validate_dataset(self.root)['episodes'], 1)


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(MetadataBackupTests(name) for name in MetadataBackupTests.__dict__
                              if name.startswith('test_'))


if __name__ == '__main__': unittest.main()
