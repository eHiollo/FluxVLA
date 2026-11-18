"""Dual-arm storage/export regressions. Synthetic inputs only; no device IO."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from fluxvla.datasets.a10_recording_schema import get_schema, pack_dual_state
from fluxvla.datasets.episode_writer import EpisodeWriter, load_v3
from fluxvla.datasets.a10_recording_io import export_openpi_dataset, validate_dataset

SPEC = get_schema('a10_dual_v1')
CAMERAS = {name: dict(width=16, height=16) for name in SPEC.cameras}


def dual_frame(i, episode=0, frames=40):
    left = np.arange(6) * .05 + .1 + episode * .5 + i * .001
    right = -np.arange(6) * .03 - .2 - episode * .5 - i * .002
    state = pack_dual_state({0: right, 1: left},
                            {'left': 9000 * i / (frames-1), 'right': 9000 * (1-i/(frames-1))})
    images = {name: np.full((16,16,3), 30+80*c+10*episode, dtype=np.uint8)
              for c, name in enumerate(SPEC.cameras)}
    return state, state.copy(), images


def make_dual_fixture(root):
    with EpisodeWriter(root, CAMERAS, schema=SPEC.name, synthetic=True, queue_size=128) as writer:
        for ep, task in enumerate(('Pick up the bottle', 'Put down the bottle')):
            writer.start(task)
            for i in range(40): writer.append(*dual_frame(i, ep), capture_monotonic_s=100+ep*2+i/30)
            writer.save()


class DualRecordingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.base = Path(self.temp.name)
        self.root = self.base/'native'

    def tearDown(self): self.temp.cleanup()

    def writer(self, **kwargs):
        return EpisodeWriter(self.root, CAMERAS, schema=SPEC.name, synthetic=True, queue_size=128, **kwargs)

    def append(self, writer):
        for i in range(4): writer.append(*dual_frame(i, frames=4), capture_monotonic_s=100+i/30)

    def test_physical_order_and_feedback_validation(self):
        state = pack_dual_state({0: [2]*6, 1: [1]*6}, {'left': 0, 'right': 9000})
        np.testing.assert_array_equal(state, [1]*6+[0]+[2]*6+[1])
        self.assertEqual(state.dtype, np.float32)
        for joints, grips in [({0:[2]*6}, {'left':0,'right':1}),
                               ({0:[2]*6,1:[1]*6}, {'left':0}),
                               ({0:[2]*5,1:[1]*6}, {'left':0,'right':1}),
                               ({0:[float('nan')]*6,1:[1]*6}, {'left':0,'right':1}),
                               ({0:[2]*6,1:[1]*6}, {'left':-1,'right':1}),
                               ({0:[2]*6,1:[1]*6}, {'left':0,'right':9001})]:
            with self.subTest(joints=joints, grips=grips), self.assertRaises(ValueError):
                pack_dual_state(joints, grips)

    def test_three_camera_roundtrip_and_source_unchanged(self):
        make_dual_fixture(self.root)
        before = {p.relative_to(self.root):p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        out = self.base/'export'; export_openpi_dataset(self.root,out)
        self.assertEqual(before, {p.relative_to(self.root):p.read_bytes() for p in self.root.rglob('*') if p.is_file()})
        for root in (self.root,out):
            result = validate_dataset(root)
            self.assertEqual((result['schema'],result['episodes'],result['frames']), (SPEC.name,2,80))
        info = json.loads((out/'meta/info.json').read_text())
        self.assertEqual(info['total_videos'],6)
        self.assertEqual(info['features']['observation.state']['names'],list(SPEC.state_names))
        for ep in range(2):
            original = pq.read_table(self.root/f'data/chunk-000/file-{ep:03d}.parquet').to_pylist()
            exported = pq.read_table(out/f'data/chunk-000/episode_{ep:06d}.parquet').to_pylist()
            self.assertEqual(original,exported)
            for c, key in enumerate(SPEC.image_aliases.values()):
                with av.open(str(out/f'videos/chunk-000/{key}/episode_{ep:06d}.mp4')) as video:
                    frames = list(video.decode(video=0))
                self.assertEqual(len(frames),40)
                self.assertEqual(frames[0].pts,0)
                self.assertAlmostEqual(frames[0].to_ndarray(format='rgb24').mean(),30+80*c+10*ep,delta=4)
        contract = json.loads((out/'meta/source_recording_contract.json').read_text())
        self.assertEqual(contract['physical_arm_ee_ids'], {'left':1,'right':0})
        self.assertEqual(contract['action_source'],'same_frame_measured_state_copy')
        with self.assertRaises(FileExistsError): export_openpi_dataset(self.root,out)

    def test_save_discard_resume_and_recover(self):
        with self.writer() as writer:
            writer.start('discard'); self.append(writer); writer.discard()
            writer.start('saved'); self.append(writer); writer.save()
            writer.start('unfinished'); self.append(writer)
        with self.assertRaises(RuntimeError): self.writer(resume=True)
        with self.writer(resume=True,recover=True) as writer:
            self.assertEqual(writer.info['total_frames'],4)
            writer.start('second'); self.append(writer); writer.save()
        self.assertEqual(validate_dataset(self.root)['frames'],8)
        with self.writer(resume=True) as writer:
            self.assertEqual(writer.info['total_episodes'],2)

    def test_commit_recovery_and_incompatible_resume(self):
        with self.writer() as writer:
            writer.start('test'); self.append(writer)
            with patch.object(EpisodeWriter,'_publish',side_effect=OSError('power loss')):
                with self.assertRaises(OSError): writer.save()
        with self.writer(resume=True,recover=True) as writer:
            self.assertEqual(writer.info['total_frames'],4)
        with self.assertRaises(ValueError):
            EpisodeWriter(self.root,{n:CAMERAS[n] for n in get_schema().cameras},resume=True,synthetic=True)
        with self.assertRaises(ValueError): self.writer(resume=True,device_metadata={'changed':True})
        with self.assertRaises(ValueError):
            EpisodeWriter(self.root,CAMERAS,schema=SPEC.name,resume=True,synthetic=False)

    def test_invalid_frames_poison_episode(self):
        with self.writer() as writer:
            for invalid in ('missing_camera','state7','nan','low_gripper','high_gripper','action_target'):
                writer.start(invalid); self.append(writer)
                state, action, images = dual_frame(0)
                if invalid=='missing_camera': images.pop(SPEC.cameras[1])
                if invalid=='state7': state=state[:7]
                if invalid=='nan': state[0]=np.nan
                if invalid=='low_gripper': state[6]=action[6]=-.1
                if invalid=='high_gripper': state[13]=action[13]=1.1
                if invalid=='action_target': action[0]+=.1
                with self.subTest(invalid=invalid), self.assertRaises(ValueError): writer.append(state,action,images)
                with self.assertRaises(RuntimeError): writer.save()
                writer.discard()
            self.assertEqual(writer.info['total_episodes'],0)

    def test_validator_rejects_tampering(self):
        make_dual_fixture(self.root)
        path=self.root/'data/chunk-000/file-000.parquet'
        table=pq.read_table(path)
        for field, index, value in [('action',0,99),('observation.state',6,2)]:
            rows=table.to_pylist(); rows[0][field][index]=value
            pq.write_table(pa.Table.from_pylist(rows,schema=table.schema),path)
            with self.assertRaises(ValueError): validate_dataset(self.root,check_videos=False)
        pq.write_table(table,path)
        path=self.root/'meta/recording_contract.json'; contract=json.loads(path.read_text())
        wrong=copy.deepcopy(contract); wrong['physical_arm_ee_ids']={'left':0,'right':1}
        path.write_text(json.dumps(wrong))
        with self.assertRaisesRegex(ValueError,'contract mismatch'): validate_dataset(self.root)
        path.write_text(json.dumps(contract))
        info_path=self.root/'meta/info.json'; info=json.loads(info_path.read_text())
        info.pop('a10_recording_schema'); info_path.write_text(json.dumps(info))
        with self.assertRaises(ValueError): validate_dataset(self.root)

    def test_native_image_statistics_match_actual_rgb_pixels(self):
        pixels=[]
        with self.writer() as writer:
            writer.start('moments')
            for i in range(4):
                state,action,images=dual_frame(i,frames=4)
                image=(np.arange(16*16*3).reshape(16,16,3)+i*17).astype(np.uint8)
                pixels.append(image.reshape(-1,3)/255.)
                writer.append(state,action,{n:image for n in SPEC.cameras})
            writer.save()
        actual=json.loads((self.root/'meta/stats.json').read_text())
        values=np.concatenate(pixels)
        expected=dict(mean=values.mean(0),std=values.std(0),min=values.min(0),max=values.max(0))
        for name in SPEC.cameras:
            for key, value in expected.items():
                np.testing.assert_allclose(np.asarray(actual[f'observation.images.{name}'][key]).reshape(3),value,atol=1e-10)

    def test_config_reuses_device_identities_without_usb_paths(self):
        from mmengine import Config
        root=Path(__file__).resolve().parents[2]
        cfg=Config.fromfile(str(root/'configs/a10/a10_dual_arm_record.py'))
        devices=Config.fromfile(str(root/'configs/a10/a10_dual_devices.py'))
        record=cfg.record.to_dict(); record.update(root=str(self.root),synthetic=True)
        for name in SPEC.cameras:
            self.assertEqual(record['cameras'][name]['identity'],devices.cameras[name].identity)
            self.assertNotIn('device',record['cameras'][name])
        self.assertTrue(record['cameras']['right_wrist_0_rgb']['rotate_180'])
        backend = record.pop('backend', 'legacy')
        if backend == 'lerobot':
            from fluxvla.datasets.lerobot_episode_writer import LeRobotEpisodeWriter
            writer_type = LeRobotEpisodeWriter
        else:
            writer_type = EpisodeWriter
        writer = writer_type(**record)
        writer.close()
        self.assertEqual(writer.info['total_frames'], 0)


if __name__ == '__main__': unittest.main()
