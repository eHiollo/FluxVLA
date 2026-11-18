"""Offline recording tests; no cameras, VR, controller or motion commands."""
import json
from fractions import Fraction
import av
from pathlib import Path
import queue
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from fluxvla.datasets.episode_writer import EpisodeWriter, CAMERAS, load_v3
from fluxvla.datasets.a10_recording_io import export_openpi_dataset, validate_dataset

CAMERA_CONFIG = {name: dict(width=16, height=16) for name in CAMERAS}


def frame(i):
    return ([i/100] * 6 + [40+i/100], [1,.001,0,0,0,0,0,.5],
            {name: np.full((16,16,3), 30+i+index*80, dtype=np.uint8) for index,name in enumerate(CAMERAS)})


def make_fixture(root, frames=40):
    with EpisodeWriter(root,CAMERA_CONFIG,queue_size=128,synthetic=True) as writer:
        for task in ('Pick up the bottle','Put down the bottle'):
            writer.start(task)
            for i in range(frames): writer.append(*frame(i),capture_monotonic_s=100+i/30)
            writer.save()


class RecordingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.root = self.base/'native'

    def tearDown(self): self.temp.cleanup()

    def writer(self, **kwargs):
        return EpisodeWriter(self.root,CAMERA_CONFIG,synthetic=True,queue_size=128,**kwargs)

    def append(self, writer, n=4):
        for i in range(n): writer.append(*frame(i),capture_monotonic_s=100+i/30)

    def test_save_resume_discard_and_export(self):
        with self.writer() as writer:
            writer.start('discard'); self.append(writer); writer.discard()
            writer.start('bottle'); self.append(writer); self.assertEqual(writer.save(),0)
            with self.assertRaises(RuntimeError): validate_dataset(self.root)
        with self.writer(resume=True) as writer:
            writer.start('second'); self.append(writer); self.assertEqual(writer.save(),1)
        report=validate_dataset(self.root)
        self.assertEqual((report['episodes'],report['frames']),(2,8))
        before={p.relative_to(self.root):p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        out=self.base/'openpi'; export_openpi_dataset(self.root,out)
        self.assertEqual(validate_dataset(out)['version'],'v2.1')
        self.assertTrue((out/'SIMULATED_DATA.json').exists())
        with self.assertRaises(ValueError):
            EpisodeWriter(out,CAMERA_CONFIG,resume=True,synthetic=True)
        self.assertEqual(before,{p.relative_to(self.root):p.read_bytes() for p in self.root.rglob('*') if p.is_file()})
        original=pq.read_table(self.root/'data/chunk-000/file-000.parquet').to_pylist()
        exported=pq.read_table(out/'data/chunk-000/episode_000000.parquet').to_pylist()
        self.assertEqual(original,exported)
        with self.assertRaises(FileExistsError): export_openpi_dataset(self.root,out)
        with self.assertRaises(ValueError): export_openpi_dataset(self.root,self.root/'bad')

    def test_invalid_frame_poison_and_no_saved_episode(self):
        with self.writer() as writer:
            writer.start('test'); self.append(writer)
            state,action,images=frame(0); state[0]=float('nan')
            with self.assertRaises(ValueError): writer.append(state,action,images)
            with self.assertRaises(RuntimeError): writer.save()
            writer.discard(); self.assertEqual(writer.info['total_frames'],0)
            for invalid in ('missing_camera','state14','action7','duplicate_time'):
                writer.start(invalid); self.append(writer)
                state,action,images=frame(0)
                if invalid=='missing_camera': images.pop(CAMERAS[0])
                if invalid=='state14': state*=2
                if invalid=='action7': action=action[:7]
                with self.assertRaises(ValueError): writer.append(state,action,images,capture_monotonic_s=100)
                writer.discard()

    def test_queue_overflow_never_publishes_partial_episode(self):
        started=threading.Event(); release=threading.Event()
        original=EpisodeWriter._encode_frame
        def slow(writer,*args):
            started.set(); release.wait(5); return original(writer,*args)
        with EpisodeWriter(self.root,CAMERA_CONFIG,queue_size=1,synthetic=True) as writer:
            with patch.object(EpisodeWriter,'_encode_frame',slow):
                writer.start('overflow'); writer.append(*frame(0))
                self.assertTrue(started.wait(5)); writer.append(*frame(1))
                try:
                    with self.assertRaises(queue.Full): writer.append(*frame(2))
                finally: release.set()
                with self.assertRaises(RuntimeError): writer.save()
                writer.discard()
            self.assertEqual(writer.info['total_frames'],0)

    def test_encoder_failure_does_not_deadlock(self):
        with self.writer() as writer:
            with patch.object(EpisodeWriter,'_encode_frame',side_effect=OSError('encoder failed')):
                writer.start('bad'); self.append(writer)
                with self.assertRaises(RuntimeError): writer.save()
            writer.discard()

    def test_unfinished_recording_is_preserved_and_recovered(self):
        with self.writer() as writer:
            writer.start('incomplete'); self.append(writer)
        self.assertTrue((self.root/'.recording/active').exists())
        with self.assertRaises(RuntimeError): self.writer(resume=True)
        with self.writer(resume=True,recover=True) as writer:
            self.assertEqual(writer.info['total_frames'],0)
            self.assertEqual(len(list((self.root/'.recording').glob('recovered-*'))),1)
            writer.start('new'); self.append(writer); writer.save()
        self.assertEqual(validate_dataset(self.root)['episodes'],1)

    def test_interrupted_commit_rolls_forward_once(self):
        import fluxvla.datasets.episode_writer as module
        real_replace=module.os.replace
        with self.writer() as writer:
            writer.start('before'); self.append(writer); writer.save()
            writer.start('after'); self.append(writer)
            def interrupted(src,dst):
                if Path(src)==writer.active/'publish/meta/info.json': raise OSError('power loss')
                return real_replace(src,dst)
            with patch.object(module.os,'replace',interrupted):
                with self.assertRaises(OSError): writer.save()
        with self.writer(resume=True,recover=True) as writer:
            self.assertEqual(writer.info['total_episodes'],2)
        self.assertEqual(validate_dataset(self.root)['frames'],8)

    def test_contract_and_lock_prevent_incompatible_resume(self):
        with self.writer() as writer:
            with self.assertRaises(RuntimeError): self.writer(resume=True)
        with self.assertRaises(ValueError): EpisodeWriter(self.root,CAMERA_CONFIG,resume=True,synthetic=False)
        changed={n:dict(width=32,height=16) for n in CAMERAS}
        with self.assertRaises(ValueError): EpisodeWriter(self.root,changed,resume=True,synthetic=True)

    def test_validator_rejects_bad_timestamps_and_video(self):
        make_fixture(self.root,4)
        path=self.root/'data/chunk-000/file-000.parquet'
        table=pq.read_table(path); rows=table.to_pylist(); rows[2]['timestamp']=.5
        pq.write_table(pa.Table.from_pylist(rows,schema=table.schema),path)
        with self.assertRaisesRegex(ValueError,'timestamps'): validate_dataset(self.root)
        pq.write_table(table,path)
        video=next((self.root/'videos').rglob('*.mp4')); video.write_bytes(b'broken')
        with self.assertRaises(Exception): validate_dataset(self.root)

    def test_export_splits_shared_video_and_parquet_by_episode(self):
        make_fixture(self.root,4)
        info,episodes,_=load_v3(self.root)
        first=self.root/'data/chunk-000/file-000.parquet'
        second=self.root/'data/chunk-000/file-001.parquet'
        pq.write_table(pa.concat_tables([pq.read_table(first),pq.read_table(second)]),first)
        second.unlink()
        episodes[1]['data/file_index']=0
        for name in CAMERAS:
            key=f'observation.images.{name}'
            directory=self.root/f'videos/{key}/chunk-000'
            images=[]
            for file in ('file-000.mp4','file-001.mp4'):
                with av.open(str(directory/file)) as container:
                    images.extend(frame.to_ndarray(format='rgb24') for frame in container.decode(video=0))
            # Deliberately make the second episode different, to detect copying the first segment.
            for image in images[4:]: image[:]=220
            with av.open(str(directory/'file-000.mp4'),'w') as container:
                stream=container.add_stream('libx264',rate=30)
                stream.width=stream.height=16; stream.pix_fmt='yuv420p'
                for index,image in enumerate(images):
                    frame=av.VideoFrame.from_ndarray(image,format='rgb24')
                    frame.pts=index; frame.time_base=Fraction(1,30)
                    for packet in stream.encode(frame): container.mux(packet)
                for packet in stream.encode(): container.mux(packet)
            (directory/'file-001.mp4').unlink()
            episodes[1][f'videos/{key}/file_index']=0
            episodes[1][f'videos/{key}/from_timestamp']=4/30
            episodes[1][f'videos/{key}/to_timestamp']=8/30
        for ep in episodes:
            pq.write_table(pa.Table.from_pylist([ep]),self.root/f"meta/episodes/chunk-000/file-{ep['episode_index']:03d}.parquet")
        validate_dataset(self.root)
        output=self.base/'exported'; export_openpi_dataset(self.root,output)
        validate_dataset(output)
        with av.open(str(output/'videos/chunk-000/observation.images.top/episode_000001.mp4')) as video:
            frames=list(video.decode(video=0))
            self.assertEqual(len(frames),4)
            self.assertEqual(frames[0].pts,0)
            self.assertGreater(frames[0].to_ndarray(format='rgb24').mean(),210)

    def test_recovery_rejects_tampered_staged_commit(self):
        with self.writer() as writer:
            writer.start('test'); self.append(writer)
            with patch.object(EpisodeWriter,'_publish',side_effect=OSError('interrupted')):
                with self.assertRaises(OSError): writer.save()
        staged=self.root/'.recording/active/publish/data/chunk-000/file-000.parquet'
        staged.write_bytes(b'tampered')
        with self.assertRaisesRegex(RuntimeError,'checksum'): self.writer(resume=True,recover=True)
        self.assertTrue(staged.exists())
        self.assertFalse((self.root/'data/chunk-000/file-000.parquet').exists())

    def test_av1_input_uses_codec_identity_not_decoder_name(self):
        make_fixture(self.root,4)
        info_path=self.root/'meta/info.json'
        info=json.loads(info_path.read_text())
        for path in (self.root/'videos').rglob('*.mp4'):
            with av.open(str(path)) as container:
                images=[f.to_ndarray(format='rgb24') for f in container.decode(video=0)]
            with av.open(str(path),'w') as container:
                stream=container.add_stream('libaom-av1',rate=30)
                stream.width=stream.height=16; stream.pix_fmt='yuv420p'
                stream.options={'cpu-used':'8','crf':'30'}
                for index,image in enumerate(images):
                    frame=av.VideoFrame.from_ndarray(image,format='rgb24')
                    frame.pts=index; frame.time_base=Fraction(1,30)
                    for packet in stream.encode(frame): container.mux(packet)
                for packet in stream.encode(): container.mux(packet)
        for name in CAMERAS: info['features'][f'observation.images.{name}']['info']['video.codec']='av1'
        info_path.write_text(json.dumps(info))
        validate_dataset(self.root)
        report = export_openpi_dataset(self.root,self.base/'from_av1')
        self.assertEqual(report['copied'], 4)
        output_info = json.loads((self.base/'from_av1/meta/info.json').read_text())
        self.assertEqual(output_info['features']['observation.images.top']['info']['video.codec'], 'av1')
        self.assertEqual(validate_dataset(self.base/'from_av1')['frames'],8)


if __name__=='__main__': unittest.main()
