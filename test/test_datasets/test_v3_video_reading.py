"""Small real-video regressions for shared v3 files and legacy v2 paths."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from fluxvla.datasets.parquet_dataset_v3 import ParquetDatasetV3
from fluxvla.datasets.utils.video_decode import decode_video_frames_pyav
from fluxvla.transforms.transform_inputs import ProcessParquetInputs


CAMERAS = ['observation.images.base_0_rgb',
           'observation.images.right_wrist_0_rgb']


def write_video(path, shift=0):
    path.parent.mkdir(parents=True, exist_ok=True)
    with av.open(str(path), mode='w') as container:
        stream = container.add_stream('mpeg4', rate=10)
        stream.width = stream.height = 16
        stream.pix_fmt = 'yuv420p'
        for index in range(12):
            pixels = np.full((16, 16, 3), 10 + shift + index * 15, dtype=np.uint8)
            frame = av.VideoFrame.from_ndarray(pixels, format='rgb24')
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)


def sequential_rgb(path, index):
    with av.open(str(path)) as container:
        for position, frame in enumerate(container.decode(video=0)):
            if position == index:
                return frame.to_ndarray(format='rgb24').transpose(2, 0, 1)
    raise AssertionError('Missing reference frame')


class VideoReadingTest(unittest.TestCase):

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        environment = patch.dict(os.environ, {
            'HF_DATASETS_CACHE': str(self.root / 'cache'),
            'HF_HUB_OFFLINE': '1', 'HF_DATASETS_OFFLINE': '1'})
        environment.start()
        self.addCleanup(environment.stop)

    def test_v3_loader_reads_per_camera_offsets_in_shared_files(self):
        info = dict(fps=10, chunks_size=1000, features={},
                    video_path='videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4')
        (self.root / 'meta/episodes/chunk-000').mkdir(parents=True)
        (self.root / 'data/chunk-000').mkdir(parents=True)
        (self.root / 'meta/info.json').write_text(json.dumps(info))
        (self.root / 'meta/stats.json').write_text('{}')
        pq.write_table(pa.Table.from_pylist([dict(task_index=0, task='move')]),
                       self.root / 'meta/tasks.parquet')
        episodes = []
        for episode in range(2):
            meta = dict(episode_index=episode, length=3,
                        dataset_from_index=episode * 3,
                        dataset_to_index=(episode + 1) * 3)
            for camera_index, camera in enumerate(CAMERAS):
                meta[f'videos/{camera}/chunk_index'] = 0
                meta[f'videos/{camera}/file_index'] = 0
                meta[f'videos/{camera}/from_timestamp'] = (episode * 6 + camera_index * 3) / 10
            if episode == 0:
                # Legacy metadata can identify episodes by record order.
                meta.pop('episode_index')
            episodes.append(meta)
        pq.write_table(pa.Table.from_pylist(episodes),
                       self.root / 'meta/episodes/chunk-000/file-000.parquet')
        rows = [dict(index=i, episode_index=i // 3, frame_index=i % 3,
                     timestamp=(i % 3) / 10, task_index=0,
                     **{'observation.state': [float(i)] * 7}) for i in range(6)]
        pq.write_table(pa.Table.from_pylist(rows),
                       self.root / 'data/chunk-000/file-000.parquet')
        for camera_index, camera in enumerate(CAMERAS):
            write_video(self.root / info['video_path'].format(
                video_key=camera, chunk_index=0, file_index=0), camera_index * 20)
        dataset = ParquetDatasetV3(str(self.root), transforms=[dict(
            type='ProcessParquetInputs', parquet_keys=['actions'],
            video_keys=CAMERAS, video_backend='pyav')],
            action_window_size=2, window_start_idx=0)
        for episode in range(2):
            sample = dataset.__getitem__(episode * 3, {'private': {}})
            np.testing.assert_array_equal(sample['actions'],
                                          [[float(episode * 3)] * 7,
                                           [float(episode * 3 + 1)] * 7])
            for camera_index, camera in enumerate(CAMERAS):
                path = self.root / info['video_path'].format(
                    video_key=camera, chunk_index=0, file_index=0)
                np.testing.assert_array_equal(
                    sample['images'][camera_index],
                    sequential_rgb(path, episode * 6 + camera_index * 3))

    def test_legacy_v2_video_path_still_reads_local_timestamp(self):
        path = self.root / 'videos/front/chunk-000/episode_000001.mp4'
        write_video(path)
        transform = ProcessParquetInputs([], ['front'], video_backend='pyav')
        sample = transform(dict(
            info=dict(chunks_size=1000,
                      video_path='videos/{video_key}/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.mp4'),
            episode_index=1, timestamp=0.4, data_root=str(self.root)))
        np.testing.assert_array_equal(sample['images'][0], sequential_rgb(path, 4))

    def test_native_pyav_preserves_query_order_and_rejects_missing_frame(self):
        path = self.root / 'frames.mp4'
        write_video(path)
        decoded = decode_video_frames_pyav(path, [0.9, 0.1], tolerance_s=0.01)
        np.testing.assert_array_equal(decoded[0].numpy(), sequential_rgb(path, 9))
        np.testing.assert_array_equal(decoded[1].numpy(), sequential_rgb(path, 1))
        with self.assertRaises(ValueError):
            decode_video_frames_pyav(path, [2.0], tolerance_s=0.01)


if __name__ == '__main__':
    unittest.main()
