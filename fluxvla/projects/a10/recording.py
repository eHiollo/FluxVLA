"""Streaming, platform-owned LeRobot v3 recording for the current A10 backend."""

from __future__ import annotations

import json
import os
import shutil
import tempfile
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from fluxvla.datasets.parquet_dataset_v3 import (_load_episode_records,
                                                _load_task_mapping)

CAMERAS = ('base_0_rgb', 'right_wrist_0_rgb')
ACTION_NAMES = ['ee.enabled', 'ee.delta_x', 'ee.delta_y', 'ee.delta_z',
                'ee.delta_rx', 'ee.delta_ry', 'ee.delta_rz', 'gripper.pos']
STATE_NAMES = [f'joint_{i}.pos' for i in range(1, 7)] + ['gripper.pos']


def _json_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False,
                                    allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def _stats(values):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, None]
    return {**{key: np.asarray(value).tolist() for key, value in {
        'mean': values.mean(axis=0), 'std': values.std(axis=0),
        'min': values.min(axis=0), 'max': values.max(axis=0),
        'q01': np.quantile(values, .01, axis=0),
        'q99': np.quantile(values, .99, axis=0)}.items()},
        'count': [len(values)]}


class EpisodeWriter:
    """One staged episode at a time. Video is encoded as frames arrive."""

    def __init__(self, root, cameras, fps=30, *, resume=False):
        self.root = Path(root).resolve()
        self.cameras = cameras
        self.fps = int(fps)
        self.active = None
        self.streams = {}
        self.rows = []
        if self.fps <= 0 or set(cameras) != set(CAMERAS):
            raise ValueError('A10 writer requires positive fps and both RGB cameras')
        info_path = self.root / 'meta/info.json'
        if info_path.is_file():
            if not resume:
                raise FileExistsError(f'Dataset exists; use --resume: {self.root}')
            self.info = json.loads(info_path.read_text())
            self.episodes = _load_episode_records(self.root / 'meta')
            self.tasks = _load_task_mapping(self.root / 'meta')
            if self.info['fps'] != self.fps:
                raise ValueError('Cannot resume a dataset with a different fps')
            if self.info['features']['observation.state']['names'] != STATE_NAMES:
                raise ValueError('Cannot resume a dataset with different joint fields')
            if self.info['features']['action']['names'] != ACTION_NAMES:
                raise ValueError('Cannot resume a dataset with different recorded actions')
            for name in CAMERAS:
                shape = self.info['features'][f'observation.images.{name}']['shape']
                if shape != [cameras[name]['height'], cameras[name]['width'], 3]:
                    raise ValueError(f'Cannot resume with a different {name} resolution')
            if (len(self.episodes) != self.info['total_episodes']
                    or sum(int(x['length']) for x in self.episodes)
                    != self.info['total_frames']):
                raise ValueError('Dataset has an incomplete metadata commit; validate it before resuming')
        else:
            if resume:
                raise FileNotFoundError(f'No existing dataset to resume: {self.root}')
            if self.root.exists() and any(self.root.iterdir()):
                raise FileExistsError(f'Record into a new empty directory: {self.root}')
            self.info = self._new_info()
            self.episodes, self.tasks = [], {}
        self.root.mkdir(parents=True, exist_ok=True)

    def _new_info(self):
        features = {
            'observation.state': {'dtype': 'float32', 'shape': [7], 'names': STATE_NAMES},
            'action': {'dtype': 'float32', 'shape': [8], 'names': ACTION_NAMES},
        }
        for name in ('timestamp', 'index', 'episode_index', 'frame_index', 'task_index'):
            features[name] = {'dtype': 'float32' if name == 'timestamp' else 'int64',
                              'shape': [1], 'names': None}
        for name, settings in self.cameras.items():
            h, w = settings['height'], settings['width']
            features[f'observation.images.{name}'] = {
                'dtype': 'video', 'shape': [h, w, 3],
                'names': ['height', 'width', 'channels'],
                'info': {'video.height': h, 'video.width': w,
                         'video.codec': 'h264', 'video.pix_fmt': 'yuv420p',
                         'video.is_depth_map': False, 'video.fps': self.fps,
                         'video.channels': 3, 'has_audio': False}}
        return dict(codebase_version='v3.0', robot_type='a10_follower',
                    total_episodes=0, total_frames=0, total_tasks=0,
                    chunks_size=1000, fps=self.fps, features=features,
                    data_path='data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet',
                    video_path='videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4',
                    splits={'train': '0:0'})

    def start(self, task):
        if self.active is not None:
            raise RuntimeError('Finish or discard the active episode first')
        if not task or not str(task).strip():
            raise ValueError('Recording task must not be empty')
        self.task = str(task)
        self.active = Path(tempfile.mkdtemp(prefix='.episode-', dir=self.root))
        self.rows = []
        try:
            for name, settings in self.cameras.items():
                container = av.open(str(self.active / f'{name}.mp4'), mode='w')
                stream = container.add_stream('libx264', rate=self.fps)
                stream.width, stream.height = settings['width'], settings['height']
                stream.pix_fmt = 'yuv420p'
                stream.options = {'crf': '18', 'preset': 'ultrafast'}
                self.streams[name] = (container, stream)
        except Exception:
            self.discard()
            raise

    def append(self, state, action, images):
        if self.active is None:
            raise RuntimeError('Start an episode before adding a frame')
        state = np.asarray(state, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)
        if (state.shape != (7,) or action.shape != (8,)
                or not np.isfinite(state).all() or not np.isfinite(action).all()):
            raise ValueError('Recording requires finite 7D state and 8D action')
        checked = {}
        for name, settings in self.cameras.items():
            image = np.asarray(images[name])
            if image.dtype != np.uint8 or image.shape != (
                    settings['height'], settings['width'], 3):
                raise ValueError(f'Invalid RGB frame for {name}: {image.shape}/{image.dtype}')
            checked[name] = image
        frame_index = len(self.rows)
        for name, image in checked.items():
            container, stream = self.streams[name]
            frame = av.VideoFrame.from_ndarray(image, format='rgb24')
            frame.pts, frame.time_base = frame_index, Fraction(1, self.fps)
            for packet in stream.encode(frame):
                container.mux(packet)
        row = {'observation.state': state.tolist(), 'action': action.tolist(),
               'timestamp': frame_index / self.fps, 'frame_index': frame_index}
        self.rows.append(row)

    def _finish_video(self):
        for container, stream in self.streams.values():
            try:
                for packet in stream.encode():
                    container.mux(packet)
            finally:
                container.close()
        self.streams = {}

    def discard(self):
        for container, _ in self.streams.values():
            container.close()
        self.streams = {}
        if self.active is not None:
            shutil.rmtree(self.active)
        self.active, self.rows = None, []

    def save(self):
        if self.active is None or len(self.rows) < 2:
            raise ValueError('An episode must contain at least two frames')
        self._finish_video()
        episode_id = max((int(x['episode_index']) for x in self.episodes), default=-1) + 1
        task_index = next((idx for idx, text in self.tasks.items() if text == self.task), None)
        if task_index is None:
            task_index = max(self.tasks, default=-1) + 1
        start, count = self.info['total_frames'], len(self.rows)
        rows = [dict(row, episode_index=episode_id, task_index=task_index,
                     index=start + idx) for idx, row in enumerate(self.rows)]
        file_index = episode_id
        def targets(idx):
            return [self.root / f'data/chunk-000/file-{idx:03d}.parquet',
                    self.root / f'meta/episodes/chunk-000/file-{idx:03d}.parquet',
                    *[self.root / f'videos/observation.images.{cam}/chunk-000/file-{idx:03d}.mp4'
                      for cam in CAMERAS]]
        while any(path.exists() for path in targets(file_index)):
            file_index += 1
        data_path, episode_path, *video_paths = targets(file_index)
        schema = pa.schema([
            ('observation.state', pa.list_(pa.float32(), 7)),
            ('action', pa.list_(pa.float32(), 8)),
            ('timestamp', pa.float32()),
            *[(name, pa.int64()) for name in
              ('frame_index', 'episode_index', 'task_index', 'index')]])
        pq.write_table(pa.Table.from_pylist(rows, schema=schema), self.active / 'data.parquet')
        record = dict(episode_index=episode_id, length=count, tasks=[self.task],
                      dataset_from_index=start, dataset_to_index=start + count,
                      **{'data/chunk_index': 0, 'data/file_index': file_index})
        for cam, destination in zip(CAMERAS, video_paths):
            destination.parent.mkdir(parents=True, exist_ok=True)
            os.replace(self.active / f'{cam}.mp4', destination)
            prefix = f'videos/observation.images.{cam}'
            record.update({f'{prefix}/chunk_index': 0,
                           f'{prefix}/file_index': file_index,
                           f'{prefix}/from_timestamp': 0.0,
                           f'{prefix}/to_timestamp': count / self.fps})
        data_path.parent.mkdir(parents=True, exist_ok=True)
        os.replace(self.active / 'data.parquet', data_path)
        episode_path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pylist([record]), episode_path)
        self.tasks[task_index] = self.task
        pq.write_table(pa.Table.from_pylist([
            {'task_index': idx, 'task': text} for idx, text in sorted(self.tasks.items())]),
            self.root / 'meta/tasks.parquet')
        tables = [pq.read_table(p) for p in sorted((self.root / 'data').rglob('*.parquet'))]
        statistics = {}
        for key in ('observation.state', 'action'):
            values = [value for table in tables for value in table[key].to_pylist()]
            statistics[key] = _stats(values)
        _json_write(self.root / 'meta/stats.json', statistics)
        self.episodes.append(record)
        self.info.update(total_episodes=len(self.episodes), total_frames=start + count,
                         total_tasks=len(self.tasks), splits={'train': f'0:{len(self.episodes)}'})
        _json_write(self.root / 'meta/info.json', self.info)
        shutil.rmtree(self.active)
        self.active, self.rows = None, []
        return episode_id
