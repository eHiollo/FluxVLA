"""Small A10 adapter around the same LeRobotDataset used by the old recorder.

LeRobot owns image writing, episode buffers, video encoding and v3 metadata.
This adapter only adds the A10 field contract, an exclusive local lock and
diagnostics. It never imports the neighbouring LeRobot checkout or robot code.
"""
from __future__ import annotations

import copy
import fcntl
import json
import logging
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np

from .a10_recording_schema import get_schema
from .episode_writer import json_write


LOG = logging.getLogger(__name__)


class LeRobotEpisodeWriter:
    def __init__(self, root, cameras, fps=30, *, resume=False,
                 schema='a10_dual_v1', device_metadata=None, synthetic=False,
                 max_episode_frames=18000, image_writer_threads=6,
                 repo_id='local/a10_dual_record'):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.dataset_type = LeRobotDataset
        self.schema = get_schema(schema)
        self.root = Path(root).resolve()
        self.cameras = copy.deepcopy(cameras)
        self.fps, self.repo_id = fps, repo_id
        if (not self.schema.dual or fps != 30 or set(cameras) != set(self.schema.cameras)
                or type(image_writer_threads) is not int or image_writer_threads < 1
                or type(max_episode_frames) is not int or max_episode_frames < 2
                or type(synthetic) is not bool):
            raise ValueError('Invalid native LeRobot dual-arm recording configuration')
        for cfg in cameras.values():
            if any(type(cfg.get(k)) is not int or cfg[k] < 64 or cfg[k] % 2 for k in ('width', 'height')):
                raise ValueError('Native AV1 camera dimensions must be even integers >=64')
        if not resume and self.root.exists() and any(self.root.iterdir()):
            raise FileExistsError('Use a new dataset directory or explicitly set record.resume=True')
        self.max_episode_frames = max_episode_frames
        self.image_writer_threads = image_writer_threads
        self.contract = dict(self.schema.contract(), cameras=self.cameras, fps=fps,
                             synthetic=synthetic, device_metadata=copy.deepcopy(device_metadata or {}),
                             writer_backend='lerobot', lerobot_version='0.4.3')
        if resume:
            path = self.root / 'meta/recording_contract.json'
            if not path.is_file() or json.loads(path.read_text()) != self.contract:
                raise ValueError('Native LeRobot resume requires the same backend/contract; '
                                 'use a NEW root, e.g. datasets/a10_dual_lerobot_record')
            info_path = self.root / 'meta/info.json'
            version = json.loads(info_path.read_text()).get('codebase_version')
            if version != 'v3.0':
                raise ValueError(
                    f'原生 LeRobot 续录要求 v3.0，当前目录标记为 {version!r}：{self.root}。'
                    '请使用新的 record.root 并设置 record.resume=False；'
                    '已有 v2.1 导出或混合格式目录不能直接追加，旧数据保持不变。')
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = (self.root / '.recording.lock').open('a+')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self._lock.close()
            raise RuntimeError('Dataset is already in use')
        self.dataset = None
        self.active = None
        self._sidecar = None
        self._closed = False
        self._error = None
        self._count = 0
        self._stage = self.root / '.recording/active'
        try:
            self._discard_pending()
            info = self.root / 'meta/info.json'
            self.info = json.loads(info.read_text()) if info.is_file() else {'total_episodes': 0, 'total_frames': 0}
            if not resume:
                self._new_dataset()
                json_write(self.root / 'meta/recording_contract.json', self.contract)
                if synthetic:
                    json_write(self.root / 'SIMULATED_DATA.json', {'simulation': True, 'purpose': 'pipeline test'})
        except BaseException:
            self.close()
            raise

    def _discard_pending(self):
        """Under the dataset lock, roll back only an uncommitted episode."""
        if not self._stage.exists():
            return
        manifest = self._stage / 'baseline.json'
        if manifest.exists():
            baseline = json.loads(manifest.read_text())
            keep = set(baseline['files'])
            # Saved episodes use closed files; this save only creates new ones.
            for area in ('data', 'videos', 'meta'):
                for path in (self.root / area).rglob('*'):
                    if path.is_file() and str(path.relative_to(self.root)) not in keep:
                        path.unlink()
            shutil.copytree(self._stage / 'baseline_meta', self.root / 'meta', dirs_exist_ok=True)
        elif json.loads((self._stage / 'episode.json').read_text()).get('phase') == 'saving':
            # Compatibility with old interrupted saves: only remove files that
            # no committed episode references. Never delete a shared saved file.
            from .episode_writer import load_v3, safe_path
            from .a10_recording_io import data_path
            info, episodes, _ = load_v3(self.root)
            if [e['episode_index'] for e in episodes] != list(range(info['total_episodes'])):
                raise RuntimeError('Old dataset metadata is damaged; cannot identify saved episodes')
            keep = set()
            for ep in episodes:
                keep.add(data_path(self.root, info, ep).resolve())
                for key, feature in info['features'].items():
                    if feature['dtype'] == 'video':
                        keep.add(safe_path(self.root, info['video_path'].format(video_key=key,
                            chunk_index=ep[f'videos/{key}/chunk_index'],
                            file_index=ep[f'videos/{key}/file_index'])).resolve())
            for area in ('data', 'videos'):
                for path in (self.root / area).rglob('*'):
                    if path.is_file() and path.resolve() not in keep:
                        path.unlink()
        shutil.rmtree(self.root / 'images', ignore_errors=True)
        for path in self.root.glob('tmp*'):
            if path.is_dir() and all(p.suffix == '.mp4' for p in path.rglob('*') if p.is_file()):
                shutil.rmtree(path)
        shutil.rmtree(self._stage)
        LOG.warning('已丢弃未保存或保存未完成的当前集，保留已保存数据')

    def _new_dataset(self):
        started = time.perf_counter()
        # create() requires a nonexistent root. Initialise beside the lock,
        # then relocate the empty native dataset before any frame paths exist.
        with tempfile.TemporaryDirectory(prefix='.lerobot-init-', dir=self.root) as tmp:
            ds = self.dataset_type.create(self.repo_id, self.fps, root=Path(tmp) / 'data',
                robot_type='a10_follower', features=self._features(), use_videos=True,
                image_writer_threads=self.image_writer_threads, video_backend='pyav')
            shutil.copytree(ds.root, self.root, dirs_exist_ok=True)
            ds.root = ds.meta.root = self.root
        ds.meta.info['a10_recording_schema'] = self.schema.name
        json_write(self.root / 'meta/info.json', ds.meta.info)
        self.dataset, self.info = ds, ds.meta.info
        LOG.info('LeRobot 数据集创建完成：%s，耗时 %.3fs', self.root, time.perf_counter()-started)

    def _features(self):
        result = {key: dict(dtype='float32', shape=(14,), names=list(names))
                  for key, names in [('observation.state', self.schema.state_names), ('action', self.schema.action_names)]}
        for name, cfg in self.cameras.items():
            result[f'observation.images.{name}'] = dict(dtype='video',
                shape=(cfg['height'], cfg['width'], 3), names=['height', 'width', 'channels'])
        return result

    def _open_dataset(self):
        if self.dataset is not None:
            return
        if self.info['total_episodes'] == 0:
            self._new_dataset()
        else:
            # Only local, already committed datasets reach the native loader.
            from .a10_recording_io import validate_dataset
            started = time.perf_counter()
            validate_dataset(self.root, check_videos=False, _locked=True)
            self.dataset = self.dataset_type(self.repo_id, root=self.root, video_backend='pyav')
            self.dataset.start_image_writer(num_threads=self.image_writer_threads)
            LOG.info('LeRobot 续录打开完成：%s，耗时 %.3fs', self.root, time.perf_counter()-started)

    def start(self, task):
        if self._closed or self.active is not None:
            raise RuntimeError('Writer closed or an episode is already active')
        if not isinstance(task, str) or not task.strip():
            raise ValueError('Task must be nonempty text')
        self._open_dataset()
        if self.dataset.episode_buffer is None:
            self.dataset.episode_buffer = self.dataset.create_episode_buffer()
        else:
            self.dataset.clear_episode_buffer()
        self.task = task.strip()
        self._stage.mkdir(parents=True)
        self.active = self._stage
        json_write(self.active / 'episode.json', dict(task=self.task, phase='recording',
                                                     episode_index=self.info['total_episodes']))
        self._sidecar = (self.active / 'diagnostics.jsonl').open('w')
        self._error, self._count = None, 0

    def append(self, state, action, images, *, capture_monotonic_s=None, diagnostics=None):
        if self.active is None or self._closed or self._error:
            raise RuntimeError('Start a healthy episode before adding frames')
        try:
            state, action = np.array(state, dtype=np.float32, copy=True), np.array(action, dtype=np.float32, copy=True)
            self.schema.validate_values(state, action)
            if set(images) != set(self.cameras) or self._count >= self.max_episode_frames:
                raise ValueError('Missing cameras or episode frame limit exceeded')
            frame = {'observation.state': state, 'action': action, 'task': self.task}
            for name, cfg in self.cameras.items():
                image = np.asarray(images[name])
                if image.dtype != np.uint8 or image.shape != (cfg['height'], cfg['width'], 3):
                    raise ValueError(f'Invalid RGB image: {name}')
                frame[f'observation.images.{name}'] = image.copy()
            detail = json.dumps(dict(frame_index=self._count, capture_monotonic_s=capture_monotonic_s,
                                     details=diagnostics or {}), allow_nan=False)
            self.dataset.add_frame(frame)
            self._sidecar.write(detail + '\n')
            self._count += 1
        except Exception as exc:
            self._error = exc
            raise

    def save(self):
        if self.active is None or self._error or self._count < 2:
            raise RuntimeError('Need at least two valid frames; discard a failed episode')
        eid = self.info['total_episodes']
        started = time.perf_counter()
        # Snapshot small metadata, not videos. Each completed episode closes its
        # parquet files, so a later failure cannot corrupt earlier saved episodes.
        # Historical sidecars are immutable; the manifest preserves their files.
        shutil.copytree(self.root / 'meta', self.active / 'baseline_meta',
                        ignore=shutil.ignore_patterns('xlevr_diagnostics'))
        files = [str(p.relative_to(self.root)) for area in ('meta', 'data', 'videos')
                 for p in (self.root / area).rglob('*') if p.is_file()]
        json_write(self.active / 'baseline.json', dict(files=files))
        json_write(self.active / 'episode.json', dict(task=self.task, phase='saving', episode_index=eid))
        backup_elapsed = time.perf_counter() - started
        LOG.info('LeRobot 元数据备份完成，耗时 %.3fs；跳过历史诊断日志', backup_elapsed)
        # Keep native parallel video encoding and the open cross-episode writers.
        # The recording entrypoint selects spawn before starting device threads.
        LOG.info('LeRobot 第 %d 集开始保存：%d 帧，%d 路视频，原生并行编码',
                 eid+1, self._count, len(self.cameras))
        try:
            native_started = time.perf_counter()
            self.dataset.save_episode()
            native_elapsed = time.perf_counter() - native_started
            finalize_started = time.perf_counter()
            self.dataset.finalize()
            self.dataset.meta.load_metadata()
            self.dataset.latest_episode = self.dataset.meta.latest_episode = None
            finalize_elapsed = time.perf_counter() - finalize_started
        except BaseException:
            LOG.exception('LeRobot 第 %d 集保存失败，耗时 %.3fs；本集将在清理时丢弃',
                          eid+1, time.perf_counter()-started)
            raise
        self.info = dict(self.dataset.meta.info)
        self._sidecar.close()
        self._sidecar = None
        dest = self.root / 'meta/xlevr_diagnostics' / f'episode_{eid:06d}.jsonl'
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(self.active / 'diagnostics.jsonl', dest)
        shutil.rmtree(self.active)
        self.active = None
        LOG.info('LeRobot 第 %d 集保存完成，总耗时 %.3fs（备份 %.3fs，原生保存 %.3fs，'
                 '文件收尾 %.3fs）；复用数据集继续下一集',
                 eid+1, time.perf_counter()-started, backup_elapsed, native_elapsed, finalize_elapsed)
        return eid

    def discard(self):
        if self.active is None:
            return
        if json.loads((self.active / 'episode.json').read_text())['phase'] == 'saving':
            self.dataset.stop_image_writer()
            self.dataset.finalize()
            self.dataset = None
            if self._sidecar is not None: self._sidecar.close()
            self._sidecar = None
            self._discard_pending()
            self.info = json.loads((self.root / 'meta/info.json').read_text())
            self.active, self._error = None, None
            return
        self.dataset.clear_episode_buffer()
        self._sidecar.close()
        self._sidecar = None
        shutil.rmtree(self.active)
        self.active, self._error = None, None

    def close(self):
        if self._closed:
            return
        self._closed = True
        started = time.perf_counter()
        try:
            try:
                if self.dataset is not None:
                    try:
                        self.dataset.stop_image_writer()
                    finally:
                        self.dataset.finalize()
                    LOG.info('LeRobot 退出收尾完成：%s，耗时 %.3fs',
                             self.root, time.perf_counter()-started)
            finally:
                if self._sidecar is not None:
                    self._sidecar.close()
                self._discard_pending()
                self.active = None
        finally:
            self._lock.close()
