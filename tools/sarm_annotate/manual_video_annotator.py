#!/usr/bin/env python
"""Local browser annotation of ordered SARM stages in LeRobot v2.1 videos."""

import argparse
import errno
import json
import math
import mimetypes
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import RLock
from urllib.parse import parse_qs, urlparse


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n',
                         encoding='utf-8')
    os.replace(temporary, path)


class AnnotationStore:
    def __init__(self, root, output):
        self.root = Path(root).resolve()
        self.output = Path(output).resolve()
        self.session_path = self.output.with_suffix('.session.json')
        self.lock = RLock()
        self.info = json.loads((self.root / 'meta/info.json').read_text())
        if self.info['codebase_version'] != 'v2.1':
            raise ValueError('目前支持 LeRobot v2.1 数据集')
        self.fps = float(self.info['fps'])
        if self.fps <= 0:
            raise ValueError('数据集 fps 必须大于零')
        self.cameras = [key for key, feature in self.info['features'].items()
                        if feature['dtype'] == 'video']
        if not self.cameras:
            raise ValueError('数据集没有视频相机')
        self.episodes = sorted(
            [json.loads(line) for line in
             (self.root / 'meta/episodes.jsonl').read_text().splitlines()
             if line.strip()], key=lambda entry: entry['episode_index'])
        self.by_id = {entry['episode_index']: entry for entry in self.episodes}
        if not self.episodes or len(self.by_id) != len(self.episodes):
            raise ValueError('episode 列表为空或编号重复')
        for episode in self.episodes:
            if episode['length'] <= 0:
                raise ValueError('episode 帧数必须大于零')
            for camera in self.cameras:
                if not self.video_path(episode['episode_index'], camera).is_file():
                    raise ValueError(f"视频缺失: episode {episode['episode_index']}, {camera}")
        entries = json.loads(self.output.read_text()) if self.output.exists() else []
        self.annotations = {entry['episode_index']: entry for entry in entries}
        if len(self.annotations) != len(entries):
            raise ValueError('已有输出存在重复 episode')
        self.session = (json.loads(self.session_path.read_text())
                        if self.session_path.exists() else {})
        if self.session.get('dataset_root', str(self.root)) != str(self.root):
            raise ValueError('会话属于其他数据集，请指定不同的 --output')
        self.names = self.session.get('names', [])
        if not self.names and entries:
            self.names = entries[0]['dense']['names']
        for entry in entries:
            if entry['dense']['names'] != self.names:
                raise ValueError('已有标签阶段列表不一致')
            cuts = [end for end in entry['dense']['end_frames'][:-1]]
            if self.make_entry(entry['episode_index'], cuts) != entry:
                raise ValueError('已有标签格式或边界不一致')
        self.drafts = self.session.get('drafts', {})
        self.skipped = set(self.session.get('skipped', []))
        if not self.skipped.issubset(self.by_id):
            raise ValueError('跳过记录包含未知 episode')
        if self.skipped.intersection(self.annotations):
            raise ValueError('同一集不能同时已标注和已跳过')
        for key, draft in self.drafts.items():
            self.validate_cuts(int(key), draft['cuts'], complete=False)
        self.current = self.session.get('current', self.episodes[0]['episode_index'])
        if self.current not in self.by_id:
            self.current = self.episodes[0]['episode_index']

    def video_path(self, episode_id, camera):
        if episode_id not in self.by_id or camera not in self.cameras:
            raise ValueError('未知 episode 或相机')
        path = (self.root / self.info['video_path'].format(
            episode_index=episode_id,
            episode_chunk=episode_id // self.info.get('chunks_size', 1000),
            video_key=camera)).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError('视频路径超出数据集')
        return path

    def validate_cuts(self, episode_id, cuts, complete=False):
        if episode_id not in self.by_id or not self.names:
            raise ValueError('先设置阶段描述并选择有效 episode')
        if not isinstance(cuts, list) or any(type(v) is not int for v in cuts):
            raise ValueError('切分点必须是整数帧列表')
        length = self.by_id[episode_id]['length']
        if (len(cuts) > len(self.names) - 1 or
                any(v < 0 or v >= length - 1 for v in cuts) or
                any(a >= b for a, b in zip(cuts, cuts[1:]))):
            raise ValueError('切分点必须递增、在末帧之前，且不能超过阶段数量')
        if complete and len(cuts) != len(self.names) - 1:
            raise ValueError(f'还需要 {len(self.names) - 1 - len(cuts)} 次切分')

    def make_entry(self, episode_id, cuts):
        self.validate_cuts(episode_id, cuts, complete=True)
        return dict(episode_index=episode_id, dense=dict(
            names=self.names.copy(), start_frames=[0] + [v + 1 for v in cuts],
            end_frames=cuts + [self.by_id[episode_id]['length'] - 1]))

    def save_session(self):
        atomic_json(self.session_path, dict(dataset_root=str(self.root),
                    names=self.names, current=self.current, drafts=self.drafts,
                    skipped=sorted(self.skipped)))

    def save_output(self):
        atomic_json(self.output, [self.annotations[key]
                                 for key in sorted(self.annotations)])

    def state(self):
        return dict(episodes=self.episodes, cameras=self.cameras, fps=self.fps,
                    names=self.names, current=self.current, drafts=self.drafts,
                    annotations=list(self.annotations.values()),
                    skipped=sorted(self.skipped),
                    output=str(self.output))

    def update(self, action, data):
        with self.lock:
            if action == 'setup':
                names = [name.strip() for name in data['names'] if name.strip()]
                if not names or len(set(names)) != len(names):
                    raise ValueError('请输入非空且不重复的阶段描述')
                if any(len(names) > episode['length'] for episode in self.episodes):
                    raise ValueError('阶段数量超过某集帧数')
                if self.names and names != self.names:
                    if not data.get('reset'):
                        raise ValueError('修改阶段列表需要确认重置')
                    self.annotations = {}
                    self.drafts = {}
                    self.skipped = set()
                    self.save_output()
                self.names = names
            else:
                episode_id = int(data['episode_index'])
                if episode_id not in self.by_id:
                    raise ValueError('未知 episode')
                if action == 'draft':
                    self.validate_cuts(episode_id, data['cuts'])
                    if episode_id not in self.skipped:
                        self.drafts[str(episode_id)] = dict(
                            cuts=data['cuts'], frame=max(0, min(
                                int(data.get('frame', 0)), self.by_id[episode_id]['length'] - 1)))
                elif action == 'save':
                    if episode_id in self.skipped:
                        raise ValueError('本集已跳过，请先点击“重标这一集”')
                    self.annotations[episode_id] = self.make_entry(episode_id, data['cuts'])
                    self.drafts.pop(str(episode_id), None)
                    self.save_output()
                elif action == 'reset':
                    self.skipped.discard(episode_id)
                    self.annotations.pop(episode_id, None)
                    self.drafts[str(episode_id)] = dict(cuts=[], frame=0)
                    self.save_output()
                elif action == 'skip':
                    self.annotations.pop(episode_id, None)
                    self.drafts.pop(str(episode_id), None)
                    self.skipped.add(episode_id)
                    self.save_output()
                elif action != 'select':
                    raise ValueError('未知操作')
                self.current = episode_id
            self.save_session()
            return self.state()


class ReviewStore(AnnotationStore):
    """Serve center-frame predictions with original videos, without writes."""

    def __init__(self, root, output, predictions, prediction_dataset, legacy_frame_offset=0):
        super().__init__(root, output)
        if type(legacy_frame_offset) is not int or legacy_frame_offset < 0:
            raise ValueError('旧版预测偏移必须是非负整数')
        self.legacy_frame_offset = legacy_frame_offset
        self.predictions_path = Path(predictions).resolve()
        metadata_root = Path(prediction_dataset).resolve() / 'meta'
        info = json.loads((metadata_root / 'info.json').read_text())
        if float(info['fps']) != self.fps:
            raise ValueError('预测数据和原视频帧率不一致')
        records = sorted([json.loads(line) for line in
                          (metadata_root / 'episodes.jsonl').read_text().splitlines()
                          if line.strip()], key=lambda row: row['episode_index'])
        if not records:
            raise ValueError('验证集没有 episode')
        mapping, offsets, offset = {}, {}, 0
        for record in records:
            validation_id = record['episode_index']
            original_id = record['source_episode_index']
            if validation_id in mapping or original_id not in self.by_id:
                raise ValueError('验证集编号重复或无法映射到原视频')
            if record['length'] != self.by_id[original_id]['length']:
                raise ValueError('预测集与原视频长度不一致')
            if original_id not in self.annotations:
                raise ValueError(f'缺少原始人工标签: episode {original_id}')
            dense = self.annotations[original_id]['dense']
            for field, metadata_key in (
                ('names', 'dense_subtask_names'),
                ('start_frames', 'dense_subtask_start_frames'),
                ('end_frames', 'dense_subtask_end_frames'),
            ):
                if dense[field] != record[metadata_key]:
                    raise ValueError(f'验证集人工标签与原标签不同: episode {original_id}')
            mapping[validation_id] = original_id
            offsets[validation_id] = offset
            offset += record['length']
        if len(set(mapping.values())) != len(mapping):
            raise ValueError('多个验证集编号映射到同一个原视频')
        priors = json.loads((metadata_root / 'temporal_proportions_dense.json').read_text())
        if list(priors) != self.names:
            raise ValueError('阶段时间占比与人工标签阶段不一致')
        weights = [float(priors[name]) for name in self.names]
        if any(not math.isfinite(w) or w < 0 for w in weights) or sum(weights) <= 0:
            raise ValueError('阶段时间占比无效')
        if abs(sum(weights) - 1.0) > 1e-6:
            total = sum(weights)
            weights = [w / total for w in weights]
        self.review_data = {}
        for validation_id, original_id in mapping.items():
            episode = self.by_id[original_id]
            dense = self.annotations[original_id]['dense']
            reference, stages = [], []
            for stage, (start, end) in enumerate(zip(dense['start_frames'], dense['end_frames'])):
                for frame in range(start, end + 1):
                    tau = (frame - start) / (end - start) if end > start else 1.0
                    if stage == len(self.names) - 1 and tau >= 1:
                        tau = .999
                    reference.append(min(1., max(0., sum(weights[:stage]) + weights[stage] * tau)))
                    stages.append(stage)
            self.review_data[original_id] = dict(
                episode_index=original_id, validation_episode_index=validation_id,
                predicted=[None] * episode['length'], reference=reference,
                stage_indices=stages, names=self.names,
                start_frames=dense['start_frames'], end_frames=dense['end_frames'])
        seen = {key: set() for key in self.review_data}
        latest_source = {key: {} for key in self.review_data}
        with self.predictions_path.open(encoding='utf-8') as handle:
            for line in handle:
                if not line.strip():
                    continue
                row = json.loads(line)
                if legacy_frame_offset and row.get('frame_alignment') == 'center':
                    raise ValueError('中心帧结果不能再添加旧版偏移')
                if not legacy_frame_offset and row.get('frame_alignment') != 'center':
                    raise ValueError('需要中心帧预测，请先运行 validate_sarm_a10_center.sh')
                validation_id = row['episode_index']
                if validation_id not in mapping:
                    raise ValueError('预测包含未知验证集编号')
                original_id = mapping[validation_id]
                frame = (row['current_index'] - offsets[validation_id]
                         if legacy_frame_offset else row['frame_index'])
                values = self.review_data[original_id]['predicted']
                if (type(frame) is not int or not 0 <= frame < len(values) or
                        row['current_index'] != offsets[validation_id] + frame or
                        row.get('source_episode_index', original_id) != original_id):
                    raise ValueError('预测帧号或原视频编号不匹配')
                value = row['pred_progress']
                if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                        not math.isfinite(value) or not 0 <= value <= 1):
                    raise ValueError('预测 progress 无效')
                if frame in seen[original_id]:
                    raise ValueError('预测包含重复帧')
                seen[original_id].add(frame)
                target = min(frame + legacy_frame_offset, len(values) - 1)
                if frame > latest_source[original_id].get(target, -1):
                    values[target] = float(value)
                    latest_source[original_id][target] = frame
        for original_id, data in self.review_data.items():
            if len(seen[original_id]) != len(data['predicted']):
                raise ValueError(f'预测未完整覆盖 episode {original_id}')
        self.episodes = [episode for episode in self.episodes
                         if episode['episode_index'] in self.review_data]
        self.by_id = {episode['episode_index']: episode for episode in self.episodes}
        self.annotations = {key: value for key, value in self.annotations.items()
                            if key in self.by_id}
        self.drafts, self.skipped = {}, set()
        self.current = self.episodes[0]['episode_index']

    def state(self):
        result = super().state()
        result.update(review=True, predictions=str(self.predictions_path),
                      frame_alignment='window_end' if self.legacy_frame_offset else 'center',
                      legacy_frame_offset=self.legacy_frame_offset)
        return result

    def update(self, action, data):
        if action != 'select':
            raise PermissionError('查看模式不允许修改标注')
        episode_id = int(data['episode_index'])
        if episode_id not in self.by_id:
            raise ValueError('未知验证集')
        with self.lock:
            self.current = episode_id
            return self.state()


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send_json(self, value, status=200):
            payload = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def do_POST(self):
            # Reject cross-origin browser writes to this local service.
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.send_json(dict(error='拒绝跨来源写入'), 403)
                return
            try:
                size = int(self.headers.get('Content-Length', 0))
                if size <= 0 or size > 1024 * 1024:
                    raise ValueError('请求大小不合法')
                data = json.loads(self.rfile.read(size))
                self.send_json(store.update(urlparse(self.path).path.removeprefix('/api/'), data))
            except (ValueError, KeyError, TypeError) as error:
                self.send_json(dict(error=str(error)), 400)
            except PermissionError as error:
                self.send_json(dict(error=str(error)), 403)
            except OSError as error:
                self.send_json(dict(error=f'保存失败: {error}'), 500)

        def do_GET(self):
            parsed = urlparse(self.path)
            try:
                if parsed.path == '/api/state':
                    with store.lock:
                        self.send_json(store.state())
                    return
                if parsed.path == '/api/predictions' and isinstance(store, ReviewStore):
                    query = parse_qs(parsed.query)
                    self.send_json(store.review_data[int(query['episode'][0])])
                    return
                if parsed.path == '/':
                    path = Path(__file__).with_name('manual_video_annotator.html')
                elif parsed.path == '/video':
                    query = parse_qs(parsed.query)
                    path = store.video_path(int(query['episode'][0]), query['camera'][0])
                else:
                    self.send_error(404)
                    return
                size = path.stat().st_size
                start, end = 0, size - 1
                byte_range = self.headers.get('Range')
                if byte_range:
                    try:
                        if not byte_range.startswith('bytes=') or ',' in byte_range:
                            raise ValueError()
                        left, right = byte_range[6:].split('-', 1)
                        if left:
                            start = int(left)
                            end = min(int(right), size - 1) if right else size - 1
                        else:
                            suffix = int(right)
                            if suffix <= 0:
                                raise ValueError()
                            start = max(0, size - suffix)
                        if start < 0 or start > end or start >= size:
                            raise ValueError()
                    except ValueError:
                        self.send_response(416)
                        self.send_header('Content-Range', f'bytes */{size}')
                        self.send_header('Content-Length', '0')
                        self.end_headers()
                        return
                self.send_response(206 if byte_range else 200)
                self.send_header('Content-Type', mimetypes.guess_type(path)[0] or 'application/octet-stream')
                self.send_header('Accept-Ranges', 'bytes')
                self.send_header('Content-Length', str(end - start + 1))
                if byte_range:
                    self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
                self.end_headers()
                with path.open('rb') as handle:
                    handle.seek(start)
                    remaining = end - start + 1
                    while remaining:
                        block = handle.read(min(256 * 1024, remaining))
                        if not block:
                            break
                        self.wfile.write(block)
                        remaining -= len(block)
            except (ValueError, KeyError, FileNotFoundError):
                self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path, default=Path('datasets/a10_9_21_sarm'))
    parser.add_argument('--output', type=Path, default=Path('datasets/a10_stages.json'))
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--check', action='store_true', help='只读检查视频帧数，不启动服务')
    parser.add_argument('--review', action='store_true', help='只读查看视频和中心帧预测')
    parser.add_argument('--predictions', type=Path,
                        default=Path('sarm_runs/a10/validation_center/progress.jsonl'))
    parser.add_argument('--prediction-dataset-root', type=Path,
                        default=Path('datasets/a10_sarm_val'))
    parser.add_argument('--legacy-frame-offset', type=int, default=0,
                        help='显式加载旧版窗口末帧预测，例如 --legacy-frame-offset 120')
    args = parser.parse_args()
    if args.review:
        store = ReviewStore(args.dataset_root, args.output, args.predictions,
                            args.prediction_dataset_root, args.legacy_frame_offset)
    else:
        store = AnnotationStore(args.dataset_root, args.output)
    if args.check:
        import av
        for episode in store.episodes:
            for camera in store.cameras:
                with av.open(str(store.video_path(episode['episode_index'], camera))) as video:
                    stream = video.streams.video[0]
                    count = stream.frames or sum(1 for _ in video.decode(video=0))
                    if count != episode['length'] or abs(float(stream.average_rate) - store.fps) > .01:
                        raise ValueError(f'帧数或 fps 不匹配: {episode["episode_index"]}, {camera}')
        print(f'检查通过: {len(store.episodes)} 集，{len(store.cameras)} 路相机，{store.fps:g} fps')
        return
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(store))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        parser.exit(1, f'端口 {args.port} 已被占用。若标注服务已启动，请打开 '
                    f'http://127.0.0.1:{args.port}。否则停止占用进程后重试，'
                    '或使用 --port 指定其他端口。\n')
    print(f'{"预测查看" if args.review else "标注"}页面: '
          f'http://127.0.0.1:{server.server_port}', flush=True)
    print(f'{"预测数据" if args.review else "输出"}: '
          f'{store.predictions_path if args.review else store.output}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
