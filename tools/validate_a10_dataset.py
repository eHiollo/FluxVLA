"""Validate real A10 v3 samples through FluxVLA without changing the data."""

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import av
import numpy as np
import pyarrow.parquet as pq
from PIL import Image, ImageDraw

from fluxvla.datasets.parquet_dataset_v3 import ParquetDatasetV3


VIDEO_KEYS = ['observation.images.base_0_rgb',
              'observation.images.right_wrist_0_rgb']


def image_hash(image):
    return hashlib.sha256(np.ascontiguousarray(image).tobytes()).hexdigest()


def file_snapshot(root):
    return {str(p.relative_to(root)): (p.stat().st_size, p.stat().st_mtime_ns)
            for p in root.rglob('*') if p.is_file()}


def verify_sequential_video(path, requests, tolerance):
    """Independently match PTS by decoding forward from the video beginning."""
    requests.sort(key=lambda item: item['timestamp'])
    cursor, previous, max_error = 0, None, 0.0
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        for frame in container.decode(stream):
            pts = float(frame.pts * stream.time_base)
            while cursor < len(requests) and requests[cursor]['timestamp'] <= pts:
                request = requests[cursor]
                chosen, chosen_pts = frame, pts
                if previous is not None:
                    previous_pts = float(previous.pts * stream.time_base)
                    if abs(previous_pts - request['timestamp']) < abs(pts - request['timestamp']):
                        chosen, chosen_pts = previous, previous_pts
                error = abs(chosen_pts - request['timestamp'])
                assert error <= tolerance, (path, request, chosen_pts)
                rgb = chosen.to_ndarray(format='rgb24')
                chw = np.transpose(rgb, (2, 0, 1))
                assert image_hash(chw) == request['hash'], (
                    f'Wrong video frame: {path}, {request["sample"]}')
                max_error = max(max_error, error)
                cursor += 1
            if cursor == len(requests):
                break
            previous = frame
    assert cursor == len(requests), f'Video ended before requested frame: {path}'
    return max_error


def validate(root, output, horizon=10):
    if horizon <= 0:
        raise ValueError('horizon must be positive')
    before = file_snapshot(root)
    info = json.loads((root / 'meta/info.json').read_text())
    fps = info['fps']
    transform = dict(
        type='ProcessParquetInputs',
        parquet_keys=['observation.state', 'action', 'actions', 'action_masks',
                      'timestamp', 'episode_index', 'frame_index', 'index'],
        video_keys=VIDEO_KEYS,
        name_mappings={'observation.state': 'states', 'action': 'recorded_actions'},
        video_backend='pyav')
    dataset = ParquetDatasetV3(
        str(root), transforms=[transform], action_key='observation.state',
        action_window_size=horizon, window_start_idx=1, statistic_name='a10',
        expose_index=True)
    # Read the original parquet files independently of FluxVLA's sample API.
    tables = [pq.read_table(p) for p in sorted((root / 'data').rglob('*.parquet'))]
    columns = {name: sum((t[name].to_pylist() for t in tables), [])
               for name in ['index', 'episode_index', 'frame_index', 'timestamp',
                            'task_index', 'observation.state', 'action']}
    states = np.asarray(columns['observation.state'], dtype=np.float32)
    actions = np.asarray(columns['action'], dtype=np.float32)
    episode_ids = np.asarray(columns['episode_index'])
    assert len(dataset) == info['total_frames'] == len(states)
    assert states.shape == (len(dataset), 7)
    assert actions.shape == (len(dataset), 8)
    assert np.isfinite(states).all() and np.isfinite(actions).all()
    np.testing.assert_array_equal(columns['index'], np.arange(len(dataset)))
    np.testing.assert_array_equal(dataset.dataset['index'], columns['index'])
    episodes = dataset.episode_metadata_by_dataset[0]
    assert len(episodes) == info['total_episodes'] == len(np.unique(episode_ids))
    assert len(dataset.tasks[0]) == info['total_tasks']

    requests = defaultdict(list)
    sample_reports, previews = [], []
    preview_episodes = {int(i) for i in np.linspace(0, len(episodes) - 1, 6)}
    stats = {'a10': dataset.stats[0]['stats']}
    for episode_id, meta in sorted(episodes.items()):
        start, end = int(meta['dataset_from_index']), int(meta['dataset_to_index'])
        assert end - start == int(meta['length']) and end - start >= 2
        np.testing.assert_array_equal(episode_ids[start:end], episode_id)
        np.testing.assert_array_equal(columns['frame_index'][start:end], np.arange(end - start))
        np.testing.assert_allclose(columns['timestamp'][start:end],
                                   np.arange(end - start) / fps, atol=1e-5, rtol=0)
        for key in VIDEO_KEYS:
            duration = meta[f'videos/{key}/to_timestamp'] - meta[f'videos/{key}/from_timestamp']
            assert abs(duration - (end - start) / fps) < 1 / fps
        positions = sorted({start, start + (end - start) // 2, end - 2})
        for index in positions:
            sample = dataset.__getitem__(index, stats)
            assert int(sample['index']) == index
            assert int(sample['episode_index']) == episode_id
            np.testing.assert_array_equal(sample['states'], states[index])
            np.testing.assert_array_equal(sample['recorded_actions'], actions[index])
            future = [min(index + offset, end - 1)
                      for offset in range(1, horizon + 1)]
            np.testing.assert_array_equal(sample['actions'], states[future])
            np.testing.assert_array_equal(sample['action_masks'],
                                          [float(index + offset < end)
                                           for offset in range(1, horizon + 1)])
            assert sample['task_description'] == dataset.tasks[0][int(columns['task_index'][index])]
            assert len(sample['images']) == 2 and sample['img_masks'].all()
            frame_report = {'index': index, 'episode': episode_id,
                            'frame': int(sample['frame_index']),
                            'timestamp': float(sample['timestamp']), 'cameras': []}
            for key, image in zip(VIDEO_KEYS, sample['images']):
                assert image.shape == (3, 480, 640) and image.dtype == np.uint8
                relative = info['video_path'].format(
                    video_key=key, chunk_index=int(meta[f'videos/{key}/chunk_index']),
                    file_index=int(meta[f'videos/{key}/file_index']))
                absolute_timestamp = float(sample['timestamp']) + float(meta[f'videos/{key}/from_timestamp'])
                digest = image_hash(image)
                requests[root / relative].append(
                    dict(timestamp=absolute_timestamp, hash=digest,
                         sample=f'episode={episode_id}, frame={sample["frame_index"]}'))
                frame_report['cameras'].append(dict(
                    key=key, path=relative, video_timestamp=absolute_timestamp, sha256=digest))
            sample_reports.append(frame_report)
            if episode_id in preview_episodes and index == start + (end - start) // 2:
                tile = Image.new('RGB', (640, 270), 'white')
                for camera, image in enumerate(sample['images']):
                    picture = Image.fromarray(np.transpose(image, (1, 2, 0)))
                    tile.paste(picture.resize((320, 240)), (camera * 320, 30))
                ImageDraw.Draw(tile).text(
                    (8, 8), f'Episode {episode_id} / frame {sample["frame_index"]} | base camera / right wrist',
                    fill='black')
                previews.append(tile)
        print(f'Checked episode {episode_id}: {end - start} rows, {len(positions)} paired image samples', flush=True)

    max_error = 0.0
    for path, items in sorted(requests.items()):
        max_error = max(max_error, verify_sequential_video(path, items, 0.5 / fps + 1e-5))
        print(f'Independent video check: {path.name} / {path.parent.parent.name} / {len(items)} frames', flush=True)
    assert before == file_snapshot(root), 'Source dataset files changed during validation.'
    output.mkdir(parents=True, exist_ok=True)
    contact = Image.new('RGB', (1280, 270 * ((len(previews) + 1) // 2)), 'white')
    for i, tile in enumerate(previews):
        contact.paste(tile, ((i % 2) * 640, (i // 2) * 270))
    contact.save(output / 'camera_preview.jpg', quality=92)
    report = dict(
        status='PASS', dataset=str(root), version=info['codebase_version'], fps=fps,
        episodes=len(episodes), frames=len(dataset), state_dim=7, recorded_action_dim=8,
        future_state_target_shape=[horizon, 7], sampled_rows=len(sample_reports),
        decoded_images=len(sample_reports) * 2, referenced_video_files=len(requests),
        max_video_timestamp_error_s=max_error, source_files_unchanged=True,
        note='All numeric rows validated; images sampled at episode start/middle/penultimate frame. This is a data-read test, not a model training test.',
        tasks=dataset.tasks[0], samples=sample_reports)
    (output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2, ensure_ascii=False))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset-root', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'datasets/a10/a10_vr_9_21_4')
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'work_dirs/a10/validation')
    parser.add_argument('--horizon', type=int, default=10)
    args = parser.parse_args()
    validate(args.dataset_root.resolve(), args.output_dir.resolve(), args.horizon)
