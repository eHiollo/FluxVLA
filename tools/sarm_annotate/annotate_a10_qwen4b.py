#!/usr/bin/env python
"""Annotate standalone A10 v3 splits with local Qwen3-VL and resumable evidence."""
import argparse
import hashlib
import json
import re
import time
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import torch
from transformers import AutoProcessor, Qwen3VLForConditionalGeneration

NAMES = ['抓起水瓶', '将水瓶放入旁边的盒子中', '将盒子盖上', '回到初始位置']
CAMERA = 'observation.images.base_0_rgb'


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def sample_video(path, length, fps, sample_fps):
    indices = sorted(set(range(0, length, max(1, round(fps / sample_fps)))) | {length-1})
    wanted = set(indices)
    frames = []
    with av.open(str(path)) as container:
        for index, frame in enumerate(container.decode(video=0)):
            if index in wanted:
                frames.append(frame.reformat(width=384, height=288).to_ndarray(format='rgb24'))
    if len(frames) != len(indices):
        raise ValueError(f'Missing video frames: {path}')
    return np.stack(frames), dict(total_num_frames=length, fps=fps,
                                 duration=length / fps, frames_indices=indices)


def prompt(length, fps):
    return f'''观察按时间顺序提供的机器人视频截图，视频时长 {length/fps:.3f} 秒，最后一帧时间 {(length-1)/fps:.3f} 秒。
三个事件的时间必须递增，且不得超过最后一帧时间。
你的任务是定位三个具体的视觉事件，不是平均分段，也不是猜测运动意图。严格读每张截图前的原始秒数。
A = 水瓶已经完整抬离台面、瓶底不再接触支撑面，并开始被搬向旁边的盒子。
特别注意：夹爪接近瓶子、夹住瓶子但瓶底仍在台面上，都不算 A。即使机器人有运动，瓶底还在台面也不算 A。
B = 水瓶已经放入盒子，夹爪已经松开水瓶，开始移向盒盖准备关盖。
注意：水瓶仍悬在盒子上方，或夹爪仍夹住水瓶时，不算 B。
C = 盒盖已经实际完全闭合，并且机械臂开始离开盒子返回初始位置。
特别注意：盖子开始运动不是 C，盖子仍打开或仍能看见盒内水瓶时不算 C。
timeline 最多六条，只描述关键变化，不要逐张重复图片。请给出 A、B、C 的秒数和画面证据。
如果某个事件因遮挡无法确定，uncertain 必须为 true，但仍提供你最合理的估计。
不要把开始接近水瓶的时刻当作 A，不要把开始操作盒盖的时刻当作 C。
仅输出以下 JSON，不要 Markdown：
{{"bottle_lifted_and_transporting_seconds": A的秒数,
"gripper_released_bottle_and_moves_to_lid_seconds": B的秒数,
"lid_fully_closed_and_returning_seconds": C的秒数,
"boundary_evidence": ["A的可见证据", "B的可见证据", "C的可见证据"], "uncertain": false,
"timeline": [{{"seconds": 秒数, "observation": "实际可见的瓶底、夹爪和盒盖状态"}}]}}'''


def validate_response(response, length, fps):
    decoder = json.JSONDecoder()
    result = None
    # The optional timeline may be cut off by the output limit. Recover only
    # a complete JSON prefix containing all actual event timestamps/evidence;
    # never infer a missing event or synthesize a model timestamp.
    candidates = [response]
    timeline = re.search(r',\s*"timeline"\s*:', response)
    if timeline:
        candidates.append(response[:timeline.start()] + '}')
    for candidate_text in candidates:
        for pos, char in enumerate(candidate_text):
            if char == '{':
                try:
                    candidate, _ = decoder.raw_decode(candidate_text[pos:])
                    keys = {'bottle_lifted_and_transporting_seconds',
                            'gripper_released_bottle_and_moves_to_lid_seconds',
                            'lid_fully_closed_and_returning_seconds'}
                    if isinstance(candidate, dict) and keys <= set(candidate):
                        result = candidate
                        if candidate_text != response:
                            result['optional_timeline_truncated'] = True
                        break
                except json.JSONDecodeError:
                    continue
        if result:
            break
    if not result:
        raise ValueError('Missing JSON')
    result['names'] = NAMES
    result['start_seconds'] = [0, result['bottle_lifted_and_transporting_seconds'],
        result['gripper_released_bottle_and_moves_to_lid_seconds'],
        result['lid_fully_closed_and_returning_seconds']]
    seconds = np.asarray(result['start_seconds'], dtype=float)
    if seconds.shape != (4,) or not np.isfinite(seconds).all() or abs(seconds[0]) > 1e-6:
        raise ValueError('Invalid starts')
    starts = np.rint(seconds * fps).astype(int)
    if starts[-1] >= length and seconds[-1] <= (length-1)/fps + .05:
        # A last-frame timestamp rounded to one decimal can round past N-1.
        starts[-1] = length-1
        result['timestamp_rounding_at_video_end'] = True
    if not np.all(np.diff(starts) > 0) or starts[-1] >= length:
        raise ValueError('Non-increasing or out-of-range stage boundaries')
    if len(result.get('boundary_evidence', [])) != 3 or not isinstance(result.get('uncertain'), bool):
        raise ValueError('Missing boundary evidence or uncertainty')
    closed_times = [event['seconds'] for event in result.get('timeline', [])
                    if any(word in event.get('observation', '')
                           for word in ('已完全闭合', '已完全关闭', '已经完全闭合'))]
    # Later closed-lid observations describe the same state, not new closures.
    if closed_times and seconds[3] < min(closed_times):
        raise ValueError(f'时间线在{min(closed_times)}秒才确认盒盖完全关闭，第四阶段不能从{seconds[3]}秒开始；请核对真实画面，修正矛盾')
    return result, starts.tolist(), (starts[1:] - 1).tolist() + [length - 1]


def save_columns(root, ep, starts, ends, fps):
    path = root / 'meta/episodes/chunk-000/file-000.parquet'
    rows = pq.read_table(path).to_pylist()
    row = next(r for r in rows if r['episode_index'] == ep)
    n = row['length']
    row.update(sparse_subtask_names=['task'], sparse_subtask_start_frames=[0],
               sparse_subtask_end_frames=[n-1], dense_subtask_names=NAMES,
               dense_subtask_start_frames=starts, dense_subtask_end_frames=ends,
               dense_subtask_start_times=[v/fps for v in starts],
               dense_subtask_end_times=[v/fps for v in ends])
    # pyarrow infers the column set from the first row. If episode zero is
    # not yet annotated, include null columns there so other labels survive.
    keys = sorted(set().union(*(r.keys() for r in rows)))
    rows = [{key: r.get(key) for key in keys} for r in rows]
    temporary = path.with_suffix('.parquet.tmp')
    pq.write_table(pa.Table.from_pylist(rows), temporary)
    temporary.replace(path)


def annotate(args):
    roots = [Path(p).resolve() for p in args.dataset_roots]
    processor = AutoProcessor.from_pretrained(args.model, local_files_only=True)
    model = Qwen3VLForConditionalGeneration.from_pretrained(
        args.model, dtype=torch.bfloat16, attn_implementation='sdpa', local_files_only=True).to('cuda').eval()
    torch.manual_seed(42)
    print(f'Model loaded, allocated GPU: {torch.cuda.memory_allocated()/1024**3:.2f} GiB', flush=True)
    total, completed, failed = 0, 0, []
    for root in roots:
        info = json.loads((root / 'meta/info.json').read_text())
        rows = pq.read_table(root / 'meta/episodes/chunk-000/file-000.parquet').to_pylist()
        evidence_dir = root / 'meta/qwen4b_annotations'
        evidence_dir.mkdir(exist_ok=True)
        for row in rows:
            ep, length, fps = row['episode_index'], row['length'], info['fps']
            if args.episodes is not None and ep not in args.episodes:
                continue
            if args.refine_short_stages:
                if not row.get('dense_subtask_names'):
                    continue
                durations = (np.array(row['dense_subtask_end_frames']) -
                             np.array(row['dense_subtask_start_frames']) + 1) / fps
                if durations.min() >= 1.:
                    continue
            total += 1
            evidence_file = evidence_dir / f'episode_{ep:06d}_qwen_4b.json'
            if evidence_file.exists() and not (args.overwrite or args.refine_short_stages):
                evidence = json.loads(evidence_file.read_text())
                save_columns(root, ep, evidence['start_frames'], evidence['end_frames'], fps)
                completed += 1
                print(f'{root.name}/{ep}: resumed saved label', flush=True)
                continue
            path = root / info['video_path'].format(video_key=CAMERA,
                chunk_index=row[f'videos/{CAMERA}/chunk_index'],
                file_index=row[f'videos/{CAMERA}/file_index'])
            frames, metadata = sample_video(path, length, fps, args.sample_fps)
            base_prompt = prompt(length, fps)
            start_time = time.monotonic()
            responses = []
            feedback = ''
            for attempt in range(3):
                instruction = base_prompt if not attempt else base_prompt + '\n上次输出存在以下问题，请根据原视频修正：' + feedback + '\n上次输出：' + responses[-1]
                content = [{'type': 'text', 'text': instruction + '\n以下图片按时间顺序采自同一视频。每张图片前明确写了原始时间戳，请严格使用这些时间戳。'}]
                for index in metadata['frames_indices']:
                    content.extend([{'type': 'text', 'text': f'原始视频时间 {index/fps:.3f} 秒：'},
                                    {'type': 'image', 'image': 'sampled_frame'}])
                messages = [{'role': 'user', 'content': content}]
                text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
                inputs = processor(text=[text], images=list(frames), do_resize=False,
                    return_tensors='pt').to('cuda')
                try:
                    with torch.inference_mode():
                        output = model.generate(**inputs, max_new_tokens=768, do_sample=False)
                    response = processor.batch_decode(output[:, inputs.input_ids.shape[1]:], skip_special_tokens=True)[0]
                    responses.append(response)
                    result, starts, ends = validate_response(response, length, fps)
                    break
                except ValueError as error:
                    feedback = str(error)
                    print(f'{root.name}/{ep}: retry {attempt+1}: {error}', flush=True)
            else:
                atomic_json(evidence_dir / f'episode_{ep:06d}_qwen_4b.failed.json', {'responses': responses})
                failed.append(f'{root.name}/{ep}')
                continue
            evidence = dict(model=args.model, source_episode_index=row['source_episode_index'],
                episode_index=ep, video_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                sample_fps=args.sample_fps, sampled_frame_indices=metadata['frames_indices'],
                input_method='chronological RGB frames with explicit original timestamps',
                response=result, raw_responses=responses, names=NAMES,
                start_frames=starts, end_frames=ends,
                elapsed_seconds=time.monotonic()-start_time,
                peak_gpu_gib=torch.cuda.max_memory_allocated()/1024**3)
            atomic_json(evidence_file, evidence)
            save_columns(root, ep, starts, ends, fps)
            completed += 1
            print(f'{root.name}/{ep} source={row["source_episode_index"]}: starts={starts}, uncertain={result["uncertain"]}, {evidence["elapsed_seconds"]:.1f}s, peak={evidence["peak_gpu_gib"]:.2f}GiB', flush=True)
            del inputs, output, frames
    # Both splits use priors from training labels only.
    train_rows = pq.read_table(roots[0] / 'meta/episodes/chunk-000/file-000.parquet').to_pylist()
    labeled = [r for r in train_rows if r.get('dense_subtask_names')]
    if labeled:
        shares = np.mean([(np.asarray(r['dense_subtask_end_frames'])-
            np.asarray(r['dense_subtask_start_frames'])+1)/r['length'] for r in labeled], axis=0)
        for root in roots:
            atomic_json(root / 'meta/temporal_proportions_dense.json', dict(zip(NAMES, shares.tolist())))
            atomic_json(root / 'meta/temporal_proportions_sparse.json', {'task': 1.})
    combined = []
    for root in roots:
        rows = pq.read_table(root / 'meta/episodes/chunk-000/file-000.parquet').to_pylist()
        exported = []
        for row in rows:
            if not row.get('dense_subtask_names'):
                continue
            evidence = json.loads((root / f'meta/qwen4b_annotations/episode_{row["episode_index"]:06d}_qwen_4b.json').read_text())
            entry = dict(episode_index=row['episode_index'], source_episode_index=row['source_episode_index'],
                model='Qwen/Qwen3-VL-4B-Instruct', uncertain=evidence['response']['uncertain'],
                review_required=(evidence['response']['uncertain'] or
                    min(b-a+1 for a,b in zip(row['dense_subtask_start_frames'],row['dense_subtask_end_frames'])) < 30),
                dense=dict(names=row['dense_subtask_names'], start_frames=row['dense_subtask_start_frames'],
                           end_frames=row['dense_subtask_end_frames']))
            exported.append(entry)
            combined.append(dict(entry, episode_index=row['source_episode_index'],
                                 split=root.name, split_episode_index=row['episode_index']))
        atomic_json(root.parent / f'{root.name}_qwen_4b_labels.json', exported)
    atomic_json(roots[0].parent / 'a10_sarm_v3_qwen_4b_labels.json', sorted(combined, key=lambda x:x['episode_index']))
    print(json.dumps(dict(requested=total, completed=completed, failed=failed), ensure_ascii=False), flush=True)
    if failed:
        raise RuntimeError(f'Incomplete annotations: {failed}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-roots', nargs='+', default=['datasets/a10_sarm_v3_train', 'datasets/a10_sarm_v3_valid'])
    parser.add_argument('--model', default='checkpoints/Qwen3-VL-4B-Instruct')
    parser.add_argument('--episodes', nargs='+', type=int)
    parser.add_argument('--sample-fps', type=float, default=1.)
    parser.add_argument('--overwrite', action='store_true')
    parser.add_argument('--refine-short-stages', action='store_true',
                        help='Re-annotate only episodes containing a stage shorter than one second')
    annotate(parser.parse_args())
