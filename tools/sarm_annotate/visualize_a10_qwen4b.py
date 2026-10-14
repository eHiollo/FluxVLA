#!/usr/bin/env python
"""Render Qwen A10 stage starts and transitions without loading the VLM."""
import argparse
import json
from pathlib import Path

import av
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pyarrow.parquet as pq

LABELS = ['Pick up bottle', 'Put bottle in box', 'Close lid', 'Return home']
COLORS = ['#2780ba', '#f3a23b', '#47a979', '#ab74ba']


def render(root, output):
    root, output = Path(root), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    info = json.loads((root / 'meta/info.json').read_text())
    rows = pq.read_table(root / 'meta/episodes/chunk-000/file-000.parquet').to_pylist()
    camera = 'observation.images.base_0_rgb'
    for row in rows:
        if not row.get('dense_subtask_start_frames'):
            continue
        starts, n, fps = row['dense_subtask_start_frames'], row['length'], info['fps']
        indices = sorted(set([0, n-1] + [max(0,s-15) for s in starts[1:]] +
                             [min(n-1,s+15) for s in starts[1:]]))
        path = root / info['video_path'].format(video_key=camera,
            chunk_index=row[f'videos/{camera}/chunk_index'],
            file_index=row[f'videos/{camera}/file_index'])
        frames = {}
        with av.open(str(path)) as c:
            for i, f in enumerate(c.decode(video=0)):
                if i in indices:
                    frames[i] = f.to_image()
        fig = plt.figure(figsize=(16, 8), layout='constrained')
        grid = fig.add_gridspec(3, 4, height_ratios=[1,1,.6])
        for j, index in enumerate(indices):
            ax = fig.add_subplot(grid[j//4, j%4])
            ax.imshow(frames[index]); ax.set_title(f'{index/fps:.2f}s (frame {index})'); ax.axis('off')
        ax = fig.add_subplot(grid[2,:])
        for j, (s,e) in enumerate(zip(starts, row['dense_subtask_end_frames'])):
            ax.barh(0,(e-s+1)/fps,left=s/fps,color=COLORS[j],label=LABELS[j],height=.5)
            if j:
                ax.axvline(s/fps,color='black',linewidth=.6)
                ax.text(s/fps,.32,f'{s/fps:.2f}s',ha='center')
        ax.set_xlim(0,n/fps);ax.set_yticks([]);ax.set_xlabel('Seconds');ax.legend(loc='lower center',ncol=4)
        evidence = json.loads((root / f'meta/qwen4b_annotations/episode_{row["episode_index"]:06d}_qwen_4b.json').read_text())
        fig.suptitle(f'{root.name} episode {row["episode_index"]} / source {row["source_episode_index"]} '
                     f'/ Qwen3-VL-4B / uncertain={evidence["response"]["uncertain"]}')
        fig.savefig(output / f'episode_{row["episode_index"]:06d}.png',dpi=100)
        plt.close(fig)
    print(f'Previews saved to {output}',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root',required=True)
    parser.add_argument('--output-dir',required=True)
    args=parser.parse_args()
    render(args.dataset_root,args.output_dir)
