#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export WANDB_MODE=disabled
export USE_TF=0
export TOKENIZERS_PARALLELISM=false
export OMP_NUM_THREADS=4
export HF_HUB_OFFLINE=1
# Fail before GPU initialization if an exported split was moved or deleted.
python - <<'PY'
import json
from pathlib import Path

for name in ('a10_sarm_train', 'a10_sarm_val'):
    root = Path('datasets') / name
    info = json.loads((root / 'meta/info.json').read_text())
    episodes = [json.loads(line) for line in
                (root / 'meta/episodes.jsonl').read_text().splitlines() if line.strip()]
    cameras = [key for key, feature in info['features'].items() if feature['dtype'] == 'video']
    if len(episodes) != info['total_episodes']:
        raise RuntimeError(f'Episode metadata mismatch: {root}')
    for episode in episodes:
        i = episode['episode_index']
        for key in cameras:
            path = root / info['video_path'].format(episode_index=i,
                episode_chunk=i // info['chunks_size'], video_key=key)
            if not path.is_file():
                raise FileNotFoundError(f'Missing training/validation video: {path}')
    print(f'Preflight OK: {root}, {len(episodes)} episodes, {len(cameras)} cameras', flush=True)
PY
torchrun --nnodes 1 --nproc-per-node 1 \
  --master_addr=127.0.0.1 --master_port="${SARM_MASTER_PORT:-29563}" scripts/train.py \
  --config configs/sarm/sarm_a10.py --work-dir ./sarm_runs/a10 "$@"
