#!/usr/bin/env bash
# Evaluate the observation at each current frame, including episode frame zero.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export USE_TF=0
export TOKENIZERS_PARALLELISM=false
export OMP_NUM_THREADS=4
export HF_HUB_OFFLINE=1
SARM_RUN_DIR="${SARM_RUN_DIR:-./sarm_runs/a10}"
python scripts/infer_sarm_progress.py \
  --config configs/sarm/sarm_a10.py \
  --ckpt-path "${SARM_RUN_DIR}/checkpoints/latest-checkpoint.pt" \
  --head-mode dense --batch-size 8 --num-workers 4 --num-visualizations 6 \
  --output-path "${SARM_RUN_DIR}/validation_center/progress.jsonl" \
  --output-dir "${SARM_RUN_DIR}/validation_center/figures" "$@"

# Check the default complete run; custom smoke/output overrides remain supported.
if [ "$#" -eq 0 ]; then
python - "${SARM_RUN_DIR}/validation_center/progress.jsonl" <<'PY_CHECK'
import json
import math
import sys
from pathlib import Path

records = [json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines() if line]
metadata = [json.loads(line) for line in Path('datasets/a10_sarm_val/meta/episodes.jsonl').read_text().splitlines() if line]
expected_total = sum(episode['length'] for episode in metadata)
assert len(records) == expected_total, (len(records), expected_total)
for episode in metadata:
    rows = [row for row in records if row['episode_index'] == episode['episode_index']]
    assert [row['frame_index'] for row in rows] == list(range(episode['length']))
    assert all(row['source_episode_index'] == episode['source_episode_index'] for row in rows)
    assert all(row['frame_alignment'] == 'center' and math.isfinite(row['pred_progress']) for row in rows)
print(f"中心帧覆盖校验通过：{len(metadata)} 集，{len(records)} 帧，均覆盖第 0 帧至末帧。")
PY_CHECK
fi
