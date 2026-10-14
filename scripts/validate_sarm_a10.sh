#!/usr/bin/env bash
# Score the six held-out episodes and render progress/stage curves.
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
  --output-path "${SARM_RUN_DIR}/validation/progress.jsonl" \
  --output-dir "${SARM_RUN_DIR}/validation/figures" "$@"
