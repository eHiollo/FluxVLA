#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python tools/sarm_annotate/manual_video_annotator.py --review \
  --dataset-root ./datasets/a10_9_21_sarm \
  --output ./datasets/a10_stages.json \
  --prediction-dataset-root ./datasets/a10_sarm_val \
  --predictions ./sarm_runs/a10/validation_center/progress.jsonl "$@"
