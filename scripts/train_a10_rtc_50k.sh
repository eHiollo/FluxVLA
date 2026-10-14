#!/usr/bin/env bash
# Run A10 PI0.5 RTC fine-tuning. Checkpoints and logs stay on /data.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="/data/allen/fluxvla"
RUN_DIR="${FLUXVLA_RTC_RUN_DIR:-${DATA_ROOT}/runs/pi05_a10_vr_9_21_4_rtc_50k}"

if [[ "$(df --output=target "${RUN_DIR%/*}" | tail -1 | xargs)" != "/data" ]]; then
  echo "Expected ${RUN_DIR} to be on the /data filesystem" >&2
  exit 1
fi

mkdir -p "${RUN_DIR}" "${DATA_ROOT}/cache/huggingface" "${DATA_ROOT}/cache/pip"
cd "${PROJECT_ROOT}"
source /home/allen/miniconda3/etc/profile.d/conda.sh
set +u
conda activate fluxvla
set -u
export HF_HOME="${DATA_ROOT}/cache/huggingface"
export PIP_CACHE_DIR="${DATA_ROOT}/cache/pip"
export TOKENIZERS_PARALLELISM=false
export WANDB_MODE=disabled

resume_args=()
if [[ -f "${RUN_DIR}/checkpoints/latest-checkpoint.pt" ]]; then
  resume_args=(--resume-from "${RUN_DIR}/checkpoints/latest-checkpoint.pt")
fi

torchrun --nnodes=1 --nproc_per_node=1 \
  --master_addr=127.0.0.1 --master_port=29553 scripts/train.py \
  --config configs/pi05/pi05_paligemma_a10_vr_9_21_4_rtc_50k.py \
  --work-dir "${RUN_DIR}" \
  "${resume_args[@]}" \
  "$@" 2>&1 | tee -a "${RUN_DIR}/train.log"
