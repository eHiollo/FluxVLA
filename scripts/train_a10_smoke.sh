#!/usr/bin/env bash
# Two optimizer updates on the local A10 LeRobot v2.1 dataset.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="/data/allen/fluxvla"
RUN_NAME="pi05_a10_vr_9_21_4_smoke"
RUN_DIR="${DATA_ROOT}/runs/${RUN_NAME}"

if [[ "$(df --output=target "${DATA_ROOT}" | tail -1 | xargs)" != "/data" ]]; then
  echo "Expected ${DATA_ROOT} to be on the /data filesystem" >&2
  exit 1
fi

mkdir -p "${RUN_DIR}" "${DATA_ROOT}/cache/huggingface" "${DATA_ROOT}/cache/pip"
cd "${PROJECT_ROOT}"
mkdir -p checkpoints
if [[ ! -e checkpoints/pi05_base ]]; then
  ln -s "${DATA_ROOT}/pretrained/pi05_base" checkpoints/pi05_base
fi
if [[ ! -e work_dirs ]]; then
  ln -s "${DATA_ROOT}/runs" work_dirs
fi

source /home/allen/miniconda3/etc/profile.d/conda.sh
set +u
conda activate fluxvla
set -u
export HF_HOME="${DATA_ROOT}/cache/huggingface"
export PIP_CACHE_DIR="${DATA_ROOT}/cache/pip"
export TOKENIZERS_PARALLELISM=false
export WANDB_MODE=disabled

torchrun --nnodes=1 --nproc_per_node=1 \
  --master_addr=127.0.0.1 --master_port=29552 scripts/train.py \
  --config configs/pi05/pi05_paligemma_a10_vr_9_21_4_smoke.py \
  --work-dir "${RUN_DIR}" \
  "$@" 2>&1 | tee "${RUN_DIR}/train.log"
