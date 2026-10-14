#!/usr/bin/env bash
# Wait for enough free VRAM before starting or resuming the RTC job.
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR=/data/allen/fluxvla/runs/pi05_a10_vr_9_21_4_rtc_50k
MIN_FREE_MIB=14336
mkdir -p "${RUN_DIR}"
exec > >(tee -a "${RUN_DIR}/start_gate.log") 2>&1
while true; do
  free_mib="$(nvidia-smi --id=0 --query-gpu=memory.free --format=csv,noheader,nounits | tr -d ' ')" || free_mib=0
  if [[ "${free_mib}" =~ ^[0-9]+$ ]] && (( free_mib >= MIN_FREE_MIB )); then
    echo "$(date -Is) GPU 0 has ${free_mib} MiB free; starting RTC training."
    break
  fi
  echo "$(date -Is) GPU 0 has ${free_mib} MiB free; need ${MIN_FREE_MIB} MiB. Checking again in 5 minutes."
  sleep 300
done
export CUDA_VISIBLE_DEVICES=0
exec bash "${PROJECT_ROOT}/scripts/train_a10_rtc_50k.sh"
