"""Load a native FluxVLA A10 checkpoint and serve it over existing ZMQ."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import torch
from mmengine import Config
from safetensors.torch import load_file

from fluxvla.engines import build_vla_from_cfg
from fluxvla.engines.runners.serving.zmq_server import create_server
from .config import DEFAULT_CONFIG, DEFAULT_SITE, PROJECT_ROOT, load_project
from .online_dataset import A10InferenceDataset
from .transforms import A10DenormalizeAction


def _statistics_path(checkpoint: Path, configured: str | None) -> Path:
    if configured:
        path = Path(configured).expanduser().resolve()
        if path.is_file():
            return path
        raise FileNotFoundError(f'A10 statistics file not found: {path}')
    for directory in (checkpoint.parent, *checkpoint.parents):
        path = directory / 'dataset_statistics.json'
        if path.is_file():
            return path
    raise FileNotFoundError(
        'dataset_statistics.json not found beside or above checkpoint; '
        'set model.statistics_path in site.local.yaml')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    args = parser.parse_args()
    cfg = load_project(os.environ.get('FLUXVLA_A10_CONFIG', DEFAULT_CONFIG),
                       os.environ.get('FLUXVLA_A10_SITE', DEFAULT_SITE))
    checkpoint = Path(args.checkpoint).expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    statistics = _statistics_path(checkpoint, cfg['model']['statistics_path'])
    model_cfg = Config.fromfile(str(PROJECT_ROOT /
                                    'configs/a10/pi05_a10_finetune.py'))
    model = build_vla_from_cfg(model_cfg.inference_model)
    if checkpoint.suffix == '.safetensors':
        state = load_file(str(checkpoint), device='cpu')
    else:
        state = torch.load(checkpoint, map_location='cpu', weights_only=True)
    if isinstance(state, dict) and 'model' in state:
        state = state['model']
    model.load_state_dict(state, strict=True)
    # The live model owns its weights now; release the extra CPU copy before CUDA.
    del state
    tokenizer_path = cfg['model']['tokenizer_path']
    if not tokenizer_path and cfg['model']['base_checkpoint']:
        base = Path(cfg['model']['base_checkpoint'])
        tokenizer_path = str(base.parent if base.suffix else base)
    if not tokenizer_path or not Path(tokenizer_path).exists():
        raise FileNotFoundError('Set model.tokenizer_path to the PI0.5 tokenizer')
    dataset = A10InferenceDataset(str(statistics), tokenizer_path)
    denormalize = A10DenormalizeAction(str(statistics))
    server = create_server(
        model, dataset=dataset, denormalize_action=denormalize,
        task_suite_name='a10', host=cfg['inference']['host'],
        port=cfg['inference']['port'], device='cuda:0',
        mixed_precision_dtype=torch.bfloat16)
    print(f'A10 FluxVLA ready: tcp://{cfg["inference"]["host"]}:'
          f'{cfg["inference"]["port"]}', flush=True)
    try:
        server.run()
    except KeyboardInterrupt:
        server.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
