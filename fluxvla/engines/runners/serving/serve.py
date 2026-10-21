"""Launch a ZMQ server that serves a VLA model for remote inference.

Usage::

    python -m fluxvla.engines.runners.serving.serve \\
        --config configs/pi05/pi05_paligemma_libero_10_full_finetune.py \\
        --ckpt-path /path/to/checkpoint.pt \\
        --host 0.0.0.0 --port 5555
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path

import torch
from safetensors.torch import load_file


def parse_args():
    parser = argparse.ArgumentParser(
        description='Serve a VLA model via ZMQ for remote inference')
    parser.add_argument(
        '--acceleration', choices=['on', 'off'], default=None,
        help='Select Triton/CUDA Graph (on) or plain PyTorch (off); omitted uses config default')
    parser.add_argument(
        '--config', required=True, help='Path to mmengine config file')
    weights = parser.add_mutually_exclusive_group(required=True)
    weights.add_argument('--ckpt-path', help='Path to native model checkpoint')
    weights.add_argument('--from-config', action='store_true',
                         help='Model factory/config loads its own pretrained weights')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5555)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument(
        '--dtype', default='bf16', choices=['bf16', 'fp16', 'fp32'])
    parser.add_argument(
        '--dataset-key',
        default=None,
        choices=['inference', 'eval'],
        help='Config key to load dataset pipeline from')
    return parser.parse_args()


def main():
    args = parse_args()

    from mmengine import Config

    from fluxvla.engines import build_vla_from_cfg
    from fluxvla.engines.utils.torch_utils import \
        configure_inference_attention_defaults

    configure_inference_attention_defaults()

    cfg = Config.fromfile(args.config)
    from fluxvla.engines.utils.inference_config import select_inference_acceleration
    select_inference_acceleration(cfg, args.acceleration)

    print('[serve] Building VLA model from config ...')
    if hasattr(cfg, 'inference_model'):
        vla = build_vla_from_cfg(cfg.inference_model)
    else:
        vla = build_vla_from_cfg(cfg.model)

    print(f'[serve] Model implementation: {type(vla).__name__}')

    ckpt_path = args.ckpt_path
    if ckpt_path:
        assert Path(ckpt_path).exists(), f'Checkpoint not found: {ckpt_path}'
        print(f'[serve] Loading checkpoint: {ckpt_path}')
        if ckpt_path.endswith('.safetensors'):
            checkpoint = load_file(ckpt_path, device='cpu')
        else:
            checkpoint = torch.load(ckpt_path, map_location='cpu')
        if isinstance(checkpoint, dict) and 'model' in checkpoint:
            state_dict = checkpoint['model']
        else:
            state_dict = checkpoint
        vla.load_state_dict(state_dict, strict=True)
        del checkpoint, state_dict

    data_stat_path = None if ckpt_path is None else os.path.join(
        Path(ckpt_path).resolve().parent.parent, 'dataset_statistics.json')
    if data_stat_path and os.path.isfile(data_stat_path):
        with open(data_stat_path, 'r') as f:
            vla.norm_stats = json.load(f)
        print(f'[serve] Loaded norm_stats from {data_stat_path}')

    from fluxvla.engines import (build_dataset_from_cfg,
                                 build_transform_from_cfg)

    dataset = None
    denormalize_action = None
    task_suite_name = ''
    dataset_key = args.dataset_key
    if dataset_key is None:
        if hasattr(cfg, 'inference') and 'dataset' in cfg.inference:
            dataset_key = 'inference'
        elif hasattr(cfg, 'eval') and 'dataset' in cfg.eval:
            dataset_key = 'eval'

    if dataset_key:
        dataset_cfg = dict(getattr(cfg, dataset_key).dataset)
        if 'norm_stats' not in dataset_cfg and data_stat_path:
            dataset_cfg['norm_stats'] = data_stat_path
        ds_type = dataset_cfg.get('type', '')
        if ckpt_path and 'model_path' not in dataset_cfg and 'Libero' not in ds_type:
            dataset_cfg['model_path'] = os.path.dirname(
                os.path.dirname(ckpt_path))
        if 'task_suite_name' not in dataset_cfg and 'Libero' in ds_type:
            eval_cfg = getattr(cfg, dataset_key, None)
            if eval_cfg and hasattr(eval_cfg, 'task_suite_name'):
                dataset_cfg['task_suite_name'] = eval_cfg.task_suite_name
        dataset = build_dataset_from_cfg(dataset_cfg)
        print(f'[serve] Dataset pipeline built from '
              f'cfg.{dataset_key}.dataset')

        eval_cfg = getattr(cfg, dataset_key, None)
        if eval_cfg and hasattr(eval_cfg, 'denormalize_action'):
            denorm_cfg = dict(eval_cfg.denormalize_action)
            if data_stat_path:
                denorm_cfg['norm_stats'] = data_stat_path
            denormalize_action = build_transform_from_cfg(denorm_cfg)
            print('[serve] Denormalize action transform built')
        if eval_cfg and hasattr(eval_cfg, 'task_suite_name'):
            task_suite_name = eval_cfg.task_suite_name
    else:
        print('[serve] WARNING: No dataset pipeline found in config.')

    dtype_map = {
        'bf16': torch.bfloat16,
        'fp16': torch.float16,
        'fp32': torch.float32
    }

    from .zmq_server import create_server

    server = create_server(
        vla=vla,
        dataset=dataset,
        denormalize_action=denormalize_action,
        task_suite_name=task_suite_name,
        host=args.host,
        port=args.port,
        device=args.device,
        mixed_precision_dtype=dtype_map[args.dtype],
        debug_log=cfg.get('inference', {}).get('debug_log'),
        model_metadata=dict(config_path=str(Path(args.config).resolve()),
                            checkpoint_dir=cfg.get('inference_model', {}).get('checkpoint_dir'),
                            checkpoint_path=args.ckpt_path,
                            acceleration=args.acceleration,
                            dtype=args.dtype,
                            dataset=cfg.get('inference', {}).get('dataset'),
                            denormalize_action=cfg.get('inference', {}).get('denormalize_action')),
    )
    print(f'[serve] ZMQ server starting on tcp://{args.host}:{args.port}')
    try:
        server.run()
    except KeyboardInterrupt:
        server.close()
        print('[serve] Server stopped.')


if __name__ == '__main__':
    main()
