"""Strict inference factory for the merged OpenPI A10 PyTorch export."""

import copy
import json
from pathlib import Path

import torch
from safetensors import safe_open
from safetensors.torch import load_file

from fluxvla.engines.utils.root import VLAS


def checkpoint_mapping(model, weight_path):
    """Check every parameter name and shape without reading tensor payloads."""
    mapping = {}
    with safe_open(str(weight_path), framework='pt', device='cpu') as weights:
        keys = set(weights.keys())
        for name, param in model.named_parameters():
            matches = [key for _, key in model._mapped_name_candidates(name) if key in keys]
            if len(matches) != 1:
                raise ValueError(f'{name}: expected one checkpoint mapping, got {matches}')
            key = matches[0]
            if tuple(weights.get_slice(key).get_shape()) != tuple(param.shape):
                raise ValueError(f'{name}: checkpoint shape mismatch')
            mapping[name] = key
        if set(mapping.values()) != keys or len(set(mapping.values())) != len(mapping):
            raise ValueError('Checkpoint contains unused or multiply mapped tensors')
    return mapping


@VLAS.register_module(name='OpenPIA10Policy')
def load_openpi_a10_policy(checkpoint_dir, model_config):
    """Return a native FluxVLA PI05 model; no OpenPI runtime dependency."""
    from fluxvla.engines import build_vla_from_cfg
    root = Path(checkpoint_dir).expanduser().resolve()
    metadata = json.loads((root / 'config.json').read_text())
    cfg = copy.deepcopy(dict(model_config))
    if (metadata.get('pi05') is not True or metadata.get('action_dim') != 32
            or metadata.get('action_horizon') != cfg['n_action_steps']
            or metadata.get('max_token_len') != 128):
        raise ValueError('Checkpoint metadata does not match A10 PI0.5 configuration')
    cfg['pretrained_name_or_path'] = None
    # CPU construction also initializes nonpersistent RoPE/position buffers.
    with torch.device('cpu'):
        model = build_vla_from_cfg(cfg)
    # OpenPI's action expert takes embeddings directly and has no token table.
    model.llm_expert.embed_tokens = None
    weight_path = root / 'model.safetensors'
    mapping = checkpoint_mapping(model, weight_path)
    tensors = load_file(str(weight_path), device='cpu')
    model.load_state_dict({name: tensors[key] for name, key in mapping.items()},
                          strict=True, assign=True)
    # Reject incomplete initialization rather than leaving meta buffers behind.
    for module in model.modules():
        for name, buffer in module.named_buffers(recurse=False):
            if buffer.is_meta:
                raise RuntimeError(f'Unmaterialized model buffer: {type(module).__name__}.{name}')
    return model.eval()
