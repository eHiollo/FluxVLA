"""Native A10 PI0.5 training config; values come from project.yaml."""

_base_ = []  # Use mmengine's regular parser for this generated configuration.

from fluxvla.projects.a10.training import build_config

_config = build_config()
model = _config['model']
inference_model = _config['inference_model']
train_dataloader = _config['train_dataloader']
runner = _config['runner']
