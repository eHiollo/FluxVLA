"""One checked A10 configuration for recording, training and inference."""

from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = PROJECT_ROOT / 'configs/a10/project.yaml'
DEFAULT_SITE = PROJECT_ROOT / 'configs/a10/site.local.yaml'


def _read_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f'Configuration must be a YAML mapping: {path}')
    return value


def _merge(base: dict, override: dict, prefix: str = '') -> None:
    for key, value in override.items():
        field = f'{prefix}.{key}' if prefix else key
        if key not in base:
            raise ValueError(f'Unknown A10 configuration key: {field}')
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f'Expected mapping at {field}')
            _merge(base[key], value, field)
        else:
            base[key] = value


def load_project(config_path: Path | str = DEFAULT_CONFIG,
                 site_path: Path | str | None = DEFAULT_SITE) -> dict:
    """Load shared settings and optional local overrides, rejecting drift."""
    config_path = Path(config_path).expanduser().resolve()
    config = copy.deepcopy(_read_yaml(config_path))
    if site_path is not None:
        site_path = Path(site_path).expanduser().resolve()
        if site_path.is_file():
            _merge(config, _read_yaml(site_path))
    config['paths']['fluxvla_python'] = config['paths']['fluxvla_python'] or sys.executable
    for section, key in [('paths', 'outputs'), ('dataset', 'root'),
                         ('vr_server', 'tls_directory'), ('model', 'base_checkpoint'),
                         ('model', 'tokenizer_path'), ('model', 'trained_checkpoint'),
                         ('model', 'statistics_path')]:
        value = config[section][key]
        if value:
            path = Path(value).expanduser()
            config[section][key] = str((PROJECT_ROOT / path).resolve()
                                        if not path.is_absolute() else path.resolve())
    validate_project(config)
    return config


def validate_project(config: dict) -> None:
    if config.get('version') != 1:
        raise ValueError('A10 configuration version must be 1')
    robot = config['robot']
    model = config['model']
    dataset = config['dataset']
    if robot['state_dim'] != 7 or robot['action_dim'] != 7:
        raise ValueError('A10 state/action must both have seven dimensions')
    if model['action_source'] != 'observation.state':
        raise ValueError('A10 PI0.5 targets must come from observation.state')
    if (model['joint_action_mode'] != 'delta_from_current_state'
            or model['gripper_action_mode'] != 'absolute'):
        raise ValueError('A10 action convention is joint deltas and absolute gripper')
    if model['family'] != 'pi05' or model['padded_action_dim'] != 32:
        raise ValueError('This workflow supports PI0.5 with 32 model dimensions')
    if int(model['action_horizon']) <= 0:
        raise ValueError('model.action_horizon must be positive')
    if int(config['inference']['max_action_chunk']) > int(model['action_horizon']):
        raise ValueError('inference.max_action_chunk exceeds model.action_horizon')
    if dataset['fps'] != config['vr']['control_fps']:
        raise ValueError('dataset.fps and vr.control_fps must agree')
    if dataset['push_to_hub']:
        raise ValueError('A10 local recording must not upload by default')
    if config['vr']['motion_shaping_mode'] != 'robot_controller':
        raise ValueError('A10 500 Hz controller owns motion shaping')
    if config['vr']['axis_remap'] != [
            [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]:
        raise ValueError('A10 VR body-to-EE axis map is inconsistent')
    cameras = config['cameras']
    if set(cameras) != {'base_0_rgb', 'right_wrist_0_rgb'}:
        raise ValueError('A10 requires the base and right-wrist cameras')
    for key, camera in cameras.items():
        if camera['fps'] != dataset['fps']:
            raise ValueError(f'{key} fps differs from dataset fps')
        if camera['width'] <= 0 or camera['height'] <= 0:
            raise ValueError(f'{key} image size must be positive')
    if not 1 <= int(robot['port']) <= 65535:
        raise ValueError('robot.port must be a TCP port')
    if robot['allow_motion']:
        step = robot['max_joint_step_rad']
        gripper = robot['gripper_mm_range']
        if step is None or not 0 < float(step) <= 1.0:
            raise ValueError('Set verified robot.max_joint_step_rad (0, 1]')
        if (not isinstance(gripper, list) or len(gripper) != 2
                or not 0 <= float(gripper[0]) < float(gripper[1]) <= 100):
            raise ValueError('Set verified robot.gripper_mm_range within [0, 100]')


def fingerprint(config: dict) -> str:
    encoded = json.dumps(config, sort_keys=True, ensure_ascii=False).encode()
    return hashlib.sha256(encoded).hexdigest()
