"""Backend selection for dual A10 control; LongWAM starts read-only."""

from copy import deepcopy
from pathlib import Path


def backend_settings(name, checkpoint_dir):
    """Return protocol and server transforms, without reading checkpoint files."""
    if name == 'openpi':
        return dict(remote_inference=dict(
            protocol='openpi_ws', server_host='127.0.0.1', server_port=8001,
            client_root='../openpi/packages/openpi-client/src'), prompt=None)
    if name == 'longwam':
        return dict(remote_inference=dict(
            protocol='longwam_http', server_host='127.0.0.1', server_port=8765),
            prompt='Fold the clothes.')
    if name != 'fluxvla':
        raise ValueError('model_backend must be fluxvla, openpi or longwam')
    root = Path(checkpoint_dir)
    stats = str(root / 'dataset_statistics.json')
    return dict(
        remote_inference=dict(protocol='fluxvla_zmq', server_host='127.0.0.1', server_port=5556),
        prompt='fold the black cloth using both arms', task_suite_name='a10_dual',
        dataset=dict(type='A10NativeInferenceDataset', norm_stats=stats,
                     tokenizer_path=str(root / 'tokenizer'), state_dim=14,
                     statistic_name='a10_dual', norm_type='quantile',
                     camera_names=['base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb']),
        denormalize_action=dict(type='A10NativeDenormalizeAction', norm_stats=stats,
                               state_dim=14, statistic_name='a10_dual', norm_type='quantile',
                               relative_mask=([True] * 6 + [False]) * 2),
    )


def select_a10_backend(cfg, backend=None, overrides=None):
    """Resolve a CLI backend override after mmengine's general config overrides.

    LongWAM starts read-only unless the CLI explicitly overrides its mode.
    Other control settings and explicit endpoint/timeout overrides remain intact.
    Replacing the transport also replaces its default port and wire-specific
    client settings; the model checkpoint belongs exclusively to the server.
    """
    if 'model_backend' not in cfg:
        if backend is not None:
            raise ValueError('--model requires the unified A10 inference config')
        return
    selected = cfg.model_backend if backend is None else backend
    settings = backend_settings(selected, cfg.model_checkpoint_dir)
    current = dict(cfg.inference.remote_inference)
    changed = current.get('protocol') != settings['remote_inference']['protocol']
    timeout = current.get('timeout_s', 3600.)
    cfg.model_backend = selected
    if changed:
        cfg.inference.remote_inference = dict(settings['remote_inference'], timeout_s=timeout)
        cfg.inference.remote_inference.server_host = current.get('server_host', '127.0.0.1')
        for key in ('dataset', 'denormalize_action', 'task_suite_name'):
            cfg.inference.pop(key, None)
        cfg.inference.prompt = settings['prompt']
        if selected == 'longwam':
            cfg.inference.mode = 'read_only'
        for key, value in settings.items():
            if key not in ('remote_inference', 'prompt'):
                cfg.inference[key] = deepcopy(value)
    if selected == 'fluxvla' and not changed:
        previous_root = cfg.get('inference_model', {}).get('checkpoint_dir')
        if previous_root and previous_root != cfg.model_checkpoint_dir:
            previous = backend_settings('fluxvla', previous_root)
            for section, keys in (('dataset', ('norm_stats', 'tokenizer_path')),
                                  ('denormalize_action', ('norm_stats',))):
                for key in keys:
                    if cfg.inference[section].get(key) == previous[section][key]:
                        cfg.inference[section][key] = settings[section][key]
    if selected != 'fluxvla':
        cfg.pop('inference_model', None)
        for key in ('dataset', 'denormalize_action', 'task_suite_name'):
            cfg.inference.pop(key, None)
    else:
        cfg.inference_model = dict(type='NativeA10Policy', checkpoint_dir=cfg.model_checkpoint_dir,
                                   weight_file='checkpoints/model.safetensors',
                                   model_config_file='model_config.json',
                                   model_overrides=dict(num_steps=10))
    # Backend defaults are selected first; explicit CLI endpoint/pipeline
    # overrides win even when the transport changed in this invocation.
    model_options = {key: value for key, value in (overrides or {}).items()
                     if key.startswith('inference.remote_inference.')
                     or key == 'inference.prompt'
                     or key == 'inference.mode'
                     or (selected == 'fluxvla' and key.startswith(
                         ('inference.dataset.', 'inference.denormalize_action.', 'inference_model.')))}
    if model_options:
        cfg.merge_from_dict(model_options)
