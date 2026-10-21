"""Explicit model implementation selection shared by inference entrypoints."""

import copy


def select_inference_acceleration(cfg, acceleration):
    """Select a declared implementation before building models or devices.

    None preserves the config default. A remote client cannot select the
    server's implementation; the option belongs on the model server command.
    """
    if acceleration is None:
        return
    if acceleration not in ('on', 'off'):
        raise ValueError('acceleration must be on or off')
    if cfg.get('inference', {}).get('remote_inference'):
        raise ValueError('Set --acceleration on the model server, not the remote client')
    if 'inference_model_accelerated' not in cfg or 'inference_model' not in cfg:
        raise ValueError('This config does not declare selectable inference implementations')
    if acceleration == 'on':
        cfg.inference_model = copy.deepcopy(cfg.inference_model_accelerated)
