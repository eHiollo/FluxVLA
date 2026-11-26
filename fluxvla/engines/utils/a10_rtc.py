"""A10 RTC guidance bridge: absolute robot targets to normalized model actions."""
import inspect

import numpy as np
import torch


def validate_rtc(config, action_chunk=None, execute_horizon=None):
    config = dict(config or {})
    if not config.get('enabled', False):
        return config
    if config.get('method') not in ('guidance', 'prefix'):
        raise ValueError('A10 RTC supports method=guidance or prefix')
    if type(config.get('use_vjp', False)) is not bool:
        raise ValueError('RTC use_vjp must be boolean')
    if config.get('schedule', 'exp') not in ('exp', 'linear', 'ones', 'zeros'):
        raise ValueError('Unsupported RTC weight schedule')
    weight = config.get('max_guidance_weight', 5.)
    if not np.isfinite(weight) or weight <= 0:
        raise ValueError('RTC max_guidance_weight must be positive and finite')
    prefix = config.get('prefix_len', 9)
    if prefix is None:
        # Runners resolve automatic prefix lengths from their previous timing.
        if action_chunk is not None:
            if execute_horizon >= action_chunk:
                raise ValueError('RTC needs an unexecuted prefix')
            decay = config.get('decay_end')
            if decay is not None and (type(decay) is not int or not 0 < decay <= action_chunk):
                raise ValueError('RTC decay_end must be a positive integer within the action chunk')
            return config
        raise ValueError('RTC prefix_len=None must be resolved by the runner')
    decay = config.get('decay_end', prefix * 2)
    if type(prefix) is not int or prefix <= 0 or type(decay) is not int or decay < prefix:
        raise ValueError('RTC requires positive integer prefix_len and decay_end >= prefix_len')
    if action_chunk is not None and (prefix > action_chunk - execute_horizon or decay > action_chunk):
        raise ValueError('RTC prefix_len must fit the unexecuted chunk; decay_end <= action_chunk')
    config.update(prefix_len=prefix, decay_end=decay)
    return config


def resolve_rtc_config(config, *, elapsed_s, dt, available, action_chunk):
    """Resolve optional automatic prefix length before crossing the model wire."""
    config = dict(config)
    if config.get('prefix_len') is None:
        config['prefix_len'] = min(available, max(1, int(np.ceil(elapsed_s / dt)) + 1))
        config.setdefault('decay_end', min(action_chunk, config['prefix_len'] * 2))
    return validate_rtc(config)


def inject_rtc(batch, request, *, state, denormalizer, model):
    config = validate_rtc(request['config'])
    if not config.get('enabled', False):
        raise ValueError('RTC request must enable guidance')
    # Accelerated implementations accepting only **kwargs may silently ignore RTC.
    if 'rtc_config' not in inspect.signature(model.predict_action).parameters:
        raise ValueError('This model implementation does not support RTC guidance; restart with --acceleration off')
    info = dict(enabled=True, method=config['method'], applied=False, prefix_len=0,
                execution=request.get('execution', 'synchronous_remaining_prefix'), config=config)
    absolute = request.get('absolute_actions')
    if absolute is None:
        return info  # First chunk establishes the history.
    absolute = np.asarray(absolute, dtype=np.float32)
    state = np.asarray(state, dtype=np.float32)
    if (absolute.ndim != 2 or absolute.shape[1] not in (7, 14) or not len(absolute)
            or not np.isfinite(absolute).all() or state.shape != (absolute.shape[1],)
            or not np.isfinite(state).all()):
        raise ValueError('RTC requires finite retained [T,7/14] targets and matching new state')
    if len(absolute) > model.n_action_steps:
        raise ValueError('RTC retained targets exceed the model action chunk')
    normalize = getattr(denormalizer, 'normalize_absolute', None)
    if normalize is None:
        raise ValueError('A10 RTC requires a denormalizer supporting normalize_absolute')
    normalized = normalize(absolute, state)
    prefix_len = min(config['prefix_len'], len(normalized))
    reference = batch['states']
    # Physical targets use 7/14 columns; the flow model commonly predicts 32.
    width = getattr(model, 'max_action_dim', normalized.shape[1])
    if width < normalized.shape[1]:
        raise ValueError('RTC model action width is smaller than the robot state')
    padded = np.pad(normalized, ((0, 0), (0, width - normalized.shape[1])))
    batch.update(prev_actions=torch.as_tensor(padded[None], device=reference.device,
                                              dtype=reference.dtype),
                 prefix_len=prefix_len, rtc_config=config)
    info.update(applied=True, prefix_len=prefix_len, retained_steps=len(normalized),
                normalized_prefix=normalized.tolist(), absolute_prefix=absolute.tolist())
    return info
