# Copyright 2026 Limx Dynamics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys
from pathlib import Path

# Launch this checkout even if pip's editable install points to another worktree.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mmengine import Config, DictAction

from fluxvla.engines import build_runner_from_cfg, initialize_overwatch
from fluxvla.engines.utils.torch_utils import \
    configure_inference_attention_defaults

overwatch = initialize_overwatch(__name__)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run inference with the given configuration.')
    parser.add_argument(
        '--acceleration', choices=['on', 'off'], default=None,
        help='Select Triton/CUDA Graph (on) or plain PyTorch (off); omitted uses config default')
    parser.add_argument(
        '--config',
        type=str,
        required=True,
        help='Path to the configuration file.',
    )
    parser.add_argument("--model", choices=["fluxvla", "openpi", "longwam"], default=None,
                        help="Select the model backend in the common dual A10 config")
    parser.add_argument('--rtc', nargs='?', const='open', choices=('open', 'close'),
                        help='Client-side asynchronous fusion, enabled by default for A10 OpenPI')
    parser.add_argument(
        '--ckpt-path',
        type=str,
        default=None,
        help='Path to the checkpoint file.')
    parser.add_argument(
        '--cfg-options',
        nargs='+',
        action=DictAction,
        help=  # noqa: E251
        'override some settings in the used config, the key-value pair in xxx=yyy format'  # noqa: E501
    )
    args, unknown = parser.parse_known_args()
    return args, unknown


def load_config(args):
    cfg = Config.fromfile(args.config)
    if 'rtc_overrides' not in cfg and args.rtc is not None:
        raise ValueError('--rtc requires the unified A10 inference config')
    overrides = args.cfg_options or {}
    backend = args.model or overrides.get('model_backend', cfg.get('model_backend'))
    default_rtc = ('rtc_overrides' in cfg and backend == 'openpi'
                   and not cfg.inference.get('rtc_config', {}).get('enabled', False))
    if args.rtc == 'open' or (args.rtc is None and default_rtc):
        cfg.merge_from_dict(cfg.rtc_overrides)
    if args.cfg_options is not None:
        cfg.merge_from_dict(args.cfg_options)
    from fluxvla.engines.utils.a10_inference_backends import select_a10_backend
    select_a10_backend(cfg, args.model, overrides=args.cfg_options)
    from fluxvla.engines.utils.inference_config import select_inference_acceleration
    select_inference_acceleration(cfg, args.acceleration)
    cfg.inference.cfg = cfg
    cfg.inference.ckpt_path = args.ckpt_path
    return cfg


if __name__ == '__main__':
    configure_inference_attention_defaults()
    args, _ = parse_args()
    cfg = load_config(args)
    inference_runner = build_runner_from_cfg(cfg.inference)
    inference_runner.run_setup()
    inference_runner.run()
