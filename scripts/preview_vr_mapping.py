"""Visualize raw VR and old/new virtual mapping targets without robot IO."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main(argv=None):
    from mmengine import Config, DictAction
    from fluxvla.engines.operators.vr.sandbox import build_sandbox
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'configs/a10/a10_dual_vr.py'))
    parser.add_argument('--cfg-options', nargs='+', action=DictAction)
    parser.add_argument('--preview', action='store_true', help='只验证配置，不启动输入服务')
    args = parser.parse_args(argv)
    cfg = Config.fromfile(args.config)
    if args.cfg_options:
        cfg.merge_from_dict(args.cfg_options)
    sandbox = build_sandbox(cfg.teleop)
    if args.preview:
        print('VR测试台配置有效：90 Hz，双手，新旧模式对照；未启动服务或构造机器人/夹爪/相机。')
        return 0
    sandbox.run()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
