"""Start native VR teleoperation from a FluxVLA Python configuration."""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mmengine import Config, DictAction
from fluxvla.engines.operators.vr.runtime import build_runtime


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--cfg-options', nargs='+', action=DictAction)
    parser.add_argument('--preview', action='store_true', help='只检查配置，不启动服务或连接设备')
    args = parser.parse_args(argv)
    cfg = Config.fromfile(args.config)
    if args.cfg_options:
        cfg.merge_from_dict(args.cfg_options)
    runtime = build_runtime(cfg.teleop)
    if args.preview:
        print(f'配置有效：vr_only={cfg.teleop.vr_only}，未启动服务或连接设备。')
        return 0
    runtime.run(fps=cfg.teleop.get('fps', 30),
                max_steps=cfg.teleop.get('max_steps'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
