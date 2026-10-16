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
    parser.add_argument('--observe', action='store_true',
                        help='只读观察：读取机器人反馈并计算 reference 目标，不启动跟随或发送目标')
    parser.add_argument('--inspect-controller', action='store_true',
                        help='仅从监视端口读取 get 状态，不启动 VR 或任何运动')
    parser.add_argument('--samples', type=int, default=10, help='只读控制器检查的采样次数，默认 10')
    parser.add_argument('--report', type=Path, help='只读控制器检查报告 JSON 路径')
    args = parser.parse_args(argv)
    cfg = Config.fromfile(args.config)
    if args.cfg_options:
        cfg.merge_from_dict(args.cfg_options)
    if args.observe and args.inspect_controller:
        parser.error('--observe 和 --inspect-controller 不能同时使用')
    if args.inspect_controller and not 1 <= args.samples <= 100:
        parser.error('--samples 必须在 1..100 范围内')
    if args.inspect_controller:
        if args.preview:
            print('只读控制器检查预览：仅计划发送 get，未建立连接。')
            return 0
        from fluxvla.engines.operators.vr.runtime import inspect_controller
        import json
        report = inspect_controller(cfg.teleop.operator, samples=args.samples)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if report['query_success'] else 1
    if args.observe:
        cfg.teleop.update(vr_only=False, observe_only=True, control_mode='reference')
    runtime = build_runtime(cfg.teleop)
    if args.preview:
        print(f'配置有效：vr_only={cfg.teleop.vr_only}，observe_only={runtime.observe_only}，'
              f'control_mode={runtime.control_mode}，未启动服务或连接设备。')
        return 0
    runtime.run(fps=cfg.teleop.get('fps', 30),
                max_steps=cfg.teleop.get('max_steps'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
