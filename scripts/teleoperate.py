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
    parser.add_argument('--record-config', type=Path, help='双臂数据采集配置；启用手柄录制/HUD和备用终端命令')
    parser.add_argument('--record-options', nargs='+', action=DictAction, help='覆盖录制配置，如 record.resume=True capture.task=...')
    parser.add_argument('--check-record-buttons', action='store_true', help='配合--record-config，只检查VR按钮，不连接机器人/夹爪/相机')
    parser.add_argument('--record-controller', action='store_true',
                        help='同一5999连接开启控制器高频记录；需实机模式或 --observe')
    parser.add_argument('--measure-vr-latency', action='store_true', help='诊断VR发包/收包间隔、WebSocket往返与输入读取延迟，单独保存日志')
    parser.add_argument('--preview', action='store_true', help='只检查配置，不启动服务或连接设备')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--observe', action='store_true',
                        help='只读观察：读取机器人反馈并计算 reference 目标，不启动跟随或发送目标')
    mode.add_argument('--verify-calibration', action='store_true',
                      help='只读标定验证网页：显示实际与期望姿态，不运行 reference 或发送运动指令')
    mode.add_argument('--inspect-controller', action='store_true',
                        help='仅从监视端口读取 get 状态，不启动 VR 或任何运动')
    mode.add_argument('--calibrate-pose', action='store_true',
                      help='只读标定固定操作坐标系：机械臂静止，摆好手柄后倒计时采样并保存')
    parser.add_argument('--pose-calibration', type=Path,
                        help='标定 JSON 路径；标定时为输出，正常运行时为加载文件')
    parser.add_argument('--calibration-countdown', type=int, default=5, help='标定倒计时秒数，默认 5')
    parser.add_argument('--calibration-duration', type=float, default=1., help='标定采样时长 1～5 秒，默认 1')
    parser.add_argument('--samples', type=int, default=10, help='只读控制器检查的采样次数，默认 10')
    parser.add_argument('--report', type=Path, help='只读控制器检查报告 JSON 路径')
    args = parser.parse_args(argv)
    if args.record_options and not args.record_config:
        parser.error('--record-options requires --record-config')
    if args.check_record_buttons and not args.record_config:
        parser.error('--check-record-buttons requires --record-config')
    if args.record_config and any((args.observe, args.verify_calibration, args.inspect_controller, args.calibrate_pose)):
        parser.error('数据录制不能与观察/标定/控制器检查模式混用；检查按钮使用--check-record-buttons')
    if args.check_record_buttons and args.record_controller:
        parser.error('按键检查不能开启控制器记录')
    if args.measure_vr_latency and any((args.inspect_controller, args.calibrate_pose, args.verify_calibration)):
        parser.error('--measure-vr-latency 用于遥操作、观察或录制按键检查模式')
    cfg = Config.fromfile(args.config)
    if args.cfg_options:
        cfg.merge_from_dict(args.cfg_options)
    if cfg.teleop.input.get('arm') == 'both' and (
            args.calibrate_pose or args.verify_calibration or args.pose_calibration):
        parser.error('双臂模式本轮仅支持锚点相对控制，未开放双臂标定或标定验证界面')
    if args.record_controller:
        if args.calibrate_pose or args.verify_calibration or args.inspect_controller:
            parser.error('--record-controller 仅用于遥操作或 --observe')
        cfg.teleop.record_controller = True
    if args.pose_calibration:
        cfg.teleop.pose_calibration_path = str(args.pose_calibration.expanduser().resolve())
        if not args.calibrate_pose:
            if not args.pose_calibration.is_file():
                parser.error('--pose-calibration 指定的文件不存在')
            cfg.teleop.pose_calibration.vr_to_robot_quat_xyzw = None
    if args.calibrate_pose:
        from fluxvla.engines.operators.vr.runtime import calibrate_pose
        output = cfg.teleop.get('pose_calibration_path')
        if not output:
            parser.error('请配置 pose_calibration_path 或使用 --pose-calibration 指定输出')
        try:
            calibrate_pose(cfg.teleop, output_path=output, countdown=args.calibration_countdown,
                           duration_s=args.calibration_duration, preview=args.preview)
        except KeyboardInterrupt:
            print('\n标定已取消，原标定文件未改动。')
            return 130
        except Exception as error:
            print(f'标定未完成：{error}；原标定文件未改动。', file=sys.stderr)
            return 1
        return 0
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
    if args.verify_calibration:
        from fluxvla.engines.operators.vr.calibration_view import build_calibration_view
        view = build_calibration_view(cfg.teleop)
        if args.preview:
            print('只读标定验证配置有效；未启动服务或连接设备。')
            return 0
        try:
            view.run(max_steps=cfg.teleop.get('max_steps'))
        except KeyboardInterrupt:
            print('\n只读验证已关闭；未发送运动指令，标定文件未改动。')
        return 0
    if args.observe:
        cfg.teleop.update(vr_only=False, observe_only=True, control_mode='reference')
    if (args.observe or args.check_record_buttons or cfg.teleop.get('vr_only', True)) and not args.record_controller:
        # Explicit observation/input-only modes do not inherit live recording.
        cfg.teleop.record_controller = False
    if args.check_record_buttons:
        cfg.teleop.update(vr_only=True, observe_only=False)
        cfg.teleop.zx_grippers.enabled = False
    runtime = build_runtime(cfg.teleop)
    runtime.vr.measure_latency = args.measure_vr_latency
    runtime.vr.latency_context = dict(recording=bool(args.record_config),
        button_check=args.check_record_buttons, execute=runtime.execute,
        observe_only=runtime.observe_only, fps=cfg.teleop.get('fps', 30),
        record_controller=runtime.record_controller)
    if args.record_config:
        if cfg.teleop.get('fps', 30) != 30:
            parser.error('双臂数据录制要求teleop.fps=30')
        from fluxvla.engines.operators.vr.recording import RecordingSession
        record_config = Config.fromfile(str(args.record_config))
        if args.record_options: record_config.merge_from_dict(args.record_options)
        record_cfg = record_config.to_dict()
        runtime.recording = RecordingSession(runtime, record_cfg, button_check=args.check_record_buttons)
        runtime.vr.recording_enabled = True
        print(f'数据入口：backend={runtime.recording.backend}，workflow={runtime.recording.workflow}，'
              f'sampling={runtime.recording.sampling_mode}，'
              f'schema={record_cfg["record"]["schema"]}，task={record_cfg["capture"]["task"]}，'
              f'目录={record_cfg["record"]["root"]}，右手功能键={runtime.recording.buttons.button}；'
              f'只读按键检查={args.check_record_buttons}')
    if args.preview:
        print(f'配置有效：vr_only={cfg.teleop.vr_only}，observe_only={runtime.observe_only}，'
              f'control_mode={runtime.control_mode}，pose_mapping={runtime.pose_mapping}，'
              f'record_controller={runtime.record_controller}，measure_vr_latency={args.measure_vr_latency}，'
              f'zx_grippers={getattr(runtime, "zx_grippers_enabled", False)}，'
              '未启动服务或连接设备。')
        return 0
    runtime.run(fps=cfg.teleop.get('fps', 30),
                max_steps=cfg.teleop.get('max_steps'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
