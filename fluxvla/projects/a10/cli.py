"""Human-facing A10 workflow commands. Use --execute for active commands."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from .config import (DEFAULT_CONFIG, DEFAULT_SITE, PROJECT_ROOT,
                     fingerprint, load_project)


def _git_revision(path: Path) -> str:
    result = subprocess.run(['git', '-C', str(path), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else 'unavailable'


def _git_changes(path: Path) -> list[str]:
    result = subprocess.run(
        ['git', '-C', str(path), 'status', '--short', '--untracked-files=all'],
        capture_output=True, text=True, check=False)
    return result.stdout.splitlines() if result.returncode == 0 else []


def _manifest(cfg: dict, command: list[str], run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    roots = {'FluxVLA': PROJECT_ROOT}
    record = {
        'created_at': datetime.now().astimezone().isoformat(),
        'config_sha256': fingerprint(cfg),
        'config': cfg,
        'command': command,
        'source_commits': {key: _git_revision(path)
                           for key, path in roots.items()},
        'source_changes': {key: _git_changes(path)
                           for key, path in roots.items()},
    }
    (run_dir / 'run_manifest.json').write_text(
        json.dumps(record, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8')


def _run(command: list[str], *, cwd: Path, cfg: dict, name: str,
         execute: bool, env: dict | None = None) -> int:
    print(f'[{name}] {shlex.join(command)}', flush=True)
    if not execute:
        print('预览完成；加 --execute 才会启动该模块。', flush=True)
        return 0
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = Path(cfg['paths']['outputs']) / 'runs' / f'{stamp}_{name}'
    _manifest(cfg, command, run_dir)
    print(f'运行记录：{run_dir / "run_manifest.json"}', flush=True)
    child_env = dict(os.environ if env is None else env)
    child_env['FLUXVLA_A10_RUN_DIR'] = str(run_dir)
    if name in {'teleop', 'record', 'infer', 'check-hardware'}:
        lock_path = Path(cfg['paths']['outputs']) / 'robot.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        with lock_path.open('w', encoding='utf-8') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise ValueError('Another A10 teleop/record/infer/check-hardware command is active') from exc
            return subprocess.run(command, cwd=cwd, env=child_env,
                                  check=False).returncode
    return subprocess.run(command, cwd=cwd, env=child_env,
                          check=False).returncode


def _doctor(cfg: dict) -> int:
    paths = cfg['paths']
    checks = {
        'FluxVLA Python': Path(paths['fluxvla_python']).is_file(),
        '平台 VR 网页': (PROJECT_ROOT / 'fluxvla/projects/a10/vr/web-ui/index.html').is_file(),
        '数据集': (Path(cfg['dataset']['root']) / 'meta/info.json').is_file(),
        'PI0.5 基础权重': bool(cfg['model']['base_checkpoint']) and Path(
            cfg['model']['base_checkpoint']).is_file(),
        'PI0.5 tokenizer': bool(cfg['model']['tokenizer_path']) and (Path(
            cfg['model']['tokenizer_path']) / 'tokenizer.json').is_file(),
    }
    import importlib.util
    for module in ('av', 'pyarrow', 'cv2', 'scipy', 'websockets', 'zmq'):
        checks[f'平台依赖 {module}'] = importlib.util.find_spec(module) is not None
    for name, ok in checks.items():
        print(f'{"OK" if ok else "待配置"}  {name}')
    for name, item in cfg['cameras'].items():
        device = item['device']
        device_path = (Path(f'/dev/video{device}') if isinstance(device, int)
                       else Path(str(device)))
        available = device_path.exists()
        print(f'{"OK" if available else "待连接"}  相机 {name}: '
              f'{device_path}，{item["width"]}x{item["height"]}，'
              f'旋转180={item["rotate_180"]}；启动前确认实体相机身份')
    print(f'机器人：{cfg["robot"]["host"]}:{cfg["robot"]["port"]} '
          '(doctor 不连接机器人)')
    network = cfg['vr_server']
    print(f'VR 页面：https://{network["host_ip"]}:{network["https_port"]} '
          f'（WebSocket 端口 {network["websocket_port"]}，doctor 不启动服务）')
    if network['host_ip'] in {'127.0.0.1', 'localhost', '::1'}:
        print('待配置  VR 当前仅监听本机，头显无法通过局域网访问；'
              '请在 site.local.yaml 中将 vr_server.host_ip 设为本机局域网地址。')
    print(f'配置 SHA256：{fingerprint(cfg)}')
    print('基础模型：' + (str(cfg['model']['base_checkpoint'])
                      if cfg['model']['base_checkpoint'] else '尚未指定'))
    print('训练权重：' + (str(cfg['model']['trained_checkpoint'])
                      if cfg['model']['trained_checkpoint'] else '尚未指定'))
    return 0 if all(checks.values()) else 1


def _record_command(cfg: dict, args: argparse.Namespace) -> list[str]:
    ds = cfg['dataset']
    command = [cfg['paths']['fluxvla_python'], '-m', 'fluxvla.projects.a10.record',
               '--root', str(Path(args.root or ds['root']).resolve()),
               '--task', args.task or ds['task'],
               '--episodes', str(args.episodes or ds['num_episodes'])]
    if args.resume:
        command.append('--resume')
    if args.simulate:
        command += ['--simulate', '--frames', str(args.frames)]
    return command


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='A10 一站式流程入口')
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--site', default=str(DEFAULT_SITE))
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('doctor', help='只读检查环境和配置')
    hardware = sub.add_parser('check-hardware', help='采集相机并查询机器人状态，保存现场检查报告')
    selection = hardware.add_mutually_exclusive_group()
    selection.add_argument('--cameras-only', action='store_true')
    selection.add_argument('--robot-only', action='store_true')
    hardware.add_argument('--frames', type=int, default=60)
    hardware.add_argument('--timeout', type=float, default=60,
                          help='每项检查的最长秒数，包含模块启动时间')
    hardware.add_argument('--execute', action='store_true')
    teleop = sub.add_parser('teleop', help='启动 XLeVR 遥操作')
    teleop.add_argument('--vr-only', action='store_true')
    teleop.add_argument('--execute', action='store_true')
    record = sub.add_parser('record', help='用平台原生录制器采集 A10 数据')
    record.add_argument('--root')
    record.add_argument('--task')
    record.add_argument('--episodes', type=int)
    record.add_argument('--resume', action='store_true')
    record.add_argument('--simulate', action='store_true', help='只使用模拟输入验证录制')
    record.add_argument('--frames', type=int, default=6)
    record.add_argument('--execute', action='store_true')
    validate = sub.add_parser('validate', help='检查 LeRobot v3 数据')
    validate.add_argument('--root')
    validate.add_argument('--execute', action='store_true')
    train = sub.add_parser('train', help='FluxVLA 原生训练')
    train.add_argument('--work-dir')
    train.add_argument('--resume-from')
    train.add_argument('--steps', type=int,
                       help='只跑指定步数（例如 --steps 1 做短训检查）')
    train.add_argument('--execute', action='store_true')
    serve = sub.add_parser('serve', help='启动 FluxVLA A10 模型服务')
    serve.add_argument('--checkpoint')
    serve.add_argument('--execute', action='store_true')
    replay = sub.add_parser('replay', help='离线回放观测到模型服务')
    replay.add_argument('--samples', type=int, default=3)
    replay.add_argument('--execute', action='store_true')
    infer = sub.add_parser('infer', help='FluxVLA 模型控制 A10')
    infer.add_argument('--cycles', type=int)
    infer.add_argument('--execute', action='store_true')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        cfg = load_project(args.config, args.site)
        if args.command == 'doctor':
            return _doctor(cfg)
        flux_python = cfg['paths']['fluxvla_python']
        env = os.environ.copy()
        env['FLUXVLA_A10_CONFIG'] = str(Path(args.config).resolve())
        env['FLUXVLA_A10_SITE'] = str(Path(args.site).resolve())
        if args.command == 'check-hardware':
            if not 2 <= args.frames <= 600 or not 1 <= args.timeout <= 300:
                raise ValueError('frames must be within [2, 600], timeout within [1, 300] seconds')
            command = [flux_python, '-m', 'fluxvla.projects.a10.hardware_check',
                       '--frames', str(args.frames), '--timeout', str(args.timeout)]
            if args.cameras_only:
                command.append('--cameras-only')
            if args.robot_only:
                command.append('--robot-only')
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='check-hardware', execute=args.execute, env=env)
        if args.command == 'teleop':
            if args.execute and not args.vr_only and not cfg['robot']['allow_motion']:
                raise ValueError('Set robot.allow_motion=true in site.local.yaml')
            command = [flux_python, '-m', 'fluxvla.projects.a10.teleop']
            if args.vr_only:
                command.append('--vr-only')
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='teleop', execute=args.execute, env=env)
        if args.command == 'record':
            if args.episodes is not None and args.episodes <= 0:
                raise ValueError('--episodes must be positive')
            root = Path(args.root or cfg['dataset']['root'])
            if args.execute and (root / 'meta/info.json').exists() and not args.resume:
                raise ValueError(
                    f'{root} already contains a dataset; use --resume or '
                    'choose a new --root')
            if args.execute and not args.simulate and not cfg['robot']['allow_motion']:
                raise ValueError('Set robot.allow_motion=true in site.local.yaml')
            return _run(_record_command(cfg, args),
                        cwd=PROJECT_ROOT, cfg=cfg,
                        name='record', execute=args.execute, env=env)
        if args.command == 'validate':
            root = args.root or cfg['dataset']['root']
            command = [flux_python, str(PROJECT_ROOT /
                                     'tools/validate_a10_dataset.py'),
                       '--dataset-root', root,
                       '--horizon', str(cfg['model']['action_horizon']),
                       '--output-dir', str(Path(cfg['paths']['outputs']) /
                                           'validation')]
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='validate', execute=args.execute, env=env)
        if args.command == 'train':
            if args.execute and (Path(cfg['dataset']['root']) / 'SIMULATED_DATA.json').exists():
                raise ValueError('Simulation datasets are for integration checks; select real demonstrations')
            if args.steps is not None and args.steps <= 0:
                raise ValueError('--steps must be positive')
            if args.execute and (
                    not cfg['model']['base_checkpoint'] or not Path(
                        cfg['model']['base_checkpoint']).is_file()):
                raise FileNotFoundError('Install model.base_checkpoint first')
            if args.execute and not Path(cfg['model']['tokenizer_path']).is_dir():
                raise FileNotFoundError('Install model.tokenizer_path first')
            work_dir = args.work_dir or str(Path(cfg['paths']['outputs']) /
                                            'training/pi05_a10')
            command = [flux_python, '-m', 'torch.distributed.run',
                       '--standalone', '--nnodes=1', '--nproc-per-node=1',
                       str(PROJECT_ROOT / 'scripts/train.py'),
                       '--config', str(PROJECT_ROOT /
                                       'configs/a10/pi05_a10_finetune.py'),
                       '--work-dir', work_dir]
            if args.resume_from:
                command += ['--resume-from', args.resume_from]
            if args.steps is not None:
                command += ['--cfg-options',
                            f'runner.max_steps={args.steps}',
                            'runner.save_iter_interval=1']
            env.setdefault('WANDB_MODE', 'disabled')
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='train', execute=args.execute, env=env)
        if args.command == 'serve':
            checkpoint = args.checkpoint or cfg['model']['trained_checkpoint']
            if args.execute and (not checkpoint or not Path(checkpoint).is_file()):
                raise FileNotFoundError('Set an existing FluxVLA checkpoint')
            command = [flux_python, '-m', 'fluxvla.projects.a10.serve',
                       '--checkpoint', str(checkpoint or '<checkpoint>')]
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='serve', execute=args.execute, env=env)
        if args.command == 'replay':
            if args.samples <= 0:
                raise ValueError('--samples must be positive')
            command = [flux_python, '-m', 'fluxvla.projects.a10.replay',
                       '--samples', str(args.samples)]
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='replay', execute=args.execute, env=env)
        if args.command == 'infer':
            if args.execute and not cfg['robot']['allow_motion']:
                raise ValueError('Set robot.allow_motion=true in site.local.yaml')
            cycles = (cfg['inference']['max_cycles'] if args.cycles is None
                      else args.cycles)
            if cycles < 0:
                raise ValueError('--cycles must be nonnegative')
            command = [flux_python, '-m', 'fluxvla.projects.a10.bridge',
                       '--cycles', str(cycles)]
            return _run(command, cwd=PROJECT_ROOT, cfg=cfg,
                        name='infer', execute=args.execute, env=env)
        raise AssertionError(args.command)
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f'A10 配置/启动错误：{exc}', file=sys.stderr)
        return 2
