"""Native FluxVLA record/continue entrypoint."""

import argparse
from pathlib import Path
from .training import _project
from .session import run_session, simulate_recording


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--task', required=True)
    parser.add_argument('--episodes', type=int, default=1)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--frames', type=int, default=6)
    args = parser.parse_args()
    if args.episodes <= 0:
        raise ValueError('episodes must be positive')
    cfg = _project()
    if args.simulate:
        simulate_recording(cfg, args.root, args.task, args.episodes,
                           args.frames, args.resume)
    else:
        run_session(cfg, recording=True, root=args.root, task=args.task,
                    episodes=args.episodes, resume=args.resume)
    if (Path(args.root) / 'meta/info.json').is_file():
        # This existing FluxVLA tool also independently checks video PTS/pixels.
        from tools.validate_a10_dataset import validate
        validate(Path(args.root).resolve(),
                 Path(cfg['paths']['outputs']) / 'validation' / Path(args.root).name,
                 cfg['model']['action_horizon'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
