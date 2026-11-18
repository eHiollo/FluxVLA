"""把干净的 A10 LeRobot v3 数据导出为独立的 v2.1 数据集，不修改源目录。"""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class ConsoleProgress:
    def __init__(self):
        self.started = time.monotonic()
        self.last = 0.
        self.phase = None

    def __call__(self, event):
        phase = event['phase']
        now = time.monotonic()
        if phase == self.phase and now - self.last < 3 and phase not in ('video', 'video_done', 'complete'):
            return
        self.phase, self.last = phase, now
        names = dict(validate_source='检查源数据及元数据', data='转换分集数据', video='处理视频',
                     video_done='视频完成', transcode='转码视频', validate_output='逐帧校验输出', complete='完成')
        details = []
        if 'completed' in event:
            details.append(f"{event['completed']}/{event['total']}")
        if 'episode' in event:
            details.append(f"episode={event['episode']}")
        if 'camera' in event:
            details.append(event['camera'])
        if 'mode' in event:
            details.append('原样复制' if event['mode'] == 'copy' else 'H.264 转码')
        if 'path' in event:
            details.append('/'.join(Path(event['path']).parts[-2:]))
        if 'frames' in event:
            details.append(f"{event['frames']} 帧")
        print(f"[{now-self.started:.1f}s] {names.get(phase, phase)} {' '.join(details)}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='必须是尚不存在的独立新目录')
    parser.add_argument('--video-mode', choices=('auto', 'h264'), default='auto',
                        help='auto原样复制独立成集的视频，需切片时转H.264；h264强制转码全部视频')
    parser.add_argument('--resume', action='store_true', help='从同一输出目录的检查点继续，源数据和参数须不变')
    args = parser.parse_args(argv)
    print('正在加载转换依赖；输出发布前不会修改源目录或覆盖已有数据。', flush=True)
    try:
        from fluxvla.datasets.a10_recording_io import export_openpi_dataset
        report = export_openpi_dataset(args.source, args.output, video_mode=args.video_mode,
                                       resume=args.resume, progress=ConsoleProgress())
    except KeyboardInterrupt:
        print('\n转换已中断，源数据不变；如已开始写入，可用相同命令加 --resume 继续。', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'导出失败：{exc}', file=sys.stderr)
        return 1
    print('导出并逐帧检查通过：' + json.dumps(report, ensure_ascii=False))
    print(f'输出目录：{args.output.resolve()}；训练前请在 OpenPI 重新计算归一化统计。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
