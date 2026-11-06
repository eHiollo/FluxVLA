"""检查 A10 单臂/双臂数据格式、索引、任务、统计和视频对齐。"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fluxvla.datasets.a10_recording_io import validate_dataset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--skip-videos', action='store_true', help='只检查视频路径/元数据，不解码核对帧数')
    args = parser.parse_args()
    try:
        report = validate_dataset(args.dataset, check_videos=not args.skip_videos)
    except Exception as exc:
        print(f'检查失败：{exc}', file=sys.stderr)
        return 1
    print('检查通过：' + json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
