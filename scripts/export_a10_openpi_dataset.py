"""把 A10 LeRobot v3 数据导出为 v2.1；按单臂/双臂格式映射相机名称。"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fluxvla.datasets.a10_recording_io import export_openpi_dataset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='必须是尚不存在的新目录，原数据不改动')
    args = parser.parse_args()
    try:
        report = export_openpi_dataset(args.source, args.output)
    except Exception as exc:
        print(f'导出失败：{exc}', file=sys.stderr)
        return 1
    print('导出并检查通过：' + json.dumps(report, ensure_ascii=False))
    print(f'输出目录：{args.output.resolve()}；训练前请在 OpenPI 重新计算归一化统计。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
