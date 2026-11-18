# A10 LeRobot v3 → v2.1 转换

转换工具仅接受完整、独立的 A10 v3.0 数据集。它保留数值数据、任务、集索引及动作语义，
并按 OpenPI 约定映射相机：`base_0_rgb → top`、`left_wrist_0_rgb → left`、`right_wrist_0_rgb → right`。

```bash
python3 scripts/export_a10_openpi_dataset.py \
  --source datasets/your_v3_recording \
  --output datasets/your_v21_export
```

输出目录必须不存在；禁止把输出放在源目录内，也不覆盖已有输出。
`meta/info.json` 已标记为 v2.1 或混放 `file-*.parquet` / `episode_*.parquet` 的目录会尽早报错，
不会尝试猜测或修改源元数据。`datasets/cloth_folding_04` 当前属于混合目录，不能作为干净的 v3 输入；
已经校验并上传的 `datasets/cloth_folding_04_v21` 无需重复导出。

## 视频和校验

- 默认 `--video-mode auto`：完整的一集一个视频可原样复制，保留 AV1/H.264 等原始编码与字节，不损失画质。
- v3 多集共享同一视频或需要时间切片时，该相机的输出统一转为 H.264。按原始时间戳切片，输出从零计时。
- `--video-mode h264`：显式要求所有视频转为 H.264，用于需要该编码的下游环境。

v2.1 规范不要求 H.264。选择 `auto` 后，读取端仍须支持输入视频的编码；例如 AV1 可使用支持该编码的 PyAV。
源数据先检查数值、路径、统计与元数据，输出再完整解码检查帧数、时间戳、尺寸和编码。
省去先完整解码源视频的重复检查；仍然不会把未通过最终检查的目录发布成正式输出。
终端持续显示阶段、集数、相机和解码帧数。

原图统计跟随相机名称映射保留，与 LeRobot 记录时按原图计算统计的约定一致。
H.264 转码有损，若下游要按解码图像重新统计，需另外计算。
OpenPI 训练前应按其数据变换重新计算归一化统计；转换脚本不把实测动作改成指令或关节增量。

## 中断与继续

```bash
# 与首次运行使用完全相同的 source/output/video-mode，再加 --resume
python3 scripts/export_a10_openpi_dataset.py \
  --source datasets/your_v3_recording \
  --output datasets/your_v21_export --resume
```

未完成内容保留在输出旁的隐藏目录 `.your_v21_export.export/`。
每个完成文件有 SHA256 检查点；继续时核对源文件快照及参数，重新生成受损输出，复用已完成输出。
源文件或转换参数变化时拒绝复用检查点，请选择新的输出目录。
同一输出只能有一个导出任务运行；检查全部通过后才把暂存目录改名为正式输出。

## 官方依据与验证

布局和 JSONL 元数据对照 [LeRobot v0.3.3 的 v2.1 读写工具](https://github.com/huggingface/lerobot/blob/v0.3.3/src/lerobot/datasets/utils.py)，
v3 分片、集索引和视频时间偏移对照 [v0.4.3 官方 v2.1 → v3 转换器](https://github.com/huggingface/lerobot/blob/v0.4.3/src/lerobot/datasets/v30/convert_dataset_v21_to_v30.py)。

回归测试覆盖单臂、双臂、AV1、共享视频/Parquet 分片、无损复制、显式转码、源数据不变、
中断恢复、文件损坏、源变化和并发限制。真实 v2.1 读取验证应在相容依赖环境中运行；
LeRobot v0.3.3 要求 `datasets>=2.19,<=3.6.0`，不要用 datasets 4 的共享缓存替代旧版读取验证。

本机 torchvision 0.28 已移除旧版读取器使用的 `VideoReader`。独立样本验证在临时进程中
加载 torchvision 官方 v0.22.1 的 PyAV `VideoReader` 类完成，不修改当前训练环境。
不要把旧读取器的依赖/API 错误当成转换文件损坏。
