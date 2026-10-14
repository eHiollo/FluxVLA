# A10 SARM 训练与验证

任务阶段沿用 `datasets/a10_stages.json` 的四阶段定义：抓起水瓶、放入旁边盒子、
将盒子盖上、回到初始位置。归位属于任务完成条件。

## 数据

训练数据为 `datasets/a10_sarm_train`：23 集，27,017 帧。
验证数据为 `datasets/a10_sarm_val`：6 集，5,501 帧。
验证集对应原始 episode **7、11、16、20、23、24**。
原始失败集 **2、12、25** 未进入任一 split。

使用种子 42 按 episode 划分。样本数量少，同一采集批次的随机留出集只能检查
当前数据分布下的效果；跨场景或物体泛化需要另采录像。

导出数据中的 episode/index 已重新编号；原始映射在
`datasets/a10_sarm_split.json` 和各 split 的 `meta/source_episode_mapping.json`。
Parquet 和标签在导出目录，视频链接到原始数据，原始数据和标注 JSON 保持原样。
两组共享仅由训练集计算的状态归一化统计量及阶段时间占比。

如需重新导出到其他前缀：

```bash
python tools/sarm_annotate/prepare_a10_sarm.py --output-prefix datasets/a10_sarm_new
```

已有导出目录不会被覆盖，标签改变后请使用新前缀并同步训练配置。

## 训练

在 FluxVLA 根目录激活 `fluxvla` 环境：

```bash
conda activate fluxvla
bash scripts/train_sarm_a10.sh
```

配置 `configs/sarm/sarm_a10.py` 使用两路相机、7 维状态归一化后补齐到 32 维，
30 fps 下每 30 帧抽取历史观测，`n_obs_steps=8`，并使用 rewind augmentation。
CLIP 保持冻结；训练 SARM transformer 和阶段/进度头。
默认 batch 16、学习率 5e-5、bf16、5,000 步，每 100 步保存一次，保留最近 5 个 checkpoint。
种子为 42；GPU 算子未强制确定性，因此不保证逐位复现。

输出使用项目内 `sarm_runs/a10`，训练 checkpoint、resolved config、JSONL 指标都在该目录。
若由后台启动，日志为 `sarm_runs/a10/train.log`，启动进程编号为 `train.pid`。

```bash
tail -f sarm_runs/a10/train.log
```

出现 `step-005000-...pt` 并且日志无异常、训练进程退出，代表完成 5,000 步。
中断后恢复训练：

```bash
bash scripts/train_sarm_a10.sh \
  --resume-from ./sarm_runs/a10/checkpoints/latest-checkpoint.pt
```

## 验证

完成训练后运行：

```bash
bash scripts/validate_sarm_a10.sh
```

默认只读取六集验证数据，输出逐帧预测到
`sarm_runs/a10/validation/progress.jsonl`，六集进度/阶段概率/参考标签/抽帧图到
`sarm_runs/a10/validation/figures`。这些图只显示第一路外部相机缩略图，
预测输入仍包含两路相机。

验证会显示 batch 进度和预计剩余时间，默认使用 4 个进程解码视频。
首批准备数据时需要等待；共享内存不足时可加 `--num-workers 0`。

先试几个 batch：

```bash
bash scripts/validate_sarm_a10.sh --max-batches 4 --num-visualizations 1
```

使用较早 checkpoint 比较：

```bash
bash scripts/validate_sarm_a10.sh --ckpt-path /path/to/checkpoint.pt
```

重点查看阶段切换是否对应人工边界、完成进度是否与操作一致。
训练 loss 下降本身不能证明效果；参考进度是从阶段标签构造的监督信号。
本次先训练成功演示基线，失败集不在常规验证脚本中。
训练过程不使用留出集选择 checkpoint；默认最终 checkpoint 供用户独立验证。

## 视频同步查看

使用修正后的中心帧导出，使预测、参考和画面从第 0 帧对应：

```bash
bash scripts/validate_sarm_a10_center.sh
bash scripts/view_sarm_a10.sh
```

中心帧结果另存在 `sarm_runs/a10/validation_center`，原验证结果保留。
浏览器打开 <http://127.0.0.1:8765>，可同步播放视频与 progress 曲线、点击曲线跳转。
此模式只读，使用原始集编号显示六集验证视频，使用同一帧号显示预测值与人工参考。
