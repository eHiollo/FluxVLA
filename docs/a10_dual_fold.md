# A10 双臂叠衣服：十步训练适配

使用普通 PI0.5 LoRA，未启用 RTC。此配置用于训练管线和接口适配，10 步结果不能用于评价叠衣服能力。

## 数据与接口

- 原数据复用 `/home/allen/Allen/openpi/dataset/a10_dua_1113_v21`，不复制或改写数据。
- 任务提示词固定为 `Fold the garment neatly using both arms.`，不读取原始水瓶任务标签。
- 14 维顺序：左臂 6 关节 + 左夹爪，右臂 6 关节 + 右夹爪。关节单位为弧度，夹爪 0 张开、1 闭合。
- 相机按 base、left wrist、right wrist 排列，三路全部有效；标准模型输入拼为 9×224×224。
- 以未来 `observation.state` 为目标，30 帧预测窗口从当前帧开始；尾部重复最后状态并监督。
- 两臂关节分别减去当前状态，索引 6、13 的夹爪保持绝对值；不使用 ALOHA 的符号和夹爪映射。
- 复用此数据独立计算的 OpenPI `norm_stats.json`，按 q01/q99 分位数归一化；提示词状态仍为 14 维，之后补齐到 32 维。
- 训练 loss 与原 PI0.5 配置一致，覆盖 32 维内部动作；物理动作维度为 14。

## 启动

```bash
conda activate fluxvla
cd /home/allen/Allen/FluxVLA
HF_HOME=/data/allen/fluxvla/cache/huggingface \
  HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
  TOKENIZERS_PARALLELISM=false WANDB_MODE=disabled \
  torchrun --standalone --nnodes=1 --nproc_per_node=1 scripts/train.py \
  --config configs/pi05/pi05_paligemma_a10_dual_fold_10steps.py \
  --work-dir /data/allen/fluxvla/runs/pi05_a10_dual_fold_10steps_new
```

默认 batch size 1、梯度累积 1、10 个优化步骤，bf16 + gradient checkpointing。
基础权重为 `/data/allen/fluxvla/pretrained/pi05_base/model.safetensors`，需提前准备。
数据、统计与权重路径可分别通过 `A10_DUAL_DATA_ROOT`、`A10_DUAL_OPENPI_STATS`、`FLUXVLA_PI05_BASE` 覆盖。
本次已完成实验为 `/data/allen/fluxvla/runs/pi05_a10_dual_fold_10steps`；再次训练请使用新目录。

训练产物包括 `checkpoints/latest-checkpoint.pt`（续训）、
`checkpoints/latest-checkpoint.safetensors`（完整模型权重）、LoRA adapter、tokenizer、
resolved config 和 `dataset_statistics.json`。仅使用 adapter 时仍需要对应基础权重。
该权重是 FluxVLA/PyTorch 格式，与 OpenPI/JAX checkpoint 格式不同。
当前配置禁用继承的 UR3 硬件推理 runner；双臂机器人部署接口需单独适配。

## 检查

```bash
/home/allen/miniconda3/envs/fluxvla/bin/python -m pytest test/test_configs/test_a10_dual_fold.py -q
```

测试包含双臂相机与维度校验、夹爪保持绝对值、关节增量往返，以及原单臂行为回归。
