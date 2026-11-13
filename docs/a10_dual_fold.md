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
训练配置禁用继承的 UR3 硬件推理 runner；双臂部署使用下文的统一配置入口。

## 检查

```bash
/home/allen/miniconda3/envs/fluxvla/bin/python -m pytest test/test_configs/test_a10_dual_fold.py -q
```

测试包含双臂相机与维度校验、夹爪保持绝对值、关节增量往返，以及原单臂行为回归。

## 配置驱动的推理

模型服务与机器人客户端分别使用项目现有入口。OpenPI与FluxVLA的双臂客户端
共用 `A10DualInferenceRunner`，通过配置切换协议，不需要专属客户端脚本。

| 配置 | 后端 | 默认地址 |
| --- | --- | --- |
| `configs/a10/a10_dual_openpi_remote.py` | OpenPI WebSocket | `127.0.0.1:8001` |
| `configs/a10/a10_dual_fold_inference.py` | FluxVLA原生ZMQ | `127.0.0.1:5556` |

两份配置继承现有 `a10_dual_arm_operator.py` 和 `a10_devices.py`，不复制设备绑定。
模型差异在 `inference_model`、`inference.dataset`、`inference.denormalize_action`
声明；控制和通信在 `inference` 声明。公共组件支持单双臂状态维度、相机顺序、统计名、
mean/std或quantile归一化及关节增量掩码，旧单臂配置的默认行为保持兼容。

### 模型准备与启动

FluxVLA模型：[eHiollo/fluxvla-a10-dual-fold-10steps](https://modelscope.cn/models/eHiollo/fluxvla-a10-dual-fold-10steps)，
固定版本 `f2f32e6368fb6db69526098bb64a13c209eca495`。下载目录为
`checkpoints/FluxVLA-A10-Dual-Fold-10steps/`，保留原始包结构：
`checkpoints/model.safetensors`、`model_config.json`、`dataset_statistics.json`、
`inference_contract.json`、`tokenizer/` 和SHA256清单。目录由Git忽略，不能加入机器专属软链接。
可用环境变量 `A10_DUAL_CHECKPOINT` 覆盖模型根目录。

在FluxVLA仓库激活已有 `fluxvla` 环境后启动普通FP32服务：

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_dual_fold_inference.py \
  --from-config --dtype fp32 --host 127.0.0.1 --port 5556
```

LoRA已合并，严格加载完整权重，无需基础权重或adapter；首版不使用RTC及加速。
不需要继承UR3控制runner，也不依赖服务器上的数据集路径。
OpenPI服务仍在OpenPI仓库使用原有 `openpi-a10-dual.service`。

### 同一个客户端入口

```bash
# 默认只预览配置，不连接模型服务或设备
python scripts/inference.py --config configs/a10/a10_dual_fold_inference.py

# 模拟设备接FluxVLA真实模型服务
python scripts/inference.py --config configs/a10/a10_dual_fold_inference.py \
  --cfg-options inference.mode=simulate inference.cycles=2 inference.max_joint_step_rad=1.0

# 换配置即可使用OpenPI模型，控制链路相同
python scripts/inference.py --config configs/a10/a10_dual_openpi_remote.py \
  --cfg-options inference.mode=simulate inference.cycles=2 inference.max_joint_step_rad=1.0
```

模拟命令中的1.0rad只作用于该次运行，默认配置仍是0.2rad；不隐式绕过检查。
十步模型从人为模拟状态预测时可能触发默认首帧限制，不能把模拟限值直接用于实机。

真实设备只读设置 `inference.mode=read_only`；真实动作设置 `inference.mode=motion`。
**本轮只执行模拟验收，未连接真实机械臂、相机或夹爪。** 模式默认为preview，
修改配置或命令行模式会产生对应设备行为，取消了旧客户端额外的 `--execute` 开关。

跨机器时，将 `inference.remote_inference.server_host` 改为模型服务器的实际IP，
并让服务端监听可达网卡地址；controller地址仍来自设备配置。
OpenPI配置的 `remote_inference.client_root` 默认使用相邻OpenPI仓库的官方编码模块，
可设置为其他客户端src目录；已安装openpi-client时可设为None。

### 输入输出与执行约定

状态复用采集的 `pack_dual_state`：物理左臂ID1六个实际关节(rad)+夹爪raw/9000，
再物理右臂ID0及夹爪。base、left wrist、right wrist三相机保持RGB，按设备配置旋转一次。
FluxVLA模型使用随包tokenizer和 `a10_dual` 分位数统计，先生成14维状态提示词再补齐到32维。
三路图像经训练相同的PIL缩放/归一化后拼为9通道，三路mask全为True。
服务端仅对两臂六个关节恢复绝对位置，夹爪保持0张开、1闭合；客户端不重复反归一化或叠加状态。

同步循环：新观测→请求30帧动作→校验完整动作块→发送前10帧→确认停止→下一轮。
关节30Hz；真实知行夹爪各自20Hz线程消费最新目标，闭合比例限到0..1后映射0..9000，
截断数量写入日志。只使用实际反馈，机器人/相机接收年龄上限150ms，夹爪200ms，偏差150ms。

默认0.2rad同时检查推理后的最新状态到首帧，以及整个30帧动作中的相邻关节变化。
超限整段拒绝；落后发送计划超过100ms停止，不突发补发。网络超时120秒，失败不自动重连重放。
真实模式沿用共享服务所有权与故障检查，不自动回零、使能或修改运动速度/力矩。
停止不确认会报错；取消不会自动张开或掉使能夹爪，已触发目标可能仍保持。

运行记录位于 `work_dirs/a10/inference_logs/<时间_ID>_dual_client/`：
配置快照和events.jsonl记录观测时间、状态、请求耗时、原始/转换动作、发送帧数及停止确认。
不写入训练数据或模型目录。模型服务使用同目录下已有server调试日志。

公共控制与协议测试：

```bash
python -m unittest discover -s test -p test_a10_dual_inference.py -v
```

旧 `scripts/a10_openpi_client.py` 入口已移除，改用上述标准入口及配置。

### 本机验收记录（2026-11-13）

- 远程训练适配 `12012cc` 已合入本地，保留本地CLI及原设备配置；未创建提交。
- 模型12个文件共14,488,856,985字节，全部通过ModelScope SHA256校验。
- 原生模型812个权重张量严格加载成功，真实Thor CUDA普通FP32推理完成。
- OpenPI与FluxVLA各执行两周期模拟，每周期10帧，输出均为有限的[30,14]，两次停止均确认。
- OpenPI客户端请求约330/306ms；FluxVLA预热运行约2473/1329ms；均不含真实设备通信。
- 两条链路模拟关节发送平均间隔约33.33ms。最终FluxVLA验收使用不存在的本地模型路径，确认远程客户端无需模型包。
- 相关回归57项通过、1项可选GPU测试跳过，另有30个子测试通过；真实GPU推理在独立服务验收中完成。
- 最终原生客户端记录：`work_dirs/a10/inference_logs/20261113_154359_e2f3dd8b_dual_client/`。
- OpenPI客户端记录：`work_dirs/a10/inference_logs/20261113_153809_ac6fcd1f_dual_client/`。

验收时FluxVLA服务由临时用户服务 `fluxvla-a10-dual.service` 运行，未设置开机启动。
验收完成后已按用户要求停止该服务及OpenPI双臂服务，5556和8001端口均已释放。
以下命令用于服务运行期间查询或停止：

```bash
systemctl --user status fluxvla-a10-dual.service --no-pager
# 需要前台按上文命令启动时，先停止当前实例，避免端口冲突
systemctl --user stop fluxvla-a10-dual.service
```

服务重启后首次模型加载可能约需一分钟，客户端120秒请求超时包含等待服务就绪时间；
该等待不代表纯模型推理耗时。
