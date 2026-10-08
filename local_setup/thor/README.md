# 本机 Thor 运行环境

配置记录日期：2026-10-08。

环境路径：`/home/robot/miniforge3/envs/fluxvla-thor`。
已安装 FluxVLA 基础依赖、`requirements-real.txt` 中的真机／远程推理 Python 依赖，以及 `requirements-a10.txt` 的 WebSocket 依赖。

## 使用

```bash
source /home/robot/miniforge3/etc/profile.d/conda.sh
conda activate fluxvla-thor
cd /home/robot/VLA/FluxVLA
export CUDA_HOME=/usr/local/cuda-13.2
export WANDB_MODE=disabled
python scripts/zmq_inference_server.py --help
```

运行模型时，按选定模型提供配置文件及其对应的 checkpoint。
A10 的基础权重、tokenizer 和最新真实数据副本已放入 FluxVLA；机器人和物理相机现场连接尚未验收，ROS 系统环境及 LIBERO/RoboCasa 仿真未配置。

## 已安装的核心版本

| 组件 | 版本 |
| --- | --- |
| Python | 3.10.21 |
| CUDA toolkit | 13.2 |
| PyTorch | 2.13.0+cu132 |
| torchvision | 0.28.0+cu132 |
| FlashAttention | 2.8.4 |
| Transformers | 5.3.0 |
| Diffusers | 0.37.0.dev0 |
| PyAV | 14.2.0 |
| pip | 25.3 |

FluxVLA 源码提交：`6c94e73139d51fca9650fa76265c468378020558`。

Diffusers 使用仓库固定提交 `3996788b602eaae4da41a1d45726b62e662b73cf` 的官方源码归档。
FlashAttention 使用上游提交 `e9cf2c1651d2303191eb40a739a3c135fda00999`，
其 Cutlass 提交为 `7127592069c2fe01b041e174ba4345ef9b279671`；本机针对 `sm_110` 从源码构建。
FluxVLA 以 editable 模式安装，三个 CUDA 扩展使用本机 CUDA 13.2 编译。

`installed-packages.txt` 保存当前环境的包版本快照，供本机后续核对。
安装元数据中的 `/tmp` 源码路径来自当时的构建目录，重装时需重新获取上述固定提交。

## 安装时处理的问题

- GitHub Git checkout 出现 TLS 中断，改用同一固定提交的官方源码归档安装 Diffusers。
- pip 26.2.1 在代理下载时出现 `key_proxy_ssl_context` 异常，改用 uv 完成安装，随后将 pip 调整到 25.3 并验证网络下载。
- 基础依赖与真机依赖合并解析后，protobuf 为 4.25.9，Google 数据访问依赖同步调整到兼容版本。
- `pip check` 仍会报告 `nvidia-cusparselt-cu13 0.8.1 is not supported on this platform`。该包的 wheel 标签为 `manylinux2014_sbsa`；其余包版本约束已通过检查。普通 CUDA、FlashAttention 和 FluxVLA 自带 CUDA 算子需以实际运算验证为准，尚未单独测试 cuSPARSELt 的稀疏运算。

## 验证结果

- FluxVLA 导入及三个自带 CUDA 扩展导入成功。
- FlashAttention 与 PyTorch SDPA 前向结果最大绝对误差约 0.00024，反向梯度有限。
- Gemma 旋转频率、旋转位置编码与 PyTorch 参考结果一致，位置编码最大绝对误差约 2.4e-7。
- bf16 的 cuBLASLt 矩阵乘法加偏置与参考结果一致。
- 真机推理和 ZMQ 服务入口的 `--help` 均成功运行。
- pip 25.3 经现有代理下载包成功。

A10 原生 PI0.5 已用真实数据完成 1 步短训、完整恢复至第 2 步及 `.pt`／`.safetensors` 保存；保存后的真实权重通过 ZMQ 服务完成 3 个观测的离线推理，均输出有限的 `30×7` 动作。原生录制器、相机转换、本地 HTTPS/WSS 服务和模拟控制桥接的检查也通过。

这是工程流程验证，只有两步训练，预测仍可能超出物理范围，不能用于实机效果验收。Thor 单卡使用已有 `DDPTrainRunner`，EMA 默认关闭；完整恢复使用按需映射 checkpoint 以减少统一内存占用。阶段状态和详细日志见 [A10 迁移开发文档](../../docs/a10_full_migration_plan.md)。本阶段已按用户要求暂停。
后续更新依赖时，应保留已验证的 PyTorch、FlashAttention 版本，并重新检查实际 GPU 运算。
