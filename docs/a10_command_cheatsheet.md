# A10 常用命令速查

在 `/home/robot/VLA/FluxVLA` 激活 `fluxvla` 环境。
完整命令集中维护在 [CLI.txt](../CLI.txt)，参数位置见[配置索引](../configs/a10/README.md)。
机械臂配置管理连接与拓扑，`a10_dual_vr.py` 管理VR输入与遥操作；旧双臂转接配置已删除，统一使用新入口。

## 双臂世界坐标遥操作

```bash
# 只检查配置，不连接设备或启动服务
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py --preview

# 仅手柄输入和VR界面
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py \
  --cfg-options teleop.vr_only=True

# 只读真实机器人反馈，不启动跟随或下发运动
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py --observe

# 实机双臂与知行夹爪
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py
```

默认控制90Hz、世界坐标平移与局部旋转；侧键仅平移，前扳机允许旋转。
正常松键减速保持会话，不自动使能或归位。控制器高频记录默认关闭，本地JSONL诊断继续开启；
管理控制器日志容量后，显式加 `--record-controller` 开启控制器诊断记录。
映射、松键与异常恢复机制见[世界坐标说明](a10_vr_world_translation.md)。

## 双臂录制

```bash
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py \
  --record-config configs/a10/a10_dual_arm_record.py \
  --record-options record.root=datasets/cloth_folding_01 'capture.task=Fold the clothes'
```

新建数据集使用新目录；续录原数据时额外设置 `record.resume=True`，必要时使用原数据绝对路径。
独立30Hz采样保存真实关节、夹爪及三路图像。控制器高频诊断日志与训练数据录制独立。
手柄菜单、重录、保存和异常处理见[录制指南](a10_vr_recording.md)。

```bash
# 只检查录制按键及VR延迟，不连接机器人、夹爪或相机
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py \
  --record-config configs/a10/a10_dual_arm_record.py \
  --check-record-buttons --measure-vr-latency
```

## 控制台、设备与诊断

```bash
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py
python scripts/a10_devices.py
python scripts/a10_devices.py --snapshot
python scripts/a10_devices.py --update
python scripts/a10_devices.py --check-grippers
python scripts/preview_vr_mapping.py
python scripts/diagnose_vr_network.py --vr-ip 192.168.110.211
```

网络诊断中的IP应替换为当前头显地址。`--update` 自动备份设备配置，仅更新匹配的连接路径。
控制台和VR可先后任意启动，共用本地连接服务；控制台接管后，输入 `stop_follower`，
双手松键、摇杆回中，再重新接管VR。服务日志位于 `work_dirs/a10/shared_control/service.log`。

## 训练、推理与旧单臂

双臂训练资源、FluxVLA模型服务和OpenPI客户端见[双臂训练与推理](a10_dual_fold.md)。
两种双臂推理配置默认preview；旧单臂控制台、老夹爪及加速推理命令见[CLI兼容用法](../CLI.txt)。
旧单臂默认VR-only，行为保持不变。

各阶段开发与参数记录保留在[VR开发说明](a10_vr_operator.md)、
[迁移记录](a10_vr_reference_migration.md)和[分支合并记录](a10_branch_integration_20261116.md)，
其中历史双臂默认值和旧工作目录不作为当前启动依据。
