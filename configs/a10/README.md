# A10 配置索引

当前双臂 VR 统一使用 `a10_dual_vr.py`：世界坐标平移、手柄局部旋转，90Hz控制；录制独立30Hz。
运行命令见 [CLI.txt](../../CLI.txt)，映射与会话行为见 [世界坐标说明](../../docs/a10_vr_world_translation.md)。
数据集 v3 → v2.1 导出、视频模式和断点继续见 [数据转换说明](../../docs/a10_dataset_conversion.md)。

## 参数应该改在哪里

| 配置 | 职责 |
| --- | --- |
| [a10_dual_arm_operator.py](a10_dual_arm_operator.py) | 双臂机械臂连接、反馈单位、ID、关节/目标顺序、归位位置、共享通信；控制台、VR与推理共用 |
| [a10_dual_devices.py](a10_dual_devices.py) | 相机与夹爪的身份、连接路径、硬件行程和设备检查参数；换USB口可用 `scripts/a10_devices.py --update` 更新 |
| [a10_dual_vr.py](a10_dual_vr.py) | VR服务、手柄绑定、坐标映射、按键、倍率、参考轨迹速度/加速度约束、夹爪操控手感和会话策略 |
| [a10_dual_arm_record.py](a10_dual_arm_record.py) | 数据目录、任务、30Hz采样、录制按键、数据有效性阈值和保存参数 |
| [a10_dual_fold_inference.py](a10_dual_fold_inference.py) | FluxVLA双臂模型及客户端；默认preview |
| [a10_dual_openpi_remote.py](a10_dual_openpi_remote.py) | OpenPI双臂客户端；默认preview |
| [双臂训练配置](../pi05/pi05_paligemma_a10_dual_fold_10steps.py) | 双臂训练数据、模型及优化设置；资源配置见 [训练说明](../../docs/a10_dual_fold.md) |

`a10_dual_vr.py` 只继承机械臂和设备配置，不经过旧遥操作入口。
机械臂参数来自 `operator`；VR会话中覆盖运动授权和反馈策略，保留其他消费者的默认值。
夹爪硬件身份来自设备清单，VR电机速度覆盖为50%，设备检查仍为5%。
VR默认开启实机和夹爪，不自动使能或归位；先用 `--preview` 检查配置。
控制器高频记录默认关闭，清理日志容量后可显式使用 `--record-controller`；本地诊断继续记录。

## 仍使用的旧单臂配置

| 配置 | 用途 |
| --- | --- |
| [a10_single_operator.py](a10_single_operator.py) | 旧单臂控制台的机械臂配置 |
| [a10_single_vr_base.py](a10_single_vr_base.py) | 旧单臂VR默认配置，默认只看输入 |
| [a10_single_vr.py](a10_single_vr.py) | 旧单臂及老USB夹爪 |
| [a10_single_record.py](a10_single_record.py) | 旧单臂录制器配置，与双臂格式独立 |
| [a10_single_openpi_inference.py](a10_single_openpi_inference.py) | 旧单臂OpenPI权重加载、本地推理和模型服务 |
| [a10_single_remote_inference.py](a10_single_remote_inference.py) | 旧单臂远程模型客户端 |
| [a10_single_rtc_inference.py](a10_single_rtc_inference.py) | 旧单臂原生权重/RTC推理 |

双臂VR只保留 `a10_dual_vr.py`；旧双臂及旧世界坐标转接文件已删除，命令需改用新入口。
`--cfg-options teleop.…` 覆盖方式保持不变；
旧单臂入口保持原有行为，不继承双臂 `a10_dual_vr.py`。

## 旧 YAML 工作流

[a10_single_project.yaml](a10_single_project.yaml) 与 [a10_single_site.example.yaml](a10_single_site.example.yaml) 服务于
`python -m fluxvla.projects.a10` 的旧单臂工作流及 [pi05_a10_single_finetune.py](pi05_a10_single_finetune.py)。
`a10_single_site.local.yaml` 是可选的本地覆盖。原生双臂VR不读取这套YAML，修改它不会改变双臂VR参数。
本地标定文件和备份继续保留；世界坐标模式不加载旧姿态标定。
