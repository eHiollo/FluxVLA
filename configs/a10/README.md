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

## 夹爪调试界面

唯一入口为 `scripts/a10_gripper_ui.py`，网页与服务均内置，不需要桌面、DISPLAY 或额外网页依赖。
在已激活的 `fluxvla` 环境中启动：

```bash
python3 scripts/a10_gripper_ui.py
```

默认监听 `192.168.110.119:8765`。在你电脑浏览器打开终端显示的完整链接（含 `?token=...`），
电脑能访问机器人 IP 即可，无需 SSH 转发。`--host`、`--port` 可覆盖监听地址和端口，
`--config` 可选择另一份设备清单。需要监听全部网卡时使用 `--host 0.0.0.0`；
仅本机访问时使用 `--host 127.0.0.1`，SSH 用户再自行建立端口转发。

界面提供左右夹爪单独张开、关闭、指定 raw 位置、状态查询、执行日志及双爪同时开合。
运动速度为1–50%，提供5%、10%、20%、35%、50%快捷档，默认读取设备清单中的5%。
调速只在下次动作生效，不改写清单；双爪按钮使用同一速度，先检查两侧身份、端点、连接和报警，
再经独立串口并行发送，分别显示实际位置、到位/遇阻/超时结果，不提供硬件时钟同步保证。
执行中一侧失败时另一侧可能已经动作，结果会分别保留。

可选“掉落报警 B=0x1000 仅提示”，默认不勾选；只降级该位，其他报警仍阻止运动，
原始报警持续显示，不自动清除。启动和页面刷新不发送运动或自动打开串口，
操作期间禁用按钮且服务拒绝重复请求。关闭页面或服务不会停止已发送的动作或释放力矩。

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
