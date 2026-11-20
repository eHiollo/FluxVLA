# A10 VR 遥操作使用说明

**只想找运行命令：先看 [A10 常用命令速查](a10_command_cheatsheet.md)。** 本文说明当前入口与设备行为。

## 双臂当前默认入口

```bash
python scripts/teleoperate.py --config configs/a10/a10_dual_vr.py
```

默认开启实机、运动发送权限和两只知行夹爪；控制器高频记录默认关闭，本地诊断开启。
世界坐标平移、局部旋转、90Hz；侧键只控制平移，正常松键保持会话。
机械臂参数与VR参数分别维护，见[配置索引](../configs/a10/README.md)和[世界坐标说明](a10_vr_world_translation.md)。
旧双臂和世界坐标转接文件已删除，双臂参数统一修改 `a10_dual_vr.py`。
启动仍不自动使能机械臂；侧键控制对应臂，摇杆X控制对应夹爪。
`--preview`只检查配置；`teleop.vr_only=True`或`--observe`不继承默认控制器录制（`--observe --record-controller`可显式录制）。
普通夹爪通信异常只暂停该夹爪，不退出整个VR。读取CRC错误最多重读2次，写回复错误不重发原动作；
等有效反馈恢复、摇杆回中后即可继续。持续通信异常提示按5秒节流，原始收发与寄存器进入诊断。
数据集录制不将缺少有效夹爪反馈的片段写成正常帧；不会用目标替代实际位置。

## 旧单臂与老 USB 夹爪

使用 `configs/a10/a10_single_vr.py`，沿用 `a10_single_vr_base.py` 的机械臂/VR流程。
默认仍为VR-only；真实操作显式设置 `vr_only=False` 和 `operator.allow_motion=True`。
完整命令和参数见 [速查：旧单臂 .125 + 老夹爪 VR](a10_command_cheatsheet.md)。

`build_runtime()` 将USB夹爪配置交给现有 `GripperOperator`，由VR生命周期独立连接、关闭；
机械臂operator不重复打开串口。右手摇杆控制实际开口的小步目标，松开侧键也能操作。
`gripper_direction` 为1或-1，专用配置目标变化速度默认10mm/s、行程0..85mm。
夹爪IO与机械臂控制暂按现有顺序串行执行，日志增加 `gripper_command` 目标/反馈/耗时字段；
读取反馈后再检查输入是否仍新鲜，避免慢串口恢复后使用旧摇杆值继续闭合。

现场只读读取耗时约2.3～2.4ms，尚未发送实机开合目标。构造、配置预览、VR-only不打开串口；
只读观察仍拒绝USB设备配置，若需 `--observe` 请使用不带夹爪的原配置。
停止更新目标不会立即取消最后一个舵机目标；退出不自动开爪、回零或掉使能。
新知行夹爪保持独立驱动，后续另行接入。

## 双臂电机加速度

遥操作与 `m_init` 使用 `operator.motor_acceleration_deg_s2=1000`，推理使用375，单位为°/s²。
启动运动前回读匹配；不匹配时仅在厂家权限、手动、未使能、Stopped、follower退出且程序Unloaded时修正并核验。
修正后保持未使能，需要手动使能后重新启动；程序不自动使能。完整操作见 [推理与加速度说明](a10_dual_fold.md)。
