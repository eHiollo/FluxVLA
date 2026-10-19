# A10 VR 遥操作使用与开发说明

## 当前分支和工作目录

VR 开发在 `kaanh_follower_vr`，当前独立工作区为 `/home/robot/VLA/FluxVLA_vr`。
主目录 `/home/robot/VLA/FluxVLA` 仍属于 `kaanh_follower`，其中的现场修改由用户管理。
在 IDE 打开 VR 工作区即可查看本轮代码；此工作区沿用现有 `fluxvla` Conda 环境。

### 2026-10-19 现场旋转异常修复

11:50 的现场会话出现第 6 轴持续旋转和松手后停止确认超时。只读对照控制器日志后，
确认 11:50:38 已收到“停止跟随”，11:50:45 又报第 6 轴目标正限位（-503）。
**跟随模式 inactive 不代表机械臂已停止。** 详细证据和待验收项见
[参考目标迁移记录](a10_vr_reference_migration.md#2026-10-19-现场旋转异常排查与前端修复)。

当前 VR 配置选用 `teleop.operator.cartesian_wire_format='pe321'`：内部仍是 xyz 米和
xyzw 四元数，只有 UDP 编码转换为 `{"type":"321","pe":[[x,y,z,yaw,pitch,roll]]}`，
角度为带正负号的弧度。SDK 的 pq 接收路径会把微小负角度转换成接近 2π 的正角，
再逐分量滤波/规划；321 路径保留正负号，避免该绕转问题。

`A10Operator` 通用默认仍为 pq；VR 配置显式选择 pe321。新报文已做 SDK 纯数学
对照与回环收发验证，**尚未重新完成真机运动、松手停止和断流验收**。
不通过延长超时、放宽限位或自动掉使能掩盖停止未确认。

## 文件职责

```text
fluxvla/engines/operators/
├── vr_operator.py           # BaseTeleoperator + VROperator：输入、映射、事件和反馈
├── vr/                      # VR 底层与网页资源集中管理
│   ├── runtime.py           # VRTeleopRuntime：循环、follower、锚点、保持与清理
│   ├── control.py           # 锚点与参考目标算法；由 reference 模式调用
│   ├── input.py / mapper.py # 手柄输入与末端增量映射
│   ├── monitor.py / ws_server.py / server_config.py
│   ├── diagnostics.py / hud.py / vr_events.py / quaternion_utils.py
│   └── web-ui/              # 页面、脚本和样式，连同许可证一起打包
└── a10_operator.py          # 现有控制器协议与状态接口
scripts/teleoperate.py        # 配置加载和启动
configs/a10/a10_vr_teleop.py   # 唯一原生 VR 配置入口
scripts/replay_vr_control.py  # 新增：不连接设备的参考目标回放
```

`VROperator` 通过已有 `OPERATORS` 注册器构建，不连接机器人。
对外接口为 `connect/get_action/get_events/send_feedback/close`；读取动作时同时保存
`last_raw` 和 `last_action`。`send_feedback` 发送头显 HUD JSON，不表示机器人执行成功。
`reset_reference` 清空手柄映射参考。

2026-10-16 新增 `control_mode=reference`，默认 `delta` 仍为下述旧增量流程。
`--observe` 可以只读反馈并计算新目标；`--inspect-controller` 不启动 VR，只查询监视端口状态。
阶段 1～3 的范围、C++ 对照、只读命令、停止确认和 WS 不可用的现场记录见
[VR 参考目标迁移记录](a10_vr_reference_migration.md)。新模式尚未完成实机运动验收。

`VRTeleopRuntime` 组合输入与 A10Operator，保留启动锚点相对位姿、局部增量累积、
握键/精调、松手保持、输入过期、目标及跟踪误差检查。左摇杆上推只重置参考，不回零。
右摇杆可以独立控制可选 USB 夹爪，沿用共享 GripperOperator 的 0～85mm 工作范围。
USB 夹爪由 runtime 单独持有并关闭，避免与 A10Operator 重复打开同一串口；
这样无需启动机械臂 follower 也能使用摇杆夹爪。相机接口仍由 A10Operator 提供。

## 1. 检查配置：不启动服务

```bash
cd /home/robot/VLA/FluxVLA_vr
conda activate fluxvla
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py --preview
```

原生入口只加载上述 Python 配置，支持 MMEngine `--cfg-options`；
不读取 `site.local.yaml`、`project.yaml` 或 `projects/a10/config.py`。
现有环境已安装 `requirements-a10.txt` 中的依赖，网页证书在首次服务启动时创建。

## 2. 先验证头显输入（VR-only）

将下面 `YOUR_LAN_IP` 换成运行 FluxVLA 的电脑局域网地址：

```bash
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py \
  --cfg-options teleop.input.network.host_ip=YOUR_LAN_IP
```

配置默认 `teleop.vr_only=True`，不构建机器人或夹爪，不连接相机。
在头显浏览器打开终端打印的 HTTPS 地址，信任现场证书并进入 VR。
默认端口为 HTTPS 8443、WSS 8442；`127.0.0.1` 只供本机测试，头显需要电脑的局域网地址。
手柄映射动作每秒打印一次；Ctrl+C 或左摇杆下推退出。
需要短时运行可覆盖 `teleop.max_steps=60`；这是循环次数，不代表有效手柄包数。

## 3. 连接机器人前填写现场配置

在 `a10_vr_teleop.py` 中，或通过 `--cfg-options` 设置：

| 参数 | 含义 |
| --- | --- |
| `teleop.vr_only=False` | 开启机器人接入 |
| `teleop.operator.host` | 控制器地址，默认本机占位地址 |
| `teleop.operator.allow_motion=True` | 允许本会话控制机器人 |
| `teleop.operator.feedback_position_unit` | 实际 TCP 反馈平移单位 `mm` 或 `m` |
| `teleop.operator.cartesian_wire_format` | VR 配置为 `pe321`，保留小角度正负号；公共位姿输入仍是四元数 |
| `teleop.max_position_step_m` | 现场确认的每帧平移限制 |
| `teleop.max_rotation_step_rad` | 现场确认的每帧旋转限制 |
| `teleop.max_position_tracking_error_m` | 目标对反馈最大领先距离，默认 .05m |
| `teleop.max_rotation_tracking_error_rad` | 最大旋转跟踪误差，默认 .35rad |
| `teleop.operator.gripper_transport='usb'` | 可选，启用直接 USB 夹爪 |
| `teleop.operator.gripper` | USB 参数，例如 `dict(port='现场串口', max_width_mm=85., timeout=.015)` |

反馈单位和单帧限值默认未填写；缺失时运行前拒绝接入真机。
开启工控机 follower 服务，人工确认电机 Enabled、Stopped；程序不会自动使能或回零。
第一次握键选择 Tool 坐标并启动 Cartesian follower。退出请求 stop_follower，再关闭连接。
关闭连接、松手保持和退出 follower 不等于物理急停。
当前 VR 配置默认使用同步状态读取；推理配置的后台监视参数没有被改动。

## 4. 诊断和待办

默认逐帧诊断保存在当前工作区 `work_dirs/a10/vr/teleop_diagnostics.jsonl`，按行追加；
包含原始手柄输入、映射动作、已读取的机器人反馈、成功写出的相对 UDP 目标、
耗时与错误。成功写出不代表机器人已经到位；反馈时间也不是精确同步采样时间。
关闭诊断可设 `teleop.diagnostics_path=None`。

HUD 模块和反馈接口保留；录制分段、保存/重录/续录及录制 HUD 联动为下一轮工作。
旧录制链路仍在 `projects/a10`，只反向引用新 VR 输入，旧目录不再存放另一套 VR 实现。
旧 `python -m fluxvla.projects.a10 teleop ...` 入口明确提示新命令并退出。
本轮未重构其他训练/回放/检查功能，未进行真实头显或机械臂联调。

## 增量迁移与握键行为

旧 `A10_new/a10_vr_vel_plan.cpp` 的 `apply_vr_delta_to_target` 使用
`T_target = T_target · T_delta` 累积局部增量。迁移后保持这一组合顺序：
`p_next = p_target + R_target · delta_p`，
`R_next = R_target · Exp(delta_rotvec)`。如果每帧从实际反馈加增量，控制器的
跟随延迟会吞掉手柄位移；因此反馈用于建立参考和检查跟踪误差。

第一次握键触发 Tool 选择与 Cartesian follower 启动，启动确认后记录实际
TCP 为锚点，先发送 `[0,0,0,0,0,0,1]` 作为保持目标，并重置 mapper；
启动期间的手柄移动不补发。随后从
新的 VR 样本建立参考。连续握住时累积目标；重复 VR 样本产生零增量，继续
发送已有目标。松开、输入陈旧、mapper 重新建立参考或左摇杆重置时，使用
实际 TCP 替换旧目标，并持续发送该保持位姿。再次握住从实际 TCP 重新开始。
保持目标和退出 follower 都不能保证瞬时物理停止，停止过程由控制器执行。

2026-10-12 用户确认：控制器在每次 `start_follower` 时，以当时的位置和
姿态作为锚点，后续接收相对该锚点的位姿；停止后重新启动会刷新锚点。
因此平台内部保留基座坐标系中的实际 TCP 和累积目标，发送时转换：

```text
T_relative = inverse(T_start) * T_target
p_relative = inverse(R_start) * (p_target - p_start)
q_relative = inverse(q_start) * q_target   # xyzw
```

`A10Operator.send_eepose` 接收上述相对位姿，仅负责校验和协议发送；
`vr/runtime.py` 中的 `VRTeleopRuntime` 负责锚点保存和绝对目标转换。启动要求 Stopped，且读取启动
确认后的 TCP 前不发送 UDP 目标，因此用该反馈保存启动锚点。每轮退出后
清除本地锚点，重新启动重新采集。松手、再次握住和 VR 参考重置只更新内部
目标／VR 参考，follower 锚点保持不变。机器人已经移动时，松手后的保持
报文通常是非零相对位姿；发送零平移加单位四元数会要求回到启动锚点。
原 `feedback_pose_matches_target_frame` 确认项已移除，旧站点配置需删除该项。

`vr.motion_shaping_mode: robot_controller` 保留控制器承担运动整形的分工。
旧 `vr_vel` 在 500 Hz 中实现的低通、速度／加速度限制、目标回拉和关节保护
属于旧控制器计划；本次没有复制到 Python，也没有证据证明新 Cartesian
follower 使用同样参数。协议迁移完成后，仍需现场比较运动方向、延迟、松手
后的停止距离及跟随效果。

录制仍由 `record` 流程负责；本命令用于遥操作，不写演示数据。XLeVR 移植
来源和许可证见 `fluxvla/engines/operators/vr/NOTICE.md`。

## 与《遥操作使用说明(1).pdf》核对

2026-10-12 对照项目根目录的五页 PDF 原文和第 4 页原始页面：

| 说明书 | 当前实现／使用方式 |
| --- | --- |
| 第 1 页：Cartesian UDP 9998，关节 UDP 9999，按 tg_id 配置控制末端 | `A10Operator` 使用两个独立端口；VR 发送单臂的一组位姿，控制器 tg_id／ee_ids 必须配置到 A10 末端 |
| 第 2～3 页：`get` 中读取 `follower_msg.running_state` 和 `follower_mode` | 启动、停止及每次目标更新前回读；确认本会话的 Cart 模式后才发目标 |
| 第 3～4 页：Cartesian 跟随最新位置，下发增量位姿，单位米／弧度，支持 pq 与 Euler 格式 | 通用接口保留 pq；2026-10-19 起 VR 配置使用带符号的 `type=321, pe`。位姿仍相对固定启动锚点，非每包重复累加的帧增量 |
| 第 3～4 页：最大跟随速度由点动笛卡尔设置的速度与速度百分比决定 | 实际速度由控制器配置和百分比约束；VR 的位置缩放与单帧／跟踪误差限值不能代替控制器速度设置 |
| 第 4 页：关节使用 `jp --motor_pos=[...]`，绝对位置，旋转轴单位度 | `send_joints` 单独接入关节 follower，不能把该关节绝对位置格式用于 VR 位姿 |
| 第 5 页：`set_jog_vel --vel_percent=60`、`set_pgm_vel --vel_percent=60` | 在控制器原生终端／前端按当前运行模式显式设置；当前 `a10_control.py` shell 未提供这两个命令，VR 也不自动修改百分比 |

PDF 中 `start_follower`、`stop_follower`、`follower_cart`、`follower_aj`
分别列为控制指令，但未要求启动时必须连续发送两条指令。附件
`robot.7z/robot/kaanh_backend.py` 的 `start_follower()` 实际发送
`follower_cart`；当前 `A10Operator.start_follower('cartesian')` 采用同样
方式，并用反馈确认进入模式。因此 Python 方法名与 wire 指令名不同。

PDF 第 4 页的代码是简化示例：`type="321"` 展示 Euler 角次序，`pqs`
未执行 `push_back(pq)`，示例中的 `pq` 数组实际为空。四元数报文的
`type="pq"`、`xyzw` 顺序及 40 字节消息头由附件客户端补充确认；不将
PDF 中的空数组示例直接用于发送。启动锚点刷新规则与完整 SE(3) 转换公式
依据用户说明，PDF 本身只写了下发增量，未给出该公式。

第 4～5 页按虚拟／真机列出 `manual_en`／`en`，但本项目此前的现场回读
已确认真实控制器在 Manual 模式使用 `manual_en` 成功。使用既有现场流程
并确认 `activate=Enabled`；VR 本身不执行使能，也不切换模式。

现场对比手感时，先记录控制器的笛卡尔最大速度、当前模式和速度百分比。
旧 `vr_vel` 的速度／加速度参数与新 follower 的点动速度配置不是同一组
参数，不能由相同的 VR 缩放值推断两者速度或停止距离相同。

## 本轮验证

153 个不同软件测试通过：包含 VR 位姿与生命周期 22 项、新原生接口／入口 9 项、
共享流程 HTTPS/WSS 等 6 项及既有驱动／通信／推理 runner 回归。测试拦截所有
`fluxvla.projects` 导入后，新入口预览仍通过。日志在
`work_dirs/a10/vr_native/20261015/`。未进行实机或真实头显验证。
