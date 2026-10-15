# A10 XLeVR 遥操作算子

## 当前开发分支（2026-10-15）

VR 后续开发统一在 `kaanh_follower_vr`，工作目录为 `/home/robot/VLA/FluxVLA`。
本分支从 `kaanh_follower` 的 `efcdb2f` 创建，选择性迁入
`backup/vr-before-consolidation-20261015`（`e9e4b93`）中的 VR 修改，
沿用当前 A10 通信、相机、夹爪和加速推理实现。

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python -m fluxvla.projects.a10 teleop --vr-only --execute
```

上述命令仅接收 VR 输入，不连接机械臂；省略 `--execute` 时只预览命令。
头显通过本机局域网地址访问，地址由 `configs/a10/site.local.yaml` 的
`vr_server.host_ip` 设置。配置字段示例见 `configs/a10/site.example.yaml`。

本轮完成旧 VR 功能迁入，遥操作入口仍在 `projects/a10/teleop.py`，
底层输入与 mapper 仍在 `projects/a10/vr`；通用遥操作基类与 runner 的进一步
归位、新 follower 下的实机验证及录制联动是后续任务。

## 当前实现

`fluxvla/engines/operators/vr_operator.py` 组合平台现有的 XLeVR 输入与
LeRobot 移植过来的 `XLeVRDeltaEEMapper`，由 `A10Operator` 驱动 Cartesian
follower。手柄产生末端局部坐标增量；算子从实际 TCP 建立参考，并在连续
目标上累积增量，发送相对 follower 启动锚点的米／xyzw 位姿。右手摇杆可以单独
控制可选的 USB `GripperOperator`。
左手摇杆向上只重置 VR 映射参考，不会执行回零。

先用 `python -m fluxvla.projects.a10 teleop --vr-only --execute` 检查浏览器输入；
这一路不连接机器人。普通 `teleop --execute` 需要站点配置中的
`robot.allow_motion: true`，并在 `vr_operator` 下明确填入以下现场验证值：

- `feedback_position_unit`: 控制器 `motion_msg.pq` 的平移单位，`mm` 或 `m`；
- `max_position_step_m`、`max_rotation_step_rad`: 现场确认的单帧目标限值。

`max_position_tracking_error_m`、`max_rotation_tracking_error_rad` 限制累积
目标相对于实际 TCP 的领先量，默认 0.05 米、0.35 弧度，需结合现场条件调整。
超过限值时停止本轮遥操作并请求退出 follower，避免机器人卡住时持续积累目标。

这些值缺失时，算子在连接或发送运动前拒绝运行。算子不自动使能电机；
控制器需先处于 Enabled、Stopped，启动时显式选择 Tool 坐标并确认
Cartesian follower。退出时请求 `stop_follower` 并关闭连接；这不等同于急停。
`received_monotonic_s` 是主机收到反馈的时间，不能证明控制器内部采样新鲜度。
如要用 USB 夹爪，另行验证摇杆方向并配置 `vr_operator.gripper_port`。
共享项目配置加载还要求开启运动时填写 `robot.max_joint_step_rad` 和
`robot.gripper_mm_range`；它们不会代替上面的 VR 笛卡尔限值。

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
`VROperator` 负责锚点保存和绝对目标转换。启动要求 Stopped，且读取启动
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
来源和许可证见 `fluxvla/projects/a10/vr/NOTICE.md`。

## 与《遥操作使用说明(1).pdf》核对

2026-10-12 对照项目根目录的五页 PDF 原文和第 4 页原始页面：

| 说明书 | 当前实现／使用方式 |
| --- | --- |
| 第 1 页：Cartesian UDP 9998，关节 UDP 9999，按 tg_id 配置控制末端 | `A10Operator` 使用两个独立端口；VR 仅发送单臂的一组 `pq`，控制器 tg_id／ee_ids 必须配置到 A10 末端 |
| 第 2～3 页：`get` 中读取 `follower_msg.running_state` 和 `follower_mode` | 启动、停止及每次目标更新前回读；确认本会话的 Cart 模式后才发目标 |
| 第 3～4 页：Cartesian 跟随最新位置，下发增量位姿，单位米／弧度，`pq` 七维二维数组 | 持续发送 `{"type":"pq","pq":[[dx,dy,dz,qx,qy,qz,qw]]}`；位姿相对固定启动锚点，非每包重复累加的帧增量；四元数无角度单位 |
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

## 原分支验证记录（2026-10-12）

操作器回归 76 项通过，其中 VR 22 项、A10 协议 39 项；共享 A10 原生流程
和工作流回归另有 6 项通过。VR 测试使用真实 mapper 与回环 WS／UDP 控制器，
检查米／xyzw 目标、40 字节报文头、反馈滞后下的位移累积、局部旋转组合、
重复／陈旧输入、松手保持、重置参考、跟踪误差限制及启动失败清理。
新增启动锚点旋转下的相对四元数、移动后的保持目标、锚点不受 VR 重置影响、
重新启动刷新位置／姿态锚点的验证。
CLI 预览与 `git diff --check` 通过。未连接实机或发送实机运动命令。

原分支验证时曾缺少 `pyserial`。当前 follower 已在 `requirements-a10.txt`
固定 `pyserial==3.5`，迁移沿用该依赖；不再需要原验证时的临时依赖路径。


## 新分支迁入验证（2026-10-15）

`kaanh_follower_vr` 已完成软件迁入：VR 专项 22 项与 A10／驱动／控制台等
合计 124 项通过；共享项目 6 项通过（含本机 HTTPS/WSS 模拟输入）；
推理 runner／适配 20 项通过，真实 GPU 单项默认跳过。合计 150 项通过，
1 项跳过。日志在 `work_dirs/a10/vr_migration/20261015/`。本地 site.local
和示例配置均可加载；VR-only 构建和命令预览通过。尚需现场核对头显输入、
坐标方向、反馈单位、限值、松手保持及夹爪方向，再验收实机与录制联动。
