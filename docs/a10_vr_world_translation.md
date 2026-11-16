# 90 Hz 世界坐标平移与局部坐标旋转

在 FluxVLA_90hz 仓库中运行：

```bash
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop_world_translation.py --preview
```

去掉 `--preview` 即沿用原双臂配置的实机和夹爪设置启动。
当前独立配置默认关闭控制器高频记录（record_controller=False），避免控制器 runtime log 容量上限阻止启动；本地 JSONL 诊断继续记录。
管理控制器日志容量后，可用 --cfg-options teleop.record_controller=True 显式恢复高频记录。
数据集录制仍使用原有的 `--record-config` / `--record-options` 参数。
原入口 `configs/a10/a10_dual_arm_vr_teleop.py` 保留原控制方式。
新入口不自动使能或回初始位；接管时读取状态和末端朝向；位置参考使用本地零点，测试台虚拟位姿不作为实机绝对目标。

新配置选择 `pose_mapping='world_translation_local_rotation'`，控制 90 Hz；只有附带录制时订阅 30 Hz 反馈，
继承全部运动参数与左右手绑定；新增世界平移专用的每手轴矩阵，局部旋转不变。
不读取或要求 VR 标定文件，也不改变 VR 网页和机器人协议。

每侧按侧键时分别记录手位置与本地参考位姿。跟随会话位置从本地零点开始：

```text
本地参考位置 = 本地接管参考位置
         + 现有平移矩阵 × (手的世界位置 - 手的接管位置)
           × pos_scale × vr_to_robot_scale
```

SDK 适配器把相邻参考位姿转换为累计增量，不把上述本地位置作为绝对坐标发送。
初始位置原点在差分中抵消；初始朝向用于把基座方向转换为 Tool 方向。
这里的“世界”是 VR 固定追踪空间，并非当前头朝向。
WebXR 世界为 +X 右、+Y 上、−Z 前；工作台世界为 +X 前、+Y 左、+Z 上。
物理左臂基座与工作台轴同向，物理右臂基座相对绕 Z 转 180°。
新配置按物理手柄设置 world_position_axis_remap：

- 左手柄（物理左臂、ee 1、逻辑 right）：(dx,dy,dz) → (−dz,−dx,dy)。
- 右手柄（物理右臂、ee 0、逻辑 left）：(dx,dy,dz) → (dz,dx,dy)。

左右手前推都对应工作台前方，左移都对应工作台左方，上移都对应上方。
world_position_axis_remap 仅在 robot_base 平移分支生效；旧模式仍使用原局部矩阵。
精细模式比例在接管时按原逻辑捕获，位置死区、输入检查和参考目标运动约束保持原逻辑。

旋转沿用原手柄局部增量和 incremental_velocity 路径。
自然握持可以对应朝下的夹爪；不会自动锁姿态，也不会把末端强制摆成朝下。
左右臂松开、重新接管、故障停止及恢复流程继续使用已有实现。

锚点指令增加内部字段 translation_frame='robot_base'。
旧指令无此字段时仍表示 tool_anchor；两种模式拒绝混用坐标标识。
诊断日志保存至 work_dirs/a10/vr/dual_world_translation_diagnostics.jsonl，
记录 pose_mapping、translation_frame 和每手 position_axis_remap。
新增 anchor_shadow_translation_m 表示所标注坐标系下的位移；
旧 anchor_shadow_translation_ee_m 只在工具坐标模式有效，新模式为 null。

离线回放支持：

```bash
python scripts/replay_vr_control.py --demo --pose-mapping world_translation_local_rotation --fps 90
python -m unittest discover -s test -p 'test_vr_world_translation.py' -v
```

离线测试不证明实机运动效果。现场先检查三个平移轴的正负方向，再验证局部旋转；
改变接管时的握姿或末端姿态，同一世界位移应产生相同的基座平移方向。

## 正常松键保持会话

按用户明确选择，新配置设置 `stop_on_both_release=False`：正常松开双手侧键/前扳机时，
参考轨迹按已有速度、加速度约束减速，保持最后指令目标，保留本次原生跟随会话。
前端速度归零且目标不再变化后，不重复发送旧累计目标包。
再次按侧键或前扳机沿本地连续参考重新接管，不发送 stop/start，也不 GET 或重新切 Tool。
控制器可能继续到达最后一个已发送目标；这里不声称已测得机械臂停止。

首次启动，以及断流/异常后的真正重新启动，仍读取状态和当前朝向。
VR 断流、输入异常、发送失败、退出/Ctrl+C 仍请求停止跟随。
`confirm_stop=False` 表示仅发送 stop_follower，不等待执行结果或额外回读 inactive/Stopped；
日志记为 stop_requested，不把发送成功当作停止确认。
退出时尚无停止确认则不自动恢复控制器倍率。原配置的松键停止逻辑保持不变。

## 控制与录制反馈解耦

新配置设置 `continuous_feedback=False`，接管后 90 Hz 控制只推进并发送增量目标，
不用持续 GET，也不检查后台反馈缓存的年龄。成功发送前不会更新本地参考；
原始实际反馈与接收时间保持原样，不把指令位置伪装成实测位置。
原配置保留 `continuous_feedback=True` 的反馈约束路径。

不带 `--record-config` 时不订阅后台 GET；带录制配置时继续订阅 30 Hz 实际反馈，
用于真实关节状态和数据记录。新配置通过 `operator.shared_control.require_feedback=False`
解除反馈轮询错误与目标权限的耦合；录制仍会报告查询失败并使缺失反馈的一集失效。
这不掩盖运动发送链路本身的断开，也不允许抢占其他会话。

`resilient_takeover=True` 保留界面处理暂不可接管状态；已处于 Tool 时跳过重复切换。
启动/重新启动跟随时仍读取 Enabled、Stopped、故障和当前朝向，实际切坐标仍检查状态。
这些是首次或异常后启动阶段的状态检查，不在正常重新按键或每轮控制中执行。新模式不用实际绝对位置建立增量原点。

查询失败不自动重连、不重发旧目标。
VR 断流、输入异常、循环超时、目标发送失败仍停发目标；松手、回中并稳定 0.5 秒后重新按侧键接管。
运行中不再依靠实测位置判断跟踪误差或控制器状态，速度、加速度和制动约束继续作用于指令。

## 侧键只控制 XYZ

新配置启用 `teleop.input.mapping.squeeze_translation_only=True`，左右手各自生效：

- 只按住侧键（squeeze）：仅控制 XYZ 平移，末端姿态锁定。
- 按住前扳机：保留原来的平移与手柄局部旋转；精调比例沿用原实现。
- 两键同时按：以前扳机为准，允许旋转。

从旋转切到仅平移时，锁住当前已发送参考姿态并清零前端角速度；
切回旋转时重设旋转参考，不追赶锁定期间的手柄旋转。
只重设旋转参考，不重设平移锚点，也不重启整组跟随；另一只手互不影响。
内部锚点指令用 `rotation_enabled` 传递模式，不改变机器人 UDP 协议。
原配置默认 `squeeze_translation_only=False`，保留原侧键行为。
