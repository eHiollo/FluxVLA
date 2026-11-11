# A10 VR 遥操作使用与开发说明

**只想找运行命令：先看 [A10 常用命令速查](a10_command_cheatsheet.md)。** 本文保留功能说明与开发细节。

## 旧单臂与老 USB 夹爪

使用 `configs/a10/a10_vr_teleop_legacy_gripper.py`，沿用 `a10_vr_teleop.py` 的机械臂/VR流程。
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

## 当前分支和工作目录

2026-10-26 已整合到 `kaanh_follower`，统一工作目录 `/home/robot/VLA/FluxVLA`。
原 `kaanh_follower_vr` 分支和 `/home/robot/VLA/FluxVLA_vr` 目录保留作回退。
沿用 `fluxvla` Conda 环境。推理、控制台和 VR 使用各自配置；同一机器人不要同时运行推理与 VR。

本机已保留原 VR 标定文件；原 VR 证书复制到 `work_dirs/a10/vr/tls_vr`，当前 VR 配置默认使用该目录。
主目录原有 `work_dirs/a10/vr/tls` 证书和 `site.local.yaml` 未覆盖。标定仍需与现场 VR 会话、机器人匹配。

**2026-10-19 更新：检查当前标定方向请优先使用下方的 `--verify-calibration` 网页。**
现有单姿态标定的重建误差小，只能说明采样与计算一致，不能证明实际射线与夹爪方向正确。
目前的现场方向映射、奇异点及停止过程问题仍需分别核实，不能用“文件加载成功”作为实机验收。

## 只读标定验证网页（推荐先用这个）

### 启动

在 `FluxVLA` 目录、`kaanh_follower` 分支运行：

```bash
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --verify-calibration \
  --cfg-options \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0
```

默认读取 `configs/a10/vr_calibration.local.json`；自定义文件加 `--pose-calibration /绝对路径/标定.json`。
也可以加 `--preview`，只检查配置，不连接设备。无需填写 `allow_motion=True`。

程序打印两个地址：头显打开原 VR 输入页，电脑打开 `/calibration.html` 验证页。
例如本机地址仍为 `192.168.0.111` 时：

- 头显：`https://192.168.0.111:8443`
- 电脑：`https://192.168.0.111:8443/calibration.html`

电脑只打开验证页，不要再打开第二个 VR 输入页。验证页通过 HTTPS 获取显示数据，不加入手柄 WebSocket。
本地证书首次访问仍需信任。机械臂应处于 **Stopped、跟随器未运行、无故障**；程序不会替你停止、使能或掉使能。

**旧 VR 页面需要更新后才会上传新的追踪与射线字段。**缺少有效 WebXR 追踪时页面会明确提示，
不会把默认零位置当作手柄姿态。刷新或重入 VR 后坐标原点可能改变，旧标定必须重新只读核对，勿直接实机试跑。

### 怎样看图

- 灰色：实际 TCP 姿态；蓝色：按当前标定计算的期望夹爪姿态。
- 橙色：网页实际 raycaster 射线，经当前操作坐标映射后的方向。未收到射线时显示未知，不默认使用手柄 -Z。
- 图中两个夹爪放在同一位置，以便只比较朝向；平移另在下方显示，单位毫米。
- 默认假设 TCP `+Z` 指向夹爪前方、`+X` 指向上方，这是旧代码约定，**不是已确认的安装事实**。
  请先结合现场夹爪和工具坐标定义核对，在下拉框中选择正确轴。此选择仅用于看图，不改标定。
- 完整姿态差包含翻转，前方夹角只比较指向。手柄朝向与实际夹爪不一致时，夹角大本身不是错误。
- 观察方位/仰角滑块只改变看图角度；以图中基 X/Y/Z 和现场已确认方向判断左右上下。

### 五项检查

1. 手柄射线指向实际夹爪前方，并将两者上方摆成一致：灰色和蓝色应接近重合。
2. 射线向现场左/右转：目标夹爪应同向转动。
3. 射线向现场上/下转：目标夹爪应同向转动。
4. 射线基本不变，只拧转手柄：目标夹爪应绕自身前方翻转。
5. 保持朝向，按住侧键沿现场确定的基坐标方向平移：检查位移分量；松开再按下，平移应重新从零开始。

每项可选择“符合预期/不符合预期”，同时保存当时的显示样本；改变轴选项会清空这些人工判断。
“冻结画面”方便截图，冻结期间仍显示当前连接状态，并明确标注这是历史帧。
“导出检查报告”由浏览器下载一个 JSON，包含轴选择、人工结果、各项样本、标定来源和 SHA256。
默认不持续写新 JSONL，也不覆盖标定文件。

本模式只读取默认 `5888` 监视端口，强制禁止运动并过滤相机/夹爪配置，只发送 `get`。
它复用当前绝对姿态公式和平移锚点，**不运行 reference 跟随或追踪误差检查**；机械臂静止而手柄转远不会因此触发 `tracking_error`。
输入/反馈过期、故障或机器人正在运动时，实时画面清除并提示原因。Ctrl+C 只关闭连接。

### 检查能说明什么

射线和目标使用同一个 `R_base_vr`，二者重合只能辅助检查局部指向关系，不能独立证明这个基座变换正确。
还必须用真实夹爪、现场固定方向进行上述多方向检查。界面不自动给出“标定通过”，也不验证 IK、关节速度、奇异点或停止性能。
本次没有修改标定算法，也没有修复实机停止/初始对齐问题。

实现依据：当前网页使用的 [A-Frame 1.7.1 laser-controls 源码](https://github.com/aframevr/aframe/blob/v1.7.1/src/components/laser-controls.js)
允许控制器模型提供专属射线偏置，因此读取运行时 raycaster 配置，而非硬编码射线为 -Z。

## 新增：两种手柄位姿映射方式（2026-10-19）

现在可以选择“手柄怎么动，机器人应该去哪里”。这和 `control_mode` 是两个参数：
`control_mode=reference` 管目标的限速与平滑；`pose_mapping` 管目标的计算方式。

| `pose_mapping` | 平移 | 朝向 |
| --- | --- | --- |
| `anchor_relative`（默认，原方式） | 按接管时的手柄和末端坐标计算相对位移 | 从接管时末端朝向开始，跟随手柄相对转动 |
| `calibrated_absolute`（新增，要求 reference） | 按固定、标定好的操作方向移动；每次接管重新记录起点 | 始终跟随标定后的手柄绝对朝向；不会每次接管重新对齐标定 |

新方式下，例如操作坐标系的“向前”标定好之后，转动手腕或改变机械臂初始朝向都不会改变
平移的“向前”。重新握住接管时，机器人当前位置就是新的平移起点，不会跳回上次起点。

**朝向仍可能在接管后变化：** 即使手柄不动，如果机械臂当前朝向与标定后的手柄朝向不同，
机械臂也会逐步对齐。启动第一包保持当前实际位姿，随后读取新鲜手柄数据，经现有
reference 速度、加速度和跟踪误差保护生成目标；平移不必等待朝向完全对齐。
这只限制前端发送目标，控制器的物理启停效果仍需实机确认。

### 选择方式与配置检查

原方式无需修改。切换回原 reference 映射：

```bash
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py --preview \
  --cfg-options teleop.control_mode=reference teleop.pose_mapping=anchor_relative
```

使用新方式前，需要补齐 `configs/a10/a10_vr_teleop.py` 的 `pose_calibration`。
`hand_to_tool_quat_xyzw` 已复用旧局部轴映射作为默认值，只需确定现场的
`vr_to_robot_quat_xyzw`。四元数顺序均为 `[x,y,z,w]`，范数必须为 1，然后检查：

```bash
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py --preview \
  --cfg-options teleop.control_mode=reference teleop.pose_mapping=calibrated_absolute
```

`vr_to_robot_quat_xyzw` 仍默认 `None`，因此未完成现场标定时这个命令会明确报错。
它不会自动用单位四元数代替。
标定填好后也可以用 `--cfg-options teleop.pose_calibration.字段名='[x,y,z,w]'` 覆盖。
原方式会忽略这些标定值，切换参数不必删除标定配置。
去掉 `--preview` 才会启动 VR；仍默认 `vr_only=True`，不会连接机械臂。
实机连接沿用下文命令的主机、使能许可、反馈单位等配置，不会由新映射自动开启。

### 两个标定值分别是什么

- `vr_to_robot_quat_xyzw`：把 **VR 固定追踪空间的方向**旋转到**机器人基座方向**。
  它决定手往哪个方向移动时机器人也往哪个方向移动。它不是每帧随手腕变化的方向。
- `hand_to_tool_quat_xyzw`：**工具坐标轴在手柄坐标系中的方向**（右乘）。
  它决定怎样握手柄时夹爪应朝哪里，并非把此四元数取逆后填入。
  默认 `[0.7071067811865475, 0.7071067811865475, 0, 0]` 来自原 LeRobot
  `AXIS_REMAP_VR_TO_ROBOT` 的逆旋转，沿用手柄局部轴到末端局部轴的约定。
  这不是 XML 中法兰到夹爪的工具安装偏移，也不是 VR 固定空间到基座的标定。

沿用默认局部轴约定时，现有程序以一次**有意识的参考姿态对齐**拟合参数：机械臂静止，手柄握成希望与当前
夹爪对应的朝向，同时读取两者姿态，结合已知的 `R_hand_tool` 求 `R_base_vr` 并固定保存。
不要求手柄和夹爪位于同一位置。但该拟合不能替代独立方向验证；默认局部轴未被物理确认时，不应认为一次拟合就完成标定验收。
参考姿态应明确手柄的指向和绕自身轴的转角；沿用这套约定后不能再独立指定任意平移方向，
生成的固定平移坐标也由这次对应关系决定，应先只读检查方向是否符合操作习惯。
如果要独立指定另一套平移方向和握持姿态，则需要重新确定这两个旋转。
下面的交互标定程序会计算并保存现场参数，不必手填四元数。
头显重置空间、追踪原点改变或设备位置改变后，需要重新确认标定，不能沿用错误的坐标系。

开发时的精确关系如下；`R_A_B` 表示把 B 坐标表达的向量旋转到 A 坐标表达：

```text
目标位置 = 接管时实际末端位置
         + R_base_vr × (接管时平移倍率 × (当前手柄位置 - 接管时手柄位置))
目标朝向 R_base_tool = R_base_vr × R_vr_hand × R_hand_tool
沿用默认轴约定时：
标定 R_base_vr = R_base_tool_calib × inverse(R_hand_tool) × inverse(R_vr_hand_calib)
```

### 一次参考姿态标定：照着操作即可

已实现 `--calibrate-pose`。交互和采样在现有 `vr/runtime.py`，数学计算在 `vr/control.py`。
**标定只读控制器监视端口（默认 5888），不使能、不启动 follower、不发送运动或夹爪指令。**
机械臂停在你希望作为参照的姿态，先结束其他遥操作/跟随程序；控制器需已运行通信服务。

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --calibrate-pose \
  --cfg-options \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0
```

1. 在 VR 浏览器打开终端打印的 `https://电脑局域网IP:8443`。首次连接仍需信任本地证书。
2. 观察静止的夹爪，把右手手柄摆成希望对应夹爪的朝向，注意指向和翻转角度。
3. 在电脑终端按回车，会倒计时 **5 秒**；利用这段时间戴好头显、摆正手柄并按住右手握键。
4. 保持手柄及机械臂静止约 **1 秒**。程序采集至少 10 对不同序号的新鲜姿态，
   默认持续至少 1 秒；重复帧不计数，序号回退、过期、故障或跟随仍活动会拒绝。
5. 看到“标定已保存”即完成。松开握键即可，程序会关闭 VR 服务与状态连接。
   此过程不会让夹爪对齐手柄；它只记住你主动摆出的对应关系。

**标定完成后保持 VR 页面和当前追踪会话，不要刷新页面、退出 VR 或重新居中。**
启动下一条观察/遥操作命令后，网页会每秒重试 WebSocket，服务恢复即可继续发送同一追踪空间的数据。
WebSocket 重连只恢复传输，不会重新创建 XR 会话。如果追踪空间已重置，文件无法自动检测其变化，
应重新标定；不能仅凭“文件加载成功”认定旧方向仍有效。

默认保存 `configs/a10/vr_calibration.local.json`，已加入 Git 忽略；文件包括两个四元数、
控制器地址、手柄侧别、单位、时间、参考姿态和质量结果。再次明确执行标定才更新，
替换前将旧文件备份为 `.json.bak`；取消、采样不合格或连接失败不会覆盖旧标定。

可以先在命令中加 `--preview` 检查参数，不会连接设备或写标定文件。其他选项：

| 选项 | 意义 |
| --- | --- |
| `--calibration-countdown 8` | 把准备倒计时改为 8 秒（允许 0～30） |
| `--calibration-duration 2` | 保持并采样至少 2 秒（允许 1～5） |
| `--pose-calibration /绝对路径/结果.json` | 指定标定输出；正常运行时同一选项用于加载该文件 |

稳定性检查：手柄相对首帧位移不超过 15mm、转角不超过 3°；机器人不超过 2mm/1°；
反馈和手柄最大接收年龄 0.25 秒。偶数帧拟合，奇数帧检查姿态重建误差不超过 3°。
这些检查验证采样一致性，**不证明固定平移方向已经符合人的习惯**，仍需下面的只读观察。
控制器本身的真实采样年龄未知，报告显式记录 `controller_sample_age_verified=false`。

### 标定后先观察：不会发送运动指令

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py --observe \
  --cfg-options \
    teleop.pose_mapping=calibrated_absolute \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0
```

选择新模式且配置中的基座/VR 四元数为空时，会自动加载本地标定文件并打印路径。
显式配置完整四元数时优先使用配置；旧 `anchor_relative` 模式不读取本地文件。
控制器地址、手柄侧别、已声明的单位或手柄/工具轴约定不一致会拒绝加载，不会自动重新标定。
如果用了自定义结果路径，观察和运行时都加 `--pose-calibration 该路径`。
观察目标写入原 JSONL 的 `reference_control.user_target`，实际反馈在 `feedback.tcp_pq_m`；
`sent_relative_target` 应为空。核对手往各方向移动时的目标变化，以及转动时的朝向是否符合预期。
该观察模式仍运行 reference，在静止反馈下可能因追踪误差而退出。检查标定优先用上方独立验证页；观察通过不能替代实机运动与停止验收。

平移倍率沿用 `pos_scale × vr_to_robot_scale`，接管时若按精调扳机，再乘 `fine_scale_factor`；
倍率在该次接管内固定，调整扳机后需松手再接管。新方式朝向为 **1:1**，不再叠加旧方式的
`angle_scale` 或 `axis_remap`，也不对新绝对目标套用旧增量死区。输入过期、重复帧、尖峰和
序号回退保护仍经过同一个 mapper；新方式消费的是校验后的缓存位姿，不直接消费未校验的原始值。

### 离线回放

`scripts/replay_vr_control.py` 新增 `--pose-mapping` 和 `--pose-calibration`。
回放既支持上面保存的完整标定结果，也支持只有上述两个字段的 JSON。
例如用**明确仅作离线演示**的单位标定：

```bash
printf '%s\n' '{"vr_to_robot_quat_xyzw":[0,0,0,1],"hand_to_tool_quat_xyzw":[0,0,0,1]}' \
  > /tmp/vr_demo_calibration.json
python scripts/replay_vr_control.py --demo \
  --pose-mapping calibrated_absolute --pose-calibration /tmp/vr_demo_calibration.json
```

这只使用合成反馈，不连接设备，不证明现场标定或实机运动效果。新日志会记录映射名、标定值及
`hand_pose_vr/position_scale`。回放新模式日志时，应提供录制时的标定。
旧日志缺少校验后的手柄位姿，不能直接按新模式解释；本轮已将旧原始输入重新经过 mapper，
作数学目标对照，报告见开发记录。

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
│   ├── calibration_view.py  # 只读标定验证：GET 反馈、计算显示目标、网页快照
│   ├── input.py / mapper.py # 手柄输入与末端增量映射
│   ├── monitor.py / ws_server.py / server_config.py
│   ├── diagnostics.py / hud.py / vr_events.py / quaternion_utils.py
│   └── web-ui/              # 输入网页、calibration.html 验证页及脚本样式
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
cd /home/robot/VLA/FluxVLA
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

反馈单位默认未填写；缺失时运行前拒绝接入真机。2026-10-19 原速度对照验证使用
单帧上限 5mm / 0.015rad，覆盖 30Hz 下 0.12m/s / 0.32rad/s 的正常步长；
它们仍是软件检查值，不代表已经完成相应速度的实机验收。
开启工控机 follower 服务，人工确认电机 Enabled、Stopped；程序不会自动使能或回零。
第一次握键选择 Tool 坐标并启动 Cartesian follower。退出请求 stop_follower，再关闭连接。
关闭连接、松手保持和退出 follower 不等于物理急停。
当前 VR 配置默认使用同步状态读取；推理配置的后台监视参数没有被改动。

### 按原 vr_vel 速度进行实机对照（2026-10-19）

ReferenceConfig 的默认值本来就是线速度 0.12m/s、角速度 0.32rad/s；之前慢是
启动命令覆盖成了 0.01 / 0.05。以下命令显式使用原速度，并采用匹配的单帧检查值。
线/角加速度仍为 0.4m/s² / 0.96rad/s²；本轮没有修改启停、平滑、停止确认或控制器。
已有轻微启停震动尚未解决，现场先小幅接管/释放，不把本次提速视为高速验收通过。

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --cfg-options \
    teleop.vr_only=False \
    teleop.control_mode=reference \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.allow_motion=True \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0 \
    teleop.reference_options.max_linear_speed_m_s=0.12 \
    teleop.reference_options.max_angular_speed_rad_s=0.32 \
    teleop.diagnostics_path=work_dirs/a10/vr/original_speed_trial.jsonl
```

`mm` 沿用当前现场反馈单位配置。勿再附加旧命令的 1mm / 0.005rad 单帧限制，
否则速度恢复后可能触发超限退出。所有反斜杠后面都不能留空格。

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
| 第 5 页：`set_jog_vel --vel_percent=60`、`set_pgm_vel --vel_percent=60` | 原生终端可设置；VR 现可显式配置 `teleop.controller_speed_percent`，默认 None 不修改；当前 `a10_control.py` shell 仍未开放这两个命令 |

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

## 双臂 VR：独立锚点、reference 与 SDK 发送（2026-10-29）

使用新配置 `configs/a10/a10_dual_arm_vr_teleop.py`，原单臂配置和用法保留。
用户已反馈 `.150` 双臂基础控制台实机验证通过；**双臂 VR 的 SDK 发送和模拟测试已完成，
VR 实机验收仍待现场进行。** 配置继承 `a10_dual_arm_operator.py`，连接 `.150:5999`、UDP 9998，
左右编号、GET 反馈行及目标行顺序须与现场设备一致，不自动识别或切换远端模型。
本次开发目录为 `/home/robot/VLA/FluxVLA_follower`、分支 `kaanh_follower`；
原 `/home/robot/VLA/FluxVLA` 的 `console` 分支及其未提交修改不动。

### 操作逻辑

- 一个 VR 浏览器和一个服务同时接收两只手的数据；左右手各自检查输入时间和有效性。
- 按 `.150` 现场反馈，默认 `hand_to_arm=dict(left='right', right='left')`：左手接管 ee 1，
  右手接管 ee 0。operator 原分组键仍是 `left=ee0`、`right=ee1`，用于协议和反馈索引，
  不等同于现场物理左右；此处仅交换 VR 绑定，不交换 GET 或 UDP 行顺序。
  前扳机只在按住侧键时调整精细比例，不单独接管。
- 2026-11-02 撤掉左手专用平移矩阵，左右手都使用公共 `teleop.input.mapping.axis_remap`：
  `[[0,1,0],[1,0,0],[0,0,-1]]`。仅恢复共同基线，不改变左右手绑定或旋转算法。
  原 `[[0,0,-1],[1,0,0],[0,1,0]]` 是未通过实机方向验证的局部轴交换尝试，已不默认使用。
  A/B只读对照确认两套配置均生效，但局部轴交换不等于现场前后/上下交换。
  当前仍依次使用手柄锚点逆旋转、公共映射、末端锚点旋转，不能把恢复公共矩阵视为现场方向已修复。
  原启动命令无需增加参数；退出旧进程并重新启动后生效，后续由现场验证方向。
- 每次接管以当前手柄和该臂的实际位姿记录锚点，平移、旋转均使用旧相对映射；不加载绝对姿态标定。
- 每臂独立计算 reference、命令速度与跟踪误差；底层 IK、运动规划和电机控制仍在控制器。
- 默认使用 `sdk_tool_cumulative`：每次启动整个 follower 后，以两臂实际位姿调用一次
  `begin_sdk_tool_trajectory()`；随后 `send_sdk_tool_pose()` 接收两臂完整的计划位姿。
  operator 把相邻计划位姿变为各臂工具系小步，累计成 SDK 要求的 xyz/Euler321 通道。
- 两条臂的目标全部检查通过后，按 `cartesian_target_order=['left', 'right']` 发一个完整 UDP 包：
  `type='321', pe=[[ee0的6项], [ee1的6项]]`。两行对应 operator 键 `left`、`right`，不拼成一行12项。
  单手重新接管只重建该侧 VR 锚点，**不重置 SDK 累计值，也不重启整个 follower**。
- `legacy_anchor` 旧发送方式仍保留为显式选项；新双臂配置默认走上述 SDK 适配，不影响原单臂用法。

### 松手和重新接管

| 操作 | 行为 |
| --- | --- |
| 只按一侧侧键 | 该臂跟随，另一臂保持启动位置 |
| 两侧均按住 | 两条臂独立跟随 |
| 松开一侧 | 丢弃该臂旧的手柄远目标，命令速度逐步降为零，保持停止过渡的终点；另一臂继续 |
| 该臂保持后重新按侧键 | 以实际位姿重取锚点，同时保留计划与实际的固定偏差，保持累计指令连续；不重启整个 follower，不重置另一侧 |
| 在减速阶段提前按回侧键 | 有效新鲜输入可直接重新接管，沿当前命令平滑继续；不用额外再松一次 |
| 两侧均释放 | 当前默认结束原生follower并确认停止；正常确认后新的侧键按下可直接接管，不要求二次松手 |
| 跟踪位置/姿态误差超过参考值 | 当前默认只记录，不停、不锁；可配置恢复超差停止 |
| 活动输入断流或控制器故障 | 暂停并请求停止，按故障恢复流程重新接管；不属于正常松手流程 |
| 通信失败、停止未确认 | 当前默认保留VR界面、暂停新目标，现场核对状态后退出重连 |

“保持”仍持续发送该臂停止终点对应的累计值，不能漏掉这一行、清零，或用最后的手柄远目标代替。
前端停止过渡只限制**发送目标的速度/加速度**，不保证机械臂物理制动效果；本轮未验证原生
follower 的内部滤波、插补、断流或制动行为，没有新增单臂 stop 后端协议。

双臂初次试验单独配置 reference 限速：`0.04 m/s`、`0.20 rad/s`，加速度 `0.4 m/s²`、`0.96 rad/s²`，
参考超前范围 `0.02 m`、`0.12 rad`；这些是前端目标的限制，不是关节实测速度或碰撞距离。
单臂配置及用户已有单臂调参保持原样。之后调整双臂参数请改 `a10_dual_arm_vr_teleop.py`。
当前双臂开发配置关闭跟踪误差退出，命令速度为零并稳定`release_settle_s`（默认0.1秒）后进入保持，
减速期间也允许新鲜的侧键重按直接接管。实际反馈速度和位姿残差继续记录，不会清零或自动补偿。
旧严格选项仍可检查实际反馈稳定性及跟踪误差；固定单步距离/角度检查已不用于reference模式。
旧的 `release_position_tolerance_m` / `release_rotation_tolerance_rad` 为配置兼容保留，
不再作为双臂松手到位条件。这是客户端反馈稳定性判断，不代表底层电机已确认停止；
只有全组停止收到 inactive + Stopped 才记录控制器停止确认。

重新接管记录实际末端与手柄锚点，同时记录上一计划末端相对实际末端的固定位置/姿态偏差，
把新手柄目标连续衔接到原计划上；偏差只在接管时记录，不按帧积分，不重置 SDK 累计通道。
日志 `user_target` 表示锚点映射目标，`tracking_target` 表示保留连续偏差后的规划目标。
这样静止握住不会自动追赶旧终点；它并未解决累计指令与实际终点偏差的底层原因。
正常单侧松手过程中握回无需额外释放；两手均释放并确认停止后，下一段接管会重新启动SDK并取实际起点。

历史记录（以下为旧停止判定，已被上述流程替代）：2026-10-29 16:41 实机日志的 `Dual VR left: release_settle_timeout`：这里的 `left` 是 ee 0 分组。
松手后前端目标速度已降为零，但最终位置误差2.9367mm，高于停止到位容差2mm；
姿态误差约0.4446°，低于姿态容差0.02rad（约1.15°）。最后一秒30帧的反馈位置和目标位置均不变，
仍未满足到位条件，超过2秒后请求全组停止，日志确认 inactive + Stopped。
此错误说明停止到位检查超时，不等同于断连或控制器故障；现有日志不足以确定位置残差来源，
还需区分SDK几何叠加、下游滤波/死区或反馈语义等因素。本次只修正手柄绑定，不放宽停止保护。

### 命令与日志

```bash
cd /home/robot/VLA/FluxVLA_follower
conda activate fluxvla

# 默认只读取双手 VR，不连接机器人、不生成虚构的机器人反馈。
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py

# 仅构造配置，不启动网页或连接设备。
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py --preview

# 只读反馈并计算目标，不启动或停止 follower，不发送 UDP。
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py --observe
```

机械臂已由现场人员使能且停止后，先退出占用 `.150:5999` 的控制台或其他客户端，再执行：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.operator.allow_motion=True
```

仍打开终端打印的**本机** HTTPS 地址，不在头显中填写控制器 IP 或 `0.0.0.0`。
两个工作目录可以并行开发，但不能同时占用同一控制入口或向同一机械臂发送运动。
示教器原5867入口保留；本机8442/8443网页端口同样一次只运行一个 VR 服务。
双臂模式禁用旧录制摇杆事件、夹爪输出和摇杆重置；用 `Ctrl+C` 结束会话。
`--observe` 可只读计算双臂目标，无 UDP 或启停命令；静止设备上的移动目标可能触发跟踪误差。
本轮不开放双臂标定、采集、夹爪、模型推理、双臂关节轨迹或碰撞避障。

诊断追加到本开发目录下 `work_dirs/a10/vr/dual_teleop_diagnostics.jsonl`：`hands` 保存左右 mapper 诊断，
`raw_input` 分左右手，`mapped_action`、`feedback.arms`、`reference_control.arms` 和
`sdk_tool_command.arms` 按机械臂名区分；每臂有 `cumulative_pe321` 与 `planned_pose_m`。
SDK 路径的 `sent_relative_target` 为空，避免把累计通道误读为相对四元数位姿。
`local_send_success` 只表示本机发送成功，`execution_confirmed=false` 不表示确认到位。
`reengage_requires_release`为历史兼容字段，当前正常松手再握保持False，不再设置二次松手锁。
`hands.left/right.position_axis_remap` 与 `rotation_axis_remap` 分别记录每只手实际使用的平移、旋转映射。
VR-only 中反馈和发送目标为空；该日志不是训练数据集，也不能证明实际机械臂已经停止。


## SDK Tool 累计指令适配（单臂与双臂）

单臂基础配置的 `teleop.cartesian_adapter` 默认仍为 `legacy_anchor`；双臂配置默认改用
`sdk_tool_cumulative`。此适配仅用于 `control_mode=reference`、operator 的
`cartesian_wire_format=pe321`。单臂支持 `anchor_relative` 和 `calibrated_absolute`；
双臂仅支持 `anchor_relative`，尚不开放双臂标定。delta 不支持此适配。

上层锚点、标定、reference、速度/加速度和跟踪保护保持原设计。新接口把相邻平滑 command
位姿的差转换为上一计划工具系的小步运动，再累计为 SDK 的 xyz/Euler321 通道。
它是几何前馈适配，不把反馈误差逐帧积分；下游滤波、规划时差与丢包仍可能导致残差。

`A10Operator.begin_sdk_tool_trajectory(startup_pose)` 只能在本会话新启动、确认健康的 Tool
Cartesian follower 且没有发送旧路径数据时调用。`send_sdk_tool_pose()` 接收机器人基坐标系
中的米/xyzw 计划位姿，校验小步限值后发 `type=321, pe=[[...]]`。
双臂时这两个方法分别接收完整的 `{left: 位姿, right: 位姿}` 字典，输出两行6项；
双臂初始化仅发生在全组启动时，单侧重新接管不会重置累计状态。
累计状态在本地发送成功后提交；成功仅表示本机发送，不表示控制器执行或 UDP 已送达。
发送异常后废弃适配状态并撤销可发送资格，不自动重放。重复目标保持累计值；停止、关闭及
重新启动会废弃旧状态；同一会话不能混用原 `send_eepose()`。

新日志字段：`cartesian_adapter`、`sdk_tool_command.cumulative_pe321`（前三项米、后三项
Euler321 弧度累计通道）、`planned_pose_m`、`local_send_success`、`execution_confirmed=false`。
新路径不把累计六维通道伪装成 `sent_relative_target` 四元数。观察模式仍只计算目标，无适配发送。

运行命令见 [命令速查](a10_command_cheatsheet.md)。回放工具：

```bash
python test/test_a10_sdk_adapter.py \
  --replay-log work_dirs/a10/vr/teleop_diagnostics.jsonl \
  --report work_dirs/a10/vr/sdk_tool_adapter_replay.json
```

此工具选取最近完成的旧发送方式绝对朝向实机会话，以历史 command 不变为前提，比较新旧
累计指令在 SDK 几何叠加模型中的终点。不是完整滤波/规划复现，不重跑真实反馈闭环，
不能将回放误差解释为实机控制精度。本轮尚未进行新适配的实机验收。

## .150 控制器高频记录（显式开启）

`python scripts/teleoperate.py --config … --record-controller …` 开启控制器记录。
开关默认关闭，不会因单独连接或原有遥操作命令自动创建远端文件。要求本地诊断路径有效，
使用实机模式或 `--observe`，且直连记录器所在的5999，不使用共享代理或第二路后台状态连接。
`--preview` 只构造配置，不建连接；VR-only不能开启控制器记录。标定、验证网页和
`--inspect-controller` 不接受此开关。

连接后，在倍率设置、运动启动之前发送 `record_start`，检查返回成功且
`recording=true`、`closing=false`、session和文件非空、error为空；失败中止启动。
`A10Operator.start_runtime_recording()`、`get_runtime_recording_status()`、
`heartbeat_runtime_recording()` 与 `get` 使用同一事务锁、原ARIS消息头和原WebSocket。
这三个接口不需要运动权限，也不发送运动指令；远端决定是否接受该端口的记录命令。

runtime在控制循环读入手柄前每秒检查心跳，空闲和单次跟随停止时也继续。
现有GET也能续期，但仍定期确认记录状态。当前要求服务端心跳超时大于2000ms，
现场默认5000ms；过短配置在启动时拒绝。心跳在同一线程串行收发，若通信阻塞，
原控制间隔/输入时效保护照常生效。录制中断、错误、会话改变或通信失败时抛错，
实机走既有全组停止流程，不自动重连/续录；`dropped` 增加会告警，继续标记记录缺口。
这是低频监督，不能保证在远端停止记录的同一周期就停机。

开启后本机每帧增加 `controller_recording`、`client_monotonic_ns`、`client_wall_ns`；
记录状态事件含session、控制器文件路径、最近状态、请求前后本机时钟。
`last_synced_cycle` 只表示最近已同步周期，并非该命令精确执行周期。
两台主机的单调时钟不能直接相减；墙钟、session及时间窗口只能辅助关联，
当前没有跨主机4ms精确同步，也没有新增控制器UDP接收事件。

退出先处理机械臂停止和倍率恢复，再断开原连接，由远端排空缓冲并保存；没有
`record_stop`命令。本机不另连5999追查落盘完成，最终状态明确为
`disconnected_save_unconfirmed`，须在控制器检查 `.rlog` 的计数、有效性和结束标记。
控制器记录默认250Hz，实际频率取决于其master采样周期。此记录是运行诊断，不是训练数据集。

首轮无运动验证和原命令的开关位置见[命令速查⑧](a10_command_cheatsheet.md)。


## 2026-11-06：双臂知行夹爪接入

双臂配置增加可选的teleop.zx_grippers，默认关闭；开启方式及完整命令见
[a10_command_cheatsheet.md](a10_command_cheatsheet.md)。
设备范围按用户确认登记为0..9000，0张开、9000闭合；物理左右手柄分别控制对应USB串口，
不使用机械臂历史left/right键作为夹爪地址。摇杆独立于机械臂侧键，前扳机继续用于机械臂精调。

VROperator保留每手摇杆X输入；DualVRTeleopRuntime内各有一个独立串口线程，
启动只读位置、要求摇杆回中，随后做速度积分、工作范围限制和相对实际位置的超前限制。
前台不等待串口。使用ZXGripper的Modbus协议，新增可选USB身份校验、单调时钟截止时间及取消标记，
在运动寄存器写入前检查，避免迟到输入继续触发。过期后需回中，不自动重试或复位。

输入过期仅停止目标更新；串口故障/报警/反馈失效终止会话并请求机械臂停止。
清理先取消所有夹爪发送，再处理原机械臂停止和资源释放；夹爪串口只由所属线程关闭，不自动掉使能。
VR-only、observe和preview不打开知行串口。gripper_command诊断按物理手柄命名。
软件测试通过，现场联动验收待用户执行，不将软件发送成功当作真实抓取成功。

2026-11-06用户已实机确认夹爪开合手感：左右电机speed=15%、target_rate_raw_s=3000、
max_lead_raw=600，现设为a10_dual_arm_vr_teleop.py默认值。VR仍需显式开启zx_grippers；
共用设备清单保留speed=5%供设备检查/标定使用。该反馈确认开合体验，不代表已完成数据录制或抓取任务验收。


## 2026-11-06：双臂VR采集入口

已接入实时录制、右手A+左摇杆四方向、HUD、后台保存和可取消m_init；普通遥操作命令不启用录制时保持原行为。先用只读按钮模式确认实体按键，再由现场操作者验收实机短集。完整命令及状态解释见[双臂VR数据采集指南](a10_vr_recording.md)。本轮仅完成离线验证，不代表实机已验收。

### 2026-11-11：参考遥操作按时间限速，取消固定单帧超限退出

- 单臂和双臂 `control_mode=reference` 继续由 `reference_options` 的最大线/角速度与加减速生成目标，取消5毫米、0.025弧度等固定每帧步长退出条件；SDK工具累计格式转换同样使用无固定步长模式。既有CLI中的 `teleop.max_position_step_m` / `teleop.max_rotation_step_rad` 在reference模式下不再生效，可删除，delta模式仍使用它们。
- 最大速度限制使用实际dt，正常40～60毫秒周期不再因为步长较大而退出。输入/反馈过期、超过max_dt_s、跟踪误差、控制器故障及停止确认逻辑继续执行；SDK格式转换仍检查位姿有效性和旋转表示的适用范围。控制台及其他显式传入SDK步长限制的调用保持原有行为。
- 当前换向诊断：16:33:22会话中，以期望速度与上一帧发送速度夹角至少45度、两者速度超过0.02m/s定义明显换向，28个采样段均接近前端0.4m/s²加速度上限（按反馈时间计算，容差±0.02）。包含参考进度未被软预算限制的样本，能确认前端限加速度本身造成部分迟缓；这些计数不是28次独立操作。
- 0.150 XML只读核对：笛卡尔FollowerCartParam仅关联tg_id=2及UDP9998，未显式配置单独的末端加速度；MotorConfig的关节加速度上限为17.45329252rad/s²，不能与前端0.4m/s²直接比较。SDK内部笛卡尔规划未据此得到完全确认，不声称后端没有延迟。
- 当前14cm/s平移上限与0.4m/s²加速度对应全速反向至少约0.7秒；0.7rad/s与1.5rad/s²对应全速反向至少约0.93秒。可在现场停止/通信确认后单独对照平移加速度0.6或0.8，最大速度保持一致；本轮未改动加速度、控制器XML或启动任何实机运动。


### 2026-11-11：双臂VR现场调参设为默认值

`configs/a10/a10_dual_arm_vr_teleop.py` 已按用户指定固化本轮参数：控制器倍率70%，
最大线/角速度0.17m/s、1.1rad/s，最大线/角加速度1.0m/s²、3.0rad/s²，
平移/旋转跟随增益10、12，reference软领先预算0.03m、0.1rad（约5.7°）。
本次仅保存参数，不代表新增的0.1rad预算已完成实机验收；停止查询超时另行处理。

实机遥操作并记录控制器状态可缩短为（需现场正常使能）：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --record-controller \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.operator.allow_motion=True
```

配置仍默认VR-only、禁止运动，知行夹爪仍需单独开启；上述命令只省略已固化的运动参数。
命令行 `--cfg-options` 可覆盖这些默认值，单臂配置不继承本轮双臂调参。

### 2026-11-11：停止后查询超时与端口排查

- 现场只读测试：普通GET共410次全部成功，其中30Hz连续300次，最大约17.14ms；已停止状态下重复50轮VR停止确认查询也全部成功。这不代表跟随停止过渡已验证。
- 示教器使用5867，FluxVLA使用5999。5999存在连接时，新客户端立即Connection refused，原客户端GET继续成功；5888和5999的两条只读连接可同时GET。因此现有“已连接、停止后GET无回复”与本次验证的同端口抢连表现不同，不能据此断言历史故障期间没有其他客户端。
- 新增失败诊断：通信失败时，终端输出`A10 WS failure trace`，VR的原JSONL同时保存`communication_failure`。记录连接地址、失败命令与最近32条发送/接收摘要及时间戳；正常运行不逐条打印。
- `sent`仅表示本机发送完成；`empty_ack`不带命令标识，不能单凭它认定stop_follower已执行。用它结合紧邻的发送顺序、控制器日志区分“未收到任何回复”和“收到空回执但没有状态回复”。本轮未更改超时、重试、停止或运动参数，也未发送实机启停和运动指令。

### 2026-11-11：双臂旋转跟手与测试会话保留

双臂默认配置改为以下行为，启动命令保持不变：

- **旋转按手部增量驱动**：`rotation_follow_mode='incremental_velocity'`。每个有效手柄采样只使用一次，手停止时前端角速度按制动加速度归零，不补追之前积压的绝对姿态。平移仍沿用锚点闭环流程。
- 最大角速度`1.4rad/s`、角加速度`6rad/s²`、制动角加速度`10rad/s²`，微小输入速度死区`0.03rad/s`。同日后续调整已关闭软领先预算，见下一节。这些是前端目标约束，不是修改控制器或电机参数。
- **双手松开结束原生跟随**：当前`stop_on_both_release=True`。单侧松手仍不影响另一臂；两手均松开则请求stop_follower并确认停止，正常确认后新按键可直接接管。早先持续holding的默认已因现场晚执行事件撤回。
- **查询失败不关闭VR界面**：`keep_session_on_query_error=True`。运行期间GET/记录心跳发生通信异常时，暂停新目标、停止配件操作并保留界面和诊断；不自动重连、重发或使用过期反馈继续控制。若停止未确认，需现场确认停止，再Ctrl+C退出并重新连接。启动连接失败、控制器故障与非法目标不属于可忽略查询异常。

新模式会丢弃限速期间未执行的部分旋转，手柄转90°不保证机器人随后补足90°。
需要原来精确锚点姿态追赶时，可覆盖`teleop.reference_options.rotation_follow_mode=anchor`；
`rotation_gain`只影响该旧模式，新模式主要由手柄旋转增量、速度/加速度及领先预算决定。
新模式减少的是前端积压，无法取消SDK已接收目标的物理滞后；不能宣称机械臂与手瞬间同时停止。
日志增加`rotation_input_base_rad_s`、`rotation_input_stationary`、`rotation_backlog_discarded_rad`及`communication_pause`。
修改后必须退出旧进程并重新启动；启动应看到`双臂旋转模式=incremental_velocity`与角速度1.4、加速度6、制动10。
18:25那次`command_tracking_error`仍为旧anchor模式，角度差0.3523rad超0.35，停止已确认；同日后续按用户要求改成仅记录误差，见下一节。

### 2026-11-11 后续调整：双臂开发默认仅约束目标速度和加速度

按用户要求，当前双臂配置进一步改为：

```python
enforce_tracking_error_limits=False
max_reference_lead_m=None
max_reference_lead_rad=None
```

位置/姿态跟踪误差继续真实记录并告警，不再因超过原5cm/0.35rad而退出；
关闭两种软领先减速。松手状态以**前端目标已减速归零**判定保持，不再等待实际末端到位而超时。
这不代表实机已停止，`feedback_quiet`仍记录实际反馈是否安静。
最大平移速度0.17、角速度1.4、线加速度1、角加速度6和角制动10仍有效。
启动日志应显示`跟踪误差阈值=仅记录`。原来的软领先CLI参数会重新启用对应软限制，
因此本轮直接使用上方简短命令，不再附加旧的lead参数。

原有单臂及类默认仍检查误差；双臂可设置`enforce_tracking_error_limits=True`恢复超差停止。
控制器故障、非法位姿、输入/反馈过期、控制间隔异常与通信失败处理仍保留。
SDK和电机自身保护不变；移除前端误差门槛不能让控制器实际速度自动提升，也无法消除其已接收目标的响应滞后。

### 2026-11-11：正常松手再握直接接管

18:34会话中，18:36:32松开约67ms后重新按右侧键，旧流程将该臂设置为
`reengage_requires_release=true`，即使已进入holding也要求再松一次；跟踪误差提示本身没有锁定。
现移除这处正常操作锁：按下控制、松开减速、减速中再按也可继续。
重新接管保留当前命令位姿/速度、SDK累计通道与另一臂控制，不重启follower、不跳回实际位姿。
跟踪误差提示改为INFO并只提示一次；输入/通信/控制器故障恢复仍独立处理。

### 2026-11-11：无输入晚运动事件与断流修复

本轮18:44:48.167后VR不再更新，双手已松开；前端仍在holding、持续发送旧的非零累计目标。
控制器高频记录显示18:45:34.818—36.362实际轴再次运动约1.54秒，峰值约1.07rad/s。
这不是前端仍在减速；SDK为何隔几十秒重新产生目标、是否有其他控制来源，现有记录尚不能确定。

已修复前端缺口：即使双手均未按下/处于holding，输入流超过原有新鲜度期限也停止发送旧包并请求原生停止。
双臂默认恢复`stop_on_both_release=True`，不把持续发送保持包当作物理停止；
正常停止确认后仍可直接重新按键，不增加二次松手锁。故障未确认停止时不会自动重启。
本轮只做离线/本地回环验证，不发送真实机器人指令。现场已急停，SDK晚执行原因未确定前不据此宣称可恢复实机运行。
