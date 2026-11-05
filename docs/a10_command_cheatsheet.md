# A10 常用命令速查

2026-10-26：VR、双臂控制与推理已整合到 `FluxVLA` 的 `kaanh_follower`。原 VR 目录保留作回退。
无 `--config` 的控制台沿用默认共享连接；双臂配置默认直连。不要同时启动推理与 VR 驱动同一机器人。

## 旧单臂 .125 + 老夹爪 VR（2026-10-26）

在 `/home/robot/VLA/FluxVLA`、`kaanh_follower` 分支使用。
老夹爪走本机 USB 串口，机械臂仍连接 `192.168.0.125`。
右手摇杆 X 轴控制开合，不要求按住机械臂侧键。知行夹爪用另一套协议，本入口不使用它。

你现有的单臂 VR 命令，只需把配置名改为：

```text
configs/a10/a10_vr_teleop_legacy_gripper.py
```

完整命令（控制器服务应已运行，关闭占用同一夹爪串口的推理/夹爪程序）：

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop_legacy_gripper.py \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.control_mode=reference \
    teleop.cartesian_adapter=sdk_tool_cumulative \
    teleop.pose_mapping=anchor_relative \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.allow_motion=True \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0 \
    teleop.controller_speed_percent=50 \
    teleop.reference_options.max_linear_speed_m_s=0.08 \
    teleop.reference_options.max_angular_speed_rad_s=0.4 \
    teleop.reference_options.max_reference_lead_m=0.03 \
    teleop.reference_options.max_reference_lead_rad=0.20
```

可在 `--config` 后加 `--preview` 检查配置，不连接机器人或串口。
仅验证夹爪时，保持右手侧键和前扳机都松开，只拨右手摇杆；这时仍会连接并查询机械臂。
不要用该方式代替硬件隔离：若按下侧键或前扳机，机械臂也会进入原有接管流程。

| 操作 / 参数 | 效果 |
| --- | --- |
| 右手摇杆 X 正方向 | 增大开口（张开） |
| 右手摇杆 X 负方向 | 减小开口（闭合） |
| 摇杆回中 | 停止更新开合目标，夹爪可能继续完成最后一小步 |
| `teleop.gripper_direction=-1` | 反转摇杆开合方向 |
| `teleop.gripper_speed_mm_s=10` | 默认最大开口目标变化速度10毫米/秒，摇杆幅度越小越慢 |
| `teleop.operator.gripper.port=实际串口` | 现场USB设备路径变化时覆盖 |

专用配置默认：旧100mm型号的标定、开口限制0..85mm、波特率1000000、舵机ID10、
舵机原始速度参数800、串口超时0.1秒。舵机速度800与摇杆目标变化速度10mm/s含义不同。
串口沿用推理配置的 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，不用知行夹爪的FTDI串口。

连接仅执行ping和读状态；初次输入不补发历史位移。输入过期时不再下发新目标，
串口读期间输入变旧也丢弃该帧目标。退出不会自动张开或掉使能；断流不是立即取消已发出的舵机目标。
夹爪通信异常时结束遥操作并请求停止本次机械臂跟随，停止失败会明确报错。
正常退出可用左手摇杆向下或 Ctrl+C。默认VR-only仍不打开任何设备。

`work_dirs/a10/vr/teleop_diagnostics.jsonl` 的 `gripper_command` 记录实际反馈开口、
发送目标和 `io_elapsed_s`（秒）；`transmitted` 不代表夹爪已经到位。
本轮现场只读检查成功：ping/初次反馈约16.2ms，后续3次读位置约2.3～2.4ms，未发送运动目标。
因此先保留现有同步读写；遇到现场明显卡顿时结合该耗时与 `step_elapsed_s` 再判断是否需要后台线程。
**软件与模拟串口已验证，真实VR开合方向和实际抓取待现场确认。**

## 双臂控制台：先不用 VR（2026-10-23）

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
# 先只读查看新控制器；需要现场控制服务已运行、5867入口可用
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --read-only get --raw
# 核对左右映射、单位及工具轴后，打开控制台（连接本身不会使能）
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --allow-motion shell
```

在 `a10>` 中逐条输入：`manual_en` → `set_jog_coordinate --tool` → `follower_cart`。
之后按需选择一个：

| 指令 | 用途 |
| --- | --- |
| `get` / `joints` | 查看双臂状态 / 角度 |
| `jog left x 3` | left 工具 X 方向移动3毫米，right保持 |
| `jog right rz 2` | right 工具 Z 轴旋转2度，left保持 |
| `jog left x 3 right x 3` | 两臂各沿自身工具 X 移动3毫米 |
| `stop_follower` → `exit` | 确认整个控制组停止后退出 |

`left=ee0`、`right=ee1` 仍须现场确认物理对应；平移方向不是基坐标轴。
单次限制10毫米/5度。正常到位后可连续输入 jog，不会清零累计量；完成验证后停止 follower。
**离线已验证，实机待验收；本轮SSH查看时控制服务未运行。** 单臂旧命令继续使用原配置。
完整步骤、容差和异常处理见 [现场说明：双臂最小验证](a10_onsite_guide.md)。


**用旧锚点算法调试实机，选第 ⑤ 条；检查标定方向，选第 ① 条。** 不需要每条都执行。

## SDK 工具坐标适配试验（2026-10-22）

只用于单臂 `reference` 模式；两种 `pose_mapping` 都可使用。默认仍为旧发送方式。
新适配已完成离线测试，尚未实机验收；它修正指令坐标语义，没有添加实际误差积分补偿。

在当前实机命令的 `--cfg-options` 参数中添加这一行即可：

```bash
teleop.cartesian_adapter=sdk_tool_cumulative
```

完整命令（先用 `--preview` 检查配置；现场准备好后去掉该参数启动）：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py --preview \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.control_mode=reference \
    teleop.pose_mapping=calibrated_absolute \
    teleop.cartesian_adapter=sdk_tool_cumulative \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.allow_motion=True \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0 \
    teleop.controller_speed_percent=50 \
    teleop.reference_options.max_linear_speed_m_s=0.06 \
    teleop.reference_options.max_angular_speed_rad_s=0.32 \
    teleop.reference_options.max_reference_lead_m=0.03 \
    teleop.reference_options.max_reference_lead_rad=0.20
```

- 使用旧锚点映射：只把 `teleop.pose_mapping` 改为 `anchor_relative`。
- 回退旧发送方式：停掉当前进程，改为 `teleop.cartesian_adapter=legacy_anchor` 后重新启动。
- 正常完成控制并确认停止后才能换路径；不在同一次 follower 中混用。
- 实机先验证单轴、再验证组合旋转和平移；若目标停住仍持续偏离，不提高阈值掩盖偏差。

## 先进入正确目录

每次新开终端先执行：

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
```

当前工作分支：`kaanh_follower`。下面命令都在这个目录执行。

## 我应该选哪个？

| 想做什么 | 选哪个参数 | 是否连接机器人 | 是否下发运动 |
| --- | --- | --- | --- |
| **① 看手柄与夹爪的方向是否对应** | **`--verify-calibration`** | 只读状态 | 否 |
| ② 只确认 VR 能连接、按键和手柄数据正常 | `teleop.vr_only=True` | 否 | 否 |
| ③ 只检查配置能否加载 | `--preview` | 否 | 否 |
| ④ 只检查机器人状态和通信 | `--inspect-controller` | 只读状态 | 否 |
| **⑤ 用旧锚点相对算法遥操作实机** | **`control_mode=reference` + `pose_mapping=anchor_relative`** | 是 | **是** |
| ⑥ 查询双臂控制器的基础状态 | `a10_control.py --config … --read-only get` | 只读状态 | 否 |
| ⑦ 双臂 VR 独立接管/释放 | `a10_dual_arm_vr_teleop.py` | 默认不连接；实机需显式开启 | 默认否 |
| 重新记录一份标定 | `--calibrate-pose` | 只读状态 | 否；会保存标定文件 |
| 调试 reference 的计算过程（旧观察入口） | `--observe` | 只读状态 | 否；静止时可能触发追踪误差 |

`--verify-calibration`、`--inspect-controller`、`--observe`、`--calibrate-pose` **一次只选一个**。`--preview` 可以与其中一个搭配。

## ① 只读标定验证：当前优先使用

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --verify-calibration \
  --cfg-options \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0
```

- **头显**打开终端打印的 VR 地址；**电脑**打开打印的 `/calibration.html` 地址。
- 灰色是实际 TCP 朝向，蓝色是当前标定算出的目标朝向；机械臂不动，你转动手柄看对应关系。
- 首先核对界面里“夹爪前方”和“夹爪上方”的轴选择，再做左右、上下和翻转检查。
- 默认加载 `configs/a10/vr_calibration.local.json`，不覆盖它；界面可导出检查报告。
- 机械臂应已停止、跟随器未运行、无故障。此命令不会替你停机或使能。

**本次更新后，旧 VR 输入页需要刷新，才能提供新的追踪和射线数据。** 刷新可能改变追踪坐标，旧标定先用本页面核对，不直接用于实机运动。

## ② 只看 VR 输入：不连接机器人

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --cfg-options \
    teleop.vr_only=True \
    teleop.input.network.host_ip=0.0.0.0
```

头显打开终端打印的网址，检查连接、侧键、扳机和摇杆输入。
**这不是完整数据集录制，也不会验证机器人反馈或实际运动。**

## ③ 只检查配置：不启动任何设备或网页服务

```bash
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py --preview
```

若想检查第 ① 条的配置，在第 ① 条命令中加 `--preview` 即可。

## ④ 只查询机器人：不用连接 VR

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --inspect-controller \
  --cfg-options teleop.operator.host=192.168.0.125
```

默认读取 10 次状态，输出结果后退出；不会启动 VR 服务或运动。
需要保存检查结果时，加 `--report work_dirs/a10/vr/controller_check.json`。

## ⑤ 旧锚点相对算法：VR 控制实机

控制器 follower 服务应已启动，电机为 Enabled、机械臂为 Stopped。
在上面的 VR 工作目录和 `fluxvla` 环境中执行：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_vr_teleop.py \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.control_mode=reference \
    teleop.pose_mapping=anchor_relative \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.allow_motion=True \
    teleop.operator.feedback_position_unit=mm \
    teleop.input.network.host_ip=0.0.0.0
```

- 头显打开终端打印的 **VR 输入网址**。从只读验证切过来时刷新输入页，确认不再显示“只读标定验证”。
- 按住右手侧键，以接管时的手柄和机械臂姿态为起点，跟随手柄的**相对平移和相对旋转**。
  不会主动对齐手柄绝对朝向；不读取本地姿态标定文件。松开侧键和前扳机后请求停止跟随。
- `reference` 保留限速/平滑，`anchor_relative` 选择旧位姿映射；仍由 FluxVLA 通过 UDP follower 下发，
  不是切回旧 C++ TCP follower，也不是最早的 `control_mode=delta` 逐帧累积路径。
- 该命令会真实控制机械臂，不能加 `--observe` 或 `--verify-calibration` 当作执行开关。
  程序不自动使能或回零；初次小幅操作，现场确认停止，切回旧映射不会修复已发现的控制器奇异点/停止问题。

### ⑤补充：单次倍率与 reference 超前范围

⑤的命令默认**不改控制器倍率**；单臂 reference 已启用 20 mm / 0.12 rad 超前范围。
需要指定倍率时，在⑤的 `--cfg-options` 参数列表末尾增加（25 只是示例，需现场选择）：

```text
teleop.controller_speed_percent=25
```

程序在启动 follower 前设置并回读倍率，退出且确认停止后恢复原倍率。断线、停止未确认、
现场改变模式或倍率时，不盲目恢复，终端会提示。只读观察和 VR-only 不修改倍率。

| 可选运行参数 | 含义 |
| --- | --- |
| `teleop.controller_speed_percent=None` | 默认保留现场倍率；1～100 的整数表示本次采用的百分比 |
| `teleop.reference_options.max_reference_lead_m=0.02` | reference 相对反馈最多正常超前约 20 mm，外侧一半范围逐渐减慢 |
| `teleop.reference_options.max_reference_lead_rad=0.12` | 姿态同理，约 6.9°；两项仅在 reference 模式生效 |

这些数值由离线回放选出，尚未实机验收；20 mm 是 reference 的正常预算，不保证最终 command 或机械臂
制动距离也在 20 mm 内。原 50 mm / 0.35 rad 异常保护保留。**本次没有改松手停止流程，也没有修复 SDK 停止语义。**
需要旧 reference 对照时，把两个 `max_reference_lead_*` 都设为 `None`。

**2026-10-22 实机速度对照**（在⑤的 `--cfg-options` 中覆盖；后者尚待实机验证）：

| 参数 | 本轮已测试，用户反馈顺滑 | 下一轮提速候选 |
| --- | ---: | ---: |
| `teleop.controller_speed_percent` | 25 | 50 |
| `teleop.reference_options.max_linear_speed_m_s` | 0.04 | 0.08 |
| `teleop.reference_options.max_angular_speed_rad_s` | 0.10 | 0.20 |

两组均保持 20 mm / 0.12 rad 超前范围。上一轮退出是 `input_stale`（VR 新姿态停更超过 250 ms），
提速不会解决输入停更，也不应放宽输入过期保护来掩盖它。

## ⑥ 双臂基础状态查询：.150 控制器

```bash
python scripts/a10_control.py \
  --config configs/a10/a10_dual_arm_operator.py \
  --read-only get
```

配置默认连接 `.150:5999`，旧 `.125` 单臂继续使用单臂配置。用户已反馈双臂基础控制台实机验证通过；
左右编号及反馈行仍须与现场设备一致，不自动切换控制器配置。
想连续查询可将末尾 `get` 换为 `shell`，输入 `get`、`joints`，退出输入 `exit`。
模板默认禁止运动；控制台启停作用于整个控制组。双臂 VR 独立接管见⑦，尚未实机验收；
双臂录制、标定、模型推理、关节轨迹和夹爪未开放。⑤仍是原单臂 VR 命令。

## ⑦ 双臂 VR：左手控左臂，右手控右臂

本次在 `kaanh_follower` 的独立目录开发，原 `FluxVLA` 目录的 `console` 分支可继续改代码：

```bash
cd /home/robot/VLA/FluxVLA_follower
conda activate fluxvla
```

先看双手输入，不连接机器人：

```bash
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py
```

头显打开终端打印的网址；按侧键观察终端与双手日志。仅检查配置可加 `--preview`。
日志：`work_dirs/a10/vr/dual_teleop_diagnostics.jsonl`，不等于数据集录制。

只读机器人反馈并计算目标，不启动跟随、不发送运动：

```bash
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py --observe
```

现场确认两臂已使能且停止后，启动实机双臂 VR。**先退出占用 `.150:5999` 的控制台或其他客户端；
两个代码目录独立，不代表同一台机器人能由两个程序同时控制。**

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.operator.allow_motion=True
```

- 左右侧键分别接管对应臂；松开一侧，该臂减速保持，另一臂继续。
- 按 `.150` 现场反馈对调绑定：左手→`ee_id=1`，右手→`ee_id=0`。日志的 operator 分组名仍为 `right`、`left`，分别对应上述两个编号。
- 2026-11-02 已撤掉左手专用平移轴交换，两手统一使用公共 `teleop.input.mapping.axis_remap`；左手仍控ID1、右手仍控ID0。原命令不变，重启后生效，现场方向仍待验证。
- 默认 `.150:5999`、UDP 9998，SDK Tool 累计指令；无需额外填写 host 或发送适配参数。
- 双臂初次试验目标限速 `0.04 m/s`、`0.20 rad/s`，角加速度 `0.96 rad/s²`；不修改单臂参数。
- 减速尚未完成就按回侧键，需要等保持后松开一次，再重新按住。
- 两手均新鲜释放后，立即请求全组停止并确认 inactive + Stopped，与单臂一致。
- 单侧松手按命令减速及反馈稳定判定保持，不再要求精确到达计划终点；严重误差、断流和长时间未稳定仍触发全组停止。
- 单侧重新握住时保留累计指令连续性，静止手柄不会自动补偿原有终点偏差。
- 不录制、不控制夹爪、不加载姿态标定；摇杆事件禁用，用 `Ctrl+C` 结束。
- 停止效果仍待实机确认。原生 follower 的制动、断流和双臂碰撞行为未在模拟测试中验收。

## 少量常用参数

| 参数 | 通俗解释 |
| --- | --- |
| `--config …` | 选用单臂或双臂配置；⑤单臂，⑦双臂 |
| `--cfg-options …` | 仅对这次运行修改配置，不改配置文件 |
| `teleop.operator.host=192.168.0.125` | 机器人控制器的地址 |
| `teleop.input.network.host_ip=0.0.0.0` | 允许头显从局域网连接本机；**头显里不填写 `0.0.0.0`** |
| `teleop.operator.feedback_position_unit=mm` | 按毫米解释控制器反馈的位置 |
| `--pose-calibration /绝对路径/标定.json` | 加载指定标定；配合 `--calibrate-pose` 时则是保存到该路径 |

## 两个旧入口，什么时候才用？

- **`--calibrate-pose`：重新采样并保存对应关系。** 默认替换本地标定，先备份旧文件；不负责证明物理方向正确。需要重做时参照[完整说明中的标定步骤](a10_vr_operator.md#一次参考姿态标定照着操作即可)。
- **`--observe`：给开发调试 reference 用。** 仍计算 reference，机器人静止时可能报 `tracking_error`。看标定方向用第 ① 条，不需要同时运行它。

## 使用时记住这几点

- 网页、头显和控制器地址是不同的：网页用**本机局域网 IP**，旧单臂控制器用 `.125`，新双臂用 `.150`。
- 一次运行一个 VR 命令；原终端保持打开。换命令先按 `Ctrl+C` 退出，避免网页端口被占用。
- ①～④不下发运动；⑤及⑦的实机命令会下发运动，退出会请求停止跟随，应确认控制器已停止。**`Ctrl+C` 不是机械臂急停**。
- 新绝对姿态标定、奇异点及停止过程仍在核实；⑤沿用旧相对映射，不代表这些问题已修复。
- 详细功能与历史命令见 [A10 VR 使用说明](a10_vr_operator.md)；人工使能、跟随和关节控制另见 [现场操作说明](a10_onsite_guide.md)。

## ⑧ .150 控制器高频状态记录

仅适用于已加载 `runtime_recorder` 的 `.150:5999`。控制器须由现场确认新库已加载；
本机不会重启控制器。一次5999连接对应一次记录，多次跟随启停仍在同一会话内。

先验证5秒记录，不启动机械臂、不要求连接VR手柄（仍会启动VR网页服务）：

```bash
cd /home/robot/VLA/FluxVLA_follower
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --observe --record-controller \
  --cfg-options \
    teleop.max_steps=150 \
    teleop.diagnostics_path=work_dirs/a10/vr/controller_recording_check.jsonl
```

`--observe` 禁止运动；额外的 `--record-controller` 明确允许创建控制器诊断文件。
出现“控制器高频记录已开始：session=…，file=…”表示确认开始；还应核对状态里的
`attempted/written/synced` 增长、`dropped` 与 `error`，才能判断数据是否实际采集及保存。
退出会断开连接，后台完成写盘；本机最终标记 `disconnected_save_unconfirmed`，不冒称已落盘。
现场可用控制器已有工具只读检查文件：

```bash
cd /home/kaanh/Desktop/kaanh-project
python3 runtime_recorder/log_tool.py inspect log/robot_state/
```

实机遥操作沿用原命令，在 `--config` 行之后、`--cfg-options` 之前加入：

```bash
  --record-controller \
```

不开此开关维持原行为。开启时确认录制成功后才允许运动；每秒原连接串行发送心跳并记录状态，
不另开连接。前端检测到录制停止、IO错误、心跳失败或会话改变时结束遥操作，并走既有停止流程。
此检测通常最多间隔约1秒，网络阻塞可能更长；原有控制间隔保护继续生效。丢样增加会告警。

控制器文件在 `/home/kaanh/Desktop/kaanh-project/log/robot_state/`，默认250Hz、12轴；
本机JSONL的 `controller_recording` 保存对应session、文件、状态和请求时间。
控制器没有 `record_stop`；断线或5秒没有应用消息才结束，WebSocket ping不能替代应用心跳。
录制失败不自动重连或重发 `record_start`。先退出占用5999的其他程序。

## 单臂训练数据检查与导出（离线，不连接设备）

本轮只完成录制核心，尚未把相机/VR录制循环接入 `teleoperate.py`。
`--record-controller` 是诊断日志，不是训练数据。接口及后续接线见 [录制核心说明](a10_recording_core.md)。

```bash
# 快速检查旧数据；去掉 --skip-videos 会进一步解码核对视频。
python scripts/validate_a10_dataset.py \
  /home/robot/VLA/lerobot/dataset/a10_vr_9_21_4 --skip-videos

# 以下要求已有录制核心生成的数据，并且录制程序已经关闭。
python scripts/validate_a10_dataset.py work_dirs/a10/datasets/a10_record
python scripts/export_a10_openpi_dataset.py \
  --source work_dirs/a10/datasets/a10_record \
  --output work_dirs/a10/datasets/a10_record_openpi
```

导出目标必须不存在，原数据不会修改；输出供当前 OpenPI 的 v2.1 reader 使用，训练前重新计算 norm_stats。
