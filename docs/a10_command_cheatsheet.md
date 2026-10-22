# A10 常用命令速查

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
cd /home/robot/VLA/FluxVLA_vr
conda activate fluxvla
```

当前工作分支：`kaanh_follower_vr`。下面命令都在这个目录执行。

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

## ⑥ 双臂基础状态查询：控制器部署双臂后使用

```bash
python scripts/a10_control.py \
  --config configs/a10/a10_dual_arm_operator.py \
  --read-only get
```

模板中左右编号、反馈位置和单位须按实际双臂配置核对。**0.125 当前运行的是单臂 XML**，
此时用双臂模板查询会报告反馈组不匹配，这是预期保护，不会自动切换控制器配置。
想连续查询可将末尾 `get` 换为 `shell`，输入 `get`、`joints`，退出输入 `exit`。
模板默认禁止运动；控制台启停作用于整个控制组。双臂 VR 独立接管见⑦，尚未实机验收；
录制、标定、模型推理、双臂关节和夹爪未开放。⑤仍是原单臂 VR 命令。

## ⑦ 双臂 VR：左手控左臂，右手控右臂

先看双手输入，不连接机器人：

```bash
python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py
```

头显打开终端打印的网址；按侧键观察终端与双手日志。仅检查配置可加 `--preview`。
日志：`work_dirs/a10/vr/dual_teleop_diagnostics.jsonl`，不等于数据集录制。

**以下实机命令仅在团队确认双臂 XML、左右编号与反馈布局后使用，不能用于现有单臂控制器。**

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --cfg-options \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.operator.host=192.168.0.125 \
    teleop.operator.allow_motion=True
```

- 左右侧键分别接管对应臂；松开一侧，该臂减速保持，另一臂继续。
- 减速尚未完成就按回侧键，需要等保持后松开一次，再重新按住。
- 两手释放并完成停止过渡后，全组退出 follower；故障也请求全组停止。
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

- 网页、头显和控制器地址是不同的：网页用**本机局域网 IP**，控制器用 `192.168.0.125`。
- 一次运行一个 VR 命令；原终端保持打开。换命令先按 `Ctrl+C` 退出，避免网页端口被占用。
- ①～④不下发运动；⑤及⑦的实机命令会下发运动，退出会请求停止跟随，应确认控制器已停止。**`Ctrl+C` 不是机械臂急停**。
- 新绝对姿态标定、奇异点及停止过程仍在核实；⑤沿用旧相对映射，不代表这些问题已修复。
- 详细功能与历史命令见 [A10 VR 使用说明](a10_vr_operator.md)；人工使能、跟随和关节控制另见 [现场操作说明](a10_onsite_guide.md)。
