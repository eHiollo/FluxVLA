# A10 常用命令速查

**现在检查标定方向，选第 ① 条。** 其他命令按用途查，不需要每条都执行。

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

## 少量常用参数

| 参数 | 通俗解释 |
| --- | --- |
| `--config …` | 选用哪份配置；上面的命令均使用 A10 VR 配置 |
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
- `Ctrl+C` 关闭这里的只读程序；**不能把它当作机械臂急停**。
- 当前标定方向、奇异点及停止过程还在核实，因此本速查不把实机运动命令作为默认操作。
- 详细功能与历史命令见 [A10 VR 使用说明](a10_vr_operator.md)；人工使能、跟随和关节控制另见 [现场操作说明](a10_onsite_guide.md)。
