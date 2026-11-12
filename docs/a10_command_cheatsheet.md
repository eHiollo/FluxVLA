# A10 常用命令速查

2026-11-06起统一在 `/home/robot/VLA/FluxVLA` 的 `kaanh_follower` 分支开发和运行。
旧开发工作树已归档并移除；仍停留在旧目录的终端请先执行 `cd /home/robot/VLA/FluxVLA`。
切换前已打开的控制台请退出后重开。

## 双臂控制台与 VR 同时使用（2026-11-12）

双臂默认通过本机共享服务连接 `.150`，默认开启实机、两只知行夹爪和控制器高频记录；不会自动使能机械臂。不用再手动开启第三个终端。
**控制台或 VR 都可以先开**，第二个自动复用连接。两个终端都先进入仓库并激活 `fluxvla` 环境。

终端一：

```bash
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py
```

终端二：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py
```

- 控制台输入 `get`、`joints` 只查询，不打断 VR。
- 控制台执行控制指令会先停止并接管当前跟随；VR 窗口保留，不再发送旧目标。
- `m_init` 正常到位后仍恢复原来的模式和跟随开启状态。如果它恢复了跟随，先在控制台输入 `stop_follower`；然后双手松开侧键，再按需要控制的侧键，VR 会从当前实际位姿建立新锚点。
- 关闭仅查询的控制台不会停止 VR。控制台接管后关闭 VR，也不会停止控制台的新跟随。
- 本机服务独立于两个窗口；最后一个客户端退出约 5 秒后自动释放端口。5999 为唯一机器人命令/状态连接，5870 只用于本机程序之间共享，UDP 格式与原控制器不变。
- 保留已调好的运动速度和算法。服务失败不会自动改成直连、重新使能或重发运动目标。机器人离线时控制台保留，可输入 `reconnect`；VR 启动失败后需要重新运行命令。
- 服务启动日志：`work_dirs/a10/shared_control/service.log`；所有权切换明细：`work_dirs/a10/inference_logs/*_shared_control/`。
- 本次切换前退出旧版本的直连窗口；旧的独立共享服务也应先退出。双臂不用添加 `--direct`。单臂旧配置不受影响。
- 11月12日修复：空闲期间控制器断线后，新VR启动会先恢复真实连接并读取状态；不会重发已尝试的运动/录制请求。加载修复前，退出旧控制台并等待约5秒，再按原命令重开两个窗口。

### 夹爪偶发通信问题

当前已合并状态/报警/位置等反馈，正常每轮为2读3写；左FTDI延迟为2ms，已按序列号安装重插规则。

坏包不会用于控制，只读查询遇到CRC错误最多再读2次。持续通信异常只暂停对应夹爪，
机械臂、另一夹爪和VR窗口继续；不自动张开或掉使能。反馈恢复后摇杆回中再推即可。
写指令回复不明时不自动重发旧动作。通信异常提示按5秒节流，详细寄存器/TX/RX与计数保留在诊断中。

临时不使用夹爪：在短命令后加 `--cfg-options teleop.zx_grippers.enabled=False`。
只看头显输入、不连接设备：加 `--cfg-options teleop.vr_only=True teleop.zx_grippers.enabled=False`；会自动关闭控制器记录。
单独检查配置：加 `--preview`。

## 换 USB 接口后检查设备

在 `/home/robot/VLA/FluxVLA`、`fluxvla` 环境运行：

```bash
# 1. 日常检查：按序列号核对三台相机和两只夹爪的USB转接器
python scripts/a10_devices.py

# 2. 提示“路径需更新”时：先自动备份配置，再更新可唯一确认的路径
python scripts/a10_devices.py --update

# 3. 可选：额外查询两只知行夹爪的Modbus ID和状态，不发送动作
python scripts/a10_devices.py --check-grippers
```

- 默认不启动相机画面、不打开夹爪串口；相机仅查询支持的格式。
- `--check-grippers` 仅查询清单中配置的ID（当前两只各为1），不自动遍历或更改Modbus地址。
- “已匹配”：实际序列号、接口和配置路径一致；“路径需更新”：设备找到了，路径要修正。
  相机额外核对USB接口及视频index，防止选到同一相机的深度或红外节点。
- `--update` 只更新 `a10_devices.py` 的 `device` / `port` 字符串，保留其他参数、左右归属和注释。
  原文件备份在同目录 `a10_devices.py.时间戳.bak`；没有路径变化时不改文件、不创建备份。
  设备缺失、身份不唯一或接口无法确认时，不更新该设备；其他能唯一匹配的设备仍可更新。
- 新设备会列为“未登记”，不自动替代旧设备或分配左右。更换相机/转接器本体时需人工确认后改identity。
- **USB换插口不会改变设备序列号；夹爪和转接器之间的线若互换，程序无法判断物理左右。**
  夹爪表内型号和序列号属于USB转接器，不是夹爪本体的唯一身份证。
- 外部D435i按序列号+interface 03+index 0找到RGB，再更新by-path，避免默认by-id链接重名。
- 其他参数：`--config 路径` 指定另一份设备清单；`--json` 输出所有识别到的接口、格式及匹配结果。
  返回码：0=已匹配且可选夹爪查询无报警；1=缺失/需更新/不唯一/查询失败或夹爪有报警；2=配置或扫描失败。

## 用图片和原始位置检查设备

这些命令只检查本机相机/夹爪。夹爪动作先确认空载、手指离开夹口，每次只执行一条并观察。
拍照和夹爪动作分开执行；默认设备扫描不会发送动作。

```bash
# 三台相机各拍一张，并生成带名称、用途名、序列号的overview.jpg
python scripts/a10_devices.py --snapshot

# 输入所选夹爪配置范围内的任意整数raw位置；当前初始范围为0..3000
python scripts/a10_devices.py --gripper left --position-raw 1739
python scripts/a10_devices.py --gripper right --position-raw 2500
```

拍照默认保存在 `work_dirs/a10/hardware_checks/device_check_时间戳/`，终端打印完整路径。
包含三张按相机用途命名的JPEG、`overview.jpg`、`manifest.json`。画面按各相机的 `rotate_180`
配置输出，拼图另加标签；不代表硬件曝光同步。`--snapshot --output-dir 新目录` 可指定新目录，已有目录拒绝覆盖。

`a10_devices.py` 中每侧的 `check` 独立生效：

```python
check=dict(
    min_raw=0,
    max_raw=3000,       # 当前允许的测试范围，不是已测得的物理极限
    open_raw=None,      # 确认张开位置后保存
    close_raw=None,     # 确认闭合位置后保存
    tolerance_raw=5,    # 原始位置到位容差
    move_timeout_s=10,
)
```

- 命令行目标、驱动上限及实际位置检查统一读取范围；修改配置后下次启动生效。
  范围外目标、身份/路径不匹配、报警或实际位置超出范围附近时拒绝动作。路径变化先单独执行 `--update`。
- 每次只操作选中侧，其他设备离线不阻止该侧检查。使用配置速度/力矩，当前为5%/60%，检查入口仍限制speed<=5%、torque<=60%。
- 终端输出初始值、目标、实际反馈、误差、耗时和结果。连续两次收到位置到达状态，且误差在容差内才算到位。
  `torque_reached`仅表示遇阻，`timeout`表示未确认到位，两者都返回失败，不能当成量程端点。
- 运动指令按原驱动使能夹爪；超时不自动重发，报警不自动清除，退出不自动张开或掉使能。
  最后目标及保持力矩可能继续生效。raw不是毫米，当前3000上限也不表示完整行程。

## 逐侧确认工作量程及开合方向

```bash
# 左侧做完后，再单独运行右侧
python scripts/a10_devices.py --gripper left --calibrate
python scripts/a10_devices.py --gripper right --calibrate
```

即使实际位置已超出旧配置（例如反馈9000、旧范围0..3000），也允许进入校准并只读显示。
此时先输入 `range 0 9000` 让本轮范围覆盖当前位置；未完成范围核对前，运动目标仍被拒绝，提示符保留。
进入校准本身不会自动把夹爪拉回旧范围，也不会自动将9000写成量程上限。

进入 `校准>` 后，不会自动运动。按屏幕提示输入：

| 输入 | 功能 |
| --- | --- |
| 一个整数，如 `1000`、`2000` | 发送一个目标，等待并显示实际位置；逐级验证，不直接跳未知端点 |
| `open` 或 `张开` | 观察当前确实是合适的张开端后，记录刚刚确认到位的目标和实际值 |
| `close` 或 `闭合` | 观察当前确实是合适的闭合端后，同样记录 |
| `range 0 4000` | 明确设置本轮允许范围；只改变本次校准的检查范围，不发送运动、不立即写配置 |
| `save` 或 `保存` | 展示两个端点、工作范围和原始值方向；再输入 `确认` 或 `yes` 才保存 |
| `exit` 或 `退出` | 放弃本轮修改并退出 |

具体操作：先在当前范围内逐步输入位置，观察张开方向；到达合适的张开端后输入 `open`。
再逐步移动到合适的闭合端并输入 `close`，最后 `save`、核对、`确认`。
如果当前上限不够，先明确输入如 `range 0 4000`，再分级测试。新范围必须覆盖当前实际位置。
不要依靠自动碰撞硬限位来找范围；保存的是人工确认、留有余量的工作范围。

程序要求两个端点都曾确认到位，记录时再次回读，且间距大于两倍到位容差。
遇阻/超时会结束本轮校准且不保存；无目标到位记录、端点相同/太近或未确认保存时不写配置。
`Ctrl+C` / 输入结束也放弃保存，但不能据此认为最后运动目标已经取消。

保存前自动生成 `a10_devices.py.时间戳.bak`；仅替换选中侧的 `check`，保留身份、另一侧、相机及其他参数。
若配置在校准期间被其他程序修改，拒绝覆盖。结果包括：

- `min_raw/max_raw`：两端目标中的较小值和较大值。
- `open_raw/close_raw`：人工确认的张开、闭合目标，允许原始值增大时代表闭合。
- `calibration`：USB序列号、Modbus ID、两端实际反馈和确认时间。更换设备身份后命名开合拒绝复用旧记录。

保存后可直接检查两端：

```bash
python scripts/a10_devices.py --gripper left --open
python scripts/a10_devices.py --gripper left --close
```

右侧把 `left` 改成 `right`。`--open/--close` 对应保存的工作端点，并非厂商硬限位。
以后VR的开口比例可按 `target_raw = close_raw + ratio * (open_raw - close_raw)` 映射，
其中0闭合、1张开。此处只保存端点，尚未接入双臂VR；需要毫米数据时还需另做开口测量标定。

普通动作可加 `--json` 输出结果；交互校准不能与 `--json`、更新路径、拍照、全夹爪状态查询混用。
动作返回码：0=确认到位，1=遇阻/超时；校准0=保存成功，1=退出未保存或未到位；配置、设备或通信错误返回2。

## 双臂VR + 两只知行夹爪（2026-11-06）

已按现场确认登记两侧工作范围：raw 0完全张开、9000完全闭合。夹爪的left/right按物理左右及手柄命名，
与机械臂内部历史键分开：左手摇杆→FTDI DU0EMMMM→物理左臂夹爪；右手摇杆→CP2102N→物理右臂夹爪。
设备地址、ID和工作端点直接继承 `configs/a10/a10_devices.py`，无需在VR配置中重复填写。

**双臂配置已默认开启知行夹爪、实机与控制器高频记录，直接用下面的短命令即可。**
之前的机械臂速度/加速度等覆盖参数可以继续保留。下面是不带额外机械臂调参的基本命令：

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py
```

可额外加 `--preview` 检查配置，不启动VR、机器人连接或串口；输出包含 `zx_grippers=True`。
`vr_only=True` 或 `--observe` 时也不打开知行串口。默认夹爪开关为True，旧单臂/老夹爪入口沿用原逻辑。
不要同时运行设备动作/校准脚本或其他程序占用相同夹爪串口。

机械臂需先由现场明确使能；`allow_motion=True` 仅允许发送目标，不会自动使能。
若报 `Follower start requires Enabled and Stopped`，退出VR后可先只读查询：

```bash
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --read-only get
```

当反馈为Manual、Stopped、无故障且现场允许使能时，由操作者执行：

```bash
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --allow-motion manual_en
```

等待终端确认Enabled，再重新启动VR。若反馈Running或故障，应先现场确认停止/故障原因，不绕过启动检查。

首次验证先松开两手侧键、只推摇杆，确认各自夹爪开合；确认后再按侧键联合操作机械臂。
启动、手柄断流恢复后，先让各自摇杆回中一下，避免仍按住摇杆时自动续动。

| 操作 | 效果 |
| --- | --- |
| 左/右摇杆X向右 | 分别让对应夹爪张开（目标raw减小） |
| 左/右摇杆X向左 | 分别让对应夹爪闭合（目标raw增大） |
| 摇杆推得更大 | 目标变化更快 |
| 摇杆回中 | 不再更新该侧目标，已发出的最后小步可能继续完成 |
| 松开机械臂侧键 | 沿用原机械臂停止逻辑；夹爪仍由摇杆独立控制 |
| Ctrl+C | 请求机械臂跟随停止并关闭串口，不自动张开夹爪或掉使能 |

在双臂VR配置的 `teleop.zx_grippers` 中调整：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `devices.left/right.speed` | 各15 | 两侧VR夹爪电机速度百分比，允许整数1～15 |
| `target_rate_raw_s` | 3000 | 满摇杆时的目标变化速度，raw/秒；实际运动仍受电机速度及跟踪误差限制 |
| `deadzone` | 0.1 | 摇杆回中死区 |
| `direction.left/right` | 各1 | 设-1反转对应侧的摇杆开合方向 |
| `max_lead_raw` | 600 | 目标最多领先实际位置的原始值，减少积压目标及回中后拖动 |
| `io_hz` | 20 | 每侧后台串口循环频率 |
| `input_timeout_s` | 0.25 | 手柄样本过期后不再更新目标 |
| `feedback_timeout_s` | 0.5 | 夹爪反馈失效/串口读取过慢时中止本次遥操作 |

夹爪速度分两层：

- 摇杆目标变化速度在 `configs/a10/a10_dual_arm_vr_teleop.py` 的 `target_rate_raw_s`，默认3000。
- VR配置继承 `configs/a10/a10_devices.py` 的设备身份、量程等信息，并将两侧电机速度分别覆盖为15%；
  力矩为60%。设备清单自身的速度仍为5%，设备检查/标定入口仍限制speed<=5%。
  VR调速可在命令中覆盖 `teleop.zx_grippers.devices.left/right.speed`，允许整数1～15%。

2026-11-06用户实机反馈：两侧电机15%、目标速率3000raw/s、最大领先量600raw使用效果良好，
现已设为双臂VR配置默认值。使用以下命令即可，不必再写这四项速度参数：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py
```

两侧speed可分别设为1～15的整数，超过15或填写小数会在连接设备前报错。
target_rate_raw_s是目标变化速度，不代表实际开合速度；实际速度还受电机执行、串口更新及max_lead_raw限制。

2026-11-06修复：实际串口循环超过0.1秒时，旧积分最多只计算0.1秒，导致目标速度低于配置。
现按真实间隔积分，最多计入输入有效期，仍限制工作范围和目标超前量；循环等待扣除I/O耗时。
20Hz是调度频率上限，不保证串口吞吐达到20Hz；具体手感需现场复验。
两只夹爪各有后台线程，前台只发布最新输入和读取缓存；无历史目标队列。
串口内准备参数期间会再次检查输入截止时间及退出标记，过期或取消后不继续触发新的目标。
已触发的目标不能通过关闭程序自动撤销；停止更新不等于物理立即停止。

一侧夹爪报警、断线或反馈超时会终止会话，并请求停止机械臂跟随；退出保持夹爪现有目标及力矩。
单只手柄输入过期但机械臂未接管时，该夹爪停止更新，另一只手柄可继续控制自己的夹爪。
机械臂接管期间的输入异常继续服从原双臂组停止规则。

在 `work_dirs/a10/vr/dual_teleop_diagnostics.jsonl` 的 `gripper_command` 中按物理left/right记录
实际raw、目标raw、最近发送raw、输入轴值、输入截止时间、反馈时间、报警和线程阶段。
`last_sent_raw`仅说明发送函数返回，不是实机到位证明；到位/遇阻状态来自后续反馈。
软件与模拟串口已验证，真实双手摇杆开合及抓取手感待现场验收；原始值尚未标定成毫米。

## 当前设备绑定：双知行夹爪 + 三路相机（2026-11-05）

配置保存在 [a10_devices.py](../configs/a10/a10_devices.py)。左右按手柄及物理机械臂命名。

| 设备 | 稳定标识 | 当前节点 | 设备 ID / 用途 |
| --- | --- | --- | --- |
| 左手柄对应臂的夹爪 | FTDI `DU0EMMMM` | 本次为 `ttyUSB1` | Modbus ID 1 |
| 右手柄对应臂的夹爪 | CP2102N `5c9323064b6eef11a2c4e7c2c169b110` | 本次为 `ttyUSB0` | Modbus ID 1 |
| 左腕相机（单独换接确认） | D405 `255323070485`，`video-index4` | 当前为 `video4` | `left_wrist_0_rgb`，640×480 / 30Hz |
| 右腕相机（逐台拔线确认） | D405 `255323070551`，`video-index4` | 当前为 `video10` | `right_wrist_0_rgb`，640×480 / 30Hz |
| 外部场景相机 | D435i `262923065199`，USB interface 03 彩色接口 | 当前为 `video16` | `base_0_rgb`，640×480 / 30Hz |

左右归属已通过逐台拔线/换接确认：右腕 `255323070551`，左腕 `255323070485`。
右腕换接后恢复识别；外部相机现已识别，系统报告型号为D435i。三台均为USB3 5000M。
三路同时保持打开、逐帧读取各95帧通过，640×480 RGB、整体约29.99Hz；不代表曝光硬件同步。
外部D435i的by-id index0在深度/彩色接口间重名，已选用区分interface 03的by-path：
`/dev/v4l/by-path/platform-a80aa10000.usb-usb-0:3.2.2:1.3-video-index0`。
**外部相机换USB插口后，运行上面的扫描与 `--update` 命令更新路径；两台腕部继续使用各自by-id。**
一台 D405 会产生多个 video 节点，节点数量不能当成相机数量。

使用配置中的完整设备路径，USB 重插后 `ttyUSB` / `video` 编号可能改变。
两只夹爪内部 ID 同为1没有冲突，因为串口不同；这个 ID 不是机械臂的 ee_id。
当前双臂控制器中左手对应 ee1（operator 内部键 `right`），右手对应 ee0（内部键 `left`）。

在 `/home/robot/VLA/FluxVLA`、`fluxvla` 环境查询，以下命令只读状态：

```bash
# 左夹爪
python fluxvla/engines/operators/zx_gripper.py \
  --port /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0EMMMM-if00-port0 \
  --id 1 --state

# 右夹爪
python fluxvla/engines/operators/zx_gripper.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_5c9323064b6eef11a2c4e7c2c169b110-if00-port0 \
  --id 1 --state
```

空载小幅动作验证时，将对应命令的 `--state` 改为
`--position-raw 1000 --speed 5 --torque 60`，完成后再单独发送
`--position-raw 0 --speed 5 --torque 60`。原始位置值不是毫米。
两侧均由现场观察确认所属机械臂；左侧另有位置0、实际反馈3、无报警的到位日志。

本轮曾在左夹爪空载动作中出现 B=`0x1000`，手册定义为工件掉落报警；
其触发原因未确认，不能把它当成量程越界。报警时先查状态，不反复清除后盲目重试。
确需清除时，单独使用 `--reset-alarm --state`；清除本身不发送新的位置目标。

**设备清单现由双臂VR的可选知行夹爪入口复用。**
两只夹爪已由用户确认0张开/9000闭合，raw→mm尚未标定；三路相机ID与彩色读取均已确认。
三视角清单不代表双臂数据录制已接线；知行 raw 也不能直接填入旧数据的夹爪毫米字段。

## 之前的运行命令与配置记录

2026-10-26：VR、双臂控制与推理已整合到 `FluxVLA` 的 `kaanh_follower`。原 VR 目录保留作回退。
无 `--config` 的控制台沿用默认共享连接；双臂配置默认直连。不要同时启动推理与 VR 驱动同一机器人。

## 控制台连接哪台机器人

`python scripts/a10_control.py` 默认进入 shell 并允许控制，不自动使能机器人；
`--read-only` 可显式只读。默认读取 `configs/a10/a10_operator.py`，
当前是 `192.168.0.125:5999` 的单臂机器人，使用共享连接。
启动时会显示所选配置、目标地址、单/双臂、连接方式和控制权限。

```bash
# 单臂，与推理共享连接
python scripts/a10_control.py --config configs/a10/a10_operator.py --shared

# 双臂，当前配置地址 192.168.0.150
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py

# 同一种机器人更换 IP：--host 覆盖所选配置中的地址
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --host 192.168.0.151
```

省略命令时默认进入 shell，无需 `--allow-motion`。显式指定命令时，连接参数放在命令前。更换单/双臂用 `--config` 选择整套配置；`--host` 只覆盖 IP，
关节映射和初始化点位仍来自所选配置。长期使用另一台机器人时，可复制配置并修改
`operator.host`、端口、`arms` 和 `init_position`，再通过 `--config` 选择。

## 手动验证双臂关节 follower

用上面的双臂配置启动 shell。保持现场使能设置，先 `get` 确认 Enabled、Stopped，
然后逐条执行 `follower_aj`、`joints`。`joints` 只查询，并打印一条包含全部十二轴
当前绝对角度的 `joints --deg ...` 命令，顺序取配置 `joint_target_order`。
当前沿用 VR 配置名称：left（ID 0）的 J1～J6，再 right（ID 1）的 J1～J6，单位度。

复制本次查询打印的命令，只把第 6 个角度增加 1，其余 11 个值不变，再执行。
该命令更新最新目标，并通过现有 `send_joints` 按默认 30 Hz 持续发送同一个完整关节组。
即使角度没有变化也重复发送；再次输入 `joints --deg ...` 会替换目标，不排队积累旧目标。
启动参数 `--joint-hz 30` 可调整频率，须放在 `shell` 前。
控制台 `follower_aj` 会先发送原生 `start_follower`，收到成功回复后发送 `follower_aj`
（双臂配置附带 `--vel_percent=100`），再确认 Axis 状态。首次输入目标之前不发 UDP。
`stop`、`stop_follower`、退出、重连或执行其他控制命令会结束本地持续发送并清除目标；
`get` / `joints` 查询和 `help` 不结束持续发送。状态查询或发送失败会中止发送并请求停止，
不会自动恢复旧目标。发送期间定期查询状态，慢 I/O 不追赶补发，因此不是硬实时 30 Hz。
`m_init` 同样持续发送固定初始目标，见下节。
用 `joints` 查询实际变化，测试结束执行 `stop`，并用 `get` 确认 Stopped。
不要使用旧日志角度或初始化点位替代当前实测值。若启动时报 Running 或停止未确认，
先结束控制器当前命令，不能通过 raw 绕过状态检查继续叠加目标。

## FluxVLA 本地初始化 `m_init`（2026-10-29）

console 优先执行 FluxVLA 注册的命令；未注册名称继续原文透传。
本地命令参数错误或执行失败不会转发机器人端。`raw m_init` 可显式调用机器人端旧命令。
本地命令统一放在 `fluxvla/engines/operators/kaanh_commands/`，每个命令一个文件，
在该目录的 `__init__.py` 中登记；`m_init.py` 实现回位逻辑。

```bash
# 双臂控制台；单臂可省略 --config
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py
```

```text
manual_en
m_init
```

`m_init` 无参数，只返回 config 中的唯一 `operator.init_position`（弧度，`{臂名: 关节数组}`）。
单臂点位在 `configs/a10/a10_operator.py`，双臂点位在 `configs/a10/a10_dual_arm_operator.py`。
不带 `--config` 时控制台读取单臂配置。operator 保存传入的属性，未配置点位时 `m_init` 明确报错。
命令本身不定义点位；回位流程是 `kaanh_commands/m_init.py` 中的独立函数，BaseOperator 无须继承命令实现。
`joint_dofs` 记录每臂维度，公共回位逻辑支持六轴、七轴及多臂；具体机器人提供实际反馈与 follower 读写，A10 每臂仍为六轴。

双臂 A10 沿用 VR 名称映射：`left→ee_id=0`、`right→ee_id=1`，
关节目标顺序 `joint_target_order=['left', 'right']`。名称是 operator 分组键；VR 手柄绑定仍为左手→ID 1、右手→ID 0。
`cartesian_target_order` 独立描述末端控制组行顺序。关节 follower 使用原有 40 字节头及
`jp --motor_pos=[...]` 协议（度），双臂一次发送完整十二维；需机器人 follower 接收对应维度。
2026-10-29 的双臂试验出现严重滞后和现场报告的震动，未通过实机验收。
2026-11-06 按用户指定，双臂配置设置 `joint_follower_velocity_percent=100`，
启动命令为 `follower_aj --vel_percent=100`。该值是 follower 参数，不修改 `set_jog_vel`
或 `set_pgm_vel`。单臂未配置此项时仍发送 `follower_aj`。未切换 TG。
用户随后现场确认加上该参数后运动与停止均正常。保留该配置；SDK 内部原因未确认。
`m_init` 直接调用 `A10Operator.send_joints()`，默认按 30 Hz 持续发送配置中的同一个完整绝对关节目标。
控制器 follower 负责向目标运动；FluxVLA 不插值、不生成中间点、不分块，也不调用推理轨迹执行器。

回位前保存 follower 是否开启、Axis/Cart 模式和 jog 坐标系，再暂停已有跟随；
共享 console 会先撤销推理控制权。确认 follower 已退出后才启动关节 follower，不对正在运行的 follower 重复启动。
读取实际关节后，
重复发送 `init_position`，同时按反馈间隔查询实际位置。所有关节连续三次误差不超过
`0.01 rad` 后请求并确认停止 follower。回位前保存当前模式的速度倍率，临时设为 **5%**；
手动模式发送 `set_jog_vel --vel_percent=5`，自动模式发送 `set_pgm_vel --vel_percent=5`。
运动结束并确认停止后恢复原倍率；取消或异常退出也会尝试恢复。停止未确认、连接中断或
现场已切换模式／修改倍率时不强行覆盖，并明确报告恢复未确认。
正常到位并恢复速度后，恢复原 follower 模式、开启／关闭状态及 jog 坐标系；原来关闭则仍关闭，
原来开启笛卡尔跟随则恢复 Cart。不会重发旧目标，VR 接管时按回位后的实际位姿重新建立参考。
取消、故障或恢复失败时保持停止，不自动重新开启 follower。不会自动使能、恢复其他推理进程或重试运动命令。
`stop` 可以取消等待并请求停止。中断、故障、反馈缺失及超时也会尝试停止。

`init_motion` 配置发送频率和到位检测：`publish_hz=30`、`tolerance_rad=0.01`、
`arrival_timeout_s=60`、`feedback_interval_s=0.1`。旧插值速度、加速度和分块配置项已删除。
同一目标持续发送到到位、取消、故障或超时；慢 I/O 不追赶补发。
点位仍属于各机器人配置，按 `joint_dofs` 验证各臂维度；A10 最终报文单位度。
本轮为离线实现和回环测试，不代表双臂实机已通过验收。

下面是已删除的分块实现所产生的历史故障及调查记录：

2026-10-30 用户反馈：规划 39.37 秒、已发 300 帧后，下一块被
`0.226298 rad > 0.200000 rad` 拒绝；停止反馈仍为 inactive + Running。
该检查同时覆盖块首目标相对实际位置的偏差和块内相邻目标步长，不能将报错中的
0.226298 rad 直接解释为单帧下发跳变，也不能通过增大阈值解决不跟随。
本次仅离线核对，有以下依据及限制：

- 历史单臂部署记录（见 `a10_controller_follower_review.md` 的 2026-10-22 章节）
  包含自定义六轴 streaming follower；本地保存的 `A10JointFollower` 也注册
  `follower_aj`。双臂已保存的 `get_xml` 则将同名命令绑定到 SDK 的
  `kaanh.FollowerAjAbsolute2`。命令名一致不证明底层实现一致；尚未核实用户那次
  成功推理实际运行的二进制。
- 仓库参考 `test/test_follower/udpFollower.cpp` 中，该 SDK 类将 `vel_percent`
  声明为 `int`，再执行 `/= 100.0`，因此默认 10 会截断为 0，并用于速度、加速度
  限制。双臂保存的 XML 中该参数默认值为 10。这是参考源码中的确定缺陷，
  但尚无运行库与源码对应证据，不能宣称已确定现场根因。
- 该参考实现的 `stop_follower` 只清除运行标记，实时计划还要执行收尾并返回，
  因而 inactive 本身不等于 Stopped；尚不能据此解释 `manual_stop` 未确认的全部原因。

下一步需核实双臂实际加载的 SDK/关节 follower 实现及对应源码，优先对齐已验证的
控制器实现。不能自动切换 TG、把速度比例改成 100、扩大跟随误差阈值或继续试跑。
本轮未修改控制器、发送参数或现场状态。

控制台状态显示当前模式的速度倍率；`m_init` 检测到倍率为 0 时在启动前拒绝，执行中变成 0 也会取消。
双臂直连的关节 UDP 会读取控制器返回的拒绝信息，例如 `pos or vel size is not equal to motor size`，
此错误表示目标数量与控制器配置的关节控制组不一致。没有 UDP 回复不表示目标已被接收或执行。

`init_gripper_positions` 默认每臂 `0 mm`；命令模块中的 `initialize_grippers(robot, targets_mm)` 是后续驱动接入点。
当前只执行机械臂回位，夹爪保持原状，console 明确显示夹爪初始化尚未执行。
`last_init_motion` 分别记录机械臂到位、夹爪执行、follower 停止及完成状态。

## 执行中停止：`stop`

在 `a10>` 输入 `stop`：取消当前命令和停止前排队的命令，禁止继续下发关节、末端和夹爪目标，
中断长命令的回复等待，再发送 `stop_follower` 和 `manual_stop`，请求退出跟随并结束控制器当前运动命令。
`stop_follower` 本身只清除 SDK 跟随标志，可能留下 `follower_active=False, motion=Running`；
仅重复它无法保证当前实时命令退出。命令执行与输入分开，因此 `m_init`、`jog`
或原生命令执行中也能输入 `stop`，控制台保持打开，可继续查询 `get`。
新动作必须由停止后重新输入的命令触发，旧命令不会恢复或自动重试。

只有 follower 不再运行且 `robot_msg.motion=Stopped` 时才报告停止成功。
停止反馈失败会保留“本地下发已取消，但机器人停止未确认”的提示；软件 stop 不是硬件急停，
不能保证中断机器人端任意原生程序或停止不支持停止接口的夹爪；异常运动仍用现场控制器停止/急停。
共享模式下 `stop` 同时撤销推理的发送权限。取消长回复可能关闭原连接以隔离迟到消息，
仅为下发停止指令重新连接，不自动使能或重发原命令。
若初始化提示 `Follower start requires Enabled and Stopped feedback`，先查看其中的
`activate` 和 `motion`。对于 `Enabled + Running`，输入 `stop`，确认报告 `Stopped` 后再输入 `m_init`。
修改停止逻辑后需重启控制台；旧进程不会自动加载新代码。

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
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py
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
| ⑦ 双臂 VR 独立接管/释放 | `a10_dual_arm_vr_teleop.py` | 默认连接实机，侧键接管 | 默认开启知行夹爪与控制器记录 |
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
operator 模板默认禁止运动；交互控制台默认允许控制，`--read-only` 可覆盖。启停作用于整个控制组。
双臂关节回位见上方 `m_init`；双臂 VR、夹爪与录制用法见下方对应章节。⑤仍是原单臂 VR 命令。
合并统一以 VR 映射为准：left→ID 0、right→ID 1，目标顺序均为 left、right。
既有手柄绑定保持左手控制 ID 1、右手控制 ID 0；初始化点位按 ID 对应，十二轴实际目标不变。

## ⑦ 双臂 VR：左手控左臂，右手控右臂

本次在 `kaanh_follower` 的独立目录开发，原 `FluxVLA` 目录的 `console` 分支可继续改代码：

```bash
cd /home/robot/VLA/FluxVLA
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
- 双臂当前目标限速 `0.17 m/s`、`1.4 rad/s`，线/角加速度`1/6`，角制动`10 rad/s²`；不修改单臂参数。
- 正常操作：按下控制、松开减速、再按继续；减速尚未结束也可直接重新接管，不用额外松开一次。
- 两手均释放后结束原生follower并确认停止；正常停止确认后，再按侧键可直接接管，不要求二次松手。VR断流时停止重发旧目标。
- 前端目标减速归零稳定后判定保持；跟踪误差仅记录，不因此锁住控制。输入断流、控制器故障及通信异常仍暂停新目标。
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
cd /home/robot/VLA/FluxVLA
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


### 双臂数据校验与导出（实时入口见下一节，实机待验收）

在FluxVLA主目录运行；只有真实录制目录产生且写入器关闭后才使用以下命令。它们不启动设备，`--record-controller`也不会生成训练数据。

```bash
python scripts/validate_a10_dataset.py datasets/a10_dual_record
python scripts/export_a10_openpi_dataset.py \
  --source datasets/a10_dual_record \
  --output datasets/a10_dual_record_openpi
```

输出目录必须不存在；完整约定见[双臂数据交接说明](a10_dual_dataset_handoff.md)。14维顺序是物理左臂+左夹爪、物理右臂+右夹爪，夹爪0张开/1闭合，训练端需独立适配policy。


### 双臂VR采集：右手A功能键（2026-11-06）

2026-11-12：`kaanh_lerobot_recording`分支默认用原生LeRobotDataset；新目录为`datasets/a10_dual_lerobot_record`，不与旧后端数据混写。依赖已装好；初始化新环境时执行`python -m pip install --no-deps -r requirements-a10-recording.txt`，保留Thor的PyTorch构建。

先检查按钮，不连接机器人/夹爪/相机：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --record-config configs/a10/a10_dual_arm_record.py \
  --check-record-buttons
```

确认实体右A对应后，由现场操作者启动真实采集：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --record-config configs/a10/a10_dual_arm_record.py \
  --cfg-options \
    teleop.record_controller=False \
    teleop.vr_only=False \
    teleop.observe_only=False \
    teleop.operator.allow_motion=True \
    teleop.zx_grippers.enabled=True
```

按住右A，左摇杆回中后沿用旧LeRobot阶段：准备时右=开始；录制中右=结束采样并整理场景；整理好再右=保存并自动下一集。左=标记重录，整理好右=废弃并重录本集；下=停止录制，保存有效当前集后退出；上=复位确认。复位须释放两手接管键，过程中不录数据；成功后保持A、回中再向右即可。备用终端：start/save/discard/reset/stop/status/quit。

夹爪故障不直接关闭VR/录制界面；故障夹爪禁发目标，当前集标记中断。现场排除碰擦、两手释放且停止确认后，可在当前VR终端输入`gripper-reset left`或`gripper-reset right`。需要掉使能时先放稳物体，再输入`gripper-disable left/right`（会释放夹持力）。不另开进程争用串口、不自动清报警；持续故障不会自动恢复运动。见[夹爪故障与旧流程说明](a10_vr_recording.md#14-夹爪故障不退出阶段顺序对齐旧工具链)。

首次使用空输出目录；续录在命令末尾追加 `--record-options record.resume=True`，任务和输出目录也可通过 `capture.task` / `record.root` 覆盖。未保存中断可加`record.recover=True`归档片段；原生保存中断须保留目录检查、改用新目录，不自动修复。输入过期确认停止后保留界面，松手回中、稳定、新按侧键后重新接管；中断集需废弃再开始。中断恢复、HUD及日志见[采集指南](a10_vr_recording.md#13-原生-lerobot-采集分支2026-11-12)。

### 测VR输入延迟（2026-11-09）

VR＋现有录制头显，无设备控制：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --record-config configs/a10/a10_dual_arm_record.py \
  --check-record-buttons --measure-vr-latency
```

真实采集负载测试：在原完整采集命令的`--cfg-options`前加`--measure-vr-latency`，其他参数保留。刷新头显网页，采样约2分钟后Ctrl+C退出；报告在`work_dirs/a10/vr/latency/<本次目录>/summary.json`，逐包数据在同目录`events.jsonl`。测的是应用层RTT及发包/收包/读取间隔，不是机械臂响应时间。具体对照流程见[录制指南第8节](a10_vr_recording.md#8-vr-网络延迟诊断2026-11-09)。
