# A10 现场接入操作说明

> 当前入口：`configs/a10/a10_dual_vr.py`，配置分工见[配置索引](../configs/a10/README.md)，运行命令见[CLI.txt](../CLI.txt)。下文保留当时的接入/开发记录，旧路径和双臂默认行为以当前入口为准。

2026-10-26 整合说明：旧单臂推理与共享控制台保持兼容，双臂配置控制台默认直连。
原 VR 证书独立保存在 `work_dirs/a10/vr/tls_vr`；主目录旧证书和现场配置未覆盖。
本次是代码与离线验证整合，不代表双臂或 VR 实机已验收。

## 双 A10 反馈查看器

启动后在同一台电脑的浏览器打开 `http://127.0.0.1:8765`。

```bash
python scripts/a10_visualize.py --host 192.168.0.198
```

查看器经 WebSocket 在后台按 10 Hz 读取 `get`，浏览器从本机服务读取同一份最新状态。关节表会显示
每轴实际角 `actual_pos`、目标角 `motor_pos` 和差值。默认从 `others/A10_fixed.zip` 读取 URDF 和七个 STL，
用实际关节角驱动实体。模型轴向已与控制器反馈核对，两臂的 J1–J3 在 URDF 中反号。
末端坐标轴用控制器的 `pq`。两臂按基座侧孔相对、中心距 115 cm 摆放。基座旋转和位移用于显示，关节表下的 TCP 读数保留控制器坐标。
默认模型给双 A10 使用，组号以 ID 0 和 ID 1 显示。更换控制器时先核对反馈单位、坐标系和部件顺序。

网格是查看器绘制的参考平面。当前 XML 的 `wobj` 是工件坐标系，工作台实体的尺寸和安装位姿需另行提供。
全零关节角时双臂接近竖直。模型包缺失或浏览器无法使用 WebGL 时，页面仍可用 `part_pq` 和 XML 关节标记显示骨架。

- 模型包放在其他位置时，用 `--model-archive /path/to/A10_fixed.zip` 指定。
- 用 `python scripts/a10_visualize.py --initial-pose --listen-port 8766` 离线预览双臂 `m_init` 关节角。
- 单独查看时默认使用 `5999`。并行运行运动控制台时，可用
  `--status-port 5888` 将查看器接到另一个已验证可读取 `get` 的控制器端口。
- 已有本机 A10 共享服务时，可加 `--shared-port 端口号`。共享服务的控制器 IP 和状态端口必须与查看器配置一致。
- 在离线环境加 `--replay 文件.json`，可读取原始 `get` 单帧或帧数组。数组按配置频率播放一次，停在最后一帧。
  关节轨迹可使用 `a10_joint_feedback_v1` 格式，每帧以 `actual_deg` 和 `target_deg` 保存两臂各六轴。
  页面使用 URDF 回放姿态，并标记未记录的 TCP。若轨迹采样频率不同，用 `--poll-hz` 指定。
- 反馈过期或连接断开时保留最后一帧并提示。默认配置见 [a10_visualizer.py](../configs/a10/a10_visualizer.py)。

2026-11-20 验证记录：Chrome 已验证离线回放、.198 实时显示、缩放和重置视角。已确认从 5888 和 5999 可同时读取 `get`。
现场 EtherCAT 主站链路为 DOWN，物理从站为 0。虚拟运动验收停在使能阶段，控制器记录“用户未登陆，使能信号无效”。
请在控制器侧登录具备 Follower 权限的账号，再核对目标角、实际角和骨架是否随两个臂的单关节测试更新。

## 双 A10 控制台最小验证（2026-10-23，不需要 VR）

当前目录 `/home/robot/VLA/FluxVLA`，分支 `kaanh_follower`（2026-10-26 已整合）。
这次增加的是控制台通信和小幅移动，旧单臂命令、单臂配置、VR 运行代码保持原样。

**进度：离线回环测试已通过；实机未验收。** 2026-10-23 只读 SSH 复查时，
`192.168.0.150` 上没有运行 `kaanhbin`，5867/5999/5888/9998/9999 未监听。
先由控制器维护同事启动正确工程并提供可用 WS 入口；不要重复启动已运行的服务。
只知道磁盘 XML，不据此认定正在运行的程序版本或真实反馈格式已通过验证。

2026-10-29 更新：确认原有5867入口被 `.232` 示教器连接后，按用户要求备份并修改
`/home/kaanh/Desktop/kaanh-project/kaanh.xml`，新增5888、5999两个WebSocket入口。
5867继续供示教器使用；FluxVLA双臂配置改为5999查询和控制，5888暂作备用入口。
写入时控制程序已未运行；需要现场从该工程目录重新启动 `sudo ./kaanhbin` 加载新XML。
本次没有启动程序、使能或发送运动目标，新端口监听和真实GET仍待现场确认。

### 先只读连接

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --read-only get --raw
```

配置按更新后的 XML 使用 WS 5999、笛卡尔 UDP 9998、关节 UDP 9999；只用一个 WS 连接，
不打开后台状态连接。示教器使用5867，不必为FluxVLA断开示教器；仍需协调双方的运动控制权限。
若团队提供其他命令端口，可在 `get`/`shell` 前加 `--ws-port 实际端口`。

核对反馈与示教器：配置暂定 `left=ee0/TCP第0行/关节第0组`，
`right=ee1/TCP第1行/关节第1组`，每组6轴，反馈毫米/度。
**这只是待现场确认的映射，不能凭名称认定物理左右。** 如果反馈不符，调整该双臂配置中的
`arms` 字段；`cartesian_target_order` 必须按 XML 控制组的 ee 顺序对应。
XML 笛卡尔组是 tg=2、ee_ids={0,1}，无需向 JSON 自行添加 tg/id 字段。

### 打开可控制的控制台

确认单位、左右映射、工具轴方向和运动范围后执行：

```bash
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py --allow-motion shell
```

打开只会连接并读取状态。配置默认 `allow_motion=False`，`--allow-motion` 显式允许你输入控制命令，
不会自动使能。`--read-only` 和 `--allow-motion` 不能同时使用。

出现 `a10>` 后，每次输入一条，确认返回结果后再执行下一条：

```text
get
manual_en
set_jog_coordinate --tool
follower_cart
```

随后选择一个小幅运动，**不要将以下全部命令一次粘贴执行**：

| 命令 | 含义 |
| --- | --- |
| `jog left x 3` | left 沿本次开始时工具 X 轴移动 3 mm，right 保持 |
| `jog left x -3` | left 沿本次开始时工具 X 轴反向移动 3 mm |
| `jog right z 3` | right 沿本次开始时工具 Z 轴移动 3 mm，left 保持 |
| `jog right rz 2` | right 绕本次开始时工具 Z 轴旋转 2 度，left 保持 |
| `jog left x 3 right x 3` | 两臂分别沿自身工具 X 轴移动 3 mm |
| `get` / `joints` | 分别查看两臂状态 / 六轴角度（度） |
| `stop_follower` | 停止整个控制组，并确认 inactive + Stopped |
| `manual_ds` | 手动掉使能，不是急停 |
| `exit` | 请求停止本次 follower 后关闭连接，不自动掉使能 |

`x/y/z` 用毫米，`rx/ry/rz` 用度；每次每臂只能指定一个轴，最大 10 mm / 5 度。
方向采用各臂本次开始时的**计划工具朝向**，不是机器人基坐标轴、屏幕方向或 VR 方向。
连续命令沿上一目标继续，不把滞后的反馈重新当作累计指令零点；开始前检查实际反馈接近上一目标。

内部以平滑起止的小步发送，标称30Hz；前端计划速度上限约5 mm/s、0.05 rad/s，
控制器速度倍率保持现场设置，不自动改成50%。每步先读取反馈，读取较慢会拉长运动时间，不追赶补发。
这不是高频实时遥操作入口，也不是关节速度或避碰保护。

每个 UDP 包有两行 `[x,y,z,rz,ry,rx]`，线性量米、角度弧度；使用 SDK 工具累计增量语义。
未选择的臂继续发送保持值；重复发送末值不会重复累计移动。包头沿用40字节ARIS格式。

终端提示“连续3次落入容差”表示两臂 TCP 反馈都在目标的 **1 mm / 0.02 rad** 范围内，
不代表完全静止，也不是UDP确认；小于容差的命令不能用于证明实际运动。
运动中超出20 mm / 0.15 rad 跟踪误差、反馈异常、Ctrl+C 或最终5秒内未确认到位，
会请求停止本次整个控制组。若未确认停止，终端会说明；需现场检查，不能继续盲目发送。
正常小步完成后 follower 仍运行；不要让会话长期闲置，验证完成后输入 `stop_follower` 和 `exit`。

本轮不提供双臂关节、夹爪、VR控制适配；双臂 `pq`、`joints --deg`、`follower_aj`、`g_move` 继续拒绝。
`follower_cart` / `stop_follower` 是公共启停；指定单臂 jog 不等于独立启动该臂的 follower。

### 实现与验证记录

- `a10_operator.py`：保留单臂 SDK 接口；双臂独立计算、按配置行序一次发送；先校验全部行，
  发送成功后统一提交累计状态。发送失败禁止在同次 follower 内重新建零点；必须停止后重新启动。
- `scripts/a10_control.py`：新增双臂 `jog` 和 `--allow-motion`；只读配置无独立状态端口时沿用其命令端口。
- 测试覆盖单臂原报文、双臂行序、保持、连续命令、工具坐标方向、旋转、错误数据、发送失败、
  未到位、故障、Ctrl+C、停止确认与只读权限。使用回环 WS/UDP 和理想工具增量积分接收端，不能替代实机验收。
- 回归结果：`PYTHONPATH=.:test python -m unittest test_a10_operator test_a10_sdk_adapter test_a10_control`，101项全部通过；`git diff --check` 通过。
- 未修改远端文件、启动服务或发送机器人运动指令；未修改 VR runtime 或旧单臂配置。


**VR 当前入口**：在 `/home/robot/VLA/FluxVLA` 使用
`python scripts/teleoperate.py --config configs/a10/a10_single_vr_base.py`。
默认 VR-only；详细配置和新布局见 [VR 说明](a10_vr_operator.md)。
下文 `projects/a10` 相关命令为历史流程，不作为新 VR 遥操作入口。

2026-10-26 VR 已合入 `kaanh_follower`，原 VR 分支保留作回退；
VR 使用与迁移进度见 [VR 遥操作说明](a10_vr_operator.md)。工作目录统一为
`/home/robot/VLA/FluxVLA`。

## 原生 follower 命令行控制台（2026-10-09）

本节用于直接连接工控机上的 KAANH follower。命令解析放在 `scripts/a10_control.py`，机器人通信仍由 `fluxvla/engines/operators/a10_operator.py` 完成。

### 默认自动选择推理连接

推理启动时自动选择连接：本机5870共享服务存在就复用；没有服务就直接连接机器人5999（控制）和5888（状态）。无需先启动控制台。
模型服务已启动后直接执行：

```bash
python scripts/inference.py --config configs/a10/a10_single_remote_inference.py
```

控制台已打开时，直接使用上述推理命令即可同时使用，无需配置参数。
仅当本机共享端口明确拒绝连接时才采用直连；服务超时、协议错误或机器人地址/端口不匹配会明确报错，不会静默回退。
连接方式在启动或显式重连时选择；运行中不会切换连接或自动重发运动命令。
若先启动推理并已直连，之后再打开控制台无法接管现有直连；需结束推理，先开控制台，再启动推理。

### 可选：推理期间使用指定控制器速度倍率

默认 `inference.controller_speed_percent=None`，保留现场倍率。需要指定本次运行倍率时：

```bash
python scripts/inference.py --config configs/a10/a10_single_remote_inference.py \
  --cfg-options inference.controller_speed_percent=25
```

也可以直接修改配置文件中的 `controller_speed_percent`，取值为1～100的整数。
这是控制器速度百分比，不是关节目标发送频率；30Hz发送节奏和每轮15个动作保持原配置。
Manual模式发送 `set_jog_vel --vel_percent=25`，Auto模式发送 `set_pgm_vel --vel_percent=25`，不会自动切换模式。
程序在首次启动follower前设置并回读确认，退出且确认停止后恢复原倍率。
dry-run不修改倍率。断线、停止未确认、控制台接管、模式或倍率发生变化时，不覆盖现场设置；结果记录在
`events.jsonl` 的 `controller_speed_override` 事件和终端日志中。

直连与共享连接均支持。已有共享服务进程需要退出并重新启动控制台一次，以加载新增的倍率RPC；
模型推理服务无需重启。

### 双臂控制台与 VR 同时使用（2026-11-11）

双臂默认通过本机共享服务连接 `.150`，不用再手动开启第三个终端。
**控制台或 VR 都可以先开**，第二个自动复用连接。两个终端都先进入仓库并激活 `fluxvla` 环境。

终端一：

```bash
python scripts/a10_control.py --config configs/a10/a10_dual_arm_operator.py
```

终端二：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_vr.py
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

### 单臂旧配置：控制台与推理共用连接

机器人5999和5888端口按单连接方式共享；5867留给示教器。仅在需要控制台与推理同时使用时启用共享连接，
共享服务统一持有机器人两条WebSocket及UDP发送socket，控制台默认自动启动共享服务。
首次切换需退出旧的直连控制台/推理客户端一次；机器人端服务不必关闭。
之后控制台窗口可以一直保留，`get` 与推理状态查询并行，命令回复不会串到其他会话。

同一 `fluxvla` 环境、仓库根目录，启动控制台即可：

```bash
python scripts/a10_control.py shell
```

看到“共享连接已自动启动”及连接成功后，保持控制台打开，在另一终端启动推理。
已有共享服务时自动复用，不会再连接一份机器人。模型服务保持原启动方式；推理客户端仍使用：

```bash
python scripts/inference.py --config configs/a10/a10_single_remote_inference.py
```

- 查询不会抢占动作控制权；`--read-only shell` 仍禁止控制指令。
- 控制台 `m_init`、使能/掉使能、夹爪等控制器指令会先停止共享服务所属的推理follower并回读确认，再执行。停止未确认时不执行后续命令。
- 控制台可 `stop_follower` 停止当前推理；也可显式启动自身follower。被接管的推理终止当前发送并清理，不自动恢复，也不会停止控制台新启动的follower。
- 单个时刻只允许一个会话发送关节/末端目标。推理不会抢占控制台已启动的follower；控制台停止后再显式重启推理。
- 控制连接意外关闭时，服务尝试停止该会话所属follower并确认；服务/网络异常仍不能保证物理停止。日志目录后缀 `_shared_control` 保存控制权切换和确认结果。
- 共享服务只监听本机127.0.0.1:5870，机器人控制端口仍固定5999，状态5888、UDP9998/9999不变。相机与USB夹爪仍由推理客户端管理。
- 控制台退出时会关闭自己启动的共享服务，并尝试停止当前共享跟随；推理期间请保持该控制台打开。复用外部服务时，退出控制台不会关闭外部服务。

需要临时回到独占直连时，关闭共享服务后，控制台加 `--direct`；推理配置可显式设置 `inference.operator.shared_control=None`。

### 1. 独占直连控制台（未使用共享服务时）

机器人默认 IP 为 `192.168.0.125`，控制 WebSocket 端口固定为 `5999`（按2026-10-16新版控制器接口分配；5867留给示教器），状态端口为 `5888`，控制台与原生推理配置已统一；独占连接时加 `--direct`，用 `--host` 覆盖地址。模型服务的 `--host 127.0.0.1` 是本机监听地址，与机器人 IP 无关。

工控机的 kaanhbin 需已正常运行；已运行时不要再启动一份。以下命令在运行 FluxVLA 的机器终端执行：

```bash
cd /home/robot/VLA/FluxVLA
/home/robot/miniforge3/envs/fluxvla/bin/python scripts/a10_control.py --direct --host 192.168.0.125 shell
```

推理、follower 和 VR 现在统一在 **kaanh_follower** 的 `/home/robot/VLA/FluxVLA` 工作目录。原 VR 开发快照保存在 `backup/vr-before-consolidation-20261015` 分支；迁回前的版本和附件备份位置见迁移计划末尾。迁回验证已通过，三个旧的独立工作目录已移除；后续统一从本目录运行。

出现 `a10>` 后输入短指令即可。启动只连接并查询，不会自动使能、启动跟随或发送运动目标。

只想查询一次可运行：

```bash
/home/robot/miniforge3/envs/fluxvla/bin/python scripts/a10_control.py --direct --host 192.168.0.125 get
```

只读交互模式：在 shell 前加 `--read-only`。查看帮助用 `--help`，查看指令列表用 `--host 192.168.0.125 help`；帮助不连接设备。默认 WS 5999、Cartesian UDP 9998、关节 UDP 9999、超时 5 秒，可在 shell/get 前用 `--ws-port`、`--udp-port`、`--joint-udp-port`、`--timeout` 修改。两路 UDP 端口不能相同。

### 2. 首次现场验证：每次输入一条，检查成功后再下一条

以下内容输入到 `a10>`，不要把 `a10>` 提示符一起复制：

```text
get
set_jog_coordinate --tool
manual_en
follower_cart
get
stop_follower
```

预期结果依次是：状态可读取且无故障 → 坐标系 Tool → 电机 Enabled → 跟随运行中、Cart → 状态再次确认 → 跟随未运行。

- 若 get 报警，先核对故障及现场控制器文档；控制台支持透传已知的控制器原生命令（如 `m_init`），不推测清错命令的语义。
- manual_en/manual_ds 仅适用于确认的 Manual 模式；不会自动切换 Auto/Manual。
- 若已有其他程序启动的 follower，控制台拒绝接管。请由原操作方结束。
- 本轮只验证启停，不包含 pq 或夹爪目标，不能据此宣称运动接口已完整验收。
- 测试结束若需要掉使能，再输入 `manual_ds` 并检查 Disabled；最后输入 `exit`。

### 3. 指令速查

| 输入 | 做什么／如何判断 |
| --- | --- |
| `help` | 查看命令及单位 |
| `reconnect` | 断线后重新连接并查询；仅 shell，不补发任何控制命令 |
| `get` | 中文显示模式、电机、坐标系、跟随、故障、关节及夹爪反馈 |
| `get --raw` | 原始状态 JSON（含接收的嵌套字段），排查用 |
| `manual_en` | 手动使能，等待 Enabled；仅 ACK 不算成功 |
| `manual_ds` | 手动掉使能，等待 Disabled；不是急停 |
| `set_jog_coordinate --tool` | 设置工具坐标系，等待 Tool；影响后续手动点动参考系 |
| `set_jog_coordinate --axis` | 显式设置关节坐标系，等待 Axis |
| `follower_cart` | 在本连接启动笛卡尔跟随并回读确认，只能在 shell 使用 |
| `follower_aj` | 在本连接启动六轴关节跟随并回读 Axis，只能在 shell 使用 |
| `joints` | 只查询实际角度，按关节1到6打印度数及可复制的目标文本 |
| `joints --deg q1 q2 q3 q4 q5 q6` | 替换为六个绝对角度（度），不含夹爪；仅 shell、仅已确认的关节模式 |
| `stop_follower` | 停止本连接启动的跟随并确认，只能在 shell 使用 |
| `pq x y z qx qy qz qw` | 将占位符替换成 7 个实际数字；仅 shell，发送一个 UDP 目标 |
| `g_move --pos=毫米数` | 替换成 0..100 的数字，发送夹爪目标；回复成功不等于到位 |
| `m_init` 等其他指令 | 未内置的指令按输入原文透传至控制 WebSocket，显示控制器完整 JSON 回复 |
| `raw 控制器指令` | 显式透传，例如 `raw manual_en --原生参数`；可处理与内置名称重名的原生命令 |
| `exit` | 尝试停止本次跟随并关闭连接，不自动掉使能 |

pq 的 xyz 单位是 **米**，四元数为 **xyzw** 且长度须为 1；采用附件模板的相对启动基准目标，不是绝对位置，也不是“每输入一次就再走这么远”。七个零不是有效四元数。该控制台不累加、不插值、不做坐标变换；规划属于控制器。单次发送没有应用层 ACK，显示“已发送”不证明收到或执行。具体方向、尺度、旋转组合和接收频率需要现场单独验收，未验证前不要直接套用其他机器人的目标数值。

get 没有实际夹爪字段时显示“暂无反馈”，不填假数据。夹爪尚未验证到位条件。控制台不后台记录数据、不新增 work_dirs 结果目录；需要开发排查时可手动保留终端输出。

在 `a10>` 输入 `m_init` 或带参数的其他控制器指令即可发送，参数、引号及空格按原文保留。内置命令继续执行原有格式检查；需要使用同名命令的原生参数时加 `raw ` 前缀。也支持单次执行 `python scripts/a10_control.py m_init`。单次入口的参数先经系统 shell 解析，控制台会重新引用后发送；需要字节一致的文本时使用交互入口。

透传也受 `--read-only` 限制；未连接时不会发送，不自动重试。完整回复中的非零 `ret_code` 会明确显示为失败，零只表示收到接受回复，不表示机器人动作完成。透传前后会查询状态，期间撤销本地 UDP 目标发送资格。已知 `follower_cart`／`follower_aj` 的启动尝试、原有本次跟随，以及透传后反馈确认从未跟随变为跟随，会保留停止清理义务；普通 `m_init` 仅因回复超时不会被当作启动了 follower。透传后需要发送 UDP 目标时，如有待停止的本次跟随先 `stop_follower`，再用内置 `follower_aj` 或 `follower_cart` 重新确认启动。未接管其他会话已运行的跟随。

原生控制器命令回复默认等待 **60 秒**，与连接、`get` 和内置反馈确认的 `--timeout`（默认 5 秒）独立。长时命令可以这样增加等待时间：

```bash
python scripts/a10_control.py --command-timeout 120 shell
```

本地控制器源码 `A10_new/robot.cpp` 中 `m_init` 会持续运行到关节到位或约 28000 个控制周期；该磁盘源码不证明实机一定使用同一版本，但说明不能把它视为必须立即回复的短命令。回复等待超时只表示**执行结果未知**，不证明未执行、失败或机器人已停止。客户端关闭本地控制连接以隔离迟到回复；输入 `reconnect` 只重连并查询，不补发 `m_init`。普通命令超时不产生虚假的跟随停止提醒；确有本次 follower 未确认停止时仍保留提醒。

### 4. 退出及错误处理

- 普通错误显示“操作未完成”。断线或启动时设备离线也保留 `a10>`，可继续 help/exit；设备恢复后输入 `reconnect`，再输入 `get`。重连失败仍保留控制台；未连接时控制命令不会发送。
- 当前连接仍有本次跟随待停止时，reconnect 只查状态并提示先 stop_follower，保留已有连接及停止归属。断线后的新连接不继承旧会话控制权限；如果状态仍在跟随，请通过现场控制器处理。旧跟随未确认停止时，退出仍会提醒，不能误报已停止。
- `keepalive ping timeout` 是 WebSocket Ping/Pong 保活超时，不是 get 的业务错误码。原库默认每 20 秒 Ping、等待 Pong 最多 20 秒；截图不足以区分远端未处理 Pong、网络或服务故障。适配器已禁用主动周期 Ping，get/控制回复仍有 `--timeout` 总等待上限；它不是后台断线监视器，空闲时设备断开通常在下次命令时发现。
- 单纯 manual_ds 掉使能不等于关闭网络；工控机断电或 kaanhbin 退出会使通信失效。manual_ds 报通信错误时不算掉使能确认成功，需恢复后查询状态；控制台不自动重发失败命令。
- exit、Ctrl+D、Ctrl+C 都尝试停止本次会话启动的 follower；不会停止预先已运行的其他会话的 follower，也不会自动掉使能。
- 断线、强制杀进程、第二次中断或停止超时不能保证停机。若显示“未能确认跟随停止”，请在现场控制器确认并停止，异常运动时使用急停。关闭网络不代表机器人停止。
- 单次命令失败退出码为 1，格式错误为 2；shell 正常退出为 0（不表示此前每条命令都成功），Ctrl+C 为 130，清理失败为 1。

### 5. 本机模拟测试

```bash
cd /home/robot/VLA/FluxVLA
/home/robot/miniforge3/envs/fluxvla/bin/python -m unittest discover -s test -p 'test_a10_*.py' -v
```

测试端点仅为 127.0.0.1 随机端口，不访问工控机。开发记录见 `a10_controller_follower_review.md` 和 `a10_full_migration_plan.md`。

### 6. 六轴关节 follower 的现场验证（2026-10-12）

这是新加的功能，目前通过本机模拟测试，真实关节启停和运动尚待现场验证。SSH 只读检查时未观察到 kaanhbin/9999 监听；测试前由现场确认控制程序已正常运行，其他程序未在控制机械臂。先使用第 1 节命令打开控制台。

第一轮只检查启动/停止，以下指令逐条输入，每步成功后再继续：

```text
get
set_jog_coordinate --axis
manual_en
joints
follower_aj
get
stop_follower
```

预期：无故障 → 明确选择 Axis 点动坐标系 → Enabled → 能读取六轴实际角度 → running_state=true、follower_mode=Axis → 再次查询一致 → running_state=false。点动坐标系与 follower_mode 是两个不同字段，单看点动 Axis 不证明已启动关节跟随。当前所读库中未发现关节启动必须点动 Axis 的检查；这里显式选择以方便现场辨认，不自动切换。关节启动不依赖 TCP 位姿反馈。

第二轮验证发送目标：

1. 输入 `follower_aj`，确认启动成功。
2. 输入 `joints`，打印六个**实际角度**及一条 `joints --deg ...` 指令。
3. 首次将刚打印的完整指令复制回同一控制台，作为当前实际位置的保持目标；不要用六个零代替当前姿态。程序显示“已发送”仅证明本地发包，不能确认控制器接收或已到位。
4. 核对现场姿态和 `joints` 反馈；再由现场选一个关节，对完整目标中的那个数作小幅改变，核对关节顺序、方向、幅度。六个数是绝对角度，不是每次增加的偏移量。下一次查询只输出参考文本，不会自动发送。
5. 输入 `stop_follower`，确认未运行。需要掉使能时再 `manual_ds`，确认 Disabled，最后 `exit`。

UDP wire 使用 40 字节 ARIS 消息头 + UTF-8 `jp --motor_pos=[六个度数]`，发送到 9999。Python 的公开 `send_joints({'right': ...})` 接收六轴**弧度**，终端 `--deg` 输入则是**度**；CLI 和 operator 在边界完成转换，不能把模型弧度直接当 CLI 度数。夹爪通过单独的 `g_move` 路径控制，不加入 motor_pos 作为第七轴；关节与夹爪同步执行尚未验收。

在同一连接切换回末端跟随时，先 stop_follower 并确认，再 set_jog_coordinate --tool、follower_cart。关节模式拒绝 pq，Cartesian 模式拒绝 joints 目标。启动被拒绝、模式确认超时或断线后，只读查询不恢复失败会话的发送资格；先处理停止/重连，再显式启动。reconnect 和退出清理规则同第 4 节。

目前每条目标命令只发送一个 UDP 数据报，无自动重发、持续保持或动作块调度，也没有接收到位确认。若实际响应不明显，保留 get --raw 和现场现象检查，不能据此盲目加大目标。模型动作块按频率逐步下发、序号/过期丢弃、丢包和控制器失联停止属于下一阶段。

---

## 原生 A10 推理接入（2026-10-12）

当前已经接入 **相机观测 → 模型推理 → 关节 follower 与 USB 夹爪**。2026-10-15 用户确认集成目录已跑通，现统一迁回主目录；下面从只读观测开始介绍使用方法。远程配置已按现场设置为实际执行（`dry_run=False`、`allow_motion=True`），只读示例必须显式覆盖；本节介绍推理，VR 入口见本文顶部链接。

### 1. 进入本轮开发目录

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
```

本目录已统一为 `kaanh_follower` 的主工作目录，使用 `fluxvla` Conda 环境。模型服务和机器人客户端都从本目录启动；脚本仍强制优先使用本目录代码。旧终端需要先执行上面的 `cd`，避免继续停留在已移除的工作目录中。

### 2. 配置文件分别负责什么

| 文件 | 用途 |
| --- | --- |
| `configs/a10/a10_single_openpi_inference.py` | 本机加载原来训练好的模型，并连接相机、夹爪、follower |
| `configs/a10/a10_single_remote_inference.py` | 本机接设备，通过 FluxVLA ZMQ 服务请求模型动作 |

两种方式都使用 `A10InferenceRunner → A10Operator`，不调用旧 `projects/a10` 推理循环，也不启动原 OpenPI 服务。模型名称有 OpenPI 是因为权重来自那里，运行使用 FluxVLA 原生模型。

本机已准备好忽略的资产：`checkpoints/a10_openpi_40000/`（权重、config.json、assets 内统计文件）和 `checkpoints/tokenizers/paligemma_tokenizer.model`。搬到另一台机器时要一起复制；它们没有提交进 Git。也可以设置 `A10_OPENPI_CHECKPOINT` / `A10_OPENPI_TOKENIZER` 指向准备好的资产，运行不需要旁边的 OpenPI 仓库。

USB 夹爪依赖已在 fluxvla 环境安装；新环境安装：

```bash
python -m pip install -r requirements-a10.txt
```

### 自动调试日志

A10 配置默认开启持久化调试记录，保持现有启动命令即可。客户端和模型服务各自
打印 `A10 debug log directory: ...`，输出位于
`work_dirs/a10/inference_logs/<UTC时间_随机编号>_client/` 或 `_server/`。
更新后须重启模型服务和客户端才能启用新记录。

- `metadata.json`：实际配置、模型目录/精度/实现、有效归一化统计、代码版本与源码哈希、软件版本、单位。
- `events.jsonl`：每轮状态、完整预测和选择的动作、夹爪限幅前后目标、跨块目标跳变、名义目标速度/加速度、耗时和停止原因。
- 客户端轨迹 trace：每帧计划/实际发包时间、相邻发包间隔、关节/USB发送成功标记及已有状态反馈；异常时保留部分发送结果。
- 服务端：归一化模型输出、实际状态/token/mask、反归一化目标、请求编号和模型身份；请求编号关联客户端与服务端记录。
- `console.log`：Python 日志和异常；`frames/`：原始RGB图像抽样，默认每5轮一次，另保存启动观测。
- `summary.json`：日志关闭时的丢弃事件数与写入错误数。没有该文件可能仍在运行或未正常关闭。

磁盘写入在有界后台队列中执行；关节发送循环只积累内存 trace，不增加机器人查询。
队列满会记录丢弃数，写盘失败会报告；需要连同日志健康信息检查记录是否完整。
反馈时间是本机接收时间，跨主机 monotonic 时钟不可直接比较；发送成功不等于
控制器接收或物理到位。名义目标速度/加速度也不是实测机械臂速度/加速度。

生成便于粘贴的摘要：

```bash
python scripts/a10_log_summary.py work_dirs/a10/inference_logs/实际的_client目录
```

调试时保留对应 client 和 server 两个目录；可先贴摘要和目录路径，后续可直接读取
完整动作、图像及异常定位。用 `inference.debug_log.image_every_n_cycles=0`
关闭图像抽样；用 `inference.debug_log.enabled=False` 关闭客户端记录。

### 原生 RTC 训练 50000 步模型

新模型位于 `checkpoints/FluxVLA-A10-RTC-50k/`，专用配置是
`configs/a10/a10_single_rtc_inference.py`。它严格加载 812 个原生权重张量（LoRA
已合并，不再注入），使用随包 tokenizer、PIL 缩放、训练配置中的
`mean_std` 状态/动作归一化，并将前六维关节 delta 还原为绝对目标；
夹爪仍为毫米绝对值。不能直接替换旧 OpenPI 加载器的权重目录。

先停止旧模型服务，再运行：

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_single_rtc_inference.py \
  --from-config --dtype fp32 --acceleration off --host 127.0.0.1 --port 5555
```

机器人端继续使用 `python scripts/inference.py --config configs/a10/a10_single_remote_inference.py`。
客户端默认任务已同步为训练指令：抓水瓶、放入旁边盒子、关盖并回到初始位置。
设备、执行步数及限值保留现场配置。OpenPI 配置现默认加载 `checkpoints/a10_openpi_40000/`；90000 权重仍保留作比较。40000 来自 OpenPI 的 `checkpoints/pi05_a10_finetune/40000`，已验证 811 个参数严格加载、数值有限且统计文件与源一致；该 checkpoint 未保存历史任务指令，不能将当前代码默认 prompt 当作已验证的训练记录。

这是更换 **RTC 训练权重**，当前 A10 同步 runner 尚未启用推理时的前缀条件
或异步执行。`rtc_training_config.max_delay=9` 的本仓库实现采样 **0..8** 步，
延迟前缀作为已知动作并在损失中屏蔽，并非每个样本固定丢弃前九步。
加速开关不修改此训练参数。将来接入实时 RTC 时须根据实际观测到发送的延迟
计算对应前缀，不能仅将九改成纯模型推理耗时除以动作间隔。

### 推理加速开关（2026-10-15）

为比较推理效果，`a10_single_openpi_inference.py` 默认使用普通 PyTorch
`PI05FlowMatching`。启动入口支持 `--acceleration off`（普通 PyTorch）和
`--acceleration on`（原生 Triton + CUDA Graph，BF16 融合内核）。省略开关
时使用配置默认值。两条路径使用同一份 checkpoint、归一化统计和 tokenizer，
均为 10 次去噪、30 步预测。此处 OpenPI 配置默认使用 40000 步权重，新 RTC 权重使用上节专用配置。

远程推理把开关加在**模型服务**命令上，切换时先 Ctrl+C 退出旧模型服务，再重启：

```bash
# 普通 PyTorch FP32，用于本轮效果对比
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_single_openpi_inference.py \
  --from-config --dtype fp32 --acceleration off --host 127.0.0.1 --port 5555

# Triton + CUDA Graph；--dtype fp32 关闭外层 autocast，融合内核仍为 BF16
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_single_openpi_inference.py \
  --from-config --dtype fp32 --acceleration on --host 127.0.0.1 --port 5555
```

服务启动打印实际模型类型：`PI05FlowMatching` 为普通路径，
`A10PI05FlowMatchingInference` 为加速路径。机器人端仍运行
`python scripts/inference.py --config configs/a10/a10_single_remote_inference.py`，
不要把加速开关加到远程客户端，它不能切换已运行的服务。
本机推理可使用 `python scripts/inference.py --config configs/a10/a10_single_openpi_inference.py --acceleration off`（或 `on`）。

普通路径通常更慢，可能拉长动作块之间的等待；关闭加速不等于修复动作衔接。
首次加速预测仍包含内核准备和图录制。历史加速耗时与数值对照见迁移记录，
不代表普通路径的性能或实机任务成功率。

### 3. 可以先只启动模型服务，不接设备

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_single_openpi_inference.py \
  --from-config --dtype fp32 --host 127.0.0.1 --port 5555
```

看到 `Server is ready` 说明模型服务已就绪。第一次加载和第一次预测会比较慢。这个入口不构建机器人 operator，不连接或使能机械臂。

### 4. 设备到位后，先运行只读观测和推理

工控机的 kaanhbin 需已运行；先核对示例配置中的两路相机、夹爪串口路径。不要同时运行旧夹爪服务占用同一串口。夹爪当前使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，换 USB 插口通常无需改配置；同型号无独立序列号的多个转接器同时接入时需重新核对。相机分辨率和腕部旋转应与训练一致。

本机推理：

```bash
python scripts/inference.py \
  --config configs/a10/a10_single_openpi_inference.py \
  --cfg-options inference.operator.host=192.168.0.125 \
    inference.dry_run=True inference.operator.allow_motion=False
```

使用上一步启动的 ZMQ 服务（另开一个终端进入相同目录/环境）：

```bash
python scripts/inference.py \
  --config configs/a10/a10_single_remote_inference.py \
  --cfg-options inference.operator.host=192.168.0.125 \
    inference.dry_run=True inference.operator.allow_motion=False
```

以上只读示例显式设置 `dry_run=True` 和 `allow_motion=False`：读取设备实际反馈、预测并检查动作，但不发关节/夹爪运动目标，不使能、不启动跟随。没有两路相机或 USB 夹爪时会报错，不会拿假图像/零位置代替。仅启动模型服务不要求设备在线。

### 5. 真正执行动作时的参数

先完成上面的只读步骤，确认关节顺序/弧度、夹爪毫米、相机画面和现场控制器设置。远程配置已按用户现场设置保存 `dry_run=False`、`operator.allow_motion=True`、`max_joint_step_rad=0.2`；直接运行远程配置会发送运动目标，无需再输入覆盖参数。电机使能仍由现场用户使用控制台单独完成，runner 不自动使能或回零。下表为当前远程配置；本机配置可能不同。

| 参数 | 当前默认含义 |
| --- | --- |
| `action_chunk=30` | 模型一次预测 30 步，和 checkpoint metadata 对齐 |
| `execute_horizon=15` | 每轮只发送前 15 步，然后重新拍照、读取状态、预测 |
| `action_dt=1/30` | 原数据每秒 30 帧，每个模型目标的持续时间约 33.3ms |
| `joint_target_repeat=2` | 当前默认：每个关节目标以约 16.7ms 间隔发两遍，UDP 60Hz；USB夹爪仍每个模型目标发一次。设为1恢复30Hz |
| `max_publish_step=3000` | 本次运行累计选择 3000 步后结束 |
| `max_joint_step_rad` | 检查第一步相对实际观测，以及随后各步的关节目标变化 |
| `gripper_mm_range=(0,85)` | 默认 USB 夹爪工作范围；模型预测超出范围时限制到边界并打印提示，不把毫米当速度 |

`execute_trajectory()` 按模型时间逐行发送；当前每行前六个关节数通过 UDP 9999
在 `t` 和 `t+1/60秒` 各发一次，下一行在 `t+1/30秒` 开始；夹爪仅第一遍写本机USB。
15个模型目标对应30个UDP包，计划时长仍为0.5秒（另加状态检查耗时），不是把动作加速两倍。
日志保留模型 `selected/sent` 计数，并增加 `joint_packets_sent`、`gripper_targets_sent`、
`joint_publish_hz`、逐包 `action_index/repeat_index`，用于核对60Hz实际间隔。
超时/取消后不突发补发重复包。推理等待期间的跨块发包空档仍存在。
机器人内部插补/运动规划由控制器负责；UDP没有每帧ACK，发送结束不证明实际到位。
提高刷新频率是否改善跟随须现场对比，不能仅据频率认定运动速度加倍。

推理配置用 WebSocket 5888 每 0.05 秒请求一次 `get` 并保存最新状态；5999 负责 follower 启停，UDP 9999 仍逐行发送六轴目标。拍照推理和动作块检查读取最近状态，超过 0.5 秒未收到新反馈、监视连接断开、出现故障或跟随模式变化就停止发新目标。状态查询和动作发送互不占用同一 WebSocket 请求；USB 夹爪仍单独读写。查询时间和相机／夹爪时间不构成严格硬件同步；UDP 仍没有每帧确认。手动控制台没有启用后台监视时保留同步查询方式。

Ctrl+C 请求停止当前会话的 follower 并关闭资源；`p` + Enter 在当前动作块完成后结束。停止失败会明确输出未确认日志，需现场处理。关闭串口/网络不代表机械臂/夹爪物理停止，也不自动掉使能。

### 6. 预测关节单步变化超限时

若看到 `Joint target step exceeds configured limit`，该轮在 `start_follower()` 之前结束，没有发送运动目标。这里比较的是第一步相对实际关节角，以及后续每一步相对前一目标的最大变化；单位是弧度。不要仅为消除报错而调大限值。

用现有模型服务执行下面的只读诊断，程序会打印 30 步中最大变化发生在第几步、哪个关节、变化量与前后角度：

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
python scripts/inference.py \
  --config configs/a10/a10_single_remote_inference.py \
  --cfg-options \
    inference.operator.host=192.168.0.125 \
    inference.operator.allow_motion=False \
    inference.dry_run=True \
    inference.execute_horizon=30 \
    inference.max_publish_step=30
```

`dry_run=True` 不会启动 follower 或发送关节/夹爪目标；设备仍需能读取状态、相机和 USB 夹爪。记录 `A10 dry-run planned largest joint step` 的值与位置，再由现场核对目标角度、训练数据动作语义和允许的控制限值。

首次现场 30 步运行已确认控制器收到 30 条六轴目标，但模型目标全程每轴仅变化约 0.08°～0.26°；实际关节跟随量也很小。`max_joint_step_rad` 是拒绝超限目标的检查，不会放大或缩小模型动作。用户停止后只读 `get` 显示 follower 未运行、Stopped、无故障，电机仍 Enabled。 最新现场限值为 `max_joint_step_rad=0.1` 弧度，仍不改变模型实际预测的幅度。后续只读相机检查显示画面中有水瓶、纸箱、金属件，没有黄色柠檬；当前模型默认指令是 `Reach the yellow lemon`，应先核对任务与训练场景是否对应。

### 7. 本机测试（不连接真机）

2026-10-12：一次设置 `max_publish_step=3000` 的运行实际在首块 preflight 的 `get` 查询超时退出，未发送本块运动目标。旧 15ms 查询预算低于只读实测的部分响应（20 次中最大约 35.8ms）；现已改为 100ms，并暂停周期内查询的发送计时。报错显示明确的命令和超时秒数，runner 同时打印 `last_trajectory` 的 `sent`，设定的上限不代表已经执行。修复后先用原命令将 `inference.max_publish_step` 设为 30，确认出现 `A10 cycle` 且机器人反馈正常，再延长。若出现 `follower stop failed`，需通过现场控制器确认停止。

```bash
python -m unittest discover -s test -p 'test_a10_*.py' -q
python -m unittest discover -s test/test_runners -p 'test_a10_*.py' -q
python -m unittest discover -s test -p 'test_camera_operator.py' -q
python -m unittest discover -s test -p 'test_gripper_operator.py' -q
```

可选的实际 GPU 权重测试（仍不连接机器人）：

```bash
A10_TEST_REAL_MODEL=1 python -m unittest discover \
  -s test/test_runners -p 'test_a10_openpi_inference.py' -q
```

该项检查本地原生推理与实际 ZMQ 收发一致，输出 `30×7` 且数值有限。不能替代原 JAX 模型的逐数值对照、任务成功率或实机验收。

## 旧版项目流程现场说明（相机、VR、录制与旧 TCP 接口）

下面保留早期迁移阶段的流程，其分支和配置与上面的原生 follower 控制台不同。验证本次 WS/UDP 接口时使用上面的命令。

当前平台已通过离线流程检查。机械臂及 OpenPI 仍由其他同事开发，现场验证按用户要求延期。以下步骤供设备空闲、有人在场配合时执行。

## 1. 环境和站点配置

```bash
source /home/robot/miniforge3/etc/profile.d/conda.sh
conda activate fluxvla-thor
cd /home/robot/VLA/FluxVLA
git branch --show-current
python -m fluxvla.projects.a10 doctor
cp -n configs/a10/a10_single_site.example.yaml configs/a10/a10_single_site.local.yaml
```

分支应为 `kaanh`。`cp -n` 保留已有站点文件；编辑 `a10_single_site.local.yaml` 后会自动加载，它已被 Git 忽略。模板中的相机名是占位符，地址是回环地址，须按现场修改。

| 字段 | 应填写什么 |
| --- | --- |
| `cameras.base_0_rgb.device` | 外部视角相机的设备路径 |
| `cameras.right_wrist_0_rgb.device` | 腕部相机的设备路径，不能和 base 指向同一节点 |
| `rotate_180` | 默认 base=false、right_wrist=true，现场看图确认 |
| `robot.host` / `port` | 运行 A10_new 控制服务的机器地址及端口，与控制器同事确认 |
| `vr_server.host_ip` | 运行 FluxVLA 的机器局域网地址，用 `ip -brief address` 查看 |
| `robot.allow_motion` | 准备阶段保持 `false` |
| `max_joint_step_rad` / `gripper_mm_range` | 现场确认的单步关节上限和夹爪毫米范围，准备阶段可保持 null |

控制器地址与 FluxVLA 地址可能不同。`doctor` 检查软件和文件，不连接机器人或打开相机；它成功时仍可能显示相机“待连接”。要使用其他站点文件，将 `--site 文件.yaml` 放在子命令前。

## 2. 检查相机并核对画面

```bash
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
python -m fluxvla.projects.a10 check-hardware --cameras-only
python -m fluxvla.projects.a10 check-hardware --cameras-only --frames 60 --execute
```

优先使用 by-id 稳定路径；没有唯一序列号的同型号相机可用 by-path，并保持 USB 接口不变。一个相机可能有多个节点，需选能输出图像的节点。没有 `--execute` 时仅预览命令。

终端打印报告位置 `work_dirs/a10/hardware_checks/<时间>/report.json`。成功时同目录有两张 PNG，已应用配置中的旋转；人工核对两路视角、上下方向、清晰度和遮挡。

`paired_capture_fps` 是两路连续读取吞吐，排除了打开／预热，不包含机器人、VR 和录像写盘耗时，也不能证明传感器没有重复画面。`meets_90_percent_target=false` 提示低于目标的 90%，最终须检查真实录制频率。

设备缺失、两个角色指向同一节点、尺寸异常或驱动超时会失败。默认每项检查最长 60 秒；检查设备路径和占用情况后重跑，不关闭同事正在使用的进程。

## 3. 查询 A10 状态

与控制器同事协调好后执行：

```bash
python -m fluxvla.projects.a10 check-hardware --robot-only --execute
```

此命令只发送 GET 查询：空闲状态、7 维反馈、再次检查空闲状态。控制器忙时会停止检查。前六个反馈应为关节弧度，最后一个应为夹爪毫米值，需现场核对实际姿态、顺序及单位。

自动通过只证明协议与有限数值，不能确认反馈新鲜度或物理限值。平台互斥锁只约束本机通过统一入口启动的任务，无法协调其他电脑／仓库的客户端。`check-hardware --execute` 默认检查相机和机器人；控制器开发期间使用 `--cameras-only`。

## 4. 验证 VR 输入

```bash
python -m fluxvla.projects.a10 teleop --vr-only --execute
```

此模式不连接机器人。头显打开终端打印的 `https://本机局域网地址:8443` 并进入 VR。首次启动会在 `work_dirs/a10/vr/tls/` 生成本机证书，浏览器可能需要信任证书；手柄输入使用 WebSocket 端口 8442。页面能打开但没有输入时，检查 WebSocket、证书和终端诊断。网页还需要从 CDN 加载 A-Frame。

现场核对手柄跟踪、使能／松开、重锚和掉帧保护。结束时按 Ctrl+C。实际动臂验证待控制器团队方便配合时进行。

### LeRobot 能连接，但 FluxVLA 头显浏览器连不上

先看终端打印的页面地址。若是 `https://127.0.0.1:8443`，服务只允许电脑本机访问，头显不能连接。原 XLeVR 默认监听 `0.0.0.0`，而平台共享默认值是回环地址，因此需要设置站点地址。

用 `ip -brief address` 查出这台电脑当前的局域网地址，在 `configs/a10/a10_single_site.local.yaml` 中加入或修改下面两行，然后重启 `teleop --vr-only --execute`：

```yaml
vr_server:
  host_ip: 192.168.110.119  # 此次排查时的本机地址，换网络后须重新确认
```

头显打开 `https://192.168.110.119:8443`。FluxVLA 使用自己的证书，第一次访问可能需要重新信任；旧 LeRobot 证书的信任不代表已信任平台证书。若页面能打开但输入没有连接，核对终端打印的 `wss://地址:8442` 及头显证书状态。不要同时启动占用相同端口的 LeRobot 和 FluxVLA VR 服务。

`ss -ltnp` 应显示 HTTPS 8443 和 WebSocket 8442 绑定在本机局域网地址上。本机通过该地址测试成功，仍不能代替实际头显对网络、证书及 WebXR 的验证。

## 5. 录制一小段真实数据

这一步会发送控制命令。现场核对设备和控制接口后，填写已确认的 `max_joint_step_rad`、`gripper_mm_range`，再将 `allow_motion` 设为 `true`。配置允许的范围为关节步长 `(0, 1]` 弧度、夹爪上下界在 `0–100` 毫米内，实际值须按机器确认。

这些关节限值用于模型批次检查；VR 发的是末端增量，依赖已有 VR 输入保护和 A10_new 控制。因此关节限值不代替 VR 的现场验收。

```bash
# 先预览，确认现场条件后再加 --execute
python -m fluxvla.projects.a10 record --root datasets/a10/onsite_trial_001 --task pick_and_place --episodes 1
python -m fluxvla.projects.a10 record --root datasets/a10/onsite_trial_001 --task pick_and_place --episodes 1 --execute
python -m fluxvla.projects.a10 validate --root datasets/a10/onsite_trial_001 --execute
```

终端 Enter/n 或左摇杆向右：开始／保存；r 或左摇杆向左：放弃本集重录；s 或左摇杆向下：结束。至少录两帧；已有数据集需显式 `--resume` 才续录。控制器已接受的动作可能继续执行，软件退出不能替代现场停机手段。

## 6. 留下验收记录

在 [开发记录](a10_full_migration_plan.md) 追加 commit、站点配置摘要、设备对应关系、报告位置、实际帧率、现场核对结果与待办。通过后再确定正式训练的数据／任务划分、时长和评估方法。

目前模型只训练两步，已出现夹爪越界预测，只证明软件流程连通，不能用来直接控制机器人。正式训练与实机效果均尚未验收。


2026-10-12 启动修复：后台监视配置下，连接时首次真实状态读取也来自 5888；5999 用于跟随启停及其原有确认。若启动报监视端状态超时，尚未执行动作，需检查控制器服务；不会以默认状态继续运动。

模型夹爪输出允许出现有限的边界偏差，runner 将其限制在 gripper_mm_range（默认 0～85mm）；关节目标不因此改变，手动夹爪目标越界仍报错。


持续推理：在原运行命令中设置 `inference.max_publish_step=None`，取消总步数上限。输入 p 后回车在当前块结束后停止；Ctrl+C 请求停止 follower。当前没有自动识别抓取成功，完成任务后需现场停止。

来回拉动诊断：A10 cycle 增加 observed_qpos（推理前实际状态）、previous_last_action（上一块发送的末目标）、first_action/last_action（本块首尾）、tracking_error_at_observation_rad（实际六轴减上块末目标）。这些日志用于区分观测时残余跟踪误差与模型目标跳变；同步发送不等于机械臂已到位。


## 双臂VR录制与手柄复位（2026-11-06）

采集入口已接入，使用右手A作为功能键、左摇杆四方向操作录制。先进行无设备按钮检查，现场再验证录制与m_init。请按[双臂VR数据采集指南](a10_vr_recording.md)执行；保存/停止采样不等于停止机械臂，复位另行确认。
