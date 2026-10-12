# A10 现场接入操作说明

## 原生 follower 命令行控制台（2026-10-09）

本节用于直接连接工控机上的 KAANH follower。命令解析放在 `scripts/a10_control.py`，机器人通信仍由 `fluxvla/engines/operators/a10_operator.py` 完成。

### 1. 打开控制台

工控机的 kaanhbin 需已正常运行；已运行时不要再启动一份。以下命令在运行 FluxVLA 的机器终端执行：

```bash
cd /home/robot/VLA/FluxVLA_integration
/home/robot/miniforge3/envs/fluxvla/bin/python scripts/a10_control.py --host 192.168.110.124 shell
```

当前 **kaanh_follower** 分支位于 `/home/robot/VLA/FluxVLA_integration`。主目录 `/home/robot/VLA/FluxVLA` 的 `vr` 分支及其未提交修改保留，`FluxVLA_runner` 仍是推理迁移来源；早期 `FluxVLA_follower` 为旧版本 detached worktree。

出现 `a10>` 后输入短指令即可。启动只连接并查询，不会自动使能、启动跟随或发送运动目标。

只想查询一次可运行：

```bash
/home/robot/miniforge3/envs/fluxvla/bin/python scripts/a10_control.py --host 192.168.110.124 get
```

只读交互模式：在 shell 前加 `--read-only`。查看帮助用 `--help`，查看指令列表用 `--host 192.168.110.124 help`；帮助不连接设备。默认 WS 5999、Cartesian UDP 9998、关节 UDP 9999、超时 5 秒，可在 shell/get 前用 `--ws-port`、`--udp-port`、`--joint-udp-port`、`--timeout` 修改。两路 UDP 端口不能相同。

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

- 若 get 报警，先在控制器界面核对并清除，不继续后面的控制命令。本控制台不提供猜测的清错命令。
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
| `exit` | 尝试停止本次跟随并关闭连接，不自动掉使能 |

pq 的 xyz 单位是 **米**，四元数为 **xyzw** 且长度须为 1；采用附件模板的相对启动基准目标，不是绝对位置，也不是“每输入一次就再走这么远”。七个零不是有效四元数。该控制台不累加、不插值、不做坐标变换；规划属于控制器。单次发送没有应用层 ACK，显示“已发送”不证明收到或执行。具体方向、尺度、旋转组合和接收频率需要现场单独验收，未验证前不要直接套用其他机器人的目标数值。

get 没有实际夹爪字段时显示“暂无反馈”，不填假数据。夹爪尚未验证到位条件。控制台不后台记录数据、不新增 work_dirs 结果目录；需要开发排查时可手动保留终端输出。

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
cd /home/robot/VLA/FluxVLA_integration
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

这一轮完成的是 **推理 → follower 的软件链路**。相机、夹爪和控制器的真实联动还没有验收；下面先使用只读观测模式。VR 未纳入本轮。

### 1. 进入本轮开发目录

```bash
cd /home/robot/VLA/FluxVLA_integration
conda activate fluxvla
```

本目录是 `kaanh_follower` 分支的独立 worktree。`FluxVLA` 主目录仍是 `vr`，其中的未提交修改保留；`FluxVLA_runner` 是迁移来源分支。这里的脚本强制使用本目录代码，避免 editable 安装指向其他 worktree。

### 2. 配置文件分别负责什么

| 文件 | 用途 |
| --- | --- |
| `configs/a10/a10_openpi_inference.py` | 本机加载原来训练好的模型，并连接相机、夹爪、follower |
| `configs/a10/a10_remote_inference.py` | 本机接设备，通过 FluxVLA ZMQ 服务请求模型动作 |

两种方式都使用 `A10InferenceRunner → A10Operator`，不调用旧 `projects/a10` 推理循环，也不启动原 OpenPI 服务。模型名称有 OpenPI 是因为权重来自那里，运行使用 FluxVLA 原生模型。

本机已准备好忽略的资产：`checkpoints/a10_openpi_90000/`（权重、config.json、assets 内统计文件）和 `checkpoints/tokenizers/paligemma_tokenizer.model`。搬到另一台机器时要一起复制；它们没有提交进 Git。也可以设置 `A10_OPENPI_CHECKPOINT` / `A10_OPENPI_TOKENIZER` 指向准备好的资产，运行不需要旁边的 OpenPI 仓库。

USB 夹爪依赖已在 fluxvla 环境安装；新环境安装：

```bash
python -m pip install -r requirements-a10.txt
```

### 3. 可以先只启动模型服务，不接设备

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_openpi_inference.py \
  --from-config --dtype fp32 --host 127.0.0.1 --port 5555
```

看到 `Server is ready` 说明模型服务已就绪。第一次加载和第一次预测会比较慢。这个入口不构建机器人 operator，不连接或使能机械臂。

### 4. 设备到位后，先运行只读观测和推理

工控机的 kaanhbin 需已运行；先核对示例配置中的两路相机、夹爪串口路径。不要同时运行旧夹爪服务占用同一串口。夹爪当前使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，换 USB 插口通常无需改配置；同型号无独立序列号的多个转接器同时接入时需重新核对。相机分辨率和腕部旋转应与训练一致。

本机推理：

```bash
python scripts/inference.py \
  --config configs/a10/a10_openpi_inference.py \
  --cfg-options inference.operator.host=192.168.110.124
```

使用上一步启动的 ZMQ 服务（另开一个终端进入相同目录/环境）：

```bash
python scripts/inference.py \
  --config configs/a10/a10_remote_inference.py \
  --cfg-options inference.operator.host=192.168.110.124
```

示例保留 `dry_run=True` 和 `allow_motion=False`：读取设备实际反馈、预测并检查动作，但不发关节/夹爪运动目标，不使能、不启动跟随。没有两路相机或 USB 夹爪时会报错，不会拿假图像/零位置代替。仅启动模型服务不要求设备在线。

### 5. 真正执行动作时的参数

先完成上面的只读步骤，确认关节顺序/弧度、夹爪毫米、相机画面和现场控制器设置。执行需要显式设置 `dry_run=False`、`operator.allow_motion=True` 和已现场确认的 `max_joint_step_rad`；不提供未经核对的限值。电机使能仍由现场用户使用控制台单独完成，runner 不自动使能或回零。

| 参数 | 当前默认含义 |
| --- | --- |
| `action_chunk=30` | 模型一次预测 30 步，和 checkpoint metadata 对齐 |
| `execute_horizon=10` | 每轮只发送前 10 步，然后重新拍照、读取状态、预测 |
| `action_dt=1/30` | 原数据每秒 30 帧，连续训练目标的间隔为约 33.3ms |
| `max_publish_step=30` | 本次运行累计选择 30 步后结束 |
| `max_joint_step_rad` | 检查第一步相对实际观测，以及随后各步的关节目标变化 |
| `gripper_mm_range=(0,85)` | 默认 USB 夹爪工作范围；不越界裁剪，不把毫米当速度 |

`execute_trajectory()` 是把 10 行动作按时间逐行发送：前六个数走关节 UDP 9999，第七个数走本机 USB 夹爪。一个周期发送一次各自目标，机器人内部插补/运动规划由控制器负责；UDP 没有每帧 ACK，返回完成仅代表发送流程结束，不能据此证明实际到位。

动作块开头、每约 0.1 秒和结尾有 WS/USB 状态检查；WS 检查超时默认 100ms，夹爪配置读写超时仍为 15ms。周期内查询期间暂停发送计时，查询结束后继续逐行发送，不跳帧、不突发补发；因此动作块总耗时会大于 T×dt，查询期间控制器保持上一个目标，实际效果需现场确认。开始时隙的容许滞后默认 `dt/2`。故障、断线、取消、真正超时仍中止，记录在 operator 的 `last_trajectory`，不是硬实时保证。

Ctrl+C 请求停止当前会话的 follower 并关闭资源；`p` + Enter 在当前动作块完成后结束。停止失败会明确输出未确认日志，需现场处理。关闭串口/网络不代表机械臂/夹爪物理停止，也不自动掉使能。

### 6. 预测关节单步变化超限时

若看到 `Joint target step exceeds configured limit`，该轮在 `start_follower()` 之前结束，没有发送运动目标。这里比较的是第一步相对实际关节角，以及后续每一步相对前一目标的最大变化；单位是弧度。不要仅为消除报错而调大限值。

用现有模型服务执行下面的只读诊断，程序会打印 30 步中最大变化发生在第几步、哪个关节、变化量与前后角度：

```bash
cd /home/robot/VLA/FluxVLA_integration
conda activate fluxvla
python scripts/inference.py \
  --config configs/a10/a10_remote_inference.py \
  --cfg-options \
    inference.operator.host=192.168.110.124 \
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
cp -n configs/a10/site.example.yaml configs/a10/site.local.yaml
```

分支应为 `kaanh`。`cp -n` 保留已有站点文件；编辑 `site.local.yaml` 后会自动加载，它已被 Git 忽略。模板中的相机名是占位符，地址是回环地址，须按现场修改。

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

用 `ip -brief address` 查出这台电脑当前的局域网地址，在 `configs/a10/site.local.yaml` 中加入或修改下面两行，然后重启 `teleop --vr-only --execute`：

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
