# Wall-X API 接入记录（A10，2026-11-25）

实验分支：`experiment/wallx-api`，从 `inference_1118` 新建；现场原有未提交改动保留。

已完成鉴权、二进制 MessagePack 客户端，以及复用遥操作 SDK Tool 通道的双臂 EE 执行闭环。
离线双周期闭环验证通过，云端握手和一次真实观测推理实测成功。**尚未完成实机动作验收**：
原生基座实验映射使右臂坐标超出模型训练范围，预测首步跳变被本地执行检查拒绝。
云端能力列表没有 A10，现有实验映射未标定，没有下发机器人动作。

## 本机直接使用

已在 `work_dirs/wallx/venv` 创建环境，复用现有 FluxVLA 依赖并单独安装 `msgpack-numpy`。
不需要切换当前机器人推理环境。

```bash
work_dirs/wallx/venv/bin/python scripts/wallx_client.py --probe \
  --output work_dirs/wallx/probe.json
```

本次实测：HTTP `101`，`action_horizon=32`，`row_dt_s=0.0337`，顶层 `valid_rows=31`。
服务公布了 Quanta、TeleTwin、YAM、AgileX、Franka 等本体，没有 A10。
完整 metadata 存在 `work_dirs/wallx/probe.json`，该目录已被 Git 忽略。
生产 metadata 没有官方示例中的 `model` 字段，客户端以能力列表检查握手；如果存在 `model`，则校验它。

个人 key 已保存到仓库根目录 `.env.wallx`，权限 `0600`，Git 已忽略。
进程环境变量 `WALLX_API_KEY` / `WALLX_URL` 优先于文件。不要将真实 key 写到命令行或请求 JSON 中。
配置格式参考 [示例](../.env.wallx.example)。客户端仅向官方固定地址发送 key，默认直连；
现场确实需要系统代理时加 `--use-proxy`。

另一台机器可在自己的虚拟环境中安装：

```bash
python -m pip install -r requirements-wallx.txt
```

## 单次推理到文件

仅在已确认本体及状态语义时使用。请求 JSON 使用官方 public schema；
`observation.images` 的值在 JSON 中写成相对该 JSON 的 JPEG 文件路径，脚本读取后发送二进制 JPEG。
双臂请求需要 `front`、`left_wrist`、`right_wrist` 三路图像和 `left_ee`、`right_ee` 两臂状态。
每臂状态含 `position`（3 个米值）、`rotation_rpy`（3 个弧度值）和 `gripper`。
顶层需要已支持的 `embodiment`、`normalization: "provided"` 和非空 `instruction`。

```bash
work_dirs/wallx/venv/bin/python scripts/wallx_client.py \
  --request /path/to/request.json --timeout 120 \
  --output work_dirs/wallx/inference.json
```

客户端先收 metadata，再发 MessagePack bin 请求，验证返回本体、32 帧动作形状及有限数值；
超时或服务端错误后关闭连接。结果中的末端位置和姿态保持原始语义，不能当作关节角。
独立客户端只保存结果；下述 EE 入口实现同步分块闭环。根据官方新版 `action_public.decode_response`，
public-v2 返回第 0 行就是未来动作，不能当作内部 RTC 的观测锚点丢弃。
有效行数取响应 `valid_rows`，其次响应 `rtc.valid_rows`，未提供时取完整 32 行；
握手中的内部 RTC `valid_rows=31` 不覆盖 public 响应语义。
2026-11-25 18:03 已取得一次 A10 实时观测的云端预测，动作检查拒绝原因见下文。

## 网上查明的模型映射与剩余参数

现有 A10 推理运行器接收 `[T,14]` 关节角及夹爪闭合比例，而公共 API 返回分臂末端绝对位姿。
A10 控制器存在 TCP 与笛卡尔通道，但这不能证明其坐标系、工具偏置、夹爪定义与任何已支持本体一致。

官方 [API 配置说明](https://x2robot.com/docs/quantax1pro/v1/model-development/wallx-api/client-inference-configuration)
要求输入米、弧度物理量，`normalization="provided"` 由服务按 `embodiment` 选择统计参数，
客户端不再做数值归一化。公开云端没有提供 A10 的本体名或上传自定义统计参数的接口。
开源自部署代码可以加载自定义统计量，这与个人云端 API 是两条不同路径。

新版官方 [执行端规范](https://github.com/X-Square-Robot/xr-harness/blob/main/xr-cerebellum/docs/reference/VLA_BACKENDS.md)
和 [位姿/夹爪契约](https://github.com/X-Square-Robot/xr-harness/blob/main/xr-cerebellum/docs/reference/VLA_POLICY_FRAMES.md)
明确区分原生参考系、工具点、标准开度和模型夹爪量。TeleTwin 的原生参考系可以作为实验约定的参考，
但没有给出 A10 到训练本体的已标定变换。A10 侧几何已落实，以下内容仍需标定或确认：

- A10 应使用哪个 `embodiment`，或是否需要新增 A10 归一化参数；不能直接冒用 TeleTwin/YAM。
- 模型输入与输出所用的基座坐标、TCP/工具定义和夹爪语义，以及如何与下面已查明的 A10 定义对应。
- A10 跨本体策略是否适用，以及实际模型参考点和夹爪行程的对应关系。

已实现 A10 侧转换和执行器；几何往返通过不能证明跨本体策略适用。
没有给现有 `--model` 增加一个无法执行的 Wall-X 选项，原有推理入口继续使用原配置。

## A10 参数核对结果

本次已读取仓库的 `robot_hardware.jpg`，可确认双臂、中央外部相机和夹爪的布置；
没有从单张照片推算精确尺寸。用户最终确认桌面左右宽 1.5 m、前后深 1.0 m，
覆盖此前 2 m / 1.3 m 的描述。

已通过本机共享状态接口（存在时）或控制器 `5999` 仅发送 `get`，并通过 `5888` 的
`get_tool --ee_id=… --tool=…` 查询两臂工具定义。没有发送设置工具、使能或运动指令。
原始只读反馈、工具查询与提取记录分别保存在：

- `work_dirs/wallx/a10_status.json`
- `work_dirs/wallx/a10_tools.json`
- `work_dirs/wallx/a10_hardware_audit.json`

| 项目 | 查明的值 | 来源与适用范围 |
| --- | --- | --- |
| 物理左臂 | `ee_id=1`，operator 字典键 `right`，TCP 第 1 行 | `a10_dual_vr.py` / `a10_dual_devices.py` 的现场绑定 |
| 物理右臂 | `ee_id=0`，operator 字典键 `left`，TCP 第 0 行 | 同上；不能直接按 operator 字典键解释物理左右 |
| 基座相对朝向 | 绕 Z 相差 180° | 世界平移遥操作轴映射；不含基座平移和安装高度 |
| 基座中心距 | 1.15 m，两基座同高、前后对齐 | 用户已确认；与查看器中心距一致 |
| 当前工具 / 工件 | 两臂均 `tool=0`、`wobj=0` | 实机只读快照；后续运行需重新核对 |
| tool0 | 零偏置 | 本地 XML 和实机工具查询一致；这是控制器工具参考点，不能直接称为指尖 |
| tool1 | 相对 tool0 沿工具 Z 轴 +0.295 m，旋转为零 | 两臂本地 XML 和实机查询一致；用户已确认对应当前夹爪抓取中心 |
| TCP 反馈格式 | `pq=[x_mm,y_mm,z_mm,qx,qy,qz,qw]` | 同帧 `pe` 的 Euler321 角（度）转四元数与 PQ 一致，误差小于 3e-16 |
| 控制器模型基座 | 两臂各自 `base_pe=0`，`wobj0=0` | 本地 XML 和当前独立模型；没有编码双臂共同工作台坐标 |
| 关节位置限位 | 12 个 `MotorConfig` 均为 −175° 到 +175° | 本地 `kaanh.xml` 配置值；不代表读取过当前运行时限位或已验证的防碰撞范围 |
| 夹爪开合 | raw 0 完全张开，9000 完全闭合；闭合比例 `raw/9000` | 现有夹爪程序与现场配置；实际毫米开口未标定 |
| 桌面 | 左右宽 1.5 m、前后深 1.0 m；桌面高于基座安装平面 0.20 m | 用户已确认尺寸与高度；桌边相对基座的位置尚未测量 |

TCP（Tool Center Point）是工具上用于定位的参考点及其坐标轴。
`tool1` 的 295 mm 描述的是一个几何偏置，夹爪 Modbus 开合程序描述的是手指开合；
两者不能互相替代。`set_jog_coordinate --tool` 选择运动表达坐标系，不能据此认定已切换到 `tool1`。

现有 VR 使用相对增量，接管时的位置原点在差分中抵消。因此 VR 能正常工作，并不能补全
工作台到两个基座的绝对平移、桌面高度或模型所需的工具参考点。
工作台长宽也不能直接作为两臂都能到达的动作范围；还需检查关节限位、可达性和碰撞。

## 已实现的本地几何转换

参数在 `configs/a10/a10_wallx_geometry.json`，实现位于 `fluxvla/projects/a10/wallx.py`。
物理左右、相机绑定以及基座朝向沿用已核对的遥操作配置。

统一坐标 `a10_rig` 定义为：原点在两基座安装中心的中点，X 向工作台前方，Y 向物理左臂，Z 向上。
左基座在 `[0,+0.575,0]`，朝向为零；右基座在 `[0,-0.575,0]`，绕 Z 转 180°。
桌面 Z 为 `0.20 m`。这是一套明确的 A10 本地坐标定义，尚未假定它等于任何 Wall-X 本体的训练坐标。

转换按完整刚体变换处理：

```text
T_rig_grasp = T_rig_base × T_base_active_tool × T_active_tool_grasp
T_base_active_tool = inverse(T_rig_base) × T_rig_grasp × inverse(T_active_tool_grasp)
```

实际反馈为 `tool0` 时，沿工具自身 Z 轴加 295 mm；实际反馈为 `tool1` 时不再重复添加偏置。
转出的 RPY 使用 `Rz(yaw) × Ry(pitch) × Rx(roll)`；毫米到米只转换一次。
几何模块拒绝未标定的工具、非 `wobj0`、故障反馈及无效四元数。
夹爪 raw 到闭合比例的本地转换已实现；闭合比例到目标模型夹爪语义的映射仍单独待验证。

读取一帧实机状态并生成预览：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_preview.py --live-read-only
```

该命令优先复用已运行的共享状态接口，否则建立仅发送 GET 的状态连接；不启动共享服务、
不获取运动控制权，不切换工具或坐标，也不发 UDP。结果在 `work_dirs/wallx/a10_geometry_preview.json`。
离线回放已有反馈：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_preview.py \
  --feedback work_dirs/wallx/a10_status.json
```

实机只读快照计算的抓取点高度：左侧高于桌面约 0.1559 m，右侧约 0.1754 m；
本地往返位置误差小于 2e-16 m，旋转误差小于 1e-15 rad。
这些是同一帧数学转换的一致性结果，不是抓取点外部测量精度或轨迹执行精度。
以上几何采样阶段没有向模型发送观测；随后已完成下述一次推理。`model_contract.verified` 仍为 false。

## EE 闭环入口

入口 `scripts/a10_wallx_inference.py`，参数 `configs/a10/a10_wallx_inference.py`。
完整流程为新鲜三路相机/机器人/夹爪观测 → 云端同步推理 → 重读状态并检查推理期间漂移 →
整段动作与插值检查 → 遥操作同款 Tool/Cart SDK 轨迹 → 反馈确认到位 → 下一轮观测。
默认每轮执行前 6 个未来动作，每个动作插 3 步，约 89 Hz。只有 `motion` 模式会启动 follower。
复用 joint 推理的相机、USB 夹爪线程、超时和停止清理，EE 数据不走 joint/timed_aj 通道。

不连接设备的双周期验证：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode simulate --cycles 2
```

只采集到本地，三张 JPEG 和请求 JSON 保存到打印的日志目录，云端请求为零：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode capture \
  --contract configs/a10/a10_wallx_native_experiment.json
```

单轮真实观测推理并校验动作，不下发机器人目标：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode read_only --cycles 1 \
  --contract configs/a10/a10_wallx_native_experiment.json
```

`a10_wallx_native_experiment.json` 明确采用每臂 A10 原生 base/tool0，选择
`teletwin_desktop/gripper-g`，把 A10 闭合比例反向线性映射到模型 `[1.526,0]`。
**这是只读跨本体实验假设，不是官方 A10 适配。** G 的数值端点不能证明机械行程相同；
H 的上限在官方 API 页面和新版源码间还存在 4.6/5.5 的差异，实验不使用 H。
`verified=false` 时 `motion` 会在打开设备前拒绝运行，不能仅为通过检查而修改此标记。

实际标定文件须提供每个物理臂的 `model_frame_from_rig`、完整刚体
`model_ee_in_grasp_m_xyzw`（均为 `[x,y,z,qx,qy,qz,qw]`）、
`model_gripper_from_closure` 的张开/闭合端点，以及确认状态 `verified=true`。
标定完成后的执行命令：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode motion --cycles 1 \
  --contract /path/to/calibrated_a10_contract.json
```

日志 `work_dirs/a10/wallx_inference/<时间>/events.jsonl` 保存输入几何、预测、插值计划、
发送时间和到位结果；拒绝动作也保留云端原始预测。发送超时不补发、不追赶；
故障或退出关闭设备并停止已有 follower。当前约束检查步长、漂移、桌面高度和抓取点间距，
没有全连杆碰撞模型或 IK 可达性预检；桌边位置也未测量。

## 首次真实观测推理结果

2026-11-25 18:03，控制服务恢复后通过共享接口取得三路 RealSense、两臂和双夹爪观测，
使用 `a10_wallx_native_experiment.json` 完成一次云端推理。端到端 0.209 s，
服务推理约 64.8 ms，返回双臂各 32 行，响应 `valid_rows=31`。

服务提示右臂原生 tool0 的 X=-0.342 m 超出该部署训练范围 X=[-0.12,0.90] m，
并提示右臂首个 tool0 命令跳变 0.305 m。本地完整工具变换后的抓取点首步分别为左 6.35 mm、
右 302 mm，右臂超过 20 mm 的单步阈值，整段拒绝，follower 未启动、机器人目标发送数为零。
这直接说明当前原生基座映射不适合下发；没有放宽限幅来通过检查。

原始日志为 `work_dirs/a10/wallx_inference/20261125_180300_ee64d388/events.jsonl`，
精简审计为 `work_dirs/wallx/a10_first_inference_audit.json`。

新增 `configs/a10/a10_wallx_forward_experiment.json` 供下一次只读试验：保留各自基座原点，
两臂坐标轴均朝 rig 前方；右臂位置和姿态同时消除原生基座 180° 朝向。
同一保存观测在该约定下右臂输入为 `[+0.342,+0.024,0.625]` m，离线变换往返通过，
但**尚未重新上传或获得此映射的云端预测**。该文件仍为 `verified=false`。
第二次上传被自动审批拒绝，理由是先前授权被界定为单次，新的实时相机/状态外发需再次授权。
后续只读命令（待取得该次上传授权后）：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode read_only --cycles 1 \
  --contract configs/a10/a10_wallx_forward_experiment.json
```

## 验证

```bash
work_dirs/wallx/venv/bin/python -m unittest discover -s test -p test_wallx_client.py -v
work_dirs/wallx/venv/bin/python -m unittest discover -s test -p test_a10_wallx_geometry.py -v
work_dirs/wallx/venv/bin/python -m unittest discover -s test -p test_a10_wallx_execution.py -v
```

共 34 项测试：7 项协议测试覆盖 metadata 先收、鉴权头、JPEG 二进制编码、NumPy 动作解码、
拒绝未支持的 A10、异常握手、无效观测/动作以及超时后关闭连接；8 项几何测试覆盖左右绑定、
旋转工具偏置、tool1 不重复偏置、双向变换、RPY 顺序、异常反馈和夹爪方向。
19 项执行器测试覆盖双周期、只读、完整工具变换、物理左右绑定、首行未来动作、全段拒绝、
取消、超时不追赶、到位反馈和异常清理。另有原 joint 推理与 SDK 几何的 33 项回归测试通过。
已实测官方 API 的 HTTP 101、能力列表，以及此前实机 GET 的几何转换。

官方参考：

- [API 使用说明](https://x2robot.com/docs/quantax1pro/v1/model-development/wallx-api/api-usage-instruction)
- [Client 与 WebSocket 协议](https://x2robot.com/docs/quantax1pro/v1/model-development/wallx-api/client-side-websocket-communication-protocol)
- [官方机器人客户端启动方法](https://x2robot.com/en/docs/quantax1pro/v1/model-development/wallx-api/client-run)

官方客户端启动脚本针对其支持的机器人；不能将 X2 的容器启动命令直接套到 A10。
