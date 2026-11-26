# Wall-X API 接入记录（A10，2026-11-25）

实验分支：`experiment/wallx-api`，从 `inference_1118` 新建；现场原有未提交改动保留。

已完成鉴权、二进制 MessagePack 客户端，以及复用遥操作 SDK Tool 通道的双臂 EE 执行闭环。
离线双周期闭环验证通过；修正右臂模型坐标朝向后，连续三轮真实观测推理和轨迹检查通过。
后续五轮小步实机日志确认发送 150 个目标并到位，反馈显示动作执行有效。
用户已声明完成标定并明确确认前向 JSON 参数，按其指令设置 `verified=true`，记录来源且不改数值。
完整动作块/模型夹爪的任务执行仍待现场启动验收；当前执行器为同步停等模式，不声明 RTC。
用户明确持续授权本次 Wall-X 对接中的观测上传和只读推理重试，无需逐次询问上传授权。

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
2026-11-25 已取得 A10 实时观测的云端预测；原生坐标试验的拒绝原因和前向坐标试验结果见下文。

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

已按用户要求再次核对遥操作：`a10_dual_vr.py` 第 73–76 行的左/右手矩阵分别为
`(dx,dy,dz)→(-dz,-dx,dy)`、`(dx,dy,dz)→(dz,dx,dy)`。
将右基座绕 Z 的 180° 纳入变换后，两侧都对应统一的前 +X、左 +Y、上 +Z，
与前向 Wall-X 配置的轴方向一致。物理左右/ee_id/内部别名也逐项核对。
但 `WorldTranslationAnchorState` 使用 `anchor_position + offset`，只控制相对位移；
VR 代码不测量绝对基座原点，文档也明确“初始位置原点在差分中抵消”。
每臂原点取 controller base/wobj0 是本项目的坐标定义，基座相对位置来自用户确认的安装尺寸，
不能写成“遥操作已标定绝对原点”，也不能据此验证模型训练坐标或夹爪量。
核对报告为 `work_dirs/wallx/a10_teleop_frame_audit.json`，配置仅补充审计来源，没有改变数值或全局 verified 标记。

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
以上几何采样阶段没有向模型发送观测；随后已完成下述多轮推理。`model_contract.verified` 仍为 false。

## EE 闭环入口

取消试步裁切并启用完整预测/夹爪的[完整部署入口](a10_wallx_deployment_proposal.md)
使用 `configs/a10/a10_wallx_deploy.py`，每轮原始未来行 0–5，默认 100 轮，
自适应插值及完整块回放通过。用户已明确确认前向 JSON 参数并要求 `verified=true`，
确认来源已写入文件。`motion` 使用该文件通过原有检查，不加入绕过选项。
`trial` 保留为独立诊断入口。

入口 `scripts/a10_wallx_inference.py`，参数 `configs/a10/a10_wallx_inference.py`。
完整流程为新鲜三路相机/机器人/夹爪观测 → 云端同步推理 → 重读状态并检查推理期间漂移 →
整段动作与插值检查 → 遥操作同款 Tool/Cart SDK 轨迹 → 反馈确认到位 → 下一轮观测。
默认每轮选择前 6 个未来动作，每个动作插 6 步，控制频率上限 90 Hz，沿用遥操作频率。
执行允许比模型标称的 33.7 ms/行动更慢，6 行约需 0.4 s，避免增加插值后提高发送频率。
模型时间间隔校验与实际发送节拍分别处理。只有 `motion` 模式会启动 follower。
复用 joint 推理的相机、USB 夹爪线程、超时和停止清理，EE 数据不走 joint/timed_aj 通道。

不连接设备的双周期验证：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode simulate --cycles 2
```

只采集到本地，三张 JPEG 和请求 JSON 保存到打印的日志目录，云端请求为零：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode capture \
  --contract configs/a10/a10_wallx_forward_experiment.json
```

单轮真实观测推理并校验动作，不下发机器人目标：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode read_only --cycles 1 \
  --contract configs/a10/a10_wallx_forward_experiment.json
```

`a10_wallx_forward_experiment.json` 使用各臂独立基座原点、统一朝前的模型轴与 A10 tool0。
保留的 `a10_wallx_native_experiment.json` 使用原生基座轴，已实测右臂坐标不适用，不建议继续使用。
两者选择 `teletwin_desktop/gripper-g`，把 A10 闭合比例反向线性映射到模型 `[1.526,0]`。
**这是只读跨本体实验假设，不是官方 A10 适配。** G 的数值端点不能证明机械行程相同；
H 的上限在官方 API 页面和新版源码间还存在 4.6/5.5 的差异，实验不使用 H。
`verified=false` 时 `motion` 会在打开设备前拒绝运行，不能仅为通过检查而修改此标记。

用户已明确要求现场启动实机试验，新增 `trial` 作为独立的受限实验入口。
该入口保留 `verified=false`，不代表完成标定，也不改变常规 `motion` 的要求。
不加 `--trial-step` 时只允许一轮，选择首个未来目标，每臂抓取点位移上限 15 mm、旋转上限 0.05 rad，
30 个插值步、发送上限 30 Hz，夹爪保持新鲜反馈的当前开度。
桌面上方至少保留 50 mm，继续检查云端告警、工具点步长、状态新鲜度、跟踪误差和到位。
超过限幅直接拒绝，不缩放或裁剪模型动作来通过检查。只试一轮，不自动重试。

机器人已使能，且相机、夹爪未被遥操作或其他推理占用时，由现场用户启动：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode trial --cycles 1 \
  --contract configs/a10/a10_wallx_forward_experiment.json \
  --prompt "fold the black cloth using both arms"
```

这是会实际移动双臂的命令。Ctrl+C 会请求停止并清理，成功到位后也会自动停止 follower。
进程退出、网络断开或强杀不能替代机器人急停。当前代理未代替现场用户启动该命令。

如果首个预测超过完整目标试验的 15 mm / 0.05 rad 限制，可显式选择**小步接近诊断**：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode trial --trial-step --cycles 1 \
  --contract configs/a10/a10_wallx_forward_experiment.json \
  --prompt "fold the black cloth using both arms"
```

`--trial-step` 仅允许与 `trial` 同用。先拒绝超过 50 mm / 0.15 rad 的原始预测，
再沿首个目标线段取一个双臂共用的比例，使每臂抓取点最多移动 5 mm、旋转最多 1°。
这是显式的诊断小步，不是完整模型目标执行；原始预测与比例记录在 `trial_step` 日志事件中。
之后仍做原有桌面、间距、工具点插值和漂移检查，夹爪保持。
试验到位阈值收紧为位置 1 mm、旋转 0.01 rad 并稳定 0.1 s，避免 5 mm 试步在原地就算到位。

用户要求多步后，`trial --trial-step` 已支持 `--cycles 1` 到 `--cycles 5`。
每轮重新观测和推理，只执行一个受限小步，确认到位并停止 follower 后才开始下一轮。
停止未确认、动作超限、反馈异常或取消都结束本次试验，不继续推理/发送。
每臂整个试验的累计计划路程不超过 25 mm、累计计划旋转不超过 5°，
所有插值目标相对初始位姿也必须在 25 mm / 5° 内。逐轮检查工具/工件绑定不变。
这是同步停等小步试验，不使用 RTC，不重放同一个旧预测。五轮实机命令：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode trial --trial-step --cycles 5 \
  --contract configs/a10/a10_wallx_forward_experiment.json \
  --prompt "fold the black cloth using both arms"
```

代理仅用模拟设备验证五轮回路，没有代替现场用户运行此实机命令。

18:40 用户试验的原始目标左 28.71 mm / 6.12°、右 26.26 mm / 2.84°，
发送前被拒绝，目标发送数零。对该保存观测离线验证 `--trial-step` 后，
左步长 4.69 mm / 1°、右步长 4.29 mm / 0.46°，30 个插值帧通过检查。
离线报告在 `work_dirs/wallx/trial_step_offline_audit.json`，未替现场用户重新启动机器人。

若启动时报 `Arbitrary controller commands require the console role`：
18:56 的实测在切换 Tool 坐标时被旧共享服务拦截，尚未启动 follower 或发送目标。
源码已为 inference 角色增加仅限 `set_jog_coordinate --tool` 的支持，要求停止、
无故障、follower 未启动、没有其他控制者或控制台事务；不授予任意控制器命令权限。
已经运行的共享服务不会热加载修改。当前可在已有 `a10>` 控制台执行：

```text
set_jog_coordinate --tool
```

确认成功后重跑上面的 `trial --trial-step` 命令；启动检测到已是 Tool 时不会重复切换。
也可以在所有客户端退出、无在运动作后重新启动共享服务加载更新。不要把推理角色改成 console。

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
用户随后明确持续授权本次上传和推理，已继续完成下面的试验。该文件仍为 `verified=false`。
连续只读命令：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py --mode read_only --cycles 3 \
  --contract configs/a10/a10_wallx_forward_experiment.json
```

18:07 单轮试验通过，左右抓取点首步约 11.0/10.4 mm，端到端 0.221 s，无输入告警。
后续连续试验出现一次工具点插值超过 5 mm，仍在发送前拒绝；
把每行动插值从 3 步增加到 6 步，并新增 90 Hz 发送上限，没有放宽几何阈值。

18:10 再次连续三轮全部通过，日志为
`work_dirs/a10/wallx_inference/20261125_181012_66e89bc6/events.jsonl`，
精简报告为 `work_dirs/wallx/a10_forward_validation.json`：

| 轮次 | 端到端耗时 | 左/右最大抓取点步长 | 插值帧数 |
| --- | --- | --- | --- |
| 1 | 0.246 s | 12.43 / 5.34 mm | 36 |
| 2 | 0.180 s | 7.49 / 6.65 mm | 36 |
| 3 | 0.177 s | 10.70 / 5.18 mm | 36 |

三轮均无坐标超范围或首步锚点漂移告警，动作/插值检查通过，实机目标发送数为零。
第三轮收到 `rtc_disabled_in_closed_loop` 建议：服务按连续请求判定闭环，提示开启 RTC。
当前只读试验未执行预测，无法评估真实闭环的块边界和伺服滞后；
同步停等执行器不声明实现官方 RTC。实际连续执行前仍需验证模型映射、夹爪对应关系和控制时序。
已对云端 `state_coordinate_origin_ood`、`action_step0_anchor_drift` 增加明确拒绝，
防止仅因为本地数值步长较小就放行这类已发现的坐标问题。

## 验证

```bash
work_dirs/wallx/venv/bin/python -m unittest discover -s test -p test_wallx_client.py -v
work_dirs/wallx/venv/bin/python -m unittest discover -s test -p test_a10_wallx_geometry.py -v
work_dirs/wallx/venv/bin/python -m unittest discover -s test -p test_a10_wallx_execution.py -v
```

共 53 项 Wall-X 测试：7 项协议测试覆盖 metadata 先收、鉴权头、JPEG 二进制编码、NumPy 动作解码、
拒绝未支持的 A10、异常握手、无效观测/动作以及超时后关闭连接；8 项几何测试覆盖左右绑定、
旋转工具偏置、tool1 不重复偏置、双向变换、RPY 顺序、异常反馈和夹爪方向。
38 项执行器测试覆盖双周期、只读、完整工具变换、物理左右绑定、首行未来动作、全段拒绝、
取消、超时不追赶、到位反馈、异常清理、云端坐标告警和加密插值时的 90 Hz 频率上限。
另覆盖独立受限试验的未标定状态保留、夹爪保持、大动作拒绝及多轮请求拒绝。
小步诊断覆盖双臂共同比例、5 mm/1° 上限、巨大原始预测拒绝、实际闭环清理和原地反馈不得算到位。
新增旧共享服务权限报错提示测试；共享服务测试覆盖 inference 仅选 Tool、任意命令拒绝、
状态连接拒绝、其他控制者/控制台事务不被抢占，以及必须确认停止。
多步试验新增五轮重新观测/停止回路、累计路程预算、停止失败阻止下一轮、最多五轮的测试。
新增自适应插值保留完整位姿和夹爪终点、速度/步长上限、过多插值拒绝和实际到位反馈日志测试。
完整部署新增超过五轮/25 mm 的原目标与夹爪执行、保留标定检查及停止失败不继续测试。
本次 53 项 Wall-X 测试和此前 34 项共享控制测试通过。扩展 VR 回归未全通过：
14 项因 mmengine 配置继承的 lazy_import 类型不一致报错，另有 1 项记录连接关闭断言失败。
用修改前的共享服务 handle 函数回放代表测试，同样复现配置错误和连接关闭失败；
本次未改动这些 VR 配置或记录逻辑。检查记录在 `/tmp/wallx_shared_checks.log`、
`/tmp/wallx_vr_baseline_checks.log`。
另有原 joint 推理与 SDK 几何的 33 项回归测试通过。
已实测官方 API 的 HTTP 101、能力列表，以及此前实机 GET 的几何转换。

官方参考：

- [API 使用说明](https://x2robot.com/docs/quantax1pro/v1/model-development/wallx-api/api-usage-instruction)
- [Client 与 WebSocket 协议](https://x2robot.com/docs/quantax1pro/v1/model-development/wallx-api/client-side-websocket-communication-protocol)
- [官方机器人客户端启动方法](https://x2robot.com/en/docs/quantax1pro/v1/model-development/wallx-api/client-run)

官方客户端启动脚本针对其支持的机器人；不能将 X2 的容器启动命令直接套到 A10。

## 可选 timed_ee 和旋转坐标测试（2026-11-25）

新增 `configs/a10/a10_wallx_timed_ee.py` 可选后端：原始模型目标每点一次 30 Hz，普通块保持同一流，全流最后一点才 final=true。原 follower 配置保留。接入约定、控制器加载要求与命令见 [timed_ee 部署](a10_timed_ee.md)。

新增本地 [旋转测试面板](a10_rotation_test.md)，支持 follower 和 timed_ee 两个后端，逐臂逐轴 ±5°、返回起点、停止和观察报告。相关 167 项离线测试通过，两种面板模式均完成 Chrome 模拟点击验证；没有启动实机或重启控制器。GET TCP 与命令 FK 的来源限制在新日志和文档中明确标注。
