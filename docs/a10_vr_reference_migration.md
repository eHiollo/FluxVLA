# VR 控制算法迁移：阶段 1～3 开发记录

## 现在完成到哪里？

2026-10-16，在 `kaanh_follower_vr` 完成接口梳理和离线算法迁移。
新增算法已经可以读旧 VR 日志，并通过显式选择的 `reference` 模式接入 UDP 发送路径。
**默认仍为原 `delta` 模式；新路径只完成本机模拟收发，尚未实机运动验收。**
按用户后续限定，本轮对 0.125 只读查看，控制器文件、进程和运行方式均未修改。

| 阶段 | 状态 | 本次交付 |
| --- | --- | --- |
| 1：功能与协议对照 | 已完成梳理，现场待确认项如下 | 明确已有功能、缺失功能和控制器职责 |
| 2：离线迁移 | 已完成 | 集中算法模块、原 C++ 数值对照、回放工具和测试 |
| 3：UDP 接入与对照 | 前端接入/模拟验证完成；实机验收待进行 | 新控制链、只读观察、停止确认；当前控制器 WS 未监听 |
| 4：完整采集 | 未执行 | 相机、夹爪、episode 和训练数据联动 |

## 一、分工与文件位置

```text
VR 原始输入 → mapper.py → control.py → runtime.py → A10Operator → UDP
                          ↓               ↑
                      参考目标诊断     实际反馈

--observe：同样计算参考目标，只读取反馈，禁止启停/发送运动。
```

| 文件 | 职责 |
| --- | --- |
| `vr/mapper.py` | 复用输入映射、死区、异常样本处理；输出锚点偏移 |
| `vr/control.py` | 新增，集中锚点计算、参考目标、限速、加速度整形、跟踪检查与离线回放 |
| `vr/runtime.py` | 运行循环；保留 delta 基线，新增 reference 接入和只读观察 |
| `scripts/replay_vr_control.py` | 离线入口，直接运行纯数学模块，不加载模型/GPU，也不构建硬件对象 |
| `test/test_vr_control.py` | C++ 数值对照及前端适配测试 |
| `test/data/vr_control/` | 原 C++ 参考结果、生成工具及无 SDK 的 C++ 对照入口 |

不依赖 `projects/a10`。既有配置 `configs/a10/a10_vr_teleop.py` 保持原控制模式；
阶段 3 的新模式需显式选择，未把原配置静默切换成新算法。

## 二、阶段 1：功能与接口对照

来源：此前对 `192.168.0.125:/home/kaanh/A10_new` 的只读调查，分支 `kaanh_rtc`，
提交 `97feb3131e48964c092371b266ebb489d8235686`。本轮使用本地源码快照及内容一致的
本地 `A10_new/a10_anchor_protocol.cpp`，没有连接机器人或修改控制器。
源码 SHA256：`ee282317b9b7b361d8b39a049e63f71f9b62a21c5be6e5e9796b10ffa82b3464`。
此信息说明审查的代码版本，不证明当前正在运行的控制器版本。

| 原功能 | 迁移前 FluxVLA 状态 | 本轮处理 |
| --- | --- | --- |
| 手柄坐标、比例、死区、握键、精调 | mapper 已有 | 复用 |
| 异常跳变、过期样本、重复输入 | mapper 已有 | 保留，新增控制层检查输入年龄 |
| 锚点偏移 | mapper 已输出 `_xlevr.anchor_command`，runtime 尚未消费 | 离线控制模块开始消费 |
| 机器人锚点与偏移组合 | runtime 原来累积逐帧增量 | 移植 `EeAnchorShadow` 的组合和编号规则 |
| 有界参考位姿、跟踪冻结/故障 | C++ `EeAnchorReferenceGovernor` | 移植并做数值对照 |
| 速度限制、逐轴速度变化限制 | C++ 数学原语 | Python 对照实现 |
| 前端参考加速度整形 | 后端原为 500 Hz 控制链 | 新增离线适配，明确不是完整后端等价移植 |
| 松手的锚点包序号 | 旧日志沿用最后一次活动序号 | mapper 修正为当前输入序号；离线模块兼容旧日志同序号释放 |
| 夹爪速度转目标开度 | runtime 已有可选 USB 驱动 | 本轮复用，不增加第二套积分与串口操作 |
| 500 Hz 反馈控制、逆解、电机保护 | 控制器 | 保留在控制器，不迁入 Python |
| 录制/重录/续录与相机同步 | 新 VR 流程尚未打通 | 留到阶段 4 |

### 数据约定

- mapper 锚点包：`active/session_id/anchor_id/sample_sequence/offset/gripper`。
- `offset`：相对 VR 接管时机器人末端坐标系的 xyz（米）+ 旋转向量（弧度），不是四元数。
- 内部位姿：基座系 xyz（米）+ 单位四元数 xyzw。
- 目标组合：`T_target = T_VR_robot_anchor * T_offset`。
- UDP 发送前：`T_relative = inverse(T_follower_start) * T_command`。
- VR 重新接管锚点与 follower 启动锚点独立；重新握键不应把 follower 锚点重置成当前位置。
- 夹爪包内输入为 `[-1,1]` 的速度方向，不是毫米开度；离线模块保留协议校验，不执行夹爪。

### 原生 UDP 接口的已知与未知

当前 A10Operator 通过 WS 5999 控制、UDP 9998 发送 `pq` 末端目标；关节 UDP 为 9999。
现有平台接口使用 40 字节头和 `{"type":"pq","pq":[[x,y,z,qx,qy,qz,qw]]}`。
这些是当前平台/已有验证的约定，不是本次重新完成的控制器收发验收。

原生关节 `follower_aj` 在审查 XML 中绑定 `FollowerAjAbsolute2`，头文件注明使用
`MoveAbsolute2` 规划。**不能据此推断末端 `follower_cart` 也使用同一规划器。**
自定义 TCP `vr_vel` 不经由原生 UDP 目标接收接口。

阶段 3 前必须确认：原生末端 follower 的缓冲、插值/平滑、乱序/重复包策略、断流行为、
停止确认方式；末端反馈的坐标/单位/实际采样时间及更新频率；启动锚点与工具坐标的准确语义。
现有 `received_monotonic_s` 是本机接收时间，不能证明控制器反馈本身没有缓存。
未知能力保持“未确认”，不能用 Python 超时检查替代控制器断流保护。

## 三、阶段 2：已实现的算法

### 与原 C++ 对照的部分

- `AnchorState`：接管/释放、会话和锚点切换、重复/旧序号判定、非单位坐标系中的位姿组合。
- `ReferenceGovernor`：参考位姿限速，外部保持时冻结，跟踪误差过大时冻结并进入故障。
- `clamp_vector/slew_vector`：向量限速和 C++ 逐轴 slew 数学原语。

默认参考速度 0.12 m/s、0.32 rad/s，跟踪阈值 0.05 m、0.35 rad。
原 C++ 的 50 个 2 ms 冻结周期改为累计 0.1 秒；前端改成 30 Hz 后不会错误地等 50 帧。
对照测试用 2 ms 输入，验证原算法状态和位姿一致；另外单独测试不等时间步下的 0.1 秒故障阈值。

### 面向前端新增的适配

`VRPoseController` 将上述算法组合，计算参考位姿、候选命令位姿和相对 follower 目标：

1. 输入/反馈年龄与控制间隔检查；重复活动包不延长输入有效期。
2. 锚点目标经过参考限速与跟踪保护。
3. 可选的参考加速度整形，默认开启；速度变化使用向量范数限制，比原 C++ 逐轴 slew 更保守。
4. 参考与候选命令之间按位置/旋转增益 6.0/4.8 生成速度，再限制速度、加速度并积分。
   这里的误差是两个前端参考的差，不是原 500 Hz 真实反馈速度闭环。
5. 候选命令对实际反馈仍有距离/角度检查；过大则请求停止，不继续输出相对目标。

默认加速度 0.4 m/s²、0.96 rad/s²，输入/反馈年龄上限 0.25 秒，单次控制间隔上限 0.1 秒。
**没有新增低通滤波，没有移植逆运动学、关节保护或实时电机控制。**
加速度整形可能增加延迟、短时越过参考，故保留关闭对照和命令/反馈误差限制。
它不是完整 `vr_vel` 的等价实现，仍需阶段 3 测量。

释放/故障时返回 `stop_requested=true`、`relative_target=null`；这表示请求下游停止，
不是“机器人已经停稳”，也不是“停止发送 UDP 就一定会停止”。阶段 2 时适配器尚未接入；
阶段 3 新增的停止确认和接管流程见下节。故障在算法模块中保持，需显式 `start()` 重置。
`release/force_fault` 可以把算法参考置为实际位姿，但它不能被冒充为连续制动轨迹下发。

## 四、现在可以怎么验证？

以下命令均在 VR 工作区执行，不打开设备、不发送 UDP：

```bash
cd /home/robot/VLA/FluxVLA_vr
conda activate fluxvla

# 无需 VR 和机器人，运行内置示例
python scripts/replay_vr_control.py --demo \
  --output work_dirs/a10/vr/reference_migration/demo.jsonl

# 基线对照：关闭新增前端加速度整形
python scripts/replay_vr_control.py --demo --no-acceleration-limit

# 回放此前 VR-only 日志；显式说明机器人反馈是合成的
python scripts/replay_vr_control.py \
  --input work_dirs/a10/vr/teleop_diagnostics.jsonl \
  --feedback-source synthetic-follow

# 自动化测试
python -m unittest discover -s test -p test_vr_control.py -v
python -m unittest discover -s test -p test_vr_operator.py -q
```

`--feedback-source` 三种选择：

| 选项 | 含义 |
| --- | --- |
| `recorded` | 输入文件默认值，要求日志中有真实位姿和可计算年龄的反馈时间 |
| `synthetic-follow` | 假设机器人下一帧完美跟随上一帧候选命令；不能证明实机延迟与稳定性 |
| `synthetic-hold` | 假设机器人固定不动，可观察目标领先后的保护行为 |

`--fps` 默认 30，仅用于首帧或缺少时间的非活动记录；有效操作缺少时间会报错。
`--max-steps` 限制离线帧数。未指定 `--output` 只打印摘要，不新增逐帧文件。
没有 `--execute` 参数。日志记录 `offline_only=true`、`transmitted=false` 和反馈来源。
回放假设每段第一帧反馈为 follower 启动锚点，显式标注该假设，不认定它是实机原始锚点。

### C++ 参考结果从哪里来？

`reference.json` 由原 `a10_anchor_protocol.cpp` 编译运行生成，不是另一份 Python 生成的期望值。
参考输入覆盖旋转坐标系、多轴偏移、重复和旧包、释放、重新接管、冻结/故障、微小角和近 180° 旋转。
`reference.cpp` 只链接锚点算法和 JSON 头文件，不链接机器人 SDK。

如有同一版本源码，可以在本机重新生成：

```bash
python test/data/vr_control/generate_reference.py --source-root /path/to/A10_new
```

生成工具核对原 C++ SHA256，拒绝未经审查的其他版本；源码目录保持只读，编译产物放在临时目录。
对照夹具保存源文件和头文件哈希。普通测试直接读取夹具，不要求安装 C++ 编译器或保留其他仓库。

## 五、阶段 1、2 验证结果与开发记录（前一轮）

- 原 C++ 参考结果在本机编译生成成功；锚点、参考生成器与限幅原语对照通过。
- 新增控制模块测试 11 项，既有 VR 测试增加释放序号检查后共 23 项。
- 内置示例 120 帧：90 帧 tracking、30 帧 inactive；无故障，没有发送任何指令。
- 旧 VR-only 日志合成跟随回放 4,375 帧：492 帧 tracking、3,883 帧 inactive，无故障。
  其中 2,370 帧使用名义时间（主要是设备尚未提供有效样本的非活动记录）；不是实机验证。
- 回放发现旧 mapper 松手包复用活动包序号：修复新包序号，同时保留同序号释放兼容，
  避免把松手误当活动重复包。更旧的锚点/序号仍拒绝，重复活动包不会刷新超时。
- 前一轮没有连接相机、VR 或机器人；当时既有 VR runtime、A10 UDP operator 和主分支未修改。
- 前一轮后续计划为阶段 3；本轮实际接入和只读边界见下节，仍未执行实机运动测试。

## 六、阶段 3：前端接入、只读调查与模拟收发（2026-10-16）

### 本轮范围

用户明确要求控制器只读查看、代码全部写在 FluxVLA。已完成前端接入、只读工具和本机测试；
未修改控制器、未发送实机目标、未尝试启动或停止现场 follower。
**本阶段尚未完成真实 UDP 链路与运动表现验收。**

### 三种使用方式

| 方式 | 选择 | 行为 |
| --- | --- | --- |
| 原增量基线 | `teleop.control_mode=delta`（默认） | 保留原来的逐帧增量与松手保持逻辑 |
| 新参考目标模式 | `teleop.control_mode=reference` | 消费锚点包，经 control.py 计算目标，再由 A10Operator 发 UDP |
| 只读观察 | `--observe` | 强制 reference、`execute=false`、`allow_motion=false`，经监视端口读反馈，计算候选目标；不启动/停止 follower、不发 UDP 或夹爪目标 |

两种控制模式都不自动使能、不自动回零。`vr_only=true` 时仍只采集输入，没有假机器人反馈。
只读观察拒绝打开相机或夹爪设备；如配置了这些附件，需要先移除对应设置。
模式选择保持在现有配置/入口中，没有新增后端服务，也不依赖旧 projects/a10。

reference 模式要求 mapper 输出 safe_frame_delta 锚点包并绕过原输入整形，防止两套前端整形叠加。
原 `robot_controller` 名称只表示 mapper 不整形，诊断已改标 `downstream_controller_unverified`，
不能把该名称误读为“已验证原生 UDP 使用了 vr_vel 算法”。新前端仍不增加低通滤波。

### 启停和重新接管

1. 第一次有效握键前检查包结构、数值和输入年龄，再选 Tool、请求 follower 并确认模式。
2. 记录启动位姿，先发启动零偏移保持；丢弃阻塞启动前的手柄样本，重新建立 VR 参考。
3. 新样本产生参考目标；发送前再次检查实际跟踪误差、逐帧平移和旋转变化。
4. 松手请求一次 `stop_follower`，先确认退出模式，再用新公开接口 `wait_stopped()` 查询
   “follower inactive + robot motion=Stopped”。只查询状态，不自动下电或重启。
5. 再次握键时重新启动 follower，更新它的启动锚点。与 delta 模式松手后仍保持原 follower 不同。
6. 活动中突然出现不同锚点/会话（例如异常输入重建参考）会停止，要求先松手再接管，
   不直接清零速度后继续运动。
7. 输入/反馈异常、超时或启动中断均进入停止清理；停止未确认会保留错误，不在清理中自动重发或重新连接控制。

`Stopped` 是控制器报告的状态，不是独立物理测量或急停承诺；控制器自身断流规则仍未确认。
每次松手都退出跟随，是当前新路径明确停止责任的实现，其延迟和操作感仍需现场对照。
只读观察不运行以上启停操作；它把首次观察位姿明确标记为“假设锚点”，不声称取得真实 follower 启动锚点。

### 新命令

仅检查控制器状态，不启动 VR、相机、夹爪，不要求预先猜测位姿单位：

```bash
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py \
  --inspect-controller --samples 5 \
  --report work_dirs/a10/vr/reference_migration/controller_readonly.json \
  --cfg-options teleop.operator.host=192.168.0.125
```

工具固定使用监视端口（默认 5888），只允许 `get`。报告包括实际完成的查询数、原始末端值、
客户端查询耗时和未知能力；失败时也保存报告并返回非零退出码，不会回退到运动接口。
`--inspect-controller --preview` 仅预览，不建立连接。

后续服务恢复且现场确认单位后，可准备带 VR 的只读观察：

```bash
# 将 YOUR_PC_IP 替换为电脑局域网地址，VERIFIED_UNIT 替换为已核对的 mm 或 m。
# 先保留 --preview 检查；准备接头显、只读观察时再去掉 --preview。
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py \
  --observe --preview \
  --cfg-options teleop.operator.host=192.168.0.125 \
    teleop.operator.feedback_position_unit=VERIFIED_UNIT \
    teleop.input.network.host_ip=YOUR_PC_IP
```

只读观察日志中的 `reference_control.relative_target` 是候选值，
`reference_control.transmitted=false`、`sent_relative_target=null`。
实际 reference 模式成功写 UDP 后才标 transmitted；写出依然不代表执行成功。
日志另加 `control_mode/observe_only/stop_status`，区分算法状态、模式退出和控制器报告停止。
本轮不提供已经验收的实机运动命令。

### 0.125 现场只读结果

| 检查 | 结果 |
| --- | --- |
| SSH | 可登录，查询后退出 |
| 仓库 | kaanh_rtc，97feb31，与此前审查版本一致 |
| 控制进程 | kaanhbin，PID 2120884；未确认运行二进制与仓库一致 |
| 相关监听 | UDP 9998/9999、TCP 8080 在监听 |
| WS 5888/5999 | 查询时均未监听；5888 连接返回 ConnectionRefusedError |
| 原生状态查询 | 未完成，连接建立失败，未发出 get 业务请求；未尝试启停或运动 |

现场检查报告保存在 `work_dirs/a10/vr/reference_migration/controller_readonly.json`，
包括 SSH 只读证据。没有尝试恢复、重启或更改配置，需由控制器负责人确认 WS 服务。
UDP 端口在监听不证明 follower 正在运行，也不允许绕过状态确认直接发目标。
因此插值、缓存、乱序/重复包、断流及实际反馈采样年龄继续保持“未确认”。

### 测试与下一步

新增本机测试覆盖：真实 mapper → 新参考目标 → loopback WS/UDP 的完整报文链；
启动丢弃旧样本、松手双重停止确认、半启动失败、输入断开、慢反馈导致控制超时、
活动中换锚点、停止超时不重试、只读工具无运动，以及只读观察预览不连接设备。
这些测试使用假控制器/本机回环，不连接真实机器人。

验证：VR 测试 59 项、既有 A10 operator 测试 39 项通过；新增阶段 3 测试 16 项已包含在前者。

```bash
python -m unittest discover -s test -p 'test_vr_*.py' -q
python -m unittest discover -s test -p test_a10_operator.py -q
```

下一步仍属于阶段 3：控制器侧 WS 可用后先读反馈，确认单位和采样语义；在不改后端的前提下
完成只读观察。真实移动、松手停稳和原生 UDP 动态对照需另行安排现场验证。
