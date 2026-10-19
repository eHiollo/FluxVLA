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


## 2026-10-19 现场旋转异常排查与前端修复

### 现场证据

- 工作区/分支：`FluxVLA_vr / kaanh_follower_vr`。控制器 `192.168.0.125` 全程只读，
  未修改文件、重启进程、清错、使能或发送运动目标。
- 11:47:40 启动，VR 浏览器 `192.168.0.186` 已连接。最近会话共 5343 行，
  真正向 UDP 下发 17 个目标（含初始保持），有效目标发送跨度约 0.51 秒。
- 下发最后目标的旋转向量约 `[0.447, 0.626, -0.585]` 度，总角度约 0.97 度；
  最后一帧运动反馈的第 6 轴相对启动约 +1.28 度。前端日志未覆盖停止等待期间的运动。
  右手摇杆 x/y 在这些帧都为零，握持键有效且手柄姿态在变化；当前证据不是夹爪摇杆映射到腕部。
- 11:50:38 控制器 SYSLOG ID 1359：收到“停止跟随”。前端记录模式 inactive 已确认，
  但 `wait_stopped()` 未确认 `motion=Stopped`，约 11:50:40 退出并关闭 VR。
- 11:50:45 SYSLOG ID 1360，错误 -503：电机 6 超出正限位，允许最大位置
  `3.054326 rad`（约 175 度），目标位置 `3.054850 rad`。这是**规划目标**越限，
  不能写成已经测得实际机械角度到达该值。
- 此时段 kernel journal 没有上次 11:37 的 EtherCAT 掉线记录。后续监视端口查询被拒绝，
  再次只读检查时 kaanhbin 已不在运行，未自行重启。

### 当前实际接口与此前审查版本的差异

`/home/kaanh/A10_new` 仍为 kaanh_rtc / 97feb31，但运行目录
`/home/kaanh/A10_new/build/bin/kaanh.xml` 已包含 `A10UdpFollower` 与 `A10JointFollower`。
相应新增源码位于 `.worktrees/kaanh-integration`，分支 kaanh_test，HEAD 1f1a056。
不能再仅凭主工作区 Git HEAD 判断运行实现；本轮没有证明磁盘源码与已结束的进程逐字一致。
包装层只替换关节 UDP 回调，Cartesian 回调仍由安装的 KAANH 2.5.7 库提供。

SDK 库 SHA256：`fbd638320e3025f5e3c4ec8ec8d9761c5f8499a7c69a96123b05dce039eb52b5`。
本地静态分析及纯数学验证发现：

1. pq 路径调用 `s_pq2pe(..., "321")`，转换得到的 yaw/roll 负角会加 2π。
   对库的纯数学函数输入绕 z 轴 -0.01 度，输出为 +359.99 度；-1 度变为 +359 度。
   没有创建机器人、模型或控制器对象。
2. 接收器在欧拉分量上调用 LowPass，再把结果放入队列。执行路径按欧拉分量调用
   MoveAbsolute2，之后转回位姿。它并非直接在四元数上按最短路径插值。
3. 接收 `type="321", pe=[...]` 的分支直接使用输入六维向量，不经过上述 pq 归一化。
   因此换发四元数的符号 q/-q、继续减小负角度或只降低前端速度，都不能消除原 pq 路径的分支跳变。

上述证据支持“微小负角度被解释为接近一整圈，再进入滤波/规划”是本次异常的主要原因。
缺少控制器逐周期轨迹，不能声称已完整复现第 6 轴后续运动和停止路径全部细节。

### 本轮修改

- `a10_operator.py`：增加可选 `cartesian_wire_format='pe321'`；公共输入仍为米/xyzw，
  在线编码为带符号的 Euler ZYX（321）弧度。yaw/roll 连续展开，跨 ±π 时不跳整圈；
  遇到欧拉奇异或单包欧拉分支变化大于 π/2 时拒绝发送。角度状态只在发送成功后推进，
  并在开始新 follower/关闭连接时重置。
- VR 配置显式使用 pe321，通用 operator 的 pq 默认保持兼容。推理关节通道不受影响。
- 停止条件仍要求 inactive 和 Stopped 同时满足。超时提示补充最后模式、运动与使能状态，
  VR 终端明确提示“停止未确认”，不自动重新跟随、掉使能或放宽保护。
- JSONL 增加 `cartesian_wire_format`，`sent_relative_target` 仍记录内部米/xyzw 目标，
  不冒充实际 UDP 字节或控制器已到位。
- `test/data/vr_control/kaanh_cartesian_wire.json` 保存 SDK 纯数学样本。
  带符号 pe321 经 SDK 转换后与原四元数矩阵最大差约 `1.11e-16`。

### 验证与剩余工作

回归覆盖负角度 UDP 完整报文、SDK 坐标样本、跨 ±π 连续性、新会话重置、奇异与大分支
跳变零发送，以及 inactive 但 motion=Moving 必须停止确认失败。既有 pq 编码测试保留。

验证命令（仅回环网络和合成输入）：

```bash
PYTHONPATH=/home/robot/VLA/FluxVLA_vr/test \
  /home/robot/miniforge3/envs/fluxvla/bin/python -m unittest \
  test_a10_operator test_vr_reference_runtime test_vr_operator test_vr_control
```

结果：**93 项通过**，耗时约 12 秒；当前环境未装 pytest，使用已有 unittest，未新增依赖。

**修复尚未再次实机验收。** 控制器侧仍需核对 stop_follower 在 Cartesian 模式下的
取消/减速契约及断流行为。此次超时不能简单归因于“2 秒太短”，也不能用自动重试替代停止确认。
恢复现场时先确认第 6 轴位置及停止状态，排除报警后再进行小范围验收；不自动返回初始姿态。


## 2026-10-19 恢复原速度进行现场对照

用户要求先用原速度实机验证，再决定启停优化方案。本轮不修改算法或控制器。

- 旧控制器 2026-10-16 14:39:37 轨迹日志确认实际配置 vmax/ref_vmax=0.12m/s、
  wmax/ref_wmax=0.32rad/s、amax=0.4m/s²、jacc=0.96rad/s²。
- ReferenceConfig 已是上述默认速度及加速度；此前低速来自 CLI 覆盖 0.01 / 0.05。
  使用说明提供原速度命令，并将本轮日志独立写入 original_speed_trial.jsonl。
- VR 配置单帧检查改为 5mm / 0.015rad；30Hz 原速度正常步长为 4mm / 0.01067rad。
  保留超限退出及反馈/停止保护，调度延迟较大时仍可能触发检查。
- 不调整 mapper 缩放、reference/P/加速度算法、松手方式或原生 follower。
  提速不等于解决启停震动，实机效果待用户现场对照。
- 验证仅使用配置预览，不启动服务、连接控制器或执行机械臂运动。

## 2026-10-19 新增标定后的绝对朝向映射

### 本轮范围与实现

按用户批准方案，仅在 `kaanh_follower_vr` 开发；本轮未访问控制器或执行实机运动。
新增的数学计算、输入字段和生命周期接入均放在已有 `vr` 模块内：

- `control.py`：增加 `CalibratedAnchorState`，复用原 `AnchorState` 的接管、序号、重复帧和
  释放协议。平移采用固定 `R_base_vr`，每次接管只重记手柄位置、末端位置和位置倍率；
  姿态按 `R_base_vr * R_vr_hand * R_hand_tool` 计算，不叠加旧 axis_remap/angle_scale。
  `VRPoseController.start()` 保留标定及模式，继续复用现有 reference、限速、加速度和误差保护。
- `mapper.py`：内部锚点命令增加校验后的 `hand_pose_vr` 与接管时 `position_scale`。
  重复序号使用已接受位姿缓存；尖峰、过期和无效输入不提供有效手柄目标，防止绕过原保护。
  旧 offset 六维字段及旧模式仍保留。新模式不套用旧增量死区，没有新增输入滤波层。
- `runtime.py`：接入 `pose_mapping` / `pose_calibration`，新方式只允许 `control_mode=reference`。
  开始 follower 前先按所选映射校验命令；第一包实际位姿保持、启动后重新取新鲜样本、停止确认
  沿用原流程。JSONL 增加模式和标定，目标结果也记录映射名。
- `a10_vr_teleop.py`：`anchor_relative` 仍是默认；两个标定值默认 `None`。
  新方式缺少有效标定即报错；旧方式忽略标定，可只改模式参数切换，不必删除已有标定值。
- 回放 CLI 增加 `--pose-mapping` / `--pose-calibration`，不连接设备；拒绝把缺少已校验手柄
  字段的旧锚点日志直接解释为新模式，拒绝以不同标定冒充原日志的同一次回放。

### 离线验证

完整回归使用已有环境的 unittest（无新增依赖）：

```bash
PYTHONPATH=/home/robot/VLA/FluxVLA_vr/test \
  /home/robot/miniforge3/envs/fluxvla/bin/python -m unittest \
  test_a10_operator test_vr_reference_runtime test_vr_operator test_vr_control
```

覆盖固定平移方向与手腕/末端初始朝向解耦、非对易旋转的标定乘法顺序、重新接管位置零跳变与
绝对朝向一致性、q/-q 与 ±π 最短旋转、标定/输入缺失拒绝、速度/加速度边界、平移与朝向同时
收敛、原模式回归，以及真实 mapper→runtime→A10Operator→本机模拟 UDP 的带符号 321 报文。
模拟报文检查首包保持、负角保留、固定方向平移、释放停止确认及诊断字段。

结果：**104 项全部通过，约 12.9 秒**。直接调用配置预览入口也确认：原模式通过；新模式缺少
标定明确拒绝；通过 `--cfg-options` 提供两个离线示例四元数后预览通过，全程未启动服务。

另读取 `original_speed_trial.jsonl` 中的原始 VR 输入，重新通过当前 mapper 校验并对比两种
数学目标：处理 3085 帧，其中 164 帧为接受的有效活动输入，验证 16 次重锚点位置连续。
对照中使用**显式的单位示例标定**，不冒充现场标定；未运行机器人、未验证原生 follower
的跟踪或停止效果。输出仅保存在忽略目录：

- `work_dirs/a10/vr/calibrated_absolute_offline_comparison.jsonl`
- `work_dirs/a10/vr/calibrated_absolute_offline_summary.json`

上述旧日志为文件内全部记录的离线处理，不能把多次接管或追加记录算成同一次实机试验。

### 使用与待完成项

两种方式的说明、标定方向、公式、预览与离线命令见
[VR 使用说明](a10_vr_operator.md)。

新绝对朝向可能在手柄静止的接管时仍要求转动；只重记平移锚点，不重新标定朝向。
尚需现场确定固定操作坐标系及工具偏置、确认头显空间重置后的标定有效性，再进行实机验收。
本轮未修改控制器 SDK、UDP 协议、原生 follower 或已知的停止过程；不能据离线通过宣称启停
震动和控制器停止行为已解决。

## 2026-10-19 复用旧局部轴约定，简化标定方案

用户要求先把可复用的关系写入默认值，交互标定程序先给方案。本轮只修改默认配置、提示和
说明，未开发标定入口，未连接控制器，未改动其他分支。

- 上轮只读核查确认进程工作目录为 `/home/kaanh/A10_new/build/bin`，默认加载 `kaanh.xml`。
  运行目录和源码工作区的 XML 有工具/工件坐标及 VR 控制参数，但没有 VR 追踪空间的标定。
  `tool1/tool2` 是法兰侧工具安装参数，不能当作手柄与工具的关系。
- 原 LeRobot `quaternion_utils.py` 已定义 `AXIS_REMAP_VR_TO_ROBOT`：局部 X/Y 交换，Z 反向。
  新模式右乘项使用其逆旋转，得到 xyzw 四元数
  `[0.7071067811865475, 0.7071067811865475, 0, 0]`；已写入
  `a10_vr_teleop.py` 的 `hand_to_tool_quat_xyzw` 默认值。
- 该值复用原手柄/末端局部轴约定，不冒充现场物理测量。`vr_to_robot_quat_xyzw` 继续为
  `None`，缺少它时新模式仍明确报错；原默认模式不变。
- 修正之前“两个值都要现场重新测量”的说明：沿用已知局部轴约定时，只需一次有意识的
  手柄/末端参考姿态对齐，计算
  `R_base_vr = R_base_tool * inverse(R_hand_tool) * inverse(R_vr_hand)` 并保存。
  固定平移方向也由这次关系确定；如果要独立指定别的平移方向与握持习惯，则需另行确定关系。
- 交互方案记入 `a10_vr_operator.md`：复用现有入口和 VR 文件，新增只读标定选项，倒计时后
  采集稳定姿态，检查新鲜度、序号、停止状态及重建一致性，保存本地标定文件并在后续新模式
  加载。该选项及自动加载尚未实现，待用户审核。

验证：默认四元数的旋转矩阵与旧轴映射的逆一致；原配置可构造；只缺 VR/基座关系时新模式
拒绝；只补第一个离线示例四元数后可构造且未连接设备。另运行已有的新模式配置校验用例。

## 2026-10-19 实现只读姿态标定程序和自动加载

用户批准实现上节方案。本轮在 `kaanh_follower_vr` 开发，未连接现场 VR 或机械臂，
未访问/修改控制器，未触及 `kaanh_follower` 工作区。

### 实现

- `scripts/teleoperate.py` 新增 `--calibrate-pose`，与观察/控制器检查互斥；支持
  `--pose-calibration` 指定输出或加载路径、`--calibration-countdown`（默认 5 秒）、
  `--calibration-duration`（默认 1 秒）。`--preview` 只校验参数，取消与失败给出中文提示。
- `vr/runtime.py` 复用白名单只读 A10 实例构造（也供原 inspect_controller 使用），强制
  `allow_motion=False`、监视端口默认 5888；不继承摄像头、USB 夹爪或状态后台轮询选项。
  读取实际末端姿态前确认无故障、跟随 inactive 且 motion=Stopped，单位必须明确。
- 倒计时后收集至少 10 对、覆盖至少 1 秒的不同序号姿态。重复帧不计数；序号回退、时间不递增、
  过期、释放握键或机械臂未停止会拒绝。读取 GET 后再次检查手柄年龄，避免阻塞期间变旧。
- `vr/control.py` 新增纯数学 `calibrate_reference_pose`：沿用手柄/工具轴约定求
  `R_base_vr = R_base_tool * inverse(R_hand_tool) * inverse(R_vr_hand)`，偶数帧旋转均值拟合，
  奇数帧验证。四元数均值和误差用 Rotation 计算，q/-q 不产生平均抵消。
  手柄稳定阈值 15mm/3°、机器人 2mm/1°，留出样本重建误差上限 3°；它们检查一致性，
  不证明人摆出的平移/朝向关系符合期望。
- 采样与清理成功后才原子替换 `configs/a10/vr_calibration.local.json`；旧结果备份为
  `.json.bak`。记录时间、主机、侧别、单位、样本范围、参考姿态、质量及两个四元数。
  接收时间不是控制器实际采样时间，因此记录 `controller_sample_age_verified=False`。
- 配置新增 `pose_calibration_path`；新模式缺少显式基座/VR 旋转时自动加载。核对主机、
  侧别、已声明单位、轴约定及四元数格式；旧模式忽略文件，完整显式配置优先。
  默认结果和备份均已被 Git 忽略。回放 CLI 可直接读取新保存的完整报告。
- 网页补充每秒一次 WebSocket 重连及组件清理，避免标定结束/下一进程启动时必须刷新页面。
  重连不创建新的 XR 会话。需保持页面和 XR 追踪空间；刷新、退出 VR、重新居中后应重新
  确认/标定，程序目前不能自动识别追踪原点是否变化。

### 验证

```bash
PYTHONPATH=/home/robot/VLA/FluxVLA_vr/test \
  /home/robot/miniforge3/envs/fluxvla/bin/python -m unittest \
  test_a10_operator test_vr_reference_runtime test_vr_operator test_vr_control test_vr_calibration
PYTHONPATH=/home/robot/VLA/FluxVLA_vr/test \
  /home/robot/miniforge3/envs/fluxvla/bin/python -m unittest test_vr_native_entry
```

结果：**111 + 9 = 120 项通过**（约 14.1 秒 + 6.9 秒）。新增用例覆盖非对易旋转拟合、
符号等价、稳定性拒绝、重复/回退/过期、慢 GET、真实一秒回环采样、只读命令、保存/备份、
失败不覆盖、自动加载及不匹配拒绝、CLI 零连接预览、完整标定文件回放。模拟控制器的命令
集合为 `{get}`，笛卡尔和关节 UDP 均无数据，带入摄像头/USB/allow_motion 配置也未打开设备。

网页脚本另在内置 V8 中用假 WebSocket/定时器验证语法、断线重连与组件移除后取消重试，
未创建真实连接或 XR 会话。使用步骤及可复制命令已同步 `a10_vr_operator.md`。

**仍需用户现场标定与只读方向核对。** 开发测试没有产生现场标定文件，没有执行实机运动，
也没有解决原生 follower 的启停行为；本轮校验不能替代真实方向、跟踪及停止验收。

## 2026-10-19 16:34 / 16:35 两轮异常：标定有效性与奇异点证据

本轮仅只读分析，本机未发送运动、使能或停止命令，未修改控制器、标定结果或控制逻辑。
上轮提出的“初始对齐阶段”方案尚未获准实施，当前运行代码仍会直接生成绝对朝向参考。
用户确认：15:53 标定后保持同一 VR 页面/会话；16:35 没有人工急停或操作使能。

### 两轮独立记录

从追加的 teleop_diagnostics.jsonl 按 control_frame_index=0 分段，分别为：

| 会话 | 采样帧数 | 接管时朝向差 | 手柄握持内变化 | 结束情况 |
| --- | --- | --- | --- | --- |
| 16:34:24 启动，16:34:59 接管 | 1101 | 118.15° | 最大 8.12mm / 3.56° | 旋转参考误差 20.12°，tracking_error；停止状态确认成功 |
| 16:35:21 启动，16:35:28 接管 | 244 | 88.71° | 最大 5.46mm / 3.08° | 16:35:29 松开握键；停止确认期间收到 -501，未确认 Stopped |

第二轮不能当作又一次 tracking_error：最后控制结果为 inactive，随后是停止过程中控制器返回错误。
本机正常回传采样中 J5 速度峰值分别约 4.63°/s、7.79°/s；停止阶段缺少逐周期反馈，
不能据此断言用户感到的快速运动没有发生，也不能还原停止瞬间真实峰值。

控制器 syslog.db 使用 mode=ro&immutable=1 查询，原始事件为：

- 16:35:01：停止跟随（ID 1448）。
- 16:35:29：停止跟随（ID 1449）。
- 16:35:30：连续六条“机器人1接近奇异点”，CODE=987298（ID 1450～1455）。
- 随后“电机1没在使能模式，当前周期818”，CODE=-501（ID 1456）。

这支持把奇异构型/控制器停止路径纳入主因调查，不能再只解释为前端参考超差。
没有逐周期控制器轨迹或使能状态变更原因，尚不能确定是谁/哪个保护环节改变了使能状态。
J5 最后回传值约 -13.20°；奇异点由控制器明确报告，不仅是根据关节角猜测。

### 标定链路复核与限制

- 同一保存文件被加载。用 B*H*C 重算两轮 user_target，最大差约 7.1e-15°；
  标定参考姿态重建误差约 0.0263°。没有查到本轮公式乘法顺序、xyzw 排列或额外重复轴映射问题。
- 根据 sent_relative_target 重建带符号 321 角，单帧最大变化分别约 0.620° / 0.554°，
  没有日志证据支持先前 pq 负角变成整圈的同类错误；重建值不冒充线上实际抓包。
- 但标定的物理有效性未被证明。单姿态可按选定的手柄/工具轴约定构造出一个精确拟合的 B，
  静止样本留出验证只能检验一致性，不能独立确认 B 是 VR 固定空间到真实机器人基座的关系。
- 本文件的 B 把 VR +Y（向上）映射为基座向量约 [-0.3882,-0.4585,0.7994]，
  与基座 +Z 相差 36.92°。若基座 Z 为现场竖直向上，则需重新审查标定过程及轴约定；
  不能仅凭约 0.187° 的采样重建误差宣称真实坐标标定已正确。
- 用户确认同一会话，不能继续把本轮变化归因于用户刷新/重置 VR。握键时手柄数据相对标定
  姿态分别变化约 157.24° / 145.02°，而模型仍把它作为大角度绝对旋转需求接受。

### 后续修复顺序（待确认，不在本轮实现）

先独立确认固定操作坐标方向，再以它为已知量确定手柄/工具姿态偏置；旧增量轴映射只能作为
约定起点，不能替代物理方向验证。实机接管前需要大角度目标拒绝/明确对齐阶段；Cartesian
路径还需控制器侧关节速度、加速度及避奇异保护，并确认 stop_follower 是否取消旧目标并制动。
仅降低前端末端角速度或扩大跟踪误差门限不能覆盖这些问题。

独立摘要：work_dirs/a10/vr/two_trials_1634_1635_analysis.json（Git 忽略）。
本轮未开发或执行新的对齐算法，不应继续按原命令重复实机验证。

## 2026-10-19：新增只读标定验证网页

### 实现范围

- 在 `kaanh_follower_vr` 新增 `--verify-calibration`，与观察、检查、标定入口互斥。
- `vr/calibration_view.py` 集中管理只读采样、绝对目标计算和线程安全网页快照；复用
  `CalibratedAnchorState` 的 B*H*C 和平移锚点，不运行 reference governor、IK 或运动生命周期。
- 机器人使用白名单构造的监视连接，强制 `allow_motion=False`，过滤相机、USB 夹爪和状态轮询配置。
  启动、运行及关闭只查询 get / 关闭连接，不调用坐标切换、使能、follower、UDP 目标或夹爪接口。
- 复用现有 HTTPS 服务，新增 `/calibration.html`、只读 `/api/calibration-view`。
  电脑观察页不接入手柄 WebSocket，关闭观察页不会触发 VR 输入客户端释放逻辑。
- 网页使用原生 Canvas 投影绘制简化夹爪，叠加实际/目标姿态、前/上箭头和射线，单独显示平移。
  支持调整观察角度、冻结历史帧、人工逐项记录、下载一份含样本的 JSON 报告；不持续写新日志或覆盖标定。
- 实际射线从 A-Frame 运行时 raycaster.direction 读取（包括设备偏置），不是默认猜测 -Z。
  新增诊断元数据贯穿网页→WS→VRInput；使用当前 XRFrame 的有效手柄追踪状态，旧页面缺字段会明确报错。
- 默认 TCP +Z 前、+X 上仅作为待确认的旧约定；允许选择正负坐标轴并拒绝前/上共轴。
  修改显示轴不更改标定，且清空旧人工结果，避免把旧判断用于新定义。
- 反馈故障、运动中、follower 运行、VR 追踪失效或样本过期时清除实时姿态；即使采样阻塞，
  HTTP 快照也按时间失效。冻结帧保持历史标签，连接状态继续更新。

### 验证记录

- 回归命令：

  ```bash
  PYTHONPATH=test /home/robot/miniforge3/envs/fluxvla/bin/python -m unittest \
    test_a10_operator test_vr_reference_runtime test_vr_operator test_vr_control \
    test_vr_calibration test_vr_native_entry test_vr_calibration_view
  ```

- 共 **129 项通过**（35.175 秒），其中新增 9 项覆盖大角度无 governor 故障、非交换旋转、
  q/-q、射线映射、平移重新接管、过期/错误/回退、元数据传递、HTTP 与预览入口。
- 本地模拟 WebSocket 控制器验证：即使传入 allow_motion=True、相机和 USB 夹爪配置，
  实际命令集合仍只有 `get`，两路 UDP 均未收到数据，标定文件 SHA256 不变。
- 修复一项旧测试对未跟踪现场标定文件的依赖：配置校验测试显式关闭本地文件加载，
  不改用户的现场文件，也不更改生产配置加载逻辑。
- JavaScript 语法及 7 项射线/追踪辅助检查通过。Firefox 本地模拟网页已验证实时绘图、
  冻结、逐项样本记录、报告导出及切换轴后清空记录；截图在忽略目录
  `work_dirs/a10/vr/browser_check/view_live.png`，使用的是明确的模拟姿态。

### 当前限制与现场下一步

本轮没有连接 0.125 或使机械臂运动；VR 真设备射线、夹爪物理轴和现场坐标对应仍待用户只读核对。
射线与目标共用 B，它们重合不能独立证明 B 正确；应以实际夹爪和现场已知方向作独立参照。
原单姿态标定算法、初始对齐、控制器奇异点/关节速度/停止问题本轮没有修改，也没有被网页检查证明通过。
入口与五项检查步骤见 `docs/a10_vr_operator.md`。

按用户追加需求，新增精简入口 `docs/a10_command_cheatsheet.md`，集中区分只读标定验证、
VR-only、预览和控制器状态检查，注明旧观察/标定入口的区别；使用说明顶部增加链接。
速查表优先展示当前需要的只读命令，避免把未验收的实机绝对控制命令当作默认流程。
