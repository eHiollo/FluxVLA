# VR 控制算法迁移：阶段 1、2 开发记录

## 现在完成到哪里？

2026-10-16，在 `kaanh_follower_vr` 完成接口梳理和离线算法迁移。
新增算法已经可以读旧 VR 日志，计算将来要发给 UDP follower 的参考目标。
**目前只计算、记录，不发送。`teleoperate.py` 的真实机器人控制路径尚未切换。**

| 阶段 | 状态 | 本次交付 |
| --- | --- | --- |
| 1：功能与协议对照 | 已完成梳理，现场待确认项如下 | 明确已有功能、缺失功能和控制器职责 |
| 2：离线迁移 | 已完成 | 集中算法模块、原 C++ 数值对照、回放工具和测试 |
| 3：UDP 实机对照 | 未执行 | 确认原生 follower 行为，再接入新控制链 |
| 4：完整采集 | 未执行 | 相机、夹爪、episode 和训练数据联动 |

## 一、分工与文件位置

```text
VR 原始输入 → mapper.py → control.py → 参考目标及诊断结果
                                      ↓ 阶段 3 再接入
                               runtime.py → A10Operator → UDP
```

| 文件 | 职责 |
| --- | --- |
| `vr/mapper.py` | 复用输入映射、死区、异常样本处理；输出锚点偏移 |
| `vr/control.py` | 新增，集中锚点计算、参考目标、限速、加速度整形、跟踪检查与离线回放 |
| `vr/runtime.py` | 当前实际运行循环；本次未替换原有控制链 |
| `scripts/replay_vr_control.py` | 离线入口，直接运行纯数学模块，不加载模型/GPU，也不构建硬件对象 |
| `test/test_vr_control.py` | C++ 数值对照及前端适配测试 |
| `test/data/vr_control/` | 原 C++ 参考结果、生成工具及无 SDK 的 C++ 对照入口 |

不依赖 `projects/a10`。既有配置 `configs/a10/a10_vr_teleop.py` 保持原控制模式；
新增模块不是该配置中某个开关已经开启的隐含功能。

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
不是“机器人已经停稳”，也不是“停止发送 UDP 就一定会停止”。真实 runtime 适配器尚未接入；
阶段 3 必须处理停止确认、停稳后接管和故障复位。故障在离线模块中保持，需显式 `start()` 重置。
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

## 五、本次验证结果与开发记录

- 原 C++ 参考结果在本机编译生成成功；锚点、参考生成器与限幅原语对照通过。
- 新增控制模块测试 11 项，既有 VR 测试增加释放序号检查后共 23 项。
- 内置示例 120 帧：90 帧 tracking、30 帧 inactive；无故障，没有发送任何指令。
- 旧 VR-only 日志合成跟随回放 4,375 帧：492 帧 tracking、3,883 帧 inactive，无故障。
  其中 2,370 帧使用名义时间（主要是设备尚未提供有效样本的非活动记录）；不是实机验证。
- 回放发现旧 mapper 松手包复用活动包序号：修复新包序号，同时保留同序号释放兼容，
  避免把松手误当活动重复包。更旧的锚点/序号仍拒绝，重复活动包不会刷新超时。
- 本轮没有连接相机、VR 或机器人；既有 VR runtime、A10 UDP operator 和主分支未修改。
- 下一步为阶段 3：先确认未知 UDP 行为，再接新算法与实机做小范围对照；尚未授权/执行实机测试。
