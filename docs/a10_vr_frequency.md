# 双臂遥操作：控制90Hz，采集30Hz

2026-11-13，开发分支 `kaanh_vr_90hz`，工作目录 `/home/robot/VLA/FluxVLA_90hz`。
主目录的分支和同事未提交修改不受本次开发影响。

## 状态查询为什么不必跟控制一样快

原流程每轮等待控制器 GET 回包，再计算 reference、下发 UDP。查询耗时会直接占用控制周期。
现在共享服务有一个独立查询线程，本机另有一个后台线程读取共享回复并解析。
控制循环每轮只复制已解析的内存快照，不等待GET、本机缓存通信或JSON解析。
后台重复读到同一序号时复用解析结果，不刷新原始接收时间。

| 环节 | 本次默认 | 配置位置 |
| --- | --- | --- |
| 遥操作主循环、reference更新、双臂UDP目标 | 90Hz目标 | `teleop.fps=90` |
| 控制器状态GET | 30Hz目标，独立调节 | `teleop.feedback_poll_hz=30` |
| 三路图像与真实关节/夹爪采样 | 30Hz目标 | `record.fps=30`、`capture.sampling_mode=background_30hz` |
| 夹爪串口线程 | 保留20Hz | `teleop.zx_grippers.io_hz=20` |

30Hz查询时，约3轮控制共用一次真实反馈；每轮仍检查缓存年龄、故障、跟随模式与双臂数据完整性。
降低查询频率不会直接卡住UDP，但会降低实际位置和故障的更新速度，影响误差判断与重新接管时的定位。
因此不能无限降低，更不能刷新旧数据的时间戳伪装成新反馈。

reference按照实际控制时间推进，控制器内的电机闭环并不靠这个GET线程执行。
本次保留运动速度、加速度、力矩和行程参数；90Hz是更新频率，不是提高机器人运动速度。
开始跟随、停止确认等状态切换仍查询新反馈；确认阶段不承诺90Hz下发目标。
释放后的实际静止判断使用反馈更新时间，重复读取同一帧不算多次静止观测。

## 首版测量（本机缓存通信尚未移到后台）

 `.150` 静止状态只读查询，各5秒，无使能、运动或控制器文件修改：

| 设定查询频率 | 实际观测 | GET平均 / P95 / 最大 | 缓存读取平均 / 最大 |
| --- | --- | --- | --- |
| 30Hz | 29.88Hz | 10.87 / 14.15 / 20.13ms | 3.54 / 6.61ms |
| 60Hz | 59.22Hz | 10.98 / 13.89 / 19.03ms | 3.98 / 7.24ms |

两轮缓存读取均达到450次/5秒。90Hz每轮预算约11.11ms，GET的慢回复明显超过这个预算，因此应解耦。
默认先取30Hz；可单独尝试60Hz，不必把GET也强行设成90Hz。短时测量不保证运动时、长时间或多客户端时的表现。

额外离线验证：

- 实际双臂runtime、reference、共享服务、UDP协议和JSONL日志，在本机假控制器上运行3秒；每次GET故意延迟20ms，控制约89.67Hz，发出270个双臂包，单步平均9.24ms、P95为10.30ms。手柄与机器人反馈为合成输入，没有连接实机。
- 录制独立测试包含实际LeRobot图片、视频、Parquet写入：控制侧录制调用90Hz，后台采样30Hz，数据时间轴仍为1/30秒。慢写盘、停止等待、spike续录和m_init录制有回归覆盖。
- 上述为两项分开的验证；尚未验证真实VR、三路相机、双夹爪、运动与录制同时运行时能否持续90Hz。现场应检查实际循环间隔和数据采样间隔。

原始报告位于 `work_dirs/a10/frequency_checks/controller_get.json`、`control_loopback.json`（运行产物不提交Git）。

## 本机后台缓存与反馈过期恢复（2026-11-13补充）

现场16:28运行中，一轮反馈调用耗时约235ms，使用时反馈年龄约260ms，超过250ms限制而退出。
这个耗时包含通信等待、解析和线程被暂停的时间，尚不能认定解析本身就是235ms停顿的根因。

本次把本机缓存通信和解析也移入后台，正常90Hz循环只读取内存。启动、停止等明确的状态确认仍用新GET，并把确认状态发布到内存；晚到的旧回复不能覆盖新确认。后台错误仍会传给前端，不自动重连或重发目标。

反馈过期后的操作：

1. 锁定双臂接管、暂停夹爪输入，请求停止并确认；头显显示 `robot paused`。
2. 缓存仍旧或机器人状态不满足要求时，显示 `waiting for robot feedback`，不反复发送停止，不发送运动目标。
3. 必须取得停止确认之后的新反馈，且无故障、跟随未运行、运动状态Stopped、电机Enabled；双手松开、摇杆回中、VR及机器人新反馈连续稳定至少0.5秒后，显示 `robot ready`。
4. 重新按侧键，用当前实际位姿重建锚点；一直按住侧键不会自动恢复，旧目标不重放。
5. 若停止无法确认，显示 `stop unconfirmed; check controller`，本会话保持锁定；即使后来状态变好，也须现场核对后退出重启。不会自动使能。

这里只改变反馈过期导致的控制退出；真实控制器故障、畸形反馈或断开的通信仍走原有错误处理。录制入口与头显保留，录制质量保护不放宽：反馈过期引起的暂停仍沿用当前episode失效规则，可在恢复后废弃重录；不会用过期关节值补造有效训练帧。

验证结果：

- 纯本机假控制器，每次GET故意延迟20ms：内存读取270次/3秒，平均0.096ms、P95 0.118ms、最大0.161ms；后台反馈约30.04Hz。报告 `work_dirs/a10/frequency_checks/local_cache_background.json`。
- 实际双臂runtime/reference/共享UDP/JSONL本机模拟3秒：约89.37Hz、269个双臂包，单步平均9.03ms、最大14.77ms。无真实VR、相机或夹爪负载，不能当作现场持续90Hz验收。报告 `work_dirs/a10/frequency_checks/control_loopback_memory.json`。
- 282项相关Python回归及12项新恢复测试通过，头显JS检查、配置preview通过。覆盖慢缓存、线程退出、旧回复倒灌、保持侧键不恢复、掉使能不解锁、停止未确认锁定、非法时间戳和真实故障不伪装成普通过期。

继续使用本页的启动入口，不需要新增参数。退出旧进程后重新启动；刷新头显网页载入新版提示。本次未操作实机、未修改CLI或运动参数。降低正常读取开销不代表已经排除了原始235ms停顿的所有诱因。

## 使用方法

先从新工作目录检查配置，不连接设备：

```bash
cd /home/robot/VLA/FluxVLA_90hz
conda activate fluxvla
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --record-config configs/a10/a10_dual_arm_record.py --preview
```

正式验证继续使用原有teleoperate入口。新分支默认90Hz控制、30Hz查询和30Hz录制。
建议使用独立的新数据目录，避免本次相机fps写入采集约定后与旧数据目录的严格resume校验冲突：

```bash
python scripts/teleoperate.py \
  --config configs/a10/a10_dual_arm_vr_teleop.py \
  --record-config configs/a10/a10_dual_arm_record.py \
  --record-options record.root=datasets/a10_dual_90hz_trial \
  --cfg-options teleop.record_controller=False
```

此命令沿用双臂配置的实机、夹爪与运动开关。首次用新目录；续录该目录时再加 `record.resume=True`。
若主目录旧控制台还在运行，先正常退出旧控制台和旧VR，等共享服务自动退出后再从新目录启动；不要让旧共享服务承接新API。
旧数据不自动迁移、不覆盖。

只读重新测试查询性能，无需VR：

```bash
python scripts/a10_feedback_frequency_test.py \
  --config configs/a10/a10_dual_arm_operator.py \
  --rates 30 60 --duration 5 \
  --output work_dirs/a10/frequency_checks/controller_get.json
```

加 `--synthetic --get-delay-ms 20` 则只访问本机假控制器。无需运动或使能。

## 修改记录

- 双臂配置增加独立查询频率；共享服务统一GET缓存，查询与UDP发送锁解耦，目标所有权保护保留。
- A10Operator增加后台查询订阅和缓存读取接口；原同步接口继续用于明确的状态确认。
- runtime读缓存、保留真实时间戳，记录目标控制/查询频率；释放静止判断区分新旧反馈。
- 录制默认后台30Hz，写帧移出主循环状态锁，避免图片入队拖住接管和按钮；相机fps明确传入30Hz。
- 未修改控制器、OpenPI、数据state/action格式、运动速度、夹爪力矩或行程。

验证记录：274项相关回归首次发现1项旧同步查询断言不适用于后台查询，已拆为同步/后台两例并修复缓存错误来源传递；修正后29项反馈/控制权测试通过，其余247项原轮已通过，覆盖合计276项。头显JS检查通过，配置preview通过。另抽查完整旧operator套件时有6项旧断言失败，使用HEAD原代码复现相同结果，未在本次修改无关测试。
