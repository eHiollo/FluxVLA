# A10 timed_ee 接入

`timed_ee` 是新增的可选末端后端。现有 Cartesian follower / UDP9998 和 timed_aj 保留；旧配置继续走原路径。新模式严格使用物理右臂 ee0 在前、物理左臂 ee1 在后的两个绝对 tool1/wobj0 位姿，单位米、xyzw 四元数，UDP10000 普通 JSON，无 ARIS 头，无夹爪字段。

## 加载前提

已核对现场 2026-11-26 提供的 `timed_ee_upstream_protocol.zip` 全部说明、三个客户端源码、两份说明和五项 SHA256，适配其 tool1 / 默认 2 mm 快速收尾协议。本次没有重启 `kaanhbin`、使能电机或启动任何实机流。注册 XML 之后仍需现场按原有流程重启加载新版底层库。运行旧版 Python 共享服务的进程也必须退出后重新加载代码；请先结束所有控制程序，在控制器确认 Stopped 后进行。

项目现有 `configs/a10/a10_dual_inference.py` 已配置控制器网卡的 UDP10000 和独立 Plan 5888 路由，新 EE 配置复用它。不要让 timed_ee 和 timed_aj 同时启动。协议不匹配、旧共享服务不认识新 RPC 或底层没有 TimedEe 时明确报错，不自动切回 follower。

命令、状态和停止复用现有共享的串行 command WebSocket。长 Plan 使用与现有 timed_aj 一样的独立 5888 回复连接，避免完成时才返回的 Plan 回复混入 GET/stop 的回复。发送循环不建立新的 command 连接。

## 旋转按钮面板

```bash
cd /home/robot/VLA/FluxVLA
work_dirs/wallx/venv/bin/python scripts/a10_rotation_test.py \
  --backend timed_ee --timed-ee-network-ready
```

在浏览器打开终端打印的完整本地地址。每个按钮只测一侧的单轴 ±5°，另一侧显式保持当前姿态；支持台面轴、工具自身轴和 Wall-X RPY。每次测试约 2 秒，60 个 30 Hz 新点，只有最后一点 final=true。另有返回、停止、观察记录和 JSON 下载。详见 [旋转测试](a10_rotation_test.md)。原 follower 面板仍可直接不加 `--backend` 运行。

## Wall-X 多块推理

若要从机器人当前位姿仅执行模型预测块内部的变化，可选择 [相对动作模式](a10_wallx_delta.md)。它有意舍弃每块第一帧的绝对动作，不改变云端归一化或现有模型映射。

先检查配置（不连接任何设备）：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py \
  --config configs/a10/a10_wallx_timed_ee.py --mode preview \
  --contract configs/a10/a10_wallx_forward_experiment.json
```

在确认新底层和共享服务已加载后，由操作员启动运动：

```bash
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py \
  --config configs/a10/a10_wallx_timed_ee.py \
  --mode motion --cycles 100 \
  --contract configs/a10/a10_wallx_forward_experiment.json \
  --prompt "fold the black cloth using both arms"
```

每轮使用原始预测的前 6 个未来目标，每个新目标只发一次，30 Hz；没有 follower 插值、5 mm 试验缩放、块间 stop/start 或到位门控。普通块 final=false，整次推理流最后一个目标才 final=true。同步观测和云端推理在块间进行，底层可收尾保持再续接；这是同步推理，尚未加入 RTC。模型标注周期 33.7 ms 与发送周期 33.333 ms 的对应在配置中明确检查。

模型输出仍按已确认 JSON 的 model EE/tool0 定义转换到 rig 抓取中心，再输出各自原生基座的 tool1/wobj0。SDK 自动选择已标定 tool1；上游不重复添加 295 mm。界面选 tool0 或 tool1 的反馈均先还原同一抓取中心。夹爪仍走既有独立 USB 控制，不混入 EE 数据包。

## 接口与结束语义

`A10Operator` 新增：

```python
start = robot.start_timed_ee(options)
robot.send_timed_ee_target([right_tool1, left_tool1], final=False)
# 普通块只续发；全流最后一点：
robot.send_timed_ee_target([last_right, last_left], final=True)
command_status = robot.finish_timed_ee(timeout_s=10.)
# 独立验证上游 GET 反馈，然后释放自己的任务：
robot.stop_timed_ee()
```

`timed_ee_status()` 是只读命令 FK 状态，不会认领任务。Operator 自动记住本次 stream_id，目标/完成/停止同时检查会话与编号，拒绝影响其他任务。未确认停止时保留租约，避免故障后让新发送者误用原任务。完成等待期间共享服务仍允许控制台停止。

新版完成要求 final 消费计数匹配、`endpoint_capture_complete=true`、关节命令在状态声明的候选角度容差内（上限 0.3°，可自动收紧）、命令关节速度 <=0.5°/s、tool1 命令 FK 残差 <=2 mm / 0.2°，满足即完成，不额外等待 40 ms。低速度或 final 已消费都不能单独认定结束。只有同一个仍 active 的任务可以成功完成；提前 inactive 视为中断，与包内客户端一致。`waiting_for_input` 不表示整个流结束。reason=10/11 等错误会中断，不自动重发或换任务续跑。

状态通过 `ee_endpoint_policy=bounded_cartesian_capture` 识别新版，要求 `parameters.endpoint_tolerance_deg<=0.3` 及布尔 `endpoint_capture_complete`。历史 `exact_ik_goal` 仍单独按 0° 关节容差、1 mm / 0.1° 几何阈值识别，不把旧协议猜成新版。未知策略或旧 0.5° 版本明确拒绝。新版 Wall-X 配置和旋转面板的 GET 到位比较也按 tool1/wobj0 的 2 mm / 0.2° 进行。旧 tool0 状态或报文不兼容新版接口，不能仅替换报文标签而复用旧 TCP 坐标。SDK 启动自动选择 tool1 后，已知 UI 工具变化不会误判为标定变化。

现场只读核对发现，inactive 的 `tee_status` 会保留上一轮样例运行的 `input_source=sample30_ee_fk_ik`。现可读取这种已停止的历史状态；active 状态仍必须是 UDP，旧历史不能用于发送，也不会被当成本次任务。面板在记录启动尝试前先只读查询 EE 状态；若共享服务不认识该接口，会提示重载，且不会误记一次未确认停止。

现场在 joint/init 后也会返回 inactive 的 `implementation=joint_tracking.TimedAj` / `control_space=joint` 历史。未认领 EE 任务时只校验这份历史的 ret_code、active 和 stream_id；只有新启动后的 EE 任务才校验完整 tool1、UDP 和完成策略，历史状态不授予发送权限。服务会在启动错误中携带“是否已发送 Plan / 已知任务编号 / 是否释放租约”，上游据此清理：确定未启动不发送 stop；已知任务只停止本会话编号；启动结果不确定且无编号时不猜测停止其他任务。原始只读诊断可用共享 `timed_ee_status` RPC 的 `raw=true`，不抢占控制。

UDP 四元数处理与包内 `encode_ee_target` 一致：平方范数误差 <=0.002 才进行归一化，并保留符号。EE 的 Plan 启动帧与包内 `Client.start_plan` 对齐（seq=1、保留字段为零、命令以 NUL 结尾），接受 NUL 结尾的 Plan 回复。旧 follower/timed_aj 的编解码未修改。

2026-11-26 最新 tool1 包 SHA256 为 `7dbfd46c589131ffe75f80ef6cad5c9cf642ad6cf2046b619c6c57a402975eff`；包内五个文件均符合 SHA256.json。旧 tool0 包的对照属于历史记录，当前以 tool1 包为准。结果在 `work_dirs/wallx/timed_ee_upstream_protocol_audit.json`。没有导入或启动包内机器人客户端，没有连接实机。

新配置仍保留上游整块校验，包括观测漂移、末端目标变化、桌面和抓取中心间距；这些只验证目标点。底层执行逐目标 IK 后的关节受限跟踪，不保证严格笛卡尔直线，也没有独立 TCP V/A/J 上限。初始每轴 0.035 rad 衔接、后续 IK 每轴 0.35 rad 跳变检查由底层执行，不自动生成大距离接近动作。

最终 GET TCP 数值检查与 command FK 完成分开记录。项目已证明关节 `actual_pos` 是实际反馈；尚未确认 `motion_msg.pq` 来自实际关节 FK，故新日志标记 `tcp_pose_measured_fk_verified=false`、`confirmation_scope=GET_tcp_pose_only`，不会把 GET 到位或虚拟测试当作真实电机精度证据。现场旋转方向由操作员的观察报告确认。
