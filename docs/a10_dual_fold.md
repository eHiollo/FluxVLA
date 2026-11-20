# A10 双臂推理与叠衣服模型

## 共用推理控制配置与后端选择

双臂推理统一使用 `configs/a10/a10_dual_inference.py`。默认选择
`fluxvla`，权重目录为 `checkpoints/FluxVLA-Cloth-Folding-04-50k`。
模型选择仅改变传输协议、默认服务端口、提示词和服务器数据预处理，
两种模型共用 12% follower、手动倍率 100%、20 个 30Hz 策略点各发两遍
（60Hz、40 包）、到位后停止并获取新观测的控制流程。
默认 motion、100 万轮、单次推理超时 3600 秒、双夹爪报警 warn。

启动 FluxVLA 模型服务（单独的终端，不会打开机器人）：

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_dual_inference.py \
  --model fluxvla --from-config --dtype fp32 \
  --host 127.0.0.1 --port 5556
```

服务就绪后启动运动客户端：

```bash
python scripts/inference.py --config configs/a10/a10_dual_inference.py
```

切回已经启动的 OpenPI 服务，只改后端选择：

```bash
python scripts/inference.py --config configs/a10/a10_dual_inference.py \
  --model openpi
```

也可在配置中修改 `model_backend`，或使用 `A10_MODEL_BACKEND=openpi`；
CLI `--model` 优先。OpenPI 默认连接 `127.0.0.1:8001`，
FluxVLA 默认连接 `127.0.0.1:5556`。端点和超时仍可通过
`--cfg-options inference.remote_inference.server_host=...` 等覆盖。
`--cfg-options inference.mode=preview` 仅打印并校验配置，不连接设备/模型。
远程客户端不会加载本地权重、tokenizer 或统计文件；只有模型服务需要它们。
原 `a10_dual_openpi_remote.py` 和 `a10_dual_fold_inference.py` 保留为兼容入口，
控制参数均继承同一公共配置；前者始终选择 OpenPI，后者选择新版 50k FluxVLA。
换其他 FluxVLA 权重时，可设置 `A10_DUAL_CHECKPOINT=/完整目录`，
模型结构、权重、tokenizer、统计文件须来自同一兼容模型包。

## 控制流程与参数

每轮依次执行：新观测 → 模型输出 → 完整动作校验 → 发送20个策略点 →
实际位置到位 → 停止follower并核验 → 获取下一轮新观测。
策略频率30Hz，每点重复两遍，以60Hz发送40包；整块原始时长20/30秒。
客户端不插值、不规划、不生成滤波补偿点，推理等待期间不发送保持目标。
关节UDP每包明确 `filter=false`，保留原40字节ARIS消息头和角度单位。
这要求控制器已支持JSON关节报文；笛卡尔遥操作报文和滤波不由推理配置改变。

最终关节目标的12轴最大位置误差≤0.5°时即可停止，无固定驻留或低速门槛。
停止前必须保持Enabled和关节follower运行；停止后用新反馈确认位置仍在容差内、
follower inactive、motion Stopped。停止完成前不允许下一轮观测。
机器人及三路相机接收时间须晚于最终核验，避免使用旧缓存。
到位等待上限30秒；过期反馈、故障、超时及停止未确认均阻止下一轮。

推理follower倍率12%、手动倍率100%；电机加速度目标为±375°/s²。
运动连接建立后先回读加速度，由公共预检机制处理不匹配；不会自动使能。
电机速度150/180°/s时，有效速度上限18/21.6°/s，有效加速度约129.9°/s²：
`v=motor_v*follower*manual`，`a=motor_a*sqrt(follower)`。
电机限制是控制器全局参数；推理入口与遥操作入口分别声明目标，不能只改本地注释。
手动倍率正常退出时恢复启动前值。

## 训练与数据约定

物理左臂ID1六个实际关节(rad)+左夹爪，再物理右臂ID0六关节+右夹爪，共14维。
夹爪0张开、1闭合；相机为base、left wrist、right wrist三路RGB，按设备配置旋转一次。
FluxVLA模型服务使用模型包内的tokenizer、统计及预处理；客户端接收绝对关节目标，
不重复反归一化或叠加状态。两只夹爪由原20Hz线程消费最新目标。

训练适配示例保留在 `configs/pi05/pi05_paligemma_a10_dual_fold_10steps.py`，
该10步示例用于接口验证；当前部署默认使用50k模型，不使用该测试权重。

动作完整块在下发前检查有限值、维度、夹爪范围、相邻关节变化和电机位置范围；
默认相邻步长上限0.2rad。同步发送前查询状态并检查权限、新鲜度、故障和发送期限。
机器人/相机反馈年龄上限150ms，夹爪200ms，时间偏差150ms。
取消及故障沿用现有停止清理，不自动回零、使能或重放网络失败的动作。

## 加速度控制台命令

`set_acc 375` 修改全部12轴，`set_acc 375 --id=0` 仅修改索引0。
单位°/s²，ID为控制器电机索引0..11，不是slave_id。
手动命令要求厂家权限、手动模式和无故障；按既有流程停止、卸载、掉使能，
保留完整配置，只修改max_acc/min_acc，逐轴回读。
备份与部分失败记录位于 `work_dirs/a10/motor_acc_commands/`；超时不重发、不自动回滚。
命令结束保持Disabled/Stopped/Unloaded，不恢复加载或使能。

## 日志与离线验证

日志位于 `work_dirs/a10/inference_logs/`，包含配置、观测、模型耗时、发送及反馈时间。
`arrival_position_reached.after_last_packet_s` 是末包UDP发送起始至首次位置到位反馈的耗时；
`arrival_confirmed.after_last_packet_s` 还包括停止与停止后新反馈核验。
这些是主机时间，不是控制器采样时间；UDP发送成功也不证明控制器接收。
`a10_execution_trace` 的速度/稳定统计仅为诊断，不给生产到位门控增加驻留。
检查summary中的dropped_events和write_errors，非零时日志不能视为完整证据。

```bash
PYTHONPATH=.:test python -m unittest test_a10_follower_tail test_a10_framework_loop \
  test_a10_dual_inference test_a10_inference_backends -q
```
