# A10 双臂推理与叠衣服模型

## LongWAM 远程推理

现场客户端支持 `--model openpi|fluxvla|longwam`。LongWAM 使用 HTTP JSON，
默认端口 8765，任务固定为 `Fold the clothes.`；无需现场加载模型或统计文件。
服务器继续运行已有 LongWAM 服务，不使用本项目的 ZMQ serve 命令启动 LongWAM。

在现场电脑终端一建立隧道并保持运行：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:8765:127.0.0.1:8765 jupyter-xiaoj@10.18.1.7
```

现场电脑终端二检查服务，然后启动只记录、不发送运动目标的客户端：

```bash
curl -fsS http://127.0.0.1:8765/health
conda activate fluxvla
python scripts/inference.py --config configs/a10/a10_dual_inference.py \
  --model longwam --cfg-options inference.mode=read_only inference.cycles=3 \
  inference.remote_inference.timeout_s=30
```

`--model longwam` 切换时默认使用 `read_only`；环境变量
`A10_MODEL_BACKEND=longwam` 和兼容配置 `configs/a10/a10_dual_longwam_remote.py`
也可选择该后端。`preview` 仅检查配置，不连接模型或设备。
确认预测和时间日志后，可显式指定 `inference.mode=motion`；不要用该命令作为连通性检查。
夹爪标定、故障和机器人状态检查继续使用原有机制。

三路后台相机各保留 64 帧原始 640×480 RGB（约 169 MiB 图像内存），
以顶视相机序号取 `-48,-44,…,0`，另外两路按主机接收时间匹配。
启动等待至少 49 帧，最长 10 秒；历史不补零、不复制首帧。
采样间隔、组内同步和最新帧与状态对齐容差为 40ms；缺帧或时序不符时拒绝请求。
这些时间为主机接收时间，并非硬件曝光同步。相机缓存贯穿推理等待及运动阶段，
不会在停止后重建整段历史。右腕已按设备配置旋转一次，不重复旋转。
历史窗口只在获取模型输入时校验；执行前及动作发送期间使用最新单帧检查设备反馈，
不重复校验13帧历史，仍保留原有相机、机器人及夹爪反馈新鲜度检查。

客户端将 39 张图编码为标准 Base64 JPEG。默认质量 90，可用
`inference.remote_inference.jpeg_quality=80` 调整；请求超过 16 MiB 会在发送前报错。
HTTP 请求不自动重试。健康检查必须 ready，并记录 checkpoint；响应校验
`obs_id`、checkpoint、32×14 有限动作、30Hz、零首步偏移和服务器耗时。
动作已经反归一化；继续采用现有夹爪范围裁剪和关节轨迹校验。

日志仍在 `work_dirs/a10/inference_logs/`，包含 `history_window` 的序号和时间、
`longwam_response` 的 obs_id/checkpoint/耗时、`prediction` 的完整动作。
通过 cycle 将观测、历史和 obs_id 对齐。只记录模式不会启动 follower 或下发夹爪目标。

运动模式采用串行执行：获取观测 → 等待推理返回 → 校验完整动作 → 从第0行开始，
按30Hz执行前 `execute_horizon` 个点（默认20）→ 获取下一轮观测。
普通块不等待到位、不停启；新版下游在同步推理空档保持末点，下一块到来后续接。
上游允许块间等待，不补发；无参启动及自动编号说明见 [timed_aj 接入状态](a10_timed_aj.md)。
HTTP超时、设备反馈新鲜度、关节步长和故障检查继续生效；推理及网络耗时仍记录到日志。

## 共用推理控制配置与后端选择

双臂推理统一使用 `configs/a10/a10_dual_inference.py`。默认选择
`openpi`，连接现场 `127.0.0.1:8001` WebSocket 服务。
选择 `fluxvla` 时使用 `checkpoints/FluxVLA-Cloth-Folding-04-50k` 模型包及ZMQ/5556服务。
模型选择仅改变传输协议、默认服务端口、提示词和服务器数据预处理，
OpenPI 与 FluxVLA 共用 timed_aj 固定 A、20 个 30Hz 策略点各发一次
（30Hz、20 包）的控制流程。上游保留现场倍率与电机配置。
默认 motion、100 万轮、单次推理超时 3600 秒、双夹爪报警 warn。

启动 FluxVLA 模型服务（单独的终端，不会打开机器人）：

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_dual_inference.py \
  --model fluxvla --from-config --dtype fp32 \
  --host 127.0.0.1 --port 5556
```

FluxVLA服务就绪后，显式选择对应的运动客户端后端：

```bash
python scripts/inference.py --config configs/a10/a10_dual_inference.py --model fluxvla
```

使用现场已有 OpenPI 服务（也是不指定模型时的默认选择）：

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

执行接口支持项目通用的 `async_execution` 与 `rtc_config`；默认仍是下面的同步基线。
异步供点、时间对齐、RTC 模型服务及启动方式见 [公共执行框架](a10_framework_execution.md)。

每轮依次执行：新观测 → 模型输出 → 完整动作校验 → 发送20个策略点 → 下一轮观测。
普通块尾不等待实际位置、不发stop/start、不标final。启动确认一次，发送阶段没有逐点GET/taj_status查询。
客户端不插值、不补发、不生成滤波补偿点；两只夹爪继续走独立USB工作线程。

timed_aj使用普通JSON UDP10000：绝对rad、物理右ee0六轴后物理左ee1六轴、不含夹爪。
长Plan回复独占5888，GET/taj_status复用共享命令连接，控制权仍由共享服务管理。
只有整个运行的已知末点final=true；正常结束时排空末点再释放Plan，取消/故障时请求制动停止。
生产固定A和V/A/J由下游代码管理，上游不改12%/100%倍率、不改250/375/1000电机加速度。
新版协议及当前网络、同步供点限制详见 [接入说明](a10_timed_aj.md)。

`m_init` 已取消全部电机加速度预检、临时改写和恢复；包括其关节启动与原模式恢复。
仍按原5%手动倍率回位、检查实际到位并恢复原倍率/控制模式。

## 训练与数据约定

物理左臂ID1六个实际关节(rad)+左夹爪，再物理右臂ID0六关节+右夹爪，共14维。
夹爪0张开、1闭合；相机为base、left wrist、right wrist三路RGB，按设备配置旋转一次。
FluxVLA模型服务使用模型包内的tokenizer、统计及预处理；客户端接收绝对关节目标，
不重复反归一化或叠加状态。推理夹爪电机速度为30%，线程以100Hz为轮询上限，
运行中只读位置、速度、电流（0x0418起4个寄存器），瞬时丢读跳过、不重试，
保留上次有效反馈时间；连续超过0.2秒无有效反馈仍暂停。
报警和使能在串口初始连接/故障重连时检查，精简循环不把旧报警/使能信息标成新采样。
两只现场夹爪已经验证可将参数与触发合并为单次RTU写入，约10–12ms确认。
读写共用串口，100Hz不是连续写入时的实收保证；单独采样约99Hz，
同一故障片段两次连续换目标测试实收左约70–74Hz、右约64–65Hz。
直接使用最新目标，不排队补发过期点，不等待到位或接触，不阻塞下一轮观测/推理。

推理串口下发时，两侧默认增加 `closure_bias_raw=450` 的闭合偏置，
在45 raw死区之前应用；例如模型8000对应下发8450，8800对应9000。
模型位于0～`closure_bias_fade_raw=900`时，偏置按模型位置线性减小，
所以全开0仍下发0，30只下发45，不把接近全开的模型噪声变成大幅闭合。
每次都从本次模型目标计算，不累计、不等待到位；原始预测与实测反馈不变。
`gripper_command`同时记录`model_target_raw`、最终`target_raw`和`applied_bias_raw`。
两侧可分别用`--cfg-options grippers.left.closure_bias_raw=300 grippers.right.closure_bias_raw=300`
调整；设为0关闭本侧偏置。该补偿是部署侧临时措施，不能替代夹持效果实机核验。

训练适配示例保留在 `configs/pi05/pi05_paligemma_a10_dual_fold_10steps.py`，
该10步示例用于接口验证；当前部署默认使用50k模型，不使用该测试权重。

动作完整块在下发前检查有限值、维度、夹爪范围和相邻关节变化；电机位置限位仍由下游SDK执行。
默认相邻步长上限0.2rad。观测/启动阶段检查新鲜度、故障和权限；发送阶段只做本地取消、所有权和发送期限处理。
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

用于底层调试的真实模型动作样例位于 `test/data/a10_model_action_sample/`，
包含完整 30 点预测、实际执行的前 20 点、历史运行的60Hz重复发送目标、初始姿态和来源日志。
历史样例保留原记录；当前执行使用30Hz单发，可直接读取 `actions_executed` 的20个策略点。
字段、单位及物理臂 ID 映射见目录内的 `说明.txt`；样例文件只供离线读取。

日志位于 `work_dirs/a10/inference_logs/`，包含配置、观测、模型耗时、发送及反馈时间。
每次连接后，`model.json` 和 `events.jsonl` 的 `connected` 事件记录模型服务报告的配置和 checkpoint。
OpenPI 模型目录有 ModelScope 来源记录时，也会包含仓库 ID 和版本。旧服务没有报告模型时，
`model.json` 中的 `source` 是 `unavailable`。
以下arrival事件仅用于旧follower路径或整个timed流的最终排空，不再作为普通块门控。
`arrival_position_reached.after_last_packet_s` 是末包UDP发送起始至首次位置到位反馈的耗时；
`arrival_confirmed.after_last_packet_s` 还包括停止与停止后新反馈核验。
这些是主机时间，不是控制器采样时间；UDP发送成功也不证明控制器接收。
`a10_execution_trace` 的速度/稳定统计仅为诊断。普通块发送成功不代表实际到位。
检查summary中的dropped_events和write_errors，非零时日志不能视为完整证据。

```bash
PYTHONPATH=.:test python -m unittest test_a10_follower_tail test_a10_framework_loop \
  test_a10_dual_inference test_a10_inference_backends -q
```
