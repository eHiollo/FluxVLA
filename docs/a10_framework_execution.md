# A10 与项目公共执行框架

A10 双臂继续使用 `BaseInferenceRunner._run_action_chunk` 的公共阶段：
`_preprocess → _predict_action → _postprocess_actions → _execute_actions`。
同步、异步和 RTC 使用同一个入口及同一个硬件适配器，不另建推理循环。

本次取消 A10 的强制同步限制。A10 与 ALOHA 共用
`engines/utils/chunk_execution.py` 的动作选择和旧块剩余前缀计算；
A10 与 `BaseOperator` 共用后台发布线程的启动、替换、等待及错误回传。
RTC 前缀约束复用模型已有的 `predict_action(prev_actions, prefix_len, rtc_config)`，
引导算法继续使用项目原有 `rtc_guidance.py`。

## 配置语义

| 配置 | 执行 | execute_horizon 的含义 |
| --- | --- | --- |
| `async_execution=False` | 阻塞发送，再观测/推理 | 本轮选取并发送前 K 点 |
| `async_execution=True` | 后台发送完整预测，前台观测/推理 | 发起下一轮观测前等待 K 个控制周期 |
| 上述异步配置加 `rtc_config.enabled=True` | 同时让模型受旧轨迹前缀约束 | K 必须给下一轮推理留下足够旧轨迹 |

默认 `a10_dual_inference.py` 仍为已部署的同步20点基线。
现有 OpenPI 可直接使用无模型改动的异步指数融合：
`python scripts/inference.py --config configs/a10/a10_dual_openpi_async_inference.py`。
参数和离线对照见 [a10_action_fusion.md](a10_action_fusion.md)。
公共配置下异步默认启用融合；需要未融合基线时显式设置 `inference.action_fusion.enabled=False`。
异步首块必须把完整30点交给后台发布，才能在等待 K 点后继续供点；
否则首块到 K 点即耗尽，下一轮推理期间仍然断流。
后续新预测按推理开始时刻建立参考时间轴，并复用项目原有
`resample_remaining` 跳过已过去的部分、对齐小数时间步。
这与同步模式“每次原始点单发”的动作选择不同，但不会重复发送旧点、
追赶补发过期点或放慢模型时间轴。夹爪与关节使用同一动作行对齐。

新块替换旧块的未发部分，不形成待逐块完成的队列。旧发送线程退出后新线程才能发送，
块间不停止或重启下游 Plan。跨块保持原30Hz发送间距，不允许新线程突发补发。
`stream_id/index` 仍由客户端内部管理，物理轴映射、UDP报文及下游限幅保持原约定。
现场下游原40ms主动播放缓冲已关闭；上游不另加40ms等待，也不自动配置或重新启用该缓冲。
末点停稳判定中的40ms稳定观察窗口用途不同，继续保留。
最后一块发完其剩余预测，只有整个任务最后一点带 `final=true`；
普通换块、模型推理或缓存暂空都不标记任务完成。

后台发布错误会立即请求停止，并传回主循环；下一次模型返回不能清除该错误并续发。
取消/关闭先通知发布线程退出，再停设备、关闭连接。键盘 `p` 暂停也停止后台发布并清除公共上下文。

## RTC 模型服务

14维物理绝对关节/夹爪前缀在模型服务内相对**新观测**重新编码，使用同一反归一化器的逆变换。
关节使用相对量，夹爪使用绝对量；最后补零到模型动作宽度（通常32维）。
这样不能把上一块相对旧观测的归一化输出直接当成新一块的相对量。

RTC 入口默认匹配现场 OpenPI WS/8001，不再默认选择 FluxVLA ZMQ/5556。
OpenPI WS 与 FluxVLA ZMQ 都按服务能力检查；握手必须声明支持14维绝对前缀和所选方法，
每次响应还必须确认启用、方法及实际前缀应用情况；缺少能力/确认时不发新动作。
当前本地 OpenPI 的 `Policy.infer` / `sample_actions` 未接入动作前缀处理，也不声明此能力。
因此该服务可以继续运行默认同步配置，但不能只换配置就当成 RTC 已完成；
显式选择 RTC 入口会在模型握手后、连接机器人之前报缺少能力。
已完成前缀应用和回环验证的模型服务是 FluxVLA ZMQ；需显式 `--model fluxvla` 才会选择它。
LongWAM HTTP 尚未接入 RTC 前缀协议。

`method='guidance'` 对应现有推理时引导；`method='prefix'` 用于经过相应 RTC 前缀训练的权重。
`use_vjp` 由模型服务的已有引导实现处理，会影响推理耗时。
仅接受 `**kwargs` 却不实际处理 RTC 的加速模型会被拒绝，不能把它当成成功启用。

提供 `a10_dual_rtc_inference.py` 作为显式入口：30Hz、K=0、prefix_len=12、decay_end=18。
K=0 沿用项目 ALOHA RTC 配置：提交后台轨迹后立即观测/推理下一块，发送仍为30Hz。
前12点（400ms）为锁定前缀。这是起始配置，尚未测量真实模型的
引导后耗时或实机衔接效果。若某次推理超出已约束前缀窗口，拒绝新块并停止；
应依据真实耗时分布调整 K/前缀，而不能靠延后整条时间轴掩盖超时。

非零 K 还要计入上一轮推理已经消耗的预测时长：稳态剩余窗口约为
`action_chunk × dt − 上轮推理耗时 − K × dt`，不能一直按 `(action_chunk − K) × dt` 计算。
例如30点/30Hz、推理277ms、K=20，稳态仅余约56ms，无法覆盖下一次277ms推理。
显式 `prefix_len=None` 可依据上次耗时向上取整再加一个周期，受剩余轨迹长度约束；
它是估计值，仍会核对本轮实际耗时，不等同于推理延迟上界。

仅查看配置，不连接模型或硬件：

```bash
python scripts/inference.py --config configs/a10/a10_dual_rtc_inference.py \
  --cfg-options inference.mode=preview
```

上述预览使用 OpenPI WS/8001，不加载 FluxVLA 权重。RTC能力仍需模型服务实际实现，
不是在配置中声明就能获得。保持现有 OpenPI 同步运行，仍使用
`python scripts/inference.py --config configs/a10/a10_dual_inference.py`。

下面仅供明确选择 FluxVLA 后端时使用，不是现场 OpenPI 服务的启动命令。
需要切换时，停止旧推理后由操作者启动本仓库的原生模型服务：

```bash
python -m fluxvla.engines.runners.serving.serve \
  --config configs/a10/a10_dual_rtc_inference.py --model fluxvla \
  --from-config --port 5556 --dtype bf16
```

此模型配置直接加载原生 PyTorch，无可选加速实现，不额外传 `--acceleration off`。
模型服务准备好且现场允许运动时，客户端仍用项目入口：

```bash
python scripts/inference.py --config configs/a10/a10_dual_rtc_inference.py --model fluxvla
```

## 离线验证范围

覆盖 ALOHA/A10 同步与异步的时间选择一致性、真实后台线程在模型等待期间继续发送、
新旧发布线程不重叠、编号连续、final不泄漏到旧块、发送错误立即停止、同步/异步取消与关闭。
回环 ZMQ 测试经过真实客户端、序列化、服务端、14维重新归一化、32维补齐和项目 RTC 引导函数，
核对握手和每次响应确认。另覆盖不支持的服务、超时、过期预测和未确认前缀不能续发。
这些测试不证明真实模型延迟、机器人跟踪精度或任务成功率，未启动实机运动。

2026-11-25 相关回归共312项：311项通过，1项真实GPU模型用例按默认设置跳过。
配置预览和 `git diff --check` 通过。复现命令：

```bash
PYTHONPATH=.:test:test/test_runners HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
python -m unittest -q test_a10_async_execution test_a10_framework_loop \
  test_a10_dual_inference test_a10_follower_tail test_a10_timed_aj \
  test_a10_rtc_guidance test_a10_inference_backends test_a10_operator \
  test_a10_init_motion test_m_init_grippers test_a10_vr_shared_control \
  test_a10_shared_control test_a10_longwam test_a10_inference_runner \
  test_a10_openpi_inference test_a10_gripper_tracking
```
