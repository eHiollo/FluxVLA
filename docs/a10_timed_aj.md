# 双臂 timed_aj 接入与离线验证

已接入2026-11-25新版“自动编号、块间保持续接”协议，包内SHA256逐项通过。
[协议快照](a10_timed_aj_protocol.txt)和[来源指纹](a10_timed_aj_source.json)来自最新ZIP；
根目录单独提供的旧TXT不是本次依据。

## 控制流程

启动一次：健康/初始衔接检查 → 无参异步`timed_aj` → ready/accepting确认。
通信层从ready状态读取控制器实际分配的内部编号，不自行计算N+1。
业务循环：观测 → 推理 → 选择前20点 → 以30Hz逐点单发 → 下一轮观测。

- 首点可以一直等待；Plan就绪查询仍有通信超时，不是首点超时。
- 普通块尾不等待到位、不停止、不重启，final=false。
- 推理期间不补发保持点。下游走到已收末点后保持，下一块到来续接。
- 每块以当前时刻建立30Hz发送deadline，块间等待不算超期；块内超期则报告/中止，不突发追赶。
- 业务只提交关节目标和final，通信层自动封装内部stream_id/index，点号跨块连续。
- 发送循环没有GET/taj_status网络查询、相机采样或到位门控。
- 共享服务UDP发送只取目标所有权锁，不等待命令查询锁；取消和其他会话接管仍可撤销发送。
- 只有整个运行的最后一点final=true。正常结束时等待最终参考、命令和实际反馈收敛，再taj_stop释放。
- 停止确认后清除本地编号、计数、final、接受状态及目标缓存，关闭旧UDP套接字；不重放失败报文。
- 其他会话接管后，旧客户端清理不能停止后来任务。下游负责拒绝旧任务迟到包和清理接收缓存。

这是同步观测/推理流程，没有增加后台推理或RTC。块间允许暂停，不承诺运动全程无停顿。
下游暂停模型时钟并在续接时扣除暂停时间，不能把暂停后的运行称为连续全局30Hz时轴。
时延、误差和耗时验收必须保留真实墙钟的推理等待及保持时间。

## 报文与状态

UDP10000为普通UTF-8 JSON，无ARIS头、不转deg、不含夹爪。wire内部编号仍保留：

```json
{"stream_id":24,"index":0,"position_rad":[0,0,0,0,0,0,0,0,0,0,0,0],"final":false}
```

示例零值只说明字段，不能直接当作实机姿态。14维模型动作仅重排一次：
`[7,8,9,10,11,12,0,1,2,3,4,5]` → 物理右ee0六轴、左ee1六轴，绝对rad。
模型/操作者不填写wire编号。GET和taj_status复用共享命令5999；长Plan回复独占5888。

状态在taj_status响应顶层：parameter_policy=fixed_A、motion_limits.source=fixed_A、
gap_policy=hold_and_resume。waiting_for_input/hold_transition是临时保持续接状态，
queue_depth=0和state=3均不代表整个任务完成。
完成必须明确final_reference_reached=true且consumed等于本任务已发送点数；
end_received=true本身也不够。之后命令位置误差≤0.3°、估计速度≤0.5°/s持续40ms，
并检查实际编码器误差≤0.5°、速度≤0.5°/s持续40ms。
taj_stop的ACK仅表示请求，必须再等本任务active=false和GET motion=Stopped；主动停止reason=1正常。

## 参数归属与网络

生产固定A：ω25、速度前馈0.8、加速度前馈0、速度带宽3Hz；初始缓冲40ms、4ms控制。
每臂J1–J3速度135°/s、J4–J6为162°/s；加速度±225°/s²、jerk5000°/s³。
上游只发无参timed_aj，不调参、不从XML再算90%、不改全局倍率或电机加速度。
XML位置限位和SDK保护继续有效，推理不自动使能或回初始姿态。

现场用户于2026-11-25确认网络入口已配置。默认发送地址为`192.168.0.150:10000/UDP`，
`network_ready=True`记录该现场配置确认；原协议快照仍保留当时仅监听本机的描述。
本机经既有共享中继只读GET已收到Manual/Disabled/Normal/Stopped反馈。
未发送UDP运动目标，因此没有将用户的配置确认当作实测UDP接收或运动验证。
这个标记不会在控制器上创建监听或转发，也不会自动使能或开始运动。
现场用户随后已重启控制台；只读确认新共享中继的GET及timed_status均返回成功。
现场taj_status报告joint_tracking.TimedAj、fixed_A、hold_and_resume，完整状态字段校验通过；
任务inactive、stream_id=0。机器人Manual/Disabled/Normal/Stopped，未自动使能或启动Plan。
网络就绪配置的24项定向测试通过；上述查询仍不证明UDP目标实际接收。

## 离线验证

本轮完整回归174项通过，git diff --check通过。
测试使用内存设备、虚拟时钟和回环WebSocket/UDP，不证明实机运动或跨机器网口可达。
覆盖原有121项推理测试，以及无参启动、控制器分配非连续编号、自动点号、临时保持续发、
首点长等待、块间7秒推理空档、final专用完成标志、故障/取消/所有权和停止后缓存清理。
真实模型已选20点与构造1000点分别验证单发；1000点33.333秒为虚拟时钟名义时轴，
不是实机回放，也不是模型真实连续输出。有暂停的用例单独保留墙钟空档。

```bash
PYTHONPATH=.:test:test/test_runners HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
python -m unittest -q test_a10_timed_aj test_a10_shared_control \
  test_a10_follower_tail test_a10_framework_loop test_a10_dual_inference \
  test_a10_longwam test_a10_inference_runner
```

本次仅修改上游协议和控制流程，不据此声称下游跟踪误差、jerk或机械振动已改善。
