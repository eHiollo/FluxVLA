# 双臂 joint follower 排查记录（已恢复）

2026-11-06：设置 `follower_aj --vel_percent=100` 后，用户先确认能运动，随后进一步确认全部正常，运动与停止均已恢复。保留当前配置。以下为此前故障及接口核对记录，SDK 内部原因尚未确认。

## 调用链路

1. WebSocket 连接 `192.168.0.150:5999`，用 `get` 查询状态。
2. 已使能、Stopped 时，发送原生命令 `follower_aj`。此前获取的控制器 XML 将其绑定到 `kaanh.FollowerAjAbsolute2`。
3. 确认 `follower_msg.running_state=true`、`follower_mode=Axis` 后，向 **UDP 9999** 发送关节目标。
4. 当前代码默认 **30 Hz 重复发送最新目标，即使目标不变也重复发送**；不生成中间插值点。约每 100 ms 查询一次状态，查询会与发送串行，实际间隔可能受 I/O 影响，慢时不突发补包。

控制器配置：关节 `tg_id=1`、`is_cart=false`、`ee_ids={0,1,2,3,4,5,6,7,8,9,10,11}`、`model_ids={0,1}`；`FollowerAjParam tg_id=1` 监听 UDP 9999。

## 报文样例

目标为绝对关节角度，单位**度**，我们按 `right J1～J6`、`left J1～J6` 顺序发送十二个数：

```text
jp --motor_pos=[21,38,-131,28,111,-3,-18,-36,128,-18,-106,-5]
```

UDP 包不是裸文本，使用 40 字节小端 ARIS 消息头。当前打包代码等价于：

```python
payload = b'jp --motor_pos=[21,38,-131,28,111,-3,-18,-36,128,-18,-106,-5]'
packet = struct.pack('<IIQqqq', len(payload), 0, 0, 0, 0, 0) + payload
# 每个发送周期：joint_udp.send(packet)，目标不变也继续发送。
```

WebSocket 命令也用二进制帧，头为 `struct.pack('<IIQqqq', len(payload), 1, 0x1000, 0xA1B2C3D4, 0, 0)`，后接 UTF-8 命令文本。`jp` 只发 UDP；之前误经 WebSocket 发送时返回过 `invalid command name: jp`。

## 停止异常：现场已复现

本地停止发目标后，逐条通过 WebSocket 发送：

```text
stop_follower
manual_stop
get
```

两条停止命令都返回 `{"ret_code":0,"ret_context":"","ret_msg":""}`，但随后 `get` 持续反馈：

```text
robot_msg:    activate=Enabled, op_mode=Manual, motion=Running
follower_msg: running_state=false, follower_mode=Axis
故障：无；手动速度倍率：10%；jog_coordinate=Tool
```

实际关节位置基本没有变化。重启上位机控制台后查询仍是 Running。控制器日志曾持续打印 `RT0 4---execute cmd in count: 192000 / 193000 / …`。此前 `about --version` 读到运行库为 `v2.5.10.3`。

早期上位机曾只发一帧，现已改成持续发送；持续发送修改尚未完成实机验收，不把旧故障归因于已确认的控制器缺陷。

2026-11-06 后续试验配置：按用户要求将启动命令改为 `follower_aj --vel_percent=100`；
只改 follower 参数，未修改控制器手动速度。随后用户现场反馈“现在好了”，确认加上该参数后能运动。
上面的故障记录来自此前参数；用户随后确认运动与停止均正常，具体 SDK 内部原因尚未确认。

## 当时向接口开发方核对的问题

- `follower_aj` 是否需要配合 `start_follower`，正确启动顺序是什么？
- 上述十二轴排列、单位、ARIS 包头和 30 Hz 连续发送方式是否正确？
- 为什么 `stop_follower`、`manual_stop` 均返回成功，但 follower 关闭后 motion 一直 Running？应如何正确结束当前 joint follower 计划？
