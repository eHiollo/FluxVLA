# A10 推理时序诊断

## 先画已有日志

在 FluxVLA 目录、`fluxvla` Python 环境下执行。动作块编号从 1 开始。

```bash
python scripts/a10_inference_timing.py \
  work_dirs/a10/inference_logs/20261123_171128_5b70c81a_dual_client \
  --cycle 85 --count 4 --ee-id 0 --joint 4 \
  --output work_dirs/a10/inference_timing/20261123_171128
```

`detail.png` 是带中文图例的放大图，横轴以第一个实际 UDP 包为 0 ms。默认放大前 6 个点，关节曲线最多看首包后 1.5 秒，并在下一块开始执行前截断。可用 `--detail-points 4` 减少发包图内的点，用 `--detail-ms 800` 进一步缩短关节曲线窗口。

| 放大图 | 图例含义 |
|---|---|
| 上面，逐点发包 | 灰竖线是计划首发，深蓝点是实际首发，浅蓝点是同一个目标重发一次；点旁的数字是实际首发相对计划的延迟。深橙叉和浅橙叉分别是控制器网口收到首发和重发，随抓包导入显示。 |
| 中间，关节角度 | 蓝阶梯是 FluxVLA 实发的该关节目标，紫线是控制器目标，绿线是实测关节角。橙虚线是末包发出，浅橙底是之后的等待。绿点线表示停前到位，棕实线表示收到停止回复，只在事件位于放大窗口内时显示。 |
| 下面，角度差 | 紫线是控制器目标减去当时最近一次发送的上位机目标。绿线是实测关节角减去同一反馈中的控制器目标。灰色横线是零误差。旧日志使用 GET 回包时间，比较的是 GET 采样时的差距。 |

`timeline.png` 用来回看整轮。上半图中 `FluxVLA schedule` 是计划时刻，`UDP send 1/2` 是首发和重发，`Controller UDP ingress 1/2` 是网口收包。`Internal target near waypoint` 和 `Actual position near waypoint` 分别标内部目标和实际关节经过轨迹点附近的采样时刻。`Response read and decoded` 是旧日志里整块回复读完且解码完成的时刻，新日志会标 `Complete response read` 和 `OpenPI response pack start`。下半图的三条角度线与放大图含义相同。蓝色阴影 `Wait for OpenPI` 是等待模型回复，橙色阴影 `Tail wait / stop` 是末包到停止回复，绿点线 `Pre-stop arrival` 是停前到位，棕线 `Follower stop reply` 是停止回复。`points.csv` 的时间原点与总览相同，是发起模型请求的时刻。

OpenPI 一次发送整块动作。新版日志额外记录 OpenPI 打包起点、完整 WebSocket 回复到达 FluxVLA 的时间和服务端推理耗时。打包起点只在 OpenPI 和 FluxVLA 同机时画到同一时间轴上。加载该时间点需要重启本地 OpenPI 服务和推理客户端。旧的 `inference_end` 相当于读完并解码后的时刻。

## 实机同时录三路数据

1. 在 Thor 的 FluxVLA 目录测一次时钟偏差。实机测试结束后用同一命令再测一次。

   ```bash
   python scripts/a10_capture_inference_timing.py clock kaanh@192.168.0.150
   ```

2. 在控制器 `.150` 上开启短时入站 UDP 抓包，保持这个终端运行。

   ```bash
   sudo tcpdump -i any -s 0 -w /tmp/a10_inference_ingress.pcap \
     'udp dst port 9999 and dst host 192.168.0.150'
   ```

3. 在 Thor 的第二个终端启动高频记录。使用正在运行的 A10 共享中继。终端会打印本次 `.rlog` 相对路径。等抓包和高频记录都开始后，再从推理终端启动现场推理。推理结束后分别关闭抓包和高频记录，等约 5 秒让控制器完成刷盘。

   ```bash
   python scripts/a10_capture_inference_timing.py record
   ```

4. 在控制器项目目录导出刚才的会话，只选同一会话的分卷。下方用 `RECORD_FILE` 表示第二个终端打印的路径。

   ```bash
   cd /home/kaanh/Desktop/kaanh-project
   sudo python3 runtime_recorder/log_tool.py inspect "$RECORD_FILE" > /tmp/a10_rlog_inspect.json
   sudo python3 runtime_recorder/log_tool.py export "$RECORD_FILE" -o /tmp/a10_rlog.csv
   ```

   把抓包、CSV 和 inspect JSON 放到 Thor。原始高频日志由 root 创建，导出用 sudo。

5. 在 Thor 合并同一时间段内的三路记录。把 `RUN_DIR`、`OFFSET_MS` 和 `CLOCK_ERROR_MS` 换成实验值。时钟偏差取测试前后测量的中间值，误差范围取较大的半 RTT 再加偏差变化量的一半。12 个轴的顺序是 ID 1 J1–J6、ID 0 J1–J6，下面的编号对应已核对的 `.150` 映射。

   ```bash
   python scripts/a10_inference_timing.py "$RUN_DIR" \
     --cycle 1 --count 4 --ee-id 0 --joint 4 \
     --udp-pcap /tmp/a10_inference_ingress.pcap \
     --rlog-csv /tmp/a10_rlog.csv --rlog-inspect /tmp/a10_rlog_inspect.json \
     --rlog-axes 6 7 8 9 10 11 0 1 2 3 4 5 \
     --controller-offset-ms "$OFFSET_MS" --clock-error-ms "$CLOCK_ERROR_MS" \
     --output work_dirs/a10/inference_timing/onsite
   ```

## 读图时的几个时间点

- **UDP 收到**指包到达控制器网口。默认按关节目标、发送顺序和 10 ms 的预计传输窗口匹配，可用 `--max-udp-transit-ms` 调整。空值表示这次抓包和匹配条件下没有足够证据，可能是未捕获、时钟误差或重复包无法唯一匹配。
- **内部目标或实机关节经过**指首个可归到该轨迹点、12 轴最大误差不超过容差的高频样本。默认容差是 0.5°。多个候选轨迹点同时合格会标记为 `overlap`，已处于邻近范围的点标记为 `near_at_start`。逐点经过时刻用于分析轨迹，末点停稳以推理日志中的停前到位和停止确认事件为准。
- **旧日志的反馈**使用 GET 收到的时刻绘制，用于看执行趋势。有高频数据时才输出逐点内部目标和实际关节经过的时间。跨机器的时延若小于时钟误差范围，只适合判断大致顺序。
