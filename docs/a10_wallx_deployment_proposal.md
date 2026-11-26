# A10 Wall-X 完整动作块部署方案

状态：完整动作块配置 `configs/a10/a10_wallx_deploy.py` 已完成，53 项测试通过。
用户已说明标定完成，并明确确认 `configs/a10/a10_wallx_forward_experiment.json` 参数，
按用户指令将该文件的 `verified` 设为 `true`，同时记录确认来源；数值没有改动。
原 `motion` 的标定检查保留；`trial` 仍是独立的小步诊断入口。代理不启动机器人。

用户要求取消 5 mm 试步裁切，使用完整模型动作。拟开放以下具体范围：

- 使用用户已明确确认的 `a10_wallx_forward_experiment.json`，通过原有 `motion` 检查，不加入绕过选项。
- 同步闭环，每轮新鲜三路图像与机器人/夹爪状态 → 推理 → 原始未来行 0–5 → 执行 → 到位 → 下一轮。
- 双臂执行原始绝对目标，不把目标缩放到 5 mm；夹爪使用模型预测，不再保持开度。
- 每次启动最多配置 100 轮，共最多 600 个双臂模型目标。由现场用户启动，可中断。
- 不使用试验的五轮/25 mm 累计限制，工作范围内连续执行。此实现为同步停等，不声明 RTC。

拟用的执行参数：

| 参数 | 值 |
| --- | --- |
| 模型行时间间隔 | 33.7 ms |
| 每轮选取未来行 | 6 |
| 控制发送上限 | 90 Hz |
| TCP 线速度上限 | 0.15 m/s |
| TCP 角速度上限 | 0.5 rad/s |
| 模型相邻抓取点位移/旋转检查 | 50 mm / 0.15 rad |
| 一块相对观测的位移/旋转检查 | 150 mm / 0.5 rad |
| 单次控制器 TCP 发送步长检查 | 5 mm / 0.04 rad |

此处 5 mm 是发送到控制器的插值间隔检查，不是模型目标裁切。自适应插值增加中间帧，
根据工具杆臂和速度上限延长大动作的执行时间，完整保留每个选中目标及其夹爪终值。
保留反馈新鲜度、云端坐标告警、桌面高度、抓取点间距、跟踪误差与故障停止检查。
仍没有全连杆碰撞模型。实际坐标与夹爪定义由用户指定的标定文件提供。

已经完成：自适应插值、末帧真实反馈/误差日志，以及保持原目标、速度/步长和异常清理测试。
完整块离线预演在 `work_dirs/wallx/a10_full_chunk_preview.json`，没有打开硬件或发送动作。

此前自动审批拒绝了对未标定映射绕过检查执行 100 轮的改动；该绕过选项没有加入代码。
用户随后明确说明已经完成标定，并确认了上述文件，验证检查继续生效。

## 运行步骤

先退出占用相机/夹爪的遥操作及其他推理进程，机器人处于 Enabled / Stopped。
在 A10 控制台执行 `set_jog_coordinate --tool`；保持控制台开启即可。
使用用户已确认的映射文件启动：

```bash
cd /home/robot/VLA/FluxVLA
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py \
  --config configs/a10/a10_wallx_deploy.py \
  --mode motion --cycles 100 \
  --contract configs/a10/a10_wallx_forward_experiment.json \
  --prompt "fold the black cloth using both arms"
```

不要加 `--trial-step`。程序每轮打印选中模型行、目标距离、预测夹爪闭合比例、发送帧数和到位误差。
每块实际到位并停止跟随后进入下一轮；此实现为同步闭环，尚未实现 RTC。
Ctrl+C 请求停止并清理，控制器故障、失联、超时或超范围会结束；程序退出不替代现场急停。
API key 自动读取 `.env.wallx`。日志在 `work_dirs/a10/wallx_inference/<时间>/events.jsonl`。

另有新增可选的 `configs/a10/a10_wallx_timed_ee.py`，接入新底层 EE 受限轨迹执行器：30 Hz 原始目标、普通块续接、末流 final、固定 tool1/wobj0。该后端与上面的逐块停止 follower 不同，完整启动要求和命令见 [timed_ee 部署](a10_timed_ee.md)。新接口没有替换旧 follower，也没有在本次工作中实机启动。
