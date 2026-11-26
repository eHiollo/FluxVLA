# Wall-X 相对动作模式

用于诊断模型绝对落点偏移：保留同一预测块内的位姿变化，从 A10 当前位姿执行。现有绝对动作配置保留，新配置是 `configs/a10/a10_wallx_timed_ee_delta.py`。

```bash
cd /home/robot/VLA/FluxVLA
work_dirs/wallx/venv/bin/python scripts/a10_wallx_inference.py \
  --config configs/a10/a10_wallx_timed_ee_delta.py \
  --mode motion --cycles 100 \
  --contract configs/a10/a10_wallx_forward_experiment.json \
  --prompt "fold the black cloth using both arms"
```

等效 CLI 选择为正常 timed_ee 配置加 `--ee-action-mode delta`；`--ee-action-mode absolute` 可恢复原模式。启动动作仍由操作员执行，本次开发没有启动实机运动。

## 计算定义

令同一块模型预测为 `(p[k], R[k])`，新鲜 A10 反馈经当前标定转换到模型 EE 的位姿为 `(p_robot, R_robot)`，目标为：

```text
p_target[k] = p_robot + translation_scale * (p[k] - p[0])
R_target[k] = Exp(rotation_scale * Log(R[k] * inverse(R[0]))) * R_robot
```

平移增量和旋转增量均在模型世界轴表达；姿态使用旋转矩阵/四元数，处理 ±π 包角，不直接相减欧拉角。运算在当前 JSON 定义的 model EE/tool0 上进行，之后经原变换转到抓取中心与各自基座的 SDK tool1/wobj0。295 mm 工具变换只按原标定转换一次。

`p[0], R[0]` 是服务器的第一帧**未来预测**。这里有意舍弃这帧的动作、把它替换为当前位姿保持；不是把公开协议第零帧误认为观测锚点。每轮仍发 6 个点，其中第一个是保持，其余 5 个保留预测内部变化；普通块不停流，全流仅最后一点 final=true。

默认 `delta_translation_scale=1`、`delta_rotation_scale=1`，没有隐含放大、缩小或逐轮 5 mm 截断。夹爪保持模型的绝对开合预测，未改成夹爪增量。观测仍使用 A10 实测数据；请求继续用 `normalization=provided`，不增加客户端归一化。

可加 `--execute-horizon 16` 来执行同一预测的前 16 帧（约 0.53 秒），再重新观测/推理；默认仍是 6 帧。此选项使用更长的原始预测，不放大动作。最近 17 份保存预测的离线回放，delta 抓取中心终点位移中位数由 6 帧时左/右 5.8/9.5 mm，变为 16 帧时 30.3/25.1 mm；其中 16 份满足原有目标边界，另 1 份仍会因范围超限拒绝。更长的执行段会减少观测更新频率，离线回放不代表实际任务效果。

## 适用范围

这是一种相对轨迹执行方法，不是模型归一化参数修复。它可取消每块首帧的绝对平移/姿态落点偏差，也会同时丢弃首帧的有效任务动作。不能保证解决动作尺度错误、视觉与本体差异、工具语义或任务策略错误。固定预测会变成保持；若任务动作主要集中在每块首帧，此模式会削弱任务推进。

直接用 `prediction - observation` 再叠加同一机器人观测，在机器人未移动时等于原绝对预测，无法移除绝对落点偏差，因此这里选择的是预测首帧作为参考。完整输入与预测仍按原协议校验，目标点仍经过原执行边界检查。

最近实机日志 `20261126_112158_a1852c89` 实际使用旧 follower，17 轮预测后因跟踪偏差结束；不是 timed_ee 的实机结果。第 0 轮两臂第一目标相对输入偏移约 8.5 / 9.0 mm，而第 5 帧相对第 0 帧仅约 1.3 / 1.4 mm。用此模式可能明显减小运动，不能仅凭去掉绝对偏移就宣称归一化问题已解决。

日志新增 `action_reference`：保存原首帧、机器人起点、原首帧与输入偏差、内部终点变化及重设后的目标，原始 `prediction` 事件保留。可用于比较实际变化与模型输出；本地离线回放不证明真实电机运动效果。

官方协议仍定义绝对米/弧度输入与输出，归一化由服务端处理：[API 接口说明](https://x2robot.com/docs/quantax1pro/v1/model-development/wallx-api/api-usage-instruction)。
