# A10 现场接入操作说明

当前平台已通过离线流程检查。机械臂及 OpenPI 仍由其他同事开发，现场验证按用户要求延期。以下步骤供设备空闲、有人在场配合时执行。

## 1. 环境和站点配置

```bash
source /home/robot/miniforge3/etc/profile.d/conda.sh
conda activate fluxvla-thor
cd /home/robot/VLA/FluxVLA
git branch --show-current
python -m fluxvla.projects.a10 doctor
cp -n configs/a10/site.example.yaml configs/a10/site.local.yaml
```

分支应为 `kaanh`。`cp -n` 保留已有站点文件；编辑 `site.local.yaml` 后会自动加载，它已被 Git 忽略。模板中的相机名是占位符，地址是回环地址，须按现场修改。

| 字段 | 应填写什么 |
| --- | --- |
| `cameras.base_0_rgb.device` | 外部视角相机的设备路径 |
| `cameras.right_wrist_0_rgb.device` | 腕部相机的设备路径，不能和 base 指向同一节点 |
| `rotate_180` | 默认 base=false、right_wrist=true，现场看图确认 |
| `robot.host` / `port` | 运行 A10_new 控制服务的机器地址及端口，与控制器同事确认 |
| `vr_server.host_ip` | 运行 FluxVLA 的机器局域网地址，用 `ip -brief address` 查看 |
| `robot.allow_motion` | 准备阶段保持 `false` |
| `max_joint_step_rad` / `gripper_mm_range` | 现场确认的单步关节上限和夹爪毫米范围，准备阶段可保持 null |

控制器地址与 FluxVLA 地址可能不同。`doctor` 检查软件和文件，不连接机器人或打开相机；它成功时仍可能显示相机“待连接”。要使用其他站点文件，将 `--site 文件.yaml` 放在子命令前。

## 2. 检查相机并核对画面

```bash
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
python -m fluxvla.projects.a10 check-hardware --cameras-only
python -m fluxvla.projects.a10 check-hardware --cameras-only --frames 60 --execute
```

优先使用 by-id 稳定路径；没有唯一序列号的同型号相机可用 by-path，并保持 USB 接口不变。一个相机可能有多个节点，需选能输出图像的节点。没有 `--execute` 时仅预览命令。

终端打印报告位置 `work_dirs/a10/hardware_checks/<时间>/report.json`。成功时同目录有两张 PNG，已应用配置中的旋转；人工核对两路视角、上下方向、清晰度和遮挡。

`paired_capture_fps` 是两路连续读取吞吐，排除了打开／预热，不包含机器人、VR 和录像写盘耗时，也不能证明传感器没有重复画面。`meets_90_percent_target=false` 提示低于目标的 90%，最终须检查真实录制频率。

设备缺失、两个角色指向同一节点、尺寸异常或驱动超时会失败。默认每项检查最长 60 秒；检查设备路径和占用情况后重跑，不关闭同事正在使用的进程。

## 3. 查询 A10 状态

与控制器同事协调好后执行：

```bash
python -m fluxvla.projects.a10 check-hardware --robot-only --execute
```

此命令只发送 GET 查询：空闲状态、7 维反馈、再次检查空闲状态。控制器忙时会停止检查。前六个反馈应为关节弧度，最后一个应为夹爪毫米值，需现场核对实际姿态、顺序及单位。

自动通过只证明协议与有限数值，不能确认反馈新鲜度或物理限值。平台互斥锁只约束本机通过统一入口启动的任务，无法协调其他电脑／仓库的客户端。`check-hardware --execute` 默认检查相机和机器人；控制器开发期间使用 `--cameras-only`。

## 4. 验证 VR 输入

```bash
python -m fluxvla.projects.a10 teleop --vr-only --execute
```

此模式不连接机器人。头显打开终端打印的 `https://本机局域网地址:8443` 并进入 VR。首次启动会在 `work_dirs/a10/vr/tls/` 生成本机证书，浏览器可能需要信任证书；手柄输入使用 WebSocket 端口 8442。页面能打开但没有输入时，检查 WebSocket、证书和终端诊断。网页还需要从 CDN 加载 A-Frame。

现场核对手柄跟踪、使能／松开、重锚和掉帧保护。结束时按 Ctrl+C。实际动臂验证待控制器团队方便配合时进行。

## 5. 录制一小段真实数据

这一步会发送控制命令。现场核对设备和控制接口后，填写已确认的 `max_joint_step_rad`、`gripper_mm_range`，再将 `allow_motion` 设为 `true`。配置允许的范围为关节步长 `(0, 1]` 弧度、夹爪上下界在 `0–100` 毫米内，实际值须按机器确认。

这些关节限值用于模型批次检查；VR 发的是末端增量，依赖已有 VR 输入保护和 A10_new 控制。因此关节限值不代替 VR 的现场验收。

```bash
# 先预览，确认现场条件后再加 --execute
python -m fluxvla.projects.a10 record --root datasets/a10/onsite_trial_001 --task pick_and_place --episodes 1
python -m fluxvla.projects.a10 record --root datasets/a10/onsite_trial_001 --task pick_and_place --episodes 1 --execute
python -m fluxvla.projects.a10 validate --root datasets/a10/onsite_trial_001 --execute
```

终端 Enter/n 或左摇杆向右：开始／保存；r 或左摇杆向左：放弃本集重录；s 或左摇杆向下：结束。至少录两帧；已有数据集需显式 `--resume` 才续录。控制器已接受的动作可能继续执行，软件退出不能替代现场停机手段。

## 6. 留下验收记录

在 [开发记录](a10_full_migration_plan.md) 追加 commit、站点配置摘要、设备对应关系、报告位置、实际帧率、现场核对结果与待办。通过后再确定正式训练的数据／任务划分、时长和评估方法。

目前模型只训练两步，已出现夹爪越界预测，只证明软件流程连通，不能用来直接控制机器人。正式训练与实机效果均尚未验收。
