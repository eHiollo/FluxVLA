# A10 录制核心：保存数据，再交给 OpenPI 训练

> 2026-11-06更新：已新增双臂14维/三路相机的独立格式、保存、校验和导出，见[双臂数据交接说明](a10_dual_dataset_handoff.md)。下文记录旧单臂格式及其既有验证；双臂实时VR入口随后已接入，见[采集指南](a10_vr_recording.md)，实机待验收；旧单臂实时采集仍未接入。OpenPI双臂policy留给训练设备适配。

本轮完成的是“保存一集数据”的底座。它能接收已经准备好的关节、夹爪、VR动作和两路图片，保存成旧数据集的结构，再导出给当前 OpenPI 使用。

**目前还没有把它接到实时 VR 采集循环里。** 现有 `teleoperate.py` 的运行方式不变，`--record-controller` 仍只是控制器诊断记录，不会生成训练数据。双臂采集、现场图像同步、手柄开始/重录按钮将在下一阶段接入。

## 1. 数据保持什么不变

对照来源：`lerobot/dataset/a10_vr_9_21_4`、旧 `projects/a10/recording.py`、OpenPI 的 `LeRobotA10DataConfig`。

| 每帧内容 | 保存格式 | 含义 |
|---|---|---|
| `observation.state` | float32，7维 | 六个实际关节角（rad）+ 实际夹爪开度（mm） |
| `action` | float32，8维 | 旧遥操作的 enabled、xyz增量、旋转增量、归一化夹爪动作 |
| `observation.images.base_0_rgb` | RGB视频 | 场景相机 |
| `observation.images.right_wrist_0_rgb` | RGB视频 | 手腕相机 |
| `timestamp` | float32 | 本集帧号 / 30，单位秒 |
| `frame_index`、`episode_index`、`index`、`task_index` | int64 | 集内帧号、集号、全局帧号、任务号 |
| 任务描述 | 任务表 + 每帧任务号 | 例如 `Pick up the water bottle` |

固定30Hz，默认两路640×480。图片必须是 RGB uint8；改变分辨率要新建数据集，不能在已有数据集里混用。

**`action` 的8维不能直接当作模型的7维动作训练，也不是发给 UDP follower 的位姿报文。** 当前 OpenPI A10 配置使用 `observation.state` 构造30步动作窗口，六个关节转换为相对当前关节的增量，夹爪仍为绝对开度。原始8维 `action` 原样保留供追溯。

录制核心不读取机器人，也不把控制目标冒充实测状态。缺少夹爪反馈、任一相机或合法数值时，调用方不能补零蒙混过去。双臂14维状态会被拒绝，不会偷偷截成单臂。

## 2. 文件放在哪里

| 文件 | 用途 |
|---|---|
| `fluxvla/datasets/episode_writer.py` | 开始一集、逐帧写入、保存、丢弃、续录与恢复 |
| `fluxvla/datasets/a10_recording_io.py` | 离线校验、v3到v2.1格式转换 |
| `configs/a10/a10_single_record.py` | 输出位置、分辨率、30Hz、队列大小等配置 |
| `scripts/validate_a10_dataset.py` | 终端检查数据集 |
| `scripts/export_a10_openpi_dataset.py` | 终端导出 OpenPI 数据集 |
| `test/test_datasets/test_a10_recording.py` | 本地模拟数据测试 |
| `test/test_datasets/check_a10_reader_compatibility.py` | 在实际 LeRobot/OpenPI 环境中检查兼容性 |

新核心放在框架原有的 `datasets` 下，不导入 `projects/a10`。原先的旧模块暂时保留，避免影响旧调用入口；新开发使用新核心。

原始输出是 LeRobot v3：

```text
a10_record/
  meta/info.json                 数据格式、总集数、帧数
  meta/tasks.parquet             任务文字及编号
  meta/stats.json                全局统计
  meta/episodes/chunk-000/       每集索引、统计、视频位置
  meta/recording_contract.json   单位、相机、模拟/实机约定
  meta/xlevr_diagnostics/        可选诊断内容、原始采样时刻
  data/chunk-000/                每帧数值，Parquet格式
  videos/observation.images.*/   两路MP4
  .recording/                    未完成集、恢复归档（不参与训练）
```

模拟数据额外有 `SIMULATED_DATA.json` 标记。模拟与实机数据不能续录到同一数据集。视频使用 H.264，旧样本是 AV1；字段、单位、帧率保持一致，编码器不同且如实写入元数据，不保证像素逐位相同。

## 3. 开发者如何接入

下面是调用顺序说明，`state7/action8/images` 需要由后续采集程序提供，不能直接把这段当作实机启动命令。

```python
from mmengine import Config
from fluxvla.datasets.episode_writer import EpisodeWriter

cfg = Config.fromfile('configs/a10/a10_single_record.py')
with EpisodeWriter(**cfg.record) as writer:
    writer.start('Pick up the water bottle')
    # 在采集循环中，每次提供一组完整数据：
    writer.append(state7, action8, images,
                  capture_monotonic_s=capture_time,
                  diagnostics={'vr_sequence': sequence})
    # 一集至少两帧。循环结束、用户决定保留时：
    writer.save()
    # 如果用户决定重录，改为 writer.discard()，再 start(...)。
```

`images` 的键是 `base_0_rgb`、`right_wrist_0_rgb`，值是 H×W×3 的RGB数组。

- `append()` 复制当前数据后放入有界后台队列，视频编码不在控制线程执行。
- 队列满、非法输入或编码失败会使本集失效，`save()` 不会保存缺帧的半成品；调用方必须捕获异常并退出录制。录制器本身没有停止机械臂的权限或接口。
- `save()` 会等待编码完成并校验视频，需要在退出录制循环后调用。每集提交完成才更新总帧数。
- `discard()` 只删除本次未保存集，不删已有集。至少两帧、默认单集上限18000帧（10分钟）。
- 保存时写临时目录和校验清单；正常退出但未保存的内容也会保留，便于排查。
- `resume=True` 续录仅支持本录制器创建的数据，要求相机、单位、fps和模拟标记一致；旧 LeRobot 采集的数据只做读取/导出，不直接续写。
- 出现未完成集时默认拒绝续录。检查后设置 `resume=True, recover=True`：未开始提交的集归档到 `.recording/recovered-*`；已开始提交的集按校验清单继续提交。校验不符会停止恢复，不覆盖已有数据。恢复归档占用空间，需要在确认不再需要后人工清理。

**时间对齐的职责**：写入器按旧格式生成 `frame_index/30`；原始单调时钟写进诊断文件。它检查提供的采样时间单调递增，但不负责相机/机器人同步，也不能识别调用方传入的旧画面。下一阶段采集循环必须按30Hz取完整的新鲜样本，检查反馈/图像时间差和采样中断；不能把低频数据贴上30Hz标签就宣称同步成功。

## 4. 终端怎么检查和导出

在 `/home/robot/VLA/FluxVLA`、`fluxvla` 环境运行。依赖已列入 `requirements-a10.txt`，不需要在 FluxVLA 环境安装另一套 LeRobot。

检查已有旧数据的数值、索引和元数据（不解码视频）：

```bash
python scripts/validate_a10_dataset.py \
  /home/robot/VLA/lerobot/dataset/a10_vr_9_21_4 --skip-videos
```

录制程序关闭后，完整检查新数据：

```bash
python scripts/validate_a10_dataset.py work_dirs/a10/datasets/a10_record
```

导出到一个**尚不存在**的目录：

```bash
python scripts/export_a10_openpi_dataset.py \
  --source work_dirs/a10/datasets/a10_record \
  --output work_dirs/a10/datasets/a10_record_openpi
```

只有实际创建了 `a10_record` 数据集后才能运行以上新数据命令；本轮不会自动创建或采集实机数据。

导出会完整检查源数据、按每集拆分数值和视频，再检查输出。旧v3视频含多集时按元数据时间段截取，每集时间从0开始，不能直接复制整个视频。导出视频统一重编码为 H.264，原图像统计保留供参考；原始数据不变，目标已存在则拒绝覆盖。数据多时处理较慢，属于离线操作。

输出 v2.1 用每集 Parquet/MP4，任务和每集统计改为 JSONL；相机名称映射如下：

```text
base_0_rgb        → observation.images.top
right_wrist_0_rgb → observation.images.right
```

这正是本机 OpenPI 当前 A10 配置读取的名字。模拟标记和诊断文件也一并保留。

## 5. 交给 OpenPI 时注意什么

训练仓库不需要迁移。本轮不修改 OpenPI 配置、权重或已有训练数据。

1. 将训练配置的 `repo_id` 指向导出的数据，并让 `HF_LEROBOT_HOME/repo_id` 对应这个目录。
2. 继续使用 A10 的 `use_state_as_action_targets=True`、30步窗口和原来的关节增量转换。
3. 在 OpenPI 重新计算这份数据的 `norm_stats`，不要复用另一数据集的统计。写入器的 `meta/stats.json` 不等于 OpenPI 模型归一化文件。
4. 检查任务文字：当前 OpenPI A10 的 Repack 没有传递 `prompt`，模型仍使用 `default_prompt`。任务表已保留，但仅开启 `prompt_from_task=True` 不足以实现多任务训练；单任务先把 `default_prompt` 设成实际任务，多任务需要在 OpenPI 补上 prompt 转发。本轮只记录这个现有行为，不擅自更改训练仓库。

## 6. 本轮验证及边界（2026-11-05）

- 11项离线测试通过：保存/丢弃/续录、无效帧、队列满、编码失败、中断恢复、提交校验、任务与视频对齐、合并视频按集拆分、AV1原视频兼容。
- 旧 `a10_vr_9_21_4` 的35集、37225帧通过数值/元数据校验，本轮未对整个旧数据集解码全部视频。
- 两集各40帧、16×16双相机的模拟数据，通过本机真实 LeRobot v3 reader、OpenPI 环境 v2.1 reader读取，检查集首/集末、相机区分、30步窗口不跨集。
- 通过 OpenPI 原 A10 数据变换、10批归一化统计计算，以及图片缩放、文本编码和动作补齐到30×32的模型输入流程。没有启动模型训练或机器人。
- 测试数据仅在 `/tmp`，带模拟标记，不是可用于任务训练的样本。未做640×480持续采集吞吐、设备同步及现场录制验证。

离线回归命令：

```bash
python -m unittest discover -s test/test_datasets -p test_a10_recording.py -v
```

下一步是把这个核心接入相机、实际关节/夹爪反馈和 VR 录制按钮，增加采样时效检查，再做短集现场验收。双臂的字段与 OpenPI 训练约定需要单独确定。
