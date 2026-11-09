# 双臂数据格式与训练端交接说明

本阶段已在 FluxVLA 实现双臂数据保存、校验和 LeRobot v2.1 导出。**现已接入双臂实时VR采集入口，见[采集指南](a10_vr_recording.md)，实机尚待验收；没有修改 OpenPI policy 或验证训练。** 本机的标准 LeRobot 读取器验证只是确认文件可读，不能代替训练端适配。

## 1. 和原单臂保持一致的部分

仍使用 LeRobot 的标准字段、30Hz、按集保存、Parquet 数值、RGB MP4、任务表和每集统计。原始录制为 v3，导出为 v2.1。旧单臂默认接口及元数据保持兼容；双臂显式选择 `schema='a10_dual_v1'`，必须新建数据集，不能续写到旧单臂数据里。

```text
数据集/
  meta/info.json                       格式、特征名称、维度、视频信息
  meta/tasks.jsonl                     任务编号和文字
  meta/episodes.jsonl                  集号、长度、任务
  meta/episodes_stats.jsonl            每集统计
  meta/stats.json                      汇总统计（不是OpenPI norm_stats）
  meta/source_recording_contract.json  字段单位、左右/ID对应、设备身份、动作语义
  meta/export_manifest.json           导出来源、图片改名和动作来源
  meta/xlevr_diagnostics/              可选帧诊断/采样接收时刻
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.top/episode_000000.mp4
  videos/chunk-000/observation.images.left/episode_000000.mp4
  videos/chunk-000/observation.images.right/episode_000000.mp4
```

模拟数据额外有 `SIMULATED_DATA.json`，不能用于任务训练或续录实机数据。导出拒绝覆盖目标目录，不修改源数据；视频重新编码为 H.264，保留源图像统计供参考，不保证逐像素相同。

## 2. 14维状态：按物理左右，不按控制器ID排序

`observation.state` 为 float32，严格按下面的顺序：

| 下标（从0开始） | 字段名称 | 含义/单位 |
|---|---|---|
| 0～5 | `left.joint_1.pos` ～ `left.joint_6.pos` | 物理左臂6轴实际关节角，rad；控制器ID1 |
| 6 | `left.gripper.closure` | 左夹爪实际闭合比例，0～1 |
| 7～12 | `right.joint_1.pos` ～ `right.joint_6.pos` | 物理右臂6轴实际关节角，rad；控制器ID0 |
| 13 | `right.gripper.closure` | 右夹爪实际闭合比例，0～1 |

夹爪 `closure = actual_raw / 9000`，0完全张开、1完全闭合，不是毫米。范围超出或反馈缺失会报错，不静默裁剪或补零。`pack_dual_state()` 接受以整数 ee_id 为键的实际关节反馈，以及以物理 left/right 为键的夹爪反馈，避免误用 operator 内部相反的 left/right 别名。该函数不访问设备，调用方负责保证传入的是实测值和新鲜反馈。

设备身份继承自 `configs/a10/a10_devices.py`；录制约定保存序列号、夹爪Modbus ID、图片尺寸及旋转设置，不保存USB设备路径。换USB口不改变数据格式；更换实际设备或旋转设置后，应新建数据集，续录会检查约定是否一致。

## 3. action 的含义务必告知训练端

双臂 `action` 也是14维 float32，**等于同帧实测 `observation.state` 的副本**。元数据记录：

```text
action_source = same_frame_measured_state_copy
action_time_offset_frames = 0
training_source = observation.state
```

它是“实测轨迹代理动作”，不是手柄指令、不是UDP目标、不是下一帧目标，也不是关节增量。写入器和校验器都会拒绝 state/action 不相等的双臂帧。控制输入和下发目标如需追溯，放在诊断记录里。

旧单臂 `action` 是8维旧遥操作量，夹爪状态单位是mm；新双臂不会冒充旧语义，也不能直接交给旧7维policy。推荐训练设备沿用“从 observation.state 读取未来状态窗口”的方案：两个臂的12个关节减去当前关节角，两只夹爪保留绝对闭合比例。这一转换本阶段不执行。

## 4. 图片、任务与时间

| 物理位置 | v3原始字段后缀 | v2.1导出字段 |
|---|---|---|
| 外部场景 | `base_0_rgb` | `observation.images.top` |
| 左腕 | `left_wrist_0_rgb` | `observation.images.left` |
| 右腕 | `right_wrist_0_rgb` | `observation.images.right` |

默认640×480，输入是 H×W×3、uint8、RGB。右腕的180°旋转由采集层执行一次，写入器不做旋转/缩放，仅保存调用方提供的画面；配置登记旋转用于追溯。测试读取器返回的是 C×H×W、0～1浮点图像。

`timestamp=frame_index/30`，每集从0开始；frame_index/episode_index/index/task_index 均为int64。任务文字保存在任务表，标准读取器解析为 `task`。时间戳是名义采样网格，不代表硬件同步；可选单调接收时刻放在诊断文件，实时采样时效/相机对齐是下一阶段工作。

## 5. 如何使用现有保存和导出工具

开发接入示意（不是可以直接启动机器人采集的命令）：

```python
from mmengine import Config
from fluxvla.datasets.a10_recording_schema import pack_dual_state
from fluxvla.datasets.episode_writer import EpisodeWriter

cfg = Config.fromfile('configs/a10/a10_dual_arm_record.py')
with EpisodeWriter(**cfg.record) as writer:
    writer.start('Pick up the water bottle')
    # 由后续采集循环提供新鲜的实测 joints_by_ee_id、gripper_raw_by_side、images：
    state = pack_dual_state(joints_by_ee_id, gripper_raw_by_side)
    writer.append(state, state.copy(), images,
                  capture_monotonic_s=capture_time, diagnostics=details)
    # 重复append至少两帧后，调用save；重录则discard。
    writer.save()
```

真实数据目录产生且录制器关闭后，在 `/home/robot/VLA/FluxVLA` 的 fluxvla 环境运行：

```bash
python scripts/validate_a10_dataset.py datasets/a10_dual_record
python scripts/export_a10_openpi_dataset.py \
  --source datasets/a10_dual_record \
  --output datasets/a10_dual_record_openpi
python scripts/validate_a10_dataset.py datasets/a10_dual_record_openpi
```

这些命令不会采集设备或创建实机样本。`--record-controller` 依然只用于控制器诊断，不生成此数据集。保存失败/队列满使本集失效；录制器本身没有机械臂停止权限。

## 6. 训练设备需要适配的地方

- 增加独立的双臂policy/data config，输入和有效输出都是14维；不要修改旧单臂配置的含义。
- 接入 top/left/right 三路图片；左右图像掩码都有效，不再用零图代替左腕。
- 若沿用关节增量训练，掩码为“6个关节、1个绝对夹爪、6个关节、1个绝对夹爪”。
- 明确动作窗口来源和时间偏移：数据未提前/移位；由训练端选择窗口长度。已验证标准读取器30步窗口在集尾重复末帧，不跨集。
- 将任务文字转发到模型的 prompt；不能仅有任务表而在 Repack 时丢掉它。
- 重新计算该数据集的归一化统计；不能使用旧单臂数据或本机模拟数据的统计。
- 单臂 checkpoint 不等于经过双臂适配和训练的模型。

## 7. 本阶段验收（2026-11-06）

- 18项离线测试通过：旧单臂11项、双臂7项；覆盖保存、丢弃、续录、未完成集/提交恢复、数据约定、非法帧、左右ID、三路视频及导出源数据不变。
- 双臂模拟样本：两集各40帧、三路16×16 RGB，共80帧，明确模拟标记，位于临时目录。
- 旧真实单臂 `a10_vr_9_21_4` 的35集、37225帧通过数值/元数据校验（未全量解码视频）；终端校验与导出入口通过双臂模拟验证。
- 标准 LeRobot v3 与 v2.1 读取器均通过；检查集首、中间、集末的14维数值、任务、三路画面及30步窗口集边界。
- 只复用本机已有Python环境中的LeRobot依赖，没有修改OpenPI仓库、导入OpenPI policy、运行模型或训练，没有连接设备。
- 未验证640×480持续写入吞吐、实时同步、现场完整任务和训练效果。下一阶段接实时采集循环。

离线回归：

```bash
python -m unittest discover -s test/test_datasets -p 'test_a10*recording.py' -v
```

标准读取器复验入口为 `test/test_datasets/check_a10_dual_reader_compatibility.py`，仅用于 `test_a10_dual_recording.make_dual_fixture()` 生成的2×40帧模拟样本：在相应LeRobot环境运行 `--reader v3 --dataset 原始目录` 或 `--reader v21 --dataset 导出目录`。它不加载模型。
