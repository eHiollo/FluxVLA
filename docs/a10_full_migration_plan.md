# A10 整套 VLA 迁移开发记录

## 目标和边界

最终交付是一个独立的 FluxVLA 平台：VR、相机、录制、数据处理、训练、推理和开发记录均位于 FluxVLA 内，正常运行不依赖旁边的 LeRobot、XLeRobot 或 OpenPI 仓库及其 Python 环境。现有模块中经验证的必要实现按功能迁入，保留来源和许可证；平台中已有的数据读取、变换、统计、训练 runner、注册器及 ZMQ 推理服务优先复用。

A10_new 暂时是底层运动控制后端，通过协议适配器连接。未来新的 follower 接管逆运动学和运动规划，平台通过目标姿态／位姿接口发送四元数等目标；当前关节批次接口与未来姿态接口分别声明能力，禁止把旧关节动作直接当作四元数。录制与 VR 输入为 30 Hz，当前 A10 底层控制为 500 Hz。

OpenPI 的 `a10_vr_9_21_4_h30/90000` 权重仍在调试。本次先接通原生 FluxVLA 流程，旧权重转换和对照待 OpenPI 那边确认其实际训练配置后补入，不以目录名推断真实动作步数。

## 当前进度（2026-10-08）

当前处于“单平台业务内迁与离线流程验收”阶段：业务代码、共享配置、数据副本、训练和模型服务入口已经进入 FluxVLA。最新真实数据读取、原生录制器、本地 VR 服务、短训、保存、完整训练恢复和保存权重后的真实离线推理均已通过。本阶段已完成并按用户要求暂停，现场硬件验收与正式训练留到下一阶段。这里的完成仅指离线工程阶段，整套实机迁移仍待现场验收。

## 实施清单

| 模块 | 交付内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| 环境 | 固定 Thor 可用环境与仓库版本 | CUDA 算子和 CLI 能运行 | 已完成，见 `local_setup/thor/README.md` |
| 数据 | LeRobot v3 视频、帧、状态和未来动作读取 | 现有 35 集、37,225 帧校验 | 30 步验收通过，见下方记录 |
| 统一配置与入口 | A10 项目配置、检查与统一命令 | 命令预览和配置校验 | 已实现，见下方记录 |
| VR/录制内迁 | VR 网页、服务、输入保护和原生录制器迁入 FluxVLA | 移除外部仓库路径后可运行，录制输出被现有 reader 读取 | 本地服务、模拟录制／续录通过；待现场验收 |
| 相机内迁 | 平台内相机采集，训练／推理共用名称与旋转 | 无 A10_new Python 路径依赖，RGB 内容校验 | 已实现；待物理相机验收 |
| follower 接口 | 当前关节批次、EE 增量后端与上层流程解耦 | A10 legacy 后端继续工作 | 已实现注册式后端；新接口待团队确定 |
| 原生训练 | A10 7 维状态、未来关节状态目标、PI0.5 配置 | 样本变换、短训、恢复训练 | 1 步短训、完整恢复至第 2 步及两种格式保存均通过 |
| 离线回放 | 历史观测到模型输出，不发机器人命令 | 输出维度、有限值与日志，诊断范围 | 保存后的真实权重与真实数据回放通过；未验收动作质量 |
| 实机推理 | FluxVLA 模型服务接 A10 TCP 批量动作 | 现场看护下完成闭环 | 模拟闭环通过，待正式训练权重和现场 |
| 旧权重迁移 | 核实 OpenPI h30 配置、JAX 到 FluxVLA 映射 | 参数及固定输入数值对照 | 等 OpenPI 定版 |

## 配置规则

`configs/a10/project.yaml` 保存可共享的流程参数。站点私有值使用 `configs/a10/site.local.yaml` 覆盖，后者不提交。运行时保存解析后的配置及 FluxVLA 提交版本。设备名优先改成 `/dev/v4l/by-id/...`，避免 USB 插拔导致相机编号互换。录制相机定义为 `base_0_rgb` 和 `right_wrist_0_rgb`，模型训练目标从未来 `observation.state` 取得；录制的 8 维末端增量动作保留在数据中供诊断，不当作 PI0.5 的 7 维关节目标。

## 调整后的迁移顺序与验收

1. 抽象 follower 后端并迁入相机模块；现有 A10 TCP 继续承担关节批次和 EE 增量控制。未来 follower 的动作类型和实际协议等待团队确定后接入。
2. 将现有 VR 输入与保护算法、WebXR 页面及 HTTPS/WebSocket 服务迁入平台内部，移除动态导入其他仓库的逻辑；保留来源记录与许可证，不复制旧私钥。
3. 使用 PyArrow/PyAV 实现平台原生 LeRobot v3 格式录制器，复用 FluxVLA 的 v3 reader、视频路径处理、统计工具和数据变换；保留按集录制、重录、续录、任务标签和 VR 诊断。
4. 将统一命令改成调用平台内部模块；已有数据导入 FluxVLA 的 datasets 目录，训练产物进入平台的 work_dirs。以模拟输入录制短集，检查续录、重录及旧数据读取。
5. 接通真实基础模型的短训、离线回放、保存与恢复；现场连接当前 A10 后端验收整条流程。新 follower 开发完成后更换 backend，按新动作空间独立验证采集、训练和执行。

最终独立性验收：在临时工作目录中仅提供 FluxVLA 和运行数据，配置不含 LeRobot/XLeRobot/OpenPI 源码路径或解释器路径，VR、模拟录制、校验、训练配置和推理入口仍可运行。A10_new 仅作为当前机器人底层服务保留。已有外部命令包装属于过渡成果，不能作为单平台迁移完成的依据。用户已明确要求未来四元数／完整位姿及其协议现在暂不设计，由团队确定后修改 backend。

## 开发记录

### 2026-10-08：已完成的基础工作

- 配好 Thor 专用 `fluxvla-thor` 环境，测试 PyTorch CUDA、FlashAttention 及 FluxVLA 的三个 CUDA 扩展。此环境与 LeRobot、OpenPI 环境独立。
- 修复 FluxVLA 对 LeRobot v3 共享视频文件、每集时间偏移及 PyAV 解码的支持；原数据集未修改。
- 验证最新数据集全部数值记录，抽查所有 35 集的起点、中点、终点附近共 105 个样本及 210 张图像，结果为 PASS。该结果只证明读取，不代表训练或实机效果。

### 2026-10-08：迁移约定

- 用户确定统一入口、保留模块的结构；最终验收包含实机闭环。
- OpenPI h30 权重问题暂缓，后续仅需在确认配置后补充转换和对照，不阻塞当前流程开发。
- 本文每完成一个模块即补充：改动位置、运行命令、验证结果、未完成事项。

### 2026-10-08：用户调整最终架构

- 最新要求取代“统一入口、保留多个业务仓库”的旧约定：目标改为 FluxVLA 单平台，业务实现内迁，常规运行不再需要其他业务仓库。
- A10_new 的控制方法暂时保留为 legacy 后端；未来 follower 接收目标四元数／位姿并承担底层规划。未来完整位姿还是仅姿态，以及 wire 协议、坐标系和消息时序，待 follower 定义后确认；目前不虚构可用的四元数控制命令。
- 既有 7 维关节训练目标和旧数据继续可用；未来姿态目标是另一种动作空间，需要相应观测、数据和模型配置，不能通过改字段名完成转换。
- 后续每完成一个内迁模块，追加实际改动、依赖消除情况、测试结果与待办；下面此前的开发记录作为历史保留。

### 2026-10-08：统一配置、入口及录制命令接入

- 新增 `configs/a10/project.yaml`，集中描述仓库路径、机器人地址、两路相机、VR、数据、模型和推理参数；`site.local.yaml` 可覆盖站点值且已加入 `.gitignore`。
- 新增 `python -m fluxvla.projects.a10` 统一入口。`doctor` 只读检查环境；`teleop` 和 `record` 调用 LeRobot 自有 Python 环境，保留 XLeVR 的输入保护与诊断旁路；`validate` 调用已有的数据检查脚本。训练、服务、回放和实机子命令也已定义，待对应模块代码完成后验收。
- 默认展示即将执行的完整命令，`--execute` 才启动；每次实际启动保存解析配置、SHA256 和五个仓库的提交编号。录制已有数据集时必须显式使用 `--resume`，避免误覆盖。
- 验证：`doctor` 的路径检查和 LeRobot 依赖检查通过，`record --episodes 1`、`teleop --vr-only` 的命令预览成功。又用 LeRobot 自己的 Draccus 解析器解析实际录制参数，确认两路相机、30 Hz 和 `robot_controller` 模式正确。LeRobot 使用 `/home/robot/miniforge3/envs/lerobot/bin/python`；原 `.venv` 缺少运行依赖。尚未启动 VR 服务或实机录制，需现场确认相机实体设备与地址。
- VR 输入的缩放、死区、掉帧保护、坐标映射、夹爪摇杆和诊断参数已放入 FluxVLA 的统一 YAML。录制入口把这些参数逐项传给 LeRobot；遥操作示例入口新增 `--teleop-json` 接收同一份参数。用 LeRobot 解析器检查完整参数后，坐标映射、`pos_scale=0.9` 和 30 Hz 均与配置一致。

### 2026-10-08：A10 原生训练配置与统计

- 新增 `configs/a10/pi05_a10_finetune.py` 与 `fluxvla/projects/a10/training.py`。模型结构沿用 FluxVLA 的 PI0.5 实现，A10 参数从统一 YAML 读取。原生 FluxVLA 流程按用户确认的目标设为未来 30 步；尚未确认的 OpenPI h30 权重仍单独处理。
- 数据管道使用 `ParquetDatasetV3`，两路相机放到模型的 base、空 left、right 三个图像位置；训练动作取未来 `observation.state`，前六轴减去当前关节位置，夹爪维度原样保留。
- 给统计工具增加 `a10` 变换定义及 LeRobot v3 数据源支持。实际遍历 35 集后，得到 `a10.proprio` 和 `a10.action` 各 7 维统计量；模型动作窗口为 30，起点偏移为 1。训练入口会在工作目录保存统计量和解析配置。
- 已从 FluxVLA 官方固定提交 `04b5d94d6aebf23ce15b89443df08501740f2868` 开始下载 PI0.5 基础模型（模型文件约 14.5 GB），统一配置指向 `checkpoints/pi05_base/`。下载完成后再做短训。
- 用最新数据集跑过完整训练样本变换，输出 `states=(32,)`、`actions=(30,32)`、`images=(9,224,224)`、`img_masks=(3,)`、文本 token 长度 128。去掉训练专用的随机图像增强后，同一帧的训练和在线输入在状态、图像、相机掩码与文本 token 上完全相同。

### 2026-10-08：30 步数据验收、服务和控制适配

- 数据检查脚本增加可指定的 `--horizon`；统一入口自动取配置中的 30 步。运行 `python -m fluxvla.projects.a10 validate --execute` 后，`/home/robot/VLA/outputs/a10_fluxvla/validation/report.json` 为 PASS：35 集、37,225 帧，105 个跨集抽样，210 张图像，动作窗口为 `30×7`；13 个视频文件的帧校验通过，源数据文件未改变。
- 新增 `A10InferenceDataset`、`A10DenormalizeAction` 和 A10 专用模型服务。在线观测通过与训练一致的相机、状态归一化、提示词、token 和图像处理；预测结果反归一化，前六轴加回当前关节状态后成为绝对关节命令。
- 新增离线回放命令：从原始 LeRobot 视频和状态抽取观测，发送给 FluxVLA ZMQ 服务，保存每个样本的动作摘要；不打开机器人 TCP。真实模型回放需等原生训练权重。
- 新增 A10 TCP 适配器和实机桥接：取当前状态、读取相机、请求模型、检查 7 维动作与现场限值、发送 `SET_JOINTS_BATCH`、轮询 `GET_POLICY_STATUS` 至空闲，之后才启动下一轮。断线和超时停止发送新批次；已发出的批次是否继续执行由 A10 控制器决定。
- 使用假模型服务与假 TCP 控制器完成一轮完整闭环测试，确认收到且只收到一批动作；超出关节步长的动作会被拒绝。没有连接真实机器人，也没有验证物理限值。
- 用真实数据集的第 0、20,011、36,652 行和模拟 ZMQ 模型服务执行离线回放，三次都返回有限的 `30×7` 动作并生成 JSON 报告；测试过程未连接机器人。
- 实际运行 `teleop`、`record`、`infer` 时，统一入口加本机互斥锁防止三种模式在同一机器上同时控制 A10。桥接每轮的状态、首个动作、动作数和耗时记录在运行目录的 `cycles.jsonl`；运行清单还记录五个仓库的提交号和已修改的追踪文件。

## 日常操作入口

以下为当前平台内部入口；前文调用外部仓库的记录仅作为历史。路径改为平台内部 `datasets/a10/` 和 `work_dirs/a10/`，Python 使用当前 FluxVLA 环境。

先在终端激活已配置的环境：

```bash
source /home/robot/miniforge3/etc/profile.d/conda.sh
conda activate fluxvla-thor
cd /home/robot/VLA/FluxVLA
python -m fluxvla.projects.a10 doctor
```

下面的命令先显示实际参数；加 `--execute` 才运行。每次运行的解析配置、命令、FluxVLA 提交号及包含未追踪文件的修改状态保存在 `work_dirs/a10/runs/`。

```bash
python -m fluxvla.projects.a10 teleop --vr-only
python -m fluxvla.projects.a10 record --root datasets/a10/新数据集 --episodes 1
python -m fluxvla.projects.a10 record --simulate --root /tmp/a10_模拟检查 --episodes 2 --frames 6 --execute
python -m fluxvla.projects.a10 validate --execute
python -m fluxvla.projects.a10 train --steps 1
python -m fluxvla.projects.a10 serve --checkpoint /path/to/FluxVLA模型.safetensors
python -m fluxvla.projects.a10 replay --samples 3
python -m fluxvla.projects.a10 infer --cycles 1
```

进行实际录制或实机推理前，现场人员须先核对相机实体、A10 地址、夹爪范围和关节单步范围；这些站点参数填写在 `configs/a10/site.local.yaml`，并在核对后将 `robot.allow_motion` 设为 `true`。已有数据集续录需加 `record --resume`。同一时刻只运行一个 XLeVR 实例。

### 2026-10-08：单平台内迁第一批实现

- 新增平台相机 `cameras.py`，录制和推理共用设备名、尺寸、RGB 转换和旋转设置。推理桥接已删除 `sys.path` 和 A10_new 相机源码导入。
- 新增 `Follower` 边界和注册到 FluxVLA operator 注册器的 `A10LegacyFollower`，适配现有 `SET_JOINTS_BATCH`、`SET_EE_DELTA`、`reset` 和仅诊断的 `SET_EE_ANCHOR`；未来 follower 由团队确定接口后再接入，不定义虚假的可执行四元数协议。
- 将必要 VR 实现迁入 `fluxvla/projects/a10/vr/`，删除 LeRobot 基类、外部仓库导入和全局目录切换。保留已有坐标映射、掉帧保护、重锚、诊断和 WebXR 手柄解析。来源及许可证保存在该目录的 `NOTICE.md` 和 `LICENSE.*`；没有复制旧 TLS 私钥，现场首次启动生成本地证书。
- 新增 PyArrow/PyAV 流式 `EpisodeWriter` 和平台内部录制／遥操作入口。按集暂存视频和数值记录，支持保存、放弃、重录、续录与任务标签；模拟数据有 `SIMULATED_DATA.json` 标记。
- 配置和命令删除 LeRobot/XLeRobot/OpenPI 仓库路径与其他 Python 环境引用；相对路径按平台目录解析。最新 1.2 GB 数据集已复制到 `datasets/a10/a10_vr_9_21_4`，原数据仍可供对照。
- 官方固定版本 PI0.5 基础权重及 tokenizer 下载完成，模型文件为 14,467,165,872 字节。短训尚待本次模块检查完成后执行。
- 测试：平台模拟录制 2 集、12 帧，现有 FluxVLA 验证工具读取成功，30×7 未来动作窗口、12 张图像及 4 个视频文件检查全部 PASS。源录制数据在校验期间未改变。
- 又使用同一原生入口续录 1 集，数据成为 3 集、18 帧；原有集没有覆盖，新的 30 步检查、18 张图像、6 个视频文件均 PASS。独立 writer 单测还验证了放弃重录不改变已有数据，以及新任务标签、episode 边界填充。
- 注册式 `A10LegacyFollower` 接入后，原有模型服务到 TCP 的模拟闭环测试再次通过。相机 RGB 通道转换和 180 度旋转的单测通过。
- VR 本地 HTTPS 和 WebSocket 完整启动、接收模拟手柄、首次使能重锚、拒绝过期输入及退出释放服务的测试通过。录制器、相机和 VR 的 3 个原生回归测试通过，命令为 `python test/test_datasets/test_a10_native.py -v`；控制桥接的 2 个测试也通过。
- 平台环境增加 `requirements-a10.txt` 中的 `websockets==15.0.1`；其余使用 FluxVLA 已有的 NumPy、SciPy、OpenCV、PyArrow 和 PyAV。VR 页面已移除没有实现后端的旧串口配置菜单，WebSocket 端口由平台服务注入。
- 前期为外部 LeRobot 遥操作脚本添加的临时参数已撤销，当前入口直接调用平台内部模块。模拟集和真实演示数据禁止互相续录，训练入口会拒绝误选的模拟数据。
- 自动审批曾拒绝默认广泛绑定网卡的 VR 配置；已采用本机 `127.0.0.1` 默认值。现场 VR 访问需在 site 配置填写本机的具体局域网地址。

### 2026-10-08：原生训练验证与后续待办

- 使用下载的真实 PI0.5 基础权重、平台内的最新 A10 数据和 30 步动作窗口启动 1 步短训；前向、反向和优化步执行到保存阶段，FSDP 在 PyTorch 2.13 的完整 state_dict 导出时报错。日志保存在 `work_dirs/a10/smoke_train.log`，该轮不算训练验收成功。
- 本机单 Thor 配置改为复用 FluxVLA 的 `DDPTrainRunner`，取消不需要的 FSDP 专用项；EMA 默认关闭以减少统一内存占用，配置仍可设置 EMA。正在验证完整保存和恢复流程。
- DDP 实际短训成功：最新平台内 A10 数据、batch 1、30 步动作窗口，完成前向、反向、优化与权重保存，loss=0.0799，退出码 0。产物为 `work_dirs/a10/smoke_train_ddp/checkpoints/step-000001-epoch-000-loss=0.0799.pt` 和同名 `.safetensors`，可通过 `latest-checkpoint.*` 使用；这是一小步流程检查，不能证明抓取效果。
- 基础权重 SHA256 为 `da96d3abaabe6b9039e1d5bfe30e0af00432d7bb248f1bcaf0501ee4a6e237ea`，与 Hugging Face 固定版本的文件记录一致。
- 未来 follower 接口、四元数顺序、位置字段和坐标系全部按用户最新回复等待团队确定；这次只接现有 A10_new 方法。

### 2026-10-08：完整恢复训练与平台内真实数据复验

- 完整恢复测试第一次因约 38 GiB 的模型／优化器 checkpoint 与 GPU 状态同时驻留，占用过多统一内存，恢复耗时过长；已中断本次测试进程，没有改动其他训练任务。
- 修改 FluxVLA 现有 `fluxvla/engines/runners/ddp_train_runner.py`：采用 `torch.load(mmap=True, weights_only=True)` 按需映射 checkpoint，复制到模型后释放 CPU 权重引用，再恢复优化器。旧序列化格式仍有兼容回退。
- 修复后恢复 PASS：从第 1 步恢复模型，成功匹配并载入 805 个具有梯度的优化器状态（共 812 个参数），恢复学习率调度器，再完成第 2 步训练，loss=0.0799，保存 `.pt` 与 `.safetensors`，程序退出码 0。完整日志：`work_dirs/a10/resume_train_mmap.log`。恢复及大文件保存仍需数分钟。
- 产物：`work_dirs/a10/smoke_train_ddp/checkpoints/step-000002-epoch-000-loss=0.0799.pt` 和同名 `.safetensors`；`latest-checkpoint.*` 已指向第 2 步。目录含统计量、解析配置和 tokenizer。完整训练 checkpoint 含优化器，约 38 GiB；仅推理权重约 14 GiB。
- 平台自有真实数据副本检查 PASS：35 集、37,225 帧、全部数值记录、105 个跨集抽样、210 张图像、13 个视频文件，以及 30×7 未来关节目标。视频时间戳最大误差约 3.05e-6 秒，校验期间数据文件未改变。报告：`work_dirs/a10/validation/report.json`；日志：`work_dirs/a10/validation_native.log`。
- 控制桥接测试现为 3 项，均通过：模拟闭环、相机／动作约定、A10 EE 增量和 reset 无 ACK 的实际协议行为；记录在 `work_dirs/a10/backend_tests.log`。原生录制／相机／本地 HTTPS+WSS 服务的 3 项测试也通过，记录在 `work_dirs/a10/native_tests.log`。
- A10 模型服务载入完成后主动释放 CPU 权重副本，减少转入 GPU 时的额外内存占用。

### 2026-10-08：真实模型离线推理通过，本阶段暂停

- 用第 2 步 `.safetensors` 权重启动平台原生 A10 服务，严格载入模型参数，自动读取训练目录内的 `dataset_statistics.json`，复用 FluxVLA 的 ZMQ 服务；监听仅为 `127.0.0.1:5568`。
- `replay --samples 3 --execute` 从平台自有真实数据的第 0、20,011、36,652 行（episode 0、17、34；覆盖两个任务）读取状态与两路 RGB 图像。实际模型三次均返回有限的 `30×7` 动作，客户端退出码 0；过程中未打开机器人 TCP。
- 报告：`work_dirs/a10/replays/20261008_163245/report.json`；日志：`work_dirs/a10/replay_native.log`、`work_dirs/a10/serve_native.log`。已通过该测试服务的 kill 接口关闭服务。
- 动作质量尚未验收：仅两步短训的预测中，第一个样本首动作的夹爪约为 -1.22 mm，超出当前物理 0–100 mm 范围。离线检查保留原输出用于诊断；不能把这些流程验证权重拿来直接控制机器人。实机桥接已有站点范围拒绝逻辑，后续仍须正式训练与现场验证。
- 复现命令如下。此次端口及超时覆盖保存在 `/tmp/fluxvla_a10_offline_site.yaml`，只包含本机 host、端口 5568 和 timeout_s=120；长期配置可放入不提交的 `configs/a10/site.local.yaml`。

```bash
# 激活 fluxvla-thor 并进入 FluxVLA；两条命令分别运行在两个终端
python -m fluxvla.projects.a10 --site /tmp/fluxvla_a10_offline_site.yaml serve --checkpoint work_dirs/a10/smoke_train_ddp/checkpoints/latest-checkpoint.safetensors --execute
python -m fluxvla.projects.a10 --site /tmp/fluxvla_a10_offline_site.yaml replay --samples 3 --execute

# 恢复训练检查：steps 是目标总步数，因此从 1 恢复到 2 只追加一步
python -m fluxvla.projects.a10 train --steps 2 --work-dir work_dirs/a10/smoke_train_ddp --resume-from work_dirs/a10/smoke_train_ddp/checkpoints/step-000001-epoch-000-loss=0.0799.pt --execute
```

- 离线独立性检查：当前项目模块和配置不引用其他业务仓库的源码／解释器路径，无 `sys.path` 注入或全局 `os.chdir`；保留的 `lerobot_a1` 只是已迁入算法的模式名称。原 LeRobot 遥操作文件上此前临时接入改动已撤销。`git diff --check` 与修改 Python 模块的编译检查均通过。
- 非阻塞提示：当前依赖有 Pydantic／Transformers 弃用提示，单卡训练退出时有 NCCL 清理提示；本轮训练与服务均正常退出。依赖升级和多卡支持不属于本次 Thor 离线验收。
- 阶段结论：共享配置、原生 VR／相机／录制、真实数据读取、原生训练保存／恢复、模型服务／离线回放已在 FluxVLA 中接通。按用户要求暂停；下一阶段不自动启动。
- 当前 30 Hz 是采集配置目标；录制会记录实际采集时间与有效帧率，硬件能否持续达到该频率仍待现场测量。测试模型服务已正常退出，端口 5568 已释放；没有遗留本次训练或推理任务。

### 2026-10-08：保存迁移阶段的 Git 快照

- 按用户要求创建开发分支 `kaanh`，提交本阶段代码、配置、环境记录、测试和本文；当前离线阶段继续保持暂停。
- 初始状态为原始提交 `6c94e73139d51fca9650fa76265c468378020558`，使用 `kaanh-initial` 标签标记；原 `main` 分支仍指向此提交。`kaanh` 保留原始历史，可对比迁移前后的全部改动。
- 数据集、模型权重、训练产物及日志继续保存在本机已配置的目录，由 `.gitignore` 排除；本次 Git 快照保存源码和开发记录，不包含这些大文件或站点私有配置。未推送远程仓库。
- 首次暂存全部新文件时发现迁入 VR 文件中的行末空白，已仅清理行末空白，未改功能或许可证内容；完整暂存检查通过。
- 查看本阶段变化：`git diff kaanh-initial..kaanh`；查看初始文件：`git show kaanh-initial:文件路径`。

## 暂停后的下一阶段：现场验收与正式训练

本阶段收尾后按用户要求暂停，不启动实机动作或长时间训练。恢复工作时从下面的第一项开始：

1. 接入真实两路相机，核对 base／right_wrist 身份、180 度旋转、图像尺寸及实际采集频率，将设备写入站点配置。
2. 连接实际 VR 头显，填写本机具体局域网地址，验证页面、手柄、重锚、过期输入保护及人工保存／重录操作；本地模拟手柄通过不等同于现场 VR 验收。
3. 核对 A10_new 服务地址、现有状态和控制命令、夹爪及关节单步限值，再开启站点 `robot.allow_motion`；先完成一小段实际遥操作／录制并检查输出。
4. 用真实演示数据开展正式微调，确定训练时长、任务划分、评估方法和可用权重。当前仅跑 2 步，权重用于证明流程连通，不能用于宣称任务成功率。
5. 使用正式验证后的权重，在现场人员看护下验证一轮 A10 闭环，再按结果调整执行与日志。A10_new 暂时保留为底层控制服务。

延期事项：OpenPI 旧权重转换等原配置确认后再做；未来 follower 的目标类型、四元数顺序、坐标系与协议由团队定版后接入；其他旧业务仓库暂保留作对照，验收之后再决定归档。本次没有删除原仓库、原数据或现场控制实现。
