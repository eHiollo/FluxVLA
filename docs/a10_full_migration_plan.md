# A10 整套 VLA 迁移开发记录

## 目标和边界

最终交付是一个独立的 FluxVLA 平台：VR、相机、录制、数据处理、训练、推理和开发记录均位于 FluxVLA 内，正常运行不依赖旁边的 LeRobot、XLeRobot 或 OpenPI 仓库及其 Python 环境。现有模块中经验证的必要实现按功能迁入，保留来源和许可证；平台中已有的数据读取、变换、统计、训练 runner、注册器及 ZMQ 推理服务优先复用。

A10_new 暂时是底层运动控制后端，通过协议适配器连接。未来新的 follower 接管逆运动学和运动规划，平台通过目标姿态／位姿接口发送四元数等目标；当前关节批次接口与未来姿态接口分别声明能力，禁止把旧关节动作直接当作四元数。录制与 VR 输入为 30 Hz，当前 A10 底层控制为 500 Hz。

2026-10-09 补充边界：单平台指采集管理、数据处理、训练、推理和通信适配统一在 FluxVLA。控制器侧可以有独立部署的仿真或实机执行服务；它们共同提供目标接收和状态／图像反馈接口。仿真场景与底层控制不迁入 FluxVLA。仅变更接收端的前提是动作、观测、单位、坐标系和时序契约相同，当前尚未满足全部条件。

原 OpenPI 项目仍由团队调试。2026-10-12 已接入其合并 LoRA 的 PyTorch 导出：严格检查 metadata 和 811 个参数映射，确认 horizon=30。FluxVLA 本地与 ZMQ 推理一致；原 JAX 的逐数值对照、任务质量仍未验收。

## 当前进度（2026-10-15）

**分支分工**：`kaanh_follower` 保存已验证的推理／控制基线；`kaanh_follower_vr` 从 `efcdb2f` 创建，接入旧 VR 开发成果，后续 VR 开发在该分支进行。当前为保护主目录未提交修改，VR 使用独立工作区 `/home/robot/VLA/FluxVLA_vr`，主目录保持 `kaanh_follower`。

**提交约定**：按用户最新要求，每次工作对话完成后统一提交一次，提交标题及内容摘要使用中文；开发文档同步记录，不再按每个小任务单独提交。此约定取代下文历史记录中的逐小任务提交要求。

用户确认集成目录已跑通，现将 `kaanh_follower` 的完整实现迁回 `/home/robot/VLA/FluxVLA`。迁入基准为 `b98edd2`，包含原生 Triton + CUDA Graph 加速推理。正式链路为 `scripts/inference.py → A10InferenceRunner(BaseInferenceRunner) → A10Operator(BaseOperator)`，使用原有注册器、dataset/transform 和 ZMQ 服务；原有相机、USB 夹爪、关节 follower 接口随分支保留。

原目录 VR 开发内容已保存至 `backup/vr-before-consolidation-20261015`（`e9e4b93`），不混入当前推理版本。模型、tokenizer、CUDA 扩展及现场配置已核对保留；迁回后的测试结果和旧工作目录清理结果见本文末尾。默认配置保留 dry_run 和运动开关。用户确认的跑通情况不替代抓取成功率、失联行为和目标跟踪质量的专项验收。

下方历史方案和开发记录保留用于追踪变化，其中旧工作目录路径只代表当时的位置。当前软件接口和操作命令见 [现场说明](a10_onsite_guide.md)。后续每次工作对话统一提交一次，详细进度在本文末尾。

### 2026-10-09 进度快照（历史）

“单平台业务内迁与离线流程验收”已完成，用户已确认 VR 浏览器连接问题解决，但完整实机验收尚未完成。2026-10-09 用户调整下一步为先用控制器侧仿真跑通流程：仿真、IK、运动规划与实机伺服留在控制器侧；FluxVLA 负责业务流程、发送目标及接收观测。已核查 A10_new 与用户提供的 follower 客户端，具体协议和视觉反馈待控制器证据确认。继续按独立小任务更新开发记录并提交 commit；实机验证和 OpenPI 权重问题仍暂缓。

SSH 只读核查补充：工控机安装 KAANH／ARIS 2.5.7，已有编译后的关节与 Cartesian follower；完整目标处理源码及视觉仿真尚未取得。工控机 A10_new 已发展到异步批次协议 v2，平台还未迁入相关能力。下一小任务优先适配新版停稳状态，再分步迁移异步协议；控制器仿真／图像契约继续确认。详见 [工控机核查记录](a10_controller_follower_review.md)。

## 2026-10-09 架构纠正：使用框架扩展接口

用户指出此前把运行实现集中在 `projects/a10`，形成了独立业务流程，不利于新增机器人。这一问题成立：注册 `A10LegacyFollower` 并不能替代对 `BaseOperator`／`BaseInferenceRunner` 的实际继承和调用。下列结构调整优先于新版控制器异步协议迁移。

核查原项目：`engines/operators/base_operator.py` 提供观测和动作接口，已有 Aloha／UR／Franka／Tron2／Oli 子类；`engines/runners/base_inference_runner.py` 提供预处理→推理→后处理→执行模板和远程 ZMQ。没有找到框架级遥操作基类；Oli 的 teleop 消息是控制协议，迁入的 `BaseInputProvider` 则仅是 WebXR 服务内部接口。

本轮小任务（逐项验证、记录和提交）：

1. **机器人接口**：在 `engines/operators` 实现继承 `BaseOperator` 的 `A10Operator`，TCP 编解码放同层 transports，相机作为共享组件；现有 A10 协议保持单六轴＋夹爪，控制器负责轨迹规划。兼容旧导入不保留第二套实现。
2. **推理接口**：在 `engines/runners` 实现 `A10InferenceRunner(BaseInferenceRunner)`，复用基类远程预测与处理阶段，经 `RUNNERS` 和现有 inference 脚本构建。项目 CLI 只组装配置和调用 runner。
3. **遥操作接口**：新增 `engines/teleoperators/BaseTeleoperator` 及 `TELEOPERATORS` 注册器，WebXR 输入成为子类；输入采集、动作映射、机器人发送分别负责。遥操作循环进入 runners，支持替换输入方式，基类不写死单臂维度。
4. **数据与录制归位**：数据转换和在线数据集移入现有 transforms／datasets；录制 writer 进入 datasets，字段和相机由 schema 配置；A10 的具体字段留在 A10 配置／实现中。保留已验证的保存、重录、续录和合成数据隔离。
5. **验收与说明**：验证注册器构建、基类继承、假服务闭环、VR 服务与录制兼容；新增机器人扩展说明，说明单臂／双臂／人形是各自 operator、动作映射和数据 schema，不代表本轮已实现双臂／人形控制。

依赖方向：项目配置／CLI → runners → operators／teleoperators／datasets。框架实现不得反向导入 `projects/a10`。`projects/a10` 只保留站点检查、配置装配和兼容入口。重构期间不连接工控机或真实运动，不修改控制器团队仓库，不启动正式训练。

## 2026-10-09 执行范围：单文件 follower 接入（历史）

用户确认先做 follower，并要求运行实现全部集中在 `engines/operators/a10_operator.py`，使用 test 验证通信格式。当前在 `kaanh_follower` 分支实施；上方广泛结构重构计划暂缓。本次不创建 transports／teleoperators 等目录，不修改 runner、训练、录制或 VR，不连接真实控制器。

通信沿用附件 WS 5999／UDP 9998 和 40 字节消息头，A10 仅解析六轴单臂；PQ 是相对目标 xyz 米＋xyzw，不是六关节＋夹爪状态。夹爪独立使用源码已有的 `g_move --pos=...` 封装，真实 WS 路由／反馈字段仍待控制器确认，默认反馈缺失。已完成单文件 operator、离线及本机网络测试；真实控制器／VR 联调仍未执行。

## 实施清单

| 模块 | 交付内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| 环境 | 固定 Thor 可用环境与仓库版本 | CUDA 算子和 CLI 能运行 | 已完成，见 `local_setup/thor/README.md` |
| 数据 | LeRobot v3 视频、帧、状态和未来动作读取 | 现有 35 集、37,225 帧校验 | 30 步验收通过，见下方记录 |
| 统一配置与入口 | A10 项目配置、检查与统一命令 | 命令预览和配置校验 | 已实现，见下方记录 |
| VR/录制内迁 | VR 网页、服务、输入保护和原生录制器迁入 FluxVLA | 移除外部仓库路径后可运行，录制输出被现有 reader 读取 | 本地服务、模拟录制／续录通过；待现场验收 |
| 相机内迁 | 平台内相机采集，训练／推理共用名称与旋转 | 无 A10_new Python 路径依赖，RGB 内容校验 | 已实现；待物理相机验收 |
| 现场检查工具 | 双相机快照／采集测速、A10 只读状态查询、报告 | 模拟设备与故障检查 | 已实现并通过 5 项测试；实机验证按用户要求延期 |
| follower 接口 | 当前关节批次、EE 增量后端与上层流程解耦 | A10 legacy 后端继续工作 | 已实现注册式后端；新接口待团队确定 |
| 原生训练 | A10 7 维状态、未来关节状态目标、PI0.5 配置 | 样本变换、短训、恢复训练 | 1 步短训、完整恢复至第 2 步及两种格式保存均通过 |
| 离线回放 | 历史观测到模型输出，不发机器人命令 | 输出维度、有限值与日志，诊断范围 | 保存后的真实权重与真实数据回放通过；未验收动作质量 |
| 实机推理 | FluxVLA 模型服务接 A10 TCP 批量动作 | 现场看护下完成闭环 | 模拟闭环通过，待正式训练权重和现场 |
| 控制器仿真／新 follower 接入 | 控制器提供执行与观测，平台增加协议适配 | 协议、状态、双相机与录制／推理闭环 | 本机及 SSH 只读核查完成；确认 follower 库，完整语义和仿真图像仍待确认 |
| 新版控制器兼容 | 停稳语义、能力版本、观测年龄与异步批次代际 | 离线协议测试后再联调 | 发现 TCP v2 差异；待适配，尚未连接业务端口验证 |
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

### 2026-10-08：下一阶段小任务 1——现场检查工具

- 新增统一命令 `python -m fluxvla.projects.a10 check-hardware`，默认预览，`--execute` 才执行；支持 `--cameras-only`、`--robot-only`，可分开验证。沿用统一配置、运行清单及本机设备互斥锁。
- 相机检查枚举设备及 by-id/by-path 别名，拒绝缺失设备和两个角色实际指向同一节点；同时保持两路采集，检查 RGB 尺寸与类型，保存应用旋转后的 PNG 快照及成对采集吞吐。该测速不包含机器人／VR／编码成本，也不能证明传感器持续帧率；相机实体身份与朝向仍须人工看图确认。
- A10 检查仅发送 `GET_POLICY_STATUS`、`GET_FOLLOWER_STATE`、`GET_POLICY_STATUS`。控制器忙时停止检查，避免把批次执行期间的旧状态当作本轮有效观测；不发送 reset、EE 增量或关节批次。协议通过不代表状态新鲜度、物理单位或机械限值已经验收。
- 相机和机器人分别在子进程内检查，默认每项 60 秒总时限，超时终止本次检查进程，避免相机驱动卡住；机器人单次网络操作限制 3 秒。报告存放在 `work_dirs/a10/hardware_checks/<时间>/report.json`，失败退出码为 1。自动检查通过时，现场人工验收状态仍为 PENDING。
- 验证：`python test/test_datasets/test_a10_hardware_check.py -v` 的 5 项测试通过，覆盖纯查询 TCP、控制器忙、RGB 快照／成对测速、尺寸异常释放相机、缺失／重复设备、超时不得误报成功。日志：`work_dirs/a10/hardware_check_tests.log`。
- 本机初查未发现 `/dev/video*` 或 USB 相机；当前局域网 IP 为 `192.168.110.119`，仅作现场信息，不自动写成共享配置。用户明确其他同事正在开发机械臂／OpenPI，因此本轮不连接真实 A10，不启动实机动作、VR 局域网服务或正式训练。
- 实际执行 `check-hardware --cameras-only --execute` 正确报告两路设备缺失，退出码 1；报告为 `work_dirs/a10/hardware_checks/20261008_165946_380163/report.json`。这是缺少硬件的预期失败，不记为硬件验收通过。执行过程中未进行机器人探测。

### 2026-10-08：下一阶段小任务 2——站点模板与现场操作说明

- 小任务 1 已提交为 `9a92f6b`（`feat(a10): add bounded onsite hardware checks`）。后续继续采用独立小任务完成、更新本文、提交 commit 的顺序。
- 新增 `configs/a10/site.example.yaml`，可复制为 Git 忽略的 `site.local.yaml`。模板保留运动关闭、未确认限值为空、相机路径占位以及回环网络地址，避免把本机临时设备身份写入共享配置。
- 新增 `docs/a10_onsite_guide.md`，按环境／配置、相机快照、只读状态、VR-only、短集录制、验收记录顺序说明，并解释吞吐测量与真实录制帧率的区别，以及 VR 末端增量与模型关节批次的不同检查边界。
- 验证：模板通过统一配置加载，正确继承 640×480 相机尺寸且运动保持关闭；文档中的 8 条 CLI 命令通过参数解析；硬件检查、VR-only、录制的 3 项命令预览通过，并用 mock 确认没有启动子进程。`git diff --check` 通过。
- 本轮已完成可独立推进的现场接入准备。真实设备验证按用户最新要求延期；不改动机械臂和 OpenPI 同事的开发环境。正式训练的数据／任务划分与效果评估仍待后续安排，两步检查权重不作为实机控制权重。

## 下一阶段：现场验收与正式训练

### 2026-10-08：修复 FluxVLA VR 无法从头显连接

- 用户反馈旧 LeRobot `--vr-only` 能连接，FluxVLA 不能。排查发现原 XLeVR 默认绑定 `0.0.0.0`；平台配置绑定 `127.0.0.1`，且没有站点覆盖文件。头显无法通过电脑局域网地址访问只监听回环的服务，这是已确认的配置问题。
- 创建本机 `configs/a10/site.local.yaml`，仅将 `vr_server.host_ip` 设置为当前无线网卡的 `192.168.110.119`。该文件继续由 Git 忽略；共享默认配置和机器人运动设置未改变。换网络／IP 后需更新此站点值。
- `doctor` 增加 VR 页面及 WebSocket 端口显示，并在回环配置时提示头显不能通过局域网连接；VR 启动输出也增加此提示和首次使用本地证书的说明。操作文档补充专门排查步骤。
- 用真实统一入口启动 `teleop --vr-only --execute`：两个端口均绑定本机局域网地址；通过该地址访问首页、配置脚本、业务 JS、样式及配置 API，全部 HTTP 200；WSS 握手通过，发送 75 个模拟手柄包后，服务终端显示 `ee.enabled=true`。报告：`work_dirs/a10/vr_lan_fix_report.json`，服务日志：`work_dirs/a10/vr_lan_fix.log`。
- 平台原生相机／录制／HTTPS+WSS 的 3 项回归测试全部通过，日志：`work_dirs/a10/vr_lan_fix_tests.log`。网络检查是在本机经局域网接口完成，测试客户端接受了本地自签名证书；实际头显浏览器的信任和 WebXR 行为仍需用户重试确认。
- 测试只启动 VR-only，没有连接机械臂。测试服务完成后退出，用户可重新运行原命令。

详细操作见 [现场接入操作说明](a10_onsite_guide.md)，站点模板为 `configs/a10/site.example.yaml`。

现场接入准备已恢复，实机验证按用户最新要求暂缓。后续配合现场人员，从下面的第一项开始：

1. 接入真实两路相机，核对 base／right_wrist 身份、180 度旋转、图像尺寸及实际采集频率，将设备写入站点配置。
2. 连接实际 VR 头显，填写本机具体局域网地址，验证页面、手柄、重锚、过期输入保护及人工保存／重录操作；本地模拟手柄通过不等同于现场 VR 验收。
3. 核对 A10_new 服务地址、现有状态和控制命令、夹爪及关节单步限值，再开启站点 `robot.allow_motion`；先完成一小段实际遥操作／录制并检查输出。
4. 用真实演示数据开展正式微调，确定训练时长、任务划分、评估方法和可用权重。当前仅跑 2 步，权重用于证明流程连通，不能用于宣称任务成功率。
5. 使用正式验证后的权重，在现场人员看护下验证一轮 A10 闭环，再按结果调整执行与日志。A10_new 暂时保留为底层控制服务。

延期事项：OpenPI 旧权重转换等原配置确认后再做；未来 follower 的目标类型、四元数顺序、坐标系与协议由团队定版后接入；其他旧业务仓库暂保留作对照，验收之后再决定归档。本次没有删除原仓库、原数据或现场控制实现。


### 2026-10-09：控制器侧仿真与 follower 接口核查

- 最新分工取代此前“在平台内新建 A10 物理仿真”的候选方向：仿真与实机控制由控制器承担，FluxVLA 统一发送指令、接收反馈，复用既有 VR／录制／训练／推理业务流程。
- 只读检查 A10_new：XML 是 ARIS／KAANH 控制器配置，确有虚拟驱动选项和 `follower_cart`、WebSocket 5888／5999、UDP 9998 配置；本机未找到依赖库实现、几何资源和带双相机的完整仿真场景，不能确认工控机实际启用情况。
- 用户提供 `robot.7z`，只在临时目录读取 3 个客户端文件，未将附件加入 Git 或执行网络控制。核实 40 字节头、WS 控制／UDP PQ 发送、mm→m 转换、xyzw 顺序；内存报文和合成状态解析检查通过。附件按多模型 20 轴编写，与现有单 A10 六轴＋夹爪配置不同。
- 核查发现目标相对基准需服务端确认、反馈单位注释冲突、无相机接口、缺少控制器采样时间／报文序号、启动状态未经回读确认等接入缺口。详见 [控制器与 follower 核查](a10_controller_follower_review.md)。客户端参考不等于最终协议定版。
- 下一步顺序：①确认实际机器人模型／库版本，取得控制器 follower 源码片段和 get 样例；②明确控制／状态／图像契约，保持当前关节和新位姿动作空间独立；③实现协议编解码与只读反馈适配并做离线测试；④接控制器仿真验证 VR→运动→RGB／状态→原生录制；⑤完成仿真训练／推理闭环；⑥相同契约下切回实机并校准、验收。
- 原 `record --simulate` 是纯色图像／合成数值的 writer 测试，不是物理仿真，仍禁止作为演示数据用于训练。以后有物理状态与渲染图像的仿真演示须另标明来源、模型版本和接口版本，可按独立配置训练；仿真流程通过不代表实机任务效果通过。
- 本次没有安装仿真器、修改 A10_new、SSH 工控机、启动 follower 或发送真实运动。工控机信息由用户在可访问环境中核对后提供。

### 2026-10-09：工控机 follower 只读核查完成

- 用户随后授权 SSH，并明确账号为 `kaanh`。登录后只读取控制库声明、动态库符号／依赖、A10_new 源码／文档、端口监听和进程名称；没有修改远程文件、安装或启动程序、重启控制器、调用 sudo，未向机器人业务端口发送命令。SSH 已断开，凭据未写入文档或提交。
- 确认安装版本均为 2.5.7；`udpFollower.hpp` 与 `libkaanh_lib.so` 确认关节及 Cartesian follower 有真实实现。头文件注明 Cartesian 为“相对原点的增量”，但具体原点、旋转组合、PQ 解析与超时规则在未取得的库实现中，仍不能直接将绝对位姿接入。
- 工控机 XML 分别配置关节 UDP 9999、Cartesian UDP 9998；连同 WEB 5888／5999、TCP 8080 均观察到监听。端口存在不等于运动使能或协议验收；运行的 root `kaanhbin` 的 exe／cwd 当前账号无权读取，磁盘源码版本不视为运行版本。
- 在已查路径仍未找到完整双相机视觉仿真。磁盘配置是六轴实际 EtherCAT 从站；ARIS release 库依赖 libethercat，存在 UR5 DWG 几何资源，但 XML 指定的 Ka-Ur STL 路径不存在。这些证据不足以确认可直接跑通抓取仿真。
- 发现远程 A10_new（Git HEAD `a9b0aaa`，工作区仍在开发）比本机初查版本更新：执行期间刷新反馈，状态协议 v2 区分 `remaining`、`motion_settling` 和 `idle`；批次支持观测年龄、预期批次序号和平滑选择。平台当前只发 actions、等 remaining==0，尚未具备新版异步衔接。
- 后续优先项：离线适配 `idle`／`motion_settling` 和能力兼容，再单独迁移异步时序／代际；控制器侧继续确认 follower 语义、夹爪和仿真 RGB。控制规划仍留在控制器，不复制到平台。此次未改变平台运行逻辑或切换机器人后端。
- 更新文件：本文和 `docs/a10_controller_follower_review.md`，后者保留接口证据路径、文件哈希、限制和任务顺序。验证为 SSH 只读取证及文档差异检查；未进行硬件、协议通信或仿真执行测试。

### 2026-10-09：框架结构纠正，小任务 0——核查与计划

- 核查现有 BaseOperator、BaseInferenceRunner、注册器和 inference 脚本；确认没有框架级遥操作基类，现有 VR 内部接口不足以承担跨输入设备的扩展。
- 修订结构目标和小任务顺序，优先解决用户指出的框架隔离问题；此前离线流程验收保留，但不再把它等同于框架集成完成。
- 本小任务仅修改计划，后续各项分别验证并提交。

### 2026-10-09：单文件 follower 小任务 1——协议和原生 operator

- 从 `kaanh` 创建 `kaanh_follower`。新增 `engines/operators/a10_operator.py`，包含消息编解码、内部连接类、单臂反馈和 `A10Operator(BaseOperator)`；在既有 operators 包导出并注册，无新运行目录。
- WS 头沿用模板的 id=1、type=0x1000、reserved1=0xA1B2C3D4；UDP 除载荷长度外均为 0。检查二进制帧、精确长度、JSON、ret_code、有限值及单位四元数；有界跳过空 ACK，异常事务关闭连接。
- 原生接口：显式 connect/close、get_state/get_frame、start_follower/stop_follower、send_eepose、send_gripper。启停读取 running_state／Cart 确认，不自动登录或使能。仅允许本连接启动的 follower 接收目标／停止请求。close 只关网络，不保证控制器停止。
- actual_pos 与 motor_pos 分别保留，不用目标值代替实际反馈；拒绝额外轴／多臂数组。关节按模板默认度→弧度，TCP 反馈单位默认未知，需明确配置后才转换。夹爪反馈默认 None；无实际夹爪反馈就不生成完整 qpos。配置路径只支持已确认的毫米数值字段，不宣称已找到真实字段。
- g_move 命令依据本机 A10_new 的 A10GripperMove 定义，单独走 WS；ACK 只说明收到回复，不能确认到位。wait=True、关节命令、回零及平台轨迹执行明确不支持，避免复用不适用的 ROS 路径。
- 验证：`python -m unittest discover -s test -p test_a10_operator.py -v`，7 项离线测试通过，覆盖协议字段、嵌套状态、坏帧、维度／单位、缺失反馈、四元数和注册器无网络构建。尚未进行网络收发或实机验收。

### 2026-10-09：单文件 follower 小任务 2——网络测试与说明

- 小任务 1 提交：`2d13965`。扩展 `test/test_a10_operator.py`，本机真实 WebSocket 服务与 UDP 接收器独立核对 wire 字节；仅监听 127.0.0.1 随机端口，无工控机连接。
- 新增完整查询→启动确认→UDP PQ→夹爪 WS→停止确认测试；覆盖只有 ACK 而未切换模式时禁止发目标、拒绝接管、缺实际关节禁止启动、只读门禁、损坏帧／错误返回／ACK 洪泛／超时／断线清理、非法输入不得上网、反馈故障撤销发送、并发查询序列化及重连不继承归属。
- 核对模板后补充单臂 PQ 的 `[TCP, 辅助轴位姿]` 双组格式，仅选择该单臂 TCP；六轴反馈仍严格校验，额外机械臂不会被截断后假装兼容。
- 最终验证：`python -m unittest discover -s test -p test_a10_operator.py -v` **17/17 PASS**；`python -m unittest discover -s test/test_datasets -p test_a10_workflow.py -v` **3/3 PASS**，既有相机／动作转换、旧 EE/reset 协议、假模型完整闭环保持通过。`git diff --check` 通过。测试 socket／服务均随上下文退出，没有新增测试结果大文件。
- 使用说明、接口表、配置和已知限制写入 `docs/a10_controller_follower_review.md`。依赖复用 requirements-a10.txt 已有 websockets==15.0.1，未安装新包。
- 本轮范围完成：新增运行逻辑只有 a10_operator.py 和既有包导出，未新建运行目录，未修改 runner／训练／录制／VR 或任何外部仓库。真实坐标语义、夹爪字段／命令路由及实机启停仍须另行确认；测试通过只证明符合参考报文与合成反馈，不宣称真实机器人执行通过。


### 2026-10-09：PDF 协议核对和真实只读通信通过

- 阅读用户提供的《遥操作使用说明》并 SSH 只读检查工控机，核对单六轴／单末端规划器及 WS／UDP 配置。PDF、robot.7z 保持未跟踪；没有改动远端文件或进程。
- 使用现有 A10Operator 直接连接 192.168.110.124:5999，三次 get 全部解析成功。查询时机器人未使能、Stopped、无故障，follower 为 None／未运行。未发送 UDP 数据或任何使能、启停／运动命令。
- 在 TCP 8080 状态确认 idle=true、policy_running=false 后，读取旧 GET_FOLLOWER_STATE；六轴＋夹爪七值可用，WS 度→弧度结果与 TCP 实际关节最大差约 3.63e-5 rad。旧接口夹爪读数约 0.026738 mm，但不把返回值当作夹爪硬件健康证明。
- 文档区分 start_follower 与 follower_cart；现有 Python 启动方法沿模板只发送后者，真实启动次序仍待验证。WS 状态未确认夹爪反馈字段，新 operator 尚未合并旧 TCP 夹爪读数。
- 结论：真实连接／状态读取通过；实际 follower 启停、目标接收执行和夹爪控制未验收。详情与下一步现场验证边界见 `docs/a10_controller_follower_review.md` 新增章节。
- 本小任务仅更新两份开发文档；功能代码未修改，不需要重复运行离线回归测试。SSH 及业务查询连接均已退出。


### 2026-10-09：使能指导纠正——按实际操作模式核对

- 用户发送 en 后仍 Disabled。只读调查确认控制器处于 Manual，日志 ID104（15:49:58）记录“自动使能”，历史同实例有“手动模式指令使能／掉使能”。配置 en 名称为自动模式使能，SystemSema 分开保存手动、自动和虚拟状态。
- 纠正之前将 en/manual_en 简单等同于真机/虚拟的指导；当前现场下一步应验证 manual_en，并以 Enabled 回读为成功条件，手动掉使能对应 manual_ds。保持现有模式，不改变控制器配置或保护条件。
- 本次 agent 只查询状态和远端日志（SQLite mode=ro），未执行使能或运动。功能代码未修改，无需重跑离线测试；只更新说明和现场记录。使能结果等待现场用户回传。


### 2026-10-09：现场手动使能验证通过

- 用户终端截图确认 manual_en 返回 ret_code=0，随后状态为 Manual／Enabled／Stopped／无故障。真实 WS 手动使能和状态确认通过，修正后的命令选择有效。
- 连接关闭后电机保持使能；尚未进行 follower 启停或 UDP 目标执行验收。下一步为现场同连接的 follower 启动／停止测试，不发送位姿目标。
- 本小任务只同步现场证据至两份开发文档，没有修改功能代码或远端状态。

### 2026-10-09：修复 follower 启动前坐标系准备与拒绝后的检查

- 现场启动返回 -987313“请切换到笛卡尔坐标系”。只读确认当前 Axis／Disabled／Stopped／日志错误，follower 未运行（mode 字段仍为 Cart）；远端日志 ID106 与截图一致。此前启停指导遗漏模板中的 Tool 坐标系设置，当前不记启停验收通过。
- a10_operator.py 同文件新增 set_tool_coordinate()：显式选择工具坐标系，检查无故障／Stopped／follower 未运行，并回读确认；start_follower 增加坐标系前置检查，不自动修改坐标系。
- 启动返回控制器错误后，旧异常事务连接仍关闭，仅新建连接只读查状态。确认未运行才撤销本次待清理标记；否则保留不确定性，原始异常上报，不自动重发启动或发送停止／运动目标。
- test_a10_operator.py 新增现场问题回归与失败路径检查，本机回环 **22/22 PASS**；git diff --check 通过。说明同步 docs/a10_controller_follower_review.md，仍保持单文件 operator 结构。
- agent 对工控机仅 get／SSH 只读日志，无设置或运动。下一步由现场用户清除坐标系报警，验证 Tool 设置，再手动使能及验证 follower 启停；本机测试不能代替实机验收。


### 2026-10-09：命令行控制台小任务 1——公开使能接口

- 用户批准在 kaanh_follower 实施控制台。原目录处于 kaanh_a10_runner 且有未提交开发内容，因此使用独立 worktree `/home/robot/VLA/FluxVLA_follower`，原工作区未改。现有 fluxvla 环境可用；worktree 通过未跟踪的本地符号链接复用原目录三项 CUDA 扩展，无重新安装或编译。
- 在 a10_operator.py 补充 enable()/disable()，分别发送 manual_en/manual_ds 并限时回读 Enabled/Disabled；仅支持已确认 Manual 模式，不自动切换模式。使能要求无故障、Stopped、follower 未运行；掉使能可在有故障时请求，但不视为急停。
- 启动 follower 要求 Enabled/Stopped；状态回读发现掉使能时撤销 UDP 发送资格。公开 follower_stop_pending 供控制台判断本次启动是否仍待清理；停止回读确认未运行时，即使有其他报警也可确认退出 follower，报警仍保留。
- 本机回环测试 27/27 PASS，新增使能/掉使能往返、前置条件零控制发送、仅 ACK 超时、拒绝、故障下掉使能及停止、未使能禁止启动等检查。没有连接实机或发送硬件命令。
- 用户此前截图已确认 Tool 设置成功、Disabled、follower 未运行；该现场进度同步至本分支。后续真实启停、UDP 目标和夹爪仍待验证。


### 2026-10-09：命令行控制台小任务 2——交互入口及异常清理

- 新增 scripts/a10_control.py，仅解析参数、显示中文状态、调用 A10Operator 公开方法，没有复制协议或访问内部连接。支持 shell 持续连接、单次 get/使能/掉使能/坐标系/夹爪操作、help 和 get --raw；端口默认 WS 5999、UDP 9998，host 显式填写，可用 --read-only 禁止所有控制。
- shell 指令沿用 manual_en/manual_ds/set_jog_coordinate --tool/follower_cart/stop_follower/g_move --pos=...，pq 使用七个数字，米和 xyzw。跟随启停及 pq 仅限同一 shell 会话；不提供任意原始命令透传，也不猜测清错命令。
- exit/Ctrl+D/Ctrl+C 尝试停止本次会话启动的 follower 并回读；预先已运行的 follower 不接管、不自动停止。断线后不重发命令，停止未确认时明确提示现场处理并以非零状态退出；关闭连接不自动掉使能。
- 修复发送启动指令时 Ctrl+C 的边界：在 send 之前记录待清理状态，避免指令已发出但清理标记尚未写入。新增回环用例模拟该时机并确认停止请求。
- 本机测试 37/37 PASS（operator 27 项、控制台 10 项）。覆盖完整 WS/UDP 往返、只读、错误输入、EOF/中断、已有 follower、断线、停止超时、单次命令与 help 无网络。git diff --check 通过。所有端点均为 127.0.0.1；没有实机操作。


### 2026-10-09：命令行控制台小任务 3——入口验收和现场说明

- 在 docs/a10_onsite_guide.md 顶部加入原生 follower 控制台教程：独立 worktree 启动路径、命令速查、逐条启停验证、单位、只读模式、退出和失败处理。旧相机/VR/录制流程保留在后半部分并明确区分。
- 新增真实子进程入口测试：从 /tmp 启动本 worktree 的脚本，通过回环服务完成使能、跟随启停和掉使能；验证脚本加载当前分支而不是另一个 editable checkout。补充 pq 负科学计数法输入支持及解析回归，避免 -1e-3 被当作选项。
- 最终验证：`/home/robot/miniforge3/envs/fluxvla/bin/python -m unittest discover -s test -p 'test_a10_*.py' -q`，**38/38 PASS**（operator 27、控制台 11）；git diff --check 通过。既有 Pydantic 字段警告不影响本次测试，没有修改依赖。
- 小任务 1 提交 a199c14；小任务 2 提交 6ae09ba。当前分支 kaanh_follower，工作目录 /home/robot/VLA/FluxVLA_follower。原 FluxVLA 目录的 runner 开发内容保持原状。
- 本阶段软件实现、模拟通信测试、使用说明已完成。没有访问 192.168.110.124 或发送实机控制命令。下一步由用户现场逐条验证 get → Tool → manual_en → follower_cart → get → stop_follower；UDP 目标执行和夹爪实际到位仍未验收。


### 2026-10-09：修复空闲 Ping 超时与断线后退出控制台

- 用户截图：get/manual_ds 报 sent 1011 ... keepalive ping timeout; no close frame received，随后控制台退出。核对当前 websockets 15.0.1 源码，默认 ping_interval=20、ping_timeout=20；该 1011 来自客户端保活失败，不是机器人业务 ret_code。截图不能确定是远端缺失 Pong、网络中断还是服务停止，也不能据此认定 manual_ds 已生效。
- A10Operator 连接显式使用 ping_interval=None，避免空闲会话依赖尚未验证的控制器 Pong 支持。业务 request/get 的超时、坏包检查和不确定事务断开策略保留。没有后台机器人状态监视，不宣称在空闲时立即发现所有断线。
- shell 断线及初次连接失败均保留交互提示符；新增 reconnect 显式重建连接并只读查询。重连不补发使能/掉使能/运动指令；单次终端命令失败仍非零退出，exit/Ctrl+D/Ctrl+C 仍按原规则退出。
- 保留旧会话跟随尚未停止的不确定状态，重连不会继承旧启动归属；回读明确未运行才解除该提示。连接仍有效且有待停止的本次跟随时，reconnect 保留连接并提示 stop_follower。停止不了仍提示现场处理，不因为 UI 保留而假装可以继续控制。
- 新增不回复 Pong 但可回复 get 的独立回环模拟，缩短库默认心跳时限验证空闲后查询；同时验证真实 get 超时仍关闭连接。新增设备离线启动、重复连接失败后恢复、断线保留提示符、失败指令不重发、旧跟随不接管及原连接停止归属测试。
- 验证：`python -m unittest discover -s test -p 'test_a10_*.py' -q` **43/43 PASS**（operator 28、控制台 15）；git diff --check 通过。仅测试 127.0.0.1，没有连接工控机或发送实机命令；真实控制器的空闲长连接仍需现场复验。
- 操作说明更新 reconnect、离线处理及保活说明，启动目录纠正为 /home/robot/VLA/FluxVLA（当前 kaanh_follower）；旧 FluxVLA_follower worktree 为 detached 历史版本。工作区同时存在的 projects/a10 等业务修改不纳入本次提交。


### 2026-10-12：关节 follower 小任务 1——协议核对及六轴 operator

- 用户授权在 kaanh_follower 执行。本机 /home/robot/VLA/FluxVLA 当前分支已确认；原工作区另有 OpenPI/夹爪开发修改，未纳入本任务。
- SSH 仅只读核对工控机 /usr/kaanh/kaanh-2.5.7/include/kaanh/follower/udpFollower.hpp、磁盘 XML、release 动态库符号与定向反汇编，没有发送机器人业务命令、启动进程或修改远端。检查结束关闭 SSH。本次 ps/ss 未观察到 kaanhbin 或 UDP 9998/9999 监听，不能确认当前运行程序版本。
- 动态库 SHA256：fbd638320e3025f5e3c4ec8ec8d9761c5f8499a7c69a96123b05dce039eb52b5。FollowerAjParam 接收回调约 0x28174c 搜索 jp，随后取方括号中的 JSON 数组；约 0x281d3a 对旋转关节乘 0.017453292519943295（度转弧度），约 0x281f32 入关节队列。未命中 jp 的另一路约 0x2823d5 读取 JSON motor_pos，解释旧 client.cpp 格式差异。采用 PDF 明示的 `jp --motor_pos=[...]`，不照搬 14 轴示例的 motor_vel/id。
- Header 枚举明确 Axis/Cart/None；磁盘 XML follower_aj 映射 FollowerAjAbsolute2，关节 UDP 9999、Cartesian UDP 9998。关节 prepareNrt 在所读代码中未找到必须 Axis 点动坐标系的检查，故不将其作为客户端启动前置条件；新增显式 set_jog_coordinate('Axis'/'Tool') 并回读，set_tool_coordinate 保留为兼容包装。
- a10_operator.py 继续单文件：两个独立 UDP socket，start_follower(mode='joint'/'cartesian')，send_joints({'right': 六轴绝对弧度})；wire 为六轴度数，不含夹爪。构造 command_mode 默认 cartesian，可配置 joint，启动选定模式后 command_mode 反映该选择。原 udp_port 继续指 Cartesian，joint_udp_port 默认 9999，两端口禁止相同。
- 启动要求六轴实际反馈、Enabled/Stopped、无故障、follower 未运行；Cartesian 另要求 TCP 和 Cartesian 坐标系。仅本连接确认对应模式可发目标；关节/末端互斥，待停止的启动不能被另一次启动覆盖。重连不继承归属，停止/关闭同时释放两路 UDP。get 状态与启动资格匹配，反馈模式变更/故障/掉使能撤销发送。
- 本机回环 operator 37/37 PASS（此前 28 项，新增 9 项）：独立检查 UDP 端口、40 字节头、六轴载荷、弧度转度、缺 TCP 的关节启动、互斥切换、坏输入零发送、错误模式/ACK 超时、拒绝后的只读恢复、只读及前置条件、故障/模式改变/重连、第二路 socket 初始化失败清理。没有发实机目标。
- 本小任务完成协议编码和模式管理；尚无动作块定时发送、关节到位确认或控制器丢包/失联验收。operator 不伪造这些能力。


### 2026-10-12：关节 follower 小任务 2——终端关节控制

- 小任务 1 提交 d270884。scripts/a10_control.py 增加 follower_aj、joints 只读查询、joints --deg 六轴绝对目标、set_jog_coordinate --axis；增加 --joint-udp-port（默认 9999），原 --udp-port 继续为 Cartesian 9998。
- joints 显示实际角度并给出可复制的目标文本，只显示不自动发送，缺实际反馈时报错，不以 motor_pos 代替。发送命令明确要求 --deg 和六个数，在 CLI 转弧度后调用 operator 的公开 send_joints；每次现场手动目标前先 get 回读。负科学计数法按数字解析。
- 关节启动/目标仅在 shell 中执行，末端启动显式指定 cartesian，确保两种模式切换不受上次模式影响。已有 reconnect/只读/exit/Ctrl+C 清理规则复用；夹爪命令继续独立。
- 控制台测试 22/22 PASS（此前 15、新增 7）：独立子进程六轴 UDP 完整字节验证、实际角度查询、Axis 设置、模式互斥、启动前禁止发目标、格式/单位错误、只读、缺实际反馈、退出/EOF/中断停止、断线重连不继承旧关节控制。该子进程也继续验证切回 Cartesian 的启停。
- help 已显示新命令且不连接设备；git diff --check 通过。当前总计 operator 37 + 控制台 22 = 59 项。本小任务没有发送实机控制指令。


### 2026-10-12：关节 follower 小任务 3——最终回归及现场说明

- 小任务 2 提交 3d1964a。补充独立的启动确认标记，将“待清理启动”与“确认可发送目标”分开；启动超时或被拒绝后，即使后续 get 恰好看到 Axis，也不自动恢复发送权限。需明确停止并重新启动。新增延迟模式回读和 UDP 异常/不完整发送禁止继续发送的回归。
- 最终本机测试命令：`/home/robot/miniforge3/envs/fluxvla/bin/python -m unittest discover -s test -p 'test_a10_*.py' -q`，**61/61 PASS**（operator 39、控制台 22）；git diff --check 通过。测试含独立子进程入口与回环 WS/两路 UDP，不连接真实机器人。
- docs/a10_onsite_guide.md 第 6 节新增关节现场教程，说明启动/停止、查询实际角度、复制当前六轴目标、单位、模式切换及夹爪独立通道；接口表同步。该文件原有未提交的 OpenPI 附录保留且不计入此次提交。
- 本阶段完成六轴 joint follower 软件实现与本机验收，仍保持机器人运行逻辑集中在 a10_operator.py。真实启停、运动方向/幅度、控制器接收丢包/失联规则及关节/夹爪同步尚未验收；没有扩展成 batch 定时流、关节回零或自动重发。
- 工控机仅在协议调查期间 SSH 只读，未发送任何业务控制报文或启动程序。本次已断开；现场运行前确认 kaanhbin 已运行。接下来由现场用户按教程逐项验证并回传结果，再记录实机验收状态。


### 2026-10-12：推理迁入 follower 小任务 1——原生推理适配

- 在独立 worktree `/home/robot/VLA/FluxVLA_integration` 的 `kaanh_follower` 实施；主目录 `vr` 未提交修改保留。选择性迁入 `kaanh_a10_runner` 已提交的 a9b2aea：A10InferenceRunner、OpenPIA10Policy、dataset/transform、注册、服务入口和测试。没有迁入 VR 或覆盖最新关节 follower/控制台/设备驱动。
- 默认 checkpoint/tokenizer 位于本仓库 checkpoints 内，保留 A10_OPENPI_CHECKPOINT/A10_OPENPI_TOKENIZER 覆盖；用本机已有权重的硬链接准备忽略的 13GB 资产，tokenizer 复制进本仓库。运行不导入 OpenPI/A10_new/LeRobot 仓库。
- 数据集 meta/info.json fps=30，OpenPI data_loader.py 以 t/fps 连续采样；默认 action_dt 改为 1/30 秒，预测 30 步、每轮选 10 步。默认指令 Reach the yellow lemon，夹爪工作上限 85mm。dry_run=True、allow_motion=False 保留。
- 推理测试 15 项：14 PASS、1 GPU 测试暂未启用。预处理另外与本机原 OpenPI 函数核对通过；模型实际推理留至最终验收。尚未连接 operator 的相机/USB 夹爪和动作块调度，下一小任务补齐。


### 2026-10-12：推理迁入 follower 小任务 2——相机/USB 夹爪统一观测

- 上一步提交 d1ca6f6。A10Operator 通过可选 cameras、gripper、gripper_transport 配置复用已提交 CameraOperator/GripperOperator；默认 controller 路由保持已有终端操作兼容，USB 必须显式配置，禁止同时传入 USB 配置和 controller 路由。connect 不使能、不发运动目标；USB 初始化失败释放网络。close 对所有资源逐项清理，错误明确报告。
- get_frame 返回真实六轴弧度 + USB 夹爪实际毫米的 qpos[7]、两路 HWC uint8 RGB；controller-only get_state 保持不变。保留机器人、夹爪、相机各自的 host monotonic 接收时间；顺序读取不等于硬件同步，坏输入/掉线不伪造零值。
- send_gripper USB 支持 wait=True 回读实际到位；非等待仅代表写入。USB 默认 85mm 上限，controller 旧通道保持 100mm，未知 controller 到位能力仍明确拒绝 wait=True。推理示例配置显式包含当前两路相机、USB 物理路径，需现场核对设备路径。
- requirements-a10.txt 增加 pyserial==3.5，已安装到 fluxvla 环境；pip 原代理配置报错后使用隔离环境完成安装，未修改全局代理。模拟回环 67/67 PASS（原 operator/CLI 61 + 新集成 6）；独立夹爪测试 10/10 PASS。没有连接真机或相机/USB 实设备。动作块调度仍待下一小任务。


### 2026-10-12：推理迁入 follower 小任务 3——动作块按时逐帧发送

- 上一步提交 e307c54。execute_trajectory 接收单臂 [T,6] 绝对弧度、可选 [T] USB 夹爪毫米、dt；整个动作块先检查维度、同长度、数值有限/单位转换和工作范围，不合格零目标发送。明确拒绝异步/head/旧 controller 夹爪定时轨迹，不增加 ROS/IK/插值。
- 要求本连接启动并确认 Axis/Enabled/无故障；preflight 和周期内有截止时间的 WS/USB 反馈检查，完成后再检查一次。默认每 0.1 秒监测、单通道检查最多 0.015 秒（且不超过 dt/2），时间预算可配置；不能保证瞬时检测所有故障或硬实时。
- monotonic 时钟逐行发送一个 UDP 六轴目标 + USB 夹爪非等待目标，最后等到 T*dt 后返回；同周期不等于硬件同步，到期滞后默认不超过 dt/2。错过时隙/发送超出周期/断线/故障则抛错、撤销目标授权，不重发、不积压补发。last_trajectory 记录 sent/completed，绝不把发送完成标为实际到位。
- 同步调度期间目标流归属单线程，禁止其他线程插入目标或重叠轨迹；stop_trajectory/stop_follower/close 可打断等待。停止跟随由调用方显式完成；runner 的 finally 按已有规则请求停止并关闭。串口断开不等于停止夹爪，机器人停止需要控制器确认。
- 本机 74/74 回环测试通过（新增轨迹 7）：实际包格式、发送间隔、USB 校验/位置、坏尾帧零发送、归属/模式检查、运行中故障/掉使能/模式变化/断线/USB 异常、超时无补发、取消和最终短块反馈。仅模拟设备，实机到位、控制器失联规则、丢包验收仍待现场。


### 2026-10-12：推理迁入 follower 小任务 4——完整闭环验收与操作说明

- 前三项分别提交 d1ca6f6（原生推理迁移）、e307c54（相机/USB 夹爪组合）、04358d3（动作块调度）。本项使用真实 A10InferenceRunner、注册器构建的 A10Operator 和真实回环 ZMQ/WS/UDP，只有模型/USB/相机硬件替身；覆盖多轮 2+2+1 步执行、实际夹爪反馈进入下一轮、只读零运动、模型异常和相机异常的停止/清理。没有引入第二套推理循环。
- 联调补齐：已启动 follower 时，get_frame 的相机/USB 异常撤销目标发送资格，但保留可用控制连接，让 runner finally 明确 stop_follower 后 close；网络自身损坏仍无法确认停止。CameraOperator.close 对各相机逐项尝试，避免一个 release 异常阻断其余清理。
- scripts/inference.py 强制使用脚本所在 worktree；新增从 /tmp 启动的子进程测试，验证构建本分支 A10 runner/operator，避免 editable 安装选错 checkout。开发资产、CUDA 扩展用本机硬链接准备，不改旁边仓库，不把二进制/13GB 权重加入 Git。
- 回归测试 109 项运行：108 PASS、1 GPU 项默认跳过；随后相机清理新增 1 项，相机 6/6 PASS。实际 GPU 项单独开启：5/5 PASS，严格加载 811 参数，本地原生模型与真实序列化 ZMQ 往返均输出有限 (30,7)，相同随机种子 rtol/atol=1e-5 一致。共 110 个不同测试已通过（其中 GPU 项单独运行），不等于原 JAX 数值对照或实机验收。日志在 /tmp/a10_final_regression.log、/tmp/a10_real_network_tests.log，临时日志不提交。
- 更新本计划顶部当前进度、follower 核查文档、现场教程：本地/远程入口、资产、物理设备配置、只读默认、30Hz/10步执行、限值和超时含义、错误清理/停止边界。本轮 `vr` 内容未迁移，主 worktree 未提交修改保留。
- 本阶段软件交付完成，未访问 192.168.110.124、未打开实 USB/相机、未使能或发送真机目标、未训练。下一阶段由现场先做只读观测和推理，再核对关节顺序/单位、夹爪实际开口、延迟预算与限值，逐步验收运动、到位、停止和失联行为；不宣称真机通讯已完全验收。


### 2026-10-12：首次实机推理准备——单步关节变化限值诊断

- 用户终端用 `kaanh_follower` 的远程推理入口，设置 `action_chunk=30`、`execute_horizon=30`、`max_publish_step=30` 和 `max_joint_step_rad=0.01`；两次都在 `A10InferenceRunner._execute_actions` 报 `Joint target step exceeds configured limit`。模型服务已连通、观测与推理阶段已返回；当前错误发生在 `start_follower()` 前，**这两次运行没有从该 runner 发送关节/夹爪运动目标**。不能从错误文本判断具体超限值，也不能据此推断设备已掉使能。Pydantic UnsupportedFieldAttributeWarning 与此次拒绝无因果关系。
- runner 现在对完整待执行块计算相邻绝对关节目标变化（第一步相对实时观测），只读推理直接输出最大值、度数、动作序号、关节序号和前后目标；超限错误也给出同样定位并说明没有发送运动目标。保留原有限值拦截，不自动放大、截断、平滑或发送部分动作块。
- 现场教程新增只读 30 步诊断命令。先取得真实最大变化与对应角度，再核对训练动作语义、关节顺序和现场允许的限值；仅有“0.01 超限”不足以推定适宜的新限值。没有再次连接真机；变更用假 operator 验证错误发生于 follower 启动前、只读提示可见。


### 2026-10-12：现场首个 30 步关节 follower 收发核查

- 用户报告模型推理后控制器打印 `waiting....`，肉眼未见明显机械臂位移。提供的工控机日志有一次 `follower_aj` 和完整 30 条 `jp --motor_pos=[六轴角度]` UDP 输入；控制器逐条打印解析后的弧度反馈，说明 9999 端口至少收到并解析本轮关节目标。`waiting....` 与最后几帧交错及之后出现，不能仅凭这几个字确认控制器具体内部等待条件。
- 对粘贴的 30 条目标做只读统计：各轴目标全程范围分别约 [0.084, 0.152, 0.097, 0.255, 0.124, 0.230] 度；控制器打印的实际关节范围约 [0.065, 0.089, 0.031, 0.155, 0.050, 0.140] 度。目标首尾最大差约 0.216 度，实际首尾最大差约 0.154 度。幅度很小，肉眼可能看不出来；这不是 UDP 未收到或角度单位整倍错误的证据。`max_joint_step_rad=0.01` 只是拦截大步目标，不缩放轨迹。
- 用户表示已手动 stop。随后本机仅运行 `scripts/a10_control.py --host 192.168.110.124 --read-only get` 查询：Manual、Enabled、Stopped、Axis、follower 未运行、无故障；实际六轴弧度约 [-2.089785, -0.656196, 2.261903, -0.677612, -1.110854, -0.132605]，与最后目标各轴误差约 0.004~0.077 度。该查询没有使能、停止或发运动指令。控制器 `get` 无夹爪反馈，USB 夹爪是否动作本次未证实。
- 结论：当前关节目标通信和微小幅度反馈得到现场证据；模型为何给出近似保持当前位置的动作仍未确定。继续核对当前两路画面、任务文本、实际起始姿态及原训练场景，并用只读 30 步诊断看预测变化。不要仅通过增大 `max_joint_step_rad` 试图制造明显位移；需要到位、速度、夹爪、失联等现场验收。


### 2026-10-12：现场画面与限值补充核查

- 用户更正，最新允许的 `max_joint_step_rad` 是 0.1 弧度（约 5.73 度），不是前两次拒绝运行时的 0.01 弧度。该参数只是拒绝超限目标的门槛，不缩放预测动作；成功一轮中的六轴目标各轴全程范围仍仅约 0.08~0.26 度，故改变此门槛不能使其明显移动。
- 停止跟随后只读打开当前配置的两路 USB 相机各取一帧：均输出 480×640×3 RGB。画面有水瓶、纸箱和金属件，未见黄色柠檬。当前推理配置和原 OpenPI `LeRobotA10DataConfig.default_prompt` 都是 `Reach the yellow lemon`；原训练配置 `prompt_from_task=False`。画面/任务不一致是模型近似保持姿态的可能原因，不能由一帧断定唯一原因。临时画面存 /tmp/a10_onsite_base_0_rgb.png、/tmp/a10_onsite_right_wrist_0_rgb.png，不入 Git。
- 后续先在只读模式对比真实训练场景和当前视觉输入、起始姿态、预测的 30 步目标；核对模型是否确实适合当前任务。更换提示词本身不等于模型已学会新物体任务，仍需训练数据/现场验证。没有再启动 follower 或发送运动目标。


### 2026-10-12：连续推理首块查询超时修复

- 用户设置 max_publish_step=3000，但粘贴日志显示 14:56:57 连接模型、14:57:01 在首个 execute_trajectory preflight 的 get 超时退出，尚未发送本块目标；随后网络关闭导致停止跟随未确认。3000 是上限，不能作为实际发送数量。
- 本次仅只读查询现场控制器：Manual、Enabled、Stopped、Axis、follower 未运行、无故障。20 次 get 延迟中位数约 7ms，最大 35.8ms，部分超过旧 15ms 默认预算。没有启动跟随或发送实机运动目标。
- A10Operator 查询预算改为 100ms，不再用 dt/2 截断；周期内 WS/USB 查询暂停发送时钟，完成后顺延后续行，保持故障/模式/掉使能检查，超时仍终止且不补发。发送总耗时增加，查询期间控制器保持上一个目标；不是严格连续 30Hz，实机跟随效果仍需验收。单步限值保持 0.1rad 的现场配置，USB 驱动自身 15ms 超时保持。
- 查询超时明确显示 get 和预算秒数；runner 异常打印 last_trajectory 的 sent/completed，区分设定步数和实际发送。失联后仍不自动接管新连接，停止失败需现场确认。
- 新增真实回环控制器延迟 40ms 的 30Hz 轨迹测试（完整发送且不突发补发），以及首块真超时零发送测试。本机 A10 回归 80/80 PASS；修复后的实机运动尚未执行。现场先按原命令运行 30 步再延长。


### 2026-10-12：夹爪 USB 换口后无法启动修复

- 16:07 日志在 run_setup 打开串口时报 FileNotFoundError，旧 by-path 为 USB 1.1，现场实际为 1.3；尚未进入模型控制循环。系统只检测到一个 CH340 串口 /dev/ttyUSB0。
- 两份原生 A10 推理配置改用 /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0，避免插口变化导致找不到夹爪。多台同型号且无唯一序列号转接器仍需现场明确映射。
- 已只读验证 ping 和夹爪反馈：servo_id=10，开口约 1.257mm，速度反馈 0；两份配置加载并解析到 /dev/ttyUSB0，当前两路相机配置路径均存在。未使能、启动跟随或发送运动目标；实机推理需重新运行验证。


### 2026-10-12：90 步后动作块边界查询超时

- 16:15 日志显示 9 轮 A10 cycle、累计发送 90 个动作，第 10 轮 preflight 查询 get 超过 0.100s，失败块 sent=0，不能理解为整场零发送。该日志不证明目标全部到位。模型每轮约 1.3s、发送 10 步约 0.37～0.41s，运行存在推理间隔。
- 超时关闭 WS 后，finally 的 stop_follower 因未连接而失败。随后只读检查实际仍为 Enabled、Running、Axis follower active、无故障，已提示用户在工控机 stop_follower；未代为发控制指令。100 次只读 get 中位数 6.78ms，最大 58.49ms，未复现超 100ms；原延迟原因尚未确定。
- 修复块开始/结束查询误用流内短预算：这两处使用普通 timeout_s（默认 2 秒）；流内仍使用 100ms 并保持超时终止。last_trajectory 增加 phase，便于区分 preflight/sending/final_feedback。没有自动重发或重新接管控制。
- 新增 150ms 延迟块边界成功、流内慢回复终止测试，保留真正 preflight 超时零发送验证；21 项本机集成测试全部通过。实际运动需现场停止旧跟随后重试；本次命令中的 0.5rad 与先前确认的 0.1rad 不同，不能通过放宽角度限值修复网络超时。


### 2026-10-12：A10 状态采集与动作发送解耦（无 ROS）

- 用户要求与 UR/Aloha 的流程对齐：持续取得状态、推理读取最新观测、动作按节奏发送，同时保持 A10 WebSocket/UDP/USB 协议。代码仍在 A10Operator 中；CLI 默认同步查询不受影响。
- 实机只读验证：控制 5999 已连接时第二条 5999 WebSocket 被拒绝；监视端口 5888 能并行回应 get。A10 推理配置因此用 5888 每 0.05 秒查询并缓存状态，5999 用于 follower 启停，UDP 9999 用于逐行关节目标。两端口同时查询 8 次通过，最新状态年龄约 73ms，未发送运动目标。
- get_frame 和轨迹检查使用带接收时间的最新状态；0.5 秒无新反馈、监视连接出错、故障或模式变化时停止发新目标。监视故障只关闭其连接，保留控制连接供原会话 stop_follower。旧 100ms 流内 get 限制在推理配置下不再用于每个动作块；独立控制台仍按原同步路径。
- 模拟回环 A10 测试 84/84 PASS，覆盖 150ms 状态 get 回复期间按节奏发送、状态过期拒绝发送；另加监视断开零发送且原控制连接能停止跟随的定向测试 1/1 PASS。实机只读完整观测也通过：qpos=(7,)，两路 RGB 均为 480×640×3，无控制器故障。实际运动、控制器长期双端口负载、相机/夹爪/关节反馈时序仍需现场验收，UDP 仍无到位 ACK。


### 2026-10-12：后台监视启动前仍查询控制端口的修复

- 17:23 日志在 connect 中第一次 get_state 超过 2 秒，后台监视尚未启动，本次未发送动作。随后只读分别查询 5999、5888 均通过、follower 未运行；未复现这次控制端口迟滞，不能断定控制器内部原因。
- connect 的首次状态读取改为先连接 5888 并获取真实反馈，再启动监视线程，不再要求 5999 的 get 成功才开始监视；启停控制原有确认逻辑保留。监视端口也无回复时仍明确失败，不伪造状态或自动重发运动。
- 新增独立回环测试：控制端 get 不回复、监视端正常，能建立观测，控制端零查询/零运动。25 项本机集成测试全部通过；真实运动需要用户重新运行确认。


### 2026-10-12：模型夹爪预测超量程处理

- 用户确认错误为 Gripper target exceeds configured range。使用现场配置只读 ping/反馈正常，开口约 84.710mm；随后只读预测复现范围错误，问题来自模型目标而非串口路径。
- A10InferenceRunner 后处理将有限模型夹爪目标限制到配置工作范围（默认 0～85mm），超量程日志显示原始最小/最大值与修正行数。保持机械上限、六轴目标、关节限值和 operator 直接控制目标的范围校验；NaN/Inf 仍拒绝。
- runner 13/13 测试通过，验证上下边界、六轴不变、非有限值和关节超限拒绝。实际设备只读 30 步推理通过，4 行夹爪目标受限，未启动 follower 或发送运动目标。真实抓取效果仍需现场运行。


### 2026-10-12：现场连续 300 步推理正常结束

- 用户报告执行完成，提供日志尾段显示 steps=180～300，每轮 predicted=30、selected/executed=20、dry_run=False；最终 run finished after 300 selected steps，cleanup completed，尾段未出现查询超时或停止失败。记录累计 300 步正常发送结束；完整早期轮次未包含在附件，不单独声称逐条到位或抓取成功。
- 尾段推理每轮约 1.317～1.323s，20 步发送约 0.684～0.686s，整轮约 2.02s；发送期间近似 30Hz，轮与轮之间仍存在约 1.3s 推理等待，尚未做异步推理衔接。
- 夹爪边界限制生效：85.002/85.049mm 等轻微超限预测限制到 85mm；显示的各轮首帧夹爪目标约 84.5～85mm，处于接近全开状态，不能据此判断已经完成夹紧或抓取。
- 用户尚未确认水瓶是否实际抓起；本次只记录日志验证，不再连接控制器或发实机指令。下一步需要现场反馈机械臂是否接近水瓶、夹爪是否闭合与物体是否被提起，再区分通信流程与模型任务效果。


### 2026-10-12：不限步数选项与来回拉动日志核查

- 用户要求取消 300 步上限。A10 runner 支持 max_publish_step=None，持续重新观测/推理/发送，直到 p+Enter、Ctrl+C 或错误；保留原正整数上限及全部故障/范围检查。未新增自动抓取成功判断。
- 用户报告同步运行来回拉动，并将单步允许变化设为 0.3rad（约17.19度）；该值是拒绝门槛，不调节跟踪。用户授权参考前次 300 步附件。尾段每轮20步发送约0.685s、推理约1.32s，存在停顿；第4轴轮首目标 -0.883→-0.872→-0.927rad 有方向变化，但轮首目标不足以重建块边界轨迹。同步仅等待发送流程结束，并非实际关节到位。
- 原日志缺少实际关节与上块末目标，不能据此认定跟踪误差过高。新增 observed_qpos、last_action、previous_last_action、tracking_error_at_observation_rad，使用已取的观测，不新增网络查询或改变动作。误差是下一次观测时实际角度减上块最后发送目标，不是实时全过程误差或到位证明。
- runner 14/14 测试通过，包括不限步数人工退出/停止清理与误差日志计算；CLI None 解析验证通过，未启动实机测试。


### 2026-10-12：18:03 现场来回拉动量化核查

- 用户提供包含 observed_qpos、first_action、last_action、previous_last_action 的 300 步尾段；11 个完整轮次（steps=100～300）。仅解析已有日志，没有连接或控制机器人。
- 第4轴在观测时相对上一块末目标的残差最高 0.297217rad（17.03度）。steps=120 时：上一块末目标 -1.568839、实际观测 -1.271622、新首目标 -1.278033、新末目标 -1.470822rad。新旧块目标边界向正方向跳 0.290806rad（16.66度），新块随后向负方向走 0.192788rad（11.05度）。多轮具有同类回跳。
- 结论：有明显观测残差和目标块边界反向跳变，支持用户反馈的拉动现象；不能单凭残差将责任归为底层控制器。观测在推理前采集，新目标在约1.32秒后发送；发送前的实际关节未记录，是否在推理间隔追上末目标尚未知。同步只等待发送，不等待实际到位。
- 0.3rad 是准许单步目标变化门槛（约17.19度），不是跟踪调节参数；当前首目标检查相对推理前观测，此例只差 -0.006411rad，与相对旧目标的0.290806rad回跳不同。后续需核对发送前实际状态与控制器停发目标后的保持/收尾行为，再选择到位等待或持续保持末目标；本次未自动改变控制逻辑。


### 2026-10-12：18:12 全部15轮300步日志复核

- 用户提供完整15轮，每轮20步，发送约0.685秒、推理约1.32秒，正常300步结束。第4轴观测残差在steps=80最高0.436069rad（24.985度）；新旧块边界目标回跳约0.429809rad（24.626度）。steps=60首末目标约 -0.7214→-1.1321rad（23.53度），steps=80观测约-0.6960、新首目标约-0.7023、末目标约-0.7074rad。现象与上一轮一致且更明显。
- 只读通过5888查询get_speed_limit：关节手动最大速度比例30%，末端手动线速度200、末端线速度上限6000、角速度上限720（单位依回复字段）。未修改速度或发运动。该配置是否作用于Axis follower尚未确认，不能归因或据此自动提速。
- 0.3rad检查不覆盖相对上块末目标的回跳；同时观测发生在推理前，缺少发送前实际角度，因此观测残差不代表控制器稳态误差。后续需补发送前反馈和验证保持末目标时的实际收敛，再确定块边界保护/到位等待/持续保持的处理。夹爪首目标仍接近85mm，不宣称抓取成功。本次未更改控制算法。


### 2026-10-15：主目录迁回——备份与资源归位

- 用户批准将已跑通的 integration 工作区迁回原目录；检查发现最新加速推理修改尚未提交，先备份，待用户确认已提交后以 `b98edd2` 为迁入基准。原目录现已切换到 `kaanh_follower`，完整保留 Git 历史。
- 原目录 19 个 VR 开发文件保存为 `backup/vr-before-consolidation-20261015` 分支提交 `e9e4b93`；原 `vr` 分支指针保持。附件 `robot.7z`、PDF 和 `configs/a10/site.local.yaml` 保留在原目录。
- 备份目录为 `/home/robot/VLA/.fluxvla_backups/20261015_consolidation`：含原修改 binary patch、原未跟踪文件压缩包、新推理修改快照、本地配置、资源核对清单及工作区信息。Git 标签 `backup/integration-before-consolidation-20261015` 指向 `cf2fb0b`，`backup/integration-latest-before-consolidation-20261015` 指向 `b98edd2`，`backup/follower-old-20261015` 指向 `f8e2985`，可恢复早期 detached 版本。
- 已核对约 13.4GB 模型文件及 metadata/统计文件一致、三个本机 CUDA 扩展一致；补齐 tokenizer，使用本地硬链接保留文件，无需重新下载。原数据集和基础权重保留。integration 推理日志归档至 `work_dirs/a10/archive/20261015_consolidation/a10_inference.log`。
- 现场说明的执行路径统一为 `/home/robot/VLA/FluxVLA`。现有 Conda `fluxvla` 的 editable 安装已经指向该目录，无需重装依赖。后续先进行本机回环和模型推理验证，通过后再清理三个旧工作区。


### 2026-10-15：主目录迁回——验证通过与旧工作区清理

- 迁回与操作路径说明提交为 `50eccb6`。在 `/home/robot/VLA/FluxVLA` 使用现有 `fluxvla` 环境验证：通信编解码/本机回环 39 项通过，相机/夹爪/控制台/轨迹集成模拟 63 项通过，runner 和 OpenPI 适配共 21 项发现、20 项通过、1 项真实模型测试默认跳过；随后单独启用该真实 GPU 测试并通过，合计 123 个不同测试通过。
- 真实权重验证加载 811 个参数、成功录制 Triton CUDA Graph；本地推理和实际序列化 ZMQ 收发均得到有限的 `(30,7)` 动作，固定噪声下结果一致。完整验证日志为 `work_dirs/a10/migration/20261015/gpu_zmq.log`，模拟设备测试日志为同目录 `device_tests.log`。GPU 单项用时约 72 秒，包含首次加载和图录制，不是稳定推理耗时。
- 配置验证确认 `a10_openpi_inference.py`、`a10_remote_inference.py` 解析正常，模型、统计和 tokenizer 路径存在，默认 dry_run=True、allow_motion=False；Python 实际导入 `/home/robot/VLA/FluxVLA/fluxvla/__init__.py`。全部验证为本机测试和合成观测，不连接控制器、相机或夹爪，不发送实机动作。
- 清理前再次确认三个旧工作区均无未提交代码，非缓存忽略文件全部保留且资源无指向旧目录的符号链接。通过 `git worktree remove` 正常移除 `FluxVLA_integration`、`FluxVLA_follower`、`FluxVLA_runner`，未使用强制删除；现在 `git worktree list` 只保留主目录 `kaanh_follower`。
- 保留 `kaanh_a10_runner`、`vr`、VR 备份分支和全部备份标签；`lerobot`、`openpi`、`A10_new`、`XLeRobot` 不在此次清理范围。原附件仍保留为未跟踪文件。备份目录内 `cleanup-results.json` 保存三次移除的成功结果。
- 后续模型服务和机器人客户端先执行 `cd /home/robot/VLA/FluxVLA`，再 `conda activate fluxvla`，其余启动参数沿用已验证的配置。旧目录里的终端需重新 cd，IDE 重新打开主目录；重新启动服务以保证加载当前代码。

复现本次单项真实 GPU/ZMQ 验证（不接设备）：

```bash
cd /home/robot/VLA/FluxVLA
conda activate fluxvla
PYTHONPATH=test/test_runners:. A10_TEST_REAL_MODEL=1 python -m unittest \
  test_a10_openpi_inference.A10OpenPITest.test_real_checkpoint_forward_and_service -q
```


### 2026-10-15：建立 VR 开发分支并迁入已有实现

- 按用户要求，从最新 `kaanh_follower`（`efcdb2f`）创建 `kaanh_follower_vr`；VR 后续开发统一在新分支，仍使用主目录，不新建独立工作区。本次工作对话统一提交一个中文 commit，此后沿用每次对话一次提交的约定。
- 来源为 `backup/vr-before-consolidation-20261015`（`e9e4b93`）。选择性迁入 `VROperator`、teleop 组装入口、VR 输入关闭修复、按键说明、VR 专项测试及文档；没有合入来源提交中的旧 OpenPI 服务／回放修改。当前相机、USB 夹爪、后台监视、关节轨迹发送和 Triton/CUDA Graph 推理实现继续使用最新基线。
- `VROperator` 复用 `A10Operator` 和独立 `GripperOperator`，保留末端局部增量累积、启动锚点相对米／xyzw 转换、握键启动、松手保持、参考重置、输入过期与跟踪误差检查、摇杆控制夹爪。A10Operator 只补充位姿接口注释，没有替换通信或轨迹逻辑。
- 配置增加 `vr_operator` 分节和站点模板；保留现有本地配置中的 `model.checkpoint_format` 字段及校验，使 VR 入口可加载当前 site.local.yaml。依赖继续使用 follower 已有的 `pyserial==3.5`，未放宽版本或重装环境。本地和示例配置解析、VR-only 构建及 CLI 预览通过。
- 回归结果：VR/A10/轨迹/相机/夹爪/控制台 124 项通过（其中 VR 专项 22 项）；共享项目 6 项通过，覆盖本机 HTTPS/WSS 模拟浏览器输入、数据写入和假服务闭环；推理 runner/适配 21 项发现、20 项通过、1 项真实 GPU 模型测试按默认跳过。合计 150 项通过、1 项跳过，日志在 `work_dirs/a10/vr_migration/20261015/`。本次没有启动真实头显服务、连接实机或发送运动指令；真实 GPU 路径沿用上一轮迁回时的验证结果。
- 当前功能迁入完成。入口仍是 `projects/a10/teleop.py`，输入和 mapper 仍在 `projects/a10/vr`；统一遥操作基类／runner 整理、真实 VR→Cartesian follower 与夹爪联调、录制链路接入与验收留待后续开发。使用说明见 `docs/a10_vr_operator.md`。


### 2026-10-15：VR 紧凑布局落地——独立工作区与原生入口

- 用户批准紧凑布局，并明确禁止改动／提交 `kaanh_follower` 中的未提交内容。因此仅新增 `/home/robot/VLA/FluxVLA_vr` 工作区检出 `kaanh_follower_vr`（原基线 `325b074`）。没有切换主目录、暂存其改动、合并或推进 follower 分支。主目录已有现场配置、控制台等内容由用户独立开发，本轮不纳入 VR 提交。
- 将 `projects/a10/vr` 整体迁到 `engines/operators/vr`，保留网页、样式、图片、来源 NOTICE 和 Apache/MIT 许可证；setup.py 的 package_data 指向新包。mapper 只引用底层 TRANSFORMS 注册器，避免包初始化循环。
- `vr_operator.py` 提供小型 `BaseTeleoperator` 和注册到 OPERATORS 的 `VROperator`，负责输入、映射、事件与反馈。已测试的 A10 控制状态机迁入 `vr/runtime.py` 的 `VRTeleopRuntime`；保留启动锚点转换、目标累积、松手保持、重置、过期及误差检查、启停和失败清理。未新增 runner 文件，未改 A10Operator、相机或夹爪驱动。
- 新增 `scripts/teleoperate.py` 和 `configs/a10/a10_vr_teleop.py`，支持 `--cfg-options`、`--preview`，默认 VR-only，不再使用旧项目的 YAML/配置加载器。现场单位、位姿限值和运动开关必须显式确认；不读取主目录 site.local.yaml，不自动使能、回零、重连或抢占 follower。
- USB 夹爪通过现有 GripperOperator 单独组装，由 runtime 管理连接；机器人实例不重复持有串口。默认未启用 USB 设备，配置后沿用 85mm 范围。这样松开机械臂握键时仍能用摇杆操作夹爪。
- 诊断 JSONL 已接入循环，记录原始输入、映射动作、反馈、成功写出的相对目标、错误及耗时，关闭时刷盘并释放线程。HUD 模块与 send_feedback 接口保留；完整录制状态机／录制 HUD 留待下一轮。旧 teleop 入口明确报出新命令并退出；旧录制 session 反向引用新 VR 包的轻量适配器，不保留重复实现。
- 验证：原 VR 专项 22 项通过；VR＋通信／轨迹／相机／夹爪／控制台＋新入口测试合计首次 132 项通过，再增一项原生输入门面到 runtime 的集成测试并通过，新入口专项最终 9 项通过；共享原生／工作流 6 项通过（含本机 HTTPS/WSS）；A10 推理 runner 14 项通过。合计 153 个不同测试通过。导入拦截测试禁止所有 fluxvla.projects 导入，新 CLI 预览仍通过；资源打包模式与网页／许可证存在性通过。未运行真实 GPU 权重，未接真实头显或机器人。
- 验证产物：VR 工作区 `work_dirs/a10/vr_native/20261015/`。系统已有编译扩展通过本地硬链接复用，不修改 Conda 的 editable 安装；测试与脚本优先导入本工作区代码。
- 使用：先进入 `/home/robot/VLA/FluxVLA_vr`，激活 `fluxvla`，再运行 `python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py --preview`。实际头显输入去掉 preview，并在配置中指定本机局域网 host_ip。详细说明见 `docs/a10_vr_operator.md`。
- 本轮仅在 VR 分支统一提交一次中文 commit；下一轮先验收真实 VR 输入及现场位姿方向，再接入分段录制、保存/重录/续录与 HUD。


### 2026-10-16：新增面向新手的工程结构说明

- 按用户审核通过的方案新增 `docs/project_structure_guide.md`，按“角色和流程 → 根目录 → 核心子目录 → 配置/工具/测试 → A10实例 → 修改速查”组织，使用中文说明、目录表和文字流程图。
- 说明覆盖框架自有目录及主要子目录，第三方模型深层实现按模型归组；区分根目录数据/权重文件与 Python 数据/模型代码，解释注册器、base 模板和运行产物，避免把目录名称误解为已经接通的功能。
- 对照实际代码确认新 VR 通过 scripts/teleoperate.py + 原生 Python 配置 + operators/vr/runtime.py 运行；推理经 A10InferenceRunner、OpenPI 观测/模型/动作适配和 A10Operator。标明 projects/a10 剩余历史录制及训练配置依赖、新 VR 诊断不等于可训练数据集、仿真 follower 尚需协议确认。
- README_zh-CN.md 增加入门链接，方便团队从首页找到。所有变更仅在 `/home/robot/VLA/FluxVLA_vr` 的 kaanh_follower_vr 分支完成；不改运行代码、现场配置或主目录开发内容。
- 验证采用本地文档链接存在性、目录覆盖清单及 git diff --check；本轮为说明文档变更，无需运行设备/GPU或软件功能测试。统一提交一个中文 commit。


### 2026-10-16：整理公司技术分享工程部分底稿

- 新增 `docs/a10_engineering_share_brief.md`，面向算法、控制、软件及应用等同事，按约 10～15 分钟、9 页 PPT 组织。每页分投屏要点和讲解参考，覆盖整合动机、目标闭环、平台/控制器分工、模块组织、VR 与模型推理流程、完成度、后续路线和协作边界。
- 复用已提交基线与既有验证记录，区分软件回归、链路运行和任务效果；历史短训练/数据规模、新 VR 输入日志、实机 300 步和 GPU 推理各注明适用范围。完整新录制、仿真 follower 与稳定抓取未写成已完成；主线正在开发的未提交 RTC/共享连接也未列为已验收成果。
- 附常见提问、PPT 制作取舍和证据备查，避免正文堆砌路径、命令与通信细节；中文 README 的工程结构入口旁增加分享底稿链接。
- 验证本地链接和 git diff --check；本轮仅编写文档，无设备连接、训练或推理任务。仅在 kaanh_follower_vr 统一提交一次中文 commit，主目录及其未提交开发内容保持由用户管理。


### 2026-10-16：分享底稿补充部署、采集及训练成本

- 根据用户补充要求调整 `a10_engineering_share_brief.md` 为 11 页、约 15～18 分钟，保留整体架构、流程、成果和规划，重点增加 Thor 自有设备兼容/编译/外设适配、真实 VR 输入问题与任务场景规范、训练保存/恢复与资源成本三部分。提供短时分享的合并建议。
- 只读核查 2026-10-08 成功日志 `smoke_train_ddp_retry.log` 与 `resume_train_mmap.log`：初始化至一步训练并保存日志跨度 4分45秒，恢复并追加一步至保存结束 11分42秒；优化器恢复阶段 5分28秒，保存阶段分别约 1分54秒、2分55秒。明确不是稳定每步训练速度，也不是正式训练总时长。
- 对实际文件核查：完整恢复 `.pt` 为 40,456,313,994 字节（40.46GB/37.68GiB），仅模型 `.safetensors` 为 14,466,989,776 字节（14.47GB/13.47GiB）。文件容量不作为内存峰值；正式 90000 checkpoint 训练硬件、总时长、峰值与功耗缺少完整记录，底稿单列待补充并已向用户询问。
- 采集部分区分已遇到的网络、映射、USB和输入间断问题与待建立的场景规范，不声称已经完成泛化实验。部署版本使用历史记录口径，不推断当前运行环境版本。更新仅为文档，未改动其他分支或运行代码；统一一次中文提交。


### 2026-10-16：补充正式训练来源，区分 OpenPI 与 FluxVLA 成本

- 用户确认已有部署模型在 OpenPI 上使用“GTX5090”训练，耗时约 16 小时、峰值内存约 12GB。本轮据此更新分享第 8 页、完成度表和证据备查；正文简写“5090 显卡”，保留原述和来源，不推断卡数或把“内存”擅自改成“显存”。
- 正式训练数据明确归属 OpenPI，FluxVLA 两步短训练仅作为另一次流程验证，其耗时和文件大小移至备查用途；不同配置、环境和资源指标不作框架性能对比。验证链接、11 页结构及 git diff --check，仅修改 VR 分支文档并统一一次中文提交。

### 2026-10-16：VR 参考目标迁移阶段 1、2（离线）

- 按用户确认的布局在 `kaanh_follower_vr` 开发，集中新增 `operators/vr/control.py`。
  已有 `kaanh_follower` 工作区及其未提交修改保持原样；未跟踪的 VisionLoop PDF 不纳入提交。
- 完成自定义 TCP `vr_vel` 与平台现有 VR/UDP 接口对照，依据控制器 `97feb31` 的只读源码。
  原生末端 UDP 内部平滑、缓存、乱序与断流行为、反馈实际采样时间仍列为阶段 3 的确认项，
  不以关节 `FollowerAjAbsolute2` 的实现推断末端行为。详细记录见
  [VR 控制算法迁移阶段 1、2](a10_vr_reference_migration.md)。
- 迁入锚点组合、参考位姿限速、跟踪冻结/故障和限幅数学原语；冻结计时按实际秒数换算，
  避免把原 500 Hz 的 50 周期直接用于 30 Hz。新增前端参考加速度适配可独立关闭，
  没有增加低通滤波，没有移植 500 Hz 电机反馈闭环、IK 或关节保护。
- 新增 `scripts/replay_vr_control.py`：内置示例、旧 JSONL 回放、明确的真实/合成反馈选择、
  逐帧输出与故障摘要。工具不加载模型/GPU，不构建硬件对象，不发送 UDP，缺少真实反馈时
  不冒充实测结果。既有 `runtime.py` 的真实发送路径尚未切换到新算法。
- 从原 C++ 源码在本机编译生成固定参考结果，保存提交与源码/头文件哈希；测试无需其他仓库
  或编译器，生成工具可以在持有匹配源码时复现。新增 11 项控制测试，既有 VR 测试增加
  松手序号检查，共 23 项。原 C++ 对照、速度/加速度边界、重复/旧包、超时、释放与故障保持均覆盖。
- 旧日志回放发现松手包编号未更新：修复 mapper 的释放编号，并允许旧日志同编号释放，
  避免误判成重复运动包。重复活动包仍不延长输入有效期。
- 内置 120 帧示例通过（90 tracking、30 inactive）；现有 VR-only 日志 4,375 帧合成跟随回放，
  492 tracking、3,883 inactive，无故障，其中 2,370 帧使用名义时间；不作为实机验收。
  产物集中于 `work_dirs/a10/vr/reference_migration/`，不修改源日志或提交运行产物。
- 本轮只完成阶段 1 的接口梳理与阶段 2 的离线实现；阶段 3 UDP 实机对照、阶段 4 录制联动待后续。

### 2026-10-16：阶段 3 前端接入、只读观察和模拟 UDP 验证

- 用户明确控制器只读、所有代码写在 FluxVLA，保持前后端分离。本轮在 `kaanh_follower_vr`
  接入新参考目标路径，没有修改 0.125 的文件、配置或运行进程，没有向实机发送运动/启停指令。
- `VRTeleopRuntime` 新增显式 `control_mode=reference` 和 `reference_options`，保留默认 delta
  作为基线；启动前校验锚点包与输入年龄，启动后丢弃旧样本，新目标经过参考控制和逐帧变化检查后
  使用原 A10Operator 发 UDP。mapper 原整形必须绕过，避免前端重复处理；不新增低通滤波。
- 松手/异常时请求一次 stop_follower，新增 `A10Operator.wait_stopped()` 读取新 get 回复确认
  follower inactive 与控制器 motion=Stopped；停止失败保留错误，不自动重试/使能/回零。
  活动中更换锚点要求先松手再接管，不能清零命令速度后继续。Stopped 仍只是控制器报告。
- 新增 `--observe`：强制只读，经监视端口读取真实反馈，计算候选目标，不启停 follower、不发
  UDP/夹爪目标，不打开相机或夹爪。新增 `--inspect-controller`：不启动 VR，只向监视口请求 get，
  成功或失败均可保存结构化报告。日志区分候选目标、实际写出目标、观察模式和停止确认。
- 只读实查 192.168.0.125：SSH 正常，仓库 kaanh_rtc/97feb31，kaanhbin PID 2120884；
  UDP 9998/9999、TCP 8080 在监听，WS 5888/5999 均未监听。监视连接返回 ConnectionRefusedError，
  实际未发送 get 业务请求。没有尝试启动/重启或改配置；检查报告与 SSH 证据集中保存在
  `work_dirs/a10/vr/reference_migration/controller_readonly.json`。
- 新增 16 项阶段 3 集成测试，覆盖本机 WS/UDP 完整链、只读无运动、握键/释放/换锚点、
  半启动失败、输入断开、慢反馈、停止超时及 CLI 观察预览。VR 测试合计 59 项，A10 operator
  既有 39 项；均使用本机回环或模拟设备，不是实机验收。
- 更新 [VR 迁移记录](a10_vr_reference_migration.md)、VR 使用说明、控制器审查文档和结构指南。
  阶段 3 的前端与模拟验证已完成；真实反馈、UDP 内部动态与实机运动对照仍待控制器 WS 可用后确认。
  未进入阶段 4 录制联动；本轮统一一次中文提交，用户 PDF 与 kaanh_follower 的修改保留原样。

### 2026-10-19：双 A10 新任务资源与配置盘点

- 按用户收窄后的范围新增 [双 A10 资源与配置盘点](a10_dual_arm_resources.md)，只整合已有资源、配置依据和缺失资料；不制定算法／技术路线，不宣称任务已验证可行。
- 只读检查用户附件 `知行.zip` 的文件清单及两本手册，整理适用型号、24V 接线信息、Modbus RTU、默认 115200／8N1／ID 1 与位置单位。明确实物型号、固件及保存参数尚未确认，原夹爪协议和 85mm 工作范围不可直接沿用。
- 核对主工作区已有数据集元数据：35 段、37,225 帧、30fps；区分八维原始 action、七维状态与单臂模型执行目标。确认主工作区已有 90000 权重、统计文件和分词器，VR 工作区仅有占位文件；没有复制、链接或修改这些资源。
- 整理两臂／两夹爪配置登记表与相机、安装、电源、计算设备、任务物料等公共资源清单；未知值保持待确认。结构入门说明增加入口。
- 验证采用本地 Markdown 链接存在性与 git diff --check；本轮只有文档，不运行设备或算法测试。仅在 kaanh_follower_vr 统一一次中文提交，用户压缩包／PDF 不纳入 Git，kaanh_follower 及其未提交修改不作处理。

### 2026-10-23：双 A10 控制台通信与最小运动验证

- 按用户缩小后的范围，在 `kaanh_follower_vr` 实现控制台双臂小步运动，暂不接 VR。
- A10Operator 保留旧单臂接口和报文；双臂扩展 SDK 工具累计增量，每臂独立状态、统一校验后
  按 ee 顺序发送两行321数据。发送不确定时禁止同会话重建零点；双臂停止确认 inactive + Stopped。
- 控制台新增 `jog 臂名 轴 数值 [另一臂 轴 数值]`，支持单侧保持/双侧运动/连续命令；平移毫米、
  旋转度，限制单次10毫米/5度。检查真实反馈、误差、到位；异常尝试停止，禁止自动重发或使能。
- 双臂配置更新为新控制器192.168.0.150、WS5867、UDP9998/9999，保持默认禁止运动；
  `--allow-motion` 显式开启控制权限。左右及GET分组/单位保留现场核对要求。
- 只读SSH复查时 kaanhbin 未运行、控制端口未监听，未取得实时GET，未启动服务、改XML或发送运动。
  与此前核查文档中的运行状态属于不同时间快照；不宣称实机已验收。
- 使用本机WS/UDP回环、独立理想工具积分接收端验证格式与状态逻辑；旧单臂测试一并回归。
  操作步骤和边界记录在 [现场说明](a10_onsite_guide.md)，精简命令加入 [速查](a10_command_cheatsheet.md)。
- 下一步由现场启动正确控制服务，核对实际反馈和物理左右，再主动进行小幅移动验收。
- 验证结果：`PYTHONPATH=.:test /home/robot/miniforge3/envs/fluxvla/bin/python -m unittest test_a10_operator test_a10_sdk_adapter test_a10_control`，101 项全部通过（约26秒）；`git diff --check` 通过。

### 2026-10-26：将 VR 与双臂开发整合回 kaanh_follower

- 目标工作目录统一为 `/home/robot/VLA/FluxVLA`，目标分支 `kaanh_follower`。
  起点 `542da0d`，合入 VR 分支 `d54727f`；共同祖先 `efcdb2f`，双方分别10/27个独有提交。
  采用保留双方历史的 merge，在隔离工作目录 `FluxVLA_merge_20261026` 验证后快进正式分支。
- 回退引用：`backup/kaanh_follower-before-vr-20261026`、
  `backup/kaanh_follower_vr-before-merge-20261026`。原 VR 分支及 `FluxVLA_vr` 目录保留，不删除。
- 人工整合5个文本冲突：operator、控制台、控制台测试、现场示例配置及现场说明。
  保留推理的共享连接、知行夹爪、RTC、关节重复发送和调试日志，同时保留VR、标定、双臂接口。
  去除自动合并产生的同名速度方法，统一使用支持共享连接及 expected_percent 的实现。
- 控制台兼容规则：无配置时继续默认 `.125` / 5999 / 共享服务；显式配置优先，CLI显式参数再覆盖；
  双臂配置默认直连，不自动连接本机单臂共享服务。显式双臂共享模式会在联网前拒绝，尚未验证其反馈路由。
  保留原生命令透传，同时将 jog 注册为内置命令；推理与VR不同时驱动同一机器人。
- 本地文件处理：复制原 VR `vr_calibration.local.json` 及 `.bak` 到统一目录（目标此前不存在）；
  两个目录证书不同，原 VR 证书独立复制到 `work_dirs/a10/vr/tls_vr`，VR配置默认指向该目录。
  主目录原 `work_dirs/a10/vr/tls` 及 `configs/a10/site.local.yaml` 不覆盖；私钥、标定和日志不纳入Git。
  `CLI.text` 作为本地命令文件复制到主目录，命令目录指向统一目录；原文件保留。
  用户压缩包、PDF、others目录、未跟踪的双臂检查文档及历史日志仍留在原VR目录，不删除或自动提交。
- 文档更新：命令速查、VR说明、现场说明和结构指南统一工作目录与分支；历史迁移记录中的旧路径保留作证据。
- 验证结果（均为离线/回环，未连接机器人）：
  - A10通信/控制台/共享连接/SDK/速度/集成：168项通过；另外新增4项合并默认参数兼容测试通过。
  - VR映射、控制、双臂、标定与入口：120项通过。
  - 推理/RTC/日志：40项中37项通过、3项按原条件跳过（外部OpenPI参考、本地模型包、显式选择的真实GPU模型测试）。
  - 知行夹爪与follower分析/回环：33项通过；A10数据集：11项通过。
  - 共376项不同用例，373项通过、3项跳过；受调整影响的控制台/VR入口另行回归56项通过，不重复计数。
  - 单臂/双臂VR `--preview`、双臂控制台帮助、AST同名方法检查及 `git diff --check` 通过。
- 隔离工作目录最初缺少未跟踪的本机CUDA扩展，导致导入失败；复用主目录已编译的3个扩展后通过，
  没有修改或重新安装依赖。临时扩展链接不纳入提交；最终主目录继续使用原有编译文件。
- 测试和预览没有使能、启停机器人或发送实机目标。代码合并不代替VR/双臂实机验收；双臂控制仍待现场确认。

### 2026-10-26：旧单臂 .125 与老夹爪 VR 链路

- 范围按用户现场设备收窄为旧单臂A10、192.168.0.125和老USB夹爪，不接知行驱动。
- 新增继承配置 `a10_vr_teleop_legacy_gripper.py`：复用原VR配置和老GripperOperator，
  明确串口、1000000波特率、ID10、100mm标定/85mm软件行程、舵机速度800和0.1秒超时。
  默认VR-only不联网、不打开串口；启用真实控制仍需显式参数，旧机械臂独立配置继续可用。
- 专用配置摇杆目标变化速度10mm/s，新增方向参数gripper_direction=±1；
  复用右手X轴开合、未接管机械臂时也可操作、按实测开口计算小步目标与范围限制。
- 补充输入新鲜度检查：包含本帧机械臂查询耗时，并在串口读完成后再次检查，过期目标不发送。
  日志增加gripper_command的反馈、目标、发送标记和IO耗时；串口异常沿原流程请求停止机械臂。
- 系统检测到旧USB串口；只读ping/反馈验证成功，连接约16.2ms、后续3次读取约2.3～2.4ms。
  未发送开合、使能、回零或掉使能指令；保留同步IO，尚不引入后台线程。
- 测试使用模拟串口执行真实旧协议，覆盖开合方向、回中、不按侧键、反向、上下限、
  过期及读取期间过期、VR-only/预览、故障停止、退出无额外开爪和IO日志。实机开合与抓取待现场确认。
- 更新命令速查及VR说明；用户已有未提交的CLI.txt保留，不纳入本次提交。
- 验证结果：新增7项老夹爪VR用例；`python -m unittest discover -s test -p test_vr*.py` 共127项通过；
  `test_vr_legacy_gripper/test_vr_native_entry/test_vr_operator/test_vr_reference_runtime/test_gripper_operator`
  定向验证共69项通过（与VR全量有重叠）；真实控制参数的 `--preview` 和 `git diff --check` 通过。

### 2026-10-26：显式配置 VR 前端角加速度

- 将原提交 `1c73d85` 的角加速度配置迁入正确的 `kaanh_follower` 分支。
- 在 `configs/a10/a10_vr_teleop.py` 的 `reference_options` 中显式设置
  `max_angular_accel_rad_s2=1.5`，并注明单位及原算法默认值0.96，方便现场调整旋转加速、减速和换向响应。
- 1.5为待实机验证的候选值；继承此项且未覆盖它的配置同步生效，命令行仍可覆盖。
- 验证基础配置及老夹爪配置读取结果均为1.5，参数可构建ReferenceConfig；未连接机器人。

### 2026-10-26：恢复 CLI 中的 VR 启动记录

- 当前 `kaanh_follower` 的 `CLI.txt` 仅保留推理命令；从暂存记录 `e908e94` 中找回原单臂及双臂 VR 命令，追加到文件末尾。
- 恢复旧单臂 `.125` + 老 USB 夹爪的完整启动参数（旋转速度0.6rad/s、平移速度0.08m/s），以及双臂VR-only与配置预览入口。
- 原有推理启动内容完整保留；命令语法及配置路径离线核对，未运行真实控制。

### 2026-10-29：为双臂控制器增加独立 WebSocket 入口

- 只读确认 `.150` 的5867曾被 `.232` 示教器连接，且没有监听新连接；旧 `.125` 的A10_new工程额外配置了5888、5999。
- 按用户要求修改 `.150` 的 `/home/kaanh/Desktop/kaanh-project/kaanh.xml`，只新增
  `tcp_interface:5888`、`get_interface:5999`，保留5867、HTTP8001和双臂模型/运动配置。
- 原文件备份为同目录 `kaanh.xml.before_fluxvla_websocket_20261029_143440_931321.bak`。
  写入前核对原文件哈希，使用原子替换，保留权限及属主；新XML解析通过，移除新增节点后与原XML结构一致。
- 本地双臂配置将命令及同步查询入口均改为5999，保持单WebSocket连接和原UDP端口；5888暂不启用后台轮询。
- 修改时远端控制进程已未运行，未启动/重启服务或发送运动指令。现场须启动该目录的kaanhbin加载新配置，
  再只读GET验证新入口、双臂映射和单位；增加入口不提供多客户端运动仲裁。
- 经用户审核方案后再次只读复核：备份和修改文件哈希一致，XML仅增加上述两个接口；
  控制程序仍未运行，5888/5999均拒绝TCP连接，因此未执行真实GET，不宣称新端口已上线。
- 验证双臂operator与双臂VR继承配置均能加载且使用5999单连接，默认禁止运动，UDP端口和双臂顺序保持原配置；`git diff --check`通过。

### 2026-10-29：隔离开发目录并接通双臂 VR SDK 发送

- 用户反馈 `.150` 双臂基础控制台实机验证通过，下一步接入双手 VR。此次没有重新连接机器人，
  不将基础控制台验证视为双臂 VR 实机验收。
- 从 `628d268` 为 `kaanh_follower` 创建独立 worktree：`/home/robot/VLA/FluxVLA_follower`。
  原 `/home/robot/VLA/FluxVLA` 保持 `console` 分支；不修改或提交该目录的未提交工作。
  两目录可并行改代码，实际使用时 `.150:5999` 控制入口及本机8442/8443网页端口仍不能重复占用。
- 复用现有 `A10Operator.begin_sdk_tool_trajectory()` / `send_sdk_tool_pose()`，
  双臂 runtime 接收两臂完整计划位姿，经 SDK Tool 累计适配后按配置顺序发送两行6项 pe321。
  每次全组启动仅初始化一次；单侧松手减速保持、重新接管只更新该侧 VR 锚点，
  不重置累计值、不重启另一侧。先校验两臂目标再整包发送，保留断流、故障和停止未确认保护。
- 双臂配置默认仍为 VR-only；实机显式启用后连接 `.150:5999`、UDP9998、相对锚点和SDK累计模式。
  双臂试验参数独立设置：线/角速度0.04m/s、0.20rad/s，加速度0.4m/s²、0.96rad/s²，
  超前范围20mm、0.12rad。原单臂配置和 `legacy_anchor` 兼容路径不变。
- 不新增机器人基类，不改 `A10Operator`、BaseOperator或控制台；不增加双臂标定、录制、
  夹爪、模型推理、控制器IK或碰撞避障。诊断保留每臂累计通道和计划位姿，
  本机发送成功不标记为送达或执行确认。
- 新目录首次导入缺少被Git忽略的本机CUDA扩展；复用主目录已编译的3个扩展和原VR TLS证书，
  不安装依赖、不复制权重或标定、不将扩展/证书/私钥纳入提交。
- 验证（全部离线或本机回环，未发送实机指令）：
  - VR全量146项通过，包含新增19项SDK双臂相关用例：单侧释放、迟接管/再接管、
    初始化与发送失败、过期/缺失反馈、停止失败锁定、只读模式、诊断和真实WS/UDP回环。
  - A10 operator、SDK适配与控制台117项通过；加上VR共263项通过，重叠的定向测试不另计。
  - SDK回环核对40字节包头、type='321'、两行6项和反向配置的行顺序；按SDK几何模型积分
    累计报文后与计划位姿一致（容差1e-5），不等同于验证真实SDK滤波、断流或物理制动。
  - 双臂默认/只读/实机参数预览，以及原单臂老夹爪SDK参数预览通过；`git diff --check`通过。
- 在 `CLI.txt`、命令速查和VR说明中补充新目录、最短启动命令、端口占用和现场验证边界。
  下一步由现场人员确认左右臂对应关系、单侧接管/释放和双侧同步运动；本次没有修改控制器文件，
  没有使能、启停 follower 或执行实机运动。

### 2026-10-29：双臂现场左右绑定与松手停止超时排查

- 用户运行双臂VR后反馈手柄与实体臂左右相反；仅在 `a10_dual_arm_vr_teleop.py` 对调
  `hand_to_arm` 为左手→operator `right`（ee 1）、右手→operator `left`（ee 0）。
  原operator分组名、GET解析、UDP行序、单臂配置和控制台均不改，避免重复交换协议数据。
- 只读分析本地 `work_dirs/a10/vr/dual_teleop_diagnostics.jsonl` 最新会话2463帧，对照用户
  16:41:33的 `release_settle_timeout`：错误分组为原 `left`/ee 0，松手过渡约0.23秒后
  前端命令线/角速度均为零，最终位置残差2.9367mm、姿态残差0.4446°。
  最后一秒30帧的反馈位置与计划位置各自保持不变；2mm停止到位条件未满足，持续超过2秒退出。
  最终停止状态为 requested=true、mode_inactive=true、reported_stopped=true。
- 直接触发原因是松手停止到位检查超时，不是WebSocket断连，也没有报告控制器故障。
  已收到新鲜反馈不代表内部滤波/SDK叠加完全一致，当前日志不能确定2.94mm残差的底层原因。
  本次不更改停止算法、2mm容差或2秒预算，不以延长等待或取消保护掩盖残差。
- 增加使用现场绑定的SDK双臂用例，验证左手路由至ee 1、右手至ee 0，松开一侧后另一侧
  继续运动且不重启全组。通用生命周期用例显式使用其原同名测试绑定；现场绑定单独验证。
  更新CLI、速查及VR说明中的编号和故障解释；所有修改仅在 `FluxVLA_follower/kaanh_follower`。
- 验证：双臂专项41项通过，VR全量147项通过（包含专项，不重复计数）；实机参数配置预览和
  `git diff --check`通过。本轮没有连接、使能或驱动机器人，未改控制器配置。

### 2026-10-29：修正左手到 ee 1 的前后/上下平移映射

- 只读检查VR日志最新会话（读取时5233帧），确认右手→原left分组/ee 0、左手→原right分组/ee 1
  已生效，两个分组的手柄与末端锚点朝向不同，但代码仍为两手套用相同局部轴映射。
  日志不能直接标注操作者所指的房间“前方”；用户现场补充确认左手上抬时ee 1向前、向前时向上。
- `VROperator` 增加 `mapping_by_hand`，在公共mapping上按手覆盖参数；未知手名、参数名及
  会绕开双臂握键/safe_frame_delta约定的覆盖在启动前拒绝。
- mapper新增可选 `position_axis_remap`，统一用于锚点平移和逐帧平移；默认继续使用原axis_remap。
  平移矩阵单独校验3x3、有限、正交，可交换两个轴；不将这种交换矩阵用于旋转向量。
  新日志记录实际的position_axis_remap与rotation_axis_remap，便于区别平移和旋转配置。
- 仅双臂配置的左手使用 `[[0,0,-1],[1,0,0],[0,1,0]]`，等价于交换原工具局部平移X/Z输出。
  手柄绑定、右手平移、两手旋转、单臂配置、UDP协议和停止保护保持原样。启动命令无需新增参数，
  重启程序后加载此配置；这仍是相对锚点方案，不新增固定操作坐标系或双臂标定。
- 增加方向用例验证左手前/上交换、侧向不变、右手平移及两手旋转一致性，并验证无效覆盖在连接前拒绝。
  变更只在 `FluxVLA_follower/kaanh_follower`，未连接或驱动实机，未修改控制器文件。
- 验证：双臂专项43项、VR全量149项通过（包含专项，不重复计数）；双臂实机参数的配置预览及
  `git diff --check`通过。实机的最终方向仍待用户重启后验证。

### 2026-10-29：17:31左手方向实机复核，工具轴交换未解决空间串轴

- 用户反馈应用0003599后左手仍然“前推变上抬、上抬变前推”。只读分析本地最新完整会话
  3754帧（文件更新时间17:31:40），确认诊断中左手position_axis_remap为新矩阵、右手为原矩阵，
  SDK累计模式及左手→ee 1绑定均已生效，排除“仍运行旧矩阵”的解释。
- 最新会话第3420～3441帧（左手锚点编号anchor_id=45）：手柄相对VR锚点位移
  [-4.604,60.488,1.985]mm，mapper局部位移[32.203,7.740,43.441]mm。
  用实际手柄锚点朝向和新矩阵重算，与记录的局部位移一致；转换后的用户基坐标目标位移为
  [51.741,16.735,-5.182]mm，经reference限速的计划位移为[22.827,7.353,-1.992]mm，
  反馈位移为[16.518,5.189,0.849]mm。后两项相对该次锚点建立时的反馈位置，不能当作到位误差。
  上抬主要变为X方向，在前端目标中已经发生；反馈主要沿相同X方向变化。
- 本次修正的坐标系假设问题：当前位移链为 R_robot_anchor × A_position × inverse(R_hand_anchor) × VR世界位移。
  0003599只交换A中的工具局部X/Z，并没有建立固定的VR操作空间到每臂基坐标的对应。
  末端及握姿在两次实机间变化后，旧会话末次接管的VR向上单位向量映射为
  [0.9900,0.1314,-0.0509]，新会话末次接管为[0.9502,0.2995,-0.0868]，仅相差约10.12°，
  所以工具通道虽变，现场“上抬主要前进”的感受仍然相似。此前离线用例只验证单位握姿下
  的局部交换，不足以证明实际握姿下的空间方向已正确，此次实机结果推翻了该修正已解决问题的假设。
- SSH只读核查 `.150`：运行的是kaanh-project下kaanhbin.original；XML笛卡尔组仍为tg 2、
  ee_ids={0,1}，两模型的ground/wobj0/tool0均为各自单位位姿，没有手柄操作空间标定。
  17:28～17:32的SYSLOG记录使能及多次stop follower，未见解释此次串轴的故障事件。
  log目录为空，未取得逐包UDP接收/电机高频历史；不能声称已核对控制器逐包执行，
  方向判断依据前端计划及其记录的控制器GET反馈。未修改远端文件或发送控制指令。
- 建议下一步：平移采用固定、单独定义的操作坐标转换，接管时只重记位置锚点；旋转保留当前
  相对算法。先确认每臂基坐标与操作空间方向，再设置固定矩阵，避免继续猜工具局部轴。
  本轮只更新分析记录，不修改映射代码、参数、停止保护或控制器配置。

### 2026-10-29：18:20提速试验再次触发松手停止到位超时

- 用户以控制器倍率30%、reference线/角速度0.08m/s和0.6rad/s、角加速度1.5rad/s²运行双臂VR。
  只读检查最新会话（1090条记录，含倍率恢复事件），第1089帧为release_settle_timeout。
  出错分组left实际是ee 0、右手柄；ee 1/左手分组当时为holding，与此前左手串轴问题分开处理。
- ee 0第1030帧进入releasing，第1036帧（约0.232秒）计划线/角速度均降为零；超过2秒时
  仍有4.3067mm位置残差、2.4005°姿态残差。二者分别超过2mm及0.02rad（约1.146°）
  的松手到位容差；最后一秒反馈位置不变，最后半秒15帧反馈姿态也不变，计划位置未再变化。
  末帧关节反馈与控制器目标角最大差约0.0013°，不能直接把上述末端残差解释为电机追踪很差。
- controller has_error=false、follower仍为Cart，失败清理后确认mode_inactive=true、
  reported_stopped=true，倍率恢复事件标记restored。30%倍率初次设置已确认，本次并非倍率设置失败。
- 当前step_released把“命令减速归零、反馈低速稳定、目标精确到位”绑定为holding条件；
  固定末端残差无法消除时，到位条件始终不满足，反复触发2秒超时。提速增大了旋转滞后，
  但延长等待不能保证消除已稳定的位置/姿态残差；SDK累计几何、下游处理与反馈语义之间的
  残差来源仍未完全确认，不据此断言控制器滤波或UDP丢包。
- 建议后续方案（未实施）：区分停止与到位；双手释放、前端减速完成后请求全组stop并回读
  inactive+Stopped；单侧释放保持独立减速/固定目标，以反馈稳定与经现场验证的有界残差判断，
  保留大误差、输入/反馈过期及停止未确认的全组保护，不使用无限等待或取消故障检查。
- 本轮仅分析和记录，未修改运行代码或参数，未连接控制器、发送运动或启停指令；
  用户修改的CLI.txt保留，不纳入此次提交。文档`git diff --check`通过。


### 2026-11-02：统一双臂松手流程与单侧连续接管

- 工作目录 `FluxVLA_follower`，分支 `kaanh_follower`；只修改前端和离线测试。
  不操作控制器，不修改速度、轴映射和 UDP 报文，不提交用户的 `CLI.txt` 改动。
- 最新实机日志第4160帧旧逻辑 `release_settle_timeout`：ID0（右手）命令速度约0.2秒归零，
  停止前残差约7.50mm / 3.27°，最后半秒TCP反馈稳定；停止已确认。该记录用于复现问题，
  不代表新流程已经实机验证，也不能据此确定SDK内部处理或UDP丢包。
- 双手新鲜释放后直接走单臂的全组stop + inactive/Stopped确认，不再发UDP追赶旧终点；
  停止未确认仍锁定会话，重新启动以实测位姿初始化累计适配器。
- 单侧松手继续限加速度减速、固定累计目标，另一侧可继续。保持判定改为命令速度为零及
  TCP反馈差分稳定持续0.1秒；取消2mm/0.02rad精确到位前置条件，不修改硬误差上限
  0.05m/0.35rad、反馈时效或2秒未稳定超时。稳定不是底层确认停止，残差仍记录。
- 单侧重新接管记录实际锚点，并固定保留上一计划与实际之间的位置/姿态偏差，确保新段目标
  连续、手柄静止时不会自动回退或补偿。SDK累计状态不重置；偏差不逐帧积分，不宣称修复
  终点偏差。`tracking_target`记录连续衔接目标，`user_target`保留原始锚点目标。
- 诊断增加 `command_stopped`、`feedback_quiet`、`release_stable_s`，供后续现场区分
  命令停止和反馈稳定。旧release位置/姿态容差字段仅为配置兼容保留。
- 新增/调整离线回归：带残差两手释放当帧不再发目标、原生停止失败锁定、单侧稳定残差
  超过2秒可保持、重新接管不跳累计值且另一臂不重置、持续抖动仍超时、过大误差仍停组。
  同时覆盖legacy及SDK两种适配器，已有单臂停止与输入/反馈异常用例继续保留。
- 验证：定向控制/单臂/双臂测试86项通过；最终完整 `test_vr*.py` 回归155项通过
  （36.129秒）。本轮文件 `git diff --check` 通过。仅使用模拟反馈和本机回环，未连接实机；
  现场下一步验证双手松开、单手保持及再次握住的行为，原启动命令无需修改。


### 2026-11-02：撤回左手专用平移轴交换，恢复公共映射基线

- 用户完成A/B只读方向测试后，授权先撤掉左手额外矩阵再做实机验证。
- `a10_dual_arm_vr_teleop.py` 删除左手 `position_axis_remap` 覆盖项，两手共同使用
  `[[0,1,0],[1,0,0],[0,0,-1]]`。保留左手→ID1、右手→ID0绑定；速度、松手行为、
  旋转映射及SDK报文均不修改。通用按手覆盖功能仍保留。
- 更新已有方向回归用例和使用说明/速查表，明确这是撤回未验证的修正尝试，不能声称现场方向已修好。
  原启动命令重启后生效，不修改或提交用户的 `CLI.txt`，不连接控制器或发送运动。
- 验证：双臂输入、配置、运行状态及本机协议回环共49项测试通过（3.047秒）；本轮文件差异检查通过。现场验证待用户执行。

### 2026-11-02：接入 .150 控制器高频周期记录命令

- 依据SSH只读核对的 `runtime_recorder/使用说明.txt` 和 `sdk_adapter.cpp`：5999沿用
  ARIS/WebSocket，提供record_start、record_status、record_heartbeat；无record_stop。
  默认5秒应用消息超时，GET可续期；断线才结束记录，多次机械臂启停不切分会话。
- 在 `kaanh_follower` 增加operator公开接口，使用现有事务锁/连接/报文封装；验证记录器
  返回结构及开始状态，不引入运动权限或运动命令。拒绝共享代理/后台第二连接的录制组合。
- 遥操作入口增加 `--record-controller`，仅显式开启；支持 `--observe` 无运动记录和preview。
  runtime在连接后、倍率或运动之前确认开始，每秒同连接心跳并监督状态，空闲和跟随停止
  仍记录；记录中断/错误/会话改变/通信失败结束遥操作并使用既有停止流程，不自动重连续录。
  dropped增加告警，保留缺口。心跳超时小于等于2秒的服务端配置拒绝启动。
- 本机JSONL关联远端session/file、状态、last_synced_cycle、收发墙钟/单调时钟；每帧含
  本机时间戳。强调不同主机单调时钟不可直接比较，此版不承诺跨机4ms精确对齐。
- 关闭控制连接结束记录，本机只标记disconnected_save_unconfirmed，不冒称后台已完成落盘；
  正常停止与故障处理先于关闭连接。文档补充5秒无运动检查及原实机命令增加开关的方法。
- 新增模拟/回环测试覆盖同连接并发GET/记录命令不串包、ACK及字段检查、未确认开始不运动、
  多次跟随启停同会话、空闲心跳、录制中断停机、无重连、只读记录、preview无网络。
- 本次仅修改FluxVLA和本机回环验证，未发送远端record_start、未重启或控制机械臂；控制器
  运行进程是否加载14:56部署的新库、真实250Hz/PDO数据及最终落盘需现场确认。
- 验证完成：新增记录器测试9项通过；完整VR回归164项通过（37.333秒），A10通信回归59项通过（14.894秒），合计223项。相关文件差异检查通过；用户CLI.txt改动不纳入提交。

### 2026-11-05：迁移单臂录制核心并验证 OpenPI 数据兼容性

- 在 `kaanh_follower` 实施本轮批准的录制核心范围。新增 `datasets/episode_writer.py`，
  复用旧采集的字段约定，不依赖 `projects/a10`；双臂/VR实时录制接线暂不在本轮。
- 保留30Hz、7维实际状态（六关节rad+夹爪mm）、8维旧VR动作、两路RGB、任务及索引；
  新增配置 `configs/a10/a10_record.py`。拒绝14维状态、缺图、非有限值等非法帧；模拟数据有独立标记。
- 后台有界视频队列，满队列/编码失败使本集失效，不能静默保存缺帧数据。
  支持开始、保存、丢弃、续录；临时集+校验提交清单+互斥锁处理保存中断。
  恢复仅支持本录制器契约一致的数据，未提交集归档，已开始提交的集按校验继续完成。
- 新增 `a10_recording_io.py`、校验及导出脚本。原始保持v3，另导出本机OpenPI实际使用的v2.1；
  base_0_rgb/right_wrist_0_rgb别名为top/right；按元数据拆开共享视频和Parquet，保留8维原始动作。
  输出视频H.264；已有AV1视频导出会重编码，不宣称像素逐位相同。目标目录必须全新，源数据不修改。
- 原始动作不是UDP位姿，也不直接用作7维训练标签；实际验证原OpenPI使用状态轨迹生成30步动作，
  关节增量+夹爪绝对值，再补齐模型32维。模型norm_stats必须在OpenPI重新计算。
- 本机OpenPI已有行为：A10 Repack未转发prompt，模型使用default_prompt；记录任务已完整保留，
  多任务训练接线后续单独处理。未修改OpenPI仓库或训练权重。
- 验证：11项新增离线测试通过；旧数据35集/37225帧通过数值及元数据校验（未全量解码旧视频）；
  2集共80帧模拟数据通过真实LeRobot v3和OpenPI v2.1 reader，含集边界、相机与30步窗口检查；
  OpenPI原统计加载器10批计算及模型输入转换通过。未启动训练/控制器/机械臂，也未做现场录制验证。
- 新增 `docs/a10_recording_core.md` 说明API、命令、恢复和后续工作；依赖列入requirements-a10.txt。
  本轮不修改用户CLI.txt，不触碰console工作目录。下一步接入现场采样循环、VR录制事件和数据时效检查。

### 2026-11-05：登记现场双知行夹爪及外部 D405 设备绑定

- 用户逐只发送小范围位置指令，并现场确认 FTDI `DU0EMMMM` 对应左手柄所控臂的夹爪，
  CP2102N `5c9323064b6eef11a2c4e7c2c169b110` 对应右手柄所控臂的夹爪。
  当前节点分别为ttyUSB0/ttyUSB1，均为115200/8N1、独立串口上的Modbus ID 1。
- 新增 `configs/a10/a10_devices.py`，按手柄/物理机械臂命名left/right，使用稳定by-id路径；
  与operator历史内部键分开说明：左手对应ee1/key right，右手对应ee0/key left。
  记录小范围验证参数speed=5%、torque=60%，不宣称完整行程或raw毫米标定完成。
- 用户确认D405为独立外部场景相机。序列号255323070551，彩色流video-index4（当前video4），
  连续读取41帧，640×480 RGB、实测约29.99Hz。仅配置base_0_rgb，不虚构第二路腕部相机。
- 本轮此前左夹爪500目标测试出现B=0x1000；依据本机厂家手册是工件掉落报警，触发原因尚未确认。
  用户清除报警后发位置0，日志反馈438→84→16→4→3、到位、报警A/B均0；后续现场确认左右归属。
  右侧按用户现场观察记录能动作，不追加未经提供日志确认的到位精度或满行程结论。
- 常用命令速查新增设备表、完整路径状态命令和小幅验证参数；设备清单尚未自动接入双臂VR，
  下一步仍需知行夹爪遥操作接线及录制单位/视角约定。本次记录不更改现有运行配置或发送实机指令。

### 2026-11-05：更正相机安装位置，确认左右腕 D405 的设备 ID

- 用户更正此前现场描述：未接入外部相机，已接线的是两台腕部相机；撤回上一条记录中
  将 D405 标为外部场景相机的判断，夹爪左右绑定不受影响。
- 最初 USB 设备树只枚举序列号 255323070551，video0..5 均属于该同一物理设备。
  用户单独拔下右腕 USB 后，该序列号和全部视频节点消失，确认其为右腕。
- 左腕单独换接到右腕已验证的接口/连接链路后，识别出序列号 255323070485，确认其为左腕。
  配置登记 left_wrist_0_rgb / right_wrist_0_rgb，使用各自稳定 by-id 的 video-index4 彩色节点，
  撤除错误的 base_0_rgb 绑定，不复制同一相机作为两路。
- 右腕此前连续读取41帧、约29.99Hz；断电重启后左腕连续读取41帧、约30.00Hz，
  均为640×480 uint8 RGB。左腕本次报告及图像位于
  work_dirs/a10/hardware_checks/20261105_164809/，未发送机器人或夹爪控制指令。
- 用户表示两台重接完毕后，本机 USB 列表仍仅枚举左腕。尚不能确认具体是数据线、接口、
  扩展坞还是供电问题；已请求右腕换接已验证接口/线材。双路同时读取暂未验收，
  不应通过修改采集程序或重复使用同一设备路径掩盖未枚举问题。
- 设备清单和命令速查同步更正；仅登记设备，不自动启用双臂 VR 夹爪或双臂录制。

### 2026-11-05：右腕换接恢复，双路腕部相机同时采集验证通过

- 用户换接右腕后，USB新枚举255323070551（路径2-3.1），与左腕255323070485
  （路径2-3.2.1）同时存在，均为5000M；当前彩色节点分别为video10/video4。
- 使用已有CameraOperator同时保持两路打开，分别读取95帧，均为640×480 uint8 RGB；
  实测成对读取约30.00Hz，单轮最大读取约33.95ms。此为短时并行开流验证，不代表长时稳定性或硬件曝光同步。
- JSON报告和两张预览图保存在work_dirs/a10/hardware_checks/20261105_164809/，
  报告名dual_wrist_cameras.json。配置和命令速查同步更新，使用稳定by-id而非video编号。
- 用户同时表示外部相机已接入，但本轮USB列表/视频节点仍只有上述两台D405，
  未发现第三台；已询问外部相机型号和连接方式，暂不填入猜测的base_0_rgb设备。
- 本轮仅读取相机，未操作机械臂或夹爪，未修改用户CLI.txt。

### 2026-11-05：确认外部 D435i，完成三路 RGB 同时读取验证

- 用户重新接入三台相机后，系统识别外部相机为D435i（用户口述D435），序列号262923065199，
  USB路径2-3.2.2、5000M；两台腕部D405序列号及左右归属不变，均以5000M连接。
- 查询V4L2格式和udev属性确认：外部彩色接口为USB interface 03、YUYV（当前video16）；
  深度为video12、红外为video14，不能拿来替代RGB。该D435i不同接口的by-id index0重名，
  实际链接当前指向深度节点，因此base_0_rgb使用区分接口的by-path，具体路径见a10_devices.py。
  该路径绑定当前USB位置，换插口必须重新核对序列号、彩色接口及路径；未修改系统udev规则。
- 使用已有CameraOperator同时保持三路打开，各读95帧，均为640×480 uint8 RGB；
  实测成组读取29.99Hz、单轮最大约34.83ms。仅验证短时RGB，不代表深度/IMU、硬件同步或长时稳定性。
- 报告three_cameras.json与三路预览图保存在work_dirs/a10/hardware_checks/20261105_164809/。
  设备清单、命令速查同步更新；不更改旧单臂推理配置，不宣称双臂录制流程已接线。
- 未连接控制器或发送机械臂/夹爪指令，未修改用户CLI.txt。

### 2026-11-05：新增统一USB设备识别及配置路径更新命令

- 新增scripts/a10_devices.py，默认仅使用Linux sysfs、稳定设备链接和V4L2格式查询，
  不依赖RealSense SDK，不打开视频流或夹爪串口；输出相机/串口转接器序列号、节点和配置匹配结果。
- a10_devices.py补充identity信息，绑定用途、型号、序列号、USB接口及相机video index。
  相机按序列号+接口+index匹配，明确排除深度或不可读节点；检测到by-id跨接口重名时选择by-path，
  外部D435i换插口后可重新发现彩色接口。型号为展示信息，不以节点编号推断物理左右。
- --check-grippers可选打开已匹配串口，仅使用原ZXGripper.read_state查询配置的Modbus ID；
  不扫描/改地址，不发送位置、使能、报警复位或掉使能命令。现场两只ID1均响应，raw=0、无报警。
- --update仅更新唯一身份匹配设备的device/port字面量；原字节备份后原子替换，
  保留注释、速度、力矩、归属及其他配置。没有可更新路径时不写文件；配置被其他程序改动时拒绝覆盖。
  缺失/重复身份/同一节点重复绑定/不可用接口不更新该项；新设备列出但不自动分配。
- 区分USB转接器序列号与夹爪Modbus ID；USB口变更可重新匹配，但夹爪与转接器间接线互换需人工确认。
  当前FTDI已是ttyUSB1、CP2102N是ttyUSB0，现有by-id绑定仍准确，说明不应依赖ttyUSB编号。
- 常用命令速查增加扫描、更新、只读夹爪查询、备份及返回码说明。未修改用户CLI.txt，
  仅在kaanh_follower开发，不更改console工作区或机械臂控制流程。
- 验证完成：新增13项测试、现有相机6项测试共19项通过；覆盖序列号变更拒配、深度/彩色区分、
  重复身份、重复节点、错误旧路径、只读Modbus仅0x03、默认不打开串口/不更新配置、UTF-8注释及备份保留、
  并发编辑拒绝覆盖。真实设备五项匹配通过；临时配置副本模拟外部相机旧路径后恢复成功、备份一致；
  真实配置--update无改动不生成备份。没有要求现场再次拔线验证，故不宣称已完成新插口实测。

### 2026-11-05：增加设备照片与左右知行夹爪辨认动作

- 在原scripts/a10_devices.py中扩展--snapshot，复用CameraOperator短时读取三路RGB，
  每路生成一张原始JPEG，并生成带中文名称、用途键和序列号的overview.jpg及manifest.json。
  默认单次独立目录、可指定新输出目录，拒绝覆盖；不启动机器人控制器、不接入VR或训练采集循环。
- 夹爪新增--gripper left/right --position-raw 0/1000；仅打开选中侧串口，
  先核对已匹配身份/路径再读报警、实际位置，使用原ZXGripper发送及等待。
  限制本次检查区间、speed<=5%、torque<=60%，实际位置不在区间附近拒绝动作；
  未确认开合方向，check.open_raw/close_raw保持None，--open/--close在串口打开前拒绝。
  填入各侧不同的0/1000并确认方向后，同一入口可执行命名开合检查，不当成全行程标定。
- 报警不自动复位、动作失败不自动重发、退出不掉使能/张开；10秒等待超时不代表设备停下。
  拍照/夹爪动作与配置更新、全夹爪查询互斥，避免检查命令混发；其他设备离线不阻止选中夹爪辨认。
- 本机实际执行三路拍照通过：640×480原图和1920×566中文拼图已生成并检查，位于
  work_dirs/a10/hardware_checks/20261105_164809/device_images/。保留现场画面方向，不代替曝光同步或数据验收。
- 新增11项测试，累计设备检查24项通过：覆盖左右端口隔离、报警/越界/未确认方向拒发、
  超时不重试和不释放、图片RGB颜色/三路输出/标签及清单、已有目录不覆盖、冲突参数无硬件访问。
  夹爪动作通过模拟Modbus验证，实物辨认动作由用户自行执行，本次未发送真实夹爪或机械臂运动指令。
- Pillow列入requirements-a10.txt（当前环境已有，无需本次安装）；命令速查和设备清单同步更新。
  本轮只在kaanh_follower改动，用户CLI.txt保留，不纳入提交。

### 2026-11-06：夹爪任意原始位置控制及工作量程确认

- 用户已将设备清单改成min_raw/max_raw，但检查脚本仍读取positions_raw导致动作前拒绝。
  本轮统一改成读取各侧check.min_raw/max_raw，保留用户当前0..3000初始测试范围；
  增加tolerance_raw=5、move_timeout_s=10。目标参数接收范围内任意整数，驱动上限与实际位置检查同步读取配置。
- 设备检查入口保留speed<=5%、torque<=60%，只操作选中侧。继续复用ZXGripper的Modbus位置发送，
  用显式回读确认两次位置到达状态及误差容差；遇阻或超时作为未确认到位返回1，不自动重发/复位/掉使能。
  输出初始/目标/实际raw、误差、耗时、outcome及arrived。原ZXGripper供其他场景使用的完成策略不修改。
- 新增--gripper left/right --calibrate，整个会话只打开该侧串口：输入数字才运动；
  open/close在成功移动后再次回读记录端点；range MIN MAX明确调整会话内测试范围，不运动、不落盘，
  并要求覆盖当前实际位置；save展示端点和方向，再显式确认后写配置。
- 保存按两端目标排序生成min/max，保留open/close方向并记录序列号、Modbus ID、实际反馈、时间，
  只替换所选check，复用字节备份和原子替换；并发修改拒绝覆盖。端点间距须大于两倍容差，
  无到位记录、遇阻/超时、取消或输入结束均不保存。校准数据是人工确认的工作范围，不是自动搜索机械硬限位。
- --open/--close读取保存端点；校准元数据与当前设备序列号/Modbus ID不同则拒绝复用。
  后续VR可按close_raw + ratio*(open_raw-close_raw)映射，0闭合/1张开；本轮不自动接VR或宣称毫米标定完成。
- 验证：设备检查测试41项通过（原24项按新格式调整，新增17项），覆盖任意值/边界、不同左右范围、
  非零下限、配置容差、旧到位位但实际未到位、遇阻不算到位、正反方向、显式扩范围、备份仅改选中侧、
  取消/EOF/重复端点/并发编辑拒绝保存和校准身份绑定。真实设备仅默认扫描，五个设备均匹配，
  配置解析通过；本次没有发送真实运动目标。真实量程及方向由用户现场逐侧确认。
- 命令速查补充完整交互用法，保留右腕180度旋转及现有设备绑定；仅提交本轮相关文件，用户CLI.txt不纳入。

### 2026-11-06：修复当前位置超出旧范围时无法进入校准

- 用户反馈实际raw=9000、配置仍为0..3000，--calibrate在提示符前被实际位置检查拦截，无法输入range或记录端点。
- 校准入口改为读取身份匹配设备的状态并检查报警，但允许只读显示旧范围外位置；明确提示先用range覆盖实际位置。
  普通位置运动、端点记录和保存仍使用原有范围检查。校准内当前位置不符时拒发目标并保留提示符，
  让用户继续核对范围，不自动回拉夹爪、扩大配置或保存端点。
- 用户报告目标10000实际停在9000，仅作为现场观察记录；本次不自动认定机械极限、不更改两侧初始上限。
  可由用户显式range 0 9000后验证9000及另一端，按实物开合方向记录，再预览并确认保存。
- 新增4项模拟回归：范围外进入/退出零运动写入、范围未覆盖时拒发但保持会话、显式覆盖后可保存9000、
  范围外入口仍拒绝报警。设备检查测试共45项通过。本轮未访问真实夹爪或发送任何实机动作。
- 命令速查同步说明恢复步骤；用户CLI.txt保持独立，不纳入本轮提交。

### 2026-11-06：双臂VR接入独立知行夹爪通道

- 用户确认两侧工作范围raw 0..9000，0完全张开、9000完全闭合；设备清单同步登记open/close及范围，
  保留各自USB转接器、Modbus ID1、115200波特率、speed5%、torque60%及右腕相机180度旋转。
  该确认不等于raw毫米标定，不复用旧夹爪标定表或推理毫米字段。
- 双臂VR配置增加zx_grippers，默认关闭；直接继承a10_devices.py的设备信息及端点。
  开启方式是原启动命令增加teleop.zx_grippers.enabled=True；VR-only、observe和preview不打开知行串口。
  原单臂老夹爪配置与控制路径保留，未改BaseOperator或机械臂UDP接口。
- VROperator双手模式保留各手柄摇杆X，独立于侧键；按物理left/right路由到USB通道，
  避开机械臂历史内部左右反向键。正X向张开端运动，负X向闭合端运动；两侧direction可分别反转。
- DualVRTeleopRuntime内新增私有ZX通道类，每个串口一个后台所有者；前台只发布最新输入、读缓存，
  不持有串口I/O锁。启动仅读取实际位置作初始目标，摇杆回中解锁；回中或断流停止更新，
  断流恢复必须重新回中。默认目标变化1800raw/s、死区0.1、目标领先实际最多300raw、串口循环20Hz、
  输入有效期0.25秒及反馈读取超时0.5秒；目标始终受工作范围限制，不积压历史目标。
- ZXGripper新增可选USB序列号/接口核对，在打开串口前验证；新通道显式使用设备清单max_raw=9000，
  不依赖旧独立驱动默认8900。新增可选运动截止时间和取消事件，在运动寄存器写入/触发前再次检查；
  过期不继续准备/触发或重试，已触发目标不能自动撤销。旧调用不传这些参数时行为保持一致。
- 一侧串口报警、断连或反馈超时触发会话异常；先取消两侧未来夹爪发送，再走原机械臂停止确认。
  停机时每侧由本线程关闭串口，不自动张开、复位或掉使能；部分启动失败也会清理已打开的另一侧。
  诊断gripper_command按物理手柄保存实际/目标/最近发送raw、反馈时刻、输入期限、报警和线程阶段。
- 验证：新增双臂知行测试18项通过，含左右方向/反转、边界与超前限制、独立路由、零启动运动、
  输入过期/回中、串口阻塞前台不阻塞、身份不符拒开、截止时间在报警查询期间过期拒发、
  参数写入后取消不触发、部分启动失败清理、机械臂跟随期间夹爪故障请求组停止、只读预览不打开串口。
  现有VR回归164项、知行驱动27项、设备检查45项通过，合计254项；开启夹爪的实机配置preview通过。
  本轮全部为模拟串口/合成反馈/本机回环及构造预览，未访问真实控制器或发送真实夹爪/机械臂运动目标。
- 命令速查和VR开发文档同步记录运行方式、参数与现场验收步骤。用户CLI.txt保持独立，不纳入提交。

### 2026-11-06：诊断现场夹爪低速及机械臂跟随启动状态

- 用户11:26:41至11:28:03实机会话完成左右夹爪操作，但机械臂接管报Enabled/Stopped前置条件不满足。
  只读get查询0.150三次，当前均为Manual、activate=Disabled、motion=Stopped、无故障、follower inactive。
  该查询是故障后当前状态；原VR反馈日志未包含robot_msg，不能将当前查询冒充故障瞬间完整原始回复。
  不发送使能、停止、坐标设置或任何运动命令。启动异常信息增加实际activate/motion，便于后续精确定位。
- 分析此次真实会话2422帧，排除早先模拟测试条目：持续大幅摇杆区间中位实际/目标变化速度约为
  左侧901/901raw/s、右侧1229/1234raw/s，低于目标速率1800；对应串口活跃周期中位约0.176/0.134秒。
  目标领先实际最大约224/258raw，未触达300raw限制；前台step中位约10.3ms、95分位14.6ms。
- 代码原因：每轮在串口I/O外额外等待完整周期，且积分dt上限写为2/io_hz=0.1秒，
  使0.13..0.18秒的真实周期被少计。现调度扣除I/O耗时，积分使用真实间隔且受输入有效期封顶，
  保留工作范围、目标领先限制、输入截止时间和取消检查。默认电机speed5%及目标速率1800不变。
- 速度配置位置与可用target_rate_raw_s覆盖命令写入速查；电机speed的5%软件保护不放宽。
  明确allow_motion不等于使能，并补充操作者手动使能和状态查询命令。
- 修复新增VR夹爪测试使用真实diagnostics_path的问题：合成反馈现在只写临时目录并核对诊断字段，
  保留现有历史日志，不删除或混淆已记录的真实会话。后续诊断仍按会话时间/类型区分已有条目。
- 验证包括真实周期0.18秒积分回归、目标超前保护、临时诊断输出，以及未使能/未停止时明确报错且仅发get。
  本轮代码及模拟测试不替代提速后的实机验收；用户CLI.txt改动不纳入提交。
- 本轮验证通过：VR知行夹爪测试19项、A10通信/协议回归60项，共79项。实机仅执行只读get，
  没有替用户使能机械臂、重启跟随或发送夹爪目标；计时修正后的实际手感待用户复验。

### 2026-11-06：VR知行夹爪电机速度允许配置为1～15%

- 按用户确认，将VR私有ZX通道的speed允许范围从整数1～5%改为整数1～15%，
  报错同步明确百分比及整数要求；0、16及以上、小数、布尔值和字符串均在连接设备前拒绝。
- 设备清单默认speed5%、torque60%保留。VR通过teleop.zx_grippers.devices.left/right.speed分别覆盖；
  设备检查/校准程序仍限制speed<=5%，无需修改共用清单即可单独提高VR电机速度。
  目标速率、领先限制、工作范围、回中及断流保护沿用原实现。
- 新增3项测试：1/5/10/15接受且不打开串口、越界及类型错误拒绝且力矩上限仍生效、
  后台线程将配置的15%传给模拟驱动。VR知行22项及设备检查45项共67项测试通过。
  左10%/右15%的实机配置仅preview验证通过，未连接真实设备或下发运动。
- 命令速查补充完整10%调速命令、1～15%允许范围和目标速度与实际速度的区别。
  用户CLI.txt原有改动保持独立，不纳入本轮提交。提速后的实际开合手感由用户现场验证。

### 2026-11-06：登记用户实机确认的双臂VR夹爪默认参数

- 用户确认两侧电机speed=15%、target_rate_raw_s=3000、max_lead_raw=600使用效果良好，
  按要求将该组合写入a10_dual_arm_vr_teleop.py，后续启动无需重复四项命令覆盖。
  该记录来自用户实机反馈，范围为夹爪开合体验，不扩大为抓取任务或数据录制验收。
- VR从共用设备清单复制两侧身份、串口、量程及力矩，并分别覆盖speed=15%；
  a10_devices.py自身speed=5%不变，设备检查/校准继续使用低速。
  保留VR-only及夹爪默认关闭开关，运行时需显式选择实机和开启夹爪。
- 使用现有配置测试核对两侧15%、目标速率3000、领先量600及检查清单5%；
  VR知行22项和设备检查45项共67项通过，不带速度覆盖的实机配置preview通过。
  本轮未连接真实设备或发送动作。命令速查及VR说明已同步，用户CLI.txt未修改或纳入提交。


### 2026-11-06：整合分支迁回FluxVLA主目录

- 核对同事的89c2563合并提交：kaanh_follower已完整包含console，无需再次合并。
  沿用合并后的既有VR别名：内部left=ID0、right=ID1，手柄绑定反向对应物理左右；
  关节初始点按ID0前6轴、ID1后6轴保存，未再次改名或交换目标。
- 主目录切换前为干净的console/d1c1070；旧FluxVLA_follower只有用户CLI.txt未提交。
  双方CLI、原始补丁、HEAD和工作树列表备份至
  /home/robot/VLA/FluxVLA_migration_backups/20261106_150312。
  旧工作树在同一89c2563提交解除分支占用，保留为detached HEAD备份；
  /home/robot/VLA/FluxVLA现使用kaanh_follower，console分支及FluxVLA_vr工作树保留。
- 将用户旧工作树CLI.txt原样带到主目录并保持未提交，未纳入本轮文档提交。
  旧工作树49个主目录缺少的运行文件复制到对应work_dirs并逐个SHA256校验，包括VR历史日志和相机检查图；
  两个同内容TLS文件不重复覆盖，一份重名的旧teleop_diagnostics.jsonl保留主目录版本。
  文件清单在备份runtime_files.json中，源目录全部保留。主目录现有模型、数据和本地配置未覆盖。
- 主目录验证：VR知行、双臂VR、控制器高频记录、设备检查共125项模拟/本机回环测试通过；
  实机VR配置preview通过，确认模块从主目录导入，m_init构造的两组角度与用户12轴目标一致。
  没有连接实机或下发动作。未扩展为对全部console/推理功能的完整回归或实机验收。
- 命令速查5处旧工作树入口已更新为主目录。迁回时主目录已有控制台进程，未替用户关闭或重启；
  该进程继续使用原已加载对象，后续使用新配置需由用户退出后重新启动。


### 2026-11-06：清理冗余FluxVLA工作树

- 用户要求检查其余工作树，冗余的直接移除。核对FluxVLA_follower/89c2563及
  FluxVLA_vr/kaanh_follower_vr/d54727f均为当前kaanh_follower的祖先，无独有未合并提交。
  follower未提交CLI与主目录逐字节一致；VR没有跟踪文件修改，但存在独有未跟踪附件、文档及历史日志。
- 主目录代码/配置和fluxvla环境未发现指向两旧目录的硬编码运行依赖或资源软链接。
  旧目录仅有两个空闲终端停留，无机器人程序运行；未关闭用户终端或任何控制程序。
- 清理前备份至/home/robot/VLA/FluxVLA_migration_backups/retire_worktrees_20261106_150938：
  每个工作树保存HEAD、状态、补丁和逐文件清单；与主目录相同的文件逐个SHA256核对，
  独有资料按相对路径写入tar.gz并重新读取校验。VR归档32项约123MB，含协议附件、知行资料包、
  控制器检查文档、标定及历史诊断资料；follower归档1份重名空日志。可重建Python字节码缓存不归档。
- 二次确认未提交状态/文件清单未变化后，通过git worktree remove移除FluxVLA_follower及FluxVLA_vr。
  当前只保留/home/robot/VLA/FluxVLA工作树及kaanh_follower分支；旧分支和备份分支均未删除。
  主目录CLI哈希清理前后相同，保持未提交；现有模型、数据、运行日志和本地配置未删除。
- 本次为工作树及文档清理，无业务代码改动。移除后主目录实机配置preview通过，
  从仓库外验证fluxvla仍从主目录导入。未连接设备或下发动作；命令速查更新统一入口及旧终端切换提示。


### 2026-11-06：双臂数据格式、保存与标准LeRobot导出

- 按用户调整后的范围，仅在kaanh_follower/FluxVLA实现数据格式和工具，不修改OpenPI仓库，不接实时VR采集，不访问硬件；用户CLI.txt保持原样且不纳入提交。
- 新增a10_recording_schema.py集中定义旧单臂和a10_dual_v1。旧单臂接口/合同保持不变；双臂为物理左(ID1)6关节+夹爪、物理右(ID0)6关节+夹爪，关节rad，夹爪实际raw/9000、0张开1闭合。
- pack_dual_state显式按ee_id打包实测值，拒绝缺失/非法数据。双臂action为同帧state副本且强制校验，不冒充控制命令、不做时间前移或模型增量转换；合同记录动作来源、单位、物理左右和ID、相机、设备身份及模拟标记。
- 扩展EpisodeWriter和a10_recording_io支持双臂三路相机，复用保存/丢弃/恢复/导出，不引入projects/a10依赖。新增a10_dual_arm_record.py继承设备身份和画面旋转；不把易变USB路径纳入续录合同。
- v3导出v2.1保留标准目录/索引/任务/统计，图片top/left/right。导出视频数按相机数计算，数值原样保留，源数据不变，目标不得覆盖。单臂与双臂格式不能混录；双臂导出保留源合同和诊断。
- 18项单元测试通过（旧单臂11、双臂7）。2集各40帧、三路16×16模拟数据在标准LeRobot v3/v2.1读取器通过，检查14维、三相机、任务、集首中末及30步窗口不跨集；集尾按读取器重复末帧。
- 本机仅读取已安装的LeRobot依赖做格式验证，没有导入OpenPI policy或运行训练；不宣称双臂OpenPI训练链路已打通。交接文档为docs/a10_dual_dataset_handoff.md，训练端后续适配14维/三图、夹爪单位、关节增量掩码、任务prompt并重算归一化统计。
- 验证数据在/tmp，带SIMULATED_DATA.json，不用于训练；640×480持续吞吐、设备时间对齐和实机录制待下一阶段。现有遥操作运行方式不变，控制器诊断记录仍不等于训练数据集。
- 补充兼容性验收：旧实测a10_vr_9_21_4共35集37225帧通过数值/元数据校验（跳过视频解码），双臂终端校验和导出入口通过模拟验证。


### 2026-11-06：实时双臂采集、右手功能键与头显状态

- 在kaanh_follower实现用户确认的“右手A+左摇杆”方案，未修改用户CLI.txt和OpenPI，未访问实机。
- 新增operators/vr/recording.py集中协调机器人/夹爪缓存、三路后台图像、录制状态、异步开始/保存/丢弃、备用终端和HUD。反馈按实际ee_id打包；逐设备时间/相机序号、下发目标和VR原始输入进入每集诊断，主VR JSONL新增dataset_recording状态。
- 新增BufferedCameraOperator，独立线程持有每台相机、短缓存最近画面和主机接收时间；复用原相机旋转及设备身份检查。原同步CameraOperator接口保留，机器人/夹爪不增加连接。
- 模式进出需左摇杆回中，方向边沿触发、斜拨忽略，模式内屏蔽左夹爪新输入。丢弃/复位二次确认默认8秒，断流/输入序号重启清除确认。网页修正xr-standard摇杆按钮索引，A/B采用4/5并暴露原始按钮供现场核对，头显补录制状态/任务/健康/错误和断线提示。
- m_init复用现有实现，仅增加外部cancel_event检查。复位先确认跟随停止、独占机器人发送、最后确认停止；断流/向下/stop/退出可取消，取消或失败锁住本会话运动接管。复位不自动使能/张开夹爪，当前集必须先处理。后台复位与主线程机器人异常清理避免并发下发。
- teleoperate新增--record-config、--record-options、--check-record-buttons，预览不做IO；按钮检查强制VR-only，不连接机器人/夹爪/相机。终端只是备用入口，与手柄共用状态机。相机/磁盘故障使本集失效不发布，保存不阻塞运动；停止采样不拼接暂停前后的片段。
- 实测发现原逐像素float64图像统计使三路640×480/30Hz合成写入在80帧约2.7秒队列满。改用现有OpenCV通道统计，增加数值一致性测试；150帧复测保存成功，约5.15秒，append P99约0.76ms。不扩大缓冲掩盖吞吐问题。
- 最终112项录制/VR/夹爪/相机/控制器记录测试及19项数据保存导出测试通过，共131项。JS语法检查通过。旧test_a10_init_motion的11项有3项在修改前HEAD和修改后均失败（旧初始角/已删除的速度加速度参数），8项通过；新增外部取消、复位互斥测试通过，不宣称旧套件全通过。
- 合成数据仅/tmp，未进行三台USB相机联采、实体A键/头显可读性、实机m_init或任务录制验收。下一步先运行只读按钮检查，再由用户短集现场验收；具体步骤见docs/a10_vr_recording.md。


### 2026-11-06：沿用旧HUD改进录制提示与A键方向菜单

- 根据用户现场确认与最新按钮日志，在现有recording.py、vr_app.js和index.html内改进显示，不改录制指令状态机、运动参数、数据格式或用户CLI.txt。
- 状态标题单独着色，白色正文、深色背景；隐藏常驻调试字段，突出确认/异常。右手A按下展开四方向菜单，根据待机、录制、停止采样、保存、复位和失效状态显示真实可用操作；松开收起，回中门槛和确认立即提示。
- 常规HUD仍2Hz；状态/输入门槛变化下一次控制循环推送。保存成功停留约1.5秒，显示刚保存的集号，非阻塞、不自动续集；VR断线清理菜单。
- 新增5项Python行为测试和Node真实渲染检查；相关111项Python回归及JS语法检查通过。没有访问控制器或执行设备运动，头显实测清晰度待用户刷新页面确认。使用说明同步docs/a10_vr_recording.md。


### 2026-11-09：双臂实采数据目录调整

- 按用户要求将默认保存路径改为datasets/a10_dual_record，校验、导出、交接及录制指南同步。原work_dirs/a10/datasets/a10_dual_record在确认未被写入器占用、目标目录不存在后迁入；已有0集0帧初始化元数据保留，继续使用需record.resume=True。
- datasets已有Git忽略规则；不提交设备数据或用户CLI.txt。配置预览验证不连接设备。

### 2026-11-09：采集状态头显下的VR输入延迟诊断

- 在kaanh_follower增加可选`--measure-vr-latency`，复用现有WebSocket、输入快照和头显；默认关闭，不改运动/录制参数，不修改用户CLI.txt。
- 浏览器给原VR数据包附加序号、单调发送时间、发送积压、页面/XR可见性及左右跟踪状态；同一WebSocket每秒一次应用层RTT探测。探测单独分流，不进入ControlGoal队列或刷新输入新鲜度。重连分配新连接ID并清理浏览器计时器，旧网页仍兼容。
- 服务端记录接收回调时间，快照携带该标记直到控制循环；区分首次读取和重复读取旧帧。复用后台JSONL写入器，每次运行独立目录`work_dirs/a10/vr/latency/`，正常退出输出中位数/P95/P99/最大值、100/250ms次数、丢诊断数及运行模式。分位数最多覆盖每项最近100000样本，累计次数/最大值覆盖整次运行。
- HUD新增每秒一次的RTT/接收数据年龄/队列行，无近期回包时明确显示等待。RTT包含端点调度，首版不跨设备直接相减、不输出单程估计，不把序号缺口解释为UDP丢包。
- 新增3项Python诊断测试（时钟独立、重复读取/重连、探测隔离及快照标记、旧客户端兼容），和Node浏览器计时/HUD/清理检查；114项Python录制、双臂、夹爪、入口、控制器记录回归通过，原HUD Node测试通过。增强快照字段传递后的3项测试再次通过；只读按钮模式加诊断的配置预览通过，未启动服务或连接设备。
- 使用步骤同步录制指南与命令速查：先VR+HUD只读对照，再由现场操作者在真实采集负载下录一小段静止数据。尚未进行头显/局域网现场测量，本次没有访问控制器或发送运动指令。

### 2026-11-09：双臂VR输入异常暂停及显式重新接管

- 按审核方案在kaanh_follower实现：已识别的手柄位置突跳、输入过期、追踪丢失、无效采样/输入序号重启等输入保护进入暂停；丢弃异常目标、整组停止，确认inactive + Stopped后保留VR和录制入口。控制器/反馈/通信/夹爪故障、停止未确认和未知映射错误继续走原故障退出流程。只读观察不声称已停止实机。
- 双手释放侧键/前扳机、两个摇杆回中，持续收到新鲜且跟踪有效、无位置突跳的数据至少0.5秒才进入ready。重复帧不能完成检查，异常/按住接管键会重置等待；ready后新一次侧键按下才能解除锁定，使用当前手柄与机器人反馈重新建锚点。既有2m/s输入阈值和运动参数保留，双臂配置显式增加input_recovery_stable_s=.5（0.5～5秒）。
- 沿用现有头显渲染增加红色INPUT PAUSED及黄色INPUT STABLE状态，提示松手/回中/重新按侧键；主VR JSONL新增input_pause状态、异常手柄/机械臂、停止确认、稳定时长及输入突跳速度。普通遥操作没有录制入口时也能显示提示。
- 知行夹爪暂停新目标，独立串口取消标志阻止尚未执行的旧请求；USB状态读取继续。暂停期间的合成零输入不会解除锁定，恢复后需要真实新鲜回中输入再拨动。不自动张开或掉使能，也不把已经执行的有限位置目标声称为物理急停。
- 正在录制或后台正在开始一集时遇到输入暂停，本集失效、停止采样，禁止正常保存；允许用既有二次确认丢弃，暂停期间禁止新集和m_init。恢复后仍需处理异常集再开始新集，避免拼接时间空洞。
- 127项Python离线回归通过，包含真实mapper位置突跳、旧输入/追踪丢失、持续握持不恢复、冻结帧/触发器/摇杆门槛、再次异常、停止失败、控制器故障、实际反馈重建锚点、只读语义、夹爪取消及异步开始录制的异常；原HUD与新增暂停/恢复Node渲染检查及JS语法检查通过。用户当前完整参数加--preview验证通过，未连接设备或发送运动指令。使用说明更新docs/a10_vr_recording.md第9节；实机手感与头显提示待现场验证。
- 本次只提交上述VR配置、运行/录制/HUD、测试和说明；保留工作区中既有CLI、operator/共享通信、m_init及其他文档的未提交修改。

### 2026-11-09：0.150控制器记录模块适配SDK 2.5.10.4（已构建，待退出旧进程后切换）

- 按用户审核的最小修改方案，在0.150的`/home/kaanh/Desktop/kaanh-project`复用现有`startup_login`与`runtime_recorder`，保留`sudo ./kaanhbin`入口；不新建另一套记录协议或启动命令。当前直接运行的新版ELF没有加载旧记录库，导致`record_start`返回-40。
- 先备份当前二进制、旧库、构建/初始化/适配源码、相关测试和说明及XML，目录为`runtime_recorder/backup/sdk_2_5_10_4_20261109_161200`，附逐文件SHA256。新版实际ELF保留为`kaanhbin.sdk-2.5.10.4`，SHA256为`b9a7eb58549a5d259ece9c3db503336efb2da14b8ec79efbe6aa459b51c31ba3`；旧`kaanhbin.original`仍为旧SDK版本，不作为本次启动程序。
- `startup_login/CMakeLists.txt`增加SDK版本参数，当前为2.5.10.4；新构建目录`build/auto-login-sdk-2.5.10.4`。核对编译头文件和运行依赖全部指向新版SDK，`ldd -r`未报告缺失依赖/未定义符号。适配器日志元数据中的SDK版本改由编译配置提供，不再写死2.5.10.3。
- 初始化钩子的旧自动登录改为显式`KAANH_STARTUP_AUTO_LOGIN=1`才执行；本次待部署入口设置为0，仅恢复周期记录。保留旧能力的独立测试，并新增默认关闭登录的测试。
- 新库18项CTest全部通过（无跳过），含核心采样/心跳/溢出/分卷/恢复/错误处理、真实SDK临时端口WebSocket指令、断线保存/重连、原初始化转调及可选登录。测试只使用合成控制对象和临时端口，未访问真实总线或下发运动指令。
- 待部署库`startup_login/libkaanh_startup_login.so.sdk-2.5.10.4.next`及同名入口脚本`kaanhbin.sdk-2.5.10.4.launcher.next`已准备，脚本语法检查通过。库SHA256：`26d425ba1aaa4dd00d444a7ae212629fac7205206c5b9eaca8978809ef2f61f5`。远端状态记录`runtime_recorder/sdk_2_5_10_4_deployment.json`，使用说明同步。
- 截至准备完成，原控制进程PID2657819仍运行（15:31启动），当前入口和部署库尚未替换；XML及历史日志保持原样。已请求现场正常退出旧进程，收到确认后再核对进程、原子替换入口/库。后续由现场使用原命令启动，静止记录约10秒，验证record_start/status/heartbeat及250Hz左右的实际周期、12轴/valid位、正常结束和丢样计数，再恢复FluxVLA的--record-controller。尚未宣称正式5999与真实PDO验收完成。

### 2026-11-09：0.150入口切换、现场日志验收与断开事件修正（已完成）

- 用户确认旧进程退出后，于16:19原子切换同名kaanhbin启动脚本和SDK2.5.10.4记录库；新版ELF内容保持原SHA256，默认只加载记录模块，不执行扩展库自动登录。XML在准备至切换期间由外部/控制器更新，本次保留退出后的最新配置，切换前后XML校验一致。
- 用户通过原命令启动新进程PID4178286，确认实际执行kaanhbin.sdk-2.5.10.4、映射新版SDK及记录库。5999无其他已建立控制连接时，仅发送get、record_status、record_start、record_heartbeat，无UDP/使能/跟随/运动命令。
- 首轮约10秒采样加5秒心跳收尾，共3756周期、250.000Hz，中位间隔3.999553ms、最大4.113827ms；12轴配置、0丢样、无周期缺口、SDK元数据2.5.10.4、CRC与结束记录完整。日志`log/robot_state/2026-11-09_16-22-17.878+0800-p4178286-n8177902577150-0000.rlog`。重连record_status确认不会自动开始记录。本机验收详情`work_dirs/a10/vr/controller_recorder_sdk_2_5_10_4_validation.json`，控制器摘要`runtime_recorder/sdk_2_5_10_4_live_validation.json`。
- 开始记录前控制器已有Disabled/Stopped、错误-8456（12轴检查模式失败），整段保留该状态。实际位置有效位未置位（各轴掩码752），无效位置保持NaN，不能将“12轴结构存在”解释为实际反馈全部有效；本次没有清错或执行运动验收。
- 现场发现WebSocket正常关闭及明确TCP关闭均由heartbeat_timeout兜底收尾。加强回环测试：模拟应用在初始化后清空Interface断开回调，并要求结束原因为disconnected；在修正前可复现失败，原测试仅检查文件完整，未识别这项延迟。
- 在现有sdk_adapter.cpp补Socket::setOnLoseConnection事件桥接，保留原SDK回调，仅对选定5999 Socket原子更新连接状态并请求后台停止；不在Socket回调等待命令锁，不增加轮询线程、网络端口或运动行为。加强后的全部18项CTest通过（无跳过），原SDK临时端口回环约0.20秒完成。
- 修正库已暂存，SHA256为522062f0fb78a795ad0e61f5b5cfdfc575ef0b8d3fd263579a5c27eb6cabd6aa；源码及首版新版库备份在原备份目录before_socket_bridge/。远端部署报告disconnect_bridge状态为tested_waiting_for_controller_exit；等待用户再次退出当前进程后原子替换最终库、启动并确认现场disconnected结束原因。

- 用户再次退出控制进程后，已原子替换最终库；入口、原SDK2.5.10.4二进制及切换前后XML均保持原样。用户使用原命令重新启动PID384753。现场最终约1秒记录250周期、0丢样，周期中位4.000407ms/最大4.134166ms，实际约249.974Hz；文件CRC及结束记录完整，结束原因已确认disconnected，不再依赖5秒心跳兜底。日志为log/robot_state/2026-11-09_16-33-42.973+0800-p384753-n8862998378568-0000.rlog。
- 远端部署状态已更新为installed_live_recording_and_disconnect_validated；最终验收摘要runtime_recorder/sdk_2_5_10_4_final_validation.json。本机验收JSON同步final_inspection。最终启动后首次get在record_start前即显示Disabled/Stopped、-506“电机1位置跟随误差过大”，实际0.366514、目标-3.141569；这是当前需另查的实机状态，不能宣称正常运动验收通过。本轮未清错、使能、启动跟随或发送UDP目标。
- 日志功能恢复完成，仍用sudo ./kaanhbin启动；FluxVLA可在原命令重新加--record-controller。默认不自动开始记录，记录协议和文件格式保持原样。原始备份、首版新版库及最终源码/测试均留在控制器；本地只提交本次开发记录，保留其他会话的operator、m_init、VR和CLI修改。

### 2026-11-11：取消VR参考模式固定步长退出，核对换向迟缓

- 按用户要求，单/双臂reference模式采用现有按dt的速度/加速度约束，移除运行层每帧距离/角度超限退出，并使SDK工具累计适配器支持None表示不设固定步长。其他显式SDK步长调用与delta模式保持检查。旧CLI参数在reference模式中兼容但不生效。
- 保护输入新鲜度、反馈有效性、控制周期超时、跟踪误差与停止确认仍保留；未处理或掩盖现有stop_follower后get超时。
- 新增离线回归覆盖单臂、双臂、SDK整组UDP回环，33/40/44/60毫秒抖动下允许大于5毫米/0.025rad的合法运动，验证线/角速度和加速度上限，以及控制间隔异常仍请求停止。
- 基于最新会话，28个明显换向采样段均受到前端0.4m/s²限加速度；软领先限制也会造成额外减速。只读查0.150 XML未发现独立末端加速度配置，不能把关节17.4533rad/s²直接当作末端响应能力或断言SDK内部没有限幅。本轮保留用户运动参数，由现场后续单参数A/B验证。
- 验证：新增5项回归通过；单臂reference、双臂reference、输入暂停恢复与新增测试合计83项通过（11.653秒）。A10全套60项在本次修改前后均有相同2项断言失败、16项错误，逐项失败名称一致，属于既有测试/接口不一致；未把这些既有问题混入本次修改。另6项既有双臂SDK/控制台回环测试全部通过（10.694秒），覆盖整组UDP及显式限幅。全程未使能或驱动真实机械臂。


### 2026-11-11：固化双臂VR当前调参默认值

- 用户指定的双臂运动参数保存至a10_dual_arm_vr_teleop.py：倍率70%、线/角速度0.17/1.1、线/角加速度1.0/3.0、平移/旋转增益10/12、软领先预算0.03m/0.1rad。补充配置注释与a10_vr_operator.md简短启动命令。
- 保留默认vr_only=True、observe_only=False、operator.allow_motion=False、夹爪关闭及原网络配置。未改变单臂参数，未修改用户CLI.txt及命令速查未提交内容。
- 本轮只固化用户提供的候选组合，不将0.1rad软预算声称为已验证参数；查询超时由独立通信修复处理。验证采用mmengine配置合并及数值/开关断言，无网络连接、使能或实机运动。
- 根代理补充验证：更新既有双臂配置断言为角加速度3.0和旋转领先0.1，对应回归通过；新默认参数的实机许可加--record-controller配置预览通过，未启动服务或连接设备。

### 2026-11-11：只读实测GET与端口，补充停止故障收发证据

- 使用与双臂控制台相同的5999接口和2秒超时：410次GET全部成功（30Hz300次，最大17.14ms），再重复50轮已停止状态下的_wait_mode(False)+wait_stopped，全部成功。未使能、启动/停止跟随或发送运动目标。
- 只读SSH确认示教器使用5867。5999已连接时第二连接立即被拒绝，原GET仍正常；5888/5999双连接可同时GET。本次没有复现同端口抢连使原连接GET超时，历史故障时的连接情况仍不能由当前快照确定。
- 对照17:29会话：最后成功发送为17:29:50.950，随后双手松开请求停止，17:29:54.983记录GET超时。syslog对应时段未见stop follower记录；无法仅凭现有日志确认停止命令是否被及时处理。
- A10直连传输增加32条以内的收发摘要，仅失败时输出；VR诊断JSONL增加communication_failure，保存连接地址、收发时间和回复类型。空ACK没有命令标识，不把它作为停止执行证明。保留全部既有收发、超时与停止行为；保留本轮开始前用户未提交的operator、m_init、录制与CLI修改。
- 验证：新增3项协议收发诊断回归（无回复、空ACK后超时、空ACK后正常状态、历史有界与不重发），与既有Codec/单臂及双臂reference合计89项通过；git diff --check通过。本次端口对照及GET测试结果保存在本机/tmp/a10_port_readonly_20261111.json、/tmp/a10_get_readonly_20261111.json。尚未复现实机停止过渡，不宣称已修复超时。

### 2026-11-11：双臂旋转增量驱动与松手保持

- 18:06会话验证新收发诊断：18:07:12.599普通GET仍回复，12.600发送stop_follower与GET后两秒无任何回执。只读GET连续发送、同一TCP写入两个合法WebSocket消息均可收到两份正常回复，未复现粘包丢消息。本机超时关闭后控制器再发送可能出现Socket不在可发送STATE；该STATE不是机械臂运动状态，未将其当作根因已确认。
- 子agent只读分析本轮2003帧及控制器18179个250Hz样本：正常运动中约64%旋转跟随帧前进比例不足一半，配置1.1rad/s实际角速度中位0.402rad/s，存在约160ms下游轨迹滞后（拟合估计，非网络延迟）。控制器目标在松手前约0.68秒停止变化，零丢样和控制器/轴错误；目标冻结仍需控制器端接收/规划证据，未修改远端。
- 用户要求改善手停后的追赶与旋转速度：control.py增加可选incremental_velocity，只消费已验证相邻手柄姿态增量，保留速度/加速度/领先和硬跟踪保护。原anchor默认与标定绝对姿态保持原算法；平移计算保持原样。双臂配置启用新模式，最大角速度1.4、加速度6、制动10、领先预算0.3；限速期间未完成角度不再事后补齐。
- 双臂配置stop_on_both_release=False，双手松开分别减速保持、保留SDK会话，不重复原生停止/查询；退出及故障仍请求停止。keep_session_on_query_error=True只在本步实际GET/记录心跳通信失败时保留VR与诊断、停止新目标及配件，不自动恢复或重连，不绕过缺失反馈继续运动。
- 本轮仅修改FluxVLA和离线回归，不连接机器人控制接口、不修改控制器、不执行实机运动；SDK已接收目标的物理滞后仍需现场验证，未声称硬件立即停手或根治控制器超时。
- 开发期间用户新增18:21会话于18:25触发command_tracking_error：日志仍为旧anchor模式/角速1.1/加速度3；最终角差0.3523239rad超0.35，平移0.02890m未超0.05。参考误差仅0.25865rad，旧command追赶及制动不足使其更超前。该轮目标正常更新、周期正常、停止已确认，不能当作查询超时或新模式实机结果。新增双臂启动日志明确打印实际旋转模式/速度/加速度/制动和松手策略，现场须重启进程加载新配置，保留硬保护。
- 验证：新增旋转与保持/通信暂停回归，连同旧单双臂、控制算法、领先预算、速度、记录心跳及WS诊断共140项通过（14.664秒）；实机标志加record-controller的配置预览通过，未启动服务/连接设备。离线固定167ms下游延迟对照：前端停手后追赶约500ms降至133ms、追加角度约12.79°降至2.83°，但模拟末端仍有尾随；仅模型结果，不作为真机性能保证。
- 随后用户明确要求放宽前端运动阈值：增加enforce_tracking_error_limits开关（类/单臂默认True），双臂默认False，超差仅记录告警；双臂两种softlead设None，继续使用最大速度、加速度和制动约束。不因reference/command/release跟踪差停止，释放以command归零稳定为holding，实际quiet仍展示，非物理停止证明。runtime重复误差检查同步采用同一策略，避免内层放宽后外层仍退出。
- 输入/反馈有效性和时效、控制间隔、控制器故障、通信失败暂停及SDK驱动保护保持原流程；没有修改远端控制器。旧CLI若显式传lead数值仍会重新启用软限制。新回归验证远大于原阈值时仍以限定速度/加速度发送完整双臂SDK数据，以及恢复True后原故障与无效数据检查有效。
- 最终验证：145项相关离线/本地回环回归通过（15.085秒），包含新增放宽策略与完整双臂SDK发包检查；最终默认参数的实机标志+record-controller预览通过。未执行真实机器人控制；仅整理本轮自有修改，保留其他会话未提交内容。

### 2026-11-11：移除正常松手再握的二次释放锁

- 用户反馈误差WARNING后不便接管。读取18:34会话发现18:36:32.563释放、32.630重新握住，旧逻辑在releasing期间设blocked/reengage_requires_release=True，保持后仍需再释放；没有input_pause或communication_pause，该WARNING并未停止控制。
- 删除正常每臂blocked锁，已验证的新鲜侧键输入在减速过程中也可直接接管；continuous_reanchor保留命令位姿与速度，不回跳反馈、不重启SDK、不重置另一臂。历史reengage_requires_release字段保留False以兼容诊断。
- 跟踪误差只记录策略的终端提示改为INFO，一次提示且明确“不限制、不锁定”。没有移除失联或控制器故障的暂停。同步更新VR说明和命令速查中原“等保持后再次释放”旧说明，命令不变。
- 验证：既有双臂/SDK生命周期与保持会话共65项通过（4.414秒），新增连续快速松握、旋转连续和完整双臂目标速度上限验证；真实故障不自动恢复的既有回归仍通过。本轮未连接控制接口或执行实机运动；保留用户未提交修改。

### 2026-11-11：无输入晚运动，修复holding断流继续发包

- 用户报告自主运动，最终现场确认“已急停、双手都已松开”。未发送任何真实机器人控制命令，保持SSH只读。
- SID2026-11-11_18-43-30.360+0800-p505433-n14770080171091：18:44:47.231最后松键，47.307/47.407前端角/线速度归零、47.469进入holding，48.167最后VR采样；18:45:34.790控制器目标重新更新，34.818—36.362实际运动1.544秒，峰值约1.07rad/s，此时VR数据已过期46.7秒。另有18:44:34.770—35.522数据过期后的晚运动。高频32832样本、250Hz、零丢样/轴错误；syslog只有18:45:41的stop follower，缺UDP采纳和命令来源，不能证明SDK内部成因或排除其他控制源。
- 确认软件缺口：原_input_state仅按键按下/正在tracking时检查年龄，已holding后即使浏览器断流仍无限重发旧累计目标。当前新增全输入流超时路径覆盖holding/releasing，停止发包并走既有原生停止/确认流程；不新增运动误差阈值。未收到首个有效手柄前保持idle，不启动机器人。
- 撤回未经实机验收的双手松开持续holding默认，stop_on_both_release恢复True。正常双手释放停止确认后新按键可直接接管，保留此前移除二次松手锁的便利；未确认停止不自行重启。未使用的手柄无新数据也不会阻止活动手释放后的全组停止。
- 本轮修复只封堵前端失联继续发包；不能宣称SDK晚执行根因已修复。没有解除现场急停、重新使能或安排运动验证。
- 离线回归：双臂保持会话、双臂运行时、单臂参考运行时和控制器录制共97项通过（12.809秒）；覆盖断流后不再发旧包、正常停止确认后直接重新接管、闲置手柄无新数据时活动手松开仍能停止。没有执行实机运动测试。

### 2026-11-11：保存当日遥操作调试成果

- 现场反馈：用户确认遥操作体验良好，已完成实机测试，要求保存当前状态。此条为用户现场反馈，不代表控制器延后运动的内部原因已查明。
- 今日已提交内容包含双臂默认调参、增量旋转与制动、误差仅记录、查询异常诊断、正常重新接管简化，以及断流停止重发旧目标。
- 本次归档剩余修改：m_init保存并恢复原follower开启状态、Axis/Cart模式、jog坐标系与速度倍率；支持Base/Wobj坐标选择，同一会话同模式避免重复启动。录制回位后同步VR的follower状态，下一次接管重新建立参考。
- 同步保存CLI.txt中的快捷启动、历史调参和录制恢复命令，以及命令速查文档。CLI中的显式覆盖参数为历史调参示例；省略覆盖时以当前配置默认值为准。
- 验证限制：本次本地组合回归104项，报告2项失败、9项错误，均位于test_a10_init_motion；可见初始位断言与配置不一致、测试参数含当前接口不接受的max_velocity_rad_s/max_acceleration_rad_s2，以及模拟控制器start_follower连接关闭。尚未修复，不将本次归档标记为全量回归通过。测试只使用本地模拟设备，未连接真实控制器。
- a10_dual_record.zip为本地数据压缩包（约16MB），保留原文件，不纳入代码提交。
- 单独复核录制、双臂保持与双臂运行时：test_vr_recording、test_vr_dual_hold_session、test_vr_dual_runtime共93项通过（6.082秒）。

### 2026-11-11：双臂控制台与遥操作共存

- 问题：双臂控制台显式拒绝shared模式，双臂VR默认直连；两者争用控制器单客户端WebSocket。原共享服务按单臂解析双臂GET，还会对同一5999端口打开两条连接；VR初始化Tool坐标和record_*指令也未支持共享角色。
- 实现：复用engines/communication中的共享服务，双臂operator默认managed模式；本机首个客户端自动拉起独立服务，先绑定5870再打开唯一机器人5999连接，防止同时启动重复占用。两个UI可任意顺序启动或退出；最后一个客户端退出约5秒后服务自行退出。旧single/shared/auto模式和显式直连接口保留。
- 共享服务只解析控制器全局状态，完整双臂反馈仍由A10Operator按配置映射验证；相同命令/状态端口共用socket与事务锁。保留双臂UDP行顺序、ID映射和今日速度/旋转算法。
- 查询不接管，控制台动作先停止并撤销原发送权；m_init在读取原始控制状态后保留整个多步骤操作的控制权，结束时恢复原模式。VR被接管后保留界面，停止旧目标与配件操作，正常释放控制权并松开侧键后下次按下重新取实际锚点。VR退出不会停止控制台的新跟随；不会自动恢复仍按住的旧输入。
- --record-controller通过服务的同一机器人连接发送record_start/status/heartbeat；录制权限独立于运动权限，控制台GET不打断记录。服务故障不自动回退直连或重发运动，启动/所有权事件保留日志。
- 文件：双臂operator配置、a10_shared_connection.py、a10_shared_server.py、A10Operator、VR runtime、a10_control.py、a10_shared_control.py，以及命令速查/现场指南/CLI。新增严格单客户端的本地双臂VR集成测试；修正既有模拟控制器漏实现的start_follower模块ACK，旧双通道测试明确用独立状态端口。
- 全部验证使用回环模拟控制器和合成手柄，未连接实机、SSH或修改控制器；真实双臂并行窗口验收留待现场。
- 最终回归：`PYTHONPATH=.:test python -m unittest test_a10_vr_shared_control test_a10_shared_control test_a10_control test_vr_dual_hold_session test_vr_dual_runtime test_vr_recording test_vr_controller_recording`，185项通过（46.644秒）。覆盖双臂同连接并行查询与目标、任意/同时启动、独立退出与空闲释放、离线不回退、控制权接管/竞态旧包、完整m_init恢复、GET超时保留界面、旧连接迟到清理、控制器记录心跳、闲置手柄和HUD提示。
- 更新旧控制台测试驱动：明确等待上一条协议命令回复后再注入故障，避免异步输入导致故障被施加在错误阶段；Ctrl+C按当前stop流程断言，模拟器补齐start_follower/manual_stop ACK，双臂共享测试改为允许并验证配置。并发行为由新增真实回环集成测试独立验证。
- 命令预览以127.0.0.1覆盖机器人地址通过，未启动服务或连接设备。自动审批拒绝了最初含实机地址的预览请求，之后仅执行回环版本；没有执行实机连接。完整离线输出保存至`work_dirs/a10/shared_control/offline_tests.log`（不提交运行日志）。

### 2026-11-12：修复共享服务仍在线但控制器连接失效的启动失败

- 现场日志：本机共享服务09:32:48建立到`.150:5999`的连接，09:34:49首次记录`ConnectionClosedError: no close frame received or sent`；09:35:52、09:37:05及后续启动继续对已关闭的机器人连接执行record_start，返回`A10 operator is not connected`。控制台仍连接127.0.0.1:5870，所以本机服务一直存在。初次断线的控制器/网络原因尚不能仅凭现有记录确认。
- 确认软件缺口：hello只核对地址，未验证真正的机器人连接；VR connect返回后立即record_start，绕过可恢复连接的GET路径。昨日正常在线的并行测试没有覆盖服务存活、后端断线、再启动VR的路径。
- 修复：新VR/推理客户端握手先通过GET验证真实控制通道。共用单WebSocket的读取路径复用同一恢复函数；空闲断流时隔离旧socket、重连并仅重试一次只读GET。已尝试发送的record_start及运动指令不自动重发。若读取重开连接时存在旧运动控制者，撤销其旧UDP发送权，必须显式停止和重新接管后才能发新目标。
- 控制台hello保持本机握手，不等待机器人事务锁，确保停止用的重连仍能先interrupt正在等待回复的长命令。控制台正常GET会验证并恢复空闲通道。
- 新增5项回环集成：控制台仍在线/后端已关闭后启动录制、缓存已远端关闭的WS对象、record_start回复丢失不重发且显式重试可恢复、运动中失联不恢复旧目标、长命令期间停止用控制台连接不被GET探测阻塞。未向真实机器人发送指令，未重启用户当前运行的共享服务。
- 加载修复：当前旧控制台输入exit，等最后一个客户端退出后约5秒服务自行结束，再用原命令重开控制台和VR；不更改原速度、加速度、ID映射或控制器XML。
- 回归结果：共享/双臂VR/控制台/录制相关7个模块共190项通过（47.141秒）；完整输出保存到work_dirs/a10/shared_control/offline_reconnect_tests.log。没有执行实机运动验证。

### 2026-11-12：固化实机双臂夹爪默认入口，降低偶发串口错误对遥操作的影响

- 用户明确给出vr_only=False、observe_only=False、operator.allow_motion=True、zx_grippers.enabled=True、--record-controller的完整命令，并要求“这个固定为遥操作默认设置”。a10_dual_arm_vr_teleop.py据此设置现场默认，短命令只需teleoperate.py --config。单臂基础配置、运动参数、夹爪15%速度/60%扭矩与0..9000范围不变，不自动使能机械臂。
- 最新诊断第2430帧右夹爪CRC错误：此前实际raw3201、目标raw3801、alarm_a/b均0，右摇杆约-0.9993；左夹爪正常。旧日志缺少错误寄存器与原始TX/RX，不能确定是读回复或写ACK，也不能仅凭CRC断言具体USB硬件原因。
- ZXGripper新增通信错误/CRC错误细分。保留CRC校验与串口互斥，函数03查询CRC错误最多重读2次，失败不返回伪造位置；动作写入和触发ACK错误不自动重发。记录ID、功能码、寄存器、TX/RX与累计次数，提示按5秒节流。
- 单侧后台通道对通信错误只进入communication_wait，继续读取有效反馈，取消旧的待发送目标；前台不因此退出或停止机械臂/另一夹爪。有效反馈恢复、摇杆回中后接收新的输入，不重放持续按住的旧目标。不主动张开或掉使能；真实设备报警不当作正常数据。数据集录制拒绝把通信异常反馈写成正常采样，但不因此退出VR。
- 显式VR-only、按键检查及观察模式不继承现场默认录制；观察时仍可显式使用--record-controller。配置构造与--preview不建立连接。合成测试显式关闭现场设备默认，避免测试误开USB/实机。
- CLI/速查/VR文档同步短命令与恢复方式。保留用户CLI中已加入的参数调整注释，提交时只包含本次修改。
- 验证：ZX驱动/夹爪通道/双臂VR/共享服务/控制台/录制相关9模块共250项通过（51.544秒）；包含默认短命令preview无网络/USB/子进程、CRC读重试上限、写触发ACK丢失不重发、单侧持续坏包不停止机械臂/另一夹爪、有效反馈恢复后回中再控制。日志保存work_dirs/a10/shared_control/offline_gripper_tests.log。未连接或驱动实机。

### 2026-11-12：修复 Thor Type-C 扩展坞相机无法识别

- 现场系统为 Jetson AGX Thor 开发套件、L4T R39.2.1；USB-C 角色节点 `usb2-0-role-switch/role` 原为 `device`，设备模式 USB 网络未连接。左腕 D405（255323070485）和外部 D435i（262923065199）在 USB 列表中缺失，右腕 D405 仍在原 Type-A 扩展坞上正常。
- 按用户授权临时执行 `none -> host`；GenesysLogic Type-C 扩展坞及上述两台相机立即枚举，均为 5000M。故障确认发生在 USB 主机角色，未通过改相机 ID、帧率或驱动绕过。
- 使用既有 `scripts/a10_devices.py --update` 备份并仅更新外部相机 by-path 至 `platform-a80aa10000.usb-usb-0:2.4:1.3-video-index0`；序列号、画面旋转、夹爪参数保持现场设置。设备路径文件原已有未提交修改，继续保留为现场未提交配置；用户 CLI 与本地数据压缩包不纳入此次提交。
- 在 `local_setup/thor` 保存主机角色脚本及 systemd 服务，部署到 `/usr/local/sbin/fluxvla-usb-camera-host`、`/etc/systemd/system/fluxvla-usb-camera-host.service` 并启用。服务在 NVIDIA 原设备模式配置完成后运行，已为 host 时不重新切换；原厂脚本保留，回退步骤写入本机 README。固定该口为 host 后，如需使用该口的 USB 设备模式，须先禁用本服务并切回 device。
- 验证：脚本语法、systemd 单元检查通过，服务 active/exited 且 enabled，部署内容与仓库文件一致；两夹爪、三相机均匹配。三路 640x480 真实彩色画面读取成功并人工查看拼图，结果位于 `work_dirs/a10/hardware_checks/device_check_20261112_101537_420324/`。本轮只操作 USB-C 数据角色和相机，不发送机器人或夹爪运动指令；未重启，未将短拍照验证扩大为长时间录制验收。

### 2026-11-12：精简夹爪更新流程，去除同周期重复收发

- 用户反馈夹爪运动略有卡顿，并要求控制实现简单。检查最新10:11:17—10:12:47会话2689帧：两侧CRC计数均0，无通信错误或设备报警；运动中左反馈更新中位约128ms，右约87ms，变化目标在前台观察到的间隔中位约132ms/100ms。配置io_hz=20仅为上限，不等于实际发送频率。
- 每轮原流程为状态3次读、目标函数再读1次报警、复位触发/参数/使能/触发4次写，共8次往返。只读查询左FTDI ttyUSB0 latency_timer=16ms，空闲约48ms、运动约128ms的反馈周期与此高度吻合；未修改系统串口参数、打开设备或驱动实机。
- 小幅精简：目标更新复用同一工作线程刚读取的ZXGripperState，省去重复报警读取；已经确认使能时省去重复使能写。正常运动每周期8次减为6次往返。原独立调用不传feedback时仍沿用完整流程；状态未使能时仍先使能再触发，报警时不发送动作。不新增模式、文件夹或操作步骤。
- 手册0x0108=0含义是“不触发”，1才执行临时区目标，现有记录不能证明每次写0都会制动；保留原触发序列，不用猜测的连续控制协议替换。
- 验证：ZX协议与VR夹爪共63项通过（4.479秒），包含已使能时仅3次写、未使能时正常使能、报警反馈阻止动作，以及既有CRC容错、单侧暂停、旧输入不重放。未做实机手感验收，左侧16ms USB延迟仍可能限制流畅度。保留用户CLI注释和相机路径修改。

### 2026-11-12：合并夹爪反馈读取并设置FTDI低延迟

- 用户同意将左FTDI由16ms降到2ms，并要求减少连续问答。先对两只夹爪执行纯功能03只读探测，确认0x0401..0x041B连续27寄存器读取可用；状态、报警和实际位置与原先分开读取一致。未触发任何运动、使能、复位或Flash写入。
- ZXGripper.read_state把状态/报警与位置/速度/电流合并为一次反馈读取，使能仍单独读取。反馈从3次减少到2次，正常已使能的遥操作周期总往返由最早8次、上一版6次降为5次（2读3写）。保留参数/触发的写入回复，避免丢弃回复后串包；未合并未经现场证明的动作触发写法。
- 已按USB序列号DU0EMMMM核对左ttyUSB0，将sysfs latency_timer由16改为2ms，并安装/etc/udev/rules.d/99-fluxvla-a10-ftdi.rules后重载规则；源码保存local_setup/a10_ftdi_latency.rules。只匹配这只FTDI，未对当前设备触发重连；实际USB拔插持久化验收未执行。
- 新读法每侧10次只读复核、每次2个03请求，字段与旧读法一致，CRC均0；完整状态耗时中位左21.99ms、右25.99ms。该结果是反馈读取耗时，不等于运动周期或真实手感改善率。
- 验证：ZX驱动/VR夹爪64项通过（4.362秒）；模拟器按真实连续寄存器布局返回，运动中报警注入同步到新的读取长度，报警仍能正确上报。udevadm verify验证规则成功。日志/只读结果保存work_dirs/a10/shared_control/offline_gripper_batch_tests.log及gripper_batch_readonly.json。没有执行实机开合动作，未提交用户CLI/相机路径修改。

### 2026-11-12：final版遥操作，现场确认正常并保存

- 用户确认当前遥操作和夹爪使用正常，要求提交“final版遥操作”。本次以当前kaanh_follower状态保存现场验收节点。
- 默认入口：python scripts/teleoperate.py --config configs/a10/a10_dual_arm_vr_teleop.py，启用双臂实机、两只知行夹爪、控制器高频记录；控制台与VR共享连接，左右手独立控制。
- 保留已确认的运动参数、夹爪通信异常局部恢复、正常周期2读3写、左FTDI 2ms及重插规则。10:30:14右夹爪一次CRC错误经1次只读重试恢复，后续日志无通信错误或设备报警，未因此退出VR。
- 将当前CLI参数调整注释和已更新的外部D435i现场路径platform-a80aa10000.usb-usb-0:2.4:1.3-video-index0一并保存。此前250项相关回归与后续64项批量反馈/夹爪回归结果已记录；本次为现场验收后的文档/配置归档，不重新执行设备动作。
- a10_dual_record.zip为本地数据文件，保留但不纳入代码提交。

### 2026-11-12：三路相机观感检查与快照预热修正

- 用户反馈device_check_20261112_103414_995702的三路画面观感不同、部分偏糊。查看拼图和三张原图，均为真实640x480；外部D435i细节较软，右腕D405偏绿。三路只读V4L2查询均自动白平衡开启、锐度50，无调焦控件；未改曝光、白平衡或锐化参数。
- 只按现有分辨率/帧率读取90帧并分别保存第5/90帧。对比位于work_dirs/a10/hardware_checks/camera_quality_20261112_103828/：右腕平均RGB由96.54/120.84/102.93变为117.65/117.41/115.14，偏绿明显减轻，同一画面几何位置不变。该结果支持开流初期自动曝光/白平衡仍在稳定，并不证明软焦问题已解决。未操作机械臂或夹爪。
- 修正scripts/a10_devices.py快照流程：由取第5帧改为预热90帧后保存，并在manifest写入warmup_frames。约增加数秒检查等待，不新增调参入口，不对照片软件锐化或伪造细节。
- 外部与腕部型号/成像链不同，D405由深度成像器经ISP提供RGB，与D435i独立RGB成像存在天然差异。厂商说明RealSense为固定焦点。现有单一场景不能区分D435i偏软来自成像距离、曝光、下采样或光学状态，尚未修改正式录制/推理的相机参数。
- 官方参考：https://www.realsenseai.com/product-family/d405-series/；https://github.com/realsenseai/librealsense/issues/12030 。保留当前CLI未提交修改。
- 验证：既有设备检查回归45项通过（2.303秒）。修正后用正常--snapshot入口生成device_check_20261112_104127_654643，manifest确认warmup_frames=90，右腕偏绿明显改善。焦点/清晰度仍需要实际工作距离与更高分辨率对照，不将色彩恢复宣称为对焦修复。

### 2026-11-12：控制周期超时保留会话并修复废弃拨杆漏识别

- 排查10:49:56开始的实采会话：10:51:52开始录制，103帧约3.48秒后发生control_gap；末轮输入读取时两手数据年龄约4.4ms，机器人反馈到达较前一轮晚约269ms，整轮含停止清理约312ms。附近另有左夹爪串口超时；仅凭当前日志不能证明二者因果关系，也不能断言是纯VR网络延迟。失败片段及原数据保留，未访问或修改控制器。
- 双臂在推进任一臂的积分器前统一检查控制间隔，超过既有max_dt_s=0.1秒即丢弃本轮目标，进入已有停止/稳定/重新接管流程，停止确认后保留VR、HUD和录制入口。持续握键不自动恢复，松手/回中、新鲜有效追踪至少0.5秒后须新按键；停止失败和控制器故障仍不能恢复运动。没有放宽速度、加速度或时间保护。
- 分段记录robot_feedback/shared_control/controller_recording/target_send/stop_request/stop_confirm耗时，单位秒。control_gap暂停状态保存故障前耗时快照和实际间隔，区分等待源与事后停止耗时；目前未消除控制器、共享链路或主机调度造成长等待的底层原因。
- 修复RecordingButtons在离开0.2回中区但尚未达到0.7触发阈值时提前取消armed的错误；到达方向阈值才消费操作，主轴至少占另一轴1.25倍允许轻度斜拨，模糊对角仍需回中。保留新鲜度、序号去重、进出回中、左夹爪屏蔽和废弃/复位二次确认。重放11月9日17:47真实输入，原仅1次开始命令，现可识别17次左拨。
- 正在录制时控制周期断档仍使本集失效，禁止正常保存；暂停期间可用真实逐步左拨、回中、再次左拨废弃，废弃不绕过重新接管门槛。文档补充完整恢复顺序。原始失败片段不自动发布，不以重复状态/拼接数据假装连续30Hz。
- 验证：39项针对性回归通过，覆盖240ms成功但迟到的反馈、停止失败、只读语义、握键不恢复、显式恢复、逐步斜拨及暂停中废弃重录。另141项相关回归中139项通过，2项旧“超时必须退出”断言改为“确认停止并保留会话”，所在速度模块5项复测通过；合计180个不同用例当前通过。最后增强暂停/废弃门槛的单项复测通过。所有设备均为模拟/回环，未启动真实机械臂或串口；本机PATH无node，本轮未执行Node渲染检查，网页代码未改。
- 范围说明：本轮未实现此前讨论的异步采集/停止确认；正常双手松开时的同步停止等待仍可能导致采样断档。控制循环超时恢复不等于完整数采链路验收。只提交本轮VR、测试及说明，用户CLI.txt保留未提交。

### 2026-11-12：独立30Hz采样，修复正常松手等待导致整集失效

- 对照11:05:06会话：第一集88帧约2.98秒后在11:06:04变红，停止请求255ms、停止确认7ms，使旧机器人反馈年龄约270ms；第二集30帧约0.96秒后在11:06:34变红，GET约248ms且两夹爪短暂无回复，左/右反馈年龄约289/274ms。11:07:18电机4掉使能是后续真实故障，本轮按用户要求不修改电机速度/加速度、不处理或掩盖该报警。
- RecordingSession新增独立30Hz采样线程，运动主循环只处理输入、状态机和HUD。相机/夹爪继续各自独占设备；机器人从真实反馈缓存取样。开始/保存/废弃与采样用短RLock协调，异步任务先变更状态再提交，避免保存过程中继续append；达到帧上限只排一次保存。关闭时先停止采样，再完成磁盘任务和关闭缓存读端。诊断复制字典时冻结items视图，避免控制线程更新诊断导致采样异常。
- 共享服务发布普通GET及原生启停内部GET的完整实测回复、真实主机接收时间和序号。新增本机只读feedback_snapshot RPC，使用独立本地通道，不争用运动事务锁、不新增控制器WebSocket、不增加机器人GET、不取得运动权限；关闭只读通道不撤销控制端所有权。A10Operator统一构造状态帧并提供只读快照接口，缓存读取不刷新原时间戳，直连模式也可读取本进程各次GET产生的缓存。
- 双臂VR串口等待新增serial_timeout_s=0.05秒，原默认200ms一次丢回复就会耗尽采集年龄预算；缩短后可更快进行下一轮只读状态查询。一次communication_error不立即废弃整集，仅在原年龄/设备时间差允许范围内复用上次实测反馈，保留错误标记和原时间戳；真实报警、未使能、持续过期仍拒绝。没有重发不确定的写指令、没有提高速度/力矩，也没有放宽150ms采样间隔/机器人年龄、200ms夹爪年龄或150ms时间差检查。
- 休息时未接管的手柄可以暂时无追踪，但必须持续有新鲜VR数据；主动请求控制却无有效追踪仍不接受。既有输入暂停、控制周期异常与设备故障处理保留，不把真实丢失的机器人/相机/夹爪数据补零或改时间戳。数据格式、14维物理左右顺序、三相机字段及OpenPI交接合同不变。主JSONL新增sampling_mode=background_30hz，每集诊断新增真实反馈序号。
- 验证：严格单客户端回环控制器原生停止350ms期间持续采样，episode保存并通过视频/数据校验，机器人连接峰值为1；缓存读不等待运动锁、不允许发控制命令，缓存冻结后时间戳不刷新。另覆盖主线程等待300ms、正常松手休息、100ms夹爪回包空窗后恢复并核对保存诊断保留旧时间戳、真实反馈冻结必须失效等。最终61项录制/串口/快照回归通过（6.966秒），原用户命令preview通过。
- 扩展回归296项中发现4个旧operator用例的3个failure和3个error（旧双臂不支持断言、Mock UDP fileno、错误文案引号和关节启动命令序列）。独立进程加载未修改HEAD源码复跑同4例，完全复现3失败/3错误；不宣称全量通过。所有测试仅使用合成状态/图像、假串口和127.0.0.1回环，未访问真机或修改控制器。
- 加载要求：本机旧共享服务须退出最后一个控制台/VR客户端，等待约5秒后按原命令重开。旧服务缺少快照接口会明确提示，不静默退回阻塞采样；无需重启远端kaanhbin。采集指南同步说明。只提交本轮修复、测试及文档，CLI.txt和现有数据集不改动。

### 2026-11-12：绕过控制器高频诊断日志额度满，恢复训练采集入口

- 11:35:29启动日志确认机器人GET已两次成功，record_start被控制器以ret_code=-1、runtime log capacity limit reached拒绝；高频诊断模块启动失败导致程序退出，不是机器人端口连接失败。没有据此推断磁盘实际已满，也未清理、移动或扩大控制器日志额度。
- 当前数据录制命令移除--record-controller，并显式加teleop.record_controller=False，覆盖双臂配置中的默认True；仅删标志仍会启用诊断、保留标志又会覆盖False，故两项必须同时调整。训练采集--record-config、三相机/双臂/夹爪与本地VR JSONL保留，运动参数不变。采集指南新增额度满时的处理说明。
- 只预览修正命令，确认record_controller=False、zx_grippers=True且真实录制配置有效，未打开设备或发运动/使能指令。未访问远端或删除数据；本轮解决可选诊断记录阻挡采集的问题，远端日志归档和额度管理另行处理。CLI只提交本轮两行开关调整，原有个人修改继续保留未提交。

### 2026-11-12：新分支复用原生LeRobot录制核心并补齐输入过期恢复

- 用户授权从当前kaanh_follower建立`kaanh_lerobot_recording`，迁入旧工具链已验证的录制核心。旧lerobot仓库另有未提交工作，本轮只从其已提交HEAD `674ec05`提取源代码构建非editable LeRobot 0.4.3 wheel并以`--no-deps`安装；未修改旧仓库的源文件、未替换Thor PyTorch/torchvision/CUDA。运行代码从site-packages加载，不把相邻仓库加入PYTHONPATH。依赖声明新增requirements-a10-recording.txt，安装与源码来源见采集指南第13节。
- 新增datasets/lerobot_episode_writer.py薄适配器，直接调用LeRobotDataset.create/add_frame/save_episode/clear_episode_buffer和本地续录；原生负责PNG异步写入、AV1视频、统计、Parquet及v3元数据。适配器仅处理14维字段/身份合同、独占文件锁及真实时间诊断。保存后finalize并在下集原生重开，避免关闭的Parquet再次被覆盖；保存/编码仍在磁盘工作线程，终端/HUD保持可用。原EpisodeWriter保留供历史路径和数据使用。
- 双臂录制配置默认backend=lerobot、sampling_mode=control_loop，使用独立目录datasets/a10_dual_lerobot_record。每轮遥操作后从真实机器人/夹爪缓存与相机取样并add_frame；正常停止等待期间不补帧，结束后继续。单次测量过期/设备时间差过大跳过并记录，持续缺少数据达到默认1秒则本集失效；原150ms采样间隔与30Hz累计漂移从整集否决改为诊断。名义时间轴仍为frame_index/30，真实capture_monotonic_s、gap、skip、重复图像和有效fps供训练前检查；不宣称严格实时采样，也未消除底层网络/调度延迟。
- 保持物理左ID1+夹爪、右ID0+夹爪的14维顺序、关节rad、夹爪raw/9000以及action=同帧实测state副本。此处是既定双臂训练约定，不盲目复制旧单臂EE增量action。三相机字段、现有UDP follower、夹爪驱动、右A+左摇杆、HUD、保存后等待下一集和m_init约定保留；未修改OpenPI或控制器。旧数据和recovered片段不动；两个后端不混写。
- 原生普通录制中断保留图片/诊断，显式recover归档后可重开；保存阶段中断保留现场并拒绝自动修复，不沿用旧自写提交日志的roll-forward承诺。合同不匹配、双写、保存中断、非法数据均有明确拒绝路径。
- 对照11:53:19日志：已录约86帧、近30Hz，随后重复手柄包年龄从169ms涨到200ms，前置检查后在VRPoseController内部累计过期；该分支过去直接RuntimeError退出。现将后置input_stale/control_gap也转入已存在的确认停止、锁定接管、松手回中、稳定0.5秒、新侧键恢复流程。停止失败、控制器故障仍不能恢复；中断集标记失效，废弃后再录。没有放宽250ms输入年龄或电机故障保护，没有处理此前电机4加速度过大问题。
- 验证：117项录制/双臂/HUD/复位/速度及恢复回归通过（11.165秒），包含真实原生三集保存与退出续录、清空缓冲、未完成归档、拒绝旧后端混写/第二写者/非法action、原生读取、视频校验与OpenPI导出，及控制器内部重复输入超时的停止/重新接管。所有设备均为模拟或本机回环。
- 三路640×480随机合成RGB以30Hz录150帧，采样4.967秒、原生保存4.657秒，append耗时99分位1.17ms，最大采样间隔33.53ms，视频和数据校验通过。日志为/tmp/a10_lerobot_throughput.log与对应report.json；这是约5秒合成吞吐，不代表真实USB、长录或机器人运动验收。新配置和只读按钮入口preview有效；未访问远端、未使能或发运动指令。
- CLI与命令速查、采集指南同步为新目录并关闭可选控制器高频记录。只提交本轮CLI片段；原有CLI个人改动保留未提交。本轮工作一次中文commit，不合并回kaanh_follower。

### 2026-11-12：旧LeRobot录制阶段对齐，夹爪故障不关闭采集界面

- 对照13:41:54—13:47:00会话，录制始终idle：m_init反复成功，但健康状态长期为left gripper not healthy/enabled。此前人工掉使能后左夹爪有真实位置却被“必须Enabled”的录制条件拒绝；m_init成功又重置整个RecordingButtons，持续按A时还需额外松A才能识别下一指令。末尾左夹爪再次报真实B=0x1000，右侧另有超时/CRC/异常功能码，不能混为数据集写入错误或认定同一硬件成因。
- 原生录制允许无报警、实际位置/时间有效的未使能夹爪参与采样，enabled原值仍记诊断；不自动使能或发位置。正常m_init结束只解除摇杆方向armed，保留已确认的A会话，保持A、回中、向右即可。复位期间停止采样并保留缓冲，完成返回原准备/整理阶段，不丢失当前集；失败/取消仍保留原运动锁定边界。
- capture.workflow=lerobot默认对齐旧record()的事件及阶段顺序：初始准备RIGHT开始；录制中RIGHT结束采样进入RESET整理场景；整理好再RIGHT保存，后台完成并短暂提示后自动下一集。LEFT标记rerecord_episode，整理好RIGHT才clear_episode_buffer并自动重录同一集；DOWN设置stop_recording，保存有效集后退出，不发布标记重录/无效集。m_init保留两手释放、原停止确认和二次确认；右A组合及左夹爪模式屏蔽保留。复用原hud_phase_from_record/build_hud_message，菜单随阶段同步，不添加新HUD面板或独立状态机文件。
- 夹爪硬件报警不再直接抛异常退出VR：读取后先发布当前真实报警位/位置/速度/电流原始值，再锁定该夹爪新目标；串口线程继续只读查询，双臂确认停止后保留VR、HUD和录制入口。本集若正在采样则标记中断、禁止正常发布，但可LEFT标重录、RIGHT废弃。一般USB线程故障也保留界面并禁用目标，无法运行的串口工作线程需修复设备后重开会话；启动即有夹爪故障不再误报正常或退出整套采集。控制器故障与停止未确认不放宽保护。
- 新增当前VR终端人工维护指令gripper-reset left/right、gripper-disable left/right：须两手释放且机械臂停止已确认，由同一串口工作线程执行一次，不争用USB。清除失败保留reset_error及锁定，不自动重试；disable会释放夹持力，必须由现场操作者明确调用。清除成功仍要求松手/回中、稳定0.5秒、新侧键接管，不自动恢复或重放旧目标。没有现场清报警、掉使能、使能、张开、运动或远端操作。
- 更新采集指南第3/5/6/14节和命令速查，说明RESET是整理场景而非自动回零，区分普通录制流程与真实故障恢复。当前启动命令不用改，入口打印workflow=lerobot。当前用户CLI修改不提交；数据、控制器、运动速度/加速度、夹爪力矩及OpenPI均不改动。
- 验证：158项原生保存/重录/续录/导出、旧阶段交互、复位后保持A开始、未使能采样、夹爪锁定/人工维护/启动故障/USB线程失败及双臂保护回归通过（17.752秒）。新增原生LeRobot真实save/clear与自动下一集闭环；全部为合成数据、模拟设备和本机回环，无实机IO。初轮出现一个历史严格采样150ms间隔用例受调度影响失败，单例及最终整组复测通过，未放宽阈值。硬件0x1000或CRC的物理原因仍需现场排查，本轮解决的是软件退出及采集状态阻塞。

### 2026-11-12：夹爪报警独立隔离与空载实机恢复验证

- 新日志14:09之后实际为VR持续运行、state=idle/0帧、左actual=8136且B=4096；原实现把附件故障接进全局input_pause，导致机械臂和训练入口一起被锁。用户明确选择“界面和机械臂操作继续，但清除报警后才能录训练数据”。本轮移除这条全局耦合，保留故障夹爪只读查询/禁止新目标，不阻塞健康夹爪、机械臂接管或m_init，不修改真实VR/机器人故障保护。
- 录制会话独立接收gripper_faults并在HUD提示故障侧与ARMS AVAILABLE/CLEAR ALARM BEFORE RECORDING；报警期间无ready_sample，不能启动训练录制。正在录/异步开始时遇到报警，片段标中断且不能保存；保留旧LEFT标重录、RIGHT提交流程。没有新建诊断数据模式、没有允许异常片段混入训练或伪造夹爪反馈。
- 人工gripper-reset/disable不再要求已有全局暂停；两手释放后，由runtime针对这次明确维护请求停止跟随并重新确认Stopped，确认失败不写夹爪。仍由唯一串口线程执行，不自动掉使能/清报警/重发位置。CLI不需改参数，用户既有未提交内容保留。
- 用户确认两夹爪空载且分开、正常退出VR后，执行只读两侧参数核对：左actual8136、设备残留target8137、speed15、torque60、trigger0、reset0、A0/B4096；右actual/target0、speed15/torque60、A/B0。说明当前持续读取的是未清锁存，不是本轮不断收到新目标。没有将其他固件/型号寄存器的未知值解释成可写配置，更未写Flash。
- 用户进一步明确授权“清报警后，5%速度7000↔7500空载往返，现场由我监护”。只对左夹爪执行disable、一次reset；确认A/B均0且未使能/速度0后，按原60%力矩、5%速度发7000→7500→7000。到位实际6999/7496/7002，三次position_reached，各保持1秒未报警；后续10秒20次纯只读复查无报警，末次actual6999、speed0、enabled=True。现场确认无明显异响、偏斜或卡顿。未操作机械臂、右夹爪、相机或Flash。
- 证据保存于work_dirs/a10/hardware_checks/gripper_alarm_20261112/的read_before.json、controlled_reset_test.json、controlled_reset_packets.jsonl和stationary_monitor.json。实际命令与状态逐条可查；测试完成保留左夹爪在约7000、已使能的状态，没有自动回到9000或关闭夹持力。这里只验证当前锁存恢复及小行程，不能保证15%速度、全行程、接触抓取或再次碰擦永不触发保护。
- 历史两次真实报警13:28:11、13:46:59之前均有torque_reached；585条相关力矩到达记录没有出现闭合轴持续输入或新的last_sent目标变化。没有证据支持“达到力矩后程序一直重触发闭合”的猜测，因此未臆测修改保持/滤波算法，也未提高力矩、屏蔽工件掉落报警。首次触发的机械接触/工件滑移/固件判据仍无法完全区分。
- 更新采集指南第6/15节、命令速查；第14节保留为旧全局暂停方案历史，明确最新约定。本轮对独立运动、训练数据门槛、维护确认、清除后无需VR全局解锁新增/调整回归。所有软件回归使用模拟设备/回环；上述单独的左夹爪授权测试是真实硬件，二者不混称。最终159项相关回归通过（17.103秒）；首次扩展启动接管测试发现模拟DualRobot未配置对应适配器，改为夹具已有legacy_anchor后整组通过，生产代码未因此回退协议。

### 2026-11-12：修复VR中断后的重录恢复，头显使用简短小写提示

- 对照附件fab4f001及dual_teleop_diagnostics.jsonl：14:33:14两手输入年龄约277ms，14:33:51约254ms，均超过250ms，因此当前集失效；第二次触发vr_stream_stale并确认停止。14:34:03确已废弃旧缓冲，14:34:03.787输入已ready，但录制等待input_pause清除、菜单只剩停止。14:34:15用户DOWN后正常退出，不是新的崩溃。日志不能单独断言网络或浏览器是哪一方导致输入间断。
- LeRobot流程保留明确的重录/开始请求：弃集完成后等待松手、摇杆回中及原0.5秒稳定条件；runtime检查当轮真实机器人反馈无故障、follower inactive、停止已确认，才恢复录制输入。重新获取全部有效样本后重录同一集；不启动follower、不发送目标、不调用m_init。机械臂仍要求释放后新按侧键；停止未确认、机器人故障、输入未稳定仍阻止恢复，250ms输入门槛不变。
- 准备阶段暂停时RIGHT可排队请求，已排队时LEFT保留retry queued，HUD明确提示释放侧键/扳机、摇杆回中；不会通过“必须先动机械臂”来解除录制等待。失效集不能保存发布，夹爪报警仍只隔离对应夹爪、阻止训练数据，不扩大到全局控制锁。
- 沿用原头显面板和方向菜单，标题、菜单和提示统一小写短句；移除任务长句、只保留集数/时长，提示正文最多48字符，菜单标签最多32字符，已知故障显示短原因。完整错误保留终端/JSONL，原按钮和按键映射不变。启动命令沿用，需重启程序并刷新VR页面。
- 165项Python回归通过：105项录制阶段/双臂恢复/历史录制（8.272秒），60项原生LeRobot写入/夹爪隔离/保持会话/速度边界（9.128秒）；真实头显渲染函数的Node测试通过，包含大小写、短文本、待重录菜单和功能键B兼容。均为模拟设备、临时数据或本机回环，未访问控制器或发送硬件动作；更新采集指南第3/16节及命令速查。CLI.txt个人改动保留不提交，本轮一次中文commit。

### 2026-11-12：独立120秒VR抓包与接收心跳诊断

- 新增scripts/diagnose_vr_network.py，单独入口，默认等待两手有效追踪后测120秒、等待追踪最多90秒。强制vr_only、关闭机器人构造/连接、夹爪和控制器记录，使用现有只读A菜单；不创建数据集、不改运动算法或250ms门槛。终端只显示启动和每15秒倒计时；结果集中到work_dirs/a10/vr/network_checks/<本次目录>，正常到时或Ctrl+C均关闭自己的抓包进程组并保存证据。
- 每50ms在真实WebSocket接收事件循环里记录调度延迟和队列深度，保留原浏览器发包、服务端取包、首次控制读取指标。tcpdump按头显IPv4与VR端口过滤，自动选择到头显的实际路由网卡；推荐访问地址也用该路由源地址，避免VPN的198.18.0.1误导。仅抓160字节包头，tshark分析时间/TCP连接/重传/乱序/缺段/零窗口；不解密位姿、不抓机器人端口。
- 自动输出report.md中文说明及report.json逐次证据；严格匹配头显IP和客户端端口，分离浏览器停发、Thor入口之前停顿、TCP交付线索、接收事件循环延迟和TLS/WebSocket待查。抓包或诊断丢记录、缺少心跳、时钟跳变、未匹配指定连接、复合故障时保留证据不足，不能把TCP分析标志直接当Wi-Fi丢包率。两分钟无复现也不宣称长期稳定。
- 现有VRInput仅增加可选latency_root，默认路径不变，其他生产收发逻辑不变；心跳/抓包/归因全部在独立程序。已安装命令行tcpdump、tshark及其运行依赖。采集指南之外新增docs/vr_network_diagnostic.md，命令速查添加一条独立启动命令。
- 93项诊断/原时序/录制阶段/双臂回归通过（5.493秒），最终独立诊断17项通过（0.448秒），含真实事件循环150ms阻塞、重传PCAP解析、服务端零窗口、时钟跳变、抓包启动失败和丢记录拒绝归因；两组有重叠，不累计数量。模拟客户端最初使用同步TLS时升级未完成，改用异步合成客户端验收，没有因此修改生产协议。
- 完整120秒本机TLS合成VR＋真实回环抓包验收：实际计时120.029秒，匹配10819个位姿包、2360次心跳，抓包/诊断均无丢记录，接收心跳最大8.87ms、首次读取最大21.37ms，无>250ms收包空档，正常自动退出无遗留进程。证据为network_checks/selftest/20261112_154047_144213_a95600，test_origin.json明确synthetic=True，不代表真实头显/Wi-Fi。最终版本另做5.03秒回环报告与无追踪1秒超时退出验证，均正确；超时返回非零并标记未完成。
- 本轮仅当前kaanh_lerobot_recording分支提交一次中文commit。CLI.txt个人修改及用户新增FluxVLA空文件不改动、不提交；未访问远端控制器，未发送任何硬件动作。

### 2026-11-12：录制期间归位不中断当前 episode

- 按用户确认的方案，将采样状态与m_init运动独占分开；首次reset仅确认，已开录且未finish的归位持续采样，完成后继续原episode。finish后和未开录的归位仍不采样。
- 复用现有只读反馈快照接口和图像/夹爪缓存，无第二个机器人GET或控制线程；保留关节回位速度、目标和停止确认。样本仍为14维真实反馈＋三相机，诊断以control_phase区分teleop/m_init。
- 头显归位时明确提示capture active；取消或采集/归位失败保留失效片段，不因worker成功返回覆盖故障。
- 更新数采指南、命令速查和离线状态机/原生LeRobot保存测试。未连接实机执行运动；用户CLI.txt及未跟踪FluxVLA文件保持原样。
- 验证：录制状态机、真实原生LeRobot写入、双臂runtime共121项离线测试通过；归位帧与三路视频已在临时合成数据集中读取验证。
- 扩展m_init历史测试单独复现4项失败、3项错误（含旧plan_init_motion导入、已移除的加速度选项、旧初始角和旧指令序列断言）；已核对相关代码/配置/测试与HEAD一致，本轮未修改，未以改回位速度来迁就旧测试。日志：/tmp/a10_reset_recording_final.log、/tmp/a10_reset_existing_init_tests.log。

### 2026-11-13：双臂VR掉落报警提示策略

- 按用户要求，增加 `zx_grippers.drop_alarm_policy=warn/stop`，当前双臂配置默认warn；驱动其他使用场景默认stop。共享判断仅排除报警B的0x1000，所有其他报警位照常阻止新目标。
- 串口驱动、VR夹爪线程与录制采样统一策略；掉落位不锁存为夹爪故障，摇杆持续生成范围内目标。未改速度、力矩、目标提前量或厂家寄存器，不自动清报警、掉使能或重试不确定的写入。
- 头显短句提示掉落warning；原始报警码、策略、warning和实测位置保留在VR快照/录制sidecar，不把提示等同于成功抓取。
- 115项Python离线测试和JS头显测试通过；覆盖默认拦截、warn开合、混合报警仍拦截、无自动复位、原生LeRobot保存后诊断可读取。测试日志：`/tmp/zx_drop_policy_tests_v2.log`。
- 本轮未连接硬件；固件报警状态下是否实际接受并执行后续目标仍需现场验证。本变更并未关闭固件掉落检测。未改动用户CLI或同事的OpenPI远程适配文件。

### 2026-11-13：spike暂停与录制有效性解耦

- 用户选择“输入稳定后松开并重新按侧键接管”。复用runtime既有停止确认、双手释放/回中、稳定0.5秒、新按键及实际位姿重建锚点；不改运动恢复条件和reset权限。
- 双臂原生LeRobot录制新增默认开启的 `capture.continue_on_spike`。仅spike_pending/spike_reanchor锁定可继续有效实测采样，不让单次手柄跳变直接废弃整集；其他暂停/硬件故障仍按原逻辑处理。
- 每帧记录input_pause、control_phase=input_pause及真实采样间隙；暂停时清空诊断旧笛卡尔命令。仍采用用户原control_loop采样，不为停止等待补造帧。
- 头显黄色短提示input paused; recording，按键提示保持松手/回中/重新接管。文档明确reset在idle/recording/review可用，接管锁定期间不可用。
- 110项Python离线测试、JS头显测试通过；覆盖同集恢复保存、原生LeRobot诊断、混合故障、停止失败、时间间隙，以及原有锚点恢复运动测试。证据：`/tmp/a10_spike_recording_final_tests.log`。未连接设备或修改用户CLI/同事文件。

### 2026-11-13：知行夹爪VR电机速度提高到20%

- 用户明确确认两侧默认电机速度及VR上限从15%提高到20%；力矩60%、行程0～9000、目标速率3000raw/s、领先量600raw不变，设备检查仍为5%。
- 更新配置、输入边界及模拟线程下发测试、命令速查；未执行硬件运动。
- 检查初期发现其他任务的合并冲突，本轮未处理；提交前冲突已不在索引中，仅提交本轮5个相关文件，其他暂存与工作区修改保持原样。
- 验证：33项夹爪VR离线测试通过，包括20%下发、21%拒绝及力矩上限不变；日志 `/tmp/a10_zx_speed20_tests.log`。

### 2026-11-13：90Hz控制与30Hz采集分离（kaanh_vr_90hz）

- 独立工作树 `/home/robot/VLA/FluxVLA_90hz` 从已提交HEAD创建，不带入主工作树的模型/推理未提交修改。
- 双臂主循环/reference/UDP目标设90Hz；共享服务独立GET，默认30Hz，可单独调频。启动/停止确认继续使用新查询；缓存保留实际接收时间，重复反馈不算新的静止观测。
- 数据采集切回独立后台30Hz，相机fps明确30；图片入队不占用主循环状态锁，按钮事件排队且取消可及时处理。LeRobot数据字段、spike同集续录、m_init采集不变。
- `.150` 只读各5秒测量：30档实际29.88Hz、60档59.22Hz；GET均约11ms。完整runtime/reference/共享UDP/JSONL本地假设备测试约89.67Hz；另验证真实LeRobot写入时录制调用90Hz、采样30Hz。未做真机运动试验，不能据此宣称全套现场持续90Hz已验收。
- 操作、测量边界、报告位置见 `docs/a10_vr_frequency.md`；本分支使用新数据目录，旧目录不自动迁移或覆盖。

### 2026-11-13：反馈缓存完全后台化与过期接管锁定

- 在 `kaanh_vr_90hz` 中将本机RPC读取、JSON/状态解析移至独立后台线程；控制循环只取内存快照，原接收时间与序号保持不变。同序号不重复解析，后台旧回复不得覆盖同步启动/停止确认。
- 双臂反馈过期不再直接退出：请求停止、锁定接管、保留头显；停止后的新状态满足inactive/Stopped/Enabled、无故障，且操作者松手回中、输入和反馈稳定后，重新按侧键重建锚点。停止未确认则本会话永久锁定，不自动使能/重连/重放目标。
- 增加简短小写头显提示；保留原录制质量保护及废弃重录入口，真实故障不按普通过期自动恢复。
- 两个subagent分别实现后台缓存及恢复测试；282项相关回归+12项恢复测试、HUD JS和preview通过。本机20ms假GET下，内存读取均0.096ms；完整模拟链路约89.37Hz。未操作实机或修改用户CLI；详细边界见 `docs/a10_vr_frequency.md`。

### 2026-11-13：90Hz分支知行夹爪VR电机速度提高到50%

- 按用户要求将双臂VR两侧电机默认速度从20%提高到50%，VR整数校验范围同步扩展至1～50%；力矩60%、原始行程0～9000、目标变化速率3000raw/s、领先量600raw及其他故障处理保持不变。设备检查/标定入口仍为5%。
- 更新双臂配置、速度边界与模拟线程下发测试、命令速查。未修改用户CLI、未操作硬件；50%实际开合效果待现场验证，电机速度百分比不等于目标变化速率或实际运动速度线性增长。
- 验证：33项夹爪VR离线测试通过，包括50%下发、51%拒绝、力矩上限及默认设备检查速度不变；`git diff --check`通过。测试日志：`/tmp/a10_zx_speed50_tests.log`。

### 2026-11-16：核对VR轻载测试，明确后台采集负载验收范围

- 核对09:41/09:42两轮延迟报告及逐包记录：有效位姿轮约68秒，RTT中位8.20ms、最大32.80ms，收包后首次读取最大14.95ms；两次超过250ms的长间隔在浏览器发包时间轴已存在，恢复包伴随XR可见/会话状态变化，不能归因为Thor处理旧包过慢。
- 查明 `--check-record-buttons` 不启动相机、数据集writer或30Hz采样线程；context的 `recording=true` 仅表示加载录制配置。这轮不是采集资源竞争验收，纠正此前测试范围说明。
- 在 `docs/a10_vr_frequency.md` 记录原始路径、统计、长间隔时刻和后续完整采集30/90Hz对照要求：必须真实开始一集并核对capture_active/frames，不能仅打开头显界面；不新增硬件操作，不改同事和用户现有未提交文件。


### 2026-11-16：VR输入暂停期间保留同一集实测采集

- 新增并默认开启capture.continue_on_vr_pause，LeRobot后端对VR断流/输入过期/跟踪失效保留同一集，覆盖主循环before_step和后台采样先于runtime锁定的顺序；保持原continue_on_spike兼容。
- 机器人/夹爪/相机实测有效性检查不变；停止未确认、控制器故障、持续缺失实测数据仍阻止无效训练集发布。暂停帧保留原因、实测时间戳并清空旧下发目标；不自动开启新集或恢复接管。
- 录制HUD增加独立recording_state并保留真实失败原因，修正failed集被旧“recording”提示覆盖；VR暂停续录为黄色短句。详见a10_vr_recording.md第20节。
- 本次仅修改录制配置/录制协调/头显/相应测试及文档；现有CLI、坐标映射和其他未提交开发改动不纳入提交。未执行硬件动作。
- 验证：198项Python离线回归通过，覆盖断流同集保存、后台30Hz持续写帧、runtime发布前采样、旧策略开关、真实设备故障和停止未确认不放行，以及原生LeRobot保存、接管恢复、夹爪和复位；头显Node回归、配置preview、git diff --check通过。测试日志 `/tmp/a10_vr_stream_recording_tests.log`，现场效果待验收。

### 2026-11-16：头显状态文字透明显示并下移

- 移除录制状态HUD的黑色背景平面，仅显示原有彩色状态文字与功能键提示。状态区域、延迟文字和头显信息整体下移0.12个场景单位，保持字体大小、显示距离和各区域间距。
- 修改位置为 `fluxvla/engines/operators/vr/web-ui/index.html`；刷新头显网页后生效。通过现有头显Node回归及HTML结构检查；未进行头显现场视觉验收。


### 2026-11-16：统一录制、双臂推理和90Hz遥操作到kaanh_follower

- 以recording分支bf97d26与90Hz分支d05a595为输入，在临时工作树整合；共同历史不重复，保留双臂推理、原生LeRobot录制、90Hz后台反馈、世界坐标遥操作和图形化测试台。
- 按用户最新要求，CLI原样采用90Hz工作区最新版本，不重排或混入旧CLI；主目录原CLI和三个分支均有备份。数据/权重/日志保持原位置，90Hz工作树保留供现场验收。
- 保留最新参数；修正旧双手方向回归中写死0.9倍率的测试预期。VR385项及6个入口预览通过；旧m_init测试的6处失败已在合并前源提交复现，不修改归位逻辑。完整测试、备份、本地TLS和执行期间新增但未纳入的保存优化见 `docs/a10_branch_integration_20261116.md`。

### 2026-11-16：修复录制归位后无法重新接管的问题

- 现场日志 `work_dirs/a10/vr/dual_world_translation_diagnostics.jsonl` 显示：15:24:00归位结束时控制器仍为Cart/Running/Enabled；15:25:50保存完成、15:25:52第二集开始后，27次接管被 `await_controller_ready` 拒绝。手柄有效输入仍收到，第二集有效实测采样仍继续；不是保存关闭了VR或控制器报警。
- 原因：通用m_init结束时恢复调用前已开启的笛卡尔跟随，录制入口却清空本地锚点；新锚点流程要求inactive + Stopped，因此卡在接管条件。仅修正录制入口的复位工作线程：通用m_init返回后，停止其恢复的跟随并确认停止，再交回VR；下一次松手重新按键时正常建立新会话和累计增量编码。
- 保留控制台m_init恢复原状态的行为、自动下一集、原始数据与用户CLI；停止失败仍进入reset_failed并锁住运动，不自动接管已有跟随。等待在后台复位线程执行，未finish的本集继续录实测数据。
- 验证：录制工作流/世界坐标遥操28项、VR录制/双臂runtime/原生LeRobot124项离线测试通过，覆盖复位恢复跟随、停止交接、保存后自动下一集与停止失败不放行；`git diff --check`通过。测试记录 `/tmp/a10_record_reset_handoff_tests.log`、`/tmp/a10_record_reset_handoff_regression.log`。未发送实机指令，需正常退出旧进程、重启原录制命令后现场验收。

### 2026-11-16：合入90Hz分支的数据保存提速

- 合入 `kaanh_vr_90hz` 已提交的 `d554e62`：恢复LeRobot原生三路视频并行编码，入口在设备线程启动前选择spawn；跨集复用dataset和图片写入器，退出时统一停止写入器并finalize，增加创建、续录、保存和退出耗时日志。
- 数据格式、AV1编码参数和采集命令兼容；保存完成后继续下一集。数据集正在被采集进程持有时，外部读取可能尚缺Parquet收尾信息，验证和训练应等待正常退出完成。
- 保留本分支的归位后接管修复；两边未提交的CLI、90Hz工作区新控制台开发均未纳入。合入前备份分支为 `backup/kaanh_follower-before-save-speedup-20261116`；用户CLI已保存补丁和校验值并核对一致。
- 来源提交的三路640×480、1800帧离线对照为原生29.84秒、适配层28.83秒；该记录不是本次实机负载测试，也不保证现场固定保存时间。
- 本次合并验证：原生LeRobot读写/连续多集复用与退出收尾、录制状态机、世界坐标遥操和双臂runtime共154项离线测试通过；录制配置preview及diff检查通过，用户CLI校验一致。测试日志 `/tmp/fluxvla_save_speedup_merge_tests_20261116.log`，预览记录 `/tmp/fluxvla_save_speedup_preview_20261116.log`。未连接设备；重启采集程序后加载新保存流程。

### 2026-11-16：双臂采集只保留明确保存成功的集

- 按用户确认移除原生LeRobot的recover参数及双臂配置项。正常退出丢弃当前未保存集；意外退出后resume自动清理残留，不归档、不覆盖为新集、不自动发布。停止采集改成丢弃当前集并退出；保存中的明确请求先等待完成。
- 保存开始前记录旧元数据和文件清单；保存成功完成Parquet收尾，下一集使用新文件。dataset和图片写入器仍复用、三路视频仍并行编码，避免下一集失败损坏已保存集共用文件；失败或退出时回退本集元数据和新增文件。旧版本共享文件已经损坏时仍拒绝猜测删除。
- 补充真实LeRobot保存失败回滚、前一集视频字节不变、同编号重录、模拟进程中断后无需recover续录、连续多集及关闭时丢弃测试。只操作临时合成数据，未发送实机指令；原始CLI和用户遥操参数的其他修改未纳入提交。
- 验证：原生写入器及录制17项、更新后的原生停止行为/工作流/录制回归65项通过；配置preview、diff检查通过。记录位于 `/tmp/a10_discard_unsaved_tests.log`、`/tmp/a10_discard_final_tests.log`。保存提速的并行编码保持开启；新增每集文件收尾的现场耗时待实测。
