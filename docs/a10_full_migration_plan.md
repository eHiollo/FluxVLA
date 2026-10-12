# A10 整套 VLA 迁移开发记录

## 目标和边界

最终交付是一个独立的 FluxVLA 平台：VR、相机、录制、数据处理、训练、推理和开发记录均位于 FluxVLA 内，正常运行不依赖旁边的 LeRobot、XLeRobot 或 OpenPI 仓库及其 Python 环境。现有模块中经验证的必要实现按功能迁入，保留来源和许可证；平台中已有的数据读取、变换、统计、训练 runner、注册器及 ZMQ 推理服务优先复用。

A10_new 暂时是底层运动控制后端，通过协议适配器连接。未来新的 follower 接管逆运动学和运动规划，平台通过目标姿态／位姿接口发送四元数等目标；当前关节批次接口与未来姿态接口分别声明能力，禁止把旧关节动作直接当作四元数。录制与 VR 输入为 30 Hz，当前 A10 底层控制为 500 Hz。

2026-10-09 补充边界：单平台指采集管理、数据处理、训练、推理和通信适配统一在 FluxVLA。控制器侧可以有独立部署的仿真或实机执行服务；它们共同提供目标接收和状态／图像反馈接口。仿真场景与底层控制不迁入 FluxVLA。仅变更接收端的前提是动作、观测、单位、坐标系和时序契约相同，当前尚未满足全部条件。

原 OpenPI 项目仍由团队调试。2026-10-12 已接入其合并 LoRA 的 PyTorch 导出：严格检查 metadata 和 811 个参数映射，确认 horizon=30。FluxVLA 本地与 ZMQ 推理一致；原 JAX 的逐数值对照、任务质量仍未验收。

## 当前进度（2026-10-12）

`kaanh_follower` 在 `/home/robot/VLA/FluxVLA_integration` 完成已提交的原生推理适配迁入，以及相机/USB 夹爪组合、六轴动作块逐帧发送。正式链路为 `scripts/inference.py → A10InferenceRunner(BaseInferenceRunner) → A10Operator(BaseOperator)`，使用原有注册器、dataset/transform 和 ZMQ 服务；不反向依赖 `projects/a10` 或其他业务仓库。

已完成软件模拟闭环和真实 GPU 权重的本地/ZMQ 离线推理。默认只读 dry_run，没有操作真机、SSH 或正式训练。下一步是现场核对两路相机/USB 路径、读取观测、验证动作顺序/单位和限制，再分步验收真机运行及失联行为。VR 未迁入本轮，控制器运动规划和双臂/人形仍为后续工作。

下方 2026-10-09 方案和历史开发记录保留用于追踪变化；旧 TCP batch 控制不是本轮原生 follower 的运行入口。当前软件接口和操作命令见 [现场说明](a10_onsite_guide.md)。每个小任务分别提交，详细进度在本文末尾。

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
