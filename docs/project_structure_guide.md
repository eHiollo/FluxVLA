# FluxVLA 项目结构入门说明

适用版本：`kaanh_follower_vr` 分支，核对日期：2026-10-16。

这份文档帮你回答三个问题：每个目录做什么、程序怎样串起来、要改功能应该找哪里。
先读第 1、2 节，再按需要查目录即可；不用从头读完全部源码。

当前 VR 工作区是 `/home/robot/VLA/FluxVLA_vr`，主目录 `/home/robot/VLA/FluxVLA` 是另一个分支的工作区。
两个目录有不同的当前代码和本地文件，修改前先确认 IDE 打开的目录和 Git 分支。
以下链接均相对于本仓库，换一台机器仍可点击。

## 1. 先把几种角色分清楚

VLA 是“视觉、语言、动作”的缩写：模型结合画面、任务描述和机器人状态，预测动作。

| 角色 | 可以怎样理解 | 在本项目中的例子 |
| --- | --- | --- |
| 配置 config | 本次运行的参数表 | IP、端口、相机、权重路径、运行步数 |
| 入口 script | 终端按下的启动按钮 | `scripts/teleoperate.py`、`scripts/inference.py` |
| 流程 runner / runtime | 安排每一步先后顺序的调度员 | 读输入、算目标、发送、下一轮、退出清理 |
| 设备接口 operator | 与设备通信的接口 | 读取相机、连接控制器、发送关节目标 |
| 数据处理 transform / processor | 转换数据格式的小工具 | 图像缩放、动作归一化、手柄坐标转换 |
| 模型 model | 根据输入计算预测的部分 | PI0.5、GR00T 等 |
| 日志、数据和权重 | 程序读取或生成的文件 | JSONL 日志、采集视频、模型参数 |

`base_*.py` 一般是共用模板，具体机器人或模型在其基础上补充差异。
`__init__.py` 通常用于声明 Python 包、导出类或触发注册，帮助其他文件找到模块。
“注册器”是一张名称表；配置写 `type='A10Operator'`，框架就能找到并创建对应类。

三条主要链路如下：

```text
遥操作：手柄 → VR输入和映射 → runtime → 机器人接口 → 控制器
                                      ↑                  ↓
                                      └──── 实际状态 ────┘

推理：相机＋实际状态＋任务描述 → 数据处理 → 模型 → 动作处理 → 机器人接口
               ↑                                               ↓
               └──────────────── 新的反馈 ──────────────────────┘

训练：已有数据集 → 读取与处理 → 组成一批数据 → 模型训练 → 保存权重
```

图里的“采集反馈”不等于“已经完成数据集录制”。当前新 VR 流程可以记录诊断日志，完整录制联动仍待开发。
FluxVLA 的接口发送目标，底层运动规划与执行由控制器负责。仿真是否能替换实机，取决于接收端是否实现相同协议和反馈，不能只凭目录里有仿真代码判断。

## 2. 根目录：先认识这些入口

| 目录 | 功能 | 什么时候看 |
| --- | --- | --- |
| [configs/](../configs/) | 各模型和工程的运行参数 | 换设备地址、权重、步数、模型或数据路径 |
| [fluxvla/](../fluxvla/) | 框架核心 Python 代码 | 增加机器人、输入方式、模型和数据处理能力 |
| [scripts/](../scripts/) | 训练、推理、遥操作、评测与环境启动脚本 | 从终端运行工程 |
| [tools/](../tools/) | 数据转换、检查、统计和结果整理工具 | 检查数据能否用于训练，或整理实验结果 |
| [test/](../test/) | 自动化测试代码和样例 | 验证代码修改，有些测试使用模拟设备 |
| [docs/](./) | 使用教程、设计说明和开发记录 | 查命令、设计依据、进度 |
| [assets/](../assets/) | README 等文档的图片 | 修改项目介绍与展示图 |
| [datasets/](../datasets/) | 约定存放本地数据集的位置 | 找采集数据、视频和元数据 |
| [checkpoints/](../checkpoints/) | 约定存放模型权重的位置 | 加载训练或推理模型 |
| `work_dirs/` | 运行时生成的日志、报告和测试产物 | 排查问题、找输出文件；通常不提交 Git |
| [local_setup/thor/](../local_setup/thor/) | 本机 Thor 的环境记录 | 查历史依赖与环境准备说明 |
| [.github/](../.github/) | GitHub 协作配置 | 自动测试和问题反馈模板 |
| `.git` | Git 的版本与分支信息；独立工作区中可能是指向主仓库的文件 | 由 Git 命令管理，日常开发无需编辑 |

`datasets/`、`checkpoints/` 在 Git 中主要保留占位文件；实际的大文件要在本机另外准备。
当前 VR 工作区没有完整模型与数据资产，主目录有文件不代表这个工作区也有。
`work_dirs/` 由实际运行创建，目录数量会随运行增加；清理前区分诊断日志、正式数据和训练结果。

根目录还有几类文件：

- `README*.md`：项目总介绍；`CHANGELOG.md`：变更摘要。
- `requirements*.txt`：依赖清单；base、real、sim、a10、test 分别对应不同用途，并非每次全部安装。
- `setup.py`：Python 包安装、网页资源打包和本机计算扩展构建配置。
- `setup.cfg`、`pytest.ini`、`.pre-commit-config.yaml`：代码检查与测试配置。
- `.clang-format`、`.clang-tidy`、`.cpplint`：主要约束 C/C++ 代码格式和检查。
- `.gitignore`：告诉 Git 哪些本地文件不纳入版本管理；`LICENSE`、`CITATION.cff`：许可证和引用信息。
- 根目录附带的 PDF/压缩包可能是说明书或参考材料，不是程序启动入口。

## 3. 核心代码目录逐层看

### 3.1 `fluxvla/engines/`：运行工程的基础设施

| 子目录 | 功能 | 代表文件或使用场景 |
| --- | --- | --- |
| [operators/](../fluxvla/engines/operators/) | 机器人、相机、夹爪及 VR 接口 | `a10_operator.py`、`camera_operator.py`、`gripper_operator.py` |
| [runners/](../fluxvla/engines/runners/) | 模型推理、训练和评测流程 | `a10_inference_runner.py`、`base_train_runner.py` |
| [runners/serving/](../fluxvla/engines/runners/serving/) | 将模型开放成可请求的服务 | ZMQ、ROS/ROS2 服务与请求数据编码 |
| [runners/serving/proto/](../fluxvla/engines/runners/serving/proto/) | 服务消息格式定义及生成代码 | protobuf 文件；通常不直接改生成文件 |
| [processors/](../fluxvla/engines/processors/) | 封装模型配套的数据处理器 | 当前主要是 Transformers `AutoProcessor` 的包装 |
| [utils/](../fluxvla/engines/utils/) | 多个模块共用的工具 | 配置构建、注册器、日志、权重、图像和 RTC 辅助 |
| [losses/](../fluxvla/engines/losses/) | 共用损失计算辅助 | 例如训练样本加权；部分损失也在模型内部计算 |
| [metrics/](../fluxvla/engines/metrics/) | 训练或预测的指标记录 | 用数值观察模型表现 |

**operators 中的 VR 目录**：

| 文件或目录 | 功能 |
| --- | --- |
| [vr_operator.py](../fluxvla/engines/operators/vr_operator.py) | 小型遥操作基类和 VROperator，提供连接、读取映射动作、事件、反馈和关闭接口 |
| [vr/runtime.py](../fluxvla/engines/operators/vr/runtime.py) | 组装 VR 与 A10，运行循环，处理 follower 启停、锚点、保持和清理 |
| [vr/control.py](../fluxvla/engines/operators/vr/control.py) | 新迁入的锚点与参考目标计算，供离线回放和 reference 模式调用；实机效果待验收 |
| [vr/input.py](../fluxvla/engines/operators/vr/input.py) | 从服务取得最新手柄数据 |
| [vr/mapper.py](../fluxvla/engines/operators/vr/mapper.py) | 手柄数据转换为末端增量，包括缩放、精调、坐标映射和输入检查 |
| `vr/monitor.py`、`vr/ws_server.py`、`vr/server_config.py` | 网页/WSS 服务、消息解析、网络和证书设置 |
| `vr/vr_events.py`、`vr/input_types.py`、`vr/quaternion_utils.py` | 按键事件、内部消息类型、姿态计算 |
| `vr/diagnostics.py`、`vr/hud.py` | 保存诊断、生成头显状态信息；完整录制 HUD 尚待接通 |
| [vr/web-ui/](../fluxvla/engines/operators/vr/web-ui/) | 浏览器页面、JavaScript 和样式；`media/` 放页面图片 |
| `vr/NOTICE.md`、`vr/LICENSE.*` | 已迁入代码的来源及许可证，需要随代码保留 |

**runners 中的文件命名**：

- `base_*`：公共能力；`a10/aloha/ur/franka/tron2/oli_*`：各机器人推理差异。
- `*_rtc_*`：动作块衔接能力；RTC 会利用上一段剩余动作，具体版本的同步/异步方式不同。
- `ddp_train_runner.py`、`fsdp_train_runner.py`：不同分布式训练方式。
- `libero_eval_runner.py`、`robocasa_eval_runner.py`：对应仿真任务的评测。
- `fluxbisim_*`：FluxBiSim 的推理接入；不是现成的 A10 仿真控制器。

本工程选择让 **VR 的循环集中在 `vr/runtime.py`**，模型推理继续走 runners。
所以找 VR 的握键或保持逻辑时，应该先打开 runtime。

**utils 中先认识三个搭建工具和一个日志工具**：

- `root.py` 创建分类注册表，`registry.py` 实现登记，`builder.py` 根据配置构建对象。
- `overwatch.py` 管理日志；`name_map.py` 把 `fp32` 等文字转成程序类型。
- `checkpoint_utils.py`、`download_utils.py`、`hf_hub.py` 处理权重加载辅助、下载和缓存路径。
- `image_ops.py`、`robot_utils.py`、`trajectory_utils.py` 提供图像、夹爪和动作序列处理。
- 其余工具用于模型计算、PyTorch 环境、RTC、仿真和评测结果保存/上报，按调用需要查看。

### 3.2 `fluxvla/models/`：模型本身

| 子目录 | 功能 | 可以怎样理解 |
| --- | --- | --- |
| [vlas/](../fluxvla/models/vlas/) | 组装完整 VLA 模型或推理适配 | 模型的整体结构与预测入口 |
| [backbones/](../fluxvla/models/backbones/) | 提取图像、文字等输入特征 | 模型前面的基础模块 |
| [backbones/llms/](../fluxvla/models/backbones/llms/) | 语言模型模块 | 理解文字和序列信息 |
| [backbones/visions/](../fluxvla/models/backbones/visions/) | 视觉模型模块 | 从图片提取信息 |
| [backbones/vlms/](../fluxvla/models/backbones/vlms/) | 视觉语言模型模块 | 联合处理画面和文字；其中 `cosmos3/` 集中相关模型细节 |
| [heads/](../fluxvla/models/heads/) | 动作等输出模块 | 根据模型特征产生所需输出 |
| [projectors/](../fluxvla/models/projectors/) | 不同特征尺寸或表示之间的转换 | 模型模块间的连接器 |
| [blocks/](../fluxvla/models/blocks/) | 共用计算模块 | 可被多个模型使用的积木 |
| [compat/](../fluxvla/models/compat/) | 特定依赖版本兼容处理 | 例如 Qwen3-VL 的版本差异 |
| [third_party_models/](../fluxvla/models/third_party_models/) | 集成的模型实现或配套代码 | 需要按来源和许可证维护 |

`third_party_models/` 下的 `cosmos3/`、`dreamzero/`、`eagle2_hg_model/`、`fastwam/`、
`qwen3_tokenizer/`、`xvla_models/` 分别集中对应模型或 tokenizer 的内部代码。
它们还有较深的模块层级，先按模型整体看；接 A10 或改 VR 时通常无需逐层修改。

当前 A10 的关键文件是 `vlas/openpi_a10.py`（加载转换权重）和
`vlas/a10_pi05_inference.py`（适配加速推理）。它们都不是机器人通信驱动。

### 3.3 `fluxvla/` 中与训练数据有关的目录

| 目录 | 功能 | 什么时候看 |
| --- | --- | --- |
| [datasets/](../fluxvla/datasets/) | 读取数据并组织样本，也包含在线推理输入适配 | 视频、状态、动作怎样送给模型 |
| [datasets/utils/](../fluxvla/datasets/utils/) | 视频解码、统计量等共用功能 | 排查解码和数据统计问题 |
| [transforms/](../fluxvla/transforms/) | 图像、文字、状态和动作的转换 | 归一化、格式或模型动作解释有问题时 |
| [tokenizers/](../fluxvla/tokenizers/) | 将文本、动作或某些视频表示编码成模型使用的形式 | 查模型输入编码方式 |
| [collators/](../fluxvla/collators/) | 把多个样本合成一批，补齐长度等 | 一批数据的形状不一致时 |
| [optimizers/](../fluxvla/optimizers/) | 更新模型参数及安排学习率 | 调整训练更新方式 |
| [weighters/](../fluxvla/weighters/) | 决定训练样本对损失的贡献大小 | 希望某类样本更受重视时 |
| [evaluators/](../fluxvla/evaluators/) | 决定训练中什么时候评估、怎样执行评估 | 设置训练过程中的检查；独立仿真评测在 runners |

特别注意：根目录 `datasets/` 放**数据文件**，`fluxvla/datasets/` 放**读取数据的代码**。
同样，`checkpoints/` 放权重文件，`models/` 放模型实现；两者都准备好才能运行模型。

### 3.4 `fluxvla/ops/`：加速计算

- [cuda/](../fluxvla/ops/cuda/)：需要编译的计算扩展。`gemma_rotary_embedding/`、`rotary_pos_embedding/` 处理模型位置编码相关计算，`matmul_bias/` 处理矩阵乘法和偏置。
- [triton/](../fluxvla/ops/triton/)：GPU 计算内核，包括矩阵、注意力、归一化和位置编码等。

这部分影响模型计算速度，与机器人 UDP 网络收发是两件事。
本机生成的 `.so` 是编译产物，换机器或换环境时不保证可以直接复用。

### 3.5 `fluxvla/projects/a10/`：尚在迁移中的历史工程

新 VR 入口不依赖该目录，但整个目录还不能直接删除：

| 文件组 | 当前作用 |
| --- | --- |
| `config.py`、`cli.py`、`__main__.py` | 旧统一 YAML 配置和命令入口 |
| `follower.py`、`transport.py`、`cameras.py` | 旧控制服务协议与采集封装 |
| `session.py`、`record.py`、`recording.py` | 旧遥操作录制、分段保存等；session 已改为引用新 VR 输入 |
| `training.py`、`online_dataset.py`、`transforms.py` | 旧 A10 训练配置生成、数据与转换 |
| `serve.py`、`replay.py`、`bridge.py`、`hardware_check.py` | 旧服务、回放、推理桥接和现场检查 |
| `teleop.py` | 已停用的旧遥操作入口，会提示新命令 |

`configs/a10/pi05_a10_finetune.py` 目前仍调用这里的 `training.build_config()`。
后续功能应接入原生结构；清理旧目录前，需要逐条替换这些历史依赖。

## 4. 配置、工具和测试的子目录

### 配置 `configs/`

| 子目录 | 用途 |
| --- | --- |
| [a10/](../configs/a10/) | 我们的 A10：VR、本地/远程推理、训练和历史 YAML |
| `pi0/`、`pi05/`、`gr00tn15/`、`gr00tn17/`、`smolvla/` | 对应模型的训练、推理或评测配置 |
| `cosmos3/`、`dit4dit/`、`dreamzero/`、`fastwam/` | 其他模型方案的配置 |
| `llava/`、`openvla/`、`openai/` | LLaVA、OpenVLA 及 OpenAI 接口方案配置；用到哪种再查看 |
| `arm/`、`sarm/` | 奖励/进度模型等相关配置 |
| `pi05/fluxbisim/`、`gr00tn15/fluxbisim/` | 对应模型在 FluxBiSim 中使用的配置 |

模型配置子目录名不等于当前正在使用的功能，需要看启动命令的 `--config`。
A10 里先看三份：`a10_vr_teleop.py`、`a10_openpi_inference.py`、`a10_remote_inference.py`。
`project.yaml`、`site.example.yaml` 服务于历史流程，不会自动影响新 VR Python 配置。

### 工具 `tools/`

顶层主要是数据格式转换、归一化统计和仿真评测汇总，例如
[validate_a10_dataset.py](../tools/validate_a10_dataset.py) 用于检查 A10 数据集。

- `arm_awbc/`：ARM 相关进度重建等辅助工具。
- `sarm_annotate/`：阶段标注、解析和写入工具；`examples/` 放示例阶段配置。
- `sarm_rabc/`：SARM 重加权工具的包目录，当前仅有初始化占位文件。

### 测试 `test/`

| 子目录 | 检查对象 |
| --- | --- |
| `test_configs/` | 配置能否正确构建 |
| `test_datasets/` | 数据读取、视频、录制及共享流程 |
| `test_engines/` | 框架运行辅助功能 |
| `test_models/`、`test_cosmos3/` | 通用及 Cosmos3 模型行为 |
| `test_runners/` | 推理、评测等运行流程 |
| `test_optimizers/`、`test_tokenizers/`、`test_transforms/` | 对应训练更新、编码与转换 |
| `data/` | 测试样例；其 `datasets/`、`models/`、`transforms/` 按被测功能分类 |

`test/data/models/` 下的 `heads/llm_backbones/vision_backbones/vlas/vlm_backbones` 是不同模块的测试素材；
`test/data/datasets/rlds_dataset/` 和 `test/data/transforms/rlds_transform/` 是 RLDS 格式相关测试样例。
顶层还有 `test_vr_operator.py`、`test_vr_native_entry.py`、`test_a10_operator.py` 等专项测试。
测试通过说明对应的软件契约通过；模拟设备测试通过不代表实机已经验收。

### 其他目录

- `.github/workflows/`：自动执行代码检查和单元测试；`.github/ISSUE_TEMPLATE/`：问题和功能申请模板。
- `local_setup/thor/`：本机历史安装记录，日期较早时以实际环境为准。
- `__pycache__/`、`.pytest_cache/`、`build/`、`*.egg-info/` 等可能由 Python、测试或安装生成，不是功能源码目录，存在与否取决于本机运行情况。

## 5. 跟着 A10 的文件走一遍

### 5.1 VR 遥操作：从命令到控制器

先做不启动服务的配置检查：

```bash
cd /home/robot/VLA/FluxVLA_vr
conda activate fluxvla
python scripts/teleoperate.py --config configs/a10/a10_vr_teleop.py --preview
```

实际顺序是：

1. [scripts/teleoperate.py](../scripts/teleoperate.py) 读取配置和终端覆盖参数。
2. [a10_vr_teleop.py](../configs/a10/a10_vr_teleop.py) 提供网络、映射、设备和限值。
3. [vr/runtime.py](../fluxvla/engines/operators/vr/runtime.py) 的 `build_runtime()` 构建输入和所需设备，默认 VR-only 不构建机器人。
4. [vr_operator.py](../fluxvla/engines/operators/vr_operator.py) 组合输入与 mapper，提供映射动作和事件。
5. 正式运行时，`runtime.run()` 循环调用这些接口；只有显式开启且通过检查，才调用 [A10Operator](../fluxvla/engines/operators/a10_operator.py) 发送目标。
6. 诊断日志默认写入本工作区 `work_dirs/a10/vr/teleop_diagnostics.jsonl`，`--preview` 不启动循环也不写这些帧。

默认 VR-only 的 `feedback` 和 `sent_relative_target` 为空是正常现象。
JSONL 是逐行 JSON 日志，可用于排查输入，但不是完整的可训练演示数据集。
完整参数与现场操作见 [VR 使用说明](a10_vr_operator.md)。

### 5.2 模型推理：谁决定动作、谁负责发送

```text
scripts/inference.py + configs/a10/a10_openpi_inference.py
        ↓
A10InferenceRunner：取观测、调用模型、检查并执行动作块
        ↓
datasets/a10_openpi_inference.py：处理当前观测
        ↓
models/vlas/openpi_a10.py：加载权重并构建配置指定的模型
models/vlas/a10_pi05_inference.py：当前 A10 加速模型适配
        ↓
transforms/a10_openpi.py：反归一化，转成关节/夹爪目标
        ↓
A10Operator：逐帧发送关节目标与夹爪指令，读取反馈
```

以上短路径都是 `fluxvla/` 下的代码路径。
VR 当前发送末端位姿，模型推理发送关节目标；它们不是同一种七维数据。

远程推理时，客户端使用 `a10_remote_inference.py`，把观测发到模型服务；
服务通常由 `python -m fluxvla.engines.runners.serving.serve` 启动。
服务使用本地模型配置加载权重，返回预测动作，机器人客户端再负责检查和执行。
只启动模型服务不会自动让机械臂运动。推理资料见 [现场说明](a10_onsite_guide.md)。

### 5.3 训练与采集：怎样接到推理

```text
演示数据 → 数据集代码读取 → transforms处理 → collator组成一批
        → 训练runner调用模型并更新参数 → 保存checkpoint
        → 推理配置指定该checkpoint → 推理runner运行
```

`scripts/train.py` 是训练入口，配置选择 dataset、model、runner 等组件。
A10 训练配置目前还有第 3.5 节的历史依赖，使用前先核对配置和数据约定。

当前 A10 模型状态是六个关节角加夹爪；训练目标如何从状态构造由数据处理配置决定。
旧遥操作录制器另外保存末端增量等动作，不能仅看“都是动作”就互换。
完整采集还需要图像、实际关节和夹爪反馈，以及保存/重录等录制流程；新 VR 的 JSONL 尚未替代它。

## 6. 想改什么，先去哪里找

| 想做的事 | 优先查看 |
| --- | --- |
| 换 VR 网页地址、端口或运行频率 | `configs/a10/a10_vr_teleop.py` |
| 调整手柄缩放、精调、死区 | 同一配置中的 mapping，再看 `operators/vr/mapper.py` |
| 修改握键、保持、目标锚点或退出流程 | `operators/vr/runtime.py` |
| 修改网页或头显显示 | `operators/vr/web-ui/`、`hud.py` 和反馈调用处 |
| 排查手柄数据异常 | VR JSONL，再沿 input → mapper → runtime 查看 |
| 换机器人地址、相机或夹爪端口 | 对应运行配置，再看 A10、相机、夹爪 operator |
| 修改控制器命令或状态解析 | `operators/a10_operator.py`，对照协议与专项测试 |
| 修改模型动作执行步数、关节限值 | A10 推理配置及 `runners/a10_inference_runner.py` |
| 修改模型输入或输出含义 | `datasets/a10_openpi_inference.py`、`transforms/a10_openpi.py` |
| 更换模型或权重 | 配置和 `models/vlas/`；确认 tokenizer 与统计文件也匹配 |
| 增加机器人 | 先实现 operator 契约，再适配对应流程和配置 |
| 接通正式 VR 数据录制 | 先核对旧 recording/session 的能力，再接新 runtime 与原生数据保存位置 |

推荐阅读顺序：**配置 → 启动脚本 → runtime/runner → operator → 数据处理与模型**。
每次修改后运行与改动相关的测试；开发进度记录见 [A10 迁移计划](a10_full_migration_plan.md)。
