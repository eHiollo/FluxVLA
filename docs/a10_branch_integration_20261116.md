# 2026-11-16：录制与90Hz开发统一到kaanh_follower

> 当前入口：`configs/a10/a10_dual_vr.py`，配置分工见[配置索引](../configs/a10/README.md)，运行命令见[CLI.txt](../CLI.txt)。下文保留当时的接入/开发记录，旧路径和双臂默认行为以当前入口为准。

## 合并范围

- 目标：`kaanh_follower`，合并前为 `d3d093a`。
- 录制与双臂推理：`kaanh_lerobot_recording`，末端 `bf97d26`。
- 90Hz与世界坐标遥操作：`kaanh_vr_90hz`，末端 `d05a595`。
- 共同历史只保留一份。通过一次合并提交保留两边开发记录；普通锚点模式和世界坐标平移模式均保留，不统一替换算法。

包含原生LeRobot录制、手柄录制菜单、录制中归位、异常输入同集实测续录、夹爪策略、90Hz主循环与独立30Hz采样/反馈、后台缓存、世界坐标平移、图形化测试台、透明下移头显，以及recording分支新增的OpenPI/FluxVLA双臂推理入口。

## 使用位置与入口

统一从 `/home/robot/VLA/FluxVLA` 的 `kaanh_follower` 分支运行，环境继续使用 `conda activate fluxvla`。按用户最新要求，根目录 `CLI.txt` 原样使用90Hz分支最新版本，字节一致；不重排、不合入旧CLI、不修改其中路径或参数。

- 普通双臂遥操作：`configs/a10/a10_dual_arm_vr_teleop.py`。
- 世界坐标平移遥操作：`configs/a10/a10_dual_arm_vr_teleop_world_translation.py`。
- 两者均可配合 `--record-config configs/a10/a10_dual_arm_record.py`；数据根目录用 `--record-options record.root=...` 指定。
- 图形化测试台：`python scripts/preview_vr_mapping.py`，不连接真实机器人或夹爪。
- 双臂推理：`configs/a10/a10_dual_fold_inference.py` 与 `configs/a10/a10_dual_openpi_remote.py`，默认preview；模型与实机模式说明见 `a10_dual_fold.md`。

保留源分支中的最新运动/映射参数（包括基础平移倍率1.8）。CLI中的旧目录与历史覆盖参数也原样保留；若要从统一主目录运行，手工先 `cd /home/robot/VLA/FluxVLA` 并执行所需命令。旧双手方向测试改为依据配置倍率计算预期距离，方向检查保留。

## 本地文件与回退

分支与原CLI备份位置：`/home/robot/VLA/.merge_backups/follower_20261116_150012`。三个原分支已建立 `backup/*-before-unify-*` 引用。主目录未提交的CLI原文与补丁完整备份，按用户最新要求不混入本次CLI；`FluxVLA`空文件和`知行.zip`不纳入提交。

现有数据集、权重、日志不移动、不覆盖。90Hz目录中的 `datasets/cloth_folding_01`、`datasets/dual_90hz_test`仍在原位置，主目录运行时可用绝对 `record.root` 继续指定原目录；新数据可存到主目录datasets下。模型文件继续使用主目录已有checkpoints。

主目录保留已有站点和姿态标定文件；世界坐标模式不加载旧绝对姿态标定。为延续近期头显信任的TLS身份，先备份主目录原cert/key，再复用90Hz目录的TLS文件，私钥不进入Git。

保留 `FluxVLA_90hz` 工作树供回退及现场比较。本轮只进行离线测试/预览，不启动真实遥操作、模型服务或发送机器人/夹爪命令。已有旧进程不会自动重载合并代码，应正常退出旧客户端后从主目录启动；不要同时运行两份VR服务。

## 验证与限制

合并无文本冲突。临时工作树复用主目录已编译的三个CUDA扩展（被Git忽略），避免将本机编译产物加入仓库。验证记录：

- VR全套385项unittest通过；头显状态/录制菜单与浏览器延迟两套Node检查通过。
- 普通双臂、世界坐标+录制、旧单臂、图形化测试台、FluxVLA/OpenPI双臂推理共6个入口preview通过，未连接设备或启动模型服务。
- 双臂训练配置用本机已有OpenPI统计文件加载；验证14维关节/夹爪增量往返、三相机和旧单臂变换。未运行训练、真实模型推理或实机运动。
- 扩展核心套件225项中，m_init旧套件出现4个failure和2个error；在独立工作树的原始d05a595上复测该套件11项，出现完全相同的6个问题。涉及旧归位角度、旧速度/启停指令断言、已移除的init_motion选项及模拟断线预期；相关实现与源分支一致，本次不改归位语义。除该旧套件外，核心214项回归通过（1项模型包相关测试因临时工作树不含模型包跳过），日志为 `/tmp/fluxvla_unify_core_tests_verified.log`。
- 主目录切回kaanh_follower后，20项双臂推理适配测试全部通过（无跳过），包括本机现有模型包的预处理与训练管线一致性验证；世界坐标+录制preview通过。日志为 `/tmp/fluxvla_unify_main_inference_tests.log` 与 `/tmp/fluxvla_unify_main_preview.log`。
- 日志：`/tmp/fluxvla_unify_vr_tests.log`、`/tmp/fluxvla_unify_core_tests.log`、`/tmp/fluxvla_unify_init_baseline.log`、`/tmp/fluxvla_unify_previews.log`、`/tmp/fluxvla_unify_training_config.log`。

执行期间90Hz工作树又出现视频保存优化的未提交改动（`lerobot_episode_writer.py`、`scripts/teleoperate.py`、`test_lerobot_recording.py`及对应录制文档）。本轮以审核时的d05a595为边界，保留这些新改动在原工作树，未代为提交或纳入；后续完成后可单独合并。CLI仍按用户最新要求从90Hz工作区原样复制并核对SHA256。

本轮不改变视频保存策略；三路串行AV1编码导致保存较慢的问题仍待后续优化。控制器GET偶发超时仍需现场按日志验证，合并不代表已修复控制器端行为，也不代表真实设备持续90Hz或模型任务效果已验收。
