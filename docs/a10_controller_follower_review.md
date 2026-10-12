# A10 控制器仿真与 follower 接口核查

日期：2026-10-09。初次范围：本机 A10_new（提交 `c27a475`）及用户提供的 `robot.7z`，仅静态阅读和内存报文验证。随后经用户授权，以 `kaanh` 登录工控机做 SSH 只读核查，结果见下方新增章节。全程没有连接机器人业务端口、使能或发送运动指令。

## 最新接入状态（2026-10-12）

原生 `A10Operator` 已支持六轴 joint follower、单帧关节 UDP 和按时动作块发送；通过显式 USB 路由复用 `GripperOperator`，通过可选 cameras 复用 `CameraOperator`，get_frame 提供实际 qpos[7] 与两路 RGB。原生 `A10InferenceRunner` 与本地/ZMQ 模型适配已迁入 `kaanh_follower`，回环模拟闭环和真实权重离线推理通过。

本轮没有连接真机/SSH。UDP 无逐帧确认，发送完成不等于到位；控制器的失联停止、夹爪同步及任务效果仍待现场。相机/USB 故障发生于已启动跟随时保留可用控制连接，便于 runner 显式停止后关闭；网络本身断线则不能确认停止。此前调查结论为历史证据，最新小任务记录见文末。

## 架构约定

用户明确：仿真与实机的运动规划／执行由控制器侧承担。FluxVLA 保留 VR 输入、数据采集管理、训练、推理和通信适配，向控制器发送约定的目标，接收状态以及视觉观测。仿真引擎不迁入 FluxVLA，机器人几何、IK、轨迹插补和伺服不在平台内重复实现。

图像可以由控制器侧或独立相机服务提供；当前 FluxVLA 相机仍是本机 OpenCV 采集，远程／仿真图像接口尚未实现。把图像、状态与目标语义对齐后，上层流程才能通过选择控制器后端复用。

## A10_new 中发现的内容

| 证据 | 能证明什么 | 不能证明什么 |
| --- | --- | --- |
| `main.cpp` 通过 `aris::core::fromXmlFile` 加载 `kaanh.xml` | 这是 ARIS／KAANH 控制器配置 | 不是可直接交给 MuJoCo 的 MJCF 或 URDF |
| XML 含模型、关节、工具、惯量、驱动、计划和通信配置 | 有机器人运动学／控制模型 | 不代表已有桌面、物体、接触和相机的完整仿真场景 |
| `CMakeLists_aris.txt:70` 的 `ETHERCAT=SIMULATION` 与 `ARIS_USE_ETHERCAT_SIMULATION` | ARIS 构建模板存在总线仿真选项 | 当前 `CMakeLists.txt` 链接外部 KAANH 库，不能据此判断工控机已编译启用仿真 |
| `kaanh.xml` 的 `virtual_en`／`virtual_ds`／`set_virtual` 配置；6 个机械臂 EtherCAT 从站 `is_virtual=false` | 有虚拟驱动相关入口，当前本地配置仍指向实际驱动 | 不等于已提供可用的物理视觉仿真；本次没有切换这些选项 |
| `kaanh.xml:1508` 的 `kaanh.FollowerCart`，约 4005 行的 `UDPFollower` | 已登记 Cartesian follower 和 UDP 接收接口，9998 用于 FollowerCartParam | 具体坐标系、更新规则、超时及滤波不能由 XML 推断 |
| `kaanh.xml` 的 WebSocket 5888／5999 | 与附件客户端定义的监视／控制端口对应 | 不代表新旧库版本的消息语义完全一致 |
| `doc/model3d/model3d.xml` 的模型路径、名称为空 | 是一个 3D 模型配置占位 | 没有可直接使用的完整视觉场景 |
| XML 引用 `/robot/geometry/stl/Ka-Ur/*.stl` | 几何资源预期由控制器部署提供 | 本机这些路径不存在，不能验证模型外观与碰撞 |

结论：仓库有控制器模型、虚拟驱动和 follower 的配置线索；没有找到可在本机直接启动并返回两路图像的完整 A10 仿真环境。需查看工控机实际安装库、启动方式及资源，不应简单判定“完全没有仿真”或“改个开关即可完成 VLA 仿真”。

## robot.7z 的内容和已确认接口

压缩包 SHA256：`7e3c1ab49c65db65dd2841afc995e1556b2ac5f1984f41d5e169caf0cda8dce1`。
在临时目录只解压读取 3 个普通 Python 文件，原包保留不变，未将附件加入 Git 或导入生产入口。

| 文件 | 职责 |
| --- | --- |
| `robot/follower_udp_client.py` | UDP 目标报文打包与发送 |
| `robot/kaanh_backend.py` | WebSocket 命令／状态、follower 启停、UDP 调用以及多模型运动接口 |
| `robot/robot_state.py` | 解析控制器状态、多模型关节与 TCP 位姿 |

### 控制与状态通道

- 客户端默认 WebSocket 控制端口 5999，常量另列监视端口 5888，但此客户端主要连接 5999。
- WebSocket 使用二进制帧，内容为 40 字节小端 `<IIQqqq` 头和 UTF-8 文本命令。WS 头字段由 `_pack_header` 固定设置，和 UDP 的零值字段不同，不能互换。
- `get` 查询状态；`follower_cart` 请求启动跟随；`stop_follower` 请求停止。启停需要在实现适配后按控制器反馈确认，本次没有执行。
- 状态从 `ret_context.motion_msg`、`follower_msg`、`robot_msg`、`driver_msg` 解析，可包含关节、TCP、运行／跟随状态和错误信息。

### 目标通道

- 默认 UDP 9998，载荷形如 `{"type":"pq","pq":[[x,y,z,qx,qy,qz,qw]]}`。
- UDP 包同样是 40 字节小端头（`<IIQqqq`），首字段为 JSON 字节数，其余字段为 0。
- `send_pose_pq` 接口接收 xyz 毫米，除以 1000 后发送；底层 UDP 函数接收米。四元数顺序为 xyzw，四元数自身无长度／角度单位。
- 注释称目标相对启动 follower 时的位姿，且控制器滤波需要持续发送目标；代码保存 `follower_start_pq`，但发包函数没有做位置相减或四元数组合。这说明不能把绝对目标自动当成这里需要的相对量，必须进一步阅读控制器实现。
- 当前发包没有业务序号、采集时间戳或应用层 ACK。UDP `connect()`／`send()` 返回成功仅代表本机 socket 操作成功，不能证明控制器接收、采纳或执行完成。

## 与当前 FluxVLA A10 接口的差异

| 项目 | 当前 A10 legacy | 附件 follower 客户端 |
| --- | --- | --- |
| 网络 | TCP 8080，按行文本／JSON | WebSocket 控制＋UDP 目标，40 字节消息头 |
| 模型推理输出 | 六轴绝对关节弧度＋夹爪毫米，30 步批次 | xyz＋xyzw 末端目标，需确认相对基准与发送频率 |
| VR 输出 | 每帧 EE 平移／旋转向量增量＋夹爪输入 | 注释描述相对启动位姿的目标，不能直接累加／转发而不确认坐标系 |
| 反馈维度 | 6 个关节＋夹爪 | 客户端 `movej` 固定 20 轴：两条 7 轴臂、4 轴腰、2 个头部轴；状态解析支持分组 |
| 关节单位 | 弧度 | 客户端接口／字段为度 |
| 夹爪 | A10 单夹爪毫米位置／摇杆输入 | 另含 `hand_*` 灵巧手接口，不能当成 A10 夹爪直接使用 |
| 图像 | FluxVLA 本机两路 OpenCV | 附件没有相机／图像接口 |

现有 A10 模型的 7 维输出不能直接解释为 xyz＋四元数，虽然数字个数相同。若控制器继续接收关节目标，可保留现有训练动作空间；若改为位姿目标，需要控制器提供转换能力或建立明确的位姿训练目标／统计／执行适配，不能在 FluxVLA 暗中增加 IK 来绕过分工。

## 接入前需要确认或修正的点

1. **机器人与库版本**：附件包含多模型 20 轴约定，代码注释提及 2.5.10 multi；本地 A10_new 为 KAANH 2.5.7、六轴臂。需确认附件用于参考还是工控机已部署的接口。
2. **坐标语义**：目标是绝对位姿、启动锚点相对位姿还是逐帧增量；平移属于基座／工具／锚点哪个坐标系；旋转组合顺序；reset／重连后锚点是否变化；UDP `pq` 数组指向哪条臂。
3. **反馈单位**：状态类部分注释写米，解析器另注“样例看起来是毫米”；实际解析原样保留数值。必须提供真实 `get` 样例并结合控制器源码确认，不能从变量名判断。
4. **时序与状态有效性**：`RobotState.timestamp` 是客户端构造对象时的本机时间，不是控制器采样时间。还需采样序号／时间、掉包和目标过期规则、推荐频率、跟随到位判定。
5. **启停确认与错误处理**：`start_follower()` 发出命令后即置本地 active，未验证控制器实际进入该模式；部分错误／超时返回 None。后续适配要区分通信失败、命令拒绝、目标到达和真实停止。
6. **报文校验**：当前发送缺少长度／有限值／四元数合法性校验；接收直接截掉前 40 字节，尚未校验声明长度；需要建立协议测试，避免无效数值进入控制器。
7. **观测与夹爪接口**：仿真要返回与实机一致语义的关节／位姿及夹爪反馈；视觉 VLA 完整闭环还需要两路渲染 RGB、相机参数和时间对应关系。仅虚拟电机运动不足以完成视觉抓取评估。

## 本次离线验证

- 3 个文件通过 Python AST 语法解析。
- 注入只写内存的 fake socket，未建立网络连接：输入 `[100,200,300,0,0,0,1]` 经上层客户端输出 `[0.1,0.2,0.3,0,0,0,1]`，头长 40 字节，声明 JSON 长度正确。
- 修改本地 `follower_start_pq` 后，同一输入的 UDP 字节完全不变，确认客户端没有进行启动位姿相对转换。
- 合成多模型 `get` 回复验证：关节展开为 20 维，默认 TCP 取模型 0，xyz 数值原样返回，没有单位转换。此检查验证解析行为，不验证控制器协议或机械性能。

## 初次核查后的控制器侧取证清单

用户可在已 SSH 登录的工控机上做以下只读检查。只需提供相关版本、路径和接口片段，不需要凭据或完整配置中的账户信息。

```bash
# 当前控制／监视／follower 端口；输出存在仅说明进程正在监听
ss -lntup | grep -E ':(5888|5999|9998|8080)\b'
# 控制库及机械臂模型资源的位置
ls -ld /usr/aris /usr/kaanh /usr/kaanhbot /robot/geometry/stl/Ka-Ur
# 在已知安装目录中找声明；若 rg 未安装，可使用 IDE 文本搜索
rg -n 'FollowerCart|FollowerCartParam|UDPFollower|ARIS_USE_ETHERCAT_SIMULATION' /usr/aris /usr/kaanh /usr/kaanhbot -g '*.hpp' -g '*.h' -g '*.cpp' -g 'CMakeCache.txt'
```

优先取得：实际库版本与仿真启动方式、FollowerCart 的目标处理源码、一条去除敏感内容的 `get` 状态样例、仿真相机输出接口。无需为本次核对启用电机或发送 follower 运动目标。

## 2026-10-09：工控机 SSH 只读核查

### 范围与证据限制

按用户指定账号登录工控机，只使用目录／文件读取、定向文本搜索、`ss`、进程名称查询、`nm`／`readelf`、SHA256 和禁用可选锁的 Git 历史查询。没有执行附件客户端，没有向 8080、5888、5999、9998、9999 发送任何业务消息（包括 `get`），没有安装、编译、编辑、重启、切换虚拟模式或调用 sudo。检查完毕已关闭 SSH。凭据不写入文件或提交。

发现 root 用户运行的 `kaanhbin` 进程，但当前账号无权读取其 `/proc/<pid>/exe` 和 `cwd`；因此下述磁盘源码／XML 不能直接认定为该进程正在使用的版本。工控机 A10_new 的 Git HEAD 为 `a9b0aaa`，源码仍在由同事开发；HEAD 也不能代表全部工作区内容或运行二进制版本。

### 已确认的 follower 安装与接口

| 项目 | 只读证据 | 结论及限制 |
| --- | --- | --- |
| 安装版本 | `/usr/aris/aris-2.5.7`、`/usr/kaanh/kaanh-2.5.7`、`/usr/kaanhbot/kaanhbot-2.5.7`；A10_new 的 CMakeCache 指向这些版本 | 工控机确有本机缺少的控制库；附件注释中的 2.5.10 multi 不能当作这里的版本 |
| follower 声明 | `/usr/kaanh/kaanh-2.5.7/include/kaanh/follower/udpFollower.hpp` | 同时存在关节与 Cartesian follower，具体实现通过私有 `Imp` 隐藏 |
| 已编译实现 | `lib/release/libkaanh_lib.so` 导出 `UDPFollower::init/execute`、`FollowerCart::prepareNrt/executeRT`、`FollowerCartParam::getPe/setPe/getInitPe` | 不是仅 XML 占位；在安装目录和已查项目／桌面安装包中未找到对应 `.cpp`，尚未读到完整目标处理实现 |
| 位姿语义线索 | `udpFollower.hpp:207` 注释“下发的是相对原点的增量”；:100–108 有初始位姿、工具和工件坐标接口 | 支持相对量的判断；“原点”具体如何定义、旋转组合顺序、PQ 解码及掉包规则仍未确认，不得直接发送绝对位姿 |
| 关节跟随能力 | :60–75 的 `FollowerAjParam` 提供轨迹生成器与直接队列入口；:188–203 的 `FollowerAjAbsolute2` 注释说明使用 MoveAbsolute2 规划 | 控制库已有关节跟随路线；实际单位、消息结构、保护及适用模式仍需服务端契约，不能仅凭方法名称接入 |
| 端口配置 | `/home/kaanh/A10_new/kaanh.xml:4011–4015`：关节 UDP 9999、Cartesian UDP 9998；:1735／1738：WEB 5888／5999 | `ss` 同时看到这些端口及 TCP 8080 正在监听；监听不证明 follower 已启用，也没有据此确认进程归属 |
| 消息头 | `/usr/aris/aris-2.5.7/include/aris/core/msg.hpp:15–26` 为 uint32 size、uint32 id、uint64 type、三个 int64 reserved | 字段布局与附件 40 字节 `<IIQqqq` 头一致；仅确认结构，尚未验证固定字段值和服务端载荷处理 |

`FollowerParamBase` 还声明了 `setLastId/lastId`（:37–38），说明控制库有 ID 状态入口；不能由此推断附件的零值消息 ID 已满足去重或丢包处理。

### 仿真现状

- 工控机磁盘上的 `kaanh.xml:226–391` 仍是六个机械臂 EtherCAT 从站 `is_virtual=false`，是单臂配置；不能把附件硬编码的 20 轴布局直接套用。
- 安装的 release `libaris_lib.so` 依赖 `libethercat.so.1`，支持实际 EtherCAT 通信。这不能排除库中同时具备虚拟驱动能力，也不能证明当前运行模式。
- XML 指向的 `/robot/geometry/stl/Ka-Ur` 不存在。ARIS 的 `resource/robot/ur5` 里有 `L1.DWG` 等几何文件，但没有验证为 A10 模型，不能视为可用的抓取场景或双相机渲染器。
- 在已查 A10_new、控制库安装目录、桌面 KAANH 安装包与相关项目路径，没有找到能够直接提供双路 RGB 的完整仿真启动入口。结论是“尚未找到／验证”，不是整台工控机绝对没有仿真。

因此当前不能承诺“改一个地址就跑通视觉仿真”。仍需要控制器侧明确仿真服务、机器人／夹爪／物体场景、状态及相机输出；FluxVLA 继续只做协议与观测适配。

### 新版 A10 TCP 与现有 FluxVLA 的差异

以下来自工控机磁盘源码，尚未连接服务验证：

1. **执行中反馈刷新已更新**：`main.cpp:125–151` 每约 33 ms 刷新 actualPos 和夹爪反馈，批次执行期间也刷新。本机旧版“批次期间跳过刷新”的结论不适用于这份新版源码；周期刷新也不等于具有控制器采样时间戳。
2. **状态协议增加能力声明**：`a10_tcp_server.cpp:334` 起包含 `policy_protocol_version=2`、构建信息、`policy_running`、`batch_seq`、`motion_settling`、规划器和时间步等字段。`idle` 要求队列为空且不在收尾运动；队列为空不代表停稳。
3. **批次支持时序和代际**：:637–697 的 `SET_JOINTS_BATCH` 保留每步 7 维，同时接受 `obs_age_s`、`expected_batch_seq` 和 `smooth`，回复 `batch_seq`。`a10_policy_batch.hpp` 按观测年龄与控制器本地时间对齐，具体使用需连同异步桥接契约一起核对。
4. **平滑仍由控制器负责**：源码提供 `cubic_pv_v1`／`quintic_discrete_pva_v1` 能力，现场文档记录后者仍在开发核验。这里只记录磁盘能力，不宣称运行进程已使用最新版本，也不把其他团队的日志结论当作 FluxVLA 验收。
5. **平台尚未跟进**：当前 `fluxvla/projects/a10/transport.py` 的 `send_batch()` 只发送 actions，`wait_idle()` 只检查 remaining==0；在线流程按整批完成后再推理。应先核对并适配新状态的停稳语义、能力版本和批次拒绝行为，再独立迁移观测年龄、异步重叠与代际校验。不能宣称当前平台已具有工控机新版异步衔接能力。

上述差异作为下一小任务记录，本次只修改本地文档，没有改动平台控制逻辑或工控机实现。

### 后续任务顺序

1. 以最新 A10 TCP 源码建立离线协议样例／测试，优先修正 `idle`／`motion_settling` 的适配，明确旧版本回退规则；随后单独迁移异步推理协议。每个小任务更新文档并独立提交。
2. 控制器侧提供 2.5.7 follower 目标处理源码或正式协议说明，明确相对基准、单位、频率、超时、模型索引、启停确认与夹爪接口。真实 `get` 样例本轮尚未取得。
3. 确认控制器仿真与 RGB／状态输出后，增加相应 follower 和观测适配，再验证 VR→仿真运动→原生录制→训练／推理。保留现有 7 维关节训练路线；PQ 路线按独立动作语义接入。

核查时关键文件 SHA256（仅用于区分后续版本，不代表运行进程版本）：

```text
udpFollower.hpp  e4f04d1d2ba817d7cbcc4a91524890114fbe9ac76b4843be5df60895ecd818da
libkaanh_lib.so (release)  fbd638320e3025f5e3c4ec8ec8d9761c5f8499a7c69a96123b05dce039eb52b5
A10_new/main.cpp  7f5567c357bc3fe87beaaeec02c368a3b4ac3a7dd172934dbf6841e66e83f59b
A10_new/a10_tcp_server.cpp  dcc66ad750949d7af0f8a0c06953915956ddf6381ab85aca19f21efc9c68ab7b
```

## 2026-10-09：单文件 operator 实现与本机收发验证

分支：`kaanh_follower`。运行实现集中在 `fluxvla/engines/operators/a10_operator.py`，由现有 `OPERATORS` 注册器导出；继承 `BaseOperator`，不依赖 `projects/a10`。没有新增运行目录，也没有切换旧 A10 CLI 的 TCP 8080 后端。本轮未连接真实控制器、VR 或相机。

### 构建与接口

```python
from fluxvla.engines import build_operator_from_cfg

operator = build_operator_from_cfg(dict(
    type='A10Operator',
    host='127.0.0.1',  # 示例地址；构造不建立连接
    ws_port=5999,
    udp_port=9998,
    timeout_s=2.0,
    allow_motion=False,
    arm_name='right',
    feedback_joint_unit='deg',    # 附件约定；实机须核实
    feedback_position_unit=None,  # 未知时保留原始 pq，不猜米/毫米
    gripper_feedback_path=None,   # 未取得实际毫米反馈字段
))
```

| 方法 | 行为 |
| --- | --- |
| `connect()` / `close()` | 显式打开／关闭 WS 和 UDP；不登录、使能或自动启动 follower |
| `get_state()` | 查询 `get`，返回 A10State，保留原始回复、实际／目标关节、TCP 和运行状态 |
| `get_frame()` | 提供机器人反馈字典；完整实际关节＋实际夹爪齐全才有 qpos，不采集相机 |
| `enable()` / `disable()` | Manual 模式发送 manual_en/manual_ds，回读 Enabled/Disabled；不自动切换模式 |
| `set_tool_coordinate()` | 显式发送 set_jog_coordinate --tool 并回读 Tool；要求无故障、Stopped、follower 未运行，不发送运动目标 |
| `set_jog_coordinate('Axis'/'Tool')` | 显式选择点动坐标系并回读；不自动切换 follower |
| `start_follower(mode='joint'/'cartesian')` | 默认构造为 cartesian；无参数时使用当前 command_mode，确认 running_state=true 且 Axis/Cart；拒绝接管或覆盖待清理启动 |
| `send_eepose({'right': pose})` | 发送相对启动基准的 xyz 米＋xyzw；只允许本连接已启动并确认的 Cartesian follower；不做 VR／绝对位姿转换 |
| `send_joints({'right': joints_rad})` | 六轴绝对弧度，发送时转度，UDP 9999 的 jp 命令；仅本连接已确认关节模式，不含夹爪 |
| `send_gripper({'right': 35.5})` | 封装 `g_move --pos=35.5`，范围 0–100 mm；回复不等于实际到位，wait=True 尚不支持 |
| `stop_follower()` | 请求本连接启动的 follower 退出，并回读 running_state=false；不是物理急停 |

`close()` **只释放连接，不发送停止命令**；控制器端掉线后的行为还需核实。启动超时不得继续发送位姿；如连接仍有效，可显式停止本次已发起的 follower。断线重连不继承启动归属。运行中需持续查询状态以观察故障／模式变化，本模块没有后台状态监视或已确认的控制器 watchdog。

### 数据格式与限制

- WS 请求头为 `<IIQqqq>`：载荷长度、1、0x1000、0xA1B2C3D4、0、0；UDP 头除长度外均为 0。接收严格检查二进制与长度，但响应 ID／保留字段没有已确认的回显约定，因此不强行要求等于请求头。
- WS 回复要求整数 `ret_code`，非零作为失败；支持对象或 JSON 字符串形式的 `ret_context`。状态查询有总超时及跳过次数上限，可跳过延迟空 ACK。事务超时／协议错误后关闭连接，不继续复用不确定的回复队列。
- ACK 没有可用的业务关联序号，因此不宣称普通命令 ACK 可严格归因到具体执行；follower 启停以状态回读确认，夹爪到位仍未验证。
- `actual_pos` 与 `motor_pos` 分别解析，只接受平铺六值或单组六值；多臂／20 轴数组拒绝。PQ 接受平铺、单组或模板中的 `[TCP, 辅助轴位姿]` 两组形式，取 TCP；不把第二组作为另一条机械臂。
- 关节可配置 wire 为 deg／rad，平台统一弧度；TCP 可配置 wire 为 m／mm，未配置时仅保留 `tcp_pq_raw`，`tcp_pq_m=None`。UDP 发送接口始终用米；不自动清零小量或修改四元数。
- 夹爪路径相对 `ret_context`，只接受明确的毫米数字；默认没有路径，因此 `gripper_mm=None`、`qpos=None`。测试的 `test_gripper.actual_mm` **仅是合成字段，不是已验证的控制器字段**。
- 初次实现未提供关节 follower 或电机使能，后续补充见开发记录；当前仍未提供回零、动作块定时执行或自动登录。绝对位姿、逐帧增量与 follower 相对目标不能互换。
- 实机待确认：状态字段／单位、相对坐标语义、夹爪 WS 命令路由和反馈、启动权限、掉包／失联行为；本地协议测试不能替代这些确认。

### 复现本机测试

```bash
conda activate fluxvla
cd /home/robot/VLA/FluxVLA
python -m unittest discover -s test -p test_a10_operator.py -v
```

测试监听仅 `127.0.0.1` 随机端口，接收端独立解包并验证头字段、长度及 JSON，不调用生产编解码来替自己证明正确。覆盖 8 项离线检查和 9 项网络检查，共 17 项：WS／UDP 完整收发、空 ACK、未确认启动、禁止接管、只读门禁、坏包／错误码／断线／超时、非法目标零发送、故障回读、并发查询和重连归属。服务和 socket 在结束时释放。


## 2026-10-09：使用说明核对与真实控制器只读通信

用户提供 `遥操作使用说明(1).pdf`（5 页），并授权调查工控机。本次读取 PDF、SSH 检查磁盘配置，并由本机 FluxVLA 查询控制器真实状态；没有执行文档中的登录、使能、调速或运动指令，远端文件和进程未修改。PDF 保持原样、未加入 Git。

### PDF 与实现对照

- 第 1–2 页：关节 tg_id=0、六个关节；Cartesian tg_id=1、末端 ee_ids={0}；UDP 9999／9998。工控机 A10_new 当前磁盘 XML 与此配置一致。
- 第 2–3 页：`get` 的 ret_context 是嵌套 JSON，包含 follower_msg.running_state／follower_mode；现有解析器与真实回复兼容。
- 第 3–4 页：文档分别列出 `start_follower`（开始跟随功能）、`stop_follower`、`follower_aj`、`follower_cart`（运行 Cartesian 跟随）。当前 Python `start_follower()` 沿附件模板发送的是 **follower_cart**，没有发送同名 wire 命令 start_follower。文档未明确两者是否必须顺序调用，因此不能说已完成真实启动验证，也不在未确认时自行插入启动命令。
- 第 4 页：Cartesian 目标采用二维数组，pe 为六维、pq 为七维，发送单位为米／弧度，值为增量；四元数无角度单位。文档示例展示 Euler 的 type=321；附件提供 type=pq 的具体发包模板。文档没有完整说明增量所用坐标系及旋转组合顺序，仍需实机受控验证／控制器说明。
- 第 4–5 页：关节 jp 是绝对位置（度／毫米），与 Cartesian 增量不同。登录、虚拟使能 manual_en 和真机使能 en 是单独命令；当前 operator 不自动登录／使能。未从文档密码示例推断现机认证行为。

### 实际查询结果

使用已有 `A10Operator(host='192.168.110.124', allow_motion=False)`，连接 WS 5999，连续发送三次 get，三次均成功解包和解析。UDP socket 只进行本地 connect，没有发送载荷。

| 项目 | 查询时结果 |
| --- | --- |
| WS 5999 | 连接和三次 get 均通过 |
| 六轴实际反馈 | 解析为六维；保留 actual_pos 与 motor_pos 的区别 |
| TCP 原始位姿（首条） | xyz 约 `[8.817572, -294.220446, 617.355520]`，xyzw 约 `[0.623286, -0.696897, 0.279013, 0.219093]` |
| follower | running_state=false，mode=None |
| 机器人 | activate=Disabled，motion=Stopped，op_mode=Manual，status=Normal，cs_error_code=0 |
| 错误 | 三次 has_error=false |
| 夹爪 | 该 WS 状态中未确认独立的实际毫米字段；当前 gripper_mm、qpos 仍为 None |

随后独立只读查询 TCP 8080 的 GET_POLICY_STATUS，得到 idle=true、remaining=0、policy_running=false 后，才发送 GET_FOLLOWER_STATE：六轴弧度＋夹爪毫米共七值解析通过。构建标识为 `Oct 9 2026 10:50:24`。夹爪读数约 `0.026738 mm`，仅证明接口返回该值，不证明夹爪串口当前健康或采样新鲜度。

TCP 六轴与 WS 按度→弧度转换后的反馈最大差约 `3.63e-5 rad`（两次非同时采样），支持此控制器 WS 关节单位为度的约定。TCP 位姿数字与 PDF 示例呈毫米量级，但仅凭量级不作为坐标／单位最终验收；默认 position_unit 仍未更改。

远端磁盘 Git HEAD 为 `97feb31`；控制进程为 root 运行的 kaanhbin，PID 与上次不同。磁盘 HEAD 不等于运行二进制。监听端口 5888、5999、8080、9998、9999 均存在；端口监听不等于跟随模式已激活。SSH、WS、TCP 和本地 UDP socket 检查后均已关闭。

### 当前结论与下一步

**FluxVLA 已能连接真实控制器并读取机械臂反馈。尚不能宣称运动通信完全验收。** 本轮没有发送 follower_cart／start_follower／stop_follower、UDP PQ、g_move 或使能命令。

下一步实际运动验收需现场协调独占控制、由现场人员按真机流程使能，再确认 follower 启动顺序，以受控目标核对方向、尺度、回读及停止行为；不自动重启控制器或改 XML。用户此前要求不影响同事的工控机开发，因此本次调查没有扩展成真实运动操作。

完整 A10 六轴＋夹爪观测目前可由旧 TCP 查询得到；新 WS operator 尚未接入该补充通道。后续可选择在控制器 WS 返回实际夹爪字段，或明确配置旧 TCP 只读补充，不能给新 operator 随意填一个不存在的字段路径。


## 2026-10-09：en 未使能的调查与操作说明纠正

用户执行真机 `en` 后，回读仍为 Disabled／Stopped／无故障。再次仅查询 get 并 SSH 只读调查，没有重复发送 en 或试发 manual_en，没有修改控制模式、虚拟设置、IO／安全配置或远端文件。

证据：

- 实际 get：op_mode=Manual，activate=Disabled，六轴 driver_err_code 全 0，follower 未运行。
- A10_new XML 的 en AuthorityRule 名称为“自动模式使能”，en／manual_en／virtual_en 分别路由到 SystemSema。
- `/home/kaanh/A10_new/build/kaanh_rtc/bin/syslog.db` 通过 SQLite `mode=ro` 读取：ID 104、2026-10-09 15:49:58 记录“自动使能”，说明用户 en 已到达控制器；ID 102、14:51:01 为“手动模式指令使能”，ID 103、15:12:06 为“手动模式指令掉使能”。历史日志不是本次 manual_en 已通过的证明。
- `/usr/kaanhbot/kaanhbot-2.5.7/include/kaanhbot/robot/system_sema.hpp` 的 SystemSema 分别维护 motion_is_virtual_、auto_on_、manual_on_／manual_en_now_，区分虚拟状态与手动／自动使能逻辑。

据此，此前把 PDF 简化表述直接解释成“真机只能 en，manual_en 只能虚拟”的操作指导不适用于当前部署，应按实时操作模式核对。下一步由现场用户保持 Manual，发送 manual_en 并确认 Enabled；不自动切换 Auto 或修改虚拟配置。对应手动掉使能命令是 manual_ds。命令 ACK 与成功使能分开判断，失败时打印 ACK、故障和当前模式。

本次没有新增运动测试或修改 operator；只是纠正命令选择指导。SSH／状态查询连接均已关闭。尚未收到本轮 manual_en 的实机结果，因此不记使能验收通过。


### 现场回传：manual_en 使能成功

2026-10-09，用户在现场执行修正后的单次 manual_en 验证脚本并提供终端截图：回复 ret_code=0、ret_context=""、ret_msg=""；随后 get 回读 op_mode=Manual、activate=Enabled、motion=Stopped、has_error=false。确认此部署下 FluxVLA 经 WS 的手动使能及状态回读链路通过；这不是仅依据 ACK 判定。

脚本结束只关闭连接，未掉使能。该截图不证明 follower 启停或 UDP 位姿执行已通过。下一步由现场用户在同一连接中验证 follower_cart 启动和 stop_follower 退出，不发送 UDP 位姿或夹爪目标；异常时报告实际回读和停止结果，不把断开连接视为停止。本次 agent 未向控制器发送使能或运动命令。

### 2026-10-09：现场启动被坐标系拒绝，补充前置检查与失败恢复

- 用户执行 follower 启停检查，收到 ret_code=-987313、“请切换到笛卡尔坐标系”。旧实现关闭异常事务连接，但保留启动尝试标记，现场脚本随即尝试停止，显示“operator is not connected”。这不是停止成功，也不能仅凭这个异常判断 follower 是否已启动。
- 本轮只读 get 实测：jog_coordinate=Axis，op_mode=Manual，activate=Disabled，motion=Stopped，status=Error，log_msg.has_error=true；follower running_state=false、mode=Cart。应以 running_state 判断是否运行，不能仅看 Cart。六轴 driver_err_code 全为 0，cs_error_code=0，仍不能忽略日志错误。
- SSH 只读 SQLite（mode=ro）确认最新日志 ID106、16:00:23、code=987313：“请切换到笛卡尔坐标系”；前一条 ID105、15:57:05 为手动使能。未清错、切换坐标系、使能、启停或发送目标，远端文件未改，连接均已关闭。
- 对照 robot.7z 的 kaanh_backend.py:set_jog_coordinate 和本机 A10_new/kaanh.xml:SetJogCoordinate，模板使用 `set_jog_coordinate --tool`。此前测试漏掉此准备步骤。新增公开 `set_tool_coordinate()`，由用户显式调用；要求无故障、明确 Stopped、follower 未运行，并回读 Tool 后才返回成功。ACK 不能替代坐标系反馈，超时不宣称设置成功。
- start_follower 在发送前检查 Cartesian 坐标系，Axis／缺失／未知值均在本地拒绝并给出设置方法，不自动切换。启动收到控制器错误且旧连接被关闭时，仅重建连接查询 get，不重发启动或自动停止；只有确认 running_state=false 才清除本次启动尝试标记。恢复失败／仍运行时保留待处理标记，并禁止此次异常路径直接发送位姿；原始错误继续上报。一般超时／坏包仍按原规则关闭不确定连接。
- 回环测试新增：Axis 前置拒绝、显式 Tool 命令和反馈、仅 ACK 不算成功、有故障／未确认停止拒绝设置、启动错误后新连接只读恢复、恢复失败或仍运行不误报停止。验证命令见上；当前总计 **22 项测试通过**。所有模拟端点仅为 127.0.0.1，没有用实机测试修改后的控制命令。
- 下一步现场操作：先在控制器界面核对并清除上述报警，然后显式设置 Tool 并回读；再按 Manual 模式使能、确认 Enabled／Stopped／无错误，最后重新验证 follower 启停。此次尚未确认 Tool 设置、实际启停或 UDP 目标执行通过。选择 Tool 会改变后续手动点动的参考坐标系。


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
