# 浏览器手工阶段标注

在 FluxVLA 根目录激活项目环境后启动：

```bash
python tools/sarm_annotate/manual_video_annotator.py \
  --dataset-root ./datasets/a10_9_21_sarm \
  --output ./datasets/a10_stages.json \
  --port 8765
```

打开 <http://127.0.0.1:8765>。通过 SSH 使用时，将远端 8765 端口转发到本机。
服务仅监听本机地址。页面和视频均在本地，无需联网。

1. 每行填写一个阶段描述，顺序对应录像中的操作顺序。
2. 播放录像，在当前阶段的最后一帧按空格切分。下一阶段从下一帧开始。
3. N 个阶段切分 N−1 次；最后阶段自动覆盖到末帧。
4. 用暂停、进度条、左右方向键或单帧按钮定位；可以撤销最后切分。
5. 点击“保存并进入下一集”。“重标这一集”确认后清除该集标签。
6. 失败录像可点击“跳过本集”，无需完成切分，直接进入下一集。
   跳过记录保存在会话文件，不写入阶段标签 JSON；episode 列表显示“已跳过”。
   已有切分或标签会在确认后清除，点击“重标这一集”可取消跳过。

默认播放外部相机，可以切换腕部相机。切换后保持当前帧并暂停。
按钮获得焦点时空格仍用于切分，输入框或选择框获得焦点时不触发切分。

完整标签写入 `datasets/a10_stages.json`；未完成的切分和阶段描述写入
`datasets/a10_stages.session.json`，重启后继续。请等待保存完成后关闭页面。
修改阶段描述需要确认清除全部记录。每集必须按同一阶段顺序完整标注。
跳过的录像仍保留在数据集中。后续训练时须明确筛选已标注集；JSON 中缺少
该集不会自动使训练加载器排除它。

数据目前支持 LeRobot v2.1，视频需支持浏览器播放（本数据集实际为 H.264）。
使用以下命令只读检查全部视频帧数和帧率，需要环境中有 PyAV：

```bash
python tools/sarm_annotate/manual_video_annotator.py --check
```

全部标注完成后，将 JSON 写入 SARM 数据集元信息：

```bash
python tools/sarm_annotate/write_manual_stages.py \
  --dataset-root ./datasets/a10_9_21_sarm \
  --spec ./datasets/a10_stages.json \
  --default-sparse auto
```

生成标签采用 0-based 帧号，结束帧包含在区间内，使用 `dense_only` SARM 配置。

## 查看视频与预测曲线

中心帧验证结果生成后，在同一个环境运行：

```bash
bash scripts/view_sarm_a10.sh
```

打开 <http://127.0.0.1:8765>。可以用 `--port 8766` 更换端口。
此模式只读，加载 `sarm_runs/a10/validation_center/progress.jsonl`，使用原始
episode **7、11、16、20、23、24** 的视频和人工标注。旧的 `validation` 输出
缺少中心帧对齐标识，不能直接在查看模式中加载。

视频左上角显示当前预测百分比、进度条和人工参考。下方曲线显示预测与参考，
播放游标和视频同步。点击曲线或阶段名称可以定位，空格播放/暂停，左右键单帧移动。
切换相机保持当前时间，上一集/下一集切换验证录像。

人工参考根据人工阶段边界及训练集时间占比构造，不是额外的模型输出。
显示原始逐帧预测值，没有平滑或人工补值。此处是利用前后观测的离线预测回放。
查看操作不会写入标注 JSON 或会话文件，也不会调用 GPU 模型。
