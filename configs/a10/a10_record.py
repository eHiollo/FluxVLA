"""旧单臂 A10 录制核心参数；本配置不会启动 VR、相机或机械臂。

调用 EpisodeWriter(**record) 后，由后续采集循环提供同一帧的数据。
双臂 14 维数据需要独立训练约定，本配置不自动截取或拼接双臂数据。
"""
record = dict(
    root='work_dirs/a10/datasets/a10_record',
    fps=30,
    cameras=dict(
        base_0_rgb=dict(width=640, height=480),
        right_wrist_0_rgb=dict(width=640, height=480),
    ),
    queue_size=32,  # 两路图像约 56 MiB 缓冲；满队列报错，不静默丢帧。
    max_episode_frames=18000,  # 单集最多 10 分钟，限制内存使用。
    resume=False,  # 明确开启后才允许续录本录制器创建的数据集。
    recover=False,  # 恢复中断：归档未完成集，或完成已开始的提交。
    synthetic=False,  # 模拟数据必须为 True，不允许混入实机数据集。
)
