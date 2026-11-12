"""双臂录制配置：由 teleoperate.py --record-config 加载；单独加载不连接设备。

物理左臂 ID1、右臂 ID0；状态为左右各6关节(rad)+实测夹爪闭合比例。
设备身份从 a10_devices.py 继承；USB路径变化不改变数据字段/身份约定。
"""
_base_ = ['./a10_devices.py']

record = dict(
    backend='lerobot',  # 原工具链的 LeRobotDataset：add_frame/save_episode/clear_episode_buffer。
    schema='a10_dual_v1',
    root='datasets/a10_dual_lerobot_record',  # 新后端用新目录；不改动旧数据/失败片段。
    fps=30,
    cameras={name: dict(width=cfg.width, height=cfg.height,
                        identity=dict(cfg.identity), rotate_180=cfg.rotate_180)
             for name, cfg in _base_.cameras.items()},
    device_metadata=dict(
        grippers={side: dict(identity=dict(cfg.identity), device_id=cfg.device_id)
                  for side, cfg in _base_.grippers.items()}),
    image_writer_threads=6,  # 原生后台PNG写入，保存时由LeRobot编码视频。
    max_episode_frames=18000,
    resume=False, recover=False, synthetic=False,
)

# 独立于writer的实时采集/按键参数。30Hz为目标；接收时间并非硬件曝光时间。
capture = dict(
    sampling_mode='control_loop',  # 与原工具链一致：每轮遥操作后add_frame，不追补帧。
    task='Pick up the water bottle',
    terminal=True,  # 备用终端命令；头显/手柄可独立操作。
    buttons=dict(modifier_button='a', threshold=.7, neutral=.2, max_age_s=.25),
    confirmation_timeout_s=8., hud_hz=2.,
    max_camera_age_s=.15, max_robot_age_s=.15, max_gripper_age_s=.20,
    max_skew_s=.15, max_sample_gap_s=.15, max_clock_drift_s=.30,
    max_missing_feedback_s=1.,  # 短暂过期跳过并记录；持续缺少真实设备数据仍使本集失效。
)
