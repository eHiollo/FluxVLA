"""双臂录制配置：由 teleoperate.py --record-config 加载；单独加载不连接设备。

物理左臂 ID1、右臂 ID0；状态为左右各6关节(rad)+实测夹爪闭合比例。
设备身份从 a10_devices.py 继承；USB路径变化不改变数据字段/身份约定。
"""
_base_ = ['./a10_devices.py']

record = dict(
    schema='a10_dual_v1',
    root='work_dirs/a10/datasets/a10_dual_record',
    fps=30,
    cameras={name: dict(width=cfg.width, height=cfg.height,
                        identity=dict(cfg.identity), rotate_180=cfg.rotate_180)
             for name, cfg in _base_.cameras.items()},
    device_metadata=dict(
        grippers={side: dict(identity=dict(cfg.identity), device_id=cfg.device_id)
                  for side, cfg in _base_.grippers.items()}),
    queue_size=32,  # 三路640x480 RGB约84 MiB队列，不包含编码/工作线程开销。
    max_episode_frames=18000,
    resume=False, recover=False, synthetic=False,
)

# 独立于writer的实时采集/按键参数。30Hz为目标；接收时间并非硬件曝光时间。
capture = dict(
    task='Pick up the water bottle',
    terminal=True,  # 备用终端命令；头显/手柄可独立操作。
    buttons=dict(modifier_button='a', threshold=.7, neutral=.2, max_age_s=.25),
    confirmation_timeout_s=8., hud_hz=2.,
    max_camera_age_s=.15, max_robot_age_s=.15, max_gripper_age_s=.20,
    max_skew_s=.15, max_sample_gap_s=.15, max_clock_drift_s=.30,
)
