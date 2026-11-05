"""2026-11-05 现场确认的设备绑定，供后续采集及双臂 VR 接线复用。

left/right 按手柄和物理机械臂命名，不是 A10Operator 的内部字典键：
左手柄 -> 物理左臂 -> ee_id=1（operator key: right）；右手柄相反。
夹爪 device_id 是各自串口上的 Modbus 地址，与机械臂 ee_id 无关。

本文件仅登记设备；当前 teleoperate.py 不会自动加载它，也不启用运动。
旧单臂/老夹爪配置继续单独使用，不能把知行 raw 位置当成旧夹爪毫米值。
"""

# 现场更正：未连接外部相机。逐台拔线/换接已验证接口确认：
# 左腕 D405 255323070485；右腕 D405 255323070551。
# 不将腕部画面绑定到 base_0_rgb，也不把同一台复制为第二路相机。
# 两台均使用 index4 彩色流，index0 为深度流；分别读取验证 640x480 RGB / 30Hz。
# 断电重启后暂仅枚举左腕，双路同时读取仍待排查右腕连接后验证。
# rotate_180=False 保留原始画面方向，安装方向待现场预览确认。
cameras = dict(
    left_wrist_0_rgb=dict(
        device='/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_255323070485-video-index4',
        width=640, height=480, fps=30, rotate_180=False,
    ),
    right_wrist_0_rgb=dict(
        device='/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_255323070551-video-index4',
        width=640, height=480, fps=30, rotate_180=False,
    ),
)

# port/baudrate 用于构造 ZXGripper；device_id 用于 read_state/set_position_raw。
# speed/torque 是本轮小范围验证的百分比参数，不是 mm/s 或牛顿。
# 两只夹爪使用独立串口，所以内部 ID 都为 1 可以正常工作。
# 已现场观察小范围动作；完整行程、开合方向和 raw->mm 标定仍待确认。
grippers = dict(
    left=dict(
        port='/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0EMMMM-if00-port0',
        baudrate=115200, device_id=1, speed=5, torque=60,
    ),
    right=dict(
        port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_5c9323064b6eef11a2c4e7c2c169b110-if00-port0',
        baudrate=115200, device_id=1, speed=5, torque=60,
    ),
)
