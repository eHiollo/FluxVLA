"""2026-11-05 现场确认的设备绑定，供后续采集及双臂 VR 接线复用。

left/right 按手柄和物理机械臂命名，不是 A10Operator 的内部字典键：
左手柄 -> 物理左臂 -> ee_id=1（operator key: right）；右手柄相反。
夹爪 device_id 是各自串口上的 Modbus 地址，与机械臂 ee_id 无关。

本文件仅登记设备；当前 teleoperate.py 不会自动加载它，也不启用运动。
旧单臂/老夹爪配置继续单独使用，不能把知行 raw 位置当成旧夹爪毫米值。
"""

# 逐台拔线/换接已验证接口确认腕部归属：
# 左腕 D405 255323070485；右腕 D405 255323070551。
# 不将腕部画面绑定到 base_0_rgb，也不把同一台复制为第二路相机。
# 两台均使用 index4 彩色流，index0 为深度流；双路同时读取各95帧，640x480 RGB / 30Hz。
# 外部 D435i 序列号 262923065199，彩色接口为USB interface 03。
# 本机该D435i的by-id index0在深度/彩色接口间重名，故使用区分接口的by-path。
# 外部相机换口后运行 python scripts/a10_devices.py --update，按序列号找回彩色接口。
# 三路同时读取各95帧，640x480 RGB / 约30Hz；未启用深度、IMU或硬件同步。
# rotate_180=False 保留原始画面方向，安装方向待现场预览确认。
cameras = dict(
    base_0_rgb=dict(
        identity=dict(model='D435i', serial='262923065199', usb_interface='03', video_index=0),
        device='/dev/v4l/by-path/platform-a80aa10000.usb-usb-0:3.2.2:1.3-video-index0',
        width=640, height=480, fps=30, rotate_180=False,
    ),
    left_wrist_0_rgb=dict(
        identity=dict(model='D405', serial='255323070485', usb_interface='00', video_index=4),
        device='/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_255323070485-video-index4',
        width=640, height=480, fps=30, rotate_180=False,
    ),
    right_wrist_0_rgb=dict(
        identity=dict(model='D405', serial='255323070551', usb_interface='00', video_index=4),
        device='/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_255323070551-video-index4',
        width=640, height=480, fps=30, rotate_180=False,
    ),
)

# identity 仅用于设备识别；构造 ZXGripper 时只传 port/baudrate，勿把整个字典展开。
# device_id 用于 read_state/set_position_raw。
# speed/torque 是本轮小范围验证的百分比参数，不是 mm/s 或牛顿。
# 两只夹爪使用独立串口，所以内部 ID 都为 1 可以正常工作。
# 已现场观察小范围动作；完整行程、开合方向和 raw->mm 标定仍待确认。
# check仅用于设备辨认：先用 --position-raw 0/1000 观察；确认后填写open_raw/close_raw。
# open_raw/close_raw分别填写0和1000中的一个，不能相同；None时拒绝--open/--close。
grippers = dict(
    left=dict(
        identity=dict(model='FT231X', serial='DU0EMMMM', usb_interface='00'),
        port='/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0EMMMM-if00-port0',
        baudrate=115200, device_id=1, speed=5, torque=60,
        check=dict(positions_raw=(0, 1000), open_raw=None, close_raw=None),
    ),
    right=dict(
        identity=dict(model='CP2102N', serial='5c9323064b6eef11a2c4e7c2c169b110', usb_interface='00'),
        port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_5c9323064b6eef11a2c4e7c2c169b110-if00-port0',
        baudrate=115200, device_id=1, speed=5, torque=60,
        check=dict(positions_raw=(0, 1000), open_raw=None, close_raw=None),
    ),
)
