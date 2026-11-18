"""Single A10 (.125) + old 100mm USB gripper, software opening limited to 85mm.

Inherits VR-only defaults. For live control explicitly set vr_only=False and
operator.allow_motion=True. Construction/preview never opens the serial port.
Do not run inference or another gripper program on the same USB port.
"""

_base_ = ['./a10_single_vr_base.py']

teleop = dict(
    gripper_speed_mm_s=10.,  # Thumbstick target rate, distinct from servo speed.
    gripper_direction=1,    # +X increases opening; set -1 to reverse.
    operator=dict(
        host='192.168.0.125', feedback_position_unit='mm',
        gripper_transport='usb',
        gripper=dict(
            port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            baudrate=1000000, servo_id=10, gripper_type='100mm',
            max_width_mm=85., speed=800, timeout=.1)),
)
