"""Requires the joint/camera-capable A10Operator implementation.

python scripts/inference.py --config configs/a10/a10_remote_inference.py

Use a FluxVLA ZMQ A10 server returning absolute [T,7] rad/mm targets.
Configure the operator's real camera/feedback settings before running.
get_frame must return qpos[7], base_0_rgb/right_wrist_0_rgb (HWC uint8 RGB),
and has_error=False. Frame timestamps are retained, not treated as proof of
synchronization. Operator execute_trajectory must block until sending ends.

For motion, configure both dry_run=False and operator.allow_motion=True,
and set max_joint_step_rad to the verified site limit. 'p' + Enter ends the
run after the current chunk. Ctrl+C unwinds through follower stop and close.
This runner does not perform homing or claim physical arrival after a chunk.
"""

inference = dict(
    type='A10InferenceRunner',
    remote_inference=dict(server_host='127.0.0.1', server_port=5555,
                          timeout_s=30., serializer='msgpack', compress=True),
    operator=dict(type='A10Operator', host='127.0.0.1',
                  command_mode='joint', allow_motion=False,
                  status_poll_interval_s=0.05, status_max_age_s=0.5,
                  gripper_transport='usb',
                  gripper=dict(port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                               max_width_mm=85., timeout=0.015),
                  cameras=dict(
                      base_0_rgb=dict(device='/dev/v4l/by-id/usb-SYD_USB_Camera_200901010001-video-index0',
                                      width=640, height=480, fps=30),
                      right_wrist_0_rgb=dict(device='/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0',
                                             width=640, height=480, fps=30, rotate_180=True))),
    instruction='Reach the yellow lemon',
    action_chunk=30,
    execute_horizon=10,
    action_dt=1.0 / 30.0,  # Recorded/trained targets are 30 Hz.
    max_publish_step=30,
    dry_run=True,
    max_joint_step_rad=None,  # Set a verified limit before dry_run=False.
    gripper_mm_range=(0., 85.),
)
