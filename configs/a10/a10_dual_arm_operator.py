"""192.168.0.150 dual A10: Cartesian tg 2 / ee_ids {0,1}, joint tg 1.

XML inspected 2026-10-23; separate WS entries added 2026-10-29.
Load the updated controller XML before connecting to 5999; 5867 stays for the pendant.
Physical left/right and live GET groups/units still require onsite confirmation.
Console and VR use one local relay owning a single command/status WS connection.
No recording or VR calibration. m_init supports the full joint follower group;
m_init explicitly opens both configured ZX grippers while returning the arms.
"""

operator = dict(
    init_grippers_config='{{fileDirname}}/a10_dual_devices.py',
    type='A10Operator', host='192.168.0.150',
    ws_port=5999, status_ws_port=5999, udp_port=9998, joint_udp_port=9999,
    status_poll_interval_s=None, timeout_s=2.,
    # First console/VR auto-starts the relay; either UI may open or close first.
    shared_control=dict(mode='managed', host='127.0.0.1', port=5870, role='teleop'),
    joint_follower_velocity_percent=100,  # follower_aj parameter; not set_jog_vel.
    joint_follower_filter=False,  # Updated SDK requires JSON motor_pos packets.
    motor_acceleration_deg_s2=1000.,  # Console/m_init/teleoperation motor limits.
    allow_motion=False, command_mode='cartesian', cartesian_wire_format='pe321',
    feedback_joint_unit='deg', feedback_position_unit='mm',
    # Keep the existing VR aliases and GET mappings; physical hand binding is defined by VR.
    arms=dict(
        left=dict(ee_id=0, joint_group=0, joint_indices=[0, 1, 2, 3, 4, 5], tcp_index=0),
        right=dict(ee_id=1, joint_group=1, joint_indices=[0, 1, 2, 3, 4, 5], tcp_index=1),
    ),
    # Row order follows the controller Cartesian group, NOT dict insertion order.
    cartesian_target_order=['left', 'right'],
    joint_target_order=['left', 'right'],
    # Preserve VR aliases: left key is id 0, right key is id 1. Targets stay attached to IDs.
    init_position=dict(
        left=[q * (3.141592653589793 / 180.) for q in (21, 38, -131, 28, 111, -3)],
        right=[q * (3.141592653589793 / 180.) for q in (-18, -36, 128, -18, -106, -5)],
    ),
    init_gripper_positions=dict(right=0., left=0.),
)
