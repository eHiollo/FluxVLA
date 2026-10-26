"""192.168.0.150 dual A10: Cartesian tg 2 / ee_ids {0,1}, joint tg 1.

XML inspected 2026-10-23. Physical left/right and live GET groups/units still
require onsite confirmation. 5867 must be made available by the controller team.
Use one command/status WS connection; no background polling connection.
No recording, VR calibration, gripper or dual joint trajectory support here.
"""

operator = dict(
    type='A10Operator', host='192.168.0.150',
    ws_port=5867, status_ws_port=5867, udp_port=9998, joint_udp_port=9999,
    status_poll_interval_s=None, timeout_s=2.,
    allow_motion=False, command_mode='cartesian', cartesian_wire_format='pe321',
    feedback_joint_unit='deg', feedback_position_unit='mm',
    # TEMPLATE mappings, to verify against actual dual-controller GET replies.
    arms=dict(
        left=dict(ee_id=0, joint_group=0, joint_indices=[0, 1, 2, 3, 4, 5], tcp_index=0),
        right=dict(ee_id=1, joint_group=1, joint_indices=[0, 1, 2, 3, 4, 5], tcp_index=1),
    ),
    # Row order follows the controller Cartesian group, NOT dict insertion order.
    cartesian_target_order=['left', 'right'],
)
