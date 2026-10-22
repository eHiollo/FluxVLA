"""Single-controller dual Cartesian TEMPLATE; no confirmed physical left/right.

Based on A10_new/kaanh_lab.xml: tg 1 / ee_ids {0,1}. The controller currently
runs a SINGLE-arm XML. Do not enable this template until the team deploys and
confirms the dual model, physical sides, feedback layout and target order.
No recording, VR calibration, gripper or dual joint trajectory support here.
"""

operator = dict(
    type='A10Operator', host='192.168.0.125',
    ws_port=5999, status_ws_port=5888, udp_port=9998, joint_udp_port=9999,
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
