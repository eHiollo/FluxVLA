"""Default single-arm console deployment. Joint initialization is in radians."""

operator = dict(
    type='A10Operator', host='192.168.0.125',
    ws_port=5999, udp_port=9998, joint_udp_port=9999, status_ws_port=5888,
    timeout_s=5., allow_motion=True,
    init_position=dict(right=[
        -2.044251792983398, -0.8144404421506339, 2.385708007843569,
        -0.6513219702592439, -1.2242612038114224, -0.24410174918392694,
    ]),
)
