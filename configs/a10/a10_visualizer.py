"""Read-only viewer defaults for the dual A10 virtual controller."""

viewer = dict(
    host='192.168.0.198',
    status_port=5999,
    cartesian_udp_port=9998,
    joint_udp_port=9999,
    listen_port=8765,
    poll_hz=10.,
    stale_after_s=0.5,
    timeout_s=2.,
    position_unit='mm',
    joint_unit='deg',
    # Each A10 base's side opening faces local -Y. Rotate it toward the other base.
    display_offsets_m=((-0.575, 0., 0.), (0.575, 0., 0.)),
    display_yaw_deg=(90., -90.),
    # Joint marker positions on the parent parts, in metres, from .198 kaanh.xml.
    # Part indices refer to ground, L1, ..., L6 in each part_pq group.
    joint_markers=(
        (0, (0., 0., 0.1693)),
        (1, (0., 0., 0.1693)),
        (2, (0., 0., 0.7943)),
        (3, (0., 0., 1.3893)),
        # J5 axis runs along local Z. Use the J4 wrist height on that axis.
        (4, (0., 0.1219, 1.3893)),
        (5, (0., 0.1219, 1.4945)),
    ),
)
