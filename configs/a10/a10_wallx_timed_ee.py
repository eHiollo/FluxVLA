"""Opt-in TimedEe: one original Wall-X future waypoint per 30 Hz packet.

Requires the tool1 TimedEe shared library/controller process and relay.
Reuses the deployed timed_aj route in a10_dual_inference.py (same UDP entry).
"""
_base_ = ["./a10_wallx_deploy.py"]

wallx = dict(
    backend="timed_ee", mode="preview",
    interpolation_steps=1, adaptive_interpolation=False,
    max_control_hz=30., timed_ee_hz=30., stop_between_chunks=False,
    # GET TCP pose arrival checks after the ENTIRE stream only. GET pq's FK
    # source has not been established; this is not proof of measured accuracy.
    # Upstream fast-v2 defaults (2026-11-26 archive): 2 mm / 0.2 degrees.
    arrival_position_tolerance_m=.002,
    arrival_rotation_tolerance_rad=.003490658503988659,
    # endpoint_capture_complete already means capture trajectories ended;
    # match upstream's default of no additional settling hold.
    arrival_stable_s=0.,
    timed_ee=dict(enabled=True, udp_host="192.168.0.150", udp_port=10000,
                  plan_ws_port=5888, network_ready=True),
)
