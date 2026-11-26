"""Model-independent async execution with motion/position correction separated.

Works with the existing OpenPI WS/8001 sampler, without model-side RTC support.
The synchronous entrypoint remains available in a10_dual_inference.py.
"""
_base_ = ['./a10_dual_openpi_remote.py']

# Confirm reversals across short contradictory predictions. Aperture remains
# continuous; this adds up to 250ms of confirmation, not an arrival wait.
grippers = dict(left=dict(reversal_deadband_raw=450, reversal_confirm_s=.25),
                right=dict(reversal_deadband_raw=450, reversal_confirm_s=.25))

inference = dict(
    async_execution=True,
    execute_horizon=0,
    rtc_config=dict(enabled=False),
    action_fusion=dict(enabled=True, method='velocity', alpha=.25, tail_steps=12,
                       handover_steps=6, position_gain=6., correction_tau_s=.15,
                       correction_fraction=1., correction_floor_deg_s=3.,
                       correction_max_deg_s=30., endpoint_steps=12, gripper_alpha=.35),
)
