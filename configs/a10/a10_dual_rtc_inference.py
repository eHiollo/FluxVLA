"""Opt-in dual RTC targeting the onsite OpenPI WebSocket server on port 8001.

Use method='guidance' for a checkpoint without RTC training. This configuration
does not change the deployed synchronous baseline or start hardware by itself.
The server must advertise and implement the absolute-prefix RTC contract.
The current unmodified OpenPI sampler does not implement it: fail before motion
until its model-side RTC support is connected. --model fluxvla selects the
already implemented native ZMQ RTC server explicitly.
"""

_base_ = ['./a10_dual_openpi_remote.py']

inference = dict(
    async_execution=True,
    # Same RTC cadence as the project's ALOHA recipes: immediately observe and
    # infer again while the previous prediction keeps publishing at 30 Hz.
    execute_horizon=0,
    action_fusion=dict(enabled=False),
    rtc_config=dict(enabled=True, method='guidance', prefix_len=12,
                    decay_end=18, schedule='exp', max_guidance_weight=5.,
                    use_vjp=False),
)
