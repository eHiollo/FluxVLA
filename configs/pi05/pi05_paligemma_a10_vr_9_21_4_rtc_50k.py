"""A10 PI0.5 LoRA training with prefix-conditioned RTC for 50k steps."""

_base_ = ['./pi05_paligemma_a10_vr_9_21_4_smoke.py']

model = dict(
    rtc_training_config=dict(
        enabled=True,
        max_delay=9,
        distribution='exponential',
        temperature=1.0,
    ),
)

runner = dict(
    max_steps=50_000,
    save_iter_interval=5_000,
    max_keep_ckpts=3,
)
