"""90 Hz dual VR: fixed world-axis translation, existing local rotation.

WebXR forward/left/up maps to table forward/left/up, then to each arm base.
Physical left: ee 1 (logical right); physical right: ee 0 (logical left).
Translation is independent of hand/tool attitude. All motion tuning is inherited.
"""

_base_ = ['./a10_dual_arm_vr_teleop.py']

teleop = dict(
    # Controller runtime-log capacity is full; local JSONL diagnostics remain enabled.
    # Re-enable explicitly after controller log storage has been managed.
    record_controller=False,
    resilient_takeover=True,
    # User-approved: normal release brakes/holds the last reference and keeps
    # this session; no stop/start/GET on re-grip. Fault/exit still requests stop.
    stop_on_both_release=False,
    continuous_feedback=False,  # 90 Hz uses sent references; measured GET is for recording only.
    # A recording GET failure must not revoke this teleop target stream.
    operator=dict(shared_control=dict(require_feedback=False)),
    confirm_stop=False,  # Send stop only; no stop-result/state confirmation transaction.
    pose_mapping='world_translation_local_rotation',
    pose_calibration_path=None,
    # Keys name physical VR hands, not A10Operator's swapped logical arm aliases.
    # These matrices are used only by world translation; local rotation is inherited.
    input=dict(mapping=dict(squeeze_translation_only=True), mapping_by_hand=dict(
        left=dict(world_position_axis_remap=[
            [0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]]),
        right=dict(world_position_axis_remap=[
            [0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]),
    )),
    diagnostics_path='work_dirs/a10/vr/dual_world_translation_diagnostics.jsonl',
)
