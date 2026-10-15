"""A10 inference defaults to native Triton + CUDA Graph (BF16 kernels).

Local inference:
  python scripts/inference.py --config configs/a10/a10_openpi_inference.py
Model server only (no robot connection):
  python -m fluxvla.engines.runners.serving.serve \
    --config configs/a10/a10_openpi_inference.py --from-config --dtype fp32
Then use a10_remote_inference.py on the robot side.
Set A10_OPENPI_CHECKPOINT / A10_OPENPI_TOKENIZER to override local paths.
"""

_base_ = []
import os
from pathlib import Path
from mmengine import Config

_root = Path('{{fileDirname}}').resolve().parents[1]
_checkpoint = os.environ.get('A10_OPENPI_CHECKPOINT', str(
    _root / 'checkpoints/a10_openpi_90000'))
_tokenizer = os.environ.get('A10_OPENPI_TOKENIZER', str(
    _root / 'checkpoints/tokenizers/paligemma_tokenizer.model'))
_stats = str(Path(_checkpoint) / 'assets/dataset/a10_vr_9_21_4/norm_stats.json')
_architecture = Config.fromfile(str(
    _root / 'configs/pi05/pi05_paligemma_aloha_full_finetune.py')).model.to_dict()
_architecture.update(type='A10PI05FlowMatchingInference',
                     num_views=2, triton_max_prompt_len=128,
                     pretrained_name_or_path=None, n_action_steps=30,
                     ori_action_dim=7, num_steps=10,
                     enable_mixed_precision_training=False)
_architecture['llm_backbone']['type'] = 'ConditionGemmaInferenceModel'
_architecture['llm_expert']['type'] = 'ConditionGemmaInferenceModel'
_architecture['vision_backbone']['type'] = 'SigLIPViTBackboneInference'
for _key in ('projector', 'action_in_proj', 'action_out_proj',
             'time_mlp_in', 'time_mlp_out'):
    _architecture[_key]['type'] = 'LinearProjectorInference'
inference_model = dict(type='OpenPIA10Policy', checkpoint_dir=_checkpoint,
                       model_config=_architecture)
inference = dict(
    type='A10InferenceRunner', model_from_config=True,
    dataset=dict(type='OpenPIA10InferenceDataset', norm_stats=_stats,
                 tokenizer_path=_tokenizer),
    denormalize_action=dict(type='OpenPIA10DenormalizeAction', norm_stats=_stats),
    operator=dict(type='A10Operator', host='127.0.0.1',
                  command_mode='joint', allow_motion=False,
                  status_poll_interval_s=0.05, status_max_age_s=0.5,
                  gripper_transport='usb',
                  gripper=dict(port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                               max_width_mm=85., timeout=0.015),
                  cameras=dict(
                      base_0_rgb=dict(device='/dev/v4l/by-id/usb-SYD_USB_Camera_200901010001-video-index0',
                                      width=640, height=480, fps=30),
                      right_wrist_0_rgb=dict(device='/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0',
                                             width=640, height=480, fps=30, rotate_180=True))),
    task_suite_name='a10', instruction='Reach the yellow lemon',
    action_chunk=30, execute_horizon=10, action_dt=1.0 / 30.0,
    max_publish_step=30, gripper_mm_range=(0., 85.), dry_run=True, max_joint_step_rad=None,
    # Fused inference kernels own BF16 compute; retain checkpoint params in FP32.
    mixed_precision_dtype='float32', enable_mixed_precision=False,
    keep_params_fp32=True,
)
