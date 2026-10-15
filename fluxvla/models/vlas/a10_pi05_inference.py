"""A10 input and device adapter for the native Triton/CUDA Graph pipeline."""

import torch

from fluxvla.engines.utils.root import VLAS
from .pi05_flowmatching_inference import PI05FlowMatchingInference


@VLAS.register_module()
class A10PI05FlowMatchingInference(PI05FlowMatchingInference):
    """Keep A10's OpenPI input contract while evaluating only valid cameras.

    The generic accelerated model assumes all views are valid and creates
    some fused weights on CPU. Compact the masked left-wrist slot and move
    every materialized weight to the model device before graph capture.
    Kernels compute in BF16 even when source parameters are retained in FP32.
    """

    def __init__(self, num_views=2, triton_max_prompt_len=128, **kwargs):
        if num_views != 2 or triton_max_prompt_len != 128:
            raise ValueError('A10 acceleration requires two views and 128 prompt slots')
        super().__init__(num_views=num_views,
                         triton_max_prompt_len=triton_max_prompt_len, **kwargs)

    @staticmethod
    def compact_inputs(images, img_masks, lang_tokens, lang_masks):
        if tuple(images.shape) != (1, 9, 224, 224):
            raise ValueError('A10 acceleration requires images[1,9,224,224]')
        if (img_masks is None or tuple(img_masks.shape) != (1, 3)
                or img_masks.dtype != torch.bool
                or img_masks.tolist() != [[True, False, True]]):
            raise ValueError('A10 acceleration requires base/left/right mask [True,False,True]')
        if (lang_tokens.ndim != 2 or lang_tokens.shape[0] != 1
                or not 0 < lang_tokens.shape[1] <= 128):
            raise ValueError('A10 acceleration requires at most 128 prompt tokens, batch size 1')
        if lang_masks is not None:
            if lang_masks.shape != lang_tokens.shape or lang_masks.dtype != torch.bool:
                raise ValueError('Prompt mask must be boolean and match token shape')
            length = int(lang_masks.sum().item())
            expected = torch.arange(lang_tokens.shape[1], device=lang_masks.device) < length
            if length == 0 or not torch.equal(lang_masks[0], expected):
                raise ValueError('Prompt mask must be a nonempty valid prefix followed by padding')
        return torch.cat((images[:, :3], images[:, 6:9]), dim=1).contiguous()

    def materialize_inference_weights(self, **kwargs):
        device = self.llm_backbone.embed_tokens.weight.device
        if device.type != 'cuda':
            raise RuntimeError('A10 Triton inference requires the model on CUDA')
        with torch.cuda.device(device):
            super().materialize_inference_weights(**kwargs)
            self._triton_weights = {
                key: value.to(device=device).contiguous()
                for key, value in self._triton_weights.items()
            }

    @torch.inference_mode()
    def predict_action(self, images, lang_tokens, states, img_masks=None,
                       lang_masks=None, **kwargs):
        images = self.compact_inputs(images, img_masks, lang_tokens, lang_masks)
        device = self.llm_backbone.embed_tokens.weight.device
        if device.type != 'cuda':
            raise RuntimeError('A10 Triton inference requires the model on CUDA')
        with torch.cuda.device(device):
            return super().predict_action(
                images=images, lang_tokens=lang_tokens, states=states,
                lang_masks=lang_masks, **kwargs)
