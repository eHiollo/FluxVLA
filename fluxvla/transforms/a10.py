"""A10 observation layout used by the local OpenPI pi0.5 policy."""

import numpy as np

from fluxvla.engines import TRANSFORMS


@TRANSFORMS.register_module()
class PrepareA10OpenPIInputs:
    """Order cameras as base, absent left wrist, right wrist; fix the prompt."""

    def __init__(self, task_description: str):
        self.task_description = task_description

    def __call__(self, data: dict) -> dict:
        if len(data['images']) != 2 or len(data['img_masks']) != 2:
            raise ValueError('A10 data must contain base and right wrist images')
        base, right = data['images']
        data['images'] = [base, np.zeros_like(base), right]
        data['img_masks'] = np.asarray(
            [data['img_masks'][0], False, data['img_masks'][1]], dtype=bool)
        data['task_description'] = self.task_description
        return data
