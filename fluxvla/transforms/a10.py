"""A10 observation layout used by the local OpenPI pi0.5 policy."""

import numpy as np

from fluxvla.engines import TRANSFORMS


@TRANSFORMS.register_module()
class PrepareA10OpenPIInputs:
    """Prepare single- or dual-arm A10 cameras and a fixed task prompt."""

    def __init__(self, task_description: str, dual_arm: bool = False):
        self.task_description = task_description
        self.dual_arm = dual_arm

    def __call__(self, data: dict) -> dict:
        if self.dual_arm:
            if (len(data['images']) != 3 or len(data['img_masks']) != 3
                    or not np.asarray(data['img_masks']).all()):
                raise ValueError(
                    'Dual-arm A10 requires base, left wrist, and right wrist images'
                )
            if np.asarray(data['states']).shape[-1] != 14:
                raise ValueError('Dual-arm A10 requires 14-dimensional states')
            if 'actions' in data and np.asarray(
                    data['actions']).shape[-1] != 14:
                raise ValueError(
                    'Dual-arm A10 requires 14-dimensional actions')
            data['img_masks'] = np.asarray(data['img_masks'], dtype=bool)
            data['task_description'] = self.task_description
            return data
        if len(data['images']) != 2 or len(data['img_masks']) != 2:
            raise ValueError(
                'A10 data must contain base and right wrist images')
        base, right = data['images']
        data['images'] = [base, np.zeros_like(base), right]
        data['img_masks'] = np.asarray(
            [data['img_masks'][0], False, data['img_masks'][1]], dtype=bool)
        data['task_description'] = self.task_description
        return data
