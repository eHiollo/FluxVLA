from pathlib import Path

import numpy as np
import pytest
from mmengine import Config

from fluxvla.transforms.a10 import PrepareA10OpenPIInputs
from fluxvla.transforms.transform_actions import AbsoluteActions, RelativeActions

ROOT = Path(__file__).resolve().parents[2]
TASK = 'Fold the garment neatly using both arms.'


def sample():
    return dict(
        images=[np.full((8, 8, 3), i, dtype=np.uint8) for i in (10, 20, 30)],
        img_masks=[True] * 3,
        states=np.arange(14, dtype=np.float32),
        actions=np.arange(420, dtype=np.float32).reshape(30, 14))


def test_dual_config():
    c = Config.fromfile(ROOT /
                        'configs/pi05/pi05_paligemma_a10_dual_fold_10steps.py')
    assert c.model.ori_action_dim == 14
    assert c.model.n_action_steps == 30
    assert c.runner.max_steps == 10
    assert c.inference is None
    ds = c.train_dataloader.dataset.datasets
    assert ds.window_start_idx == 0 and ds.supervise_terminal_padding
    assert len(ds.transforms[0].video_keys) == 3
    assert ds.transforms[1].dual_arm
    assert ds.transforms[2].mask == ([True] * 6 + [False]) * 2
    assert ds.transforms[3].norm_type == 'quantile'
    assert len(c.train_dataloader.dataset.dataset_statistics.a10_dual.action.
               q01) == 14


def test_dual_cameras_and_delta_roundtrip():
    s = sample()
    original = s['actions'].copy()
    out = PrepareA10OpenPIInputs(TASK, dual_arm=True)(s)
    assert [int(x[0, 0, 0]) for x in out['images']] == [10, 20, 30]
    assert out['task_description'] == TASK and out['img_masks'].all()
    mask = ([True] * 6 + [False]) * 2
    out = RelativeActions(mask=mask)(out)
    np.testing.assert_array_equal(out['actions'][:, [6, 13]],
                                  original[:, [6, 13]])
    joints = np.where(mask)[0]
    np.testing.assert_array_equal(out['actions'][:, joints],
                                  original[:, joints] - s['states'][joints])
    np.testing.assert_array_equal(
        AbsoluteActions(mask=mask)(out)['actions'], original)


def test_single_arm_unchanged():
    out = PrepareA10OpenPIInputs('old')(
        dict(images=[np.ones((8, 8, 3))] * 2, img_masks=[True, True]))
    assert list(out['img_masks']) == [True, False, True]
    assert not out['images'][1].any()


@pytest.mark.parametrize('field,value', [('states', np.ones(7)),
                                         ('actions', np.ones((30, 7))),
                                         ('images', [np.zeros((8, 8, 3))] * 2),
                                         ('img_masks', [True, False, True])])
def test_dual_rejects_incompatible_input(field, value):
    s = sample()
    s[field] = value
    with pytest.raises(ValueError, match='Dual-arm A10'):
        PrepareA10OpenPIInputs(TASK, dual_arm=True)(s)
