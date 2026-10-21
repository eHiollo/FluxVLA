"""Original A10 bridge image processing and OpenPI PI0.5 prompt semantics."""

import cv2
import numpy as np
import torch

from fluxvla.engines.utils.root import DATASETS
from fluxvla.transforms.a10_openpi import load_a10_stats


@DATASETS.register_module()
class OpenPIA10InferenceDataset:
    def __init__(self, norm_stats, tokenizer_path, max_token_len=128,
                 model_path=None):
        import sentencepiece
        del model_path
        if type(max_token_len) is not int or max_token_len <= 0:
            raise ValueError('max_token_len must be positive')
        self.stats = load_a10_stats(norm_stats)['proprio']
        self.tokenizer = sentencepiece.SentencePieceProcessor(model_file=str(tokenizer_path))
        self.max_token_len = max_token_len

    @staticmethod
    def _image(image):
        image = np.asarray(image)
        if (image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8
                or min(image.shape[:2]) == 0):
            raise ValueError('A10 images must be nonempty HWC uint8 RGB')
        h, w = image.shape[:2]
        scale = min(224 / w, 224 / h)
        width, height = max(1, round(w * scale)), max(1, round(h * scale))
        resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        padded = np.zeros((224, 224, 3), dtype=np.uint8)
        y, x = (224 - height) // 2, (224 - width) // 2
        padded[y:y + height, x:x + width] = resized
        return padded.transpose(2, 0, 1).astype(np.float32) / 255. * 2. - 1.

    def __call__(self, observation):
        state = np.asarray(observation['qpos'], dtype=np.float32)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('Expected finite actual qpos[7]')
        low, high = self.stats['q01'], self.stats['q99']
        normalized = (state - low) / (high - low + 1e-6) * 2. - 1.
        # Do not clip or pad before discretizing: OpenPI uses all seven values.
        bins = np.digitize(normalized, np.linspace(-1, 1, 257)[:-1]) - 1
        task = observation['task_description']
        if not isinstance(task, str) or not task.strip():
            raise ValueError('Expected nonempty task_description')
        task = task.strip().replace('_', ' ').replace('\n', ' ')
        prompt = f'Task: {task}, State: {" ".join(map(str, bins))};\nAction: '
        tokens = self.tokenizer.encode(prompt, add_bos=True)[:self.max_token_len]
        mask = [True] * len(tokens) + [False] * (self.max_token_len - len(tokens))
        tokens += [0] * (self.max_token_len - len(tokens))
        base = self._image(observation['base_0_rgb'])
        right = self._image(observation['right_wrist_0_rgb'])
        # FluxVLA's vision backbone packs the three RGB views on channel axis.
        images = np.concatenate([base, np.full_like(base, -1.), right], axis=0)
        return dict(
            images=torch.from_numpy(images[None]),
            img_masks=torch.tensor([[True, False, True]]),
            lang_tokens=torch.tensor([tokens], dtype=torch.long),
            lang_masks=torch.tensor([mask], dtype=torch.bool),
            states=torch.tensor(np.pad(normalized, (0, 25))[None], dtype=torch.float32))


@DATASETS.register_module()
class A10RTCInferenceDataset:
    """Match the native RTC export's mean/std, HF tokenizer and PIL images."""

    def __init__(self, norm_stats, tokenizer_path, max_token_len=128):
        import json
        from pathlib import Path
        from fluxvla.engines import build_transform_from_cfg
        if isinstance(norm_stats, (str, Path)):
            norm_stats = json.loads(Path(norm_stats).read_text())
        self.stats = norm_stats['a10']
        pipeline = [
            dict(type='NormalizeStatesAndActions', norm_type='mean_std',
                 state_key='proprio', action_key=None, output_dtype='float32'),
            dict(type='PreparePromptWithState'),
            dict(type='ProcessPrompts', max_len=max_token_len,
                 tokenizer=dict(type='PretrainedTokenizer', model_path=str(tokenizer_path),
                                trust_remote_code=False)),
            dict(type='PadStatesAndActions', model_action_dim=32),
            dict(type='ResizeImagesWithPad', height=224, width=224, backend='pil'),
            dict(type='SimpleNormalizeImages'),
        ]
        self.transforms = [build_transform_from_cfg(item) for item in pipeline]

    def __call__(self, observation):
        state = np.asarray(observation['qpos'], dtype=np.float32)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('Expected finite actual qpos[7]')
        images = []
        for key in ('base_0_rgb', 'right_wrist_0_rgb'):
            image = np.asarray(observation[key])
            if (image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8
                    or min(image.shape[:2]) == 0):
                raise ValueError('A10 images must be nonempty HWC uint8 RGB')
            images.append(image.transpose(2, 0, 1))
        data = dict(states=state.copy(), stats=self.stats,
                    task_description=observation['task_description'],
                    images=[images[0], np.zeros_like(images[0]), images[1]])
        for transform in self.transforms:
            data = transform(data)
        return dict(images=torch.as_tensor(data['images'][None], dtype=torch.float32),
                    img_masks=torch.tensor([[True, False, True]]),
                    states=torch.as_tensor(data['states'][None], dtype=torch.float32),
                    lang_tokens=torch.as_tensor(data['lang_tokens'][None], dtype=torch.long),
                    lang_masks=torch.as_tensor(data['lang_masks'][None], dtype=torch.bool))
