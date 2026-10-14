"""A10 v3 train/valid splits labeled with Qwen3-VL-4B."""
_base_ = ['./sarm_a10.py']

data_root_path = './datasets/a10_sarm_v3_train'
model = dict(data_root_path=data_root_path)
train_dataloader = dict(dataset=dict(data_root_path=data_root_path))
inference_dataset = dict(data_root_path='./datasets/a10_sarm_v3_valid')
runner = dict(metric=dict(run_dir='./sarm_runs/a10_qwen4b'))
