"""Compatibility entrypoint: common A10 control with the OpenPI backend."""
_base_ = ["./a10_dual_inference.py"]
model_backend = "openpi"
inference = dict(
    remote_inference=dict(_delete_=True, protocol="openpi_ws", server_host="127.0.0.1",
                          server_port=8001, timeout_s=3600.0,
                          client_root="../openpi/packages/openpi-client/src"),
    prompt=None,
)
