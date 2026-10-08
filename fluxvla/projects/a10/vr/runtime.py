"""Build platform-owned VR input and mapping from the shared config."""

from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace

from .input import VRInput
from .mapper import XLeVRDeltaEEMapper


def build_vr(cfg):
    network = dict(cfg['vr_server'])
    directory = Path(network.pop('tls_directory')).resolve()
    network.update(certfile=str(directory / 'cert.pem'),
                   keyfile=str(directory / 'key.pem'))
    settings = SimpleNamespace(**cfg['vr'],
                               xlevr_path=str(Path(__file__).parent),
                               connection_timeout=10.0,
                               server_settings=network)
    names = {field.name for field in fields(XLeVRDeltaEEMapper) if field.init}
    mapper = XLeVRDeltaEEMapper(**{key: value for key, value in cfg['vr'].items()
                                 if key in names})
    return VRInput(settings), mapper
