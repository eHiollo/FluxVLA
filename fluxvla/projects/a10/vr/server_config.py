"""Local VR network settings; certificates are created at the site on demand."""

import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class XLeVRConfig:
    host_ip: str
    https_port: int
    websocket_port: int
    certfile: str
    keyfile: str
    enable_vr: bool = True
    enable_https: bool = True
    enable_keyboard: bool = False
    vr_to_robot_scale: float = 1.0

    @property
    def ssl_files_exist(self):
        return Path(self.certfile).is_file() and Path(self.keyfile).is_file()

    def ensure_ssl_certificates(self):
        if self.ssl_files_exist:
            return True
        Path(self.certfile).parent.mkdir(parents=True, exist_ok=True)
        Path(self.keyfile).parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run([
            'openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
            '-keyout', self.keyfile, '-out', self.certfile, '-days', '365',
            '-subj', '/CN=FluxVLA-local-VR'], capture_output=True, check=False)
        if result.returncode == 0:
            Path(self.keyfile).chmod(0o600)
        return result.returncode == 0
