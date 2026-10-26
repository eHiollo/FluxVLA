"""Run the pure offline VR reference tool without importing model/GPU packages."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] /
                      'fluxvla/engines/operators/vr/control.py'), run_name='__main__')
