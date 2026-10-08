"""Native FluxVLA VR entrypoint."""

import argparse
from .training import _project
from .session import run_session


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vr-only', action='store_true')
    args = parser.parse_args()
    run_session(_project(), vr_only=args.vr_only)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
