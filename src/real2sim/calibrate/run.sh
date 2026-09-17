#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname -- "$HERE")"
export PYTHONPATH="$PROJECT/deps:$PROJECT/robot_sync_v1/deps/xArm-Python-SDK-master${PYTHONPATH:+:$PYTHONPATH}"
exec /home/wsx/code/GsminiSDK/.venv/bin/python -B "$HERE/handeye.py" "$@"
