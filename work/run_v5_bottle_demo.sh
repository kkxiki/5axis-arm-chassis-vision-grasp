#!/usr/bin/env bash
set -euo pipefail

cd /home/nvidia/5axis_v5
exec python3 scripts/run_v5_fixed_demo_once.py --speed v25 --slow-speed v25
