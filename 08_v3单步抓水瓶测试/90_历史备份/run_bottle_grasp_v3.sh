#!/usr/bin/env bash
set -eu
cd /home/nvidia/calibration
exec python3 bottle_grasp_v3.py --execute --open-loop --force 60 --gripper-speed 40
