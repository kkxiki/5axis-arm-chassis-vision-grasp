#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
ROOT="${V5_ROOT:-/home/nvidia/5axis_v5}"
pkill -f "realsense2_camera_node" 2>/dev/null || true
pkill -f "direct_rgbd_server.py" 2>/dev/null || true
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true \
  -p color_width:=1280 -p color_height:=720 -p color_fps:=30 \
  -p depth_width:=1280 -p depth_height:=720 -p depth_fps:=30 \
  >"$ROOT/runtime/realsense_v5.log" 2>&1 &
sleep 5
exec python3 "$ROOT/scripts/direct_rgbd_server.py"
