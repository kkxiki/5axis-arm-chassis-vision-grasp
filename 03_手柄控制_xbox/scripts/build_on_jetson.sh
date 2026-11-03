#!/usr/bin/env bash
# Run inside isaac_ros_jetson after copying this directory to /root/XuK.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${ROOT_DIR}/bin"

mkdir -p "${OUT_DIR}"
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  "${ROOT_DIR}/src/xbox_arm_teleop.cpp" \
  -lpthread -o "${OUT_DIR}/xbox_arm_teleop"

echo "Built: ${OUT_DIR}/xbox_arm_teleop"
