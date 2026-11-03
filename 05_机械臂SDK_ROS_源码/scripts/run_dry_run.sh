#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

exec ros2 launch zhenshi_driver_py driver.launch.py \
  dry_run:=true \
  model_index:=0 \
  model_count:=2
