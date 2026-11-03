#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "${workspace_dir}"

python3 -m compileall -q src/zhenshi_driver_py/zhenshi_driver_py
colcon build --symlink-install

echo "Build complete."
echo "Run: source ${workspace_dir}/install/setup.bash"
