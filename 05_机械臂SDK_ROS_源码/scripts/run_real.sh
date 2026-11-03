#!/usr/bin/env bash
set -eo pipefail

if [[ "${I_UNDERSTAND_ROBOT_CAN_MOVE:-}" != "YES" ]]; then
  echo "Real mode blocked." >&2
  echo "Set I_UNDERSTAND_ROBOT_CAN_MOVE=YES only after the safety checklist." >&2
  exit 1
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace_dir}/scripts/sdk_env.sh"
source "${workspace_dir}/install/setup.bash"
set -u

robot_ip="${ZHENSHI_ROBOT_IP:-192.168.11.11}"
model_index="${ZHENSHI_MODEL_INDEX:-0}"
model_count="${ZHENSHI_MODEL_COUNT:-2}"

exec ros2 launch zhenshi_driver_py driver.launch.py \
  dry_run:=false \
  robot_ip:="${robot_ip}" \
  sdk_root:="${ZHENSHI_SDK_ROOT}" \
  model_index:="${model_index}" \
  model_count:="${model_count}"
