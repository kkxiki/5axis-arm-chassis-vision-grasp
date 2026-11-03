#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

echo "=== INITIALIZE / ENABLE ==="
timeout 40 ros2 service call \
  /zhenshi/initialize \
  std_srvs/srv/Trigger \
  "{}" || true
sleep 3

source scripts/sdk_env.sh
python3 scripts/read_controller_state.py \
  --robot-ip 192.168.11.11 \
  --require-enabled-model 0

request='{model_index: 0, joint_index: 6, delta_rad: 0.008726646259971648, timeout_ms: 30000}'

echo "=== JOG MODEL0 J7 +0.5 DEG ==="
if ! jog_output="$(timeout 40 ros2 service call \
  /zhenshi/jog_joint \
  zhenshi_interfaces/srv/JogJoint \
  "${request}")"; then
  echo "ROS service call failed before a successful response."
  timeout 10 ros2 service call /zhenshi/stop std_srvs/srv/Trigger "{}" || true
  timeout 15 ros2 service call /zhenshi/disable std_srvs/srv/Trigger "{}" || true
  exit 42
fi
echo "${jog_output}"

if ! grep -q "success=True" <<<"${jog_output}"; then
  echo "Jog was rejected; submitting Stop and Disable."
  timeout 10 ros2 service call /zhenshi/stop std_srvs/srv/Trigger "{}" || true
  timeout 15 ros2 service call /zhenshi/disable std_srvs/srv/Trigger "{}" || true
  exit 43
fi

sleep 3
echo "=== POST STATE ==="
timeout 8 ros2 topic echo --once /zhenshi/state

echo "=== DISABLE ==="
timeout 15 ros2 service call /zhenshi/disable std_srvs/srv/Trigger "{}"
