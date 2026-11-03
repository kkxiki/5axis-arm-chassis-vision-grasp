#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${CONTAINER:-isaac_ros_jetson}"
PROJECT="${PROJECT:-/root/XuK/arm_gateway}"
docker exec -d "${CONTAINER}" bash -lc \
  "mkdir -p '${PROJECT}/log'; LD_LIBRARY_PATH=/root/XuK/src/yolo_subscriber_cpp/lib/linux/Release:\${LD_LIBRARY_PATH:-} exec '${PROJECT}/bin/arm_gateway' --config '${PROJECT}/config/arm_gateway.yaml' >>'${PROJECT}/log/arm_gateway.log' 2>&1"
echo "ArmGateway started detached. Log: ${PROJECT}/log/arm_gateway.log"
