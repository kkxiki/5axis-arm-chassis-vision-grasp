#!/usr/bin/env bash
set -euo pipefail
CONTAINER="${CONTAINER:-isaac_ros_jetson}"
# Stops only the local process. It never sends controller {Stop} (P37).
docker exec "${CONTAINER}" bash -lc "pkill -TERM -x arm_gateway || true"
echo "ArmGateway process stop requested (no robot Stop command sent)."
