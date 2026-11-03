#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_LIBRARY="${SDK_LIBRARY:-/root/XuK/src/yolo_subscriber_cpp/lib/linux/Release}"
export LD_LIBRARY_PATH="${SDK_LIBRARY}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "${ROOT_DIR}/bin/xbox_arm_teleop" "$@"
