#!/usr/bin/env bash
set -euo pipefail

failures=0

check_contains() {
  local label="$1"
  local actual="$2"
  local expected="$3"
  if [[ "${actual}" == *"${expected}"* ]]; then
    echo "PASS ${label}: ${actual}"
  else
    echo "FAIL ${label}: expected ${expected}, got ${actual}" >&2
    failures=$((failures + 1))
  fi
}

ubuntu_version="$(. /etc/os-release && echo "${VERSION_ID:-unknown}")"
python_version="$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')"
ros_executable="$(command -v ros2 || true)"
ros_version="${ROS_DISTRO:-}"
if [[ -z "${ros_version}" && "${ros_executable}" == /opt/ros/humble/* ]]; then
  ros_version="humble (detected from ${ros_executable})"
fi
ros_version="${ros_version:-not-sourced}"
architecture="$(uname -m)"

check_contains "Ubuntu" "${ubuntu_version}" "22.04"
check_contains "Python" "${python_version}" "3.10"
check_contains "ROS_DISTRO" "${ros_version}" "humble"
check_contains "Architecture" "${architecture}" "aarch64"

echo
if command -v nvcc >/dev/null 2>&1; then
  echo "CUDA toolkit:"
  nvcc --version | tail -n 1
else
  echo "WARN nvcc not found in PATH"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA driver:"
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || true
else
  echo "WARN nvidia-smi not available (normal on some Jetson images)"
fi

echo "TensorRT packages:"
dpkg-query -W 'libnvinfer*' 2>/dev/null | head -n 20 || true

echo "cuDNN packages:"
dpkg-query -W 'libcudnn*' 2>/dev/null | head -n 20 || true

if (( failures > 0 )); then
  echo "Environment check failed with ${failures} blocking mismatch(es)." >&2
  exit 1
fi

echo "Environment baseline is compatible."
