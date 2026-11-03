#!/usr/bin/env bash

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_base="${ZHENSHI_SDK_ROOT:-${workspace_dir}/vendor/sdk_send_recevice_demo_py/Base_SDK_py}"

case "$(uname -m)" in
  aarch64|arm64)
    sdk_arch="arm"
    ;;
  *)
    echo "This project is configured for the ARM64 upper computer." >&2
    echo "Detected architecture: $(uname -m)" >&2
    return 1
    ;;
esac

rpc_lib_dir="${sdk_base}/robot_command/lib/linux/${sdk_arch}"
state_lib_dir="${sdk_base}/robot_state/lib/linux/${sdk_arch}/22.04"

if [[ ! -d "${rpc_lib_dir}" || ! -d "${state_lib_dir}" ]]; then
  echo "SDK libraries do not match ARM64 Ubuntu 22.04." >&2
  echo "RPC: ${rpc_lib_dir}" >&2
  echo "State: ${state_lib_dir}" >&2
  return 1
fi

export ZHENSHI_SDK_ROOT="${sdk_base}"
export PYTHONPATH="${sdk_base}:${sdk_base}/robot_state:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${rpc_lib_dir}:${state_lib_dir}:${LD_LIBRARY_PATH:-}"

protobuf_lib="${state_lib_dir}/libprotobuf.so.32"
if [[ -f "${protobuf_lib}" ]]; then
  export LD_PRELOAD="${protobuf_lib}${LD_PRELOAD:+:${LD_PRELOAD}}"
fi

echo "ZHENSHI_SDK_ROOT=${ZHENSHI_SDK_ROOT}"

