#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_INCLUDE="${SDK_INCLUDE:-/root/XuK/src/yolo_subscriber_cpp/include/include}"
SDK_LIBRARY="${SDK_LIBRARY:-/root/XuK/src/yolo_subscriber_cpp/lib/linux/Release}"
LOCAL_SDK="${ROOT_DIR}/build/vendor_include"
mkdir -p "${ROOT_DIR}/bin" "${LOCAL_SDK}" "${ROOT_DIR}/build/include"
cp -a "${SDK_INCLUDE}/." "${LOCAL_SDK}/"
# resp_dto.h does `#include "../include/easy_json.h"`, which from
# build/vendor_include/ resolves to build/include/easy_json.h, so mirror the
# header there too.
cp "${SDK_INCLUDE}/easy_json.h" "${ROOT_DIR}/build/include/easy_json.h"
# Vendor destructor shuts down and joins but does not close the fd. Patch only
# this private build copy, so reconnect really releases the sole 5868 socket.
python3 "${ROOT_DIR}/scripts/patch_vendor_sdk.py" "${LOCAL_SDK}/cpp_rpc.hpp"
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -DCODEIT_1_3_0 -DCPP_RPC_LIB \
  -I"${LOCAL_SDK}" -I"${LOCAL_SDK}/message" "${ROOT_DIR}/src/arm_gateway.cpp" \
  -L"${SDK_LIBRARY}" -Wl,-rpath,"${SDK_LIBRARY}" -lcpp_rpc -lpthread \
  -o "${ROOT_DIR}/bin/arm_gateway"
echo "Built: ${ROOT_DIR}/bin/arm_gateway"
