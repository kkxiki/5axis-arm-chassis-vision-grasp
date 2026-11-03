#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_dir="${workspace_dir}/vendor"
target_dir="${vendor_dir}/sdk_send_recevice_demo_py"
repo_url="https://gitee.com/zhenshi-controller-api-data/sdk_send_recevice_demo_py.git"
expected_commit="10fcb48e684d7463db11f1ae27c08c15549ee43e"

mkdir -p "${vendor_dir}"

if [[ ! -d "${target_dir}/.git" ]]; then
  if [[ -e "${target_dir}" ]]; then
    echo "Target exists but is not a git repository: ${target_dir}" >&2
    exit 1
  fi
  git clone "${repo_url}" "${target_dir}"
fi

current_commit="$(git -C "${target_dir}" rev-parse HEAD)"
if [[ "${current_commit}" != "${expected_commit}" ]]; then
  git -C "${target_dir}" fetch origin "${expected_commit}"
  git -C "${target_dir}" checkout --detach "${expected_commit}"
fi

actual_commit="$(git -C "${target_dir}" rev-parse HEAD)"
if [[ "${actual_commit}" != "${expected_commit}" ]]; then
  echo "SDK commit verification failed: ${actual_commit}" >&2
  exit 1
fi

echo "SDK ready: ${target_dir}"
echo "Commit: ${actual_commit}"

