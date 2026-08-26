#!/usr/bin/env bash

set -euo pipefail

export CCACHE_DISABLE=1

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="${script_dir}"

rm -rf "${workspace_root}/build" "${workspace_root}/install" "${workspace_root}/log"

cd "${workspace_root}"

colcon build \
  --event-handlers console_direct+ \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -G Ninja
