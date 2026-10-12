#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
source /opt/ros/jazzy/setup.bash
set -u

colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DHEX_ARM_ENABLE_CIA402=OFF -DHEX_ARM_BUILD_COMMISSIONING=OFF \
  "$@"

