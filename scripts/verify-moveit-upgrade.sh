#!/usr/bin/env bash
# CI-provider-independent ABI, startup-gate and shutdown regression entrypoint.
# Run in the proposed ROS/Rust base image, with this checkout mounted as cwd.
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_dir}"
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
colcon build --packages-up-to hex_arm_moveit_config --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DHEX_ARM_BUILD_COMMISSIONING=OFF
set +u
source install/setup.bash
set -u
python3 scripts/test-runtime-packaging.py
python3 -m pytest -q \
  src/hex_arm_moveit_config/test/test_moveit_config.py \
  src/hex_arm_moveit_config/test/test_moveit_real_launch.py \
  src/hex_arm_moveit_config/test/test_moveit_plan_only.py \
  src/hex_arm_moveit_config/test/test_deployment_planning.py \
  src/hex_arm_moveit_config/test/test_startup_execution_gate.py \
  src/hex_arm_moveit_config/test/test_moveit_mock.py
echo 'MoveIt upgrade contract passed; update the reviewed ABI/digest only after reviewing upstream teardown changes.'
