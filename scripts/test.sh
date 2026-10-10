#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
level="${1:-unit}"
cd "${workspace_dir}"
source /opt/ros/jazzy/setup.bash
test ! -f install/setup.bash || source install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1

case "${level}" in
  unit|legacy)
    python3 scripts/test-runtime-packaging.py
    python3 scripts/test-transport-benchmark.py
    python3 scripts/test-direct-transport.py
    bash scripts/test-supervised-real-launch.sh
    python3 scripts/test-shutdown-ack.py
    python3 scripts/test-read-cia402-state.py
    python3 scripts/test-calibrate-zero.py
    python3 scripts/test-commission-cia402.py
    python3 scripts/test-check-joint-direction.py
    python3 scripts/test-prepare-moveit-profile.py
    cargo_features=()
    if [[ "${level}" == legacy ]]; then
      cargo_features=(--features legacy)
    else
      cargo fmt --manifest-path src/hex_arm_controller/Cargo.toml --check
      CARGO_TARGET_DIR="${workspace_dir}/build/hex_arm_controller/cargo" \
        cargo clippy --locked --manifest-path src/hex_arm_controller/Cargo.toml --all-targets -- -D warnings
    fi
    CARGO_TARGET_DIR="${workspace_dir}/build/hex_arm_controller/cargo" \
      cargo test --locked --manifest-path src/hex_arm_controller/Cargo.toml "${cargo_features[@]}"
    colcon test --event-handlers console_direct+ \
      --packages-select \
        hex_arm_msgs \
        hex_arm_description \
        hex_arm_tools \
        hex_arm_hardware \
        hex_arm_bringup \
        hex_arm_moveit_runtime \
        hex_arm_moveit_config
    colcon test-result --verbose
    ;;
  protocol)
    python3 src/hex_arm_bringup/test/test_protocol_smoke.py
    ;;
  mock)
    python3 src/hex_arm_bringup/test/test_mock_trajectory.py
    ;;
  gz)
    python3 src/hex_arm_bringup/test/test_gz_trajectory.py
    ;;
  *)
    echo "usage: $0 {unit|legacy|protocol|mock|gz}" >&2
    exit 2
    ;;
esac
