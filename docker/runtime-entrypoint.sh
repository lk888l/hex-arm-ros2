#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/hex-arm/local_setup.bash
set -u

# Debugging the IMAGE is possible without automatically starting the arm.
if [[ "${1:-run}" != run ]]; then
  exec "$@"
fi
shift || true
if (( $# != 0 )); then
  echo "error: 'run' takes no extra arguments; use environment/profile configuration" >&2
  exit 2
fi
profile="${HEX_ARM_PROFILE:-/etc/hex-arm/hardware.yaml}"
model="/opt/hex-arm/share/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf"
if [[ ! -f "${profile}" ]]; then
  echo "error: mount a verified Meow / SocketCAN profile at ${profile}" >&2
  exit 2
fi
mkdir -p "${ROS_LOG_DIR}"

# A fresh log directory per invocation prevents stale stop confirmations.
run_log="$(mktemp -d "${ROS_LOG_DIR}/run.XXXXXXXX")"
export ROS_LOG_DIR="${run_log}"
export HEX_ARM_SHUTDOWN_REPORT="${run_log}/driver-shutdown.json"
/opt/hex-arm/lib/hex_arm_controller/hex_arm_controller \
  --profile "${profile}" --urdf "${model}" --validate-profile-only
echo "hex-arm: runtime audit directory ${run_log}"

# Production always unfolds in the verified J2 -> J4 -> J3 order.
# The wrapper retains ROS for normal return/damping before propagating SIGINT.
# TERM and repeated INT request immediate teardown; faults never initiate motion.
exec python3 /opt/hex-arm/lib/hex_arm_bringup/graceful-real-launch.py \
  --profile "${profile}" --scope moveit -- \
  ros2 launch hex_arm_moveit_config moveit_real.launch.py \
  "hardware_profile:=${profile}" enable_execution:=true startup_ready:=true use_rviz:=false
