#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-up}"
gpu_mode="${HEX_ARM_GPU:-auto}"
headless="${HEX_ARM_HEADLESS:-0}"
can_interface="${HEX_ARM_CAN_IFACE:-}"
can_serial="${HEX_ARM_CAN_SERIAL:-}"
can_channel="${HEX_ARM_CAN_CHANNEL:-}"
gpu_enabled=0
if (( $# > 0 )); then
  shift
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker is not installed or is not on PATH" >&2
  exit 1
fi

compose_files=(-f "${workspace_dir}/compose.yaml")
platform="WSL2"

case "${gpu_mode}" in
  auto|nvidia|none) ;;
  *)
    echo "error: HEX_ARM_GPU must be one of: auto, nvidia, none" >&2
    exit 2
    ;;
esac
case "${headless}" in
  0|1) ;;
  *) echo "error: HEX_ARM_HEADLESS must be 0 or 1" >&2; exit 2 ;;
esac

if [[ -n "${can_interface}" && ! "${can_interface}" =~ ^[[:alnum:]_.-]{1,15}$ ]]; then
  echo "error: HEX_ARM_CAN_IFACE must be a conventional Linux interface name (1..15 bytes)" >&2
  exit 2
fi
if [[ -n "${can_serial}" && ! "${can_serial}" =~ ^[[:xdigit:]]{32}$ ]]; then
  echo "error: HEX_ARM_CAN_SERIAL must be the exact 32-digit USB serial" >&2
  exit 2
fi
if [[ -n "${can_channel}" && ! "${can_channel}" =~ ^[0-3]$ ]]; then
  echo "error: HEX_ARM_CAN_CHANNEL must be one of: 0, 1, 2, 3" >&2
  exit 2
fi

# Names such as can0/can7 do not identify the physical USB channel. Resolve
# omitted fingerprint fields from the explicitly selected netdev, never a
# machine-specific serial or an interface-name suffix.
if [[ -n "${can_interface}" && ( -z "${can_serial}" || -z "${can_channel}" ) ]]; then
  binding="$(python3 "${workspace_dir}/scripts/bind-can-profile.py" --interface "${can_interface}")"
  read -r detected_serial detected_channel <<<"${binding}"
  can_serial="${can_serial:-${detected_serial}}"
  can_channel="${can_channel:-${detected_channel}}"
fi

if [[ "${headless}" == "1" ]]; then
  platform="headless"
  compose_files=(-f "${workspace_dir}/compose.headless.yaml")
elif ! uname -r | tr '[:upper:]' '[:lower:]' | grep -q microsoft; then
  platform="native Ubuntu"
  compose_files=(-f "${workspace_dir}/compose.ubuntu.yaml")

  host_has_nvidia=0
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
    host_has_nvidia=1
  fi

  if [[ "${gpu_mode}" == "nvidia" && "${host_has_nvidia}" != "1" ]]; then
    echo "error: HEX_ARM_GPU=nvidia was requested, but no working NVIDIA GPU was found" >&2
    exit 1
  fi
  if [[ "${gpu_mode}" == "nvidia" || ( "${gpu_mode}" == "auto" && "${host_has_nvidia}" == "1" ) ]]; then
    if ! docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
      echo "error: an NVIDIA GPU was detected, but the Docker NVIDIA runtime is unavailable" >&2
      echo "hint: install NVIDIA Container Toolkit, run 'sudo nvidia-ctk runtime configure --runtime=docker', then restart Docker" >&2
      echo "hint: use HEX_ARM_GPU=none only when software/AMD/Intel rendering is intentional" >&2
      exit 1
    fi
    gpu_enabled=1
    compose_files+=(-f "${workspace_dir}/compose.nvidia.yaml")
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    echo "error: DISPLAY is empty; run this command from the local Ubuntu graphical session" >&2
    exit 1
  fi

  display_number="${DISPLAY##*:}"
  display_number="${display_number%%.*}"
  x_socket="/tmp/.X11-unix/X${display_number}"
  if [[ ! -S "${x_socket}" ]]; then
    echo "error: X11 socket ${x_socket} does not exist (DISPLAY=${DISPLAY})" >&2
    exit 1
  fi

  host_xauthority="${XAUTHORITY:-}"
  if [[ -z "${host_xauthority}" ]]; then
    login_home="$(getent passwd "$(id -u)" | cut -d: -f6)"
    if [[ -r "${login_home}/.Xauthority" ]]; then
      host_xauthority="${login_home}/.Xauthority"
    fi
  fi
  if [[ -z "${host_xauthority}" || ! -r "${host_xauthority}" ]]; then
    echo "error: no readable XAUTHORITY file was found for DISPLAY=${DISPLAY}" >&2
    echo "hint: run 'echo \$XAUTHORITY' in the desktop terminal and retry there" >&2
    exit 1
  fi
  if [[ "${gpu_enabled}" != "1" && ! -e /dev/dri ]]; then
    echo "error: /dev/dri is missing; the non-NVIDIA Ubuntu GUI container requires a DRM device" >&2
    echo "hint: on an NVIDIA host, install NVIDIA Container Toolkit and use HEX_ARM_GPU=auto or nvidia" >&2
    exit 1
  fi
  export HEX_ARM_XAUTHORITY="${host_xauthority}"
elif [[ "${gpu_mode}" == "nvidia" ]]; then
  echo "error: HEX_ARM_GPU=nvidia is supported by this helper only on native Ubuntu" >&2
  exit 1
fi

if [[ "${HEX_ARM_REAL:-0}" == "1" ]]; then
  compose_files+=(-f "${workspace_dir}/compose.real.yaml")
fi

compose=(docker compose "${compose_files[@]}")

ensure_container_running() {
  if ! docker inspect --format '{{.State.Running}}' ros2-jazzy-arm 2>/dev/null | grep -qx true; then
    "${compose[@]}" up -d
  fi
}

new_real_launch_token() {
  local uuid token

  if [[ ! -r /proc/sys/kernel/random/uuid ]]; then
    echo "error: cannot generate a unique real-launch token from the kernel RNG" >&2
    return 1
  fi
  IFS= read -r uuid </proc/sys/kernel/random/uuid
  token="${uuid//-/}"
  if [[ ! "${token}" =~ ^[0-9a-f]{32}$ ]]; then
    echo "error: kernel RNG returned an invalid real-launch token" >&2
    return 1
  fi
  printf '%s\n' "${token}"
}

run_supervised_real_launch() {
  local launch_token real_exec_pid real_exec_status wait_status second_wait_status
  local requested_signal relay_attempted relay_status trap_count wait_trap_count

  command -v setsid >/dev/null || {
    echo "error: setsid is required for reliable host-to-container signal forwarding" >&2
    return 1
  }
  launch_token="$(new_real_launch_token)"
  real_exec_pid=""
  real_exec_status=1
  requested_signal=""
  relay_attempted=0
  relay_status=0
  trap_count=0

  relay_real_launch_signal() {
    if [[ -z "${real_exec_pid}" || -z "${requested_signal}" || "${relay_attempted}" == "1" ]]; then
      return 0
    fi
    relay_attempted=1
    echo "host supervisor: relaying ${requested_signal} to real-launch token ${launch_token}; keeping the primary exec attached" >&2
    # This second, non-TTY Docker exec has exactly one authority: ask the
    # container helper to validate this invocation's token/PGID state and
    # signal that negative PGID. It receives no CAN interface and cannot use a
    # process-name-wide selector.
    if setsid -w docker exec ros2-jazzy-arm \
      /workspaces/hex_arm_ros2/scripts/supervised-real-launch.sh \
      --forward-signal "${launch_token}" "${requested_signal}"; then
      relay_status=0
    else
      relay_status="$?"
      relay_attempted=0
      echo "error: real-launch signal relay failed with status ${relay_status}; the primary exec remains attached" >&2
    fi
  }

  record_real_launch_signal() {
    local signal_name="$1"
    trap_count=$((trap_count + 1))
    if [[ -n "${requested_signal}" ]]; then
      if [[ "${requested_signal}" == "INT" ]]; then
        # A second Ctrl-C must reach the container even while soft stop is pending.
        # TERM bypasses return/damping; it remains scoped to this launch token.
        requested_signal="TERM"
        relay_attempted=0
        relay_real_launch_signal
        return 0
      fi
      echo "host supervisor: ${requested_signal} is already pending; still waiting for verified cleanup" >&2
      if [[ "${relay_attempted}" == "0" ]]; then
        relay_real_launch_signal
      fi
      return 0
    fi
    requested_signal="${signal_name}"
    relay_real_launch_signal
  }

  trap 'record_real_launch_signal INT' INT
  trap 'record_real_launch_signal TERM' TERM
  # Rust deliberately handles INT/TERM. Translate a host hangup to TERM.
  trap 'record_real_launch_signal TERM' HUP

  # Isolate the long-lived Compose client from the host terminal process group:
  # the shell trap remains the only Ctrl-C recipient. `-w` keeps this wrapper
  # alive until the original exec returns after container-side verification.
  setsid -w "${compose[@]}" exec -T \
    -e HEX_ARM_CAN_IFACE="${can_interface}" \
    -e HEX_ARM_CAN_SERIAL="${can_serial}" \
    -e HEX_ARM_CAN_CHANNEL="${can_channel}" \
    -e HEX_ARM_REAL_LAUNCH_TOKEN="${launch_token}" \
    ros2-jazzy-arm \
    /workspaces/hex_arm_ros2/scripts/supervised-real-launch.sh "$@" &
  real_exec_pid="$!"
  relay_real_launch_signal

  # A trapped signal interrupts Bash's wait even though it does not terminate
  # the isolated Compose client. Resume waiting until that same client exits;
  # do not substitute the short-lived relay exec's status for its result.
  while true; do
    wait_trap_count="${trap_count}"
    if wait "${real_exec_pid}"; then
      wait_status=0
    else
      wait_status="$?"
    fi
    if (( trap_count != wait_trap_count )) && kill -0 "${real_exec_pid}" 2>/dev/null; then
      continue
    fi
    if (( trap_count != wait_trap_count )); then
      if wait "${real_exec_pid}" 2>/dev/null; then
        second_wait_status=0
      else
        second_wait_status="$?"
      fi
      if (( second_wait_status != 127 )); then
        wait_status="${second_wait_status}"
      fi
    fi
    real_exec_status="${wait_status}"
    break
  done

  trap - INT TERM HUP
  if [[ -n "${requested_signal}" && "${relay_status}" != "0" ]]; then
    echo "error: ${requested_signal} was requested but its token-scoped relay was not confirmed" >&2
    return 1
  fi
  return "${real_exec_status}"
}

case "${action}" in
  build)
    "${compose[@]}" build "$@"
    ;;
  up)
    "${compose[@]}" up -d "$@"
    if [[ "${gpu_enabled}" == "1" ]]; then
      echo "${platform} NVIDIA container is ready: ./scripts/docker-dev.sh shell"
    else
      echo "${platform} container is ready: ./scripts/docker-dev.sh shell"
    fi
    ;;
  down)
    "${compose[@]}" down "$@"
    ;;
  shell)
    # Do not recreate an existing container with a different override set.
    # In particular this preserves USB mappings from a prior HEX_ARM_REAL=1 up.
    ensure_container_running
    exec "${compose[@]}" exec ros2-jazzy-arm bash "$@"
    ;;
  real-launch)
    if [[ -z "${can_interface}" ]]; then
      echo "error: real-launch requires an explicit HEX_ARM_CAN_IFACE" >&2
      echo "usage: HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 $0 real-launch {bringup|moveit|startup} /absolute/container/profile.yaml [launch_arg:=value ...]" >&2
      exit 2
    fi
    if (( $# < 2 )) || [[ "$1" != "bringup" && "$1" != "moveit" && "$1" != "startup" ]]; then
      echo "error: real-launch requires target bringup, moveit or startup plus an absolute container profile path" >&2
      exit 2
    fi
    ensure_container_running
    if [[ "${headless}" == "1" ]]; then
      run_supervised_real_launch "$@" use_rviz:=false
    else
      run_supervised_real_launch "$@"
    fi
    ;;
  doctor)
    ensure_container_running
    "${compose[@]}" exec -T \
      -e HEX_ARM_EXPECT_NVIDIA="${gpu_enabled}" \
      -e HEX_ARM_HEADLESS="${headless}" \
      -e HEX_ARM_CAN_IFACE="${can_interface}" \
      -e HEX_ARM_CAN_SERIAL="${can_serial}" \
      -e HEX_ARM_CAN_CHANNEL="${can_channel}" \
      ros2-jazzy-arm bash -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      if [[ "${HEX_ARM_HEADLESS}" != "1" ]]; then
      display_number="${DISPLAY##*:}"
      display_number="${display_number%%.*}"
      test -S "/tmp/.X11-unix/X${display_number}"
      xdpyinfo >/dev/null
      echo "X11 authorization: OK"
      glx_output="$(glxinfo -B)"
      printf "%s\n" "${glx_output}" | sed -n "/OpenGL vendor string/p; /OpenGL renderer string/p; /OpenGL core profile version string/p"
      if [[ "${HEX_ARM_EXPECT_NVIDIA}" == "1" ]]; then
        test -c /dev/nvidia0 || { echo "error: /dev/nvidia0 is missing in the container" >&2; exit 1; }
        command -v nvidia-smi >/dev/null || { echo "error: nvidia-smi is missing in the container" >&2; exit 1; }
        nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
        renderer="$(printf "%s\n" "${glx_output}" | sed -n "s/^OpenGL renderer string: //p")"
        if [[ "${renderer}" != *NVIDIA* ]]; then
          echo "error: NVIDIA was requested, but OpenGL renderer is ${renderer:-unknown}" >&2
          exit 1
        fi
        echo "NVIDIA GPU acceleration: OK"
      fi
      else
        echo "Headless mode: no desktop or GPU required"
      fi
      if [[ -n "${HEX_ARM_CAN_IFACE}" ]]; then
        command -v ip >/dev/null || {
          echo "error: iproute2 is missing; rebuild the development image" >&2
          exit 1
        }
        command -v python3 >/dev/null || {
          echo "error: python3 is required for strict SocketCAN JSON validation" >&2
          exit 1
        }
        can_json_before="$(ip -details -statistics -json link show dev "${HEX_ARM_CAN_IFACE}")"
        sleep 0.2
        can_json_after="$(ip -details -statistics -json link show dev "${HEX_ARM_CAN_IFACE}")"
        printf "%s\n%s\n" "${can_json_before}" "${can_json_after}" | python3 -c "
import json, sys
snapshots = [json.loads(line) for line in sys.stdin if line.strip()]
def require(condition, message):
    if not condition:
        raise SystemExit(\"error: SocketCAN preflight: \" + message)
require(len(snapshots) == 2, \"expected exactly two link-statistics samples\")
before_links, links = snapshots
require(len(before_links) == 1, \"expected exactly one interface in the first sample\")
require(len(links) == 1, \"expected exactly one interface\")
before_link = before_links[0]
link = links[0]
require(before_link.get(\"ifname\") == \"${HEX_ARM_CAN_IFACE}\", \"first-sample interface name mismatch\")
require(link.get(\"ifname\") == \"${HEX_ARM_CAN_IFACE}\", \"interface name mismatch\")
flags = link.get(\"flags\", [])
require(\"UP\" in flags and \"LOWER_UP\" in flags, \"link is not UP/LOWER_UP\")
require(link.get(\"operstate\") == \"UP\", \"operstate is not UP\")
require(link.get(\"mtu\") == 72, \"MTU is not 72\")
require(link.get(\"link_type\") == \"can\", \"link type is not CAN\")
linkinfo = link.get(\"linkinfo\", {})
require(linkinfo.get(\"info_kind\") == \"can\", \"link info kind is not CAN\")
can = linkinfo.get(\"info_data\", {})
require(\"FD\" in can.get(\"ctrlmode\", []), \"CAN-FD is not enabled\")
require(can.get(\"state\") == \"ERROR-ACTIVE\", \"state is not ERROR-ACTIVE\")
berr = can.get(\"berr_counter\", {})
require(berr.get(\"tx\") == 0 and berr.get(\"rx\") == 0, \"TEC/REC are nonzero or missing\")
require(can.get(\"restart_ms\") == 0, \"restart-ms is not fail-closed zero\")
nominal = can.get(\"bittiming\", {})
require(nominal.get(\"bitrate\") == 1000000, \"nominal bitrate is not 1M\")
require(float(nominal.get(\"sample_point\", -1)) == 0.8, \"nominal sample point is not 0.800\")
require(nominal.get(\"sjw\") == 5, \"nominal SJW is not 5\")
data = can.get(\"data_bittiming\", {})
require(data.get(\"bitrate\") == 4000000, \"data bitrate is not 4M\")
require(float(data.get(\"sample_point\", -1)) == 0.8, \"data sample point is not 0.800\")
require(data.get(\"sjw\") == 3, \"data SJW is not 3\")
xstats = linkinfo.get(\"info_xstats\", {})
for key in (\"restarts\", \"bus_error\", \"arbitration_lost\", \"error_warning\", \"error_passive\", \"bus_off\"):
    require(xstats.get(key) == 0, \"nonzero or missing cumulative counter \" + key)
before_stats = before_link.get(\"stats64\", {})
stats = link.get(\"stats64\", {})
before_rx = before_stats.get(\"rx\", {})
before_tx = before_stats.get(\"tx\", {})
rx = stats.get(\"rx\", {})
tx = stats.get(\"tx\", {})
for key in (\"errors\", \"over_errors\"):
    require(rx.get(key) == 0, \"nonzero or missing RX counter \" + key)
for key in (\"errors\", \"carrier_errors\", \"collisions\"):
    require(tx.get(key) == 0, \"nonzero or missing TX counter \" + key)
for direction, previous, current in ((\"RX\", before_rx, rx), (\"TX\", before_tx, tx)):
    previous_dropped = previous.get(\"dropped\")
    current_dropped = current.get(\"dropped\")
    require(isinstance(previous_dropped, int) and isinstance(current_dropped, int),
            direction + \" dropped counter is missing or invalid\")
    require(current_dropped == previous_dropped,
            direction + \" dropped counter changed during the 0.2 s sample: \" +
            str(previous_dropped) + \" -> \" + str(current_dropped))
"

        netdev="/sys/class/net/${HEX_ARM_CAN_IFACE}"
        test -d "${netdev}" || { echo "error: sysfs netdev ${netdev} is missing" >&2; exit 1; }
        device="$(readlink -f "${netdev}/device")"
        driver="$(basename "$(readlink -f "${device}/driver")")"
        usb_device="${device}"
        while [[ ! -r "${usb_device}/idVendor" || ! -r "${usb_device}/idProduct" ]]; do
          parent="$(dirname "${usb_device}")"
          [[ "${parent}" != "${usb_device}" ]] || {
            echo "error: no USB adapter ancestor found for ${HEX_ARM_CAN_IFACE}" >&2
            exit 1
          }
          usb_device="${parent}"
        done
        usb_vid="$(tr "[:upper:]" "[:lower:]" < "${usb_device}/idVendor")"
        usb_pid="$(tr "[:upper:]" "[:lower:]" < "${usb_device}/idProduct")"
        usb_serial="$(tr -d "[:space:]" < "${usb_device}/serial")"
        dev_port="$(tr -d "[:space:]" < "${netdev}/dev_port")"
        dev_id="$(tr -d "[:space:]" < "${netdev}/dev_id")"
        [[ "${driver}" == "gs_usb" ]] || { echo "error: CAN driver is ${driver}, expected gs_usb" >&2; exit 1; }
        [[ "${usb_vid}" == "1209" && "${usb_pid}" == "2323" ]] || {
          echo "error: CAN adapter is ${usb_vid}:${usb_pid}, expected 1209:2323" >&2
          exit 1
        }
        [[ "${usb_serial}" == "${HEX_ARM_CAN_SERIAL}" ]] || {
          echo "error: CAN adapter serial mismatch: ${usb_serial}" >&2
          exit 1
        }
        [[ "${dev_port}" -eq "${HEX_ARM_CAN_CHANNEL}" && "$((dev_id))" -eq "${HEX_ARM_CAN_CHANNEL}" ]] || {
          echo "error: CAN adapter channel mismatch: dev_port=${dev_port}, dev_id=${dev_id}" >&2
          exit 1
        }
        ip -details -statistics link show "${HEX_ARM_CAN_IFACE}"
        echo "SocketCAN ${HEX_ARM_CAN_IFACE}: strict 1M/4M link and adapter preflight OK"
      fi
      test -r /workspaces/hex_arm_ros2/install/setup.bash && echo "ROS workspace: built" || echo "ROS workspace: not built yet"
    '
    ;;
  config|logs|ps)
    "${compose[@]}" "${action}" "$@"
    ;;
  *)
    echo "usage: HEX_ARM_HEADLESS={0|1} HEX_ARM_GPU={auto|nvidia|none} HEX_ARM_CAN_IFACE=canN HEX_ARM_CAN_SERIAL=<32-hex> HEX_ARM_CAN_CHANNEL=N $0 {build|up|down|shell|doctor|real-launch|config|logs|ps} [arguments...]" >&2
    exit 2
    ;;
esac
