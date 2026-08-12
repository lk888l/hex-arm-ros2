#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-up}"
gpu_mode="${HEX_ARM_GPU:-auto}"
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

if ! uname -r | tr '[:upper:]' '[:lower:]' | grep -q microsoft; then
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
  if [[ ! -e /dev/dri ]]; then
    echo "error: /dev/dri is missing; the Ubuntu GUI container requires a DRM device" >&2
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
    "${compose[@]}" up -d
    exec "${compose[@]}" exec ros2-jazzy-arm bash "$@"
    ;;
  doctor)
    "${compose[@]}" up -d
    "${compose[@]}" exec -T -e HEX_ARM_EXPECT_NVIDIA="${gpu_enabled}" ros2-jazzy-arm bash -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
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
      test -r /workspaces/hex_arm_ros2/install/setup.bash && echo "ROS workspace: built" || echo "ROS workspace: not built yet"
    '
    ;;
  config|logs|ps)
    "${compose[@]}" "${action}" "$@"
    ;;
  *)
    echo "usage: HEX_ARM_GPU={auto|nvidia|none} $0 {build|up|down|shell|doctor|config|logs|ps} [arguments...]" >&2
    exit 2
    ;;
esac
