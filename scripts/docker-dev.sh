#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-up}"
if (( $# > 0 )); then
  shift
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker is not installed or is not on PATH" >&2
  exit 1
fi

compose_files=(-f "${workspace_dir}/compose.yaml")
platform="WSL2"

if ! uname -r | tr '[:upper:]' '[:lower:]' | grep -q microsoft; then
  platform="native Ubuntu"
  compose_files=(-f "${workspace_dir}/compose.ubuntu.yaml")

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
    echo "${platform} container is ready: ./scripts/docker-dev.sh shell"
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
    "${compose[@]}" exec -T ros2-jazzy-arm bash -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      display_number="${DISPLAY##*:}"
      display_number="${display_number%%.*}"
      test -S "/tmp/.X11-unix/X${display_number}"
      xdpyinfo >/dev/null
      echo "X11 authorization: OK"
      glxinfo -B | sed -n "/OpenGL vendor string/p; /OpenGL renderer string/p; /OpenGL core profile version string/p"
      test -r /workspaces/hex_arm_ros2/install/setup.bash && echo "ROS workspace: built" || echo "ROS workspace: not built yet"
    '
    ;;
  config|logs|ps)
    "${compose[@]}" "${action}" "$@"
    ;;
  *)
    echo "usage: $0 {build|up|down|shell|doctor|config|logs|ps} [arguments...]" >&2
    exit 2
    ;;
esac
