#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
helper="${workspace_dir}/scripts/docker-dev.sh"
temporary_dir="$(mktemp -d)"
socket_pid=""
x_socket=""

cleanup() {
  if [[ -n "${socket_pid}" ]]; then
    kill "${socket_pid}" 2>/dev/null || true
    wait "${socket_pid}" 2>/dev/null || true
  fi
  if [[ -n "${x_socket}" ]]; then
    rm -f -- "${x_socket}"
  fi
  rm -rf -- "${temporary_dir}"
}
trap cleanup EXIT

bash -n "${helper}"

xauthority="${temporary_dir}/xauthority"
: >"${xauthority}"

# Compose config is a parser-only operation. It does not contact the daemon or
# create a container, but it proves that this installed Compose version accepts
# !reset and that the two rendering paths resolve differently.
HEX_ARM_XAUTHORITY="${xauthority}" docker compose \
  -f "${workspace_dir}/compose.ubuntu.yaml" \
  config --format json >"${temporary_dir}/generic.json"
HEX_ARM_XAUTHORITY="${xauthority}" docker compose \
  -f "${workspace_dir}/compose.ubuntu.yaml" \
  -f "${workspace_dir}/compose.nvidia.yaml" \
  config --format json >"${temporary_dir}/nvidia.json"

python3 - "${temporary_dir}/generic.json" "${temporary_dir}/nvidia.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    generic = json.load(stream)["services"]["ros2-jazzy-arm"]
with open(sys.argv[2], encoding="utf-8") as stream:
    nvidia = json.load(stream)["services"]["ros2-jazzy-arm"]

generic_devices = generic.get("devices", [])
assert any(
    device.get("source") == "/dev/dri" and device.get("target") == "/dev/dri"
    for device in generic_devices
), generic_devices
assert "devices" not in nvidia or not nvidia["devices"], nvidia.get("devices")
assert nvidia.get("gpus") == [{"count": -1}], nvidia.get("gpus")
assert nvidia["environment"]["NVIDIA_VISIBLE_DEVICES"] == "all"
PY

# Exercise the helper's host preflight without touching the real Docker daemon
# or GPU. A private X11-shaped Unix socket makes this independent of a desktop
# session; fake docker/nvidia commands cover only argument selection.
display_number="$((200 + BASHPID % 700))"
x_socket="/tmp/.X11-unix/X${display_number}"
python3 - "${x_socket}" <<'PY' &
import os
import signal
import socket
import sys
import time

path = sys.argv[1]
try:
    os.unlink(path)
except FileNotFoundError:
    pass
server = socket.socket(socket.AF_UNIX)
server.bind(path)
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
while True:
    time.sleep(1)
PY
socket_pid="$!"
for _ in $(seq 1 100); do
  [[ -S "${x_socket}" ]] && break
  sleep 0.01
done
[[ -S "${x_socket}" ]]

mock_bin="${temporary_dir}/bin"
mkdir -p "${mock_bin}"
cat >"${mock_bin}/uname" <<'SH'
#!/usr/bin/env bash
printf '%s\n' '6.8.0-test-generic'
SH
cat >"${mock_bin}/nvidia-smi" <<'SH'
#!/usr/bin/env bash
exit 0
SH
cat >"${mock_bin}/docker" <<'SH'
#!/usr/bin/env bash
if [[ "${1:-}" == "info" ]]; then
  printf '%s\n' '{"nvidia":{"path":"nvidia-container-runtime"}}'
  exit 0
fi
if [[ "${1:-}" == "compose" ]]; then
  printf '%s\n' "$*"
  exit 0
fi
exit 99
SH
chmod +x "${mock_bin}/uname" "${mock_bin}/nvidia-smi" "${mock_bin}/docker"

headless_output="$(
  PATH="${mock_bin}:/usr/bin:/bin" DISPLAY="" XAUTHORITY="" HEX_ARM_HEADLESS=1 \
  bash "${helper}" config
)"
[[ "${headless_output}" == *"compose.headless.yaml"* ]]
[[ "${headless_output}" != *"compose.nvidia.yaml"* ]]
docker compose -f "${workspace_dir}/compose.headless.yaml" config --format json >"${temporary_dir}/headless.json"
python3 - "${temporary_dir}/headless.json" <<'PY'
import json, sys
service = json.load(open(sys.argv[1]))["services"]["ros2-jazzy-arm"]
assert service["network_mode"] == "host"
assert not service.get("devices") and not service.get("gpus")
assert "DISPLAY" not in service["environment"] and "XAUTHORITY" not in service["environment"]
assert len(service["volumes"]) == 1
PY

nvidia_output="$(
  PATH="${mock_bin}:/usr/bin:/bin" \
  DISPLAY=":${display_number}" \
  XAUTHORITY="${xauthority}" \
  HEX_ARM_GPU=nvidia \
  bash "${helper}" config
)"
[[ "${nvidia_output}" == *"compose.nvidia.yaml"* ]]

if [[ ! -e /dev/dri ]]; then
  if PATH="${mock_bin}:/usr/bin:/bin" \
    DISPLAY=":${display_number}" \
    XAUTHORITY="${xauthority}" \
    HEX_ARM_GPU=none \
    bash "${helper}" config >"${temporary_dir}/none.stdout" 2>"${temporary_dir}/none.stderr"; then
    echo "error: HEX_ARM_GPU=none unexpectedly passed without /dev/dri" >&2
    exit 1
  fi
  grep -Fq "non-NVIDIA Ubuntu GUI container requires a DRM device" \
    "${temporary_dir}/none.stderr"
fi

echo "Docker GPU Compose/helper offline checks: PASS"
