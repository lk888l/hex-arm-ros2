#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
supervisor="${workspace_dir}/scripts/supervised-real-launch.sh"
docker_helper="${workspace_dir}/scripts/docker-dev.sh"
state_root="/tmp/hex-arm-real-launch-supervisors"
test_tmp="$(mktemp -d /tmp/hex-arm-supervisor-test.XXXXXX)"
target_pgid=""
unrelated_pgid=""
fixture_tokens=()

cleanup() {
  local token
  if [[ -n "${target_pgid}" ]] && kill -0 -- "-${target_pgid}" 2>/dev/null; then
    kill -TERM -- "-${target_pgid}" 2>/dev/null || true
    wait "${target_pgid}" 2>/dev/null || true
  fi
  if [[ -n "${unrelated_pgid}" ]] && kill -0 -- "-${unrelated_pgid}" 2>/dev/null; then
    kill -TERM -- "-${unrelated_pgid}" 2>/dev/null || true
    wait "${unrelated_pgid}" 2>/dev/null || true
  fi
  for token in "${fixture_tokens[@]}"; do
    rm -f -- \
      "${state_root}/${token}/launch.pgid" \
      "${state_root}/${token}/requested.signal"
    rmdir -- "${state_root}/${token}" 2>/dev/null || true
  done
  if [[ "${test_tmp}" == /tmp/hex-arm-supervisor-test.* ]]; then
    rm -rf -- "${test_tmp}"
  fi
}
trap cleanup EXIT

bash -n "${supervisor}"
bash -n "${docker_helper}"

expect_status() {
  local expected="$1"
  shift
  local actual
  set +e
  "$@" >/dev/null 2>&1
  actual="$?"
  set -e
  if [[ "${actual}" != "${expected}" ]]; then
    echo "error: expected status ${expected}, got ${actual}: $*" >&2
    exit 1
  fi
}

# These stop during argument validation. They do not invoke Docker, ROS, or
# inspect/open a CAN interface.
expect_status 2 bash "${supervisor}"
expect_status 2 bash "${supervisor}" unsupported /tmp/profile.yaml
expect_status 2 env \
  HEX_ARM_CAN_IFACE=can2 \
  HEX_ARM_CAN_CHANNEL=2 \
  HEX_ARM_CAN_SERIAL=C9E29601798421B29AC2D419C12D9502 \
  HEX_ARM_REAL_LAUNCH_TOKEN=0123456789abcdef0123456789abcdef \
  bash "${supervisor}" bringup /tmp/profile.yaml --debug
expect_status 2 env \
  HEX_ARM_CAN_IFACE=can2 \
  HEX_ARM_CAN_CHANNEL=2 \
  HEX_ARM_CAN_SERIAL=C9E29601798421B29AC2D419C12D9502 \
  HEX_ARM_REAL_LAUNCH_TOKEN=0123456789abcdef0123456789abcdef \
  bash "${supervisor}" bringup /tmp/profile.yaml hardware_profile:=duplicate.yaml
# Exercise and compile the embedded Python validator against the intentionally
# incomplete example. Its serial mismatch stops before ROS is sourced or run.
expect_status 1 env \
  HEX_ARM_CAN_IFACE=can0 \
  HEX_ARM_CAN_CHANNEL=0 \
  HEX_ARM_CAN_SERIAL=C9E29601798421B29AC2D419C12D9502 \
  HEX_ARM_REAL_LAUNCH_TOKEN=0123456789abcdef0123456789abcdef \
  bash "${supervisor}" bringup \
  "${workspace_dir}/config/hardware/firefly_y6.example.yaml"

expect_status 2 bash "${supervisor}" --forward-signal '../can1' TERM
expect_status 2 bash "${supervisor}" --forward-signal \
  0123456789abcdef0123456789abcdef HUP

# Exercise the token-scoped relay with two independent process groups. The
# target exits on TERM; the unrelated group must remain live.
install -d -m 0700 "${state_root}"
relay_token="$(tr -d '-' </proc/sys/kernel/random/uuid)"
stale_token="$(tr -d '-' </proc/sys/kernel/random/uuid)"
early_token="$(tr -d '-' </proc/sys/kernel/random/uuid)"
fixture_tokens+=("${relay_token}" "${stale_token}" "${early_token}")
install -d -m 0700 "${state_root}/${relay_token}"
install -d -m 0700 "${state_root}/${stale_token}"
install -d -m 0700 "${state_root}/${early_token}"

supervisor_ticks="$(python3 - "$$" <<'PY'
from pathlib import Path
import sys


text = Path(f"/proc/{int(sys.argv[1])}/stat").read_text(encoding="ascii")
fields = text[text.rfind(")") + 2 :].split()
print(fields[19])
PY
)"

setsid bash -c 'trap "exit 0" TERM; while :; do sleep 0.05; done' \
  >/dev/null 2>&1 &
target_pgid="$!"
setsid bash -c 'trap "exit 0" TERM; while :; do sleep 0.05; done' \
  >/dev/null 2>&1 &
unrelated_pgid="$!"
target_ticks="$(python3 - "${target_pgid}" <<'PY'
from pathlib import Path
import sys


text = Path(f"/proc/{int(sys.argv[1])}/stat").read_text(encoding="ascii")
fields = text[text.rfind(")") + 2 :].split()
print(fields[19])
PY
)"
unrelated_ticks="$(python3 - "${unrelated_pgid}" <<'PY'
from pathlib import Path
import sys


text = Path(f"/proc/{int(sys.argv[1])}/stat").read_text(encoding="ascii")
fields = text[text.rfind(")") + 2 :].split()
print(fields[19])
PY
)"
printf 'supervisor_pid=%s\nsupervisor_start_ticks=%s\nlaunch_pgid=%s\nlaunch_start_ticks=%s\n' \
  "$$" "${supervisor_ticks}" "${target_pgid}" "${target_ticks}" \
  >"${state_root}/${relay_token}/launch.pgid"
chmod 0600 "${state_root}/${relay_token}/launch.pgid"
bash "${supervisor}" --forward-signal "${relay_token}" TERM >/dev/null
wait "${target_pgid}"
target_pgid=""
kill -0 -- "-${unrelated_pgid}"

# A stale supervisor start time must never authorize a signal to an otherwise
# live group. The unrelated group remains the proof target.
printf 'supervisor_pid=%s\nsupervisor_start_ticks=%s\nlaunch_pgid=%s\nlaunch_start_ticks=%s\n' \
  "$$" "$((supervisor_ticks + 1))" "${unrelated_pgid}" "${unrelated_ticks}" \
  >"${state_root}/${stale_token}/launch.pgid"
chmod 0600 "${state_root}/${stale_token}/launch.pgid"
expect_status 1 bash "${supervisor}" --forward-signal "${stale_token}" TERM
kill -0 -- "-${unrelated_pgid}"
printf 'supervisor_pid=%s\nsupervisor_start_ticks=%s\nlaunch_pgid=%s\nlaunch_start_ticks=%s\n' \
  "$$" "${supervisor_ticks}" "${unrelated_pgid}" "$((unrelated_ticks + 1))" \
  >"${state_root}/${stale_token}/launch.pgid"
chmod 0600 "${state_root}/${stale_token}/launch.pgid"
expect_status 1 bash "${supervisor}" --forward-signal "${stale_token}" INT
kill -0 -- "-${unrelated_pgid}"

# An early signal has no process group to touch. It is recorded under only the
# exact token so the primary supervisor can exit before starting ROS.
bash "${supervisor}" --forward-signal "${early_token}" INT >/dev/null
grep -Fxq INT "${state_root}/${early_token}/requested.signal"

grep -Fq 'setsid python3' "${supervisor}"
grep -Fq 'graceful-real-launch.py' "${supervisor}"
grep -Fq 'set +u' "${supervisor}"
grep -Fq 'source /opt/ros/jazzy/setup.bash' "${supervisor}"
grep -Fq 'set -u' "${supervisor}"
grep -Fq 'kill -s "${signal_name}" -- "-${launch_pid}"' "${supervisor}"
grep -Fq 'kill -s "${signal_name}" -- "-${launch_pgid}"' "${supervisor}"
grep -Fq "trap 'forward_signal TERM' HUP" "${supervisor}"
grep -Fq 'process has finished cleanly' "${supervisor}"
grep -Fq 'unexpected_process_deaths' "${supervisor}"
grep -Fq 'a supervised ROS child failed; final drive disable was verified' "${supervisor}"
grep -Fq 'exec 9>"${lock_path}"' "${supervisor}"
grep -Fq '/workspaces/hex_arm_ros2/scripts/supervised-real-launch.sh' "${docker_helper}"
grep -Fq "trap 'record_real_launch_signal INT' INT" "${docker_helper}"
grep -Fq 'setsid -w docker exec ros2-jazzy-arm' "${docker_helper}"
grep -Fq -- '--forward-signal "${launch_token}" "${requested_signal}"' "${docker_helper}"
grep -Fq 'wait "${real_exec_pid}"' "${docker_helper}"

if grep -Eq '(^|[[:space:]])(pkill|killall)([[:space:]]|$)' \
  "${supervisor}" "${docker_helper}"; then
  echo "error: real-launch helpers must never use process-name-wide termination" >&2
  exit 1
fi

# Mock Docker proves the host trap launches the exact second exec, then waits
# for the original Compose exec to emit its final verification and exit. This
# test has no Docker daemon, ROS process, or CAN access.
mock_bin="${test_tmp}/bin"
mock_log="${test_tmp}/docker.log"
mock_token="${test_tmp}/token"
mock_stop="${test_tmp}/stop"
helper_log="${test_tmp}/helper.log"
mkdir "${mock_bin}"
cat >"${mock_bin}/uname" <<'EOF'
#!/usr/bin/env bash
if [[ "${1:-}" == "-r" ]]; then
  echo 6.6.0-microsoft-standard-WSL2
else
  /usr/bin/uname "$@"
fi
EOF
cat >"${mock_bin}/docker" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >>"${MOCK_DOCKER_LOG}"
if [[ "${1:-}" == "inspect" ]]; then
  echo true
  exit 0
fi
if [[ "${1:-}" == "exec" ]]; then
  [[ "$2" == "ros2-jazzy-arm" ]]
  [[ "$3" == "/workspaces/hex_arm_ros2/scripts/supervised-real-launch.sh" ]]
  [[ "$4" == "--forward-signal" ]]
  for _ in $(seq 1 100); do
    [[ -r "${MOCK_TOKEN_FILE}" ]] && break
    sleep 0.01
  done
  [[ "$5" == "$(<"${MOCK_TOKEN_FILE}")" ]]
  if [[ "${MOCK_WAIT_FOR_SECOND:-0}" == "1" && "$6" == "INT" ]]; then
    : >"${MOCK_STOP_FILE}.first"
    exit 0
  fi
  if [[ "${MOCK_WAIT_FOR_SECOND:-0}" == "1" ]]; then
    [[ "$6" == "TERM" ]]
  else
    [[ "$6" == "INT" ]]
  fi
  : >"${MOCK_STOP_FILE}"
  exit 0
fi
if [[ "${1:-}" == "compose" ]]; then
  token=""
  previous=""
  for argument in "$@"; do
    if [[ "${previous}" == "-e" && "${argument}" == HEX_ARM_REAL_LAUNCH_TOKEN=* ]]; then
      token="${argument#*=}"
    fi
    previous="${argument}"
  done
  [[ "${token}" =~ ^[0-9a-f]{32}$ ]]
  printf '%s\n' "${token}" >"${MOCK_TOKEN_FILE}"
  while [[ ! -e "${MOCK_STOP_FILE}" ]]; do
    sleep 0.01
  done
  echo "supervisor: VERIFIED clean controller exit after the orderly disable/heartbeat-disarm path"
  exit 0
fi
exit 1
EOF
chmod +x "${mock_bin}/uname" "${mock_bin}/docker"

expect_status 2 env \
  PATH="${mock_bin}:${PATH}" \
  HEX_ARM_CAN_IFACE=can2 \
  HEX_ARM_CAN_CHANNEL=2 \
  HEX_ARM_CAN_SERIAL=C9E29601798421B29AC2D419C12D9502 \
  bash "${docker_helper}" real-launch --forward-signal \
  0123456789abcdef0123456789abcdef

PATH="${mock_bin}:${PATH}" \
MOCK_DOCKER_LOG="${mock_log}" \
MOCK_TOKEN_FILE="${mock_token}" \
MOCK_STOP_FILE="${mock_stop}" \
HEX_ARM_CAN_IFACE=can2 \
HEX_ARM_CAN_CHANNEL=2 \
HEX_ARM_CAN_SERIAL=C9E29601798421B29AC2D419C12D9502 \
python3 -c '
import os
import signal
import sys

signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp("bash", ["bash", *sys.argv[1:]])
' "${docker_helper}" real-launch bringup /workspaces/profile.yaml \
  >"${helper_log}" 2>&1 &
helper_pid="$!"
for _ in $(seq 1 200); do
  [[ -r "${mock_token}" ]] && break
  sleep 0.01
done
test -r "${mock_token}"
kill -INT "${helper_pid}"
wait "${helper_pid}"
grep -Fq 'exec ros2-jazzy-arm /workspaces/hex_arm_ros2/scripts/supervised-real-launch.sh --forward-signal' "${mock_log}"
grep -Fq 'host supervisor: relaying INT' "${helper_log}"
grep -Fq 'VERIFIED clean controller exit' "${helper_log}"
if grep -Fq can1 "${mock_log}"; then
  echo "error: can2 host relay test unexpectedly referenced can1" >&2
  exit 1
fi

# While a normal soft stop is pending, a repeated host Ctrl-C must not be swallowed.
PATH="${mock_bin}:${PATH}" \
MOCK_DOCKER_LOG="${mock_log}.second" \
MOCK_TOKEN_FILE="${mock_token}.second" \
MOCK_STOP_FILE="${mock_stop}.second" \
MOCK_WAIT_FOR_SECOND=1 \
HEX_ARM_CAN_IFACE=can2 \
HEX_ARM_CAN_CHANNEL=2 \
HEX_ARM_CAN_SERIAL=C9E29601798421B29AC2D419C12D9502 \
python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp("bash", ["bash", *sys.argv[1:]])
' "${docker_helper}" real-launch moveit /workspaces/profile.yaml \
  >"${helper_log}.second" 2>&1 &
helper_pid="$!"
for _ in $(seq 1 200); do
  [[ -r "${mock_token}.second" ]] && break
  sleep 0.01
done
test -r "${mock_token}.second"
kill -INT "${helper_pid}"
for _ in $(seq 1 200); do
  [[ -e "${mock_stop}.second.first" ]] && break
  sleep 0.01
done
test -e "${mock_stop}.second.first"
kill -INT "${helper_pid}"
wait "${helper_pid}"
grep -Fq 'host supervisor: relaying TERM' "${helper_log}.second"
grep -Fq 'VERIFIED clean controller exit' "${helper_log}.second"

echo "supervised real-launch shell checks: PASS"
