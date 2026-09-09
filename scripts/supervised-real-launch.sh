#!/usr/bin/env bash
set -Eeuo pipefail

# Container-side supervisor for a real-arm ROS launch. This script is invoked
# by docker-dev.sh; running ros2 launch through a plain `docker exec bash -lc`
# does not give the host a stable process-group target for orderly shutdown.

usage() {
  cat >&2 <<'EOF'
usage: supervised-real-launch.sh {bringup|moveit|startup} /absolute/container/profile.yaml [launch_arg:=value ...]
       supervised-real-launch.sh --forward-signal <32-lowercase-hex-token> {INT|TERM}
EOF
}

state_root="/tmp/hex-arm-real-launch-supervisors"
token_pattern='^[[:xdigit:]]{32}$'

process_start_ticks() {
  local process_id="$1"
  python3 - "${process_id}" <<'PY'
from pathlib import Path
import sys


text = Path(f"/proc/{int(sys.argv[1])}/stat").read_text(encoding="ascii")
right_paren = text.rfind(")")
if right_paren < 0:
    raise SystemExit(1)
fields_after_comm = text[right_paren + 2 :].split()
# starttime is field 22; the first item after comm is field 3.
if len(fields_after_comm) < 20:
    raise SystemExit(1)
print(fields_after_comm[19])
PY
}

validate_state_directory() {
  local directory="$1"
  local owner mode

  [[ -d "${directory}" && ! -L "${directory}" ]] || return 1
  owner="$(stat -c '%u' -- "${directory}")" || return 1
  mode="$(stat -c '%a' -- "${directory}")" || return 1
  [[ "${owner}" == "$(id -u)" && "${mode}" == "700" ]]
}

validate_state_file() {
  local path="$1"
  local owner mode

  [[ -f "${path}" && ! -L "${path}" ]] || return 1
  owner="$(stat -c '%u' -- "${path}")" || return 1
  mode="$(stat -c '%a' -- "${path}")" || return 1
  [[ "${owner}" == "$(id -u)" && "${mode}" == "600" ]]
}

forward_requested_group() {
  local token="$1"
  local signal_name="$2"
  local state_dir pgid_path signal_path signal_tmp
  local supervisor_pid supervisor_start_ticks launch_pgid launch_start_ticks
  local current_start_ticks current_launch_start_ticks
  local deadline

  if [[ ! "${token}" =~ ${token_pattern} || "${token}" != "${token,,}" ]]; then
    echo "error: real-launch signal token must be exactly 32 lowercase hexadecimal characters" >&2
    return 2
  fi
  case "${signal_name}" in
    INT|TERM) ;;
    *)
      echo "error: real-launch forwarded signal must be INT or TERM" >&2
      return 2
      ;;
  esac
  command -v python3 >/dev/null || {
    echo "error: python3 is required for supervised signal validation" >&2
    return 1
  }

  state_dir="${state_root}/${token}"
  pgid_path="${state_dir}/launch.pgid"
  signal_path="${state_dir}/requested.signal"

  # The host can receive Ctrl-C just after starting the first Docker exec. Give
  # that invocation a short window to create its token directory. No wildcard,
  # process-name lookup, or interface-wide operation is used here.
  deadline=$((SECONDS + 10))
  while [[ ! -e "${state_dir}" ]]; do
    if (( SECONDS >= deadline )); then
      echo "error: no live supervisor state appeared for token ${token}" >&2
      return 1
    fi
    sleep 0.05
  done
  if ! validate_state_directory "${state_dir}"; then
    echo "error: unsafe or stale supervisor state directory for token ${token}" >&2
    return 1
  fi

  # Record the request first. If validation or ROS setup has not yet produced a
  # process group, the primary supervisor sees this exact token-scoped marker
  # and exits without ever starting ROS. If a group exists, validate every
  # numeric field before sending only to its negative PGID.
  signal_tmp="${state_dir}/requested.signal.$$"
  (umask 077; printf '%s\n' "${signal_name}" >"${signal_tmp}")
  mv -T -- "${signal_tmp}" "${signal_path}"
  if [[ ! -e "${pgid_path}" ]]; then
    echo "supervisor signal relay: recorded ${signal_name} before a launch process group existed"
    return 0
  fi
  if ! validate_state_file "${pgid_path}"; then
    echo "error: unsafe launch PGID state for token ${token}" >&2
    return 1
  fi

  supervisor_pid=""
  supervisor_start_ticks=""
  launch_pgid=""
  launch_start_ticks=""
  while IFS='=' read -r key value; do
    case "${key}" in
      supervisor_pid) supervisor_pid="${value}" ;;
      supervisor_start_ticks) supervisor_start_ticks="${value}" ;;
      launch_pgid) launch_pgid="${value}" ;;
      launch_start_ticks) launch_start_ticks="${value}" ;;
      *)
        echo "error: malformed launch PGID state for token ${token}" >&2
        return 1
        ;;
    esac
  done <"${pgid_path}"
  if [[ ! "${supervisor_pid}" =~ ^[1-9][0-9]*$ ||
        ! "${supervisor_start_ticks}" =~ ^[1-9][0-9]*$ ||
        ! "${launch_pgid}" =~ ^[1-9][0-9]*$ ||
        ! "${launch_start_ticks}" =~ ^[1-9][0-9]*$ ]] ||
     (( supervisor_pid <= 1 || launch_pgid <= 1 )); then
    echo "error: invalid numeric launch PGID state for token ${token}" >&2
    return 1
  fi
  current_start_ticks="$(process_start_ticks "${supervisor_pid}" 2>/dev/null || true)"
  if [[ "${current_start_ticks}" != "${supervisor_start_ticks}" ]]; then
    echo "error: stale supervisor PID state for token ${token}; refusing to signal" >&2
    return 1
  fi
  current_launch_start_ticks="$(process_start_ticks "${launch_pgid}" 2>/dev/null || true)"
  if [[ "${current_launch_start_ticks}" != "${launch_start_ticks}" ]]; then
    echo "error: stale launch process-group leader state for token ${token}; refusing to signal" >&2
    return 1
  fi
  if ! kill -0 -- "-${launch_pgid}" 2>/dev/null; then
    echo "error: launch process group ${launch_pgid} is no longer live" >&2
    return 1
  fi

  echo "supervisor signal relay: forwarding ${signal_name} only to launch process group ${launch_pgid}"
  kill -s "${signal_name}" -- "-${launch_pgid}"
}

if [[ "${1:-}" == "--forward-signal" ]]; then
  if (( $# != 3 )); then
    usage
    exit 2
  fi
  forward_requested_group "$2" "$3"
  exit "$?"
fi

if (( $# < 2 )); then
  usage
  exit 2
fi

target="$1"
profile_path="$2"
shift 2

case "${target}" in
  bringup)
    launch_package="hex_arm_bringup"
    launch_file="real.launch.py"
    ;;
  moveit)
    launch_package="hex_arm_moveit_config"
    launch_file="moveit_real.launch.py"
    ;;
  startup)
    launch_package="hex_arm_bringup"
    launch_file="startup.launch.py"
    ;;
  *)
    echo "error: real-launch target must be one of: bringup, moveit, startup" >&2
    usage
    exit 2
    ;;
esac

can_interface="${HEX_ARM_CAN_IFACE:-}"
can_channel="${HEX_ARM_CAN_CHANNEL:-}"
can_serial="${HEX_ARM_CAN_SERIAL:-}"
launch_token="${HEX_ARM_REAL_LAUNCH_TOKEN:-}"

if [[ ! "${can_interface}" =~ ^[[:alnum:]_.-]{1,15}$ ]]; then
  echo "error: supervised real launch requires a valid, explicit HEX_ARM_CAN_IFACE" >&2
  exit 2
fi
if [[ ! "${can_channel}" =~ ^[0-3]$ ]]; then
  echo "error: supervised real launch requires HEX_ARM_CAN_CHANNEL=0..3" >&2
  exit 2
fi
if [[ ! "${can_serial}" =~ ^[[:xdigit:]]{32}$ ]]; then
  echo "error: supervised real launch requires the exact 32-digit HEX_ARM_CAN_SERIAL" >&2
  exit 2
fi
if [[ ! "${launch_token}" =~ ${token_pattern} || "${launch_token}" != "${launch_token,,}" ]]; then
  echo "error: supervised real launch requires the host-generated 32-character HEX_ARM_REAL_LAUNCH_TOKEN" >&2
  exit 2
fi
if [[ "${profile_path}" != /* || "${profile_path}" == *$'\n'* ]]; then
  echo "error: hardware profile must be an absolute path inside the container" >&2
  exit 2
fi

for argument in "$@"; do
  if [[ ! "${argument}" =~ ^[[:alpha:]_][[:alnum:]_]*:=.+$ ]]; then
    echo "error: unsupported real launch argument: ${argument}" >&2
    echo "hint: only explicit ROS launch assignments such as use_rviz:=false are accepted" >&2
    exit 2
  fi
  if [[ "${argument}" == hardware_profile:=* ]]; then
    echo "error: pass the hardware profile once as the second real-launch argument" >&2
    exit 2
  fi
done

command -v python3 >/dev/null || {
  echo "error: python3 is required for supervised profile validation" >&2
  exit 1
}
command -v flock >/dev/null || {
  echo "error: flock is required for per-interface real-launch exclusion" >&2
  exit 1
}
command -v setsid >/dev/null || {
  echo "error: setsid is required for scoped signal forwarding" >&2
  exit 1
}

if [[ -e "${state_root}" ]]; then
  if ! validate_state_directory "${state_root}"; then
    echo "error: unsafe supervisor state root ${state_root}" >&2
    exit 1
  fi
else
  if ! (umask 077; mkdir -- "${state_root}") 2>/dev/null &&
     ! validate_state_directory "${state_root}"; then
    echo "error: could not create a private supervisor state root ${state_root}" >&2
    exit 1
  fi
fi
state_dir="${state_root}/${launch_token}"
if ! (umask 077; mkdir -- "${state_dir}"); then
  echo "error: supervisor token state already exists; refusing stale/colliding token ${launch_token}" >&2
  exit 1
fi
state_owned=1
state_pgid_path="${state_dir}/launch.pgid"
state_signal_path="${state_dir}/requested.signal"

cleanup_state() {
  if [[ "${state_owned:-0}" == "1" ]]; then
    rm -f -- "${state_pgid_path}" "${state_signal_path}"
    rmdir -- "${state_dir}" 2>/dev/null || true
    state_owned=0
  fi
}
trap cleanup_state EXIT

read_external_signal_request() {
  local signal_name

  [[ -e "${state_signal_path}" ]] || return 1
  if ! validate_state_file "${state_signal_path}"; then
    echo "error: unsafe external signal request for token ${launch_token}" >&2
    exit 1
  fi
  IFS= read -r signal_name <"${state_signal_path}" || true
  case "${signal_name}" in
    INT|TERM) printf '%s\n' "${signal_name}" ;;
    *)
      echo "error: malformed external signal request for token ${launch_token}" >&2
      exit 1
      ;;
  esac
}

# The lock is keyed only by the selected interface. A supervised can1 launch
# and a supervised can2 launch may coexist, but two owners of can2 may not.
lock_path="/tmp/hex-arm-real-launch.${can_interface}.lock"
exec 9>"${lock_path}"
if ! flock -n 9; then
  echo "error: another supervised real launch already owns ${can_interface}" >&2
  exit 1
fi

# Validate the exact bus binding before sourcing or starting ROS. Also refuse
# an already-running controller that names a readable profile for this same
# interface. Nothing here opens or changes the CAN interface.
python3 - "${profile_path}" "${can_interface}" "${can_channel}" "${can_serial}" <<'PY'
import os
from pathlib import Path
import sys

import yaml


def fail(message: str) -> None:
    raise SystemExit(f"error: supervised real launch: {message}")


profile_path = Path(sys.argv[1])
expected_interface = sys.argv[2]
expected_channel = int(sys.argv[3])
expected_serial = sys.argv[4].lower()

if not profile_path.is_file():
    fail(f"profile does not exist: {profile_path}")
try:
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
except Exception as error:
    fail(f"cannot parse profile {profile_path}: {error}")
if not isinstance(profile, dict):
    fail("profile root is not a mapping")

bus = profile.get("bus")
if not isinstance(bus, dict):
    fail("profile has no bus mapping")
if bus.get("transport") != "socket_can":
    fail("only a socket_can profile is accepted by this helper")
if bus.get("interface") != expected_interface:
    fail(
        f"profile bus.interface={bus.get('interface')!r} does not match "
        f"HEX_ARM_CAN_IFACE={expected_interface!r}"
    )
if bus.get("channel") != expected_channel:
    fail(
        f"profile bus.channel={bus.get('channel')!r} does not match "
        f"HEX_ARM_CAN_CHANNEL={expected_channel}"
    )

expected_link = bus.get("expected_link")
adapter = expected_link.get("adapter") if isinstance(expected_link, dict) else None
if not isinstance(adapter, dict):
    fail("profile has no bus.expected_link.adapter mapping")
if adapter.get("channel") != expected_channel:
    fail("profile expected adapter channel does not match HEX_ARM_CAN_CHANNEL")
profile_serial = str(adapter.get("serial", "")).lower()
if profile_serial != expected_serial:
    fail("profile expected adapter serial does not match HEX_ARM_CAN_SERIAL")


def process_profile(arguments: list[str]) -> Path | None:
    for index, argument in enumerate(arguments):
        if argument == "--profile" and index + 1 < len(arguments):
            return Path(arguments[index + 1])
        if argument.startswith("--profile="):
            return Path(argument.split("=", 1)[1])
    return None


conflicts: list[tuple[int, Path]] = []
for proc_dir in Path("/proc").glob("[0-9]*"):
    try:
        raw_arguments = (proc_dir / "cmdline").read_bytes().split(b"\0")
        arguments = [item.decode("utf-8", "surrogateescape") for item in raw_arguments if item]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if not arguments or os.path.basename(arguments[0]) != "hex_arm_controller":
        continue
    other_profile_path = process_profile(arguments)
    if other_profile_path is None:
        continue
    try:
        other_profile = yaml.safe_load(other_profile_path.read_text(encoding="utf-8"))
        other_interface = other_profile.get("bus", {}).get("interface")
    except Exception:
        continue
    if other_interface == expected_interface:
        conflicts.append((int(proc_dir.name), other_profile_path))

if conflicts:
    rendered = ", ".join(f"pid {pid} ({path})" for pid, path in conflicts)
    fail(f"{expected_interface} already has a hex_arm_controller owner: {rendered}")

print(
    f"profile bus binding: {expected_interface}, channel {expected_channel}, "
    f"adapter serial {expected_serial}: OK"
)
PY

test -r /opt/ros/jazzy/setup.bash || {
  echo "error: /opt/ros/jazzy/setup.bash is missing" >&2
  exit 1
}
test -r /workspaces/hex_arm_ros2/install/setup.bash || {
  echo "error: ROS workspace is not built; build it before a real launch" >&2
  exit 1
}
# ROS 2 Jazzy's generated setup files probe optional shell variables (for
# example AMENT_TRACE_SETUP_FILES) without nounset-safe expansion. Keep all
# other strict-shell guarantees, but suspend only `-u` while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
set -u

# A host signal can arrive while profile validation or ROS setup is running.
# In that case there is no controller to stop: honour the token-scoped request
# by exiting before creating any launch process group.
if early_signal="$(read_external_signal_request)"; then
  echo "supervisor: ${early_signal} was requested before ROS launch; no controller process was started"
  exit 0
fi

run_dir="$(mktemp -d "/tmp/hex-arm-real-launch.${can_interface}.XXXXXX")"
output_fifo="${run_dir}/output.fifo"
log_path="${run_dir}/launch.log"
mkfifo "${output_fifo}"

launch_pid=""
tee_pid=""
requested_signal=""

cleanup() {
  if [[ -p "${output_fifo}" ]]; then
    rm -f -- "${output_fifo}"
  fi
  cleanup_state
}

forward_signal() {
  local signal_name="$1"
  requested_signal="${signal_name}"
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    echo "supervisor: forwarding ${signal_name} only to launch process group ${launch_pid}; waiting for confirmed controller shutdown" >&2
    kill -s "${signal_name}" -- "-${launch_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'forward_signal INT' INT
trap 'forward_signal TERM' TERM
# The Rust controller handles INT and TERM deliberately, but not HUP. Convert
# a lost terminal/exec attachment into TERM so it still takes the orderly path.
trap 'forward_signal TERM' HUP

# A FIFO lets tee retain an audit log without placing ros2 launch in a shell
# pipeline. launch_pid is therefore also the exact session/process-group ID.
tee --output-error=warn-nopipe "${log_path}" <"${output_fifo}" &
tee_pid="$!"
setsid ros2 launch "${launch_package}" "${launch_file}" \
  "hardware_profile:=${profile_path}" "$@" >"${output_fifo}" 2>&1 &
launch_pid="$!"
supervisor_start_ticks="$(process_start_ticks "$$")"
launch_start_ticks="$(process_start_ticks "${launch_pid}")"
state_pgid_tmp="${state_dir}/launch.pgid.$$"
(
  umask 077
  printf 'supervisor_pid=%s\nsupervisor_start_ticks=%s\nlaunch_pgid=%s\nlaunch_start_ticks=%s\n' \
    "$$" "${supervisor_start_ticks}" "${launch_pid}" "${launch_start_ticks}" \
    >"${state_pgid_tmp}"
)
mv -T -- "${state_pgid_tmp}" "${state_pgid_path}"
echo "supervisor: ${target} launch owns ${can_interface} in process group ${launch_pid}"
echo "supervisor: audit log ${log_path}"

# Close the race where the relay records a request after the pre-launch check
# but just before the atomically published PGID file.
if [[ -n "${requested_signal}" ]]; then
  forward_signal "${requested_signal}"
elif external_signal="$(read_external_signal_request)"; then
  forward_signal "${external_signal}"
fi

launch_status=0
while true; do
  set +e
  wait "${launch_pid}"
  wait_status="$?"
  set -e
  if kill -0 "${launch_pid}" 2>/dev/null; then
    # wait was interrupted by one of the trapped signals; keep supervising.
    continue
  fi
  launch_status="${wait_status}"
  break
done

# If ros2 launch itself exited before all of its descendants, request the same
# orderly shutdown on this run's process group and keep waiting for log EOF.
if kill -0 -- "-${launch_pid}" 2>/dev/null; then
  echo "supervisor: ros2 launch exited while its scoped process group remained; requesting SIGINT cleanup" >&2
  kill -s INT -- "-${launch_pid}" 2>/dev/null || true
fi

while true; do
  set +e
  wait "${tee_pid}"
  tee_status="$?"
  set -e
  if kill -0 "${tee_pid}" 2>/dev/null; then
    continue
  fi
  break
done
if (( tee_status != 0 )); then
  echo "error: launch audit logger exited with status ${tee_status}" >&2
fi

verification_status=0
python3 - "${log_path}" <<'PY' || verification_status="$?"
from pathlib import Path
import re
import sys


text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
text = re.sub(r"\x1b\[[0-9;?]*[ -/]*[@-~]", "", text)
controller_started = bool(
    re.search(r"\[hex_arm_controller-[0-9]+\].*process started with pid", text)
)
controller_ready = "Firefly Y6 controller ready and DISABLED" in text
shutdown_started = (
    "controller stopping; disabling drives and disarming heartbeat consumers" in text
)
controller_clean = bool(
    re.search(
        r"\[hex_arm_controller-[0-9]+\].*process has finished cleanly", text
    )
)
shutdown_failed = "controller shutdown failed:" in text
unexpected_process_deaths = []
for line in text.splitlines():
    match = re.search(r"process has died .*exit code (-?[0-9]+)", line)
    if match and int(match.group(1)) not in (-2, -15, 130):
        unexpected_process_deaths.append(line.strip())

# A verified controller shutdown proves the drives were made safe, but it must
# not hide an independent ROS/MoveIt crash.  In particular, MoveIt 2.12.4 can
# otherwise report a CallbackGroup teardown SIGSEGV while this supervisor exits
# successfully because the Rust controller completed its own shutdown first.
if "Segmentation fault" in text or unexpected_process_deaths:
    details = "; ".join(unexpected_process_deaths[:3])
    raise SystemExit(
        "error: a supervised ROS child crashed during shutdown"
        + (f": {details}" if details else " (segmentation fault in launch log)")
    )

if controller_ready:
    if not shutdown_started:
        raise SystemExit(
            "error: controller reached ready state but orderly shutdown was not observed"
        )
    if shutdown_failed or not controller_clean:
        raise SystemExit(
            "error: controller disable/heartbeat-disarm completion was not confirmed; "
            "inspect the retained launch log"
        )
    print(
        "supervisor: VERIFIED clean controller exit after the orderly "
        "disable/heartbeat-disarm path"
    )
elif controller_started:
    raise SystemExit(
        "error: controller started but never reached the ready-and-DISABLED marker; "
        "full shutdown confirmation is unavailable"
    )
else:
    raise SystemExit(
        "error: controller never started; no real-controller shutdown claim can be made"
    )
PY

if (( tee_status != 0 || verification_status != 0 )); then
  echo "error: supervised real launch did not produce a verified clean shutdown" >&2
  echo "error: retained audit log: ${log_path}" >&2
  exit 1
fi

if [[ -z "${requested_signal}" ]] &&
   external_signal="$(read_external_signal_request)"; then
  requested_signal="${external_signal}"
fi
if [[ -n "${requested_signal}" ]]; then
  # ros2 launch may encode a handled Ctrl-C as 0, 130, or -SIGINT depending on
  # the launch version. The verified controller exit above is authoritative.
  exit 0
fi
exit "${launch_status}"
