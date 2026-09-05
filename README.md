# Firefly Y6 ROS 2 driver

[中文版 (Chinese)](README_cn.md)

ROS 2 Jazzy driver, simulation, and commissioning workspace for the six-axis
Firefly Y6 arm. The public motion interface is the standard
`control_msgs/action/FollowJointTrajectory` action exposed by
`joint_trajectory_controller`:

```text
/firefly_arm_controller/follow_joint_trajectory
```

The project deliberately separates the ROS control loop from the USB/CAN-FD
loop:

```text
MoveIt / FollowJointTrajectory
  -> ros2_control + firefly_arm_controller
  -> hex_arm_hardware/SystemInterface
  -> hex_arm_bridge (ROS lifecycle <-> Zenoh robot_api)
  -> hex_arm_controller (Rust safety state machine, 1 kHz soft-real-time loop)
  -> SocketCAN can0 (field) or userspace gs_usb (legacy) -> CAN-FD motors
```

The Rust process owns the motor bus and all safety decisions. The ROS bridge
does not implement a trajectory action and cannot bypass the exclusive
session or activation state machine.

## Supported backends

| Backend | Purpose | Hardware access |
|---|---|---|
| `view` | URDF, joint direction, limits, and RViz inspection | none |
| `mock` | ros2_control/JTC lifecycle and action integration | none |
| `gz` | Gazebo Harmonic physics through `gz_ros2_control` | none |
| `real` | Rust controller, Zenoh bridge, ros2_control | host `can0`, or `/dev/bus/usb` for legacy `gs_usb` |

## Starting the Docker development environment

### Native Ubuntu 24.04 (this machine)

“Native Ubuntu” means that the computer boots Ubuntu directly, rather than
running Ubuntu inside Windows. The current repository at
`/home/kk/kk_data/ros2_project/hex-arm-ros2` is in this environment. Run:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

`build` is needed only on first use or after changing the Dockerfile. A normal
daily start is:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

On native Ubuntu, the helper selects `compose.ubuntu.yaml` and passes the current
X11 authorization cookie read-only. The generic AMD/Intel path maps `/dev/dri`;
the NVIDIA override requests devices through Container Toolkit instead.

#### Direct Docker Compose commands (without the helper)

`docker compose` is Docker's Compose CLI. The following commands are equivalent
to `docker-dev.sh` on native Ubuntu. Run them from an Ubuntu graphical desktop
terminal and set `HEX_ARM_XAUTHORITY` in that terminal first:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2

# Prefer XAUTHORITY from the desktop session; otherwise use ~/.Xauthority
if [[ -n "${XAUTHORITY:-}" ]]; then
  export HEX_ARM_XAUTHORITY="$XAUTHORITY"
else
  export HEX_ARM_XAUTHORITY="$(getent passwd "$(id -u)" | cut -d: -f6)/.Xauthority"
fi

# This must print GUI prerequisites: OK
test -n "${DISPLAY:-}" \
  && test -r "$HEX_ARM_XAUTHORITY" \
  && test -e /dev/dri \
  && echo "GUI prerequisites: OK"

# Build on first use or after changing the Dockerfile
docker compose -f compose.ubuntu.yaml build

# Start the container in the background
docker compose -f compose.ubuntu.yaml up -d

# Optional: verify X11 access and show the OpenGL renderer
docker compose -f compose.ubuntu.yaml exec -T ros2-jazzy-arm \
  bash -lc 'xdpyinfo >/dev/null && glxinfo -B'

# Enter the ROS 2 container
docker compose -f compose.ubuntu.yaml exec ros2-jazzy-arm bash
```

After leaving the container, inspect logs or stop it from the same host terminal,
where `HEX_ARM_XAUTHORITY` is still set:

```bash
docker compose -f compose.ubuntu.yaml logs -f
docker compose -f compose.ubuntu.yaml down
```

Set `HEX_ARM_XAUTHORITY` again in every new host terminal before running these
Compose commands. Do not replace native Ubuntu's `compose.ubuntu.yaml` with the
WSL2-only `compose.yaml`, and do not run `xhost +`.

#### NVIDIA discrete GPU acceleration

On native Ubuntu, the helper defaults to `HEX_ARM_GPU=auto`: it automatically
adds `compose.nvidia.yaml` when a working NVIDIA GPU is detected, and otherwise
keeps the generic `/dev/dri` path. The NVIDIA override clears that base device
mapping, so an NVIDIA-only host does not need `/dev/dri`, including a real
launch with `use_rviz:=false`. `HEX_ARM_GPU=none` still requires `/dev/dri`.
Before first use, install NVIDIA Container
Toolkit on the host. Do not install the host graphics driver in the Dockerfile:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes \
      -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Restarting Docker stops all containers that are running at that moment, but
does not stop CUDA processes running directly on the host. Recreate and check
this project's container afterward:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
```

The mode can also be selected explicitly:

```bash
# Require NVIDIA; fail immediately when the GPU or Toolkit is unavailable
HEX_ARM_GPU=nvidia ./scripts/docker-dev.sh up

# Disable the NVIDIA override and use AMD/Intel DRI or software rendering
HEX_ARM_GPU=none ./scripts/docker-dev.sh up
```

Without the helper, set `HEX_ARM_XAUTHORITY` as shown above and explicitly add
the NVIDIA override:

```bash
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml up -d
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml \
  exec -T ros2-jazzy-arm nvidia-smi
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml \
  exec -T ros2-jazzy-arm glxinfo -B
```

The override uses Compose's standard `!reset` tag. If `docker compose config`
does not recognize it, update the Docker Compose plugin first.

A successful `doctor` reports the RTX model,
`OpenGL renderer string: NVIDIA ...`, and `NVIDIA GPU acceleration: OK`.
`llvmpipe` means CPU software rendering. The image already contains its
GLVND/OpenGL userspace dependencies; do not add the NVIDIA kernel or host driver
packages to the Dockerfile. RViz/Gazebo shares GPU memory and compute with
host CUDA workloads such as LeRobot training, so avoid running them together.

### WSL2 (Windows 10/11 only)

WSL2 means **Windows Subsystem for Linux 2**, the Linux virtualization
environment built into Windows. Ubuntu is running under WSL2 only when it was
started from Windows and `uname -r` contains `microsoft-standard-WSL2`.
Linux GUI windows are displayed through WSLg.

Run the following commands in the **Ubuntu/WSL terminal** in Windows, not in
PowerShell or Command Prompt. Replace the first path if the repository is
elsewhere:

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

The same helper selects `compose.yaml` and the WSLg sockets on WSL2. Neither
environment requires the overly broad `xhost +` command. The existing image is
already Ubuntu 24.04 (ROS 2 Jazzy), so a duplicate native-Ubuntu Dockerfile is
unnecessary.

Inside the container the prompt hostname is `hex-arm-dev`. The older
`ros2-jazzy` container uses the hostname `ros2-dev`; do not source this
workspace's generated `install/` tree there. A `--symlink-install` build is
bound to the `/workspaces/hex_arm_ros2` mount used by `ros2-jazzy-arm`.

Build once inside `ros2-jazzy-arm`:

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
```

Then choose one graphical mode per terminal:

```bash
# URDF + joint sliders + RViz
ros2 launch hex_arm_bringup view.launch.py

# ros2_control GenericSystem + trajectory controller + RViz
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true

# Gazebo Harmonic physics + RViz
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true
```

Stop a launch with `Ctrl-C` before starting another mode. If `source
install/setup.bash` reports paths below `/workspaces/hex_arm_ros2` as missing,
the shell is in the wrong container; enter it with
`./scripts/docker-dev.sh shell` from the host.

## Driving the simulated arm from the CLI (`ros2 action send_goal` reference)

After starting the `mock` or `gz` mode, run this in another terminal (with
`source install/setup.bash` already applied):

```bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, 1.25, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

### Command format

```text
ros2 action send_goal <action_server> <action_type> "<goal_yaml>"
```

| Argument | Example | Meaning |
|---|---|---|
| Subcommand | `ros2 action send_goal` | Sends an action goal; `ros2 action` also supports `list`, `info`, and `type` |
| Action server | `/firefly_arm_controller/follow_joint_trajectory` | The trajectory controller's action server. Start a `mock`/`gz` launch first, otherwise the CLI keeps printing `waiting` |
| Action type | `control_msgs/action/FollowJointTrajectory` | Tells the CLI how to parse the YAML; must match this workspace |
| Goal payload | YAML in double quotes | See field rules below |

### YAML field rules

- `trajectory.joint_names`: joint-name list. The order is **fixed** to
  `joint_1` ... `joint_6`, and all six must be present (the controller sets
  `allow_partial_joints_goal: false`, so partial goals are rejected).
- `trajectory.points`: array of trajectory points (one in this example). Each
  point has:
  - `positions`: target angles in **radians**, exactly six values in
    `joint_names` order. Keep them inside the URDF limits: joint_1 ±2.86,
    joint_2 −1.57~2.09, joint_3 0~3.14, joint_4 ±1.57, joint_5 ±1.54,
    joint_6 ±2.79. Note that the mock/gz configs do **not** enable command
    limit clamping, so out-of-limit values are not automatically rejected —
    make sure the numbers are safe yourself.
  - `time_from_start`: offset from the moment the goal is accepted;
    `{sec: 2, nanosec: 0}` means "reach within 2 seconds". The controller
    smooths the motion with `interpolation_method: splines`.
  - Optional: `velocities`, `accelerations`, `effort`; omitted fields are
    interpolated by the controller.
  - Add more points to build multi-segment trajectories, increasing
    `time_from_start` for each.

### Shell and usage details

- Wrap the whole YAML in **double quotes** so spaces are not split into
  separate shell arguments; the trailing `\` is just a line continuation and
  can be removed to put everything on one line.
- Source the environment first (interactive bash does this automatically;
  otherwise run `source /opt/ros/jazzy/setup.bash` and
  `source install/setup.bash`).
- After sending, expect `Goal accepted with ID: ...` and then
  `Goal finished with status: SUCCEEDED` (`error_code: 0`).
- Verify the final pose with `ros2 topic echo --once /joint_states`; the
  `position` values should match the goal.
- A new goal cancels/replaces the goal currently being executed (default
  controller behavior); `Ctrl-C` only exits the CLI client.
- The same command works for `gz`; there the trajectory advances with the
  Gazebo (simulation) clock.

More GUI and CLI troubleshooting: [docs/gui_and_cli_simulation.md](docs/gui_and_cli_simulation.md).


## Real hardware: read-only discovery first

The field bus contract is CAN-FD at `1 Mbit/s, SP=0.8, SJW=5` arbitration and
`4 Mbit/s, SP=0.8, SJW=3` data timing, with `restart-ms 0`. Configure it on the
native-Ubuntu host before starting any ROS or GUI process:

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can \
  bitrate 1000000 sample-point 0.8 sjw 5 \
  dbitrate 4000000 dsample-point 0.8 dsjw 3 \
  fd on restart-ms 0
sudo ip link set dev can0 up
ip -details -statistics link show dev can0
```

As a host-only convenience, `can-config set can0 4M` applies the 1M/4M timing
but does not change motor firmware and currently does not write `restart-ms`.
Never run it while the ROS driver, MoveIt, or the motor GUI is using `can0`, and
always verify the complete link afterwards. See
[the supervised commissioning checklist](docs/commissioning.md), then start the
normal container:

```bash
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

The Ubuntu Compose service uses host networking, so the container sees the
host `can0`; `HEX_ARM_REAL=1` is not needed for SocketCAN. Inside the built
container, identify the six arm nodes and the known auxiliary node without
loading a hardware profile or initializing a drive:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --discover-only --transport socket-can --interface can0 \
  --expected-node 1 --expected-node 2 --expected-node 3 \
  --expected-node 4 --expected-node 5 --expected-node 6 \
  --auxiliary-node 15 --timeout 2 --sdo-timeout-sec 0.25
```

This mode listens for heartbeats and permits only CANopen identity SDO uploads;
it cannot send NMT, PDO, SDO downloads, controlwords, or motor commands. See
[docs/commissioning.md](docs/commissioning.md) before connecting power. The
legacy direct `gs_usb` path still requires `HEX_ARM_REAL=1`; it is fixed at
1M/5M and must not be used on the field 1M/4M chain.

A successful discovery proves node presence and identity only. The installed
arm uses the confirmed direct mapping node 1 -> joint_1 through node 6 ->
joint_6. Node 15 (`0x0f`) is the gripper and remains excluded from every arm
initialize/enable/disable/command path. It is not ignored as a physical load:
when `tip_payload` is present, startup requires node 15's exact `0x1018`
fingerprint and the controller merges its fixed mass/COM into `link_6` for
gravity. Direction, zero, limits, torque scaling, and the real TCP still require
commissioning. The direct-GUI userspace `gs_usb` path and ROS SocketCAN path must
never own the same USB-CANFD adapter simultaneously.

Copy `config/hardware/firefly_y6.example.yaml` to an ignored `*.local.yaml`,
fill the node identities plus the strict `expected_link` timing/USB fingerprint,
keep `bus.direct_joint_mapping: true`, and review the candidate axis values.
A `validated: true` but
`calibrated: false` profile may be used for disabled observation; activation is
explicitly rejected until calibration is complete.

Validate the profile, URDF dynamics model, and single-turn command windows
without opening CAN before any real launch:

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --validate-profile-only
```

After that offline check, the only uncalibrated motion entry is the isolated
single-axis commissioning CLI (no ROS graph or Zenoh session). It requires all
four motion arguments and performs one smooth out-and-back move; this example
physically enables node 1, so use it only after confirming the sign at the arm:

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --commission-axis joint_1 --delta-rad 0.01 --duration-sec 2.0 \
  --allow-motion
```

The CLI caps `abs(delta)` at 0.03 rad, checks the profile's independent
`velocity_rad_s` and `acceleration_rad_s2` limits before enabling only the selected
node, continuously checks all feedback and limits, returns to the starting
position, rejects no-motion/wrong-direction/overshoot results using measured
signed excursion, and requires a feedback-confirmed disable even on error or Ctrl-C.
See [the commissioning checklist](docs/commissioning.md) before using it.

Run a real launch from the **host** through the supervised Docker entry point.
The interface and channel must exactly match the YAML profile (the current
field connection is shown as `can2`, channel 2):

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

The bringup launch still defaults to `activate_hardware:=false`; this helper
does not enable it. The host helper generates a random token, keeps the primary
Compose exec isolated and attached in the background, and the container binds
that token to this launch's exact process-group ID. On `Ctrl-C`, SIGTERM, or
terminal hangup, a second non-TTY `docker exec` validates that token and sends
INT/TERM only to the bound negative PGID; the host then resumes waiting for the
original Compose exec. An early request recorded before a PGID exists prevents
ROS from starting, and stale or malformed state is rejected. The command
reports `VERIFIED clean controller exit` only after `hex_arm_controller` has
returned through its confirmed all-axis disable and heartbeat-consumer disarm
path. A separate `can1` launch is neither signalled nor killed. If the
verification line is absent, treat shutdown as unconfirmed and inspect the
retained `/tmp/hex-arm-real-launch.../launch.log` before starting another owner
of the same interface.

The real MoveIt entry is also plan-only by default and always uses the slow
commissioning limits:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

Do not wrap a real launch in a bare
`docker exec ... bash -lc 'ros2 launch ...'`: interrupting that outer exec can
leave the launch/controller descendants running. Do not use `pkill`, `killall`,
container restart, or `docker compose down` as a normal stop path because none
of them can provide the controller's disable/disarm confirmation. Keep the
supervised command attached until it prints its final verification result.

Do not set `enable_execution:=true` until the profile is calibrated and the
corrected joint-2 zero/range has passed full real-hardware validation.

`0x603F=0x8130` may be retained last-error history. If current
`0x6041=0x0231`, the drive is already non-Fault/non-OE and must not be reset just
to erase history. Orderly exit now confirms all-axis disable and, while the host
heartbeat is still live, reads back `0x1016:01=0` on every arm drive so exit does
not manufacture another heartbeat-loss event. The explicitly gated recovery
CLI is only for a current `0x6041` Fault with exact last-error `0x8130`; see the
[hardware commissioning guide](docs/commissioning.md) for its command and gates.

## Safety boundary

`activate_hardware` defaults to `false`, so the command above starts the Rust
controller, bridge, robot-state publisher, and optional RViz only; it does not
start ros2_control or trajectory controllers. This observation launch does
initialize drives in the disabled state, so use `--discover-only` first on an
unknown bus.

Real startup always begins disabled. Activation requires six fresh identified
motors, a complete calibrated profile, an exclusive robot_api session, and an
explicit ros2_control lifecycle transition. A motor fault, stale feedback,
non-finite command, CAN transport failure, or command watchdog timeout latches a
whole-arm fault and initiates stop/disable.

WSL2, Docker, Zenoh, SocketCAN, and userspace USB are not hard-real-time or
safety-certified components. A working physical emergency stop is mandatory
for commissioning. Current pose evidence is approximately
`q=[0,-1.570,3.140,0,0,0]`. Historical signs
`[-1,-1,+1,+1,+1,+1]` and snapshot-derived offsets are commissioning evidence,
not motion validation. Correcting the previously omitted joint-2 minus sign
maps its full URDF range to approximately `[-0.3310,+0.2515]` motor rev, so the
old wrap-seam blocker was an artifact of the wrong offset. The local hardware,
URDF, and MoveIt lower bounds remain aligned at -1.570: a power-cycle reading
below it fails closed instead of silently widening the command envelope.
Joint 2 still starts at that boundary and must be re-commissioned
inward before real execution.

The historical `hex-ros2-arm` bridge and generated MoveIt package have been
audited rather than copied. Their joint signs, 0.85 torque factors for joints
1--3, planning chain, and FJT endpoint are already represented here; their
auto-ACTIVE bridge, unchecked action success, 6/10 rad limits, unreviewed
hard-coded gravity, and all-collisions-disabled SRDF are intentionally rejected. See the
[commissioning audit](docs/commissioning.md#legacy-hex-ros2-arm-reuse-audit).

The bridge sends empty `kp`, `kd`, and `tau_ff`: Rust selects the reviewed
per-axis low gains and computes automatic gravity feedforward from the arm URDF
plus any configured fixed `tip_payload`. A non-empty external `tau_ff` bypasses
that automatic payload model and `gravity_compensation_scale`. API `kp`/`kd`
are calibrated joint-side SI gains in `Nm/rad` and `Nm*s/rad`, and `tau_ff` is
joint-side `Nm`. The per-axis `torque_scale` therefore applies to every
torque-producing MIT term on the way to the motor--feed-forward plus the P and
D gain coefficients--and its inverse applies to measured torque on the way
back to ROS. The ROS/MoveIt bridge deliberately sends an empty vector, so it
follows the payload-aware automatic path. The local
field profile, rather than this prose, is authoritative for per-axis gains,
gravity scales, limits, and torque authority. At this checkpoint J1..J6 Kp/Kd
are `60/2.5`, `80/2.5`, `2/0.3`, `30/1`, `30/1`, and `20/1`, while staged
J2/J3/J4 gravity scales are `0.25/0/0`; all remain commissioning candidates,
not certified safety values. This repository does
not claim that zero offsets, motion direction, torque calibration, or execution
accuracy has been validated by real motion.

Real hardware profiles use schema v2 and must explicitly provide
`gravity_vector_base_m_s2` in URDF `base_link` coordinates (m/s^2); v1 and a
missing vector are rejected because mounting direction is safety-critical. The
current local commissioning candidate is `[0.0, 0.0, -9.81]` and remains
uncalibrated. Both commissioning and runtime use it before applying each
joint's `gravity_compensation_scale`; that scalar is not a gravity-direction
switch. `SetGravity` is only a current-session override, and release, shutdown,
or a new session restores the profile value. See
[real-hardware commissioning](docs/commissioning.md) for migration and
validation details.

The default strict SRDF `firefly_y6.srdf` excludes only the six directly
adjacent pairs. Mock MoveIt and real MoveIt with `enable_execution:=true`
always use that strict matrix, so all 15 non-adjacent pairs remain collision
checked on any executable path. A separate `firefly_y6.plan_only.srdf` is
loaded only by the real MoveIt launch when `enable_execution:=false`; it adds
exactly two `PlanOnlySurveyedFold` provenance-mesh exceptions for the corrected
surveyed fold: `link_1`--`link_5` and `link_2`--`link_4`. The other thirteen
non-adjacent pairs stay active there as well, and an offline FCL guard at the
previously incorrect `q=[0,+1.570,3.140,0,0,0]` pose still exposes five
unrelated contacts. A 100,000-state
full-range MoveIt sample found no permanently colliding non-adjacent pair, so
no unrelated `Never` pair or old TEMP blanket is copied. The plan-only overlay
is a documented collision-model workaround until corrected meshes are
available, not a physical safety claim. Plans produced under that relaxed
matrix are visualization/debug previews only; they must not be reused for
execution, and real hardware execution must be re-planned and re-validated
under the strict SRDF after collision geometry and the physical start posture
have been corrected. The local GR80 entry uses the trial-URDF mass/COM
(0.41 kg with an identity mount in `link_6`) only as a gravity-model
placeholder. Its `inertial_calibrated: false` value makes `calibrated: true`
invalid and therefore prevents real MoveIt execution. `link_6` remains the
provisional planning tip until calibrated tool inertials, a TCP/tool frame,
and final collision geometry are reviewed; the real MoveIt launch therefore
treats `link_6` as a provisional flange tip. Its launch contract and mock
regression are tested offline, but no real motor motion or real MoveIt
trajectory execution is claimed.

## Reproducibility

`hex_arm.repos` pins upstream source revisions. The checked-in
`xpkg_urdf_firefly_y6` package is a provenance-preserving snapshot of the
specified description revision, including original meshes. Run:

```bash
vcs import . < hex_arm.repos
```

only when updating or auditing upstream sources; the normal build does not
silently fetch mutable branches.

## Test levels

```bash
./scripts/test.sh unit
./scripts/test.sh protocol
./scripts/test.sh mock
./scripts/test.sh gz
```

`unit` is hardware-free. `protocol` starts the Rust controller with its mock
motor backend and validates Zenoh discovery/events plus the ROS lifecycle bridge.
`mock` exercises FollowJointTrajectory send/cancel and controller lifecycle. `gz`
launches Gazebo headlessly and verifies two trajectories. Real commissioning is a separate, supervised checklist in
`docs/commissioning.md`.

