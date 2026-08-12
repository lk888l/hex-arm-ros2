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
  -> userspace gs_usb -> CAN-FD motors
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
| `real` | Rust controller, Zenoh bridge, ros2_control | `/dev/bus/usb` only |

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

On native Ubuntu, the helper selects `compose.ubuntu.yaml`, passes the current
X11 authorization cookie read-only, and maps `/dev/dri` so RViz, MoveIt, and
Gazebo can open desktop windows.

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


The real profile is intentionally a separate Compose override. It maps only
the USB bus and never enables Docker privileged mode:

```bash
HEX_ARM_REAL=1 ./scripts/docker-dev.sh up
```

Copy `config/hardware/firefly_y6.example.yaml` to an ignored `*.local.yaml`,
fill every identity, direction, offset, and limit, then validate it before a
real launch. The example is marked incomplete and is rejected by both the
bringup layer and the Rust controller.

```bash
ros2 launch hex_arm_bringup real.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

## Safety boundary

Real startup always begins disabled. Activation requires six fresh identified
motors, a complete calibrated profile, an exclusive robot_api session, and an
explicit ros2_control lifecycle transition. A motor fault, stale feedback,
non-finite command, USB failure, or command watchdog timeout latches a
whole-arm fault and initiates stop/disable.

WSL2, Docker, Zenoh, and userspace USB are not hard-real-time or
safety-certified components. A working physical emergency stop is mandatory
for commissioning. This repository does not claim that joint/node mapping,
zero offsets, motion direction, torque calibration, or execution accuracy has
been validated on hardware.

The historical MoveIt SRDF is not installed because it disables all 21
self-collision pairs. MoveIt integration is limited to stable joint names,
controller name, and the standard trajectory action until a collision model is
regenerated and reviewed.

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

