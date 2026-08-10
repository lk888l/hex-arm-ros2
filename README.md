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

Start the development container without USB access. Run these commands from
the **Ubuntu 24.04 WSL shell**, not PowerShell: WSL must resolve `/mnt/wslg`
and `/tmp/.X11-unix` so RViz and Gazebo can reach WSLg.

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2
docker compose build
docker compose up -d
docker exec -it ros2-jazzy-arm bash
```

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
the shell is in the wrong container; enter it with `docker exec -it
ros2-jazzy-arm bash` from the WSL host.

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
docker compose -f compose.yaml -f compose.real.yaml up -d
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

