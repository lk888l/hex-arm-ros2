# MoveIt graphical mock simulation

**English** | [中文](moveit_simulation_cn.md)

This workflow exercises MoveIt, OMPL, collision checking, inverse kinematics,
`FollowJointTrajectory`, and the existing ros2_control `GenericSystem`. It does
not access USB/CAN hardware and is not a physics simulation.

## Native Ubuntu 24.04 startup (this machine)

Start the container and check graphical forwarding in an Ubuntu desktop host
terminal:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

After entering the `ros2-jazzy-arm` container, whose prompt hostname is
`hex-arm-dev`, run:

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh  # Run on first use or after source changes
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

The last command starts `move_group` and RViz. The MoveIt window should appear
on the Ubuntu desktop.

## WSL2 startup (Windows 10/11 only)

WSL2 means **Windows Subsystem for Linux 2**: Ubuntu runs inside Windows and
WSLg displays its graphical windows. Run the host commands in the Ubuntu/WSL
terminal in Windows, not in PowerShell:

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2  # Replace if the repository is elsewhere
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

Inside the same `ros2-jazzy-arm` container, the MoveIt commands are identical:

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh  # Run on first use or after source changes
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

Stop any existing `hex_arm_bringup mock.launch.py` session first when it uses
the same `ROS_DOMAIN_ID`; both launch files intentionally use the same
controller names. If RViz reports `could not connect to display :0`, run
`./scripts/docker-dev.sh doctor` on the host. The helper checks Xauthority on
native Ubuntu and selects WSLg on WSL2; do not bypass access control with
`xhost +`. Re-enter the container and start MoveIt after the check passes.

In RViz, select the `arm` planning group. Drag the interactive marker or choose
the named `commissioning_start` state, then use **Plan** first and **Plan &
Execute** after the preview looks correct on mock hardware. This named state is
an inset planning reference, not a calibrated real-hardware home. The mock
execution path is:

```text
MoveIt RViz -> move_group -> firefly_arm_controller
             -> ros2_control GenericSystem -> /joint_states -> RViz
```

The default `sim` profile keeps the URDF limit of `6.0 rad/s` and uses a default
MoveIt scaling factor of 0.1. To preview the deliberately slow first-hardware
policy without connecting hardware:

```bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py \
  limits_profile:=commissioning
```

Available profiles are:

| Profile | Per-joint maximum | Intended use |
|---|---:|---|
| `sim` | 6.0 rad/s | software-only graphical testing |
| `commissioning` | 0.1 rad/s | first supervised hardware commissioning (with narrow position windows) |
| `verified` | 6.0 rad/s | only after staged real-hardware verification |

MoveIt limits are planning limits, not the real safety boundary. A real launch
must use a verified local hardware profile whose `velocity_rad_s` and
`acceleration_rad_s2` are no greater than the selected MoveIt profile. The Rust
controller independently replaces position lerp with per-axis cubic Hermite
segments. Position and velocity remain continuous when a streaming command is
retargeted; exact cubic extrema are checked against both hardware-profile limits
and the requested duration is extended when needed. Acceleration may change
discontinuously within its bound (jerk is not limited). Invalid or non-positive
limits fail profile loading. MoveIt still needs its own acceleration limits for
time parameterization: `sim` uses 10.0 rad/s^2 only for software visualization,
`commissioning` uses 0.1 rad/s^2, and `verified` remains provisional at 0.2
rad/s^2 until measured.

The SRDF does not reuse the historical 21-pair collision-disable blanket. The
strict default `firefly_y6.srdf` excludes only the six kinematically adjacent
pairs because they share joint/interface geometry. FCL reports two additional
contacts at the corrected physically occupied surveyed fold
`q~=[0,-1.570,1.570,0,0,0]`:

- `link_1` with `link_5`;
- `link_2` with `link_4`.

Those exact two pairs are not part of the strict executable matrix. Instead,
the real MoveIt launch loads a separate `firefly_y6.plan_only.srdf` only when
`enable_execution:=false`, and only that plan-only overlay adds the two
`PlanOnlySurveyedFold` exceptions. Mock MoveIt and real MoveIt with
`enable_execution:=true` stay on the strict SRDF, so all 15 non-adjacent pairs
remain checked on any executable path. This is not an assertion that the two
pairs can never collide: an independent 100,000-state full-URDF-range
`moveit_setup_assistant collisions_updater` run reports zero permanently
colliding non-adjacent pairs. The overlay is therefore a documented
visualization/debug workaround until corrected collision meshes are
measured/exported. The other thirteen non-adjacent pairs remain active even in
plan-only mode. In particular, the offline FCL regression at the discarded
`q=[0,+1.570,1.570,0,0,0]` pose still exposes five non-exempt contacts. Any plan produced
under `firefly_y6.plan_only.srdf` is preview-only and must not be reused for
execution; real execution requires re-planning under the strict SRDF after the
collision geometry and the physical start posture have been corrected. The
first version uses `link_6` as the planning tip until a calibrated fixed
TCP/tool frame is available. The old out-of-window `ready` state has been
removed. `commissioning_start` matches the surveyed posture but places joints
2/3 at -1.56/1.56 rad, 0.01 rad inside their provisional limits; it is a planning
reference only and must not be executed on an uncalibrated arm.

## Real MoveIt: planning only by default

First configure the native-Ubuntu SocketCAN interface selected by the profile
exactly as described in the
[commissioning checklist](commissioning.md): 1M/SP0.8/SJW5 arbitration,
4M/SP0.8/SJW3 data, FD, and `restart-ms 0`. Then start the normal container:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
```

Start the real entry point from the host through the supervised helper. The
current example is `can2`, channel 2; both values must match the YAML profile:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

This command does not auto-enable hardware. On `Ctrl-C`, the host uses a second
exact `docker exec` to validate this invocation's random token/PGID file and
signal only that process group, while it continues waiting on the original
Compose exec for the clean Rust-controller disable/heartbeat-disarm result.
Keep it attached until it prints the verification result; an independent
`can1` launch is not signalled. Do not put the real launch behind a bare
`docker exec ... bash -lc` command.

`enable_execution` defaults to `false`: the included real bringup keeps
`activate_hardware=false`, while `move_group` sets
`allow_trajectory_execution=false` and loads `firefly_y6.plan_only.srdf`. RViz
can display `/hex_arm/internal/state` and plan, but Execute cannot reach
hardware. A `validated: true, calibrated: false` profile whose structure, node
identities, and `expected_link` have been reviewed may be used for this
disabled-state observation; activation is still explicitly rejected. Nodes 1--6
map directly to joints 1--6. Node 15 is still excluded from all drive control,
but a configured `tip_payload` requires its exact identity and includes its
fixed mass/COM in the `link_6` gravity model.

The approximate pose `q=[0,-1.570,1.570,0,0,0]`, historical sign candidates, and
one-snapshot offset fit are commissioning references, not a home or motion
validation. Correcting the omitted joint-2 minus sign maps its full URDF range
inside the single-turn seam guard. Joint 2 nevertheless starts at its lower
limit and still needs a bounded inward reposition plus bidirectional tracking
validation before `enable_execution:=true`. A `calibrated` flag alone does not
waive this independent gate.

Only after those issues are closed is the execution form:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  enable_execution:=true
```

The real launch with `enable_execution:=true` switches back to the strict
`firefly_y6.srdf`, still uses the 0.1 rad/s, 0.1 rad/s^2 commissioning
planning limits, and keeps the narrow surveyed position windows.
The downstream JointTrajectoryController also enforces 0.02 rad path,
0.005 rad goal, and 0.02 rad/s stopped-velocity tolerances with a 1.0 s goal
allowance on every axis. A 0.016 rad final joint-2 error therefore fails the
action instead of being accepted. This hardening does not make execution ready:
the calibration, joint-2 tracking, payload, TCP, and strict-collision gates below
remain closed.

Empty bridge
`kp`/`kd` vectors select the Rust per-axis low-gain defaults; an empty
`tau_ff` delegates automatic gravity feedforward to Rust. Six explicit zero
`tau_ff` values instead disable that automatic feedforward. Delegated gravity
is multiplied per axis by the hardware profile's
`gravity_compensation_scale`, exactly as in single-axis commissioning; this
real-arm identification parameter is not the motor conversion `torque_scale`.
Before that scaling, schema-v2 field `gravity_vector_base_m_s2` supplies the
mandatory gravity vector in URDF `base_link` coordinates. Real launch rejects
v1 or missing-vector profiles; after correcting the missing joint-2 sign, the
current local `-Z` value is again only an uncalibrated commissioning candidate.
A runtime `SetGravity` override lasts
only for its exclusive session and resets to the profile value on release,
shutdown, or a new acquire.
Any non-empty external `tau_ff` bypasses both the automatic payload model and
that gravity scale (motor torque conversion still applies); the ROS bridge uses
the empty, delegated path. The local GR80's 0.41 kg mass/COM comes from a trial
URDF and remains `inertial_calibrated: false`, which makes a calibrated profile
invalid and blocks real MoveIt execution. The real TCP, calibrated tool
inertials, gripper transform, and final collision geometry remain outstanding,
so `link_6` is only a provisional flange tip. The launch contract
and mock regression are verified offline; no real motion or real MoveIt
trajectory test is claimed.

### Jazzy shutdown compatibility

The current Jazzy binaries (MoveIt 2.12.4 with rclcpp 28.1.21) have a known
`move_group` callback-group destruction fault on shutdown. This project routes
both mock and real launches through `hex_arm_moveit_runtime`, which provides an
ordered-shutdown executable and a narrowly scoped compatibility shim. Its CMake
checks intentionally fail closed on other dependency versions: after a MoveIt
or rclcpp upgrade, re-audit and remove or update the workaround instead of
bypassing the version check.

Run the non-GUI planning and execution smoke test with:

```bash
python3 src/hex_arm_moveit_config/test/test_moveit_mock.py
```
