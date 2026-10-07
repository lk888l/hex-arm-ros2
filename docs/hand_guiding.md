# Hand guiding with gravity compensation

**English** | [中文](hand_guiding_cn.md) · [Back to README](../README.md#hand-guiding)

Run the commands from the built and sourced container workspace `/workspaces/hex_arm_ros2`.

## Behavior

The dedicated `gravity_comp.launch.py` provides measured-pose gravity compensation with
zero position stiffness and configurable joint damping. It starts no trajectory controller,
MoveIt process or unfolding sequence. Stop other arm controllers before using it.
## Startup

Build the three affected packages and source the workspace inside the container:

```bash
./scripts/build.sh --packages-select hex_arm_controller hex_arm_bridge hex_arm_bringup
source install/setup.bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.hand_guiding_full_range.local.yaml \
  activate_hardware:=true \
  damping:='[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]'
```

## Profile and range

The current arm uses `replacement.local.yaml`, matching its adapter and six motor identities.
The older `firefly_y6.meow.can2.local.yaml` belongs to the previous arm; rebinding its adapter
does not transfer motor identities or calibration. The dedicated `hand_guiding_full_range.local.yaml`
uses separate 2.0 rad/s hand-guiding speed trips and the URDF position ranges: J1 ±2.86, J2 [-1.57, 2.09],
J3/J4 ±1.57, J5 ±1.54, J6 ±2.79 rad. The original profile retains its narrow windows.
This is an operator-requested tuning candidate, not physical full-range validation; inherited
calibration flags refer to existing coordinates. Individual model limits do not establish
collision-free joint combinations or verified mechanical stops. All six gravity scales are 1.0;
torque caps retain prior tuning. Support may be insufficient in
new poses. Keep supporting the arm. Profile changes require no rebuild.

## Damping

The six positive damping gains are joint-side N·m·s/rad, J1–J6, defaulting to 2.0 on each axis.
More damping slows released motion faster but increases
drag resistance. Gravity/model errors can still cause drift; this mode does not lock position.
Existing gravity scales tuned alongside position PD may need separate qualification.

## Entry checks and protection

Support the arm during enable and the gravity ramp; wait for `hand_guiding_ready`.
Entry requires a calibrated schema v3 profile, fresh feedback, valid positions, and
joint speeds at most 0.02 rad/s. The hand-guiding profile sets
`controller.hand_guiding_velocity_limits_rad_s` to six absolute measured-speed trips of
2.0 rad/s (114.6°/s), applied only in `GRAVITY_COMP` without adding joint speed margins.
Command speeds and other modes continue using `joints[].limits.velocity_rad_s`.
Omitting the optional array retains the ordinary speed-plus-margin trips; supplied limits
must be six finite positive values at most 6 rad/s, ordered J1–J6. Damping cannot guarantee
a speed bound, so overspeed and non-finite-feedback protection remain enforced.
Overspeed faults disable the arm and do not indicate a locked joint or insufficient position range.
The dedicated profile sets `controller.hand_guiding_position_margin_rad` to 0.012 rad
(0.69°), replacing ordinary measured-position margins only when entering/running
`GRAVITY_COMP`. Other modes and external position commands retain strict bounds.
Omission uses each joint's ordinary margin; overrides must be finite within 0–0.012 rad
and keep the feedback window shorter than a full turn. This also fits the existing
Meow enable position-consistency check. Within the accepted margin, only the zero-Kp
MIT position field is bounded to the command range; gravity still uses the actual
encoder angles, without truncation. Out-of-envelope or non-finite feedback remains rejected.
At the
folded pose, J2 is near its -1.57 rad lower bound and must be guided inward.
This mode has no MoveIt collision checking.
## Stop

Support the arm before stopping, then call:

```bash
ros2 service call /hex_arm_gravity_comp/stop std_srvs/srv/Trigger '{}'
```

A successful response confirms disable and session release; launch then exits. Ctrl-C also
stops, without returning or folding. The owner renews a 500 ms driver lease every 50 ms;
owner death, freezing or lost communication trips a latched fault and confirmed-disable retries.
Restart only after investigating the fault. The driver writes `hand-guiding-shutdown.json`
under the current ROS log directory; require `disabled_confirmed` as the stop acknowledgement.

## Observation and mock

Without `activate_hardware:=true`, launch stays in observation mode. For software-only testing,
use `mock:=true` with `src/hex_arm_controller/test/firefly_y6.mock.yaml` as an absolute profile path.
Mock does not simulate gravity or hand-guiding feel. See the
[Chinese operating instructions](hand_guiding_cn.md) for more detail.

