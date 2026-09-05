# Supervised hardware commissioning

**English** | [中文](commissioning_cn.md)

Do not run real bringup until the arm is mechanically secured, its workspace
is clear, and the independent physical emergency stop has been tested. WSL2
and Docker are not safety systems.

The supported field path is Linux SocketCAN. Its complete timing contract is
`1 Mbit/s, SP=0.8, SJW=5` for arbitration, `4 Mbit/s, SP=0.8, SJW=3` for data,
CAN-FD enabled, and automatic bus-off restart disabled (`restart-ms 0`). Set
every field on the native-Ubuntu host before opening the bus:

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can \
  bitrate 1000000 sample-point 0.8 sjw 5 \
  dbitrate 4000000 dsample-point 0.8 dsjw 3 \
  fd on restart-ms 0
sudo ip link set dev can0 up
ip -details -statistics link show dev can0
```

The last command must report every bitrate, sample point and SJW above,
`fd on`, `restart-ms 0`, and `ERROR-ACTIVE`. TEC/REC, cumulative CAN errors,
and netdev error counters must all be zero. Historical netdev `dropped` counters
may be nonzero only when unchanged across two preflight samples 0.2 seconds
apart; any increase fails closed. A real profile's `bus.expected_link` checks
those values and the USB-adapter fingerprint before opening a CAN socket; any
mismatch fails closed.

The existing utility may alternatively set the rate on the **native Ubuntu
host**:

```bash
can-config set can0 4M
ip -details -statistics link show dev can0
```

`can-config set can0 4M` takes the link DOWN, rewrites its timing, and brings it
UP. Never run it while the ROS driver, MoveIt, the motor GUI, or another process
is using that interface. It changes only the host SocketCAN controller; it does
not change motor-firmware CAN rates. The current utility also does not write
`restart-ms`, so verify `restart-ms 0` afterwards or use the complete `ip link`
sequence above.

The legacy userspace `gs_usb` backend remains fixed at 1M/5M and is incompatible
with the field chain. In the SocketCAN path the host kernel `gs_usb` driver owns
the USB-CANFD adapter. Do not simultaneously open `hex-motor-gui` in direct
`gs_usb` mode or pass the same USB device exclusively into another container.
Exit the current control program completely before changing ownership.

## Safe observation entry points

Use the following command to inspect only the offline model. It does not open
USB, load ros2_control, or enable motors:

```bash
ros2 launch hex_arm_bringup view.launch.py
```

### Read-only identity discovery on `can0`

On native Ubuntu, the regular container uses host networking and can see the
host `can0`. No USB mapping and no `HEX_ARM_REAL=1` override are required:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

With the arm mechanically secured, the physical e-stop within reach, the
output stage disabled, and the motor electronics powered sufficiently to emit
heartbeats, run this inside the built container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --discover-only --transport socket-can --interface can0 \
  --expected-node 1 --expected-node 2 --expected-node 3 \
  --expected-node 4 --expected-node 5 --expected-node 6 \
  --auxiliary-node 15 --timeout 2 --sdo-timeout-sec 0.25
```

Each `--expected-node` flag names an arm node; the flag must be repeated. If all
expected-node flags are omitted, the default is nodes 1 through 6. The field
wiring is now confirmed as a direct mapping: nodes 1 through 6 are respectively
`joint_1` through `joint_6`. Node 15 (hex `0x0f`) is the gripper motor; it is
allowlisted only as an auxiliary device and is never initialized, enabled,
disabled, or commanded by the six-axis driver or MoveIt planning group. A
configured `tip_payload` still requires its exact `0x1018` identity before any
arm drive configuration, because the attached device's mass cannot be ignored
by gravity compensation. The command fails if an expected node is missing, an
identity upload fails, or an undeclared node is present.

`--discover-only` listens for CANopen heartbeats and permits only identity SDO
upload requests. A transport-level guard rejects NMT, PDO, heartbeat production,
SDO downloads, controlwords, and shared motor commands. This branch does not
load a profile or URDF, initialize drives, start Zenoh/ROS, or clear faults. It
is read-only with respect to drive configuration, but it is not electrically
passive because SDO upload requests are transmitted.

A successful report proves only node presence and CANopen identity. The direct
node/joint mapping is confirmed from the arm configuration, but direction,
actuator zero, joint limits, torque scaling, and TCP still lack motion-based
calibration.

### Pin the host link and adapter fingerprint

`expected_link` also requires `driver=gs_usb`, USB VID:PID `1209:2323`, an exact
32-hex-character serial, and channel. Inspect them on the host with:

```bash
readlink -f /sys/class/net/can0/device/driver
udevadm info --attribute-walk --path="$(readlink -f /sys/class/net/can0/device)"
cat /sys/class/net/can0/dev_id
cat /sys/class/net/can0/dev_port 2>/dev/null || true
```

Walk upward from `device` to the nearest USB parent containing `idVendor`,
`idProduct`, and `serial`, then enter the actual values in the ignored local
profile. The currently discovered adapter has serial
`C9E29601798421B29AC2D419C12D9502` and channel/dev_id 0. Re-read these values
after replacing the adapter or interface; never copy them as universal defaults.
Preflight also requires all current and cumulative CAN/netdev error counters to
be zero. A nonzero historical RX/TX `dropped` count is accepted only if it stays
exactly unchanged across the two 0.2-second samples; a changing count is active
loss and fails the check. Diagnose the cause and re-establish a clean link
instead of bypassing that check.

### Disabled-state observation after identity verification

Copy `config/hardware/firefly_y6.example.yaml` to an ignored local profile and
review every identity and `expected_link` field. Keep
`bus.direct_joint_mapping: true`; this makes a swapped node/joint assignment a
profile-validation error while generic profiles may opt out. Once its structure and
fingerprints are reviewed it may be marked `validated: true`; disabled-state
observation is allowed while axis values remain commissioning candidates and
`calibrated: false`.

Before opening CAN, run the offline validation path:

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --validate-profile-only
```

This parses the complete profile, loads the URDF dynamics model, and rejects a
command window that approaches or crosses the unverified single-turn seam. It
does not open USB or SocketCAN, initialize a drive, start Zenoh, or require
`calibrated: true`. A successful result is necessary but does not prove motion
calibration. Run disabled-state observation from the host with the supervised
Docker entry point. The explicit interface/channel must match the profile; this
example uses the current `can2` connection:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

`activate_hardware` defaults to `false`. This keeps the Rust controller
DISABLED and does not start `ros2_control_node`, the hardware spawner, or the
trajectory controller; RViz displays feedback from the bridge directly. This
mode opens the profile-selected SocketCAN interface, performs NMT/SDO/PDO
initialization, and arms heartbeat monitoring, so it must not replace the first
`--discover-only` check. Drive
initialization never performs an automatic fault reset: if a CiA402 Fault is
present, startup fails closed and requires the physical cause to be resolved
before an explicit recovery. Restarting the process does not clear motor faults.
Here `validated` means only that the profile structure, node identities, and host
link are trusted; it does not mean motion calibration is complete. The
activation gate explicitly rejects a `calibrated: false` profile.

The host helper isolates the primary Compose exec, generates a random launch
token, and the container binds that token to only this launch's new process
group. `Ctrl-C`, SIGTERM, and terminal hangup cause a second exact `docker exec`
to validate the token/PGID file and signal that negative PGID; the helper then
continues waiting on the original Compose exec. A request arriving before the
PGID is published cancels startup, while malformed or stale state is rejected.
It reports a verified stop only when the Rust controller has reached
ready/DISABLED, entered its orderly shutdown path, and ROS launch reports that
controller exited cleanly. A clean controller return is the executable contract
that the all-axis disable and heartbeat-consumer disarm completed; failures
return nonzero and retain an audit log. It never uses a name-wide kill and does
not signal an independent `can1` process group. Keep the command attached until
the result is printed. Do not use a bare
`docker exec ... bash -lc 'ros2 launch ...'`, `pkill`, container restart, or
Compose down as the normal stop path.

Two different drive objects must not be conflated: bit 3 of `0x6041` is the
**current CiA402 Fault**, while `0x603F` is a last-error diagnostic that may be
retained after the fault has cleared. The field-observed combination
`0x603F=0x8130, 0x6041=0x0231` is currently non-Fault and non-OE. It must not be
fault-reset merely to erase history and it does not block disabled observation;
both TPDO last-error fields remain available for diagnostics.

An orderly controller exit—including success, error, SIGINT, and SIGTERM from
single-axis commissioning—first disables all six drives and requires newer
TPDO2 frames to confirm every drive is non-OE. Only after that succeeds, while
the host heartbeat is still being produced, does it write and read back
`0x1016:01=0` on each arm drive and recheck non-OE through SDO. If disable is not
confirmed, `0x1016` remains armed as a drive-side safety watchdog.

Only after the physical cause has been removed, and only when the current
`0x6041` Fault bit is set with an exact `0x8130` last-error, use the dedicated
recovery CLI. The explicit node set must exactly equal the live set of current
heartbeat-loss faults:

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --recover-heartbeat-lost 1,2,3,4,5,6 \
  --allow-heartbeat-fault-reset
```

Recovery targets are restricted to nodes 1–6; node 15 and all other auxiliary
nodes are excluded. The command verifies all six identities, the auxiliary
allowlist, and the absence of unexpected nodes, and it never initializes,
changes mode, or enables a drive. It emits an explicit `0 -> 0x80 -> 0` control
word edge. Success additionally requires a post-reset TPDO2 with non-Fault,
non-OE status, no new arm EMCY during recovery, a new all-axis disable
confirmation, and only then disarms all six host-heartbeat consumers. Do not run
this command for the non-Fault `0x0231 + 0x8130` historical condition; start
disabled observation directly.

In a second container shell, inspect the live safety gates and temperature
telemetry without enabling motion:

```bash
ros2 topic echo /diagnostics
```

The `firefly_y6/driver` status reports the operating mode, session ownership,
profile validity, calibration state, all-motors-online state, feedback freshness,
fault code, and `joint_1_temperature_c` through `joint_6_temperature_c` when the
controller supplies all six temperatures. An empty temperature vector is a
valid "not supplied" value. A nonempty vector with the wrong length or any
non-finite value raises the diagnostic level to `WARN` and is not published as
per-axis temperature data. Stop commissioning on a rising temperature, stale
feedback, nonzero fault code, or any `ERROR` status.

### Legacy `hex-ros2-arm` reuse audit

The historical `hex-ros2-arm` repository at commit `e4901be` and its sibling
MoveIt package at commit `3f3985d` are retained as commissioning evidence, not
as an executable real-hardware profile. The following items have already been
carried into the current architecture:

- the candidate joint signs `[-1,-1,+1,+1,+1,+1]`;
- the candidate motor/ROS torque factors `[0.85,0.85,0.85,1,1,1]`;
- planning group `arm`, chain `base_link` to `link_6`, KDL kinematics, and the
  `firefly_arm_controller/follow_joint_trajectory` endpoint.

They remain candidates until this physical arm passes bidirectional per-axis
tests. The old repositories contain no CANopen identities, encoder zero
records, measured travel limits, temperature policy, can2 link fingerprint, or
successful real-arm tolerance report, so they cannot justify
`calibrated: true`.

The old Python bridge is deliberately not reused: constructing it immediately
acquires a session and enters ACTIVE, it reports a FollowJointTrajectory goal
successful after elapsed time without checking path/goal tolerance, and its
best-effort destructor is weaker than the current confirmed-disable and
heartbeat-disarm path. The historical `6 rad/s`, `10 rad/s^2`, default
`20/1.5` gains, `-Z` gravity, all-zero `home`, and SRDF entries that disable all
21 collision pairs are likewise not imported. Current can2 observations,
narrow commissioning limits, strict collision checking, and per-axis tuning
take precedence.

### Current-pose, direction, and zero evidence

The approximate URDF pose recorded at the current commissioning breakpoint is:

```text
q_ref ~= [0.000, -1.570, 3.140, 0.000, 0.000, 0.000] rad
```

Combining the per-axis read-only snapshots retained at this breakpoint with the
historical direction candidate `[-1,-1,+1,+1,+1,+1]` and
`q_ros = direction * 2*pi*q_motor_rev + zero_offset_rad` gives these one-snapshot
offset candidates:

```text
zero_offset_rad ~= [-0.025651, 0.010217, 1.545484,
                     0.000144, 0.026979, 0.121463]
```

This is commissioning evidence assembled from approximate reference poses,
per-axis read-only snapshots, and an old configuration. It is not a
motion-test result and cannot justify
`calibrated: true`. Joint 2 is near its lower bound and joint 3 is near its upper
bound. The SRDF therefore provides an inset planning reference named
`commissioning_start` with joints 2/3 at -1.56/3.13 rad; never treat either array as an automatic home, execute
it on uncalibrated hardware, or interpret it as a mechanical hard stop.

### Single-turn `0x6064` and the corrected joint-2 window

The drive's `0x6064` position is canonical single-turn `[-0.5, 0.5)` rev. The
old `+1.570` joint-2 reference produced a false wrap-seam blocker. With the
corrected `-1.570` reference, `direction=-1`, and the new zero candidate, the
full URDF range maps to approximately `[-0.3310,+0.2515]` rev and stays inside
the controller's seam guard. Keep the fail-closed seam validator, but do not
describe joint 2 as a cross-turn joint based on the discarded fit.

Joint 2 still begins at its provisional URDF lower limit. The local hardware,
URDF, and MoveIt command bounds remain aligned at `-1.570`. The profile now
separates this command envelope from a read-only measured-position margin: the
local J2 margin is `0.001 rad`, enough to represent passive hard-stop
compliance and encoder scatter without authorizing a target below `-1.570`.
The controller uses that margin only to resolve/monitor feedback; compressed
MIT mapping, ROS/MoveIt limits, selected-axis commissioning targets, and all
ordinary commands remain strict. A sample outside the explicit measured
envelope still fails closed. Its first supervised J2 move may only
be a small positive out-and-back move into the allowed range. A negative move
from the boundary is prohibited, and complete bidirectional
calibration still requires a bounded one-way reposition followed by tests from
an interior pose. This remaining calibration gate, rather than a seam, keeps
real MoveIt execution closed.

1. Configure and verify the profile-selected interface (currently `can2`) at 1M/4M, then run the exact
   `--discover-only` command above. Do not use the legacy 1M/5M direct-USB path
   on this chain and do not grant the container `privileged` access.
2. Record all six CANopen node IDs plus vendor, product, revision, and serial
   identity fingerprints. Record node 15 separately as the known auxiliary
   device and bind the same exact fingerprint to `tip_payload` when its mass is
   modeled. Unexpected, duplicate, missing, or unidentified nodes stop the
   procedure.
3. Create a `*.local.yaml` profile using the confirmed node 1 -> joint_1 through
   node 6 -> joint_6 mapping and keep `bus.direct_joint_mapping: true`; put node
   15 only in `auxiliary_node_ids`. Historical
   signs, the offset candidates above, and old torque factors are commissioning
   evidence only and do not replace per-axis verification.
4. With the physical e-stop in reach, use the lowest practical torque limit to
   verify each axis independently. Confirm positive ROS motion, then record
   `direction` and restore the joint to a safe pose before moving on.
5. Record motor zero and calculate `zero_offset_rad` from
   `q_ros = direction * 2*pi*q_motor_rev + zero_offset_rad`. Treat joint 3 as a
   known special case; do not edit the URDF origin to hide an actuator zero.
6. Verify both software limits at low speed without approaching a mechanical
   hard stop. Record torque scaling from a controlled calibration, not from the
   historical GUI default.
7. Mark `validated: true` only after the profile structure, link fingerprint,
   node identities, and candidate fields have been reviewed. Mark
   `calibrated: true` only after all six axes pass direction, zero, limit, and
   torque checks and every configured tip payload has measured/reviewed inertial
   data with `inertial_calibrated: true`.
8. Run: disabled discovery, dedicated single-axis low-speed tracking, six-axis
   hold, then one short trajectory. Stop at the single-axis stage until joint 2
   has been repositioned inward and passed bidirectional calibration. Abort on
   temperature rise, stale diagnostics, noise, unexpected direction, or any
   latched fault.

The dedicated single-axis entry point is now implemented in the Rust controller
itself. It opens neither ROS nor Zenoh and accepts a `validated: true` profile
while it is still `calibrated: false`. There is no implicit/default displacement:
the joint, signed delta, total out-and-back duration, and physical-motion
acknowledgement are all mandatory. For example, after independently deciding
that `+0.01` rad is the safe direction for joint 1:

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --commission-axis joint_1 \
  --delta-rad 0.01 \
  --duration-sec 2.0 \
  --allow-motion
```

`--allow-motion` is not a dry-run flag: its presence authorizes the selected
physical motor to enable and move. The command rejects `abs(delta) > 0.03` rad,
uses a smooth start -> delta -> start profile, and requires both
`duration >= pi*abs(delta)/velocity_limit` and
`duration >= sqrt(2*pi^2*abs(delta)/acceleration_limit)`. The current
commissioning hardware profiles use `acceleration_rad_s2: 0.1`; both velocity
and acceleration are therefore enforced before a drive can be enabled. It first preflights the interface selected by the profile (currently `can2`), verifies all identities and the auxiliary
allowlist, initializes and confirms all six drives disabled, then configures and
enables only the selected node. All six shared-frame slots are populated with
feedback-derived holds before enable; the selected drive uses capped low gains
and URDF gravity feed-forward. Feedback freshness, faults, position, velocity,
torque, tracking, and transport health are checked throughout. Completion,
Ctrl-C, or any error converges on a retried, feedback-confirmed all-axis disable;
an unconfirmed disable makes the command fail. Run one joint at a time and choose
the delta sign from the supervised physical setup, not from this example. A run
is not accepted merely because the axis finishes near its start: the measured
signed peak must move in the requested direction and reach at least 50% of the
requested delta, may never exceed `abs(delta)+0.005` rad, and must return within
the tighter of 25% of the delta or 0.005 rad (with a 0.001 rad floor). The
successful log records `measured_peak_delta_rad` for the direction record.

The local field profile is the authoritative per-axis record; do not copy gains
or torque limits from prose. At this checkpoint J1..J6 Kp/Kd are respectively
`60/2.5`, `80/2.5`, `2/0.3`, `30/1`, `30/1`, and `20/1`, all with
`velocity_rad_s <= 0.1`; J3 alone currently has `torque_permille=250`, while the
others use 200 and all axes use `kp_kd_torque_permille=100`. With the trial GR80
payload and the corrected -Z base gravity candidate, the surveyed-pose unscaled
model is approximately `[0,+1.865,-5.957,-1.157,0,0] Nm`. This is a model audit,
not measured holding torque. The corrected 25% J2 trial reached only 0.000534
rad of a +0.005 rad request; a 50% trial still reached only 0.000810 rad while
measured torque peaked at 1.264 Nm. Blind escalation stopped. J2/J3/J4 now
remain at `0.25/0/0`; these values are
commissioning candidates, not certified safe values. Empty bridge
`kp`/`kd` vectors select the Rust per-axis defaults. These gains are calibrated
joint-side SI values in `Nm/rad` and `Nm*s/rad`. An empty bridge `tau_ff`
vector delegates automatic gravity feedforward to Rust using the URDF, measured
joint positions, gravity vector, and the fixed `tip_payload` mass/COM merged into
`link_6`. An empty vector means "delegate to Rust"; six explicit zeros disable
that automatic feedforward. Any non-empty external `tau_ff` similarly bypasses
the payload-aware gravity model and `gravity_compensation_scale`. The per-axis
`torque_scale` remains active for every torque-producing MIT term: joint-side
`tau_ff`, Kp, and Kd are multiplied into the motor torque domain, while measured
motor torque is divided back into the calibrated joint-side domain. The
ROS/MoveIt bridge uses the empty vector and therefore follows the automatic
payload-aware path.

The dedicated J2 diagnostic on 2026-08-20 added a narrower command-path result.
With an unchanged position target, both the `0.025..0.250 Nm` additive shared-
RPDO target and the restored baseline matched bit-for-bit through drive uploads
of `0x2004:02/03`. Peak total feed-forward was about `0.716 Nm`, measured torque
was about `0.706 Nm`, excursion from that run's start was only `+0.000513 rad`,
and peak velocity was `0.00143 rad/s`; CAN and temperature gates remained
clean. This proves the drive consumed the command and that this torque remained
below the static load. It is not a passed J2 motion, gravity-scale, or physical-
limit calibration.

A separately authorized high-tier diagnostic then used `0.10 Nm` increments.
It stopped automatically at `1.00 Nm` additive (`1.466 Nm` total feed-forward)
when the encoder reached `+0.001002 rad`; peak velocity was `0.00618 rad/s` and
measured torque was `1.412 Nm`. The tool did not send the remaining levels up
to the requested `1.50 Nm`: it restored the baseline immediately, confirmed J2
non-torque first, confirmed the other five axes, and only then disarmed all
heartbeat consumers. The passive post-run position was about `-1.568797 rad`,
still inside the commissioning window. For this pose, the useful breakaway
bracket is therefore `0.90..1.00 Nm` additive, not a reason to rerun `1.50` or
try `1.75 Nm`. This proves a small positive J2 response and rules out a missing
software brake-release command; it still does not pass position tracking,
gravity-scale, zero-reference, or physical-limit calibration.
This one run used a process-local exact acknowledgement of the current can2
historical counters (`error_warning=10`, `error_passive=10`): all other counters
were zero and the complete runtime baseline had zero increment or reset. The
acknowledgement is not a profile setting; normal control and ordinary
commissioning still require absolute-zero historical CAN error counters.
A subsequent fixed `+0.005 rad / 4 s` position diagnostic tested J2 scale
`0.65`, but aborted before the trajectory began. Its first post-enable sample
had moved `+0.000936 rad` at `+0.03004 rad/s`, above the unchanged
`0.020 rad/s` hard velocity gate. The tool restored the baseline, confirmed all
six axes disabled, disarmed every heartbeat consumer, and exited nonzero; CAN
and USB post-checks remained clean. The local profile was therefore returned to
the previously stable `0.25`. Do not rerun the `0.65` enable step or loosen the
velocity gate. Any next experiment must separately review a low-feed-forward
enable followed by a smooth, bounded in-trajectory feed-forward ramp.
The retained raw-log metadata and abort sample are in
[the J2 position-abort evidence note](commissioning_evidence/2026-08-20-can2-j2-position-abort.md).
The reviewed follow-up removed that enable-time step: J2 enabled at scale
`0.25`, remained stable, and completed a feedback-proven 375-step / 1.5 s
fixed-position ramp to `0.65`. The ramp itself stayed within `0.000712 rad` and
`0.001149 rad/s`. At the final scale the axis was nearly stationary, but its
mandatory second stability gate settled at `+0.000735 rad`, above the unchanged
`0.000500 rad` position tolerance. The controller therefore restored its
registered baseline and shut down before publishing any part of the requested
5 mrad trajectory or performing a changed-position readback. CAN/USB checks
again remained clean. This validates the ramp mechanism, not J2 tracking or
gravity calibration; the local profile remains `0.25`, and `0.65` must not be
replayed or admitted merely by loosening the stability gate. The raw artifact
and exact telemetry are recorded in
[the J2 gravity-ramp stability-gate note](commissioning_evidence/2026-08-20-can2-j2-gravity-ramp-stability-gate.md).
One final, separately authorized fixed-position identification then started at
the retained `0.25` profile value and used feedback-censored microsteps, with no
position-trajectory branch. Displacement first crossed `+0.300 mrad` at scale
`0.363305`; that target was rejected, and the previous feedback-proven,
quantized-distinct target at scale `0.362088` matched exact `0x2004:02/03`
readback. During its one-second frozen hold, mean displacement was
`+0.326 mrad`; the final 250 ms spanned only `0.0044 mrad` with peak velocity
`0.000212 rad/s`. All six drives then disabled cleanly, and CAN/USB counters
remained clean. This is a path-dependent censor point, not a calibrated gravity
scale: hard-limit preload, static friction, and hysteresis remain inseparable
from one monotonic upsweep. J2 therefore remains frozen at profile scale
`0.25`; do not promote `0.362088` or replay this experiment. The complete
artifact is in
[the J2 censored-hold evidence note](commissioning_evidence/2026-08-20-can2-j2-censored-gravity-hold.md).
J1 was then exercised once with the independently fixed `+0.005 rad / 4 s`
first-position survey. Its low `30/20` drive permissions, strict OE proof, and
both changed/returned compressed-target readbacks passed; cleanup also
confirmed all six drives disabled. Tracking did not pass: the positive peak
was only `0.064 mrad` against the `2.5 mrad` minimum, while the commanded-peak
measured/estimated torque was `0.260/0.297 Nm`. CAN/USB and temperature gates
remained clean. Do not replay the position survey or enlarge its distance;
use a separately authorized fixed-position, feedback-censored torque
identification before considering another J1 position attempt. See
[the J1 first-position evidence note](commissioning_evidence/2026-08-20-can2-j1-first-position.md).
A single separately authorized fixed-position, positive-torque censored
identification then held the J1 position code constant and increased only
joint-side feed-forward in `0.025 Nm` levels. The `0.450 Nm` level first crossed
the `+0.300 mrad` censor, so it was rejected and the controller restored the
preceding feedback-proven `0.425 Nm` target. Exact `0x2004:02/03` readback
matched. During the bounded one-second frozen hold, mean displacement was
`+0.308718 mrad`; the final 250 ms spanned only `0.002436 mrad` with peak
velocity `0.000226 rad/s`. All six drives were then confirmed non-torque and
can2/USB remained clean. This is a path-dependent positive-response bracket,
not a reusable bias or calibration. Do not write `0.425 Nm` into the profile,
replay the upsweep, or use it to pass J1 position tracking. J1 remains frozen
pending a separately reviewed negative-direction fixed-position
identification. See
[the J1 censored-torque evidence note](commissioning_evidence/2026-08-20-can2-j1-censored-torque.md).
The reconstructed artifact and hardware record is stored in
[the 2026-08-20 J2 evidence note](commissioning_evidence/2026-08-20-can2-j2-breakaway.md).

Hardware-profile schema v2 requires `gravity_vector_base_m_s2: [gx, gy, gz]`.
The vector is acceleration in the URDF `base_link` coordinate frame, not a
world-frame label or a per-joint sign, and its magnitude must be within
8..12 m/s^2. Schema v1 and schema-v2 files that omit this field are rejected;
migrate an old profile by reviewing the physical base mounting, adding the
explicit vector, and changing `schema_version` to `2`. There is deliberately no
implicit `-Z` fallback. The current local field profile records
`[0.0, 0.0, -9.81]` as a corrected commissioning candidate and remains
`calibrated: false` until supervised holds and both motion signs confirm it.
Single-axis commissioning and normal ROS/MoveIt runtime both compute `G(q)`
from this same vector. `SetGravity` may temporarily override it for the current
exclusive session only; release, shutdown, and every new acquire restore the
profile value, and the service never edits the YAML profile.

Each joint's `gravity_compensation_scale` multiplies only that axis's URDF
gravity estimate in both this single-axis commissioning path and the normal
Rust runtime used by ROS/MoveIt. The accepted range is `0.0..=2.0`: `0.0`
disables model gravity feed-forward for that axis, `1.0` uses the unscaled URDF
estimate, and values above `1.0` increase it. Profiles that predate this field
default to `1.0`, but field profiles should record it explicitly. Treat it as a
measured real-arm identification parameter for the actual payload and mounting,
not as a generic tuning knob. It is distinct from `torque_scale`, which converts
all torque-dimensioned MIT terms between the calibrated joint and motor domains:
feed-forward, Kp, and Kd are scaled on output, and measured torque is inversely
scaled on input. Changing one must never be used to hide an error in the other.
It must not be used to flip or repair the base gravity
direction: first choose `gravity_vector_base_m_s2`, compute `G(q)`, and only
then apply this per-axis scalar. Re-run supervised hold and both signed
small-motion tests after changing the gravity scale, and keep
`calibrated: false` until every axis is verified.

### Real-hardware MoveIt entry point

The real MoveIt launch defaults to observation/planning only and always uses the
0.1 rad/s, 0.1 rad/s^2 limits and narrow surveyed commissioning position windows:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

The real/mock ros2_control configuration now fails closed with a 0.02 rad
per-joint path tolerance, 0.005 rad goal tolerance, 0.02 rad/s stopped-velocity
tolerance, and a 1.0 s goal-time allowance. In particular, a 0.016 rad final
joint-2 tracking error is a failed goal, not a successful execution. These
tolerances only detect tracking failure; they do not resolve the current
joint-2 undertracking or open the real-execution gate.

Its default `enable_execution:=false` keeps both `activate_hardware=false` and
`allow_trajectory_execution=false`. Only after per-axis calibration has set
`calibrated: true` and corrected joint-2 bidirectional tracking is complete may it
be explicitly opened:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  enable_execution:=true
```

The launch configuration and mock regression have been tested offline. This
document does not claim a real motor-motion or real MoveIt-trajectory test. The
local GR80 trial payload records 0.41 kg and its combined COM in the identity
`link_6` mount, with source URDF SHA-256
`f74b3e76b14175c788c5ef70dd0c1941958d229a8461da6f20e17543e4ba1114`.
Those values are explicit trial placeholders, not measurements;
`inertial_calibrated: false` prevents a calibrated profile and real MoveIt
execution. `link_6` is still only the planning flange tip; the real TCP,
calibrated tool inertials, gripper transform, and final tool collision geometry
remain outstanding.

Only after those checks are complete and hardware enable is explicitly intended
may the activation gate be opened:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  activate_hardware:=true
```

Fault recovery never auto-enables the arm. After every fault, inspect and
remove the physical cause before any explicit reset; initialization and process
restart never reset it. Activation remains an explicit ros2_control lifecycle
operation.
