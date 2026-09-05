# J2 breakaway diagnostic — 2026-08-20

Status: reconstructed summary, not a raw transcript.

The complete terminal output from this run was not redirected to a file. This
record was reconstructed immediately afterwards from the terminal/session
output. Values that were not retained are identified instead of being guessed.
Future hardware runs must save stdout/stderr before motion and record its
SHA-256.

## Artifact and hardware identity

- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993` (dirty worktree; the
  installed binary hash below is authoritative for this run).
- Installed controller SHA-256:
  `3a0dbd071a3a741d0254137b21c8089d2d6173824342479a29838e1f3eb8de13`.
- Arm URDF xacro SHA-256:
  `47ad07393d1a6ecdcefb23158aa38312737ad0c56dd219dab9bc880f3c529cf9`.
- GR80 trial-payload source SHA-256:
  `f74b3e76b14175c788c5ef70dd0c1941958d229a8461da6f20e17543e4ba1114`.
- Post-run annotated local-profile SHA-256:
  `4890097b2c493d220d60ddd3098f19c0e7aa05a1ade990a31892c5de96068f35`.
  Only comments were added after the run; the parsed settings were unchanged.
- Interface: `can2`, ifindex `27`, HexMeow Quad CAN-FD channel `2`, USB serial
  `C9E29601798421B29AC2D419C12D9502`.
- Selected drive: node `2`, joint `joint_2`, vendor/product/revision/serial
  `1213809740/2863267842/9/622266383`.
- Run date/timezone: `2026-08-20`, Asia/Shanghai. The exact wall-clock time was
  not retained.

## Command

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  --diagnose-axis joint_2 \
  --diagnostic-mode tau-ff-staircase \
  --diagnostic-peak-tau-ff-nm 1.50 \
  --diagnostic-step-tau-ff-nm 0.10 \
  --diagnostic-dwell-sec 0.20 \
  --allow-diagnostic-motion \
  --allow-high-torque-diagnostic \
  --diagnostic-can-error-warning-baseline 10 \
  --diagnostic-can-error-passive-baseline 10 \
  --acknowledge-historical-can-xstats
```

The historical-xstats acknowledgement was an explicit, process-local exception
for this bounded J2 high-tier diagnostic. Both preflight samples reported
`error_warning=10` and `error_passive=10`; all other CAN xstats and netdev
error/drop counters were zero, the interface was ERROR-ACTIVE, and TEC/REC were
zero. All 13 runtime counters remained exactly equal to the accepted baseline,
and the post-run sample was still `10/10` with no other errors. This exception
is not stored in the hardware profile. Normal ROS, MoveIt, heartbeat recovery,
and ordinary commissioning continue to require absolute-zero historical CAN
error counters.

## Relevant settings

- J2 direction/zero: `-1` / `0.01021705 rad`.
- J2 position window: `[-1.570, -1.300] rad`.
- J2 `Kp/Kd`: `80.0/2.5`.
- Base gravity vector: `[0, 0, -9.81] m/s^2`.
- J2 gravity scale / torque scale: `0.25/0.85`.
- Model gravity feed-forward at the run pose: about `0.465825 Nm`.
- Diagnostic limits included: positive breakaway `1 mrad` or `0.01 rad/s`,
  reverse abort `0.5 mrad`, hard excursion `2 mrad`, hard speed `0.02 rad/s`,
  command/measured/estimated total torque `2.5 Nm`, both temperatures `70 C`
  and `+2 C`, and a four-second high-tier active deadline.

## Observations

- J2 reached strict MIT Operation Enabled: status `0x0237`, mode display `5`,
  control word `0x000F`. The other five axes remained confirmed non-torque.
- Enable stability started near `-1.56900048 rad`; deviation was
  `0.00020337 rad`, with peak stability velocity `0.00476584 rad/s`.
- The first changed shared-RPDO command was proven by exact drive uploads of
  `0x2004:02/03`: expected and actual lower/upper words were
  `0xFF7FF73E/0xFF0C7FF7`.
- The complete `0.90 Nm` additive level did not reach the breakaway gate:
  total feed-forward was about `1.365825 Nm`, displacement `0.000935912 rad`,
  and measured torque `1.300765 Nm`.
- At `1.00 Nm` additive (`1.465825 Nm` total feed-forward), J2 reached
  `-1.56799829 rad`, a positive displacement of `0.001002192 rad`, and the
  diagnostic stopped immediately. Velocity at the trigger was
  `0.004421875 rad/s`; measured torque was `1.41225886 Nm`; estimated total
  torque was `1.35849655 Nm`.
- Overall peaks were: displacement `0.001002192 rad`, velocity
  `0.006178515 rad/s`, measured torque `1.4122589 Nm`, estimated total torque
  `1.3777409 Nm`, driver temperature `33.2 C`, and motor temperature `30.8 C`.
  The exact cold temperature baseline was not retained. Elapsed active time was
  about `2.1004 s`.
- Levels above `1.00 Nm` were not sent. The shared target was immediately
  restored to the registered baseline, J2 was confirmed non-torque before the
  other axes, all six heartbeat consumers were disarmed, and the process exited
  successfully.
- Twenty passive post-run J2 TPDO1 samples were identical:
  raw bytes `6EAB803E`, raw position `0.251307904720 rev`, corresponding to
  `q=-1.5687970845 rad` (about `+1.203 mrad` inside the software lower bound).

## Conclusion and limits

For this pose and configuration, useful static breakaway is bracketed between
`0.90` and `1.00 Nm` additive (`1.366..1.466 Nm` total feed-forward). The command
path, RPDO consumption, strict OE state, small positive encoder response, CAN
health delta, and orderly shutdown were all demonstrated. No separate software
mechanical-brake release command is missing; the user's “mechanical brake” was
clarified as the physical travel stop.

This run does **not** calibrate J2 position tracking, gravity scale, zero
reference, physical travel limits, payload inertia, or MoveIt execution. Do not
repeat the `1.50 Nm` request to force a larger visible motion and do not try the
unused `1.75 Nm` tier. The next useful experiment is a separately reviewed,
small positive position round trip with a lower static holding torque, not a
higher torque staircase.
