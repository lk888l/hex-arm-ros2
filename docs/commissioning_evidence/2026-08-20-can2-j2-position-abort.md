# J2 position diagnostic safety abort — 2026-08-20

Status: raw transcript retained; trajectory never started.

## Artifacts and command

- Run time: `2026-08-20 12:35:21 +08:00`.
- Raw transcript:
  `log/commissioning/2026-08-20T1235+0800_can2_j2_position_round_trip.log`
  (gitignored local artifact).
- Raw transcript SHA-256:
  `cb6e12f90a2e9aaaf855bd76fa7e25d44be277f9bf87f14bd475a46efb91f008`.
- Installed controller SHA-256:
  `3a7e77d98c72c77d588f53e46583197f71b52a5ce2b47c11ceec52e5c4a5df8b`.
- Run-profile SHA-256 (J2 gravity scale `0.65`):
  `c19b72378e46917342bc3300a96f5cf27047c91ebca19d0d7539f4d5df1f0559`.
- Arm URDF xacro SHA-256:
  `47ad07393d1a6ecdcefb23158aa38312737ad0c56dd219dab9bc880f3c529cf9`.
- USB/controller identity: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, channel `2`, `can2`, USB device `11`,
  ifindex `35`; node 2 identity is recorded in the local profile.

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  --diagnose-axis joint_2 \
  --diagnostic-mode position-round-trip \
  --diagnostic-delta-rad 0.005 \
  --diagnostic-duration-sec 4.0 \
  --allow-diagnostic-motion \
  --allow-high-torque-diagnostic \
  --diagnostic-can-error-warning-baseline 0 \
  --diagnostic-can-error-passive-baseline 0 \
  --acknowledge-historical-can-xstats
```

The shared USB adapter had been stable for more than 30 minutes after its last
can1-triggered re-enumeration. The driver's own double-sample preflight passed
with ERROR-ACTIVE, TEC/REC zero, every CAN xstat and netdev error/drop counter
zero, and the expected adapter fingerprint. The acknowledgement arguments were
still mandatory for this one diagnostic mode, even though the exact baseline
was `0/0`.

## Safety abort

- Six drives initialized disabled; node 15 remained auxiliary-only.
- Node 2 reached strict MIT OE: status word `0x0237`, mode display `5`, control
  word readback `0x000F`.
- The requested 5 mrad position trajectory had **not** begun and no changed
  position target had been read back.
- Initial/commanded hold position: `-1.5689135789871216 rad`.
- First post-enable safety sample:
  - measured position `-1.5679776668548584 rad`;
  - displacement `+0.0009359121322631836 rad`;
  - velocity `+0.03004159778356552 rad/s`;
  - measured torque `1.114941120147705 Nm`;
  - model feed-forward `1.2103246450424194 Nm`;
  - estimated PD torque `-0.17644169926643372 Nm`;
  - estimated total `1.033882975578308 Nm`;
  - raw motor position/velocity/torque
    `0.25117748975753784 rev / -0.004781268537044525 rev/s / -0.947700023651123 Nm`.
- The unchanged hard velocity limit was `0.020 rad/s`. The observed
  `0.03004 rad/s` therefore caused an immediate safety abort before stability
  dwell or trajectory publication.
- The persistent baseline was republished, all six drives were confirmed
  disabled, all heartbeat consumers were disarmed, and the process exited `1`.

## Post-run gain-unit audit

The binary used for this historical run predated the correction that applies
`torque_scale` to compressed-MIT Kp and Kd as well as to feed-forward torque.
With J2 `torque_scale=0.85`, the profile values `80.0/2.5` therefore produced
effective calibrated joint-side gains of approximately `94.12 Nm/rad` and
`2.94 Nm*s/rad` during this run. The logged
`estimated PD torque=-0.1764417 Nm` was calculated from the actual motor-side
target and converted back through the same torque calibration, so it remains a
valid estimate of the command that was physically active. Under the corrected
SI semantics, the same `80.0/2.5` profile values and the recorded position and
velocity errors would instead give approximately `-0.14998 Nm` of PD torque.
This audit does not alter the recorded feed-forward or measured-torque values,
whose `torque_scale` conversion was already active, and it is not evidence that
the corrected gains have been exercised on hardware.

## Post-run checks

- Same USB device `11` and can2 ifindex `35`.
- ERROR-ACTIVE, TEC/REC zero; all CAN xstats and netdev errors/drops remained
  zero after `998` TX packets / `19188` TX bytes.
- No new `gs_usb`, `-EPROTO`, disconnect, reset, or re-enumeration log entry.
- No remaining can2 socket or controller process; the unrelated can1 socket
  was not touched.
- A two-second passive node-2 TPDO1 capture showed a stable position word
  `26AB803E`: `0.251305758953 rev`, or `q=-1.568783602264 rad`, about
  `1.216 mrad` inside the `-1.570 rad` lower limit.

## Decision

This is a correct safety abort, not a failed 5 mrad tracking result. The
`0.65` gravity scale introduced approximately `1.21 Nm` as an enable-time
feed-forward step; the position trajectory never had a chance to run. The
profile was returned to the previously stable `0.25` candidate.

Do not rerun the same `0.65` enable step and do not loosen the velocity gate.
Any next experiment must independently review a low-feed-forward enable,
followed only after stability by a smooth, bounded feed-forward ramp. The gain
unit interaction has now been audited above, but the corrected gain path still
requires a new supervised validation before another J2 motion attempt.
