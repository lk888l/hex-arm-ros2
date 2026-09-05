# J2 gravity-ramp stability-gate stop — 2026-08-20

Status: raw transcript retained; feed-forward ramp passed; position trajectory
never started.

## Artifacts and command

- Run start: `2026-08-20 13:18:15 +08:00`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993` with an intentionally dirty
  commissioning worktree; the installed binary hash below is authoritative.
- Raw transcript:
  `log/commissioning/2026-08-20T1318+0800_can2_j2_position_ramp.log`
  (gitignored local artifact).
- Raw transcript SHA-256:
  `7bfed6473c2766d0a933240528e79d175a14cec4d12510edacc63e0ac3af6b88`.
- Installed and build controller SHA-256:
  `3de9b78d632c47ba867a946907edf9bb4bedfd7771e9030779e3b05061e04c7e`.
- One-run profile SHA-256 while J2 gravity scale was `0.65`:
  `74957d87977b12365ab1b8ffe9c8f6a8450ad58f7c1abcbf075f9a001e17b2f8`.
- Restored local profile SHA-256 immediately after reverting the run setting
  to J2 gravity scale `0.25` (before appending evidence-only YAML comments):
  `306ade31fd72263c1284b9a39d089198c4b41af2299da2be7d68e22e07fea1d5`.
- Arm URDF xacro SHA-256:
  `47ad07393d1a6ecdcefb23158aa38312737ad0c56dd219dab9bc880f3c529cf9`.
- Adapter: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, `can2`, channel `2`, ifindex `35`,
  USB device `11`.

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

The driver preflight passed with ERROR-ACTIVE, TEC/REC zero, exact
warning/passive baseline `0/0`, all other CAN xstats and netdev errors/drops
zero, and the expected adapter fingerprint.

## Observed sequence

- All six arm nodes initialized disabled; node 15 remained auxiliary-only.
- Node 2 reached strict compressed-MIT Operation Enabled: status word `0x0237`,
  mode display `5`, control-word readback `0x000F`.
- The pre-enable/configuration target used gravity scale `0.25`. Its first
  stability gate passed after `0.254240 s`:
  - position deviation `+0.000152707 rad`;
  - current velocity approximately zero;
  - peak velocity `0.000211574 rad/s`.
- The feedback-driven half-cosine ramp completed all `375` transitions from
  gravity scale `0.25` to `0.65`. The logged checkpoints were:

| Ramp point | Scale | Feed-forward (Nm) | Position delta (rad) | Velocity (rad/s) | Measured torque (Nm) | Estimated total (Nm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| quarter | 0.308875 | 0.575476 | 0.000185013 | 0.000092621 | 0.557471 | 0.560444 |
| half | 0.450838 | 0.839972 | 0.000415325 | 0.000716791 | 0.743294 | 0.804953 |
| three-quarter | 0.592307 | 1.103548 | 0.000629902 | 0.000181841 | 1.003447 | 1.052700 |

- The ramp itself passed its motion gates with final feed-forward
  `1.211038 Nm`, peak displacement `0.000712276 rad`, and peak velocity
  `0.001148877 rad/s`. Driver/motor temperatures at the ramp checkpoints were
  `32.4/29.9 degC`; no thermal, torque, feedback, drive-state, or transport
  guard fired.
- With the final `0.65` target held for the second stability gate, J2 settled
  rather than continuing to move, but its position offset was too large:
  - commanded position `-1.568890691 rad`;
  - measured position `-1.568155527 rad`;
  - deviation `+0.000735164 rad`;
  - velocity approximately zero;
  - peak velocity during this gate `0.000243160 rad/s`;
  - measured torque `1.189271 Nm`;
  - feed-forward `1.210401 Nm`;
  - true joint-side `Kp/Kd = 80.0/2.5` under the corrected SI conversion;
  - estimated PD torque `-0.058813 Nm` and estimated total `1.151589 Nm`.
- The required position stability bound was `0.000500 rad` continuously for
  `0.250 s`. After the `3.000 s` deadline the controller therefore stopped
  before publishing any 5 mrad trajectory target or performing its changed-
  position readback.

## Cleanup and post-run checks

- The registered zero-additive diagnostic baseline was restored through the
  shared RPDO sender.
- Every arm drive was confirmed disabled and every `0x1016` heartbeat consumer
  was disarmed; the process exited `1`, as required for a rejected diagnostic.
- Post-run can2 remained the same ifindex/USB epoch, ERROR-ACTIVE with TEC/REC
  `0/0`. All six CAN xstats and all netdev error/drop counters remained zero.
- TX stopped at `7271 packets / 285231 bytes`; subsequent samples showed no
  additional TX.
- There was no new `gs_usb`, `-EPROTO`, disconnect, reset, or re-enumeration
  entry and no remaining can2 socket or controller process. The unrelated can1
  socket was not touched.

## Decision

This run validates the low-feed-forward enable, the 1.5-second feedback-driven
ramp, the corrected physical `80/2.5` gain semantics, and the fail-closed
cleanup path. It does **not** validate the requested 5 mrad trajectory, J2
tracking, gravity scale, zero reference, or travel limits because that
trajectory never started.

The `0.65` value is not retained as a calibrated profile setting; the local
profile was restored to `0.25` and still has `calibrated: false`. Do not replay
this command automatically and do not simply relax the 0.5 mrad admission
gate. The measured, nearly static `0.735 mrad` offset must first be used to
design and independently review the next bounded experiment.
