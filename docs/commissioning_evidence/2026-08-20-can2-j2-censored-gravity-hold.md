# J2 censored fixed-position gravity hold — 2026-08-20

Status: completed successfully; the displacement censor selected and proved a
lower, quantized-distinct fixed-position target. No position trajectory exists
in this diagnostic mode.

## Artifacts and command

- Run start: `2026-08-20 14:16:26 +08:00`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993` with an intentionally dirty
  commissioning worktree; the installed binary hash below is authoritative.
- Raw transcript:
  `log/commissioning/2026-08-20T141559+0800_can2_j2_censored_gravity_hold.log`
  (gitignored local artifact, `17189` bytes).
- Raw transcript SHA-256:
  `72fbc781f2dda49e058bd55f2122b9b0a93601471cb29fe3872b7096f7cc5d04`.
- Installed and build controller SHA-256:
  `6423e5b266e928488689e15f7ce205d3fcbe5ed58682d8cee6945f2dc8908322`.
- Run-profile SHA-256:
  `00a731b0809e4175460cae256dac9d283e2750edc408e2c45c39770cc936f1ac`.
- Controller source SHA-256 values:
  - `commissioning.rs`: `001162cbeec883e797d036d3545df42185ed82b5843c8b8c2b7777f0cb49b6b3`;
  - `main.rs`: `454c1dc3c827a302b73bcc22cc2491eca526372b22631bd73c2ec8c7d42dd9cc`;
  - `backend.rs`: `85b495619f269bc7d0c9d7fe73a567458c6526a990951e3534c7581e164e716f`.
- Adapter: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, `can2`, channel `2`, ifindex `35`,
  USB device `11`.
- J2 run parameters were locked by code and profile: gravity scale `0.25`,
  true joint-side Kp/Kd `80.0/2.5`, torque scale `0.85`, drive torque/PD
  permissions `200/100` permille, fixed position target, scale cap `0.55`,
  `375` half-cosine transitions, and a nominal `1.5 s` ramp.

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  --diagnose-axis joint_2 \
  --diagnostic-mode gravity-hold-censored \
  --allow-diagnostic-motion \
  --allow-high-torque-diagnostic \
  --acknowledge-censored-gravity-hold \
  --diagnostic-can-error-warning-baseline 0 \
  --diagnostic-can-error-passive-baseline 0 \
  --acknowledge-historical-can-xstats
```

The process-local xstats acknowledgement was required by the fixed high-tier
diagnostic even though the exact warning/passive baseline was `0/0`. The
fail-closed preflight also required ERROR-ACTIVE, TEC/REC zero, every other CAN
xstat and netdev error/drop counter zero, and the exact adapter fingerprint.
It did not relax ordinary commissioning or runtime preflight.

An unrelated controller remained on `can1`. Read-only host evidence bound its
only CAN_RAW socket to `can1`, while `can2` had no owner. Both channels share
the same USB device, so the run retained the USB-epoch and kernel-error gates;
the user supervised the other mechanism and the physical emergency stop.

## Observed sequence

- All six arm nodes initialized disabled; node 15 remained auxiliary-only.
- Node 2 reached strict compressed-MIT Operation Enabled with status word
  `0x0237`, mode display `5`, and control-word readback `0x000F`.
- The fixed `0.25` gravity-scale target passed the continuous enable-stability
  gate after `0.254222 s`:
  - position deviation `+0.000184894 rad`;
  - final velocity approximately zero;
  - peak velocity `0.004705160 rad/s`, below the unchanged `0.005 rad/s`
    identification limit.
- The target position code remained fixed. Feed-forward alone rose through
  half-cosine microsteps. Displacement milestones were observed before target
  promotion:

| Event | Step | Gravity scale | Feed-forward (Nm) | Position delta (rad) | Velocity (rad/s) | Measured torque (Nm) | Estimated total (Nm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline already above 0.1 mrad | 0 | 0.250000 | 0.465778 | 0.000184894 | ~0 | 0.371647 | 0.450977 |
| crossed 0.2 mrad | 96 | 0.295952 | 0.551391 | 0.000201106 | 0.000290646 | 0.520306 | 0.534576 |
| 0.3 mrad censor | 158 | 0.363305 | 0.676878 | 0.000301480 | 0.000439446 | 0.594635 | 0.651661 |

- The step-158 target that elicited the censor was not promoted. The controller
  selected the previous feedback-verified target at step 157, gravity scale
  `0.362088`, whose compressed words differed from both the rejected target
  and the configured `0.25` target.
- The frozen target's exact `0x2004:02/03` readback matched bit-for-bit:
  lower `0xFF7FF719`, upper `0xFEF57FF7`.
- During the bounded one-second frozen hold (`194` fresh samples):
  - mean/minimum/maximum position delta were
    `0.000326205 / 0.000301480 / 0.000335455 rad`;
  - position standard deviation was `0.000006830 rad`;
  - peak velocity was `0.000290704 rad/s`;
  - mean measured and estimated-total torque were
    `0.633524 / 0.648434 Nm`;
  - peak driver/motor temperatures were `32.1/29.6 degC`;
  - the final 250 ms contained `50` samples, position span
    `0.000004411 rad`, and peak velocity `0.000211553 rad/s`.
- The terminal-stability result was true. Across the entire diagnostic, peak
  position delta, velocity, measured torque, and estimated-total torque were
  `0.000335455 rad`, `0.000939497 rad/s`, `0.706129 Nm`, and `0.651661 Nm`.
  The torque-capable interval ended after `2.050279 s`, below its `4 s` cap.

## Cleanup and post-run checks

- The persistent frozen fixed-position baseline was republished before the
  selected-first shutdown path. The reviewed implementation disables J2 first,
  then attempts the remaining five axes; the transcript subsequently confirms
  every drive disabled and every `0x1016` heartbeat consumer disarmed.
- The command exited `0` with `commissioning completed and all drives are
  confirmed disabled`.
- Post-run can2 remained the same ifindex/USB epoch, ERROR-ACTIVE with TEC/REC
  `0/0`. All CAN xstats and netdev error/drop counters remained zero.
- TX rose from `9214 packets / 289111 bytes` to
  `12307 packets / 407322 bytes`, then remained unchanged in a second sample.
- There was no new `gs_usb`, `-EPROTO`, disconnect, reset, or re-enumeration
  entry, no remaining can2 socket, and no diagnostic/controller process.
  The unrelated can1 controller and socket were not modified.

## Decision

This run identifies a reproducible **censored upsweep point for this start
pose and this approach history**: displacement first crossed `+0.3 mrad` at
scale `0.363305`, and the prior quantized-distinct scale `0.362088` then held
stably for one second. It validates the fixed-position RPDO path, feedback-
driven censor, quantized rollback, exact readback, bounded statistics, and
confirmed cleanup.

It does **not** calibrate J2 gravity scale, position tracking, zero reference,
mechanical limits, or a reversible static-friction model. In particular,
`0.362088` is not promoted into the hardware profile: a single monotonic
upsweep cannot separate gravity, hard-limit preload, static friction, and
hysteresis. The local profile remains at the previously stable fail-closed
value `0.25` with `calibrated: false`. Do not replay this identification or
resume the rejected `0.65` position diagnostic automatically. J2 is frozen at
this evidence point while first-motion commissioning proceeds on J1.
