# J1 phase-compensated +5 mrad position surveys — 2026-08-20

Status: PARTIAL PASS for positive J1 direction/target consumption and bounded
motion. NO-GO for declaring a complete round trip, changing the persistent J1
profile, normal ROS control, or MoveIt execution.

All three trials used the fixed `joint1-first-position` CLI policy: J1/node 1
only, `+0.005 rad` start-to-peak-to-start in `4.0 s`, temporary `30/20` drive
permissions, strict-zero can2 preflight, six-axis fresh/state/dual-temperature
checks, dynamic position window `[-0.5,+6.0] mrad`, velocity hard limit
`0.020 rad/s`, measured/estimated total-torque limit `1.0 Nm`, exact changed
target readback, persistent baseline restoration, and J1-first confirmed
shutdown. The profile remained `calibrated:false` with persistent J1
Kp/Kd `60/2.5`; trajectory gains and friction envelopes were temporary.

The complete terminal outputs were captured by the commissioning session but
were not copied to durable raw-log files. Values below are reconstructed from
those captured outputs; no raw-log SHA is claimed.

## Trial 1 — `+0.20/-0.05 Nm`, Kp/Kd `80/2.5`

- Start: `2026-08-20 16:38:32 +08:00`.
- Starting position: `0.025385696 rad`; enable stability passed at zero
  displacement and velocity.
- First changed compressed target readback matched exactly.
- Quarter: command `0.027904402`, measured `0.025741853`, displacement
  `+0.356156 mrad`, velocity `0.000758698 rad/s`, total feed-forward
  `+0.199994 Nm`, estimated total `0.371102 Nm`.
- Commanded peak: displacement `+0.668868 mrad`, effectively zero velocity,
  estimated total `0.346115 Nm`.
- Three-quarter: displacement `+0.522811 mrad`, velocity
  `-0.000129125 rad/s`.
- Returned persistent baseline readback matched exactly and all drives were
  disabled. Final acceptance rejected the positive peak because it was below
  the required `2.5 mrad`. Exit `1`; this was not a safety-guard or cleanup
  failure.

## Trial 2 — `+0.425/-0.05 Nm`, Kp/Kd `80/2.5`

- Start: `2026-08-20 16:40:31 +08:00`.
- Starting position: `0.025709830 rad`; enable stability and first changed
  target readback passed.
- Quarter: displacement `+1.545403 mrad`, velocity `0.000920382 rad/s`,
  feed-forward `+0.424990 Nm`, estimated total `0.500396 Nm`, measured torque
  `0.483141 Nm`.
- Shortly after quarter, static friction released and measured velocity reached
  `0.021850 rad/s`, exceeding the unchanged `0.020000 rad/s` hard limit.
  The controller immediately restored its registered baseline and confirmed
  all six drives disabled. Exit `1`. The velocity limit was not relaxed and
  this envelope must not be replayed.

## Trial 3 — `+0.30/-0.05 Nm`, Kp/Kd `80/4.0`

- Start: `2026-08-20 16:42:38 +08:00`.
- Starting position: `0.028750453 rad`; enable stability and first changed
  target readback passed.
- Quarter: command `0.031267930`, measured `0.031371068`, displacement
  `+2.620615 mrad`, velocity `0.000056296 rad/s`, feed-forward `+0.299993 Nm`,
  estimated total `0.291517 Nm`, measured torque `0.260153 Nm`.
- Commanded peak: displacement `+2.739333 mrad`, velocity
  `-0.000211486 rad/s`, estimated total `0.181350 Nm`.
- Three-quarter: displacement `+2.632037 mrad`, velocity
  `-0.000419668 rad/s`, feed-forward `-0.049999 Nm` and estimated total
  `-0.060252 Nm`.
- No hard position, velocity, torque, temperature, CAN, or state guard fired.
  After returning to the persistent `60/2.5`, zero-feed-forward baseline, J1
  remained at `+2.511 mrad` with zero velocity. This exceeded the `1.25 mrad`
  return limit for the full three-second settle deadline, so the controller
  restored the same baseline, disabled J1 first, disabled the remaining axes,
  and exited `1`.

The final trial's source/run identities before the subsequent offline return
update were:

- `commissioning.rs` SHA-256:
  `bf94de8c9fce05af615be059eed91d82ffb2dffa94f0817ed7274182b12144e4`;
- installed controller SHA-256:
  `b50a4cc115639d0e30bad223fa9957be7584efa90343f8052663ee58ea79c7df`;
- run-profile SHA-256:
  `9874d5728acf8227bf9d34d878d266b6321561158ec062c871990da578834c3c`.

## Decision

Trial 3 proves the reviewed phase-shaped positive envelope can move J1 in the
expected ROS direction beyond the `50%` acceptance threshold without crossing
the safety envelope. It does not prove the complete round trip. At the final
`+2.511 mrad` error, persistent Kp `60` supplied only about `-0.151 Nm`, below
the independently observed negative response interval `-0.175..-0.200 Nm`.

The offline follow-up therefore retains `80/4.0`, positive peak `+0.30 Nm`,
and changes only the return phase peak from `-0.05` to the previously bounded
`-0.175 Nm`; compensation still equals zero at start, turn-around, and final
return. This change has not been exercised on hardware. Do not promote the
temporary gains or compensation into the persistent profile, and do not run
another J1 trial without a new explicit decision. Proceed with other axes
while J1 remains incomplete and MoveIt execution stays locked.

