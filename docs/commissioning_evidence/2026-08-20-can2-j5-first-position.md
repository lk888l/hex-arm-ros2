# J5 bounded first-position survey — 2026-08-20

Status: **PASS for one bounded J5 negative 5 mrad round trip; candidate gains only.**
This does not set the hardware profile `calibrated` flag and does not authorize
MoveIt execution or a general jog interface.

## Bound configuration

- Interface: `can2`, channel 2, gs_usb serial
  `C9E29601798421B29AC2D419C12D9502`, ifindex 47, USB device 14.
- Run-profile SHA-256:
  `d80e9e4f094f69a8af249187921581ad343f8f38496d86d5058f0213c452bdd7`.
- Successful installed binary SHA-256:
  `573fd23bb78a02d286aa6b188f98be32a6247a74a989f40b43a7d0f4f9d1e729`.
- Fixed command: joint-side `-0.005 rad`, complete out-and-back duration `4.0 s`.
- Drive caps were configured and read back while disabled as `50/30 permille`
  (`0x6072:00` / `0x2004:0E`).
- Enable hold used the profile's `Kp/Kd=30/1`. The successful trajectory used
  a compile-time-fixed temporary `Kp/Kd=50/2`; the run did not mutate the
  profile.
- Both runs used strict-zero SocketCAN preflight. No historical-xstats
  acknowledgement was accepted.

## Discovery run at 30/1

Raw transcript SHA-256:
`fd36658252964a88d0098a30cc4872f81d612a2a9561c2c706b178eb83de0d3c`.

The drive consumed the changed position target and returned to the exact
baseline, but the logical peak was only `-0.001793 rad`, below the required
`-0.002500 rad`. Peak speed was about `0.00116 rad/s`; measured and estimated
torque were about `0.095/0.096 Nm`. The process exited 1 only because the 50%
tracking gate rejected the result. Cleanup completed with all six drives
confirmed disabled and heartbeat consumers disarmed.

## Accepted run at temporary 50/2

Raw transcript SHA-256:
`4ef3875f93b1e4ff0b91c72f7152b427cbb2bd91875e3a47ff403734acab074b`.

- Initial logical position: `+0.012645866 rad`.
- First changed target and returned persistent baseline both matched exact
  `0x2004:02/03` readback.
- Most-negative logical displacement: `-0.002856926 rad`.
- Most-negative raw motor displacement: `-0.000454694 rev`.
- Continuous return error: `0.000747703 rad`; return velocity was effectively
  zero.
- Peak speed: `0.005616337 rad/s`.
- Peak measured / estimated total torque: `0.129870 / 0.117220 Nm`.
- Peak driver / motor temperature: `35.6 / 32.0 C`.
- Six-axis state isolation held; process exit was 0 and all six heartbeat
  consumers were disarmed after confirmed non-torque states.
- Post-run can2 remained ERROR-ACTIVE with TEC/REC=0, all CAN xstats and netdev
  error/drop counters at zero, and no new gs_usb/EPROTO/disconnect/reset log.

## Decision

`50/2` is promoted only as the next J5 profile candidate. The run proves the
small negative excursion and positive return around this start pose; it does
not prove full travel, absolute zero, thermal endurance, payload calibration,
or whole-arm trajectory execution.
