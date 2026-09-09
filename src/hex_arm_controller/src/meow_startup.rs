//! Explicit, bounded commissioning of the operator-verified folded-to-ready path.
//! This does not mark an arm calibrated or expose unrestricted motion.
use crate::backend::{FeedbackSnapshot, MotorBackend, DOF};
use crate::conversion::{
    motor_position_to_ros, motor_velocity_to_ros, ros_target_to_motor, MotorTarget, RosTarget,
};
use crate::profile::{HardwareProfile, MotorProtocol};
use anyhow::Result;
use hex_arm_dynamics::ArmDynamics;
use std::array;
use std::time::{Duration, Instant};

pub const FOLDED: [f32; DOF] = [0.0, -1.57, 3.14, 0.0, 0.0, 0.0];
pub const READY: [f32; DOF] = [0.0, -1.35, 3.0, -0.30, 0.0, 0.0];
// Order is physical commissioning knowledge supplied by the operator.
pub const STEPS: [(usize, f32, f32); 3] = [(1, -1.35, 8.0), (3, -0.30, 10.0), (2, 3.0, 6.0)];
const TRACKING_LIMIT: f32 = 0.035;
const ARRIVAL_LIMIT: f32 = 0.003 * std::f32::consts::TAU; // GUI's 0.003 Rev
const STOPPED_SPEED: f32 = 0.02;
const START_LIMIT: f32 = 0.01;
// The operator verified this fixed fold exit despite mesh/reference uncertainty.
// Match its feedback envelope to the tracking guard; command limits and the
// normal ROS runtime's profile margin stay unchanged.
const STARTUP_MEASURED_MARGIN: f32 = TRACKING_LIMIT;

fn measured(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
) -> Result<([f32; DOF], [f32; DOF])> {
    anyhow::ensure!(
        feedback.all_online_and_fresh(),
        "startup lost fresh, fault-free six-axis feedback"
    );
    let q = array::from_fn(|i| {
        motor_position_to_ros(feedback.joints[i].position_rev, &profile.joints[i])
    });
    let dq = array::from_fn(|i| {
        motor_velocity_to_ros(feedback.joints[i].velocity_rev_s, &profile.joints[i])
    });
    for i in 0..DOF {
        let limits = &profile.joints[i].limits;
        anyhow::ensure!(
            q[i].is_finite() && dq[i].is_finite(),
            "nonfinite feedback on joint {}",
            i + 1
        );
        anyhow::ensure!(
            q[i] >= limits.position_lower_rad - STARTUP_MEASURED_MARGIN
                && q[i] <= limits.position_upper_rad + STARTUP_MEASURED_MARGIN,
            "measured position limit on joint {}: {}",
            i + 1,
            q[i]
        );
        anyhow::ensure!(
            dq[i].abs() <= limits.velocity_rad_s,
            "measured speed limit on joint {}: {}",
            i + 1,
            dq[i]
        );
        anyhow::ensure!(
            feedback.joints[i].temperature_c < 60.0
                && feedback.joints[i].driver_temperature_c < 60.0
                && feedback.joints[i].motor_temperature_c < 60.0,
            "startup motor temperature limit"
        );
    }
    Ok((q, dq))
}

fn check_start(q: &[f32; DOF], dq: &[f32; DOF]) -> Result<()> {
    anyhow::ensure!(
        q.iter().all(|v| v.is_finite())
            && q[..5]
                .iter()
                .zip(FOLDED[..5].iter())
                .all(|(a, b)| a.is_finite() && (*a - *b).abs() <= START_LIMIT),
        "startup requires the confirmed J1-J5 folded reference; actual q={q:?}"
    );
    anyhow::ensure!(
        dq.iter().all(|v| v.is_finite() && v.abs() <= STOPPED_SPEED),
        "startup requires a stationary arm"
    );
    Ok(())
}

fn waypoint(
    start: [f32; DOF],
    axis: usize,
    target: f32,
    duration: f32,
    elapsed: f32,
) -> ([f32; DOF], [f32; DOF]) {
    let u = (elapsed / duration).clamp(0.0, 1.0);
    let blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    let derivative = 30.0 * u * u * (1.0 - u) * (1.0 - u) / duration;
    let mut q = start;
    let mut dq = [0.0; DOF];
    q[axis] += (target - start[axis]) * blend;
    dq[axis] = (target - start[axis]) * derivative;
    (q, dq)
}

fn motor_targets(
    profile: &HardwareProfile,
    dynamics: &ArmDynamics,
    q: [f32; DOF],
    dq: [f32; DOF],
    sensed: [f32; DOF],
    ff_fraction: f32,
) -> Result<[MotorTarget; DOF]> {
    let gravity = dynamics.gravity_torque_with(&sensed, profile.gravity_vector_base_m_s2);
    let targets = array::from_fn(|i| {
        let joint = &profile.joints[i];
        ros_target_to_motor(
            RosTarget {
                position_rad: q[i],
                velocity_rad_s: dq[i],
                torque_nm: joint
                    .clamp_gravity_feedforward(gravity[i] * joint.gravity_compensation_scale)
                    * ff_fraction,
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            },
            joint,
        )
    });
    for (i, target) in targets.iter().enumerate() {
        // Meow profiles require torque_scale=1, so the direction conversion
        // preserves magnitude. This also covers profiles without an FF clamp.
        anyhow::ensure!(
            target.torque_nm.is_finite()
                && target.torque_nm.abs() <= profile.joints[i].limits.torque_nm,
            "startup feed-forward exceeds joint {} software torque limit",
            i + 1
        );
    }
    Ok(targets)
}

/// Run only after initialize_disabled; caller always performs confirmed shutdown.
pub async fn run(
    backend: &dyn MotorBackend,
    profile: &HardwareProfile,
    dynamics: &ArmDynamics,
) -> Result<()> {
    anyhow::ensure!(
        profile.bus.protocol == MotorProtocol::Meow,
        "startup sequence requires Meow"
    );
    anyhow::ensure!(
        profile.controller.loop_hz == 500,
        "startup sequence requires the GUI 500 Hz profile"
    );
    let (q0, dq0) = measured(profile, &backend.feedback())?;
    check_start(&q0, &dq0)?;
    // Verify the complete fixed path and raw PD/gravity budgets before any enable.
    // The operator permits an arbitrary J6 orientation. Preserve its measured
    // position throughout this dedicated trial; ROS startup can align it to 0.
    let mut folded = FOLDED;
    folded[5] = q0[5];
    let mut q = folded;
    for (axis, target, duration) in STEPS {
        anyhow::ensure!(
            5.774 * (target - q[axis]).abs() / (duration * duration)
                <= profile.joints[axis].limits.acceleration_rad_s2,
            "startup acceleration exceeds profile"
        );
        for sample in 0..=100 {
            let (point, velocity) =
                waypoint(q, axis, target, duration, duration * sample as f32 / 100.0);
            for i in 0..DOF {
                let l = &profile.joints[i].limits;
                anyhow::ensure!(
                    point[i] >= l.position_lower_rad
                        && point[i] <= l.position_upper_rad
                        && velocity[i].abs() <= l.velocity_rad_s,
                    "startup path exceeds joint {} authority",
                    i + 1
                );
            }
            backend.validate_targets(motor_targets(
                profile, dynamics, point, velocity, point, 1.0,
            )?)?;
        }
        q[axis] = target;
    }
    let initial = motor_targets(profile, dynamics, folded, [0.0; DOF], q0, 0.0)?;
    backend.validate_targets(initial)?;
    backend.enable_compressed_mit(initial).await?;
    tracing::info!(
        "startup: enabled at the confirmed folded reference; 2 s gravity ramp and 1 s settle"
    );
    let mut tick = tokio::time::interval(Duration::from_millis(2));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let started = Instant::now();
    while started.elapsed().as_secs_f32() < 3.0 {
        tick.tick().await;
        anyhow::ensure!(!backend.transport_failed(), "startup transport failure");
        let (sensed, _) = measured(profile, &backend.feedback())?;
        anyhow::ensure!(
            sensed
                .iter()
                .zip(folded)
                .all(|(a, b)| (*a - b).abs() <= TRACKING_LIMIT),
            "startup hold drift exceeded 0.035 rad: {sensed:?}"
        );
        let fraction = (started.elapsed().as_secs_f32() / 2.0).min(1.0);
        backend
            .set_targets(motor_targets(
                profile, dynamics, folded, [0.0; DOF], sensed, fraction,
            )?)
            .await?;
    }
    let mut reference = folded;
    for (axis, target, duration) in STEPS {
        let phase_start = Instant::now();
        let mut stable_since = None;
        let mut next_log = 0.0;
        loop {
            tick.tick().await;
            let elapsed = phase_start.elapsed().as_secs_f32();
            anyhow::ensure!(
                elapsed <= duration + 5.0,
                "joint {} did not settle at startup target",
                axis + 1
            );
            anyhow::ensure!(!backend.transport_failed(), "startup transport failure");
            let (sensed, velocity) = measured(profile, &backend.feedback())?;
            let (command, command_velocity) = waypoint(reference, axis, target, duration, elapsed);
            for i in 0..DOF {
                anyhow::ensure!(
                    (sensed[i] - command[i]).abs() <= TRACKING_LIMIT,
                    "startup tracking limit on joint {}: sensed={}, target={}",
                    i + 1,
                    sensed[i],
                    command[i]
                );
            }
            backend
                .set_targets(motor_targets(
                    profile,
                    dynamics,
                    command,
                    command_velocity,
                    sensed,
                    1.0,
                )?)
                .await?;
            if elapsed >= next_log {
                tracing::info!(
                    joint = axis + 1,
                    elapsed,
                    ?sensed,
                    ?command,
                    "startup tracking"
                );
                next_log = elapsed + 1.0;
            }
            let arrived = elapsed >= duration
                && sensed
                    .iter()
                    .zip(command)
                    .all(|(a, b)| (*a - b).abs() <= ARRIVAL_LIMIT)
                && velocity.iter().all(|v| v.abs() <= STOPPED_SPEED);
            if arrived {
                let since = stable_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_millis(500) {
                    tracing::info!(joint = axis + 1, ?sensed, "startup step settled");
                    break;
                }
            } else {
                stable_since = None;
            }
        }
        reference[axis] = target;
    }
    tracing::info!(
        "startup_ready: fixed J2 -> J4 -> J3 sequence completed; holding for 3 s before disable"
    );
    let holding = Instant::now();
    while holding.elapsed() < Duration::from_secs(3) {
        tick.tick().await;
        anyhow::ensure!(!backend.transport_failed(), "startup transport failure");
        let (sensed, _) = measured(profile, &backend.feedback())?;
        anyhow::ensure!(
            sensed
                .iter()
                .zip(reference)
                .all(|(a, b)| (*a - b).abs() <= TRACKING_LIMIT),
            "ready hold drift"
        );
        backend
            .set_targets(motor_targets(
                profile, dynamics, reference, [0.0; DOF], sensed, 1.0,
            )?)
            .await?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_sequence_moves_one_axis_at_a_time_with_bounded_smooth_velocity() {
        let mut q = FOLDED;
        for (axis, target, duration) in STEPS {
            let mut previous_dq = [0.0_f32; DOF];
            for sample in 0..=1000 {
                let (p, v) = waypoint(q, axis, target, duration, duration * sample as f32 / 1000.0);
                for i in 0..DOF {
                    assert!(v[i].abs() < 0.06);
                    if sample > 0 {
                        assert!((v[i] - previous_dq[i]).abs() / (duration / 1000.0) < 0.1);
                    }
                    if i != axis {
                        assert_eq!(p[i], q[i]);
                        assert_eq!(v[i], 0.0);
                    }
                }
                previous_dq = v;
            }
            let (end, v) = waypoint(q, axis, target, duration, duration);
            assert!((end[axis] - target).abs() < 1e-6);
            assert_eq!(v, [0.0; DOF]);
            q[axis] = target;
        }
        assert_eq!(q, READY);
    }
    #[test]
    fn software_feedforward_limit_is_enforced_without_an_optional_clamp() {
        let mut profile: HardwareProfile = serde_yaml::from_str(include_str!(
            "../../../config/hardware/firefly_y6.meow_mit.example.yaml"
        ))
        .unwrap();
        let dynamics = ArmDynamics::from_urdf_string(include_str!(
            "../../xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf"
        ))
        .unwrap();
        let baseline = motor_targets(&profile, &dynamics, READY, [0.0; DOF], READY, 1.0).unwrap();
        assert!(baseline[1].torque_nm.abs() > 0.01);
        profile.joints[1].gravity_compensation_limit_nm = None;
        profile.joints[1].limits.torque_nm = 0.001;
        assert!(motor_targets(&profile, &dynamics, READY, [0.0; DOF], READY, 1.0).is_err());
    }
    #[test]
    fn startup_boundary_uncertainty_does_not_allow_stale_or_unbounded_feedback() {
        let profile: HardwareProfile = serde_yaml::from_str(include_str!(
            "../../../config/hardware/firefly_y6.meow_mit.example.yaml"
        ))
        .unwrap();
        let mut feedback = FeedbackSnapshot::default();
        for (i, joint) in feedback.joints.iter_mut().enumerate() {
            joint.online = true;
            joint.fresh = true;
            joint.position_rev = ros_target_to_motor(
                RosTarget {
                    position_rad: FOLDED[i],
                    ..Default::default()
                },
                &profile.joints[i],
            )
            .position_rev;
        }
        // Folded J3 may sag just past the nominal mesh boundary while holding.
        feedback.joints[2].position_rev += 0.02 / std::f32::consts::TAU;
        measured(&profile, &feedback).unwrap();
        feedback.joints[2].fresh = false;
        assert!(measured(&profile, &feedback).is_err());
        feedback.joints[2].fresh = true;
        feedback.joints[2].position_rev += 0.02 / std::f32::consts::TAU;
        assert!(measured(&profile, &feedback).is_err());
    }
    #[test]
    fn j6_offset_is_held_without_redefining_the_folded_reference() {
        for j6 in [-2.7, -0.219, 0.497, 2.7] {
            let mut q = FOLDED;
            q[5] = j6;
            check_start(&q, &[0.0; DOF]).unwrap();
            for (axis, target, duration) in STEPS {
                q = waypoint(q, axis, target, duration, duration).0;
                assert_eq!(q[5], j6);
            }
            assert_eq!(&q[..5], &READY[..5]);
        }
        let mut q = FOLDED;
        q[5] = f32::NAN;
        assert!(check_start(&q, &[0.0; DOF]).is_err());
    }
    #[test]
    fn arbitrary_pose_or_moving_start_is_rejected() {
        check_start(&FOLDED, &[0.0; DOF]).unwrap();
        assert!(check_start(&READY, &[0.0; DOF]).is_err());
        assert!(check_start(&FOLDED, &[0.03; DOF]).is_err());
        let mut invalid = FOLDED;
        invalid[0] = f32::NAN;
        assert!(check_start(&invalid, &[0.0; DOF]).is_err());
    }
}
