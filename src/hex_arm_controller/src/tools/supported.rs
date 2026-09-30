//! Explicit bounded motion with named supporting axes. Legacy single-axis
//! authority is unchanged. Caller owns signal handling and confirmed shutdown.
use super::*;

#[derive(Debug, Clone)]
pub struct SupportedRequest {
    pub motion: CommissioningRequest,
    pub supports: Vec<usize>,
    /// Support-only preparation steps, returned in reverse after the trial.
    pub preparation: Vec<(usize, f32)>,
    pub max_temperature_rise_c: f32,
}

impl SupportedRequest {
    pub fn validate(&self, profile: &HardwareProfile) -> Result<[bool; DOF]> {
        let r = self.motion;
        anyhow::ensure!(
            self.max_temperature_rise_c.is_finite()
                && (2.0..=10.0).contains(&self.max_temperature_rise_c),
            "supported temperature rise limit must be in [2,10] C"
        );
        anyhow::ensure!(
            r.expanded_motion && r.selected_index < DOF,
            "support requires explicit expanded commissioning and a valid selected axis"
        );
        let mut active = [false; DOF];
        active[r.selected_index] = true;
        anyhow::ensure!(!self.supports.is_empty(), "name at least one support axis");
        for &i in &self.supports {
            anyhow::ensure!(i < DOF && !active[i], "duplicate/invalid support axis {i}");
            active[i] = true;
        }
        anyhow::ensure!(
            self.preparation.len() <= 2,
            "at most two support preparation steps"
        );
        let mut prepared = [false; DOF];
        for &(i, delta) in &self.preparation {
            anyhow::ensure!(
                i < DOF
                    && self.supports.contains(&i)
                    && !prepared[i]
                    && delta.is_finite()
                    && delta.abs() <= 0.25
                    && delta.abs() >= 0.0001,
                "preparation requires distinct named supports and bounded nonzero deltas"
            );
            prepared[i] = true;
            let limits = &profile.joints[i].limits;
            anyhow::ensure!(
                PI * delta.abs() / 40.0 <= limits.velocity_rad_s
                    && PI * PI * delta.abs() / 800.0 <= limits.acceleration_rad_s2,
                "preparation exceeds profile rates"
            );
        }
        anyhow::ensure!(
            r.delta_rad.is_finite()
                && r.delta_rad.abs() <= MAX_TRAVEL_TEST_DELTA_RAD
                && (r.delta_rad == 0.0 || r.delta_rad.abs() >= MIN_COMMISSION_DELTA_RAD)
                && r.duration_sec.is_finite()
                && r.duration_sec > 0.0
                && r.duration_sec <= 30.0,
            "supported motion requires finite delta <=0.25 rad and duration (0,30] s"
        );
        let rate = profile.controller.gravity_startup_slew_rate_nm_s;
        anyhow::ensure!(
            rate.is_some_and(|x| x.is_finite() && x > 0.0 && x <= 1.0),
            "supported motion requires gravity slew in (0,1] Nm/s"
        );
        let torque_caps = [6.0, 6.0, 7.5, 1.5, 1.5, 1.5];
        let drive_caps = [200, 200, 250, 200, 200, 200];
        for (i, j) in profile.joints.iter().enumerate() {
            if let Some(motion) = &j.motion_feedforward {
                motion.validate(&j.limits)?;
                anyhow::ensure!(
                    motion.positive_nm <= 2.0 && motion.negative_nm <= 2.0,
                    "{} motion feed-forward exceeds 2 Nm commissioning bound",
                    j.name
                );
            }
            let (kp, kd) = if active[i] {
                (200.0, 20.0)
            } else {
                (80.0, 4.0)
            };
            anyhow::ensure!(
                j.default_kp.is_finite()
                    && (0.0..=kp).contains(&j.default_kp)
                    && j.default_kd.is_finite()
                    && (0.0..=kd).contains(&j.default_kd),
                "{} exceeds explicitly active-axis gain bounds",
                j.name
            );
            anyhow::ensure!(
                j.limits.velocity_rad_s <= 0.1
                    && j.limits.acceleration_rad_s2 <= 0.1
                    && j.limits.torque_nm <= torque_caps[i]
                    && j.torque_permille <= drive_caps[i]
                    && j.kp_kd_torque_permille <= 100,
                "{} exceeds supported commissioning caps",
                j.name
            );
        }
        let limits = &profile.joints[r.selected_index].limits;
        anyhow::ensure!(
            PI * r.delta_rad.abs() / r.duration_sec <= limits.velocity_rad_s
                && 2.0 * PI * PI * r.delta_rad.abs() / r.duration_sec.powi(2)
                    <= limits.acceleration_rad_s2,
            "supported trajectory exceeds profile velocity/acceleration"
        );
        Ok(active)
    }
}

fn position_guards(
    active: [bool; DOF],
    selected: usize,
    initial: &[f32; DOF],
    q: &[f32; DOF],
    command: f32,
    holding: bool,
    hold_limits: &[f32; DOF],
) -> Result<()> {
    for i in 0..DOF {
        let target = if i == selected { command } else { initial[i] };
        let limit = if !active[i] {
            TRAVEL_PASSIVE_DRIFT_RAD
        } else if i != selected || holding {
            hold_limits[i]
        } else {
            TRAVEL_TRACKING_ERROR_RAD
        };
        anyhow::ensure!(
            (q[i] - target).abs() <= limit,
            "joint {} {} error {:.6} exceeds {:.6} rad (q={:.6}, target={:.6})",
            i + 1,
            if !active[i] {
                "passive"
            } else if i != selected {
                "support"
            } else {
                "selected"
            },
            q[i] - target,
            limit,
            q[i],
            target
        );
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    active: [bool; DOF],
    command: &[f32; DOF],
    commanded_velocity: &[f32; DOF],
    q: &[f32; DOF],
    applied: &mut [f32; DOF],
    dt: f32,
) -> Result<([MotorTarget; DOF], bool)> {
    let gravity = dynamics.gravity_torque_with(q, profile.gravity_vector_base_m_s2);
    let rate = profile
        .controller
        .gravity_startup_slew_rate_nm_s
        .context("missing gravity slew")?;
    let mut output = [MotorTarget::default(); DOF];
    let mut ready = true;
    for i in 0..DOF {
        let j = &profile.joints[i];
        let nominal = if active[i] {
            j.clamp_gravity_feedforward(gravity[i] * j.gravity_compensation_scale)
        } else {
            0.0
        };
        let motion = if active[i] {
            j.motion_feedforward_nm(commanded_velocity[i])
        } else {
            0.0
        };
        anyhow::ensure!(
            nominal.is_finite()
                && (nominal + motion).is_finite()
                && nominal.abs() <= j.limits.torque_nm
                && (nominal + motion).abs() <= j.limits.torque_nm,
            "{} gravity plus motion feed-forward exceeds torque limit",
            j.name
        );
        applied[i] = slew_gravity_torque(applied[i], nominal, rate, dt)?;
        ready &= applied[i] == nominal;
        let torque_nm = applied[i] + motion;
        anyhow::ensure!(
            torque_nm.is_finite() && torque_nm.abs() <= j.limits.torque_nm,
            "{} applied feed-forward exceeds torque limit",
            j.name
        );
        let target = if active[i] {
            command[i]
        } else {
            q[i].clamp(j.limits.position_lower_rad, j.limits.position_upper_rad)
        };
        anyhow::ensure!(
            target.is_finite()
                && (j.limits.position_lower_rad..=j.limits.position_upper_rad).contains(&target),
            "{} supported target exceeds command limits",
            j.name
        );
        output[i] = ros_target_to_motor(
            RosTarget {
                position_rad: target,
                velocity_rad_s: 0.0,
                torque_nm,
                kp_nm_rad: if active[i] { j.default_kp } else { 0.0 },
                kd_nm_s_rad: if active[i] { j.default_kd } else { 0.0 },
            },
            j,
        );
    }
    Ok((output, ready))
}

fn checked_feedback(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    phase: &str,
    axis: usize,
) -> Result<[f32; DOF]> {
    let result = validate_feedback(profile, feedback, None);
    if let Err(error) = &result {
        let q = feedback_positions_unchecked(profile, feedback);
        let dq: [f32; DOF] = array::from_fn(|i| {
            motor_velocity_to_ros(feedback.joints[i].velocity_rev_s, &profile.joints[i])
        });
        let torque: [f32; DOF] = array::from_fn(|i| {
            motor_torque_to_ros(feedback.joints[i].torque_nm, &profile.joints[i])
        });
        tracing::error!(phase,axis=axis+1,?q,?dq,?torque,%error,"supported feedback guard");
    }
    result
}

#[allow(clippy::too_many_arguments)]
async fn move_support(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    active: [bool; DOF],
    reference: &mut [f32; DOF],
    index: usize,
    delta: f32,
    applied: &mut [f32; DOF],
    baseline: SixAxisDiagnosticTemperatureBaseline,
    hold_limits: &mut [f32; DOF],
) -> Result<()> {
    let start = *reference;
    let mut excursion = MeasuredExcursion::new(start[index], delta);
    let mut metrics = TravelMetrics::default();
    let target = start[index] + delta;
    let limits = &profile.joints[index].limits;
    anyhow::ensure!(
        (limits.position_lower_rad..=limits.position_upper_rad).contains(&target),
        "support preparation endpoint outside profile"
    );
    let started = Instant::now();
    let mut last = started;
    let mut log_at = started;
    let mut gate = ContinuousStabilityGate::new(Duration::from_millis(500), Duration::from_secs(5));
    tracing::info!(
        axis = index + 1,
        delta_rad = delta,
        target_rad = target,
        "support preparation started (20 s)"
    );
    loop {
        let now = Instant::now();
        let elapsed = now.duration_since(started).as_secs_f32();
        let u = (elapsed / 20.0).clamp(0.0, 1.0);
        let mut command = start;
        let mut commanded_velocity = [0.0; DOF];
        command[index] += 0.5 * (1.0 - (PI * u).cos()) * delta;
        if elapsed < 20.0 {
            commanded_velocity[index] = PI * delta / 40.0 * (PI * u).sin();
        }
        let feedback = backend.feedback();
        let q = checked_feedback(profile, &feedback, "support_preparation", index)?;
        metrics.observe(
            &profile.joints[index],
            &feedback,
            index,
            command[index],
            q[index],
        );
        excursion.observe(q[index])?;
        let dq: [f32; DOF] = array::from_fn(|i| {
            motor_velocity_to_ros(feedback.joints[i].velocity_rev_s, &profile.joints[i])
        });
        backend.ensure_supported_commissioning_state(active)?;
        validate_six_axis_diagnostic_temperatures(
            profile,
            &feedback,
            baseline,
            "support preparation",
        )?;
        position_guards(
            active,
            index,
            &start,
            &q,
            command[index],
            false,
            hold_limits,
        )
        .with_context(|| {
            format!(
                "support preparation axis {} q={q:?}, command={command:?}",
                index + 1
            )
        })?;
        let (out, _) = targets(
            profile,
            dynamics,
            active,
            &command,
            &commanded_velocity,
            &q,
            applied,
            now.duration_since(last).as_secs_f32(),
        )?;
        last = now;
        backend.set_targets(out).await?;
        if now.duration_since(log_at) >= Duration::from_secs(1) {
            let commanded_feedforward_nm: [f32; DOF] =
                array::from_fn(|i| motor_torque_to_ros(out[i].torque_nm, &profile.joints[i]));
            tracing::info!(
                axis = index + 1,
                elapsed_sec = elapsed,
                ?q,
                ?dq,
                ?command,
                ?applied,
                ?commanded_feedforward_nm,
                "support preparation feedback"
            );
            log_at = now;
        }
        if elapsed >= 20.0 {
            let stable = (0..DOF).all(|i| {
                !active[i]
                    || ((q[i] - command[i]).abs()
                        <= if i == index {
                            TRAVEL_ENDPOINT_ERROR_RAD
                        } else {
                            hold_limits[i]
                        }
                        && dq[i].abs() <= STABLE_VELOCITY_RAD_S)
            });
            match gate.observe(Duration::from_secs_f32(elapsed - 20.0), stable) {
                StabilityGateStatus::Stable => {
                    let peak = excursion.validate_completed()?;
                    anyhow::ensure!(
                        (peak - delta).abs() <= TRAVEL_ENDPOINT_ERROR_RAD,
                        "support preparation missed measured endpoint"
                    );
                    tracing::info!(
                        axis = index + 1,
                        ?q,
                        ?command,
                        prep_peak_delta_rad = peak,
                        prep_max_tracking_error_rad = metrics.max_tracking_error_rad,
                        prep_max_velocity_rad_s = metrics.max_velocity_rad_s,
                        prep_max_torque_nm = metrics.max_torque_nm,
                        "support preparation passed"
                    );
                    *reference = command;
                    // The axis has deliberately moved and passed the existing
                    // trajectory goal tolerance. Keep that command as its hold
                    // reference; never rebase to hide the residual error.
                    hold_limits[index] = TRAVEL_ENDPOINT_ERROR_RAD;
                    return Ok(());
                }
                StabilityGateStatus::TimedOut => anyhow::bail!(
                    "support preparation axis {} arrival failed: q={q:?}, command={command:?}",
                    index + 1
                ),
                _ => {}
            }
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

pub async fn run_supported_commissioning(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: SupportedRequest,
) -> Result<()> {
    let active = request.validate(profile)?;
    profile.validate_single_turn_command_windows()?;
    let r = request.motion;
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let mut initial = validate_feedback(profile, &initial_feedback, None)?;
    let mut baseline = joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    baseline.rise_limit_c = request.max_temperature_rise_c;
    tracing::info!(
        max_temperature_rise_c = baseline.rise_limit_c,
        max_temperature_c = MAX_DIAGNOSTIC_TEMPERATURE_C,
        "supported temperature protection configured"
    );
    for i in 0..DOF {
        if active[i] {
            let j = &profile.joints[i];
            let delta = if i == r.selected_index {
                r.delta_rad
            } else {
                (j.limits.position_lower_rad + j.limits.position_upper_rad) * 0.5 - initial[i]
            };
            initial[i] = canonical_travel_start(j, initial[i], delta)?;
        }
    }
    let selected = r.selected_index;
    let mut prepared_endpoint = initial;
    for &(i, delta) in &request.preparation {
        prepared_endpoint[i] += delta;
        let limits = &profile.joints[i].limits;
        anyhow::ensure!(
            (limits.position_lower_rad..=limits.position_upper_rad).contains(&prepared_endpoint[i]),
            "joint {} preparation endpoint exceeds profile",
            i + 1
        );
    }
    let joint = &profile.joints[selected];
    anyhow::ensure!(
        (joint.limits.position_lower_rad..=joint.limits.position_upper_rad)
            .contains(&(initial[selected] + r.delta_rad)),
        "supported endpoint exceeds profile"
    );
    let mut hold_limits = [ENABLE_STABILITY_POSITION_RAD; DOF];
    let mut applied = [0.0; DOF];
    let (initial_targets, _) = targets(
        profile,
        dynamics,
        active,
        &initial,
        &[0.0; DOF],
        &initial,
        &mut applied,
        0.0,
    )?;
    let mut order = request.supports.clone();
    order.push(selected);
    // A cancelled partial enable converges on the caller's all-axis shutdown.
    // No configuration/enable await can hide a position/thermal guard failure.
    let monitor = async {
        loop {
            let feedback = backend.feedback();
            let q = checked_feedback(profile, &feedback, "activation", selected)?;
            position_guards(
                active,
                selected,
                &initial,
                &q,
                initial[selected],
                true,
                &[ENABLE_STABILITY_POSITION_RAD; DOF],
            )?;
            validate_six_axis_diagnostic_temperatures(
                profile,
                &feedback,
                baseline,
                "group activation",
            )?;
            tokio::time::sleep(LOOP_PERIOD).await;
        }
        #[allow(unreachable_code)]
        Ok::<(), anyhow::Error>(())
    };
    tokio::select! {
        result = backend.enable_supported_commissioning_axes(&order,initial_targets) => result?,
        result = monitor => result?,
        _ = tokio::time::sleep(Duration::from_secs(30)) => anyhow::bail!("supported activation timed out"),
    }
    tracing::info!(
        ?active,
        ?initial,
        delta_rad = r.delta_rad,
        "supported axes enabled with zero gravity feed-forward"
    );
    let mut metrics = TravelMetrics::default();
    let mut max_hold_error = [0.0_f32; DOF];
    let mut excursion = MeasuredExcursion::new(initial[selected], r.delta_rad);
    let mut stage = "gravity_hold";
    let mut stage_started = Instant::now();
    let mut last_tick = stage_started;
    let mut last_log = stage_started;
    let mut stability =
        ContinuousStabilityGate::new(Duration::from_secs(1), Duration::from_secs(10));
    loop {
        let now = Instant::now();
        let elapsed = now.duration_since(stage_started);
        let mut command = initial;
        let mut commanded_velocity = [0.0; DOF];
        if stage == "trajectory" {
            command[selected] +=
                round_trip_phase(elapsed.as_secs_f32() / r.duration_sec) * r.delta_rad;
            if elapsed.as_secs_f32() < r.duration_sec {
                commanded_velocity[selected] = PI * r.delta_rad / r.duration_sec
                    * (2.0 * PI * elapsed.as_secs_f32() / r.duration_sec).sin();
            }
        }
        let feedback = backend.feedback();
        let q = checked_feedback(profile, &feedback, stage, selected)?;
        let dq: [f32; DOF] = array::from_fn(|i| {
            motor_velocity_to_ros(feedback.joints[i].velocity_rev_s, &profile.joints[i])
        });
        let guard = (|| -> Result<()> {
            anyhow::ensure!(
                !backend.transport_failed(),
                "CAN failed during supported motion"
            );
            backend.ensure_supported_commissioning_state(active)?;
            validate_six_axis_diagnostic_temperatures(
                profile,
                &feedback,
                baseline,
                "supported commissioning",
            )?;
            position_guards(
                active,
                selected,
                &initial,
                &q,
                command[selected],
                stage == "gravity_hold" || r.delta_rad == 0.0,
                &hold_limits,
            )?;
            if stage != "gravity_hold" && r.delta_rad != 0.0 {
                excursion.observe(q[selected])?;
            }
            Ok(())
        })();
        if let Err(error) = guard {
            tracing::error!(stage,?q,?dq,?command,?applied,?active,%error,"supported motion guard stopped the trial");
            return Err(error);
        }
        for i in 0..DOF {
            max_hold_error[i] = max_hold_error[i].max((q[i] - initial[i]).abs());
        }
        if stage != "gravity_hold" {
            metrics.observe(joint, &feedback, selected, command[selected], q[selected]);
        }
        let (output, gravity_ready) = targets(
            profile,
            dynamics,
            active,
            &command,
            &commanded_velocity,
            &q,
            &mut applied,
            now.duration_since(last_tick).as_secs_f32(),
        )?;
        last_tick = now;
        backend.set_targets(output).await?;
        if now.duration_since(last_log) >= Duration::from_secs(1) {
            let commanded_feedforward_nm: [f32; DOF] =
                array::from_fn(|i| motor_torque_to_ros(output[i].torque_nm, &profile.joints[i]));
            tracing::info!(
                stage,
                elapsed_sec = elapsed.as_secs_f32(),
                ?q,
                ?dq,
                ?command,
                ?commanded_velocity,
                ?applied,
                ?commanded_feedforward_nm,
                "supported feedback"
            );
            last_log = now;
        }
        let quiet = (0..DOF).all(|i| !active[i] || dq[i].abs() <= STABLE_VELOCITY_RAD_S);
        match stage {
            "gravity_hold" => match stability.observe(elapsed, gravity_ready && quiet) {
                StabilityGateStatus::Stable => {
                    for &(i, delta) in &request.preparation {
                        move_support(
                            backend,
                            profile,
                            dynamics,
                            active,
                            &mut initial,
                            i,
                            delta,
                            &mut applied,
                            baseline,
                            &mut hold_limits,
                        )
                        .await?;
                    }
                    stage = "trajectory";
                    stage_started = Instant::now();
                    last_tick = stage_started;
                    let prepared_q =
                        checked_feedback(profile, &backend.feedback(), "prepared", selected)?;
                    tracing::info!(
                        ?prepared_q,
                        command = ?initial,
                        ?applied,
                        "supported gravity hold passed; starting bounded trial"
                    );
                }
                StabilityGateStatus::TimedOut => {
                    anyhow::bail!("supported gravity hold did not stabilize")
                }
                _ => {}
            },
            "trajectory" if elapsed.as_secs_f32() >= r.duration_sec => {
                stage = "return_settle";
                stage_started = now;
                stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
            }
            "return_settle" => {
                let error = (q[selected] - initial[selected]).abs();
                let tolerance = if r.delta_rad == 0.0 {
                    ENABLE_STABILITY_POSITION_RAD
                } else {
                    return_tolerance_rad(r.delta_rad)
                };
                let state = stability.observe(elapsed, quiet && error <= tolerance);
                if state != StabilityGateStatus::Waiting {
                    tracing::info!(
                        samples = metrics.samples,
                        max_tracking_error_rad = metrics.max_tracking_error_rad,
                        max_velocity_rad_s = metrics.max_velocity_rad_s,
                        max_torque_nm = metrics.max_torque_nm,
                        start_position_rad = initial[selected],
                        final_position_rad = q[selected],
                        return_error_rad = error,
                        most_positive_delta_rad = excursion.most_positive_delta_rad,
                        most_negative_delta_rad = excursion.most_negative_delta_rad,
                        ?max_hold_error,
                        "supported commissioning measured metrics"
                    );
                    anyhow::ensure!(
                        state == StabilityGateStatus::Stable,
                        "supported return did not settle: {error} rad"
                    );
                    if r.delta_rad != 0.0 {
                        let peak = excursion.validate_completed()?;
                        validate_travel_endpoint(r, peak)?;
                        tracing::info!(
                            measured_peak_delta_rad = peak,
                            "supported round trip passed"
                        );
                    }
                    hold_limits[selected] = tolerance;
                    for &(i, delta) in request.preparation.iter().rev() {
                        move_support(
                            backend,
                            profile,
                            dynamics,
                            active,
                            &mut initial,
                            i,
                            -delta,
                            &mut applied,
                            baseline,
                            &mut hold_limits,
                        )
                        .await?;
                    }
                    return Ok(());
                }
            }
            _ => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn profile() -> HardwareProfile {
        let mut p = super::super::tests::profile_with_velocity_limit(0.1);
        p.controller.gravity_startup_slew_rate_nm_s = Some(1.0);
        for j in &mut p.joints[3..] {
            j.limits.torque_nm = 1.5;
        }
        p
    }
    #[test]
    fn explicit_temperature_authority_keeps_absolute_and_nonfinite_guards() {
        for value in [39.9, 40.0] {
            validate_temperature_with_rise_limit("joint_3", "motor", value, 30.0, 10.0).unwrap();
        }
        for value in [40.1, 71.0, f32::NAN] {
            assert!(
                validate_temperature_with_rise_limit("joint_3", "motor", value, 30.0, 10.0)
                    .is_err()
            );
        }
        assert!(
            validate_temperature_with_rise_limit("joint_3", "motor", 70.1, 69.0, 10.0).is_err()
        );
        assert!(
            validate_joint1_first_position_temperature("joint_3", "motor", 32.1, 30.0).is_err()
        );
        for limit in [1.9, 10.1, f32::NAN, f32::INFINITY] {
            assert!(
                validate_temperature_with_rise_limit("joint_3", "motor", 30.0, 30.0, limit)
                    .is_err()
            );
        }
    }

    #[test]
    fn supported_authority_rejects_duplicate_axes_inactive_gains_rates_and_caps() {
        let mut p = profile();
        p.joints[2].default_kp = 200.0;
        p.joints[3].default_kp = 100.0;
        let mut r = SupportedRequest {
            motion: CommissioningRequest {
                selected_index: 2,
                delta_rad: -0.015,
                duration_sec: 24.0,
                expanded_motion: true,
            },
            supports: vec![3],
            preparation: vec![],
            max_temperature_rise_c: 2.0,
        };
        r.validate(&p).unwrap();
        assert!(
            r.motion.validate(&p).is_err(),
            "legacy selected-axis authority must remain separate"
        );
        r.supports = vec![3, 3];
        assert!(r.validate(&p).is_err());
        r.supports = vec![2];
        assert!(r.validate(&p).is_err());
        r.supports = vec![6];
        assert!(r.validate(&p).is_err());
        r.supports = vec![3];
        p.joints[0].default_kp = 100.0;
        assert!(r.validate(&p).is_err());
        p.joints[0].default_kp = 2.0;
        r.motion.duration_sec = 0.01;
        assert!(r.validate(&p).is_err());
        r.motion.duration_sec = 24.0;
        r.motion.delta_rad = 0.0;
        r.validate(&p).unwrap();
        p.controller.gravity_startup_slew_rate_nm_s = None;
        assert!(r.validate(&p).is_err());
        p.controller.gravity_startup_slew_rate_nm_s = Some(f32::NAN);
        assert!(r.validate(&p).is_err());
        p.controller.gravity_startup_slew_rate_nm_s = Some(1.0);
        p.joints[2].torque_permille = 251;
        assert!(r.validate(&p).is_err());
        p.joints[2].torque_permille = 100;
        r.preparation = vec![(2, 0.03)];
        assert!(r.validate(&p).is_err());
        r.preparation = vec![(3, f32::NAN)];
        assert!(r.validate(&p).is_err());
        r.preparation = vec![(3, -0.03), (3, -0.03)];
        assert!(r.validate(&p).is_err());
        r.preparation = vec![(3, -0.03)];
        r.validate(&p).unwrap();
        p.joints[2].motion_feedforward = Some(crate::profile::MotionFeedforward {
            positive_nm: 0.5,
            negative_nm: 1.0,
            velocity_scale_rad_s: 0.001,
        });
        r.validate(&p).unwrap();
        assert!(r.motion.validate(&p).is_err());
        p.joints[2].motion_feedforward.as_mut().unwrap().negative_nm = 2.01;
        assert!(r.validate(&p).is_err());
    }
    #[test]
    fn supported_targets_hold_support_position_and_validate_full_gravity_before_zero_enable() {
        let mut p = profile();
        p.joints[2].direction = -1;
        p.joints[2].torque_scale = 0.85;
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut links = vec![(0.0, [0.0; 3]); DOF];
        links[3] = (0.2, [0.1, 0.0, 0.0]);
        let dynamics = hex_arm_dynamics::ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            links,
            [0.0, 0.0, -9.81],
        );
        let active = [false, false, true, true, false, false];
        let command = [0.0; DOF];
        let mut q = command;
        q[3] = 0.001;
        let mut applied = [0.0; DOF];
        let (out, ready) = targets(
            &p,
            &dynamics,
            active,
            &command,
            &[0.0; DOF],
            &q,
            &mut applied,
            0.0,
        )
        .unwrap();
        assert!(!ready);
        assert_eq!(out[2].torque_nm, 0.0);
        let (out, _) = targets(
            &p,
            &dynamics,
            active,
            &command,
            &[0.0; DOF],
            &q,
            &mut applied,
            0.1,
        )
        .unwrap();
        assert!((motor_torque_to_ros(out[2].torque_nm, &p.joints[2]) - applied[2]).abs() < 1e-6);
        assert!(applied[2].abs() <= 0.100001);
        assert_eq!(
            motor_position_to_ros(out[3].position_rev, &p.joints[3]),
            command[3]
        );
        assert_eq!(out[0].kp_nm_rev, 0.0);
        assert_eq!(out[0].torque_nm, 0.0);
        p.joints[2].motion_feedforward = Some(crate::profile::MotionFeedforward {
            positive_nm: 0.2,
            negative_nm: 0.4,
            velocity_scale_rad_s: 0.001,
        });
        let mut velocity = [0.0; DOF];
        velocity[2] = -0.01;
        let previous_gravity = applied[2];
        let (out, _) = targets(
            &p,
            &dynamics,
            active,
            &command,
            &velocity,
            &q,
            &mut applied,
            0.0,
        )
        .unwrap();
        assert!(
            (motor_torque_to_ros(out[2].torque_nm, &p.joints[2]) - previous_gravity + 0.4).abs()
                < 1e-6
        );
        assert_eq!(
            applied[2], previous_gravity,
            "motion assistance must not enter the gravity ramp state"
        );
        p.joints[2].limits.torque_nm = 0.25;
        assert!(targets(&p, &dynamics, active, &command, &velocity, &q, &mut applied, 0.0).is_err(),
            "combined force must be validated even though gravity and motion terms each have their own bounds");
        p.joints[3].limits.torque_nm = 0.01;
        assert!(targets(
            &p,
            &dynamics,
            active,
            &command,
            &[0.0; DOF],
            &q,
            &mut [0.0; DOF],
            0.0
        )
        .is_err());
    }
    #[test]
    fn supported_hold_and_passive_guards_are_independent_of_selected_tracking() {
        let active = [false, false, true, true, false, false];
        let start = [0.0; DOF];
        let mut q = start;
        q[2] = -0.014;
        q[3] = 0.002;
        position_guards(
            active,
            2,
            &start,
            &q,
            -0.015,
            false,
            &[ENABLE_STABILITY_POSITION_RAD; DOF],
        )
        .unwrap();
        assert!(position_guards(
            active,
            2,
            &start,
            &q,
            0.0,
            true,
            &[ENABLE_STABILITY_POSITION_RAD; DOF]
        )
        .is_err());
        q[3] = 0.0031;
        let mut arrived_limits = [ENABLE_STABILITY_POSITION_RAD; DOF];
        arrived_limits[3] = TRAVEL_ENDPOINT_ERROR_RAD;
        position_guards(active, 2, &start, &q, -0.015, false, &arrived_limits).unwrap();
        let mut outside = q;
        outside[3] = 0.0051;
        assert!(
            position_guards(active, 2, &start, &outside, -0.015, false, &arrived_limits).is_err()
        );
        assert!(position_guards(
            active,
            2,
            &start,
            &q,
            -0.015,
            false,
            &[ENABLE_STABILITY_POSITION_RAD; DOF]
        )
        .is_err());
        q[3] = 0.0;
        q[4] = 0.0051;
        assert!(position_guards(
            active,
            2,
            &start,
            &q,
            -0.015,
            false,
            &[ENABLE_STABILITY_POSITION_RAD; DOF]
        )
        .is_err());
        q[4] = f32::NAN;
        assert!(position_guards(
            active,
            2,
            &start,
            &q,
            -0.015,
            false,
            &[ENABLE_STABILITY_POSITION_RAD; DOF]
        )
        .is_err());
    }
}
