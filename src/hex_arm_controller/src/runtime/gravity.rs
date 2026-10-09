//! Feedback-derived holds, gravity feed-forward, and startup slew.
use super::*;

impl ArmRuntime {
    pub fn set_gravity(&self, session_id: u32, gravity: [f32; 3]) -> Result<()> {
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        anyhow::ensure!(
            self.profile.calibrated,
            "gravity compensation is locked until zero calibration is complete"
        );
        validate_gravity_vector(gravity)?;
        let mut data = self.data.write();
        anyhow::ensure!(
            !data.closing && !data.damped_stopping,
            "controller is shutting down"
        );
        anyhow::ensure!(
            session_id != 0
                && data
                    .session
                    .as_ref()
                    .is_some_and(|session| session.id == session_id),
            "request does not hold the exclusive session"
        );
        anyhow::ensure!(
            data.safety.mode != OperatingMode::GravityComp,
            "disable hand guiding before changing the gravity vector"
        );
        data.gravity = gravity;
        Ok(())
    }

    pub(super) fn gravity_comp_targets(
        &self,
        feedback: &FeedbackSnapshot,
        damping: [f32; DOF],
    ) -> Vec<RosTarget> {
        let mut targets: Vec<_> = feedback
            .joints
            .iter()
            .zip(&self.profile.joints)
            .zip(damping)
            .map(|((state, joint), kd)| RosTarget {
                position_rad: bounded_feedback_position(
                    motor_position_to_ros(state.position_rev, joint),
                    joint,
                    self.profile.controller.hand_guiding_position_margin_rad,
                ),
                velocity_rad_s: 0.0,
                torque_nm: 0.0,
                kp_nm_rad: 0.0,
                kd_nm_s_rad: kd,
            })
            .collect();
        self.apply_gravity_only(&mut targets, feedback);
        targets
    }

    pub(super) fn hold_targets(&self, feedback: &FeedbackSnapshot) -> Vec<RosTarget> {
        let bounded_meow_entry = self.profile.bus.protocol == MotorProtocol::Meow
            && feedback
                .joints
                .iter()
                .zip(&self.profile.joints)
                .all(|(state, joint)| {
                    motor_velocity_to_ros(state.velocity_rev_s, joint).abs()
                        <= crate::startup_recipe::RECIPE.stopped_velocity_rad_s
                });
        let mut targets: Vec<_> = feedback
            .joints
            .iter()
            .zip(&self.profile.joints)
            .map(|(state, joint)| {
                let measured = motor_position_to_ros(state.position_rev, joint);
                let position_rad = if bounded_meow_entry {
                    bounded_feedback_position(measured, joint, None)
                } else {
                    canonical_feedback_position(measured, joint)
                };
                RosTarget {
                    position_rad,
                    velocity_rad_s: 0.0,
                    torque_nm: 0.0,
                    kp_nm_rad: joint.default_kp,
                    kd_nm_s_rad: joint.default_kd,
                }
            })
            .collect();
        self.apply_gravity_feedforward(&mut targets, feedback);
        targets
    }

    pub(super) fn apply_gravity_feedforward(
        &self,
        targets: &mut [RosTarget],
        feedback: &FeedbackSnapshot,
    ) {
        self.apply_gravity_only(targets, feedback);
        for (target, joint) in targets.iter_mut().zip(&self.profile.joints) {
            target.torque_nm += joint.motion_feedforward_nm(target.velocity_rad_s);
        }
    }

    pub(super) fn apply_gravity_only(
        &self,
        targets: &mut [RosTarget],
        feedback: &FeedbackSnapshot,
    ) {
        let measured_q = self.ros_joint_state(feedback).0;
        let gravity = self.data.read().gravity;
        let tau = self.dynamics.gravity_torque_with(&measured_q, gravity);
        for ((target, torque_nm), joint) in targets.iter_mut().zip(tau).zip(&self.profile.joints) {
            target.torque_nm =
                joint.clamp_gravity_feedforward(torque_nm * joint.gravity_compensation_scale);
        }
    }

    /// A calculated tick may outlive its session while awaiting hardware access.
    /// Validate ownership in the same lock that mutates the shared startup ramp.
    pub(super) fn apply_command_gravity_feedforward_for_owner(
        &self,
        targets: &mut [RosTarget],
        feedback: &FeedbackSnapshot,
        automatic: bool,
        now_ns: u64,
        owner: ControlOwner,
    ) -> bool {
        if automatic {
            self.apply_gravity_feedforward(targets, feedback);
        }
        let mut data = self.data.write();
        if !data.accepts_control(owner) {
            return false;
        }
        self.apply_command_gravity_startup_locked(targets, automatic, now_ns, &mut data);
        true
    }

    /// Pure-calculation tests can exercise feed-forward without admitting a mode.
    #[cfg(test)]
    pub(super) fn apply_command_gravity_feedforward(
        &self,
        targets: &mut [RosTarget],
        feedback: &FeedbackSnapshot,
        automatic: bool,
        now_ns: u64,
    ) {
        if automatic {
            self.apply_gravity_feedforward(targets, feedback);
        }
        self.apply_command_gravity_startup_locked(
            targets,
            automatic,
            now_ns,
            &mut self.data.write(),
        );
    }

    fn apply_command_gravity_startup_locked(
        &self,
        targets: &mut [RosTarget],
        automatic: bool,
        now_ns: u64,
        data: &mut RuntimeData,
    ) {
        if !automatic {
            // Explicit tau_ff belongs to the client, including an explicit
            // zero vector. Only the current owner may bypass its startup ramp.
            data.gravity_startup_ramp = None;
            return;
        }
        if let (Some(ramp), Some(rate)) = (
            data.gravity_startup_ramp.as_mut(),
            self.profile.controller.gravity_startup_slew_rate_nm_s,
        ) {
            if ramp.apply(targets, rate, now_ns) {
                // Continuous slew limiting would lag posture changes and
                // under-compensate gravity. Only startup uses this limiter.
                data.gravity_startup_ramp = None;
                tracing::info!("gravity_ready: startup feed-forward reached the measured-pose target; position motion is now accepted");
                self.push_event_locked(
                    data,
                    pb::EventSeverity::Info,
                    "gravity_ready",
                    "gravity startup ramp complete; position motion is now accepted".into(),
                    &[],
                );
            }
        }
    }
}

impl GravityStartupRamp {
    pub(super) fn new(now_ns: u64, hold_targets: Vec<RosTarget>) -> Self {
        Self {
            hold_targets,
            output_nm: [0.0; DOF],
            last_tick_ns: now_ns,
        }
    }

    pub(super) fn validate_hold_command(&self, targets: &[RosTarget]) -> Result<()> {
        anyhow::ensure!(
            targets.iter().zip(&self.hold_targets).all(|(target, hold)| {
                (target.position_rad - hold.position_rad).abs() <= MEASURED_POSITION_EPSILON_RAD
                    && target.velocity_rad_s.abs() <= MEASURED_VELOCITY_EPSILON_RAD_S
            }),
            "gravity startup is still ramping; keep the activation pose and wait for gravity_ready before commanding motion"
        );
        Ok(())
    }

    /// Every axis uses the same elapsed control tick. Return true only once
    /// all outputs have reached the current, feedback-derived gravity target.
    pub(super) fn apply(&mut self, targets: &mut [RosTarget], rate_nm_s: f32, now_ns: u64) -> bool {
        let dt_s = now_ns.saturating_sub(self.last_tick_ns) as f64 / 1_000_000_000.0;
        self.last_tick_ns = now_ns;
        let maximum_step = (rate_nm_s as f64 * dt_s) as f32;
        let mut settled = true;
        for (output, target) in self.output_nm.iter_mut().zip(targets) {
            if !target.torque_nm.is_finite() {
                // A slew limiter must not hide invalid dynamics behind a
                // finite intermediate output; preserve it for validation.
                settled = false;
                continue;
            }
            let delta = target.torque_nm - *output;
            if delta.abs() > maximum_step {
                *output += delta.signum() * maximum_step;
                settled = false;
            } else {
                *output = target.torque_nm;
            }
            target.torque_nm = *output;
        }
        settled
    }
}

pub(super) fn canonical_feedback_position(
    measured: f32,
    joint: &crate::profile::JointProfile,
) -> f32 {
    let lower = joint.limits.position_lower_rad;
    let upper = joint.limits.position_upper_rad;
    // Fixed-point quantization/f32 roundoff only; this never uses the wider
    // measured_position_margin_rad or changes external command authority.
    if measured < lower && lower - measured <= MEASURED_POSITION_EPSILON_RAD {
        lower
    } else if measured > upper && measured - upper <= MEASURED_POSITION_EPSILON_RAD {
        upper
    } else {
        measured
    }
}

pub(super) fn bounded_feedback_position(
    measured: f32,
    joint: &crate::profile::JointProfile,
    margin: Option<f32>,
) -> f32 {
    let margin = margin.unwrap_or(joint.limits.measured_position_margin_rad);
    let lower = joint.limits.position_lower_rad;
    let upper = joint.limits.position_upper_rad;
    if measured.is_finite()
        && measured >= lower - margin - MEASURED_POSITION_EPSILON_RAD
        && measured <= upper + margin + MEASURED_POSITION_EPSILON_RAD
    {
        // Confine feedback-derived holds to command limits. Gravity and fault
        // checks, and measured-state publication, retain the actual feedback.
        measured.clamp(lower, upper)
    } else {
        // Never hide feedback outside the accepted envelope or non-finite data.
        measured
    }
}
