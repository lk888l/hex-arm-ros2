//! Command/feedback validation and confirmed fault disable.
use super::*;

impl ArmRuntime {
    pub(super) fn validate_targets(&self, targets: &[RosTarget]) -> Result<()> {
        anyhow::ensure!(targets.len() == DOF, "command must contain six joints");
        for (target, joint) in targets.iter().zip(&self.profile.joints) {
            anyhow::ensure!(
                [
                    target.position_rad,
                    target.velocity_rad_s,
                    target.torque_nm,
                    target.kp_nm_rad,
                    target.kd_nm_s_rad
                ]
                .iter()
                .all(|value| value.is_finite()),
                "{} command contains a non-finite value",
                joint.name
            );
            anyhow::ensure!(
                (joint.limits.position_lower_rad..=joint.limits.position_upper_rad)
                    .contains(&target.position_rad),
                "{} position exceeds software limit",
                joint.name
            );
            anyhow::ensure!(
                target.velocity_rad_s.abs() <= joint.limits.velocity_rad_s,
                "{} velocity exceeds software limit",
                joint.name
            );
            anyhow::ensure!(
                target.torque_nm.abs() <= joint.limits.torque_nm,
                "{} torque exceeds software limit",
                joint.name
            );
            anyhow::ensure!(
                target.kp_nm_rad >= 0.0 && target.kd_nm_s_rad >= 0.0,
                "{} gains must be non-negative",
                joint.name
            );
        }
        Ok(())
    }

    pub(super) fn motor_targets(
        &self,
        targets: &[RosTarget],
    ) -> Result<[crate::conversion::MotorTarget; DOF]> {
        self.validate_targets(targets)?;
        let motor_targets = array::from_fn(|index| {
            ros_target_to_motor(targets[index], &self.profile.joints[index])
        });
        self.backend.validate_targets(motor_targets)?;
        Ok(motor_targets)
    }

    pub(super) fn ros_joint_state(&self, feedback: &FeedbackSnapshot) -> (Vec<f32>, Vec<f32>) {
        let (q, dq, _, _) = self.ros_joint_state_full(feedback);
        (q, dq)
    }

    pub(super) fn ros_joint_state_full(
        &self,
        feedback: &FeedbackSnapshot,
    ) -> (Vec<f32>, Vec<f32>, Vec<f32>, Vec<f32>) {
        let mut q = Vec::with_capacity(DOF);
        let mut dq = Vec::with_capacity(DOF);
        let mut tau = Vec::with_capacity(DOF);
        let mut temp = Vec::with_capacity(DOF);
        for (state, joint) in feedback.joints.iter().zip(&self.profile.joints) {
            q.push(motor_position_to_ros(state.position_rev, joint));
            dq.push(motor_velocity_to_ros(state.velocity_rev_s, joint));
            tau.push(motor_torque_to_ros(state.torque_nm, joint));
            temp.push(state.temperature_c);
        }
        (q, dq, tau, temp)
    }

    pub(super) fn measured_feedback_fault(
        &self,
        feedback: &FeedbackSnapshot,
    ) -> Option<(u32, String)> {
        // Ordinary position-control checks retain the configured joint limits.
        self.measured_feedback_fault_in_mode(feedback, OperatingMode::Active)
    }

    pub(super) fn measured_feedback_fault_in_mode(
        &self,
        feedback: &FeedbackSnapshot,
        mode: OperatingMode,
    ) -> Option<(u32, String)> {
        let hand_guiding_limits = if mode == OperatingMode::GravityComp {
            self.profile.controller.hand_guiding_velocity_limits_rad_s
        } else {
            None
        };
        for (index, (state, joint)) in feedback.joints.iter().zip(&self.profile.joints).enumerate()
        {
            let position_rad = motor_position_to_ros(state.position_rev, joint);
            let margin = if mode == OperatingMode::GravityComp {
                self.profile.controller.hand_guiding_position_margin_rad
            } else {
                None
            }
            .unwrap_or(joint.limits.measured_position_margin_rad);
            let measured_lower = joint.limits.position_lower_rad - margin;
            let measured_upper = joint.limits.position_upper_rad + margin;
            if !position_rad.is_finite()
                || position_rad < measured_lower - MEASURED_POSITION_EPSILON_RAD
                || position_rad > measured_upper + MEASURED_POSITION_EPSILON_RAD
            {
                return Some((
                    FAULT_MEASURED_POSITION_LIMIT,
                    format!(
                        "{} measured position {:.6} rad outside read-only feedback envelope [{:.6}, {:.6}] rad (command limits remain [{:.6}, {:.6}] rad)",
                        joint.name,
                        position_rad,
                        measured_lower,
                        measured_upper,
                        joint.limits.position_lower_rad,
                        joint.limits.position_upper_rad
                    ),
                ));
            }

            let velocity_rad_s = motor_velocity_to_ros(state.velocity_rev_s, joint);
            let measured_speed_limit = hand_guiding_limits.map_or(
                joint.limits.velocity_rad_s + joint.limits.measured_velocity_margin_rad_s,
                |limits| limits[index],
            );
            if !velocity_rad_s.is_finite()
                || velocity_rad_s.abs() > measured_speed_limit + MEASURED_VELOCITY_EPSILON_RAD_S
            {
                return Some((
                    FAULT_MEASURED_OVERSPEED,
                    format!(
                        "{} measured velocity {:.6} rad/s exceeds {:.6} rad/s",
                        joint.name, velocity_rad_s, measured_speed_limit
                    ),
                ));
            }
            let torque_nm = motor_torque_to_ros(state.torque_nm, joint);
            let torque_limit_nm = self
                .backend
                .measured_torque_limit_nm(index)
                .unwrap_or(joint.limits.torque_nm);
            if !torque_nm.is_finite()
                || !torque_limit_nm.is_finite()
                || torque_limit_nm <= 0.0
                || torque_nm.abs() > torque_limit_nm + MEASURED_TORQUE_EPSILON_NM
            {
                return Some((
                    FAULT_MEASURED_TORQUE_LIMIT,
                    format!(
                        "{} measured torque {:.6} Nm exceeds {:.6} Nm",
                        joint.name, torque_nm, torque_limit_nm
                    ),
                ));
            }
            let temperature_limit = self
                .profile
                .controller
                .max_measured_temperature_c
                .unwrap_or(DEFAULT_MAX_MEASURED_TEMPERATURE_C);
            for temperature in [
                state.temperature_c,
                state.motor_temperature_c,
                state.driver_temperature_c,
            ] {
                if !temperature.is_finite() || temperature > temperature_limit {
                    return Some((
                        FAULT_MEASURED_TEMPERATURE,
                        format!("{} measured temperature {temperature} C exceeds {temperature_limit} C or is invalid", joint.name),
                    ));
                }
            }
        }
        None
    }

    pub(super) async fn watchdog_fault_hold_and_disable(&self, feedback: &FeedbackSnapshot) {
        // This is deliberately not advertised as a ramp stop: no acceleration
        // or deceleration profile is implied. Replace the stale trajectory
        // target with a zero-velocity hold at the latest measured pose, using
        // the reviewed per-axis gains and current gravity compensation, while
        // the confirmed CiA402 disable sequence runs.
        let mut hold = self.hold_targets(feedback);
        if let Some(ramp) = &self.data.read().gravity_startup_ramp {
            for (target, output) in hold.iter_mut().zip(ramp.output_nm) {
                target.torque_nm = output;
            }
        }
        let hold_targets = self
            .motor_targets(&hold)
            .context("build feedback-derived watchdog hold target");
        self.latch_whole_arm_fault(
            FAULT_COMMAND_WATCHDOG,
            "ROS command watchdog expired; entering confirmed disable",
        );

        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        if !self.data.read().disable_pending {
            return;
        }
        match hold_targets {
            Ok(targets) => {
                if let Err(error) = self.backend.set_targets(targets).await {
                    tracing::error!(%error, "watchdog feedback hold failed; disabling immediately");
                } else {
                    let mut data = self.data.write();
                    self.push_event_locked(
                        &mut data,
                        pb::EventSeverity::Info,
                        "watchdog_feedback_hold_applied",
                        "latest measured pose and gravity feed-forward held during confirmed disable"
                            .into(),
                        &[],
                    );
                }
            }
            Err(error) => {
                tracing::error!(%error, "watchdog feedback hold was unsafe; disabling immediately");
            }
        }
        self.try_confirmed_fault_disable().await;
    }

    pub(super) async fn fault_and_disable(&self, code: u32, reason: impl Into<String>) {
        self.latch_whole_arm_fault(code, reason);
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        if self.data.read().disable_pending {
            self.try_confirmed_fault_disable().await;
        }
    }

    pub(super) fn latch_whole_arm_fault(&self, code: u32, reason: impl Into<String>) {
        let reason = reason.into();
        let mut newly_latched = false;
        {
            let mut data = self.data.write();
            if data.safety.mode != OperatingMode::Fault {
                data.safety.latch_fault(code, reason.clone());
                data.command = None;
                data.gravity_comp = None;
                data.disable_pending = true;
                data.next_disable_retry_at = None;
                self.push_event_locked(
                    &mut data,
                    pb::EventSeverity::Fatal,
                    "whole_arm_fault",
                    reason.clone(),
                    &[("fault_code", format!("0x{code:04x}"))],
                );
                newly_latched = true;
            }
        }
        if newly_latched {
            tracing::error!(fault_code = code, %reason, "whole-arm fault latched");
        }
    }

    pub(super) fn fault_disable_retry_due(&self) -> bool {
        let data = self.data.read();
        data.disable_pending
            && data
                .next_disable_retry_at
                .is_none_or(|deadline| Instant::now() >= deadline)
    }

    pub(super) async fn try_confirmed_fault_disable(&self) {
        {
            let mut data = self.data.write();
            if !data.disable_pending {
                return;
            }
            data.next_disable_retry_at = Some(Instant::now() + FAULT_DISABLE_RETRY_PERIOD);
        }

        match self.backend.disable_all().await {
            Ok(()) => {
                let mut data = self.data.write();
                if data.disable_pending {
                    data.disable_pending = false;
                    data.next_disable_retry_at = None;
                    self.push_event_locked(
                        &mut data,
                        pb::EventSeverity::Info,
                        "fault_disable_confirmed",
                        "all six axes confirmed disabled after whole-arm fault".into(),
                        &[],
                    );
                }
            }
            Err(error) => {
                {
                    let mut data = self.data.write();
                    if data.disable_pending {
                        data.next_disable_retry_at =
                            Some(Instant::now() + FAULT_DISABLE_RETRY_PERIOD);
                    }
                }
                tracing::error!(%error, "confirmed fault disable failed; retry remains armed");
            }
        }
    }
}
