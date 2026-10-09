//! Streaming command preparation and atomic admission.
use super::*;

impl ArmRuntime {
    pub fn submit_trajectory(&self, command: pb::JointTrajectory) -> Result<()> {
        let prepared = self.prepare_trajectory(command)?;
        self.commit_trajectory(prepared)
    }

    pub(super) fn prepare_trajectory(
        &self,
        command: pb::JointTrajectory,
    ) -> Result<PreparedTrajectory> {
        self.ensure_accepting_requests()?;
        self.require_session(command.session_id)?;
        let timeout_behavior = pb::TimeoutBehavior::try_from(command.on_timeout)
            .map_err(|_| anyhow::anyhow!("unknown timeout behavior {}", command.on_timeout))?;
        anyhow::ensure!(
            timeout_behavior == pb::TimeoutBehavior::Fault,
            "unsupported timeout behavior {timeout_behavior:?}; only TIMEOUT_BEHAVIOR_FAULT is commissioned"
        );
        anyhow::ensure!(
            command.points.len() == 1,
            "streaming command requires exactly one setpoint; multi-point chunks are not implemented"
        );
        anyhow::ensure!(
            command.t_from_start_ns.len() == 1,
            "streaming command requires exactly one relative setpoint time"
        );
        let point = &command.points[0];
        anyhow::ensure!(point.q.len() == DOF, "trajectory q must contain six values");
        let q = &point.q;
        let dq = vector_or(&point.dq, 0.0)?;
        let kp = if point.kp.is_empty() {
            self.profile
                .joints
                .iter()
                .map(|joint| joint.default_kp)
                .collect()
        } else {
            vector_or(&point.kp, 0.0)?
        };
        let kd = if point.kd.is_empty() {
            self.profile
                .joints
                .iter()
                .map(|joint| joint.default_kd)
                .collect()
        } else {
            vector_or(&point.kd, 0.0)?
        };
        let automatic_gravity_feedforward = point.tau_ff.is_empty();
        let tau = vector_or(&point.tau_ff, 0.0)?;
        let targets: Vec<_> = (0..DOF)
            .map(|index| RosTarget {
                position_rad: q[index],
                velocity_rad_s: dq[index],
                torque_nm: tau[index],
                kp_nm_rad: kp[index],
                kd_nm_s_rad: kd[index],
            })
            .collect();
        anyhow::ensure!(
            command.t_from_start_ns[0] >= 0,
            "relative setpoint time must be non-negative"
        );
        Ok(PreparedTrajectory {
            session_id: command.session_id,
            source_sequence: command.header.as_ref().map_or(0, |header| header.seq),
            targets,
            duration_ns: command.t_from_start_ns[0] as u64,
            automatic_gravity_feedforward,
            default_hold: automatic_gravity_feedforward
                && point.kp.is_empty()
                && point.kd.is_empty(),
        })
    }

    pub(super) fn commit_trajectory(&self, prepared: PreparedTrajectory) -> Result<()> {
        let PreparedTrajectory {
            session_id,
            source_sequence,
            mut targets,
            duration_ns,
            automatic_gravity_feedforward,
            default_hold,
        } = prepared;
        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        anyhow::ensure!(
            session_id != 0
                && data
                    .session
                    .as_ref()
                    .is_some_and(|owner| owner.id == session_id),
            "request does not hold the exclusive session"
        );
        anyhow::ensure!(
            data.safety.mode == OperatingMode::Active,
            "joint commands require ACTIVE mode"
        );
        anyhow::ensure!(!data.damped_stopping, "controller is shutting down");
        if default_hold {
            if let Some(hold) = &data.meow_entry_hold {
                let outside = targets
                    .iter()
                    .zip(&self.profile.joints)
                    .any(|(target, joint)| {
                        !(joint.limits.position_lower_rad..=joint.limits.position_upper_rad)
                            .contains(&target.position_rad)
                    });
                if outside && hold.matches(&targets, true) {
                    for (target, bounded) in targets.iter_mut().zip(&hold.bounded) {
                        target.position_rad = *bounded;
                    }
                }
            }
        }
        self.validate_targets(&targets)?;
        if automatic_gravity_feedforward {
            if let Some(ramp) = &data.gravity_startup_ramp {
                ramp.validate_hold_command(&targets)?;
            }
        }
        if data
            .meow_entry_hold
            .as_ref()
            .is_some_and(|hold| !default_hold || !hold.matches(&targets, false))
        {
            data.meow_entry_hold = None;
        }
        let rebase_from_feedback = data
            .command
            .as_ref()
            .and_then(|pending| pending.rebase_from_feedback.clone());
        data.command_generation = data.command_generation.wrapping_add(1);
        crate::trace::record("rust_accept", source_sequence, data.command_generation);
        data.command = Some(CommandEnvelope {
            generation: data.command_generation,
            source_sequence,
            targets,
            duration_ns,
            received_at: Instant::now(),
            rebase_from_feedback,
            automatic_gravity_feedforward,
        });
        Ok(())
    }
}

impl CommandEnvelope {
    pub(super) fn apply_to_interpolator(
        &self,
        interpolator: &mut Interpolator,
        now_ns: u64,
        velocity_limits_rad_s: &[f32],
        acceleration_limits_rad_s2: &[f32],
    ) -> Result<()> {
        if let Some(feedback_targets) = &self.rebase_from_feedback {
            *interpolator = Interpolator::hold(feedback_targets.clone(), now_ns);
        }
        interpolator.retarget_with_limits(
            self.targets.clone(),
            now_ns,
            self.duration_ns,
            velocity_limits_rad_s,
            acceleration_limits_rad_s2,
        )
    }
}

impl MeowEntryHold {
    pub(super) fn matches(&self, targets: &[RosTarget], allow_measured_echo: bool) -> bool {
        targets.len() == self.bounded.len()
            && targets.iter().enumerate().all(|(index, target)| {
                target.velocity_rad_s.abs() <= 1.0e-6
                    && ((target.position_rad - self.bounded[index]).abs()
                        <= MEASURED_POSITION_EPSILON_RAD
                        || (allow_measured_echo
                            && (target.position_rad - self.measured[index]).abs()
                                <= MEASURED_POSITION_EPSILON_RAD))
            })
    }
}

pub(super) fn vector_or(values: &[f32], default: f32) -> Result<Vec<f32>> {
    if values.is_empty() {
        return Ok(vec![default; DOF]);
    }
    anyhow::ensure!(
        values.len() == DOF && values.iter().all(|value| value.is_finite()),
        "trajectory vector must be empty or contain six finite values"
    );
    Ok(values.to_vec())
}
