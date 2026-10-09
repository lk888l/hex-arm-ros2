//! Serialized hardware mode transitions and rollback.
use super::*;

impl ArmRuntime {
    pub async fn set_mode(&self, session_id: u32, requested: OperatingMode) -> Result<()> {
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        anyhow::ensure!(
            requested != OperatingMode::GravityComp,
            "GRAVITY_COMP is unavailable through set_mode; use start_gravity_comp with damping and a session deadman"
        );
        self.set_mode_locked(session_id, requested, None).await
    }

    pub async fn start_gravity_comp(&self, session_id: u32, damping: &[f32]) -> Result<()> {
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        anyhow::ensure!(
            damping.len() == DOF && damping.iter().all(|d| d.is_finite() && *d > 0.0),
            "hand-guiding damping must contain six finite positive joint-side gains"
        );
        anyhow::ensure!(
            self.mode() == OperatingMode::Disabled,
            "hand guiding must start from DISABLED; stop the current owner first"
        );
        self.set_mode_locked(
            session_id,
            OperatingMode::GravityComp,
            Some(damping.try_into().expect("six gains checked")),
        )
        .await
    }

    pub fn gravity_comp_heartbeat(&self, session_id: u32, sequence: u64) -> Result<()> {
        let mut data = self.data.write();
        anyhow::ensure!(
            !data.closing && !data.damped_stopping,
            "controller is shutting down"
        );
        anyhow::ensure!(
            session_id != 0 && data.session.as_ref().is_some_and(|s| s.id == session_id),
            "request does not hold the exclusive session"
        );
        anyhow::ensure!(
            data.safety.mode != OperatingMode::Fault,
            "hand guiding stopped: fault 0x{:04x}: {}",
            data.safety.fault_code,
            data.safety.fault_reason
        );
        anyhow::ensure!(
            data.safety.mode == OperatingMode::GravityComp,
            "hand guiding is not active"
        );
        let lease = data
            .gravity_comp
            .as_mut()
            .context("hand guiding has no lease")?;
        anyhow::ensure!(
            lease.renewed_at.elapsed() < GRAVITY_COMP_LEASE,
            "hand-guiding lease expired"
        );
        anyhow::ensure!(
            sequence > lease.sequence,
            "heartbeat sequence must increase"
        );
        lease.sequence = sequence;
        lease.renewed_at = Instant::now();
        Ok(())
    }

    // The caller holds mode_gate through enable, commit and rollback.
    pub(super) async fn set_mode_locked(
        &self,
        session_id: u32,
        requested: OperatingMode,
        damping: Option<[f32; DOF]>,
    ) -> Result<()> {
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        anyhow::ensure!(
            self.mode() != OperatingMode::GravityComp || requested == OperatingMode::Disabled,
            "disable hand guiding before switching to another operating mode"
        );
        anyhow::ensure!(
            !matches!(requested, OperatingMode::Fault | OperatingMode::Calibrating),
            "mode is controller-owned and cannot be requested"
        );

        let feedback = self.backend.feedback();
        let all_fresh = feedback.all_online_and_fresh();
        if requested == OperatingMode::GravityComp {
            anyhow::ensure!(
                feedback
                    .joints
                    .iter()
                    .zip(&self.profile.joints)
                    .all(
                        |(state, joint)| motor_velocity_to_ros(state.velocity_rev_s, joint).abs()
                            <= 0.02
                    ),
                "support and stop the arm before hand guiding (entry speed must be <= 0.02 rad/s)"
            );
        }
        let next_safety = {
            let mut data = self.data.write();
            data.feedback = feedback.clone();
            let initialized = data.initialized;
            let mut candidate = data.safety.clone();
            candidate.request_mode(
                requested,
                self.profile.validated && initialized,
                self.profile.calibrated,
                all_fresh,
                all_fresh,
            )?;
            candidate
        };

        if matches!(
            requested,
            OperatingMode::Active | OperatingMode::GravityComp
        ) {
            if let Some((_, reason)) = self.measured_feedback_fault_in_mode(&feedback, requested) {
                anyhow::bail!("unsafe measured state blocks mode transition: {reason}");
            }
        }

        let hold_targets = matches!(
            requested,
            OperatingMode::Active | OperatingMode::GravityComp
        )
        .then(|| {
            let mut targets = match damping {
                Some(gains) => self.gravity_comp_targets(&feedback, gains),
                None => self.hold_targets(&feedback),
            };
            // Validate the eventual gravity target before enabling, even when
            // the startup frame itself contains zero feed-forward.
            // Include firmware gain quantization and the Tff + PD budget
            // before zeroing Tff for a startup ramp and enabling any axis.
            self.motor_targets(&targets)?;
            if requested == OperatingMode::GravityComp
                || self
                    .profile
                    .controller
                    .gravity_startup_slew_rate_nm_s
                    .is_some()
            {
                // ACTIVE keeps PD support; hand guiding has damping only and
                // requires external support. Ramp after the mode commit.
                for target in &mut targets {
                    target.torque_nm = 0.0;
                }
            }
            Ok::<_, anyhow::Error>(targets)
        })
        .transpose()?;
        let initial_motor_targets = hold_targets
            .as_ref()
            .map(|targets| self.motor_targets(targets))
            .transpose()?;
        let active_targets = (requested == OperatingMode::Active)
            .then(|| hold_targets.clone().expect("ACTIVE hold targets prepared"));
        let meow_entry_hold = active_targets.as_ref().and_then(|targets| {
            if self.profile.bus.protocol != MotorProtocol::Meow {
                return None;
            }
            let measured = self.ros_joint_state(&feedback).0;
            let bounded: Vec<_> = targets.iter().map(|target| target.position_rad).collect();
            measured
                .iter()
                .zip(&bounded)
                .any(|(raw, command)| (raw - command).abs() > MEASURED_POSITION_EPSILON_RAD)
                .then_some(MeowEntryHold { measured, bounded })
        });

        let hardware_result = match requested {
            OperatingMode::Disabled => self.backend.disable_all().await,
            OperatingMode::Active | OperatingMode::GravityComp => {
                let motor_targets = initial_motor_targets.expect("enabled motor targets prepared");
                match self.backend.enable_compressed_mit(motor_targets).await {
                    Ok(()) => self.backend.set_targets(motor_targets).await,
                    Err(error) => Err(error),
                }
            }
            // SafetyState rejects PASSIVE before hardware dispatch. Keep this
            // branch fail-closed as a second line of defence: the old path
            // enabled MIT and then published an all-zero target, whose
            // torque-free behaviour has not been commissioned on this arm.
            OperatingMode::Passive => Err(anyhow::anyhow!(
                "PASSIVE hardware operation is not commissioned"
            )),
            OperatingMode::Fault | OperatingMode::Calibrating => unreachable!(),
        };

        if let Err(error) = hardware_result {
            let disable_error = self.backend.disable_all().await.err();
            let reason = if let Some(disable_error) = &disable_error {
                format!(
                    "hardware mode transition to {requested:?} failed: {error:#}; confirmed rollback disable failed: {disable_error:#}"
                )
            } else {
                format!("hardware mode transition to {requested:?} failed: {error:#}")
            };
            self.latch_whole_arm_fault(FAULT_MODE_TRANSITION, reason.clone());
            {
                let mut data = self.data.write();
                data.disable_pending = disable_error.is_some();
                data.next_disable_retry_at = disable_error
                    .as_ref()
                    .map(|_| Instant::now() + FAULT_DISABLE_RETRY_PERIOD);
                if disable_error.is_none() {
                    self.push_event_locked(
                        &mut data,
                        pb::EventSeverity::Info,
                        "fault_disable_confirmed",
                        "all six axes confirmed disabled after failed mode transition".into(),
                        &[],
                    );
                }
            }
            if let Some(disable_error) = disable_error {
                tracing::error!(%disable_error, "confirmed rollback disable failed; fault retry remains armed");
            }
            anyhow::bail!(reason);
        }

        // `begin_shutdown` deliberately does not wait for mode_gate: it first
        // latches closing so work already queued on this mutex will reject.
        // If shutdown arrived while the hardware transition above was in
        // flight, do not publish ACTIVE and roll the completed transition back
        // before releasing the gate to final backend shutdown.
        let commit_rejection = {
            let mut data = self.data.write();
            if data.closing {
                Some("controller is shutting down".to_string())
            } else if data.safety.mode == OperatingMode::Fault
                && next_safety.mode != OperatingMode::Fault
            {
                // Fault admission is synchronous and does not wait for mode_gate.
                // Hardware success must never overwrite a fault latched in flight.
                Some(format!(
                    "mode transition blocked by latched fault 0x{:04x}: {}; clear_fault is required",
                    data.safety.fault_code, data.safety.fault_reason,
                ))
            } else {
                data.safety = next_safety;
                data.control_epoch = data.control_epoch.wrapping_add(1);
                data.meow_entry_hold = meow_entry_hold;
                data.gravity_comp = damping.map(|damping| GravityCompLease {
                    damping,
                    renewed_at: Instant::now(),
                    sequence: 0,
                    ramp: Some(GravityStartupRamp::new(self.monotonic_ns(), Vec::new())),
                });
                data.gravity_startup_ramp = (requested == OperatingMode::Active
                    && self
                        .profile
                        .controller
                        .gravity_startup_slew_rate_nm_s
                        .is_some())
                .then(|| {
                    GravityStartupRamp::new(
                        self.monotonic_ns(),
                        active_targets
                            .as_ref()
                            .expect("ACTIVE hold prepared")
                            .clone(),
                    )
                });
                data.disable_pending = false;
                data.next_disable_retry_at = None;
                data.command = if let Some(targets) = active_targets {
                    data.command_generation = data.command_generation.wrapping_add(1);
                    Some(CommandEnvelope {
                        generation: data.command_generation,
                        source_sequence: 0,
                        rebase_from_feedback: Some(targets.clone()),
                        targets,
                        duration_ns: 0,
                        received_at: Instant::now(),
                        automatic_gravity_feedforward: true,
                    })
                } else {
                    None
                };
                self.push_event_locked(
                    &mut data,
                    pb::EventSeverity::Info,
                    "operating_mode_changed",
                    format!("operating mode changed to {requested:?}"),
                    &[("mode", format!("{requested:?}"))],
                );
                None
            }
        };
        if let Some(reason) = commit_rejection {
            let rollback = self.backend.disable_all().await;
            {
                let mut data = self.data.write();
                if data.safety.mode == OperatingMode::Fault {
                    // A successful rollback quiesces hardware but never resets
                    // the first fault. A failed rollback retains background retry.
                    data.disable_pending = rollback.is_err();
                    data.next_disable_retry_at = rollback
                        .as_ref()
                        .err()
                        .map(|_| Instant::now() + FAULT_DISABLE_RETRY_PERIOD);
                    if rollback.is_ok() {
                        self.push_event_locked(
                            &mut data,
                            pb::EventSeverity::Info,
                            "fault_disable_confirmed",
                            "all six axes confirmed disabled after interrupted mode transition"
                                .into(),
                            &[],
                        );
                    }
                }
            }
            rollback.with_context(|| format!("rollback mode transition rejected: {reason}"))?;
            anyhow::bail!(reason);
        }
        Ok(())
    }

    pub async fn clear_fault(&self, session_id: u32) -> Result<()> {
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        self.backend.clear_faults().await?;
        self.backend.disable_all().await?;
        let feedback = self.backend.feedback();
        anyhow::ensure!(
            feedback
                .joints
                .iter()
                .all(|joint| joint.fault_code.is_none()),
            "a motor still reports a fault"
        );
        let mut data = self.data.write();
        data.feedback = feedback;
        data.safety.clear_fault();
        data.command = None;
        data.meow_entry_hold = None;
        data.gravity_comp = None;
        data.disable_pending = false;
        data.next_disable_retry_at = None;
        self.push_event_locked(
            &mut data,
            pb::EventSeverity::Info,
            "fault_cleared",
            "latched whole-arm fault cleared; arm remains disabled".into(),
            &[],
        );
        Ok(())
    }
}
