//! One-way shutdown admission and bounded damped stop.
use super::*;

impl ArmRuntime {
    /// Latch process shutdown before waiting on any asynchronous operation.
    /// This method is synchronous on purpose: callers handling SIGINT/SIGTERM
    /// can close the API admission gate before a queued set_mode wakes up.
    pub fn begin_shutdown(&self) -> bool {
        let newly_latched = {
            let mut data = self.data.write();
            if data.closing {
                false
            } else {
                data.closing = true;
                data.command = None;
                data.gravity_comp = None;
                data.session = None;
                data.gravity = self.profile.gravity_vector_base_m_s2;
                self.push_event_locked(
                    &mut data,
                    pb::EventSeverity::Info,
                    "controller_closing",
                    "controller shutdown latched; new API operations are rejected".into(),
                    &[],
                );
                true
            }
        };
        if newly_latched {
            self.closing_tx.send_replace(true);
        }
        newly_latched
    }

    pub fn is_closing(&self) -> bool {
        self.data.read().closing
    }

    pub(crate) fn closing_receiver(&self) -> watch::Receiver<bool> {
        self.closing_tx.subscribe()
    }

    pub async fn shutdown(&self) -> Result<()> {
        self.begin_shutdown();
        // set_mode/release/clear_fault and fault-disable all use this same
        // gate.  Acquiring it after closing is latched drains any operation
        // that passed admission earlier and rejects every queued operation.
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        let shutdown_result = self.backend.shutdown().await;
        let mut data = self.data.write();
        data.safety.disable_preserving_fault();
        data.command = None;
        data.meow_entry_hold = None;
        data.session = None;
        data.gravity = self.profile.gravity_vector_base_m_s2;
        data.initialized = false;
        data.disable_pending = shutdown_result.is_err();
        data.next_disable_retry_at = None;
        shutdown_result
    }

    /// Own the stream until unloaded and settled at the fold, then confirm disable.
    /// API admission closes without terminating the monitoring/publication tasks.
    pub async fn damped_stop(&self, session_id: u32) -> Result<()> {
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        let cfg = self
            .profile
            .controller
            .shutdown_damping
            .as_ref()
            .context("shutdown damping is not configured")?;
        cfg.validate()?;
        let feedback = self.backend.feedback();
        self.check_damping_feedback(&feedback)?;
        let initial = self.hold_targets(&feedback);
        let ready = crate::startup_recipe::RECIPE.ready();
        self.motor_targets(&initial)?;
        let mut final_targets = initial.clone();
        for (target, kd) in final_targets.iter_mut().zip(cfg.kd_nm_s_rad) {
            target.kp_nm_rad = 0.0;
            target.kd_nm_s_rad = kd;
            target.torque_nm = 0.0;
        }
        self.motor_targets(&final_targets)?;
        anyhow::ensure!(
            initial
                .iter()
                .zip(ready)
                .all(|(t, q)| (t.position_rad - q).abs() <= 0.005),
            "damped stop requires verified startup_ready; return with MoveIt first"
        );
        anyhow::ensure!(
            feedback
                .joints
                .iter()
                .zip(&self.profile.joints)
                .all(|(f, j)| motor_velocity_to_ros(f.velocity_rev_s, j).abs() <= 0.02),
            "damped stop requires stationary feedback"
        );
        {
            let mut data = self.data.write();
            anyhow::ensure!(
                !data.closing
                    && !data.damped_stopping
                    && data.safety.mode == OperatingMode::Active
                    && data.gravity_startup_ramp.is_none(),
                "damped stop requires healthy ACTIVE control"
            );
            data.damped_stopping = true;
            data.command = None;
        }
        tracing::info!("damped_shutdown: ready verified; Rust owns the stream until final disable");
        let operation = self.run_damping(cfg, initial).await;
        // This call is not cancelled by the RPC client disappearing. Any error still
        // reaches confirmed disable; a process signal is observed in run_damping.
        let disabled = self.backend.disable_all().await;
        let mut data = self.data.write();
        if let Err(error) = &operation {
            data.safety
                .latch_fault(FAULT_COMMAND, format!("damped stop incomplete: {error:#}"));
        } else {
            data.safety.disable_preserving_fault();
        }
        data.disable_pending = disabled.is_err();
        if let Err(error) = disabled {
            data.safety.latch_fault(
                FAULT_MODE_TRANSITION,
                format!("damped stop disable unconfirmed: {error:#}"),
            );
            anyhow::bail!("damped stop disable unconfirmed: {error:#}; settling={operation:?}");
        }
        operation?;
        tracing::info!("damped_shutdown: folded, unloaded, settled; disable confirmed");
        Ok(())
    }

    pub(super) fn check_damping_feedback(&self, feedback: &FeedbackSnapshot) -> Result<()> {
        anyhow::ensure!(
            !self.is_closing(),
            "damping interrupted by immediate shutdown"
        );
        anyhow::ensure!(
            !self.backend.transport_failed() && feedback.all_online_and_fresh(),
            "damping feedback/transport fault"
        );
        anyhow::ensure!(
            self.data.read().safety.mode == OperatingMode::Active,
            "damping requires ACTIVE without fault"
        );
        if let Some((_, reason)) = self.measured_feedback_fault(feedback) {
            anyhow::bail!(reason);
        }
        Ok(())
    }

    pub(super) async fn run_damping(
        &self,
        cfg: &crate::profile::ShutdownDamping,
        initial: Vec<RosTarget>,
    ) -> Result<()> {
        let start = Instant::now();
        let mut settled_since = None;
        let mut interval = tokio::time::interval(Duration::from_secs_f64(
            1.0 / self.profile.controller.loop_hz as f64,
        ));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let folded = crate::startup_recipe::RECIPE.folded_position_rad;
        loop {
            interval.tick().await;
            let elapsed = start.elapsed().as_secs_f32();
            anyhow::ensure!(
                elapsed < cfg.timeout_sec,
                "damping timed out before unloaded folded settling; disabling"
            );
            let feedback = self.backend.feedback();
            self.check_damping_feedback(&feedback)?;
            self.data.write().feedback = feedback.clone();
            let x = (elapsed / cfg.unload_sec).clamp(0.0, 1.0);
            let blend = x * x * (3.0 - 2.0 * x);
            let mut targets = self.hold_targets(&feedback);
            for (i, t) in targets.iter_mut().enumerate() {
                t.position_rad = initial[i].position_rad;
                t.kp_nm_rad = initial[i].kp_nm_rad * (1.0 - blend);
                t.kd_nm_s_rad = initial[i].kd_nm_s_rad * (1.0 - blend) + cfg.kd_nm_s_rad[i] * blend;
                t.torque_nm *= 1.0 - blend;
            }
            self.backend
                .set_targets(self.motor_targets(&targets)?)
                .await?;
            let settled = x >= 1.0
                && feedback
                    .joints
                    .iter()
                    .zip(&self.profile.joints)
                    .zip(folded)
                    .all(|((f, j), q)| {
                        (motor_position_to_ros(f.position_rev, j) - q).abs() <= 0.02
                            && motor_velocity_to_ros(f.velocity_rev_s, j).abs() <= 0.02
                    });
            if settled {
                let since = settled_since.get_or_insert_with(Instant::now);
                if since.elapsed().as_secs_f32() >= cfg.settle_sec {
                    return Ok(());
                }
            } else {
                settled_since = None;
            }
        }
    }
}
