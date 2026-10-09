//! Control-task loop; owns its interpolator and consumed generation.
use super::*;

impl ArmRuntime {
    pub async fn run_control_loop(self: Arc<Self>) {
        let mut interval = tokio::time::interval(Duration::from_secs_f64(
            1.0 / self.profile.controller.loop_hz as f64,
        ));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let initial = self.hold_targets(&self.backend.feedback());
        let mut interpolator = Interpolator::hold(initial, self.monotonic_ns());
        let mut generation = 0;
        let mut closing = self.closing_receiver();

        loop {
            if *closing.borrow() {
                break;
            }
            tokio::select! {
                biased;
                changed = closing.changed() => {
                    if changed.is_err() || *closing.borrow() {
                        break;
                    }
                }
                _ = interval.tick() => {}
            }
            if self.is_closing() {
                break;
            }
            crate::trace::record("control_tick", 0, generation);
            let feedback = self.backend.feedback();
            let now_ns = self.monotonic_ns();
            {
                self.data.write().feedback = feedback.clone();
            }
            let mode = self.data.read().safety.mode;
            if self.data.read().damped_stopping {
                // The bounded stop task owns both command generation and safety checks.
                // A failed final disable must still retry without a ROS caller.
                if self.fault_disable_retry_due() {
                    let wait = crate::trace::Span::new("gate_wait", 0, 0);
                    let _gate = self.mode_gate.lock().await;
                    drop(wait);
                    self.try_confirmed_fault_disable().await;
                }
                continue;
            }
            if matches!(
                mode,
                OperatingMode::Active | OperatingMode::GravityComp | OperatingMode::Passive
            ) {
                if self.backend.transport_failed() {
                    self.fault_and_disable(FAULT_TRANSPORT, "CAN transport failed")
                        .await;
                    continue;
                }
                if let Some(code) = feedback.joints.iter().find_map(|joint| joint.fault_code) {
                    self.fault_and_disable(FAULT_MOTOR, format!("motor fault 0x{code:04x}"))
                        .await;
                    continue;
                }
                if !feedback.all_online_and_fresh() {
                    self.fault_and_disable(
                        FAULT_FEEDBACK_TIMEOUT,
                        "motor feedback timeout/offline",
                    )
                    .await;
                    continue;
                }
                if let Some((code, reason)) = self.measured_feedback_fault_in_mode(&feedback, mode)
                {
                    self.fault_and_disable(code, reason).await;
                    continue;
                }
            }

            match mode {
                OperatingMode::Active => {
                    let (command, owner) = {
                        let data = self.data.read();
                        if data.damped_stopping {
                            continue;
                        }
                        (data.command.clone(), data.control_owner())
                    };
                    let Some(command) = command else {
                        self.fault_and_disable(FAULT_COMMAND_WATCHDOG, "ACTIVE without a command")
                            .await;
                        continue;
                    };
                    if command.received_at.elapsed() > self.profile.command_watchdog() {
                        self.watchdog_fault_hold_and_disable(&feedback).await;
                        continue;
                    }
                    let startup_hold = if command.automatic_gravity_feedforward {
                        self.data
                            .read()
                            .gravity_startup_ramp
                            .as_ref()
                            .map(|ramp| ramp.hold_targets.clone())
                    } else {
                        None
                    };
                    let holding_for_gravity = startup_hold.is_some();
                    if !holding_for_gravity && command.generation != generation {
                        let velocity_limits: Vec<_> = self
                            .profile
                            .joints
                            .iter()
                            .map(|joint| joint.limits.velocity_rad_s)
                            .collect();
                        let acceleration_limits: Vec<_> = self
                            .profile
                            .joints
                            .iter()
                            .map(|joint| joint.limits.acceleration_rad_s2)
                            .collect();
                        if let Err(error) = command.apply_to_interpolator(
                            &mut interpolator,
                            now_ns,
                            &velocity_limits,
                            &acceleration_limits,
                        ) {
                            self.fault_and_disable(FAULT_COMMAND, error.to_string())
                                .await;
                            continue;
                        }
                        if command.rebase_from_feedback.is_some() {
                            let mut data = self.data.write();
                            if let Some(pending) = data
                                .command
                                .as_mut()
                                .filter(|pending| pending.generation == command.generation)
                            {
                                pending.rebase_from_feedback = None;
                            }
                        }
                        generation = command.generation;
                        crate::trace::record(
                            "control_consume",
                            command.source_sequence,
                            generation,
                        );
                    }
                    let mut targets = startup_hold.unwrap_or_else(|| interpolator.sample(now_ns));
                    if !self.apply_command_gravity_feedforward_for_owner(
                        &mut targets,
                        &feedback,
                        command.automatic_gravity_feedforward,
                        now_ns,
                        owner,
                    ) {
                        continue;
                    }
                    if holding_for_gravity && self.data.read().gravity_startup_ramp.is_none() {
                        // No trajectory time elapses while gravity is starting.
                        // The next command begins at the actual activation hold.
                        interpolator = Interpolator::hold(targets.clone(), now_ns);
                        generation = command.generation;
                    }
                    match self.motor_targets(&targets) {
                        Ok(targets) => {
                            if self.is_closing() {
                                break;
                            }
                            let update = {
                                // Serialize streaming with enable/disable. A mode
                                // transition may have completed since this tick's
                                // snapshot; never send to an already disabled drive.
                                let wait = crate::trace::Span::new(
                                    "gate_wait",
                                    command.source_sequence,
                                    command.generation,
                                );
                                let _gate = self.mode_gate.lock().await;
                                drop(wait);
                                if self.is_closing() {
                                    break;
                                }
                                {
                                    let data = self.data.read();
                                    if !data.accepts_control(owner) {
                                        // Release/reacquire or reactivation may have
                                        // completed since calculation began. The new
                                        // activation owns both targets and startup ramp.
                                        continue;
                                    }
                                }
                                crate::trace::with_context(
                                    command.source_sequence,
                                    command.generation,
                                    self.backend.set_targets(targets),
                                )
                                .await
                            };
                            if let Err(error) = update {
                                self.fault_and_disable(FAULT_TRANSPORT, error.to_string())
                                    .await;
                            }
                        }
                        Err(error) => {
                            self.fault_and_disable(FAULT_COMMAND, error.to_string())
                                .await
                        }
                    }
                }
                OperatingMode::GravityComp => {
                    match self.gravity_comp_tick(&feedback, now_ns).await {
                        Ok(true) => (),
                        Ok(false) => {
                            self.fault_and_disable(
                                FAULT_COMMAND_WATCHDOG,
                                "hand-guiding session heartbeat timeout",
                            )
                            .await
                        }
                        Err(error) => {
                            self.fault_and_disable(
                                FAULT_COMMAND,
                                format!("hand guiding stopped: {error:#}"),
                            )
                            .await
                        }
                    }
                }
                OperatingMode::Passive => {
                    self.fault_and_disable(
                        FAULT_COMMAND,
                        "PASSIVE mode is unsupported; disabling instead of publishing zero targets",
                    )
                    .await;
                }
                OperatingMode::Fault => {
                    if self.fault_disable_retry_due() {
                        let wait = crate::trace::Span::new("gate_wait", 0, 0);
                        let _gate = self.mode_gate.lock().await;
                        drop(wait);
                        if self.fault_disable_retry_due() {
                            self.try_confirmed_fault_disable().await;
                        }
                    }
                }
                OperatingMode::Disabled | OperatingMode::Calibrating => {}
            }
        }
    }

    // false denotes lease expiry; other output/validation failures are errors.
    pub(super) async fn gravity_comp_tick(
        &self,
        feedback: &FeedbackSnapshot,
        now_ns: u64,
    ) -> Result<bool> {
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        if self.is_closing() || self.mode() != OperatingMode::GravityComp {
            return Ok(true);
        }
        let damping = {
            let data = self.data.read();
            anyhow::ensure!(
                data.session.is_some(),
                "unsupported hand guiding without session"
            );
            let lease = data
                .gravity_comp
                .as_ref()
                .context("unsupported hand guiding without lease")?;
            if lease.renewed_at.elapsed() >= GRAVITY_COMP_LEASE {
                return Ok(false);
            }
            lease.damping
        };
        let mut targets = self.gravity_comp_targets(feedback, damping);
        // Validate full gravity before ramping: a finite intermediate output
        // must never hide an invalid eventual motor target.
        self.motor_targets(&targets)?;
        {
            let mut data = self.data.write();
            // begin_shutdown may clear the lease without waiting for mode_gate.
            if data.closing {
                return Ok(true);
            }
            let lease = data
                .gravity_comp
                .as_mut()
                .context("missing hand-guiding lease")?;
            if let Some(ramp) = &mut lease.ramp {
                let rate = self
                    .profile
                    .controller
                    .gravity_startup_slew_rate_nm_s
                    .unwrap_or(GRAVITY_COMP_DEFAULT_SLEW_NM_S);
                if ramp.apply(&mut targets, rate, now_ns) {
                    lease.ramp = None;
                    tracing::info!(
                        "hand_guiding_ready: gravity ramp complete; hand guiding is ready"
                    );
                    self.push_event_locked(
                        &mut data,
                        pb::EventSeverity::Info,
                        "hand_guiding_ready",
                        "gravity ramp complete; hand guiding is ready".into(),
                        &[],
                    );
                }
            }
        }
        let motors = self.motor_targets(&targets)?;
        if !self.is_closing() {
            self.backend.set_targets(motors).await?;
        }
        Ok(true)
    }
}
