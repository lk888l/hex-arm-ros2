use std::array;
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use hex_arm_dynamics::ArmDynamics;
use parking_lot::RwLock;
use tokio::sync::{watch, Mutex};
use tokio::time::MissedTickBehavior;

use crate::backend::{FeedbackSnapshot, MotorBackend, MotorIdentitySnapshot, DOF};
use crate::conversion::{
    motor_position_to_ros, motor_torque_to_ros, motor_velocity_to_ros, ros_target_to_motor,
    RosTarget,
};
use crate::interpolation::Interpolator;
use crate::profile::{validate_gravity_vector, HardwareProfile};
use crate::protocol::pb;
use crate::safety::{OperatingMode, SafetyState};

const FAULT_MOTOR: u32 = 0x1001;
const FAULT_FEEDBACK_TIMEOUT: u32 = 0x1002;
const FAULT_COMMAND_WATCHDOG: u32 = 0x1003;
const FAULT_TRANSPORT: u32 = 0x1004;
const FAULT_COMMAND: u32 = 0x1005;
const FAULT_MEASURED_POSITION_LIMIT: u32 = 0x1006;
const FAULT_MEASURED_OVERSPEED: u32 = 0x1007;
const FAULT_MODE_TRANSITION: u32 = 0x1008;
// Allow only enough margin to absorb f32 unit-conversion roundoff at an exact
// configured limit. These are not operating margins and do not relax commands.
const MEASURED_POSITION_EPSILON_RAD: f32 = 1.0e-4;
const MEASURED_VELOCITY_EPSILON_RAD_S: f32 = 1.0e-4;
const EVENT_CAPACITY: usize = 100;
const FAULT_DISABLE_RETRY_PERIOD: Duration = Duration::from_millis(50);

#[derive(Debug, Clone)]
struct SessionLease {
    id: u32,
    client_name: String,
}

#[derive(Debug, Clone)]
struct CommandEnvelope {
    generation: u64,
    targets: Vec<RosTarget>,
    duration_ns: u64,
    received_at: Instant,
    rebase_from_feedback: Option<Vec<RosTarget>>,
    /// An empty protocol `tau_ff` vector delegates gravity compensation to
    /// this controller. A non-empty vector remains an explicit client-owned
    /// feed-forward command (for example the legacy motor GUI).
    automatic_gravity_feedforward: bool,
}

impl CommandEnvelope {
    fn apply_to_interpolator(
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

#[derive(Debug)]
struct GravityStartupRamp {
    hold_targets: Vec<RosTarget>,
    output_nm: [f32; DOF],
    last_tick_ns: u64,
}

impl GravityStartupRamp {
    fn new(now_ns: u64, hold_targets: Vec<RosTarget>) -> Self {
        Self {
            hold_targets,
            output_nm: [0.0; DOF],
            last_tick_ns: now_ns,
        }
    }

    fn validate_hold_command(&self, targets: &[RosTarget]) -> Result<()> {
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
    fn apply(&mut self, targets: &mut [RosTarget], rate_nm_s: f32, now_ns: u64) -> bool {
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

#[derive(Debug)]
struct RuntimeData {
    /// Process shutdown is a one-way latch.  It is set before waiting for any
    /// in-flight mode transition so queued API work cannot reach hardware
    /// while the final disable/heartbeat-disarm sequence is pending.
    closing: bool,
    safety: SafetyState,
    session: Option<SessionLease>,
    next_session_id: u32,
    command: Option<CommandEnvelope>,
    command_generation: u64,
    /// A latched fault is not considered physically quiescent until the
    /// backend has confirmed that all six axes left Operation Enabled.
    disable_pending: bool,
    next_disable_retry_at: Option<Instant>,
    feedback: FeedbackSnapshot,
    motors: Vec<MotorIdentitySnapshot>,
    initialized: bool,
    gravity: [f32; 3],
    gravity_startup_ramp: Option<GravityStartupRamp>,
    events: VecDeque<pb::Event>,
    next_event_seq: u64,
}

pub struct ArmRuntime {
    pub profile: Arc<HardwareProfile>,
    backend: Arc<dyn MotorBackend>,
    dynamics: ArmDynamics,
    data: RwLock<RuntimeData>,
    mode_gate: Mutex<()>,
    closing_tx: watch::Sender<bool>,
    started_at: Instant,
}

impl ArmRuntime {
    pub fn new(
        profile: Arc<HardwareProfile>,
        backend: Arc<dyn MotorBackend>,
        dynamics: ArmDynamics,
    ) -> Self {
        let (closing_tx, _) = watch::channel(false);
        let gravity = profile.gravity_vector_base_m_s2;
        Self {
            profile,
            backend,
            dynamics,
            data: RwLock::new(RuntimeData {
                closing: false,
                safety: SafetyState::default(),
                session: None,
                next_session_id: 1,
                command: None,
                command_generation: 0,
                disable_pending: false,
                next_disable_retry_at: None,
                feedback: FeedbackSnapshot::default(),
                motors: Vec::new(),
                initialized: false,
                gravity,
                gravity_startup_ramp: None,
                events: VecDeque::with_capacity(EVENT_CAPACITY),
                next_event_seq: 1,
            }),
            mode_gate: Mutex::new(()),
            closing_tx,
            started_at: Instant::now(),
        }
    }

    pub async fn initialize(&self) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
        self.ensure_accepting_requests()?;
        self.backend.initialize_disabled().await?;
        let motors = self.backend.discover(false).await?;
        let feedback = self.backend.feedback();
        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        data.motors = motors;
        data.feedback = feedback;
        data.initialized = true;
        data.safety.mode = OperatingMode::Disabled;
        self.push_event_locked(
            &mut data,
            pb::EventSeverity::Info,
            "driver_initialized",
            "six-axis controller initialized in DISABLED mode".into(),
            &[],
        );
        Ok(())
    }

    pub async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        let _gate = self.mode_gate.lock().await;
        self.ensure_accepting_requests()?;
        let motors = self.backend.discover(refresh).await?;
        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        data.motors = motors.clone();
        Ok(motors)
    }

    pub fn acquire(&self, client_name: String) -> Result<(u32, u32, Option<String>)> {
        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        let event_client_name = client_name.clone();
        if let Some(holder) = &data.session {
            return Ok((0, holder.id, Some(holder.client_name.clone())));
        }
        let id = data.next_session_id.max(1);
        data.next_session_id = data.next_session_id.wrapping_add(1).max(1);
        data.gravity = self.profile.gravity_vector_base_m_s2;
        data.session = Some(SessionLease { id, client_name });
        self.push_event_locked(
            &mut data,
            pb::EventSeverity::Info,
            "session_granted",
            format!("exclusive session {id} granted to {event_client_name}"),
            &[("session_id", id.to_string())],
        );
        Ok((id, 0, None))
    }

    pub async fn release(&self, session_id: u32) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        self.backend
            .disable_all()
            .await
            .context("disable while releasing session")?;
        let mut data = self.data.write();
        data.safety.disable_preserving_fault();
        data.session = None;
        data.command = None;
        data.gravity = self.profile.gravity_vector_base_m_s2;
        data.disable_pending = false;
        data.next_disable_retry_at = None;
        self.push_event_locked(
            &mut data,
            pb::EventSeverity::Info,
            "session_released",
            format!("exclusive session {session_id} released; arm disabled"),
            &[("session_id", session_id.to_string())],
        );
        Ok(())
    }

    pub async fn set_mode(&self, session_id: u32, requested: OperatingMode) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        anyhow::ensure!(
            !matches!(requested, OperatingMode::Fault | OperatingMode::Calibrating),
            "mode is controller-owned and cannot be requested"
        );

        let feedback = self.backend.feedback();
        let all_fresh = feedback.all_online_and_fresh();
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

        if requested == OperatingMode::Active {
            if let Some((_, reason)) = self.measured_feedback_fault(&feedback) {
                anyhow::bail!("unsafe measured state blocks mode transition: {reason}");
            }
        }

        let hold_targets = (requested == OperatingMode::Active)
            .then(|| {
                let mut targets = self.hold_targets(&feedback);
                // Validate the eventual gravity target before enabling, even when
                // the startup frame itself contains zero feed-forward.
                // Include firmware gain quantization and the Tff + PD budget
                // before zeroing Tff for a startup ramp and enabling any axis.
                self.motor_targets(&targets)?;
                if self
                    .profile
                    .controller
                    .gravity_startup_slew_rate_nm_s
                    .is_some()
                {
                    // Keep feedback-derived position and PD support during drive
                    // activation; gravity ramps only after the ACTIVE commit.
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

        let hardware_result = match requested {
            OperatingMode::Disabled => self.backend.disable_all().await,
            OperatingMode::Active => {
                let motor_targets = initial_motor_targets.expect("ACTIVE motor targets prepared");
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
            // GRAVITY_COMP has no session-liveliness/deadman contract yet.
            // SafetyState rejects it before dispatch; this branch prevents an
            // accidental future bypass from leaving the arm enabled forever.
            OperatingMode::GravityComp => Err(anyhow::anyhow!(
                "GRAVITY_COMP hardware operation requires a session deadman"
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
        let committed = {
            let mut data = self.data.write();
            if data.closing {
                false
            } else {
                data.safety = next_safety;
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
                true
            }
        };
        if !committed {
            self.backend
                .disable_all()
                .await
                .context("rollback mode transition interrupted by controller shutdown")?;
            anyhow::bail!("controller is shutting down");
        }
        Ok(())
    }

    pub fn submit_trajectory(&self, command: pb::JointTrajectory) -> Result<()> {
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
        self.validate_targets(&targets)?;
        anyhow::ensure!(
            command.t_from_start_ns[0] >= 0,
            "relative setpoint time must be non-negative"
        );
        let duration_ns = command.t_from_start_ns[0] as u64;

        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        anyhow::ensure!(
            data.safety.mode == OperatingMode::Active,
            "joint commands require ACTIVE mode"
        );
        if automatic_gravity_feedforward {
            if let Some(ramp) = &data.gravity_startup_ramp {
                ramp.validate_hold_command(&targets)?;
            }
        }
        let rebase_from_feedback = data
            .command
            .as_ref()
            .and_then(|pending| pending.rebase_from_feedback.clone());
        data.command_generation = data.command_generation.wrapping_add(1);
        data.command = Some(CommandEnvelope {
            generation: data.command_generation,
            targets,
            duration_ns,
            received_at: Instant::now(),
            rebase_from_feedback,
            automatic_gravity_feedforward,
        });
        Ok(())
    }

    pub async fn clear_fault(&self, session_id: u32) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
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

    pub fn set_gravity(&self, session_id: u32, gravity: [f32; 3]) -> Result<()> {
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        anyhow::ensure!(
            self.profile.calibrated,
            "gravity compensation is locked until zero calibration is complete"
        );
        validate_gravity_vector(gravity)?;
        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        anyhow::ensure!(
            session_id != 0
                && data
                    .session
                    .as_ref()
                    .is_some_and(|session| session.id == session_id),
            "request does not hold the exclusive session"
        );
        data.gravity = gravity;
        Ok(())
    }

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
            let feedback = self.backend.feedback();
            let now_ns = self.monotonic_ns();
            {
                self.data.write().feedback = feedback.clone();
            }
            let mode = self.data.read().safety.mode;
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
                if let Some((code, reason)) = self.measured_feedback_fault(&feedback) {
                    self.fault_and_disable(code, reason).await;
                    continue;
                }
            }

            match mode {
                OperatingMode::Active => {
                    let command = self.data.read().command.clone();
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
                    }
                    let mut targets = startup_hold.unwrap_or_else(|| interpolator.sample(now_ns));
                    self.apply_command_gravity_feedforward(
                        &mut targets,
                        &feedback,
                        command.automatic_gravity_feedforward,
                        now_ns,
                    );
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
                                let _gate = self.mode_gate.lock().await;
                                if self.is_closing() {
                                    break;
                                }
                                if self.data.read().safety.mode != OperatingMode::Active {
                                    continue;
                                }
                                self.backend.set_targets(targets).await
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
                    self.fault_and_disable(
                        FAULT_COMMAND,
                        "GRAVITY_COMP mode is unsupported without a session deadman",
                    )
                    .await;
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
                        let _gate = self.mode_gate.lock().await;
                        if self.fault_disable_retry_due() {
                            self.try_confirmed_fault_disable().await;
                        }
                    }
                }
                OperatingMode::Disabled | OperatingMode::Calibrating => {}
            }
        }
    }

    pub async fn shutdown(&self) -> Result<()> {
        self.begin_shutdown();
        // set_mode/release/clear_fault and fault-disable all use this same
        // gate.  Acquiring it after closing is latched drains any operation
        // that passed admission earlier and rejects every queued operation.
        let _gate = self.mode_gate.lock().await;
        let shutdown_result = self.backend.shutdown().await;
        let mut data = self.data.write();
        data.safety.disable_preserving_fault();
        data.command = None;
        data.session = None;
        data.gravity = self.profile.gravity_vector_base_m_s2;
        data.initialized = false;
        data.disable_pending = shutdown_result.is_err();
        data.next_disable_retry_at = None;
        shutdown_result
    }

    pub fn joint_state_proto(&self) -> pb::JointState {
        let feedback = self.data.read().feedback.clone();
        let (mut q, dq, tau, temp) = self.ros_joint_state_full(&feedback);
        // ros2_control seeds its initial command from this state. Publish the
        // same numerical endpoint representation as our activation hold, so
        // an echoed hold cannot fail the otherwise strict command validator.
        // Raw feedback remains unchanged for dynamics and fault decisions.
        for (position, joint) in q.iter_mut().zip(&self.profile.joints) {
            *position = canonical_feedback_position(*position, joint);
        }
        pb::JointState {
            header: Some(self.header()),
            q,
            dq,
            tau_est: tau,
            temp,
        }
    }

    pub fn driver_state_proto(&self) -> pb::DriverState {
        let data = self.data.read();
        let feedback_fresh = data.feedback.all_online_and_fresh();
        let command_age_s = data.command.as_ref().map_or(f32::INFINITY, |command| {
            command.received_at.elapsed().as_secs_f32()
        });
        pb::DriverState {
            header: Some(self.header()),
            mode: data.safety.mode as i32,
            session_owned: data.session.is_some(),
            profile_valid: self.profile.validated && data.initialized,
            calibrated: self.profile.calibrated,
            all_motors_online: data.feedback.joints.iter().all(|joint| joint.online),
            feedback_fresh,
            fault_latched: data.safety.mode == OperatingMode::Fault,
            fault_code: data.safety.fault_code,
            fault_reason: data.safety.fault_reason.clone(),
            command_age_s,
            feedback_age_s: data
                .feedback
                .captured_at
                .map_or(f32::INFINITY, |stamp| stamp.elapsed().as_secs_f32()),
            motors: data.motors.iter().map(motor_proto).collect(),
        }
    }

    pub fn robot_status_proto(&self) -> pb::RobotStatus {
        let data = self.data.read();
        let robot_mode = match data.safety.mode {
            OperatingMode::Fault => pb::RobotMode::FatalError,
            OperatingMode::Disabled if data.session.is_none() => pb::RobotMode::Standby,
            _ => pb::RobotMode::Running,
        };
        pb::RobotStatus {
            header: Some(self.header()),
            mode: robot_mode as i32,
            session_holder: data.session.as_ref().map_or(0, |session| session.id),
        }
    }

    pub fn mode(&self) -> OperatingMode {
        self.data.read().safety.mode
    }

    pub fn event_log_proto(&self) -> pb::EventLog {
        pb::EventLog {
            events: self.data.read().events.iter().cloned().collect(),
        }
    }

    pub fn events_after(&self, after_seq: u64) -> Vec<pb::Event> {
        self.data
            .read()
            .events
            .iter()
            .filter(|event| {
                event
                    .header
                    .as_ref()
                    .is_some_and(|header| header.seq > after_seq)
            })
            .cloned()
            .collect()
    }

    fn require_session(&self, session_id: u32) -> Result<()> {
        let data = self.data.read();
        anyhow::ensure!(
            session_id != 0
                && data
                    .session
                    .as_ref()
                    .is_some_and(|session| session.id == session_id),
            "request does not hold the exclusive session"
        );
        Ok(())
    }

    fn ensure_accepting_requests(&self) -> Result<()> {
        anyhow::ensure!(!self.data.read().closing, "controller is shutting down");
        Ok(())
    }

    fn hold_targets(&self, feedback: &FeedbackSnapshot) -> Vec<RosTarget> {
        let mut targets: Vec<_> = feedback
            .joints
            .iter()
            .zip(&self.profile.joints)
            .map(|(state, joint)| {
                let measured = motor_position_to_ros(state.position_rev, joint);
                let position_rad = canonical_feedback_position(measured, joint);
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

    fn apply_gravity_feedforward(&self, targets: &mut [RosTarget], feedback: &FeedbackSnapshot) {
        let measured_q = self.ros_joint_state(feedback).0;
        let gravity = self.data.read().gravity;
        let tau = self.dynamics.gravity_torque_with(&measured_q, gravity);
        for ((target, torque_nm), joint) in targets.iter_mut().zip(tau).zip(&self.profile.joints) {
            target.torque_nm =
                joint.clamp_gravity_feedforward(torque_nm * joint.gravity_compensation_scale);
        }
    }

    fn apply_command_gravity_feedforward(
        &self,
        targets: &mut [RosTarget],
        feedback: &FeedbackSnapshot,
        automatic: bool,
        now_ns: u64,
    ) {
        if !automatic {
            // Explicit tau_ff belongs to the client, including an explicit
            // zero vector. It bypasses both the gravity clamp and startup ramp.
            self.data.write().gravity_startup_ramp = None;
            return;
        }
        self.apply_gravity_feedforward(targets, feedback);
        let mut data = self.data.write();
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
                    &mut data,
                    pb::EventSeverity::Info,
                    "gravity_ready",
                    "gravity startup ramp complete; position motion is now accepted".into(),
                    &[],
                );
            }
        }
    }

    fn validate_targets(&self, targets: &[RosTarget]) -> Result<()> {
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

    fn motor_targets(
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

    fn ros_joint_state(&self, feedback: &FeedbackSnapshot) -> (Vec<f32>, Vec<f32>) {
        let (q, dq, _, _) = self.ros_joint_state_full(feedback);
        (q, dq)
    }

    fn ros_joint_state_full(
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

    fn measured_feedback_fault(&self, feedback: &FeedbackSnapshot) -> Option<(u32, String)> {
        for (state, joint) in feedback.joints.iter().zip(&self.profile.joints) {
            let position_rad = motor_position_to_ros(state.position_rev, joint);
            let measured_lower = joint.limits.measured_position_lower_rad();
            let measured_upper = joint.limits.measured_position_upper_rad();
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
            if !velocity_rad_s.is_finite()
                || velocity_rad_s.abs()
                    > joint.limits.velocity_rad_s + MEASURED_VELOCITY_EPSILON_RAD_S
            {
                return Some((
                    FAULT_MEASURED_OVERSPEED,
                    format!(
                        "{} measured velocity {:.6} rad/s exceeds {:.6} rad/s",
                        joint.name, velocity_rad_s, joint.limits.velocity_rad_s
                    ),
                ));
            }
        }
        None
    }

    async fn watchdog_fault_hold_and_disable(&self, feedback: &FeedbackSnapshot) {
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

        let _gate = self.mode_gate.lock().await;
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

    async fn fault_and_disable(&self, code: u32, reason: impl Into<String>) {
        self.latch_whole_arm_fault(code, reason);
        let _gate = self.mode_gate.lock().await;
        if self.data.read().disable_pending {
            self.try_confirmed_fault_disable().await;
        }
    }

    fn latch_whole_arm_fault(&self, code: u32, reason: impl Into<String>) {
        let reason = reason.into();
        let mut newly_latched = false;
        {
            let mut data = self.data.write();
            if data.safety.mode != OperatingMode::Fault {
                data.safety.latch_fault(code, reason.clone());
                data.command = None;
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

    fn fault_disable_retry_due(&self) -> bool {
        let data = self.data.read();
        data.disable_pending
            && data
                .next_disable_retry_at
                .is_none_or(|deadline| Instant::now() >= deadline)
    }

    async fn try_confirmed_fault_disable(&self) {
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

    fn push_event_locked(
        &self,
        data: &mut RuntimeData,
        severity: pb::EventSeverity,
        code: &str,
        text: String,
        kv: &[(&str, String)],
    ) {
        let seq = data.next_event_seq;
        data.next_event_seq = data.next_event_seq.wrapping_add(1).max(1);
        let event = pb::Event {
            header: Some(pb::Header {
                seq,
                stamp_ns: self.monotonic_ns() as i64,
                sync_ns: None,
            }),
            severity: severity as i32,
            code: code.into(),
            text,
            kv: kv
                .iter()
                .map(|(key, value)| ((*key).to_string(), value.clone()))
                .collect::<HashMap<_, _>>(),
        };
        if data.events.len() == EVENT_CAPACITY {
            data.events.pop_front();
        }
        data.events.push_back(event);
    }

    fn header(&self) -> pb::Header {
        pb::Header {
            seq: 0,
            stamp_ns: self.monotonic_ns() as i64,
            sync_ns: None,
        }
    }

    fn monotonic_ns(&self) -> u64 {
        self.started_at.elapsed().as_nanos() as u64
    }
}

fn canonical_feedback_position(measured: f32, joint: &crate::profile::JointProfile) -> f32 {
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

fn vector_or(values: &[f32], default: f32) -> Result<Vec<f32>> {
    if values.is_empty() {
        return Ok(vec![default; DOF]);
    }
    anyhow::ensure!(
        values.len() == DOF && values.iter().all(|value| value.is_finite()),
        "trajectory vector must be empty or contain six finite values"
    );
    Ok(values.to_vec())
}

fn motor_proto(motor: &MotorIdentitySnapshot) -> pb::MotorIdentity {
    pb::MotorIdentity {
        node_id: motor.node_id as u32,
        vendor_id: motor.vendor_id,
        product_code: motor.product_code,
        revision: motor.revision,
        serial_number: motor.serial_number,
        model: motor.model.clone(),
        identity_verified: motor.identity_verified,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::MockBackend;
    use crate::profile::{
        BusProfile, BusTransport, ControllerProfile, IdentityFingerprint, JointLimits,
        JointProfile, JOINT_NAMES,
    };
    use async_trait::async_trait;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    struct ShutdownOrderBackend {
        feedback: FeedbackSnapshot,
        enabled: AtomicBool,
        enable_calls: AtomicUsize,
        shutdown_calls: AtomicUsize,
        shutdown_complete: AtomicBool,
        enable_after_shutdown: AtomicBool,
        reject_nonzero_feedforward: AtomicBool,
        disable_delay_ms: AtomicUsize,
        disable_in_progress: AtomicBool,
        targets_during_disable: AtomicBool,
    }

    impl ShutdownOrderBackend {
        fn new() -> Self {
            let mut feedback = FeedbackSnapshot::default();
            for joint in &mut feedback.joints {
                joint.online = true;
                joint.fresh = true;
            }
            feedback.captured_at = Some(Instant::now());
            Self {
                feedback,
                enabled: AtomicBool::new(false),
                enable_calls: AtomicUsize::new(0),
                shutdown_calls: AtomicUsize::new(0),
                shutdown_complete: AtomicBool::new(false),
                enable_after_shutdown: AtomicBool::new(false),
                reject_nonzero_feedforward: AtomicBool::new(false),
                disable_delay_ms: AtomicUsize::new(0),
                disable_in_progress: AtomicBool::new(false),
                targets_during_disable: AtomicBool::new(false),
            }
        }
    }

    #[async_trait]
    impl MotorBackend for ShutdownOrderBackend {
        fn validate_targets(&self, targets: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
            anyhow::ensure!(
                !self.reject_nonzero_feedforward.load(Ordering::Acquire)
                    || targets.iter().all(|target| target.torque_nm == 0.0),
                "test motor has insufficient PD/gravity headroom"
            );
            Ok(())
        }

        async fn discover(&self, _refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
            Ok(Vec::new())
        }

        async fn initialize_disabled(&self) -> Result<()> {
            self.enabled.store(false, Ordering::Release);
            Ok(())
        }

        async fn enable_compressed_mit(
            &self,
            _initial_targets: [crate::conversion::MotorTarget; DOF],
        ) -> Result<()> {
            self.enable_calls.fetch_add(1, Ordering::AcqRel);
            if self.shutdown_complete.load(Ordering::Acquire) {
                self.enable_after_shutdown.store(true, Ordering::Release);
            }
            self.enabled.store(true, Ordering::Release);
            Ok(())
        }

        async fn set_targets(&self, _targets: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
            if self.disable_in_progress.load(Ordering::Acquire) {
                self.targets_during_disable.store(true, Ordering::Release);
                anyhow::bail!("test drive is being disabled");
            }
            Ok(())
        }

        async fn disable_all(&self) -> Result<()> {
            self.disable_in_progress.store(true, Ordering::Release);
            self.enabled.store(false, Ordering::Release);
            tokio::time::sleep(Duration::from_millis(
                self.disable_delay_ms.load(Ordering::Acquire) as u64,
            ))
            .await;
            self.disable_in_progress.store(false, Ordering::Release);
            Ok(())
        }

        async fn shutdown(&self) -> Result<()> {
            self.shutdown_calls.fetch_add(1, Ordering::AcqRel);
            self.enabled.store(false, Ordering::Release);
            self.shutdown_complete.store(true, Ordering::Release);
            Ok(())
        }

        async fn clear_faults(&self) -> Result<()> {
            Ok(())
        }

        fn feedback(&self) -> FeedbackSnapshot {
            self.feedback.clone()
        }

        fn transport_failed(&self) -> bool {
            false
        }
    }

    fn runtime_and_backend_for_safety_test() -> (ArmRuntime, Arc<MockBackend>) {
        let backend = Arc::new(MockBackend::new());
        (
            runtime_with_backend(backend.clone() as Arc<dyn MotorBackend>),
            backend,
        )
    }

    fn runtime_with_backend(backend: Arc<dyn MotorBackend>) -> ArmRuntime {
        let profile = Arc::new(HardwareProfile {
            schema_version: 2,
            validated: true,
            calibrated: true,
            robot_prefix: "hexmeow/test/arm0".into(),
            urdf_path: "unused-by-this-test".into(),
            gravity_vector_base_m_s2: [0.0, 0.0, -9.81],
            tip_payload: None,
            bus: BusProfile {
                protocol: crate::profile::MotorProtocol::Cia402,
                transport: BusTransport::GsUsb,
                interface: String::new(),
                channel: 0,
                adapter_vid: 1,
                adapter_pid: 2,
                heartbeat_node_id: 16,
                hardware_timestamp: false,
                direct_joint_mapping: false,
                auxiliary_node_ids: Vec::new(),
                expected_link: None,
            },
            controller: ControllerProfile {
                loop_hz: 1000,
                state_publish_hz: 100,
                discovery_timeout_ms: 1000,
                feedback_timeout_ms: 100,
                command_watchdog_ms: 100,
                gravity_startup_slew_rate_nm_s: None,
            },
            joints: JOINT_NAMES
                .iter()
                .enumerate()
                .map(|(index, name)| JointProfile {
                    name: (*name).into(),
                    node_id: (index + 1) as u8,
                    identity: IdentityFingerprint::test_value(),
                    direction: 1,
                    zero_offset_rad: 0.0,
                    torque_scale: 1.0,
                    gravity_compensation_scale: 1.0,
                    gravity_compensation_limit_nm: None,
                    torque_permille: 100,
                    kp_kd_torque_permille: 100,
                    limits: JointLimits {
                        position_lower_rad: -2.0,
                        position_upper_rad: 2.0,
                        measured_position_margin_rad: 0.0,
                        velocity_rad_s: 0.2,
                        acceleration_rad_s2: 0.1,
                        torque_nm: 1.0,
                    },
                    default_kp: 5.0,
                    default_kd: 0.8,
                })
                .collect(),
        });
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 0.0, 1.0]); DOF],
            vec![(0.0, [0.0; 3]); DOF],
            [0.0, 0.0, -9.81],
        );
        ArmRuntime::new(profile, backend, dynamics)
    }

    fn runtime_for_safety_test() -> ArmRuntime {
        runtime_and_backend_for_safety_test().0
    }

    fn targets(positions: [f32; DOF]) -> Vec<RosTarget> {
        positions
            .into_iter()
            .map(|position_rad| RosTarget {
                position_rad,
                ..Default::default()
            })
            .collect()
    }

    fn streaming_command(session_id: u32, tau_ff: Vec<f32>) -> pb::JointTrajectory {
        pb::JointTrajectory {
            header: None,
            session_id,
            points: vec![pb::JointSetpoint {
                q: vec![0.0; DOF],
                dq: vec![],
                kp: vec![],
                kd: vec![],
                tau_ff,
            }],
            t_from_start_ns: vec![10_000_000],
            on_timeout: pb::TimeoutBehavior::Fault as i32,
        }
    }

    fn feedback_from_ros(
        runtime: &ArmRuntime,
        positions_rad: [f32; DOF],
        velocities_rad_s: [f32; DOF],
    ) -> FeedbackSnapshot {
        let mut feedback = FeedbackSnapshot::default();
        for index in 0..DOF {
            let motor = ros_target_to_motor(
                RosTarget {
                    position_rad: positions_rad[index],
                    velocity_rad_s: velocities_rad_s[index],
                    ..Default::default()
                },
                &runtime.profile.joints[index],
            );
            feedback.joints[index].position_rev = motor.position_rev;
            feedback.joints[index].velocity_rev_s = motor.velocity_rev_s;
            feedback.joints[index].online = true;
            feedback.joints[index].fresh = true;
        }
        feedback.captured_at = Some(Instant::now());
        feedback
    }

    #[tokio::test]
    async fn intentional_disable_excludes_streaming_without_latching_transport_fault() {
        let backend = Arc::new(ShutdownOrderBackend::new());
        backend.disable_delay_ms.store(30, Ordering::Release);
        let runtime = Arc::new(runtime_with_backend(
            backend.clone() as Arc<dyn MotorBackend>
        ));
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("disable-stream-race".into()).unwrap();
        runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .unwrap();
        let task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::sleep(Duration::from_millis(5)).await;
        runtime
            .set_mode(session, OperatingMode::Disabled)
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(5)).await;
        assert!(!backend.targets_during_disable.load(Ordering::Acquire));
        assert_eq!(runtime.data.read().safety.mode, OperatingMode::Disabled);
        runtime.begin_shutdown();
        task.await.unwrap();
        runtime.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn shutdown_latch_rejects_a_pending_active_transition_before_final_disarm() {
        let backend = Arc::new(ShutdownOrderBackend::new());
        let runtime = Arc::new(runtime_with_backend(
            backend.clone() as Arc<dyn MotorBackend>
        ));
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("shutdown-race-test".into()).unwrap();

        // Keep set_mode pending at the same serialization point used by final
        // backend shutdown.  Closing is latched while both operations wait.
        let held_gate = runtime.mode_gate.lock().await;
        let pending_runtime = runtime.clone();
        let mode_task = tokio::spawn(async move {
            pending_runtime
                .set_mode(session_id, OperatingMode::Active)
                .await
        });
        tokio::task::yield_now().await;

        assert!(runtime.begin_shutdown());
        let shutdown_runtime = runtime.clone();
        let shutdown_task = tokio::spawn(async move { shutdown_runtime.shutdown().await });
        drop(held_gate);

        let mode_error = mode_task.await.unwrap().unwrap_err();
        assert!(mode_error.to_string().contains("shutting down"));
        shutdown_task.await.unwrap().unwrap();

        assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
        assert_eq!(backend.shutdown_calls.load(Ordering::Acquire), 1);
        assert!(!backend.enabled.load(Ordering::Acquire));
        assert!(!backend.enable_after_shutdown.load(Ordering::Acquire));
    }

    #[tokio::test]
    async fn closing_rejects_all_mutating_api_and_control_loop_is_joinable() {
        let (runtime, _) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("closing-api-test".into()).unwrap();
        let loop_task = tokio::spawn(runtime.clone().run_control_loop());

        assert!(runtime.begin_shutdown());
        assert!(!runtime.begin_shutdown(), "shutdown latch must be one-way");
        tokio::time::timeout(Duration::from_millis(100), loop_task)
            .await
            .expect("control loop ignored shutdown cancellation")
            .expect("control loop task panicked");

        let assert_closing = |error: anyhow::Error| {
            assert!(
                error.to_string().contains("shutting down"),
                "unexpected API rejection: {error:#}"
            );
        };
        assert_closing(runtime.acquire("late-client".into()).unwrap_err());
        assert_closing(
            runtime
                .set_mode(session_id, OperatingMode::Active)
                .await
                .unwrap_err(),
        );
        assert_closing(
            runtime
                .submit_trajectory(streaming_command(session_id, vec![]))
                .unwrap_err(),
        );
        assert_closing(runtime.clear_fault(session_id).await.unwrap_err());
        assert_closing(runtime.release(session_id).await.unwrap_err());
        assert_closing(runtime.discover(true).await.unwrap_err());
        assert_closing(
            runtime
                .set_gravity(session_id, [0.0, 0.0, -9.81])
                .unwrap_err(),
        );

        runtime.shutdown().await.unwrap();
        let state = runtime.driver_state_proto();
        assert!(!state.session_owned);
        assert!(!state.profile_valid);
        assert_eq!(runtime.mode(), OperatingMode::Disabled);
    }

    #[test]
    fn measured_feedback_accepts_configured_position_and_velocity_limits() {
        let runtime = runtime_for_safety_test();
        let feedback = feedback_from_ros(
            &runtime,
            [-2.0, 2.0, -2.0, 2.0, -2.0, 2.0],
            [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
        );

        assert!(runtime.measured_feedback_fault(&feedback).is_none());
    }

    #[test]
    fn measured_position_outside_limit_reports_joint_and_position_fault_code() {
        let runtime = runtime_for_safety_test();
        let mut positions = [0.0; DOF];
        positions[1] = 2.01;
        let feedback = feedback_from_ros(&runtime, positions, [0.0; DOF]);

        let (code, reason) = runtime.measured_feedback_fault(&feedback).unwrap();
        assert_eq!(code, FAULT_MEASURED_POSITION_LIMIT);
        assert!(reason.contains("joint_2 measured position"));
        assert!(reason.contains("feedback envelope [-2.000000, 2.000000]"));
        assert!(reason.contains("command limits remain [-2.000000, 2.000000]"));
    }

    #[test]
    fn measured_position_margin_never_widens_command_validation() {
        let mut runtime = runtime_for_safety_test();
        Arc::make_mut(&mut runtime.profile).joints[1]
            .limits
            .measured_position_margin_rad = 0.001;

        let mut positions = [0.0; DOF];
        positions[1] = 2.0008;
        let feedback = feedback_from_ros(&runtime, positions, [0.0; DOF]);
        assert!(runtime.measured_feedback_fault(&feedback).is_none());

        positions[1] = 2.0012;
        let feedback = feedback_from_ros(&runtime, positions, [0.0; DOF]);
        assert_eq!(
            runtime.measured_feedback_fault(&feedback).unwrap().0,
            FAULT_MEASURED_POSITION_LIMIT
        );

        let mut commands = targets([0.0; DOF]);
        commands[1].position_rad = 2.0001;
        let error = runtime.validate_targets(&commands).unwrap_err().to_string();
        assert!(error.contains("joint_2 position exceeds software limit"));
    }

    #[test]
    fn measured_overspeed_reports_joint_and_velocity_fault_code() {
        let runtime = runtime_for_safety_test();
        let mut velocities = [0.0; DOF];
        velocities[4] = -0.21;
        let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocities);

        let (code, reason) = runtime.measured_feedback_fault(&feedback).unwrap();
        assert_eq!(code, FAULT_MEASURED_OVERSPEED);
        assert!(reason.contains("joint_5 measured velocity"));
        assert!(reason.contains("exceeds 0.200000 rad/s"));
    }

    #[test]
    fn non_finite_measured_state_cannot_bypass_limits() {
        let runtime = runtime_for_safety_test();
        let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        feedback.joints[0].position_rev = f32::NAN;
        assert_eq!(
            runtime.measured_feedback_fault(&feedback).unwrap().0,
            FAULT_MEASURED_POSITION_LIMIT
        );

        feedback.joints[0].position_rev = 0.0;
        feedback.joints[0].velocity_rev_s = f32::INFINITY;
        assert_eq!(
            runtime.measured_feedback_fault(&feedback).unwrap().0,
            FAULT_MEASURED_OVERSPEED
        );
    }

    #[tokio::test]
    async fn uncommissioned_continuous_modes_never_enable_the_backend() {
        for requested in [OperatingMode::Passive, OperatingMode::GravityComp] {
            let (runtime, backend) = runtime_and_backend_for_safety_test();
            runtime.initialize().await.unwrap();
            let (session_id, _, _) = runtime.acquire("fail-closed-test".into()).unwrap();

            let error = runtime.set_mode(session_id, requested).await.unwrap_err();
            assert!(error.to_string().contains("unavailable"));
            assert_eq!(runtime.mode(), OperatingMode::Disabled);

            // Mock feedback follows setpoints only while enabled. A post-error
            // setpoint therefore also proves that no backend enable occurred.
            let mut probe = [crate::conversion::MotorTarget::default(); DOF];
            probe[0].position_rev = 0.5;
            backend.set_targets(probe).await.unwrap();
            assert_eq!(backend.feedback().joints[0].position_rev, 0.0);
        }
    }

    #[tokio::test]
    async fn failed_mode_transition_latches_fault_after_successful_rollback() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("mode-failure-test".into()).unwrap();
        backend.fail_next_target_updates(1);

        let error = runtime
            .set_mode(session_id, OperatingMode::Active)
            .await
            .unwrap_err();

        assert!(error
            .to_string()
            .contains("mode transition to Active failed"));
        assert_eq!(runtime.mode(), OperatingMode::Fault);
        assert_eq!(
            runtime.driver_state_proto().fault_code,
            FAULT_MODE_TRANSITION
        );
        assert!(!backend.is_enabled());
        assert!(runtime
            .event_log_proto()
            .events
            .iter()
            .any(|event| event.code == "fault_disable_confirmed"));
    }

    #[tokio::test]
    async fn failed_mode_transition_and_rollback_retries_until_disabled() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("mode-rollback-retry-test".into()).unwrap();
        backend.fail_next_target_updates(1);
        backend.fail_next_disables(1);

        let error = runtime
            .set_mode(session_id, OperatingMode::Active)
            .await
            .unwrap_err();
        assert!(error.to_string().contains("rollback disable failed"));
        assert_eq!(runtime.mode(), OperatingMode::Fault);
        assert!(
            backend.is_enabled(),
            "injected rollback failure must be observable"
        );

        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(500), async {
            while backend.is_enabled() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("fault loop did not retry the failed mode-transition rollback");

        assert!(backend.disable_attempts() >= 2);
        assert_eq!(runtime.mode(), OperatingMode::Fault);
        assert!(runtime
            .event_log_proto()
            .events
            .iter()
            .any(|event| event.code == "fault_disable_confirmed"));

        loop_task.abort();
        let _ = loop_task.await;
    }

    #[tokio::test]
    async fn control_loop_disables_if_an_uncommissioned_mode_is_observed() {
        for requested in [OperatingMode::Passive, OperatingMode::GravityComp] {
            let (runtime, backend) = runtime_and_backend_for_safety_test();
            let runtime = Arc::new(runtime);
            runtime.initialize().await.unwrap();

            // Simulate an internal invariant violation with hardware already
            // enabled; public set_mode cannot create either of these states.
            backend
                .enable_compressed_mit([Default::default(); DOF])
                .await
                .unwrap();
            runtime.data.write().safety.mode = requested;

            let loop_task = tokio::spawn(runtime.clone().run_control_loop());
            tokio::time::timeout(Duration::from_millis(100), async {
                while runtime.mode() != OperatingMode::Fault {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("control loop did not reject uncommissioned mode");
            tokio::time::sleep(Duration::from_millis(5)).await;

            let stopped_position = backend.feedback().joints[0].position_rev;
            let mut probe = [crate::conversion::MotorTarget::default(); DOF];
            probe[0].position_rev = 0.5;
            backend.set_targets(probe).await.unwrap();
            assert_eq!(backend.feedback().joints[0].position_rev, stopped_position);
            assert!(runtime
                .driver_state_proto()
                .fault_reason
                .contains("unsupported"));

            loop_task.abort();
            let _ = loop_task.await;
        }
    }

    #[tokio::test]
    async fn control_loop_latches_and_disables_on_measured_overspeed() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("overspeed-test".into()).unwrap();
        runtime
            .set_mode(session_id, OperatingMode::Active)
            .await
            .unwrap();

        let mut overspeed = [crate::conversion::MotorTarget::default(); DOF];
        overspeed[0].velocity_rev_s = 1.0;
        backend.set_targets(overspeed).await.unwrap();

        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(100), async {
            while runtime.mode() != OperatingMode::Fault {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("control loop did not latch measured overspeed");
        // Fault state is published before the best-effort hardware disable.
        tokio::time::sleep(Duration::from_millis(5)).await;

        let state = runtime.driver_state_proto();
        assert_eq!(state.fault_code, FAULT_MEASURED_OVERSPEED);
        assert!(state.fault_reason.contains("joint_1 measured velocity"));

        let stopped_position = backend.feedback().joints[0].position_rev;
        let mut post_fault_target = [crate::conversion::MotorTarget::default(); DOF];
        post_fault_target[0].position_rev = 0.5;
        backend.set_targets(post_fault_target).await.unwrap();
        assert_eq!(
            backend.feedback().joints[0].position_rev,
            stopped_position,
            "fault path must disable the backend before returning"
        );

        loop_task.abort();
        let _ = loop_task.await;
    }

    #[tokio::test]
    async fn command_watchdog_holds_latest_feedback_and_retries_until_disabled() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("watchdog-contract-test".into()).unwrap();
        runtime
            .set_mode(session_id, OperatingMode::Active)
            .await
            .unwrap();

        let mut command = streaming_command(session_id, vec![]);
        command.points[0].q[0] = 1.0;
        command.t_from_start_ns[0] = 1_000_000_000;
        runtime.submit_trajectory(command).unwrap();
        backend.fail_next_disables(1);

        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(500), async {
            while runtime.mode() != OperatingMode::Fault || backend.is_enabled() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("watchdog did not retry the confirmed disable");

        assert!(backend.disable_attempts() >= 2);
        let feedback = backend.feedback();
        let held = backend.targets();
        for (target, state) in held.iter().zip(feedback.joints) {
            assert!((target.position_rev - state.position_rev).abs() < 1.0e-6);
            assert_eq!(target.velocity_rev_s, 0.0);
        }
        let state = runtime.driver_state_proto();
        assert_eq!(state.fault_code, FAULT_COMMAND_WATCHDOG);
        assert!(state.fault_reason.contains("confirmed disable"));
        let events = runtime.event_log_proto().events;
        assert!(events
            .iter()
            .any(|event| event.code == "watchdog_feedback_hold_applied"));
        assert!(events
            .iter()
            .any(|event| event.code == "fault_disable_confirmed"));

        loop_task.abort();
        let _ = loop_task.await;
    }

    #[tokio::test]
    async fn releasing_and_reacquiring_a_session_does_not_clear_a_latched_fault() {
        let runtime = runtime_for_safety_test();
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("first-client".into()).unwrap();
        {
            let mut data = runtime.data.write();
            data.safety
                .latch_fault(FAULT_FEEDBACK_TIMEOUT, "motor feedback timeout/offline");
        }

        runtime.release(session_id).await.unwrap();

        let state = runtime.driver_state_proto();
        assert_eq!(runtime.mode(), OperatingMode::Fault);
        assert!(state.fault_latched);
        assert_eq!(state.fault_code, FAULT_FEEDBACK_TIMEOUT);
        assert_eq!(state.fault_reason, "motor feedback timeout/offline");
        assert!(!state.session_owned);

        let (recovery_session, _, _) = runtime.acquire("recovery-client".into()).unwrap();
        let error = runtime
            .set_mode(recovery_session, OperatingMode::Active)
            .await
            .unwrap_err();
        assert!(error.to_string().contains("fault is latched"));

        runtime.clear_fault(recovery_session).await.unwrap();
        assert_eq!(runtime.mode(), OperatingMode::Disabled);
        assert!(!runtime.driver_state_proto().fault_latched);
        runtime.release(recovery_session).await.unwrap();
    }

    #[test]
    fn empty_tau_ff_delegates_gravity_but_explicit_zero_does_not() {
        let runtime = runtime_for_safety_test();
        let (session_id, _, _) = runtime.acquire("gravity-policy-test".into()).unwrap();
        runtime.data.write().safety.mode = OperatingMode::Active;

        runtime
            .submit_trajectory(streaming_command(session_id, vec![]))
            .unwrap();
        assert!(
            runtime
                .data
                .read()
                .command
                .as_ref()
                .unwrap()
                .automatic_gravity_feedforward
        );

        runtime
            .submit_trajectory(streaming_command(session_id, vec![0.0; DOF]))
            .unwrap();
        assert!(
            !runtime
                .data
                .read()
                .command
                .as_ref()
                .unwrap()
                .automatic_gravity_feedforward
        );
    }

    #[test]
    fn automatic_gravity_feedforward_uses_per_joint_profile_scale() {
        let template = runtime_for_safety_test();
        let mut profile = (*template.profile).clone();
        profile.gravity_vector_base_m_s2 = [0.0, 0.0, 9.81];
        profile.joints[0].gravity_compensation_scale = 0.25;
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            vec![
                (0.2, [1.0, 0.0, 0.0]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
            ],
            [0.0, 0.0, -9.81],
        );
        let runtime = ArmRuntime::new(Arc::new(profile), Arc::new(MockBackend::new()), dynamics);
        let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        let mut targets = targets([0.0; DOF]);

        runtime.apply_gravity_feedforward(&mut targets, &feedback);

        // The profile's +Z gravity reverses the model's built-in -Z default;
        // per-axis scaling is applied only after that vector-based G(q).
        assert!((targets[0].torque_nm - (1.962 * 0.25)).abs() < 1.0e-5);
        assert!(targets[1..]
            .iter()
            .all(|target| target.torque_nm.abs() < 1.0e-6));
    }

    fn runtime_with_startup_gravity() -> (ArmRuntime, Arc<MockBackend>) {
        let template = runtime_for_safety_test();
        let mut profile = (*template.profile).clone();
        profile.gravity_vector_base_m_s2 = [0.0, 0.0, 9.81];
        profile.controller.gravity_startup_slew_rate_nm_s = Some(5.0);
        profile.joints[0].gravity_compensation_scale = 0.25;
        profile.joints[0].gravity_compensation_limit_nm = Some(0.3);
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            vec![
                (0.2, [1.0, 0.0, 0.0]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
            ],
            [0.0, 0.0, -9.81],
        );
        let backend = Arc::new(MockBackend::new());
        (
            ArmRuntime::new(Arc::new(profile), backend.clone(), dynamics),
            backend,
        )
    }

    #[tokio::test]
    async fn active_starts_with_zero_gravity_then_tracks_directly_after_bounded_ramp() {
        let (runtime, backend) = runtime_with_startup_gravity();
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("startup-gravity".into()).unwrap();
        runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .unwrap();
        assert!(backend
            .feedback()
            .joints
            .iter()
            .all(|joint| joint.torque_nm == 0.0));
        let start = runtime
            .data
            .read()
            .gravity_startup_ramp
            .as_ref()
            .unwrap()
            .last_tick_ns;
        let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        let mut command = targets([0.0; DOF]);
        runtime.apply_command_gravity_feedforward(
            &mut command,
            &feedback,
            true,
            start + 20_000_000,
        );
        assert!((command[0].torque_nm - 0.1).abs() < 1.0e-6);
        assert!(runtime.data.read().gravity_startup_ramp.is_some());
        runtime.apply_command_gravity_feedforward(
            &mut command,
            &feedback,
            true,
            start + 80_000_000,
        );
        assert!((command[0].torque_nm - 0.3).abs() < 1.0e-6);
        assert!(runtime.data.read().gravity_startup_ramp.is_none());

        let mut changed_q = [0.0; DOF];
        changed_q[0] = std::f32::consts::FRAC_PI_2;
        let changed_feedback = feedback_from_ros(&runtime, changed_q, [0.0; DOF]);
        runtime.apply_command_gravity_feedforward(
            &mut command,
            &changed_feedback,
            true,
            start + 81_000_000,
        );
        assert!(
            command[0].torque_nm.abs() < 1.0e-5,
            "settled gravity must not lag feedback behind a continuous slew limiter"
        );

        runtime
            .set_mode(session, OperatingMode::Disabled)
            .await
            .unwrap();
        runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .unwrap();
        assert_eq!(
            runtime
                .data
                .read()
                .gravity_startup_ramp
                .as_ref()
                .unwrap()
                .output_nm,
            [0.0; DOF]
        );
    }

    #[tokio::test]
    async fn startup_checks_eventual_firmware_torque_budget_before_any_enable() {
        let (template, _) = runtime_with_startup_gravity();
        let backend = Arc::new(ShutdownOrderBackend::new());
        backend
            .reject_nonzero_feedforward
            .store(true, Ordering::Release);
        let runtime = ArmRuntime::new(template.profile, backend.clone(), template.dynamics);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("insufficient-budget".into()).unwrap();
        let error = runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .unwrap_err();
        assert!(error.to_string().contains("headroom"));
        assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
        assert!(!backend.enabled.load(Ordering::Acquire));
    }

    #[test]
    fn feedback_hold_snaps_only_endpoint_roundoff_and_external_commands_remain_strict() {
        let template = runtime_for_safety_test();
        let mut profile = (*template.profile).clone();
        let joint = &mut profile.joints[1];
        joint.direction = -1;
        joint.zero_offset_rad = -0.001_000_664_2;
        joint.limits.position_lower_rad = -1.57;
        joint.limits.position_upper_rad = 2.09;
        joint.limits.measured_position_margin_rad = 0.01;
        let runtime = ArmRuntime::new(
            Arc::new(profile),
            Arc::new(MockBackend::new()),
            template.dynamics,
        );
        let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        // Actual GUI node-2 park reference after signed Q8.24 quantization.
        feedback.joints[1].position_rev =
            (0.249714_f64 * 16_777_216.0).round() as f32 / 16_777_216.0;
        let joint = &runtime.profile.joints[1];
        let measured = motor_position_to_ros(feedback.joints[1].position_rev, joint);
        assert!(measured < joint.limits.position_lower_rad);
        assert!(runtime.measured_feedback_fault(&feedback).is_none());
        let hold = runtime.hold_targets(&feedback);
        assert_eq!(hold[1].position_rad, joint.limits.position_lower_rad);
        runtime.validate_targets(&hold).unwrap();
        runtime.data.write().feedback = feedback.clone();
        let published = runtime.joint_state_proto();
        assert_eq!(published.q[1], joint.limits.position_lower_rad);
        let mut echoed_hold = hold.clone();
        echoed_hold[1].position_rad = published.q[1];
        runtime.validate_targets(&echoed_hold).unwrap();
        assert_eq!(
            runtime.data.read().feedback.joints[1].position_rev,
            feedback.joints[1].position_rev
        );
        let mut external = hold.clone();
        external[1].position_rad = measured;
        assert!(runtime.validate_targets(&external).is_err());

        let truly_outside = joint.limits.position_lower_rad - 0.001;
        feedback.joints[1].position_rev = ros_target_to_motor(
            RosTarget {
                position_rad: truly_outside,
                ..Default::default()
            },
            joint,
        )
        .position_rev;
        assert!(
            runtime.measured_feedback_fault(&feedback).is_none(),
            "measurement margin is separate from command authority"
        );
        let hold = runtime.hold_targets(&feedback);
        assert!(hold[1].position_rad < joint.limits.position_lower_rad);
        assert!(runtime.validate_targets(&hold).is_err());
    }

    #[tokio::test]
    async fn startup_accepts_hold_heartbeats_but_rejects_motion_until_gravity_ready() {
        let (runtime, _) = runtime_with_startup_gravity();
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("gravity-ready-gate".into()).unwrap();
        runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .unwrap();
        runtime
            .submit_trajectory(streaming_command(session, vec![]))
            .unwrap();
        let mut motion = streaming_command(session, vec![]);
        motion.points[0].q[0] = 0.01;
        assert!(runtime
            .submit_trajectory(motion.clone())
            .unwrap_err()
            .to_string()
            .contains("gravity_ready"));
        let mut velocity = streaming_command(session, vec![]);
        velocity.points[0].dq = vec![0.01; DOF];
        assert!(runtime.submit_trajectory(velocity).is_err());
        let start = runtime
            .data
            .read()
            .gravity_startup_ramp
            .as_ref()
            .unwrap()
            .last_tick_ns;
        let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        let mut command = targets([0.0; DOF]);
        runtime.apply_command_gravity_feedforward(
            &mut command,
            &feedback,
            true,
            start + 100_000_000,
        );
        assert!(runtime
            .data
            .read()
            .events
            .iter()
            .any(|event| event.code == "gravity_ready"));
        runtime.submit_trajectory(motion).unwrap();
    }

    #[tokio::test]
    async fn startup_zero_frame_does_not_hide_an_unsafe_eventual_gravity_target() {
        let (runtime, backend) = runtime_with_startup_gravity();
        let mut profile = (*runtime.profile).clone();
        profile.joints[0].gravity_compensation_limit_nm = None;
        profile.joints[0].limits.torque_nm = 0.1;
        let runtime = ArmRuntime::new(Arc::new(profile), backend.clone(), runtime.dynamics);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("bad-gravity-ramp".into()).unwrap();
        assert!(runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .unwrap_err()
            .to_string()
            .contains("torque exceeds software limit"));
        assert!(!backend.is_enabled());
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut ramp = GravityStartupRamp::new(0, targets([0.0; DOF]));
            let mut command = targets([0.0; DOF]);
            command[0].torque_nm = invalid;
            assert!(!ramp.apply(&mut command, 5.0, 1_000_000));
            assert!(!command[0].torque_nm.is_finite());
        }
    }

    #[test]
    fn startup_ramp_uses_one_tick_for_all_axes_and_explicit_torque_bypasses_it() {
        let mut ramp = GravityStartupRamp::new(10_000_000, targets([0.0; DOF]));
        let mut command = targets([0.0; DOF]);
        command[0].torque_nm = 1.0;
        command[1].torque_nm = -1.0;
        assert!(!ramp.apply(&mut command, 5.0, 30_000_000));
        assert!((command[0].torque_nm - 0.1).abs() < 1.0e-6);
        assert!((command[1].torque_nm + 0.1).abs() < 1.0e-6);
        command[0].torque_nm = 1.0;
        command[1].torque_nm = -1.0;
        assert!(!ramp.apply(&mut command, 5.0, 30_000_000));
        assert!((command[0].torque_nm - 0.1).abs() < 1.0e-6);

        let (runtime, _) = runtime_with_startup_gravity();
        runtime.data.write().gravity_startup_ramp =
            Some(GravityStartupRamp::new(0, targets([0.0; DOF])));
        let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        let mut explicit = targets([0.0; DOF]);
        explicit[0].torque_nm = -0.7; // Greater than the gravity-only 0.3 Nm clamp.
        runtime.apply_command_gravity_feedforward(&mut explicit, &feedback, false, 1_000_000);
        assert_eq!(explicit[0].torque_nm, -0.7);
        assert!(runtime.data.read().gravity_startup_ramp.is_none());
        runtime.apply_command_gravity_feedforward(&mut explicit, &feedback, true, 2_000_000);
        assert!((explicit[0].torque_nm - 0.3).abs() < 1.0e-6);
    }

    #[tokio::test]
    async fn gravity_override_is_scoped_to_one_exclusive_session() {
        let template = runtime_for_safety_test();
        let mut profile = (*template.profile).clone();
        let profile_gravity = [0.0, 0.0, 9.81];
        profile.gravity_vector_base_m_s2 = profile_gravity;
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            vec![(0.0, [0.0; 3]); DOF],
            [0.0, 0.0, -9.81],
        );
        let runtime = ArmRuntime::new(Arc::new(profile), Arc::new(MockBackend::new()), dynamics);
        assert_eq!(runtime.data.read().gravity, profile_gravity);

        let (first_session, _, _) = runtime.acquire("first-gravity-client".into()).unwrap();
        runtime
            .set_gravity(first_session, [0.0, 0.0, -9.81])
            .unwrap();
        assert_eq!(runtime.data.read().gravity, [0.0, 0.0, -9.81]);
        runtime.release(first_session).await.unwrap();
        assert_eq!(runtime.data.read().gravity, profile_gravity);

        // New-session admission also restores the profile value defensively,
        // even if an earlier internal path left a stale runtime value behind.
        runtime.data.write().gravity = [0.0, -9.81, 0.0];
        let (second_session, _, _) = runtime.acquire("second-gravity-client".into()).unwrap();
        assert_eq!(runtime.data.read().gravity, profile_gravity);
        runtime
            .set_gravity(second_session, [0.0, 9.81, 0.0])
            .unwrap();
        runtime.shutdown().await.unwrap();
        assert_eq!(runtime.data.read().gravity, profile_gravity);
    }

    #[test]
    fn unsupported_multi_point_chunks_are_rejected_instead_of_truncated() {
        let runtime = runtime_for_safety_test();
        let (session_id, _, _) = runtime.acquire("chunk-contract-test".into()).unwrap();
        runtime.data.write().safety.mode = OperatingMode::Active;
        let mut command = streaming_command(session_id, vec![]);
        command.points.push(command.points[0].clone());
        command.t_from_start_ns.push(20_000_000);

        let error = runtime.submit_trajectory(command).unwrap_err();
        assert!(error.to_string().contains("exactly one setpoint"));
    }

    #[test]
    fn only_the_advertised_fault_timeout_behavior_is_accepted() {
        let runtime = runtime_for_safety_test();
        let (session_id, _, _) = runtime.acquire("timeout-contract-test".into()).unwrap();
        runtime.data.write().safety.mode = OperatingMode::Active;

        runtime
            .submit_trajectory(streaming_command(session_id, vec![]))
            .unwrap();
        for behavior in [
            pb::TimeoutBehavior::Unspecified,
            pb::TimeoutBehavior::Hold,
            pb::TimeoutBehavior::RampStop,
            pb::TimeoutBehavior::ShortBrake,
        ] {
            let mut command = streaming_command(session_id, vec![]);
            command.on_timeout = behavior as i32;
            let error = runtime.submit_trajectory(command).unwrap_err();
            assert!(error
                .to_string()
                .contains("only TIMEOUT_BEHAVIOR_FAULT is commissioned"));
        }

        let mut command = streaming_command(session_id, vec![]);
        command.on_timeout = i32::MAX;
        assert!(runtime
            .submit_trajectory(command)
            .unwrap_err()
            .to_string()
            .contains("unknown timeout behavior"));
    }

    #[test]
    fn activation_rebases_from_disabled_feedback() {
        let old_positions = [0.8, 0.6, 0.4, 0.2, -0.2, -0.4];
        let moved_positions = [-0.5, -0.3, -0.1, 0.1, 0.3, 0.5];

        let mut interpolator = Interpolator::hold(targets(old_positions), 0);
        let activation = CommandEnvelope {
            generation: 2,
            targets: targets(moved_positions),
            duration_ns: 0,
            received_at: Instant::now(),
            rebase_from_feedback: Some(targets(moved_positions)),
            automatic_gravity_feedforward: true,
        };

        activation
            .apply_to_interpolator(&mut interpolator, 20_000_000, &[0.2; DOF], &[0.1; DOF])
            .unwrap();

        let first_active_sample = interpolator.sample(20_000_000);
        for (target, expected_position) in first_active_sample.iter().zip(moved_positions) {
            assert!((target.position_rad - expected_position).abs() < 1e-6);
        }
    }

    #[test]
    fn ordinary_zero_duration_command_remains_rate_limited() {
        let start_positions = [0.0; DOF];
        let goal_positions = [1.0; DOF];
        let mut interpolator = Interpolator::hold(targets(start_positions), 0);
        let command = CommandEnvelope {
            generation: 1,
            targets: targets(goal_positions),
            duration_ns: 0,
            received_at: Instant::now(),
            rebase_from_feedback: None,
            automatic_gravity_feedforward: true,
        };

        command
            .apply_to_interpolator(&mut interpolator, 0, &[0.2; DOF], &[0.1; DOF])
            .unwrap();

        for target in interpolator.sample(0) {
            assert_eq!(target.position_rad, 0.0);
        }
        let one_second = interpolator.sample(1_000_000_000);
        assert!(one_second.iter().all(|target| {
            target.position_rad > 0.0
                && target.position_rad < 1.0
                && target.velocity_rad_s.abs() <= 0.2
        }));
        for index in 0..DOF {
            let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(index);
            assert!(maximum_velocity <= 0.2_f32 as f64 * (1.0 + 1.0e-9));
            assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + 1.0e-9));
        }
    }

    #[test]
    fn first_trajectory_preserves_unconsumed_activation_rebase() {
        let old_positions = [-1.0; DOF];
        let feedback_positions = [0.0; DOF];
        let goal_positions = [1.0; DOF];
        let activation = CommandEnvelope {
            generation: 1,
            targets: targets(feedback_positions),
            duration_ns: 0,
            received_at: Instant::now(),
            rebase_from_feedback: Some(targets(feedback_positions)),
            automatic_gravity_feedforward: true,
        };
        let queued_trajectory = CommandEnvelope {
            generation: 2,
            targets: targets(goal_positions),
            duration_ns: 0,
            received_at: Instant::now(),
            rebase_from_feedback: activation.rebase_from_feedback.clone(),
            automatic_gravity_feedforward: true,
        };
        let mut interpolator = Interpolator::hold(targets(old_positions), 0);

        queued_trajectory
            .apply_to_interpolator(&mut interpolator, 0, &[0.2; DOF], &[0.1; DOF])
            .unwrap();

        for target in interpolator.sample(0) {
            assert_eq!(target.position_rad, 0.0);
        }
        let one_second = interpolator.sample(1_000_000_000);
        assert!(one_second.iter().all(|target| {
            target.position_rad > 0.0
                && target.position_rad < 1.0
                && target.velocity_rad_s.abs() <= 0.2
        }));
        for index in 0..DOF {
            let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(index);
            assert!(maximum_velocity <= 0.2_f32 as f64 * (1.0 + 1.0e-9));
            assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + 1.0e-9));
        }
    }
}
