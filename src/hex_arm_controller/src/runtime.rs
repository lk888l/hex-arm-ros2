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
const FAULT_MEASURED_TORQUE_LIMIT: u32 = 0x1009;
const FAULT_MEASURED_TEMPERATURE: u32 = 0x100a;
const DEFAULT_MAX_MEASURED_TEMPERATURE_C: f32 = 70.0;
// Allow only enough margin to absorb f32 unit-conversion roundoff at an exact
// configured limit. These are not operating margins and do not relax commands.
const MEASURED_POSITION_EPSILON_RAD: f32 = 1.0e-4;
const MEASURED_VELOCITY_EPSILON_RAD_S: f32 = 1.0e-4;
const MEASURED_TORQUE_EPSILON_NM: f32 = 1.0e-4;
const EVENT_CAPACITY: usize = 100;
const FAULT_DISABLE_RETRY_PERIOD: Duration = Duration::from_millis(50);
pub const GRAVITY_COMP_LEASE: Duration = Duration::from_millis(500);
const GRAVITY_COMP_DEFAULT_SLEW_NM_S: f32 = 5.0;

#[derive(Debug)]
struct GravityCompLease {
    damping: [f32; DOF],
    renewed_at: Instant,
    sequence: u64,
    ramp: Option<GravityStartupRamp>,
}

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
    /// Terminal pre-disable owner; never reopened, even if settling fails.
    damped_stopping: bool,
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
    gravity_comp: Option<GravityCompLease>,
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
                damped_stopping: false,
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
                gravity_comp: None,
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
        anyhow::ensure!(
            !data.closing && !data.damped_stopping,
            "controller is shutting down"
        );
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
        data.gravity_comp = None;
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
        anyhow::ensure!(
            requested != OperatingMode::GravityComp,
            "GRAVITY_COMP is unavailable through set_mode; use start_gravity_comp with damping and a session deadman"
        );
        self.set_mode_locked(session_id, requested, None).await
    }

    pub async fn start_gravity_comp(&self, session_id: u32, damping: &[f32]) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
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
    async fn set_mode_locked(
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
        let committed = {
            let mut data = self.data.write();
            if data.closing {
                false
            } else {
                data.safety = next_safety;
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
        anyhow::ensure!(!data.damped_stopping, "controller is shutting down");
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
            if self.data.read().damped_stopping {
                // The bounded stop task owns both command generation and safety checks.
                // A failed final disable must still retry without a ROS caller.
                if self.fault_disable_retry_due() {
                    let _gate = self.mode_gate.lock().await;
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
                    let command = {
                        let data = self.data.read();
                        if data.damped_stopping {
                            continue;
                        }
                        data.command.clone()
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
                                if self.data.read().safety.mode != OperatingMode::Active
                                    || self.data.read().damped_stopping
                                {
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
        let command_age_s = data
            .gravity_comp
            .as_ref()
            .map(|lease| lease.renewed_at.elapsed().as_secs_f32())
            .unwrap_or_else(|| {
                data.command.as_ref().map_or(f32::INFINITY, |command| {
                    command.received_at.elapsed().as_secs_f32()
                })
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
        let data = self.data.read();
        anyhow::ensure!(
            !data.closing && !data.damped_stopping,
            "controller is shutting down"
        );
        Ok(())
    }

    /// Own the stream until unloaded and settled at the fold, then confirm disable.
    /// API admission closes without terminating the monitoring/publication tasks.
    pub async fn damped_stop(&self, session_id: u32) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
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

    fn check_damping_feedback(&self, feedback: &FeedbackSnapshot) -> Result<()> {
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

    async fn run_damping(
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

    // false denotes lease expiry; other output/validation failures are errors.
    async fn gravity_comp_tick(&self, feedback: &FeedbackSnapshot, now_ns: u64) -> Result<bool> {
        let _gate = self.mode_gate.lock().await;
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

    fn gravity_comp_targets(
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
                position_rad: hand_guiding_position_target(
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
        self.apply_gravity_only(targets, feedback);
        for (target, joint) in targets.iter_mut().zip(&self.profile.joints) {
            target.torque_nm += joint.motion_feedforward_nm(target.velocity_rad_s);
        }
    }

    fn apply_gravity_only(&self, targets: &mut [RosTarget], feedback: &FeedbackSnapshot) {
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
        // Ordinary position-control checks retain the configured joint limits.
        self.measured_feedback_fault_in_mode(feedback, OperatingMode::Active)
    }

    fn measured_feedback_fault_in_mode(
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

fn hand_guiding_position_target(
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
        // Only the zero-Kp MIT position field is bounded. Gravity, damping and
        // measured-state publication continue to use the actual feedback.
        measured.clamp(lower, upper)
    } else {
        // Never hide feedback outside the accepted envelope or non-finite data.
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

    struct DampingBackend {
        inner: ShutdownOrderBackend,
        state: RwLock<FeedbackSnapshot>,
        last: RwLock<Option<[crate::conversion::MotorTarget; DOF]>>,
        fall_to_fold: bool,
        fail_disable: AtomicBool,
    }

    impl DampingBackend {
        fn new(fall_to_fold: bool) -> Self {
            let inner = ShutdownOrderBackend::new();
            let mut state = inner.feedback.clone();
            for (f, q) in state
                .joints
                .iter_mut()
                .zip(crate::startup_recipe::RECIPE.ready())
            {
                f.position_rev = q / std::f32::consts::TAU;
            }
            Self {
                inner,
                state: RwLock::new(state),
                last: RwLock::new(None),
                fall_to_fold,
                fail_disable: AtomicBool::new(false),
            }
        }
    }

    #[async_trait]
    impl MotorBackend for DampingBackend {
        async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
            self.inner.discover(refresh).await
        }
        async fn initialize_disabled(&self) -> Result<()> {
            self.inner.initialize_disabled().await
        }
        async fn enable_compressed_mit(
            &self,
            t: [crate::conversion::MotorTarget; DOF],
        ) -> Result<()> {
            self.inner.enable_compressed_mit(t).await
        }
        async fn set_targets(&self, t: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
            self.inner.set_targets(t).await?;
            *self.last.write() = Some(t);
            // Model a supported folded resting position only once fully unloaded.
            if self.fall_to_fold && t.iter().all(|t| t.kp_nm_rev == 0.0) {
                for (f, q) in self
                    .state
                    .write()
                    .joints
                    .iter_mut()
                    .zip(crate::startup_recipe::RECIPE.folded_position_rad)
                {
                    f.position_rev = q / std::f32::consts::TAU;
                }
            }
            Ok(())
        }
        async fn disable_all(&self) -> Result<()> {
            anyhow::ensure!(
                !self.fail_disable.load(Ordering::Acquire),
                "mock disable unconfirmed"
            );
            self.inner.disable_all().await
        }
        async fn shutdown(&self) -> Result<()> {
            self.inner.shutdown().await
        }
        async fn clear_faults(&self) -> Result<()> {
            self.inner.clear_faults().await
        }
        fn feedback(&self) -> FeedbackSnapshot {
            self.state.read().clone()
        }
        fn transport_failed(&self) -> bool {
            false
        }
    }

    async fn damping_runtime(fall: bool) -> (Arc<ArmRuntime>, Arc<DampingBackend>, u32) {
        let backend = Arc::new(DampingBackend::new(fall));
        let mut runtime = runtime_with_backend(backend.clone());
        let p = Arc::get_mut(&mut runtime.profile).unwrap();
        p.bus.protocol = crate::profile::MotorProtocol::Meow;
        p.controller.shutdown_damping = Some(crate::profile::ShutdownDamping {
            kd_nm_s_rad: [1.0; DOF],
            unload_sec: 1.0,
            timeout_sec: 2.5,
            settle_sec: 0.5,
        });
        runtime.initialize().await.unwrap();
        let id = runtime.acquire("damping-test".into()).unwrap().0;
        runtime.set_mode(id, OperatingMode::Active).await.unwrap();
        *backend.last.write() = None;
        (Arc::new(runtime), backend, id)
    }

    #[tokio::test]
    async fn damping_requires_ready_and_stationary_before_taking_ownership() {
        let (rt, b, id) = damping_runtime(true).await;
        b.state.write().joints[2].position_rev += 0.02;
        assert!(rt
            .damped_stop(id)
            .await
            .unwrap_err()
            .to_string()
            .contains("startup_ready"));
        assert!(b.last.read().is_none());
        b.state.write().joints[2].position_rev -= 0.02;
        b.state.write().joints[0].velocity_rev_s = 0.01;
        assert!(rt
            .damped_stop(id)
            .await
            .unwrap_err()
            .to_string()
            .contains("stationary"));
        assert!(b.last.read().is_none());
        rt.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn damping_unloads_settles_then_disables_and_never_reopens_commands() {
        let (rt, b, id) = damping_runtime(true).await;
        let worker = tokio::spawn(rt.clone().run_control_loop());
        rt.damped_stop(id).await.unwrap();
        let final_targets = b.last.read().unwrap();
        assert!(final_targets.iter().all(|t| t.kp_nm_rev == 0.0
            && t.torque_nm == 0.0
            && t.kd_nm_s_rev > 0.0
            && t.velocity_rev_s == 0.0));
        assert!(!b.inner.enabled.load(Ordering::Acquire));
        assert!(rt.set_mode(id, OperatingMode::Active).await.is_err());
        rt.shutdown().await.unwrap();
        worker.await.unwrap();
        assert!(!b.inner.targets_during_disable.load(Ordering::Acquire));
    }

    #[tokio::test]
    async fn damping_timeout_is_not_success_even_if_motionless() {
        let (rt, b, id) = damping_runtime(false).await;
        let error = rt.damped_stop(id).await.unwrap_err();
        assert!(error.to_string().contains("timed out"));
        assert!(!b.inner.enabled.load(Ordering::Acquire));
        assert_eq!(rt.data.read().safety.mode, OperatingMode::Fault);
    }

    #[tokio::test]
    async fn damping_unconfirmed_disable_latches_fault_and_retries_without_rpc_owner() {
        let (rt, b, id) = damping_runtime(true).await;
        b.fail_disable.store(true, Ordering::Release);
        assert!(rt
            .damped_stop(id)
            .await
            .unwrap_err()
            .to_string()
            .contains("disable unconfirmed"));
        assert_eq!(rt.data.read().safety.mode, OperatingMode::Fault);
        assert!(rt.data.read().disable_pending);
        assert!(b.inner.enabled.load(Ordering::Acquire));
        b.fail_disable.store(false, Ordering::Release);
        let worker = tokio::spawn(rt.clone().run_control_loop());
        tokio::time::timeout(Duration::from_secs(1), async {
            while rt.data.read().disable_pending {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .unwrap();
        assert!(!b.inner.enabled.load(Ordering::Acquire));
        assert!(rt.acquire("late".into()).is_err());
        rt.shutdown().await.unwrap();
        worker.await.unwrap();
    }

    #[tokio::test]
    async fn damping_feedback_failure_and_process_shutdown_interrupt_unloading() {
        for failure in ["signal", "stale", "motor_fault", "overspeed"] {
            let (rt, b, id) = damping_runtime(true).await;
            let task = tokio::spawn({
                let rt = rt.clone();
                async move { rt.damped_stop(id).await }
            });
            while b.last.read().is_none() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            match failure {
                "signal" => {
                    rt.begin_shutdown();
                }
                "stale" => b.state.write().joints[0].fresh = false,
                "motor_fault" => b.state.write().joints[0].fault_code = Some(1),
                "overspeed" => b.state.write().joints[0].velocity_rev_s = 10.0,
                _ => unreachable!(),
            }
            assert!(tokio::time::timeout(Duration::from_millis(100), task)
                .await
                .unwrap()
                .unwrap()
                .is_err());
            assert!(!b.inner.enabled.load(Ordering::Acquire));
            rt.shutdown().await.unwrap();
        }
    }

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
            schema_version: crate::profile::HARDWARE_PROFILE_SCHEMA_VERSION,
            joint_coordinate_version: crate::profile::JOINT_COORDINATE_VERSION,
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
                max_measured_temperature_c: None,
                gravity_startup_slew_rate_nm_s: None,
                hand_guiding_velocity_limits_rad_s: None,
                hand_guiding_position_margin_rad: None,
                shutdown_damping: None,
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
                    motion_feedforward: None,
                    torque_permille: 100,
                    kp_kd_torque_permille: 100,
                    meow_torque_budget: Default::default(),
                    limits: JointLimits {
                        position_lower_rad: -2.0,
                        position_upper_rad: 2.0,
                        measured_position_margin_rad: 0.0,
                        measured_velocity_margin_rad_s: 0.0,
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
    fn measured_velocity_margin_never_widens_command_validation() {
        let mut runtime = runtime_for_safety_test();
        Arc::make_mut(&mut runtime.profile).joints[3]
            .limits
            .measured_velocity_margin_rad_s = 0.02;
        let mut velocity = [0.0; DOF];
        velocity[3] = 0.21;
        assert!(runtime
            .measured_feedback_fault(&feedback_from_ros(&runtime, [0.0; DOF], velocity))
            .is_none());
        velocity[3] = 0.221;
        assert_eq!(
            runtime
                .measured_feedback_fault(&feedback_from_ros(&runtime, [0.0; DOF], velocity))
                .unwrap()
                .0,
            FAULT_MEASURED_OVERSPEED
        );
        let mut commands = targets([0.0; DOF]);
        commands[3].velocity_rad_s = 0.21;
        assert!(runtime.validate_targets(&commands).is_err());
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

    #[test]
    fn measured_torque_limit_uses_joint_units_and_checks_both_signs() {
        let mut runtime = runtime_for_safety_test();
        let joint = &mut Arc::make_mut(&mut runtime.profile).joints[2];
        joint.direction = -1;
        joint.torque_scale = 0.85;
        joint.limits.torque_nm = 7.5;
        let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        for sign in [-1.0, 1.0] {
            feedback.joints[2].torque_nm = sign * 7.5 * 0.85;
            assert!(runtime.measured_feedback_fault(&feedback).is_none());
            feedback.joints[2].torque_nm = sign * 7.51 * 0.85;
            assert_eq!(
                runtime.measured_feedback_fault(&feedback).unwrap().0,
                FAULT_MEASURED_TORQUE_LIMIT
            );
        }
        feedback.joints[2].torque_nm = f32::NAN;
        assert_eq!(
            runtime.measured_feedback_fault(&feedback).unwrap().0,
            FAULT_MEASURED_TORQUE_LIMIT
        );
    }

    #[test]
    fn measured_temperature_checks_motor_and_driver_independently() {
        let runtime = runtime_for_safety_test();
        for field in 0..3 {
            for value in [70.1, f32::NAN, f32::INFINITY] {
                let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
                let state = &mut feedback.joints[3];
                match field {
                    0 => state.temperature_c = value,
                    1 => state.motor_temperature_c = value,
                    _ => state.driver_temperature_c = value,
                }
                assert_eq!(
                    runtime.measured_feedback_fault(&feedback).unwrap().0,
                    FAULT_MEASURED_TEMPERATURE
                );
            }
        }
    }

    #[test]
    fn arm_temperature_override_preserves_finite_feedback_checks() {
        let mut runtime = runtime_for_safety_test();
        Arc::make_mut(&mut runtime.profile)
            .controller
            .max_measured_temperature_c = Some(85.0);
        for field in 0..3 {
            for value in [80.0, 85.0, 85.1, f32::NAN] {
                let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
                let state = &mut feedback.joints[3];
                match field {
                    0 => state.temperature_c = value,
                    1 => state.motor_temperature_c = value,
                    _ => state.driver_temperature_c = value,
                }
                let fault = runtime.measured_feedback_fault(&feedback);
                if value.is_finite() && value <= 85.0 {
                    assert!(fault.is_none());
                } else {
                    assert_eq!(fault.unwrap().0, FAULT_MEASURED_TEMPERATURE);
                }
            }
        }
    }

    #[tokio::test]
    async fn hand_guiding_accepts_position_margin_at_startup() {
        for margin in [None, Some(0.012)] {
            let backend = Arc::new(DampingBackend::new(false));
            let mut runtime = runtime_with_backend(backend.clone());
            let profile = Arc::make_mut(&mut runtime.profile);
            profile.controller.hand_guiding_position_margin_rad = margin;
            profile.joints[2].limits.position_lower_rad = -1.57;
            profile.joints[2].limits.position_upper_rad = 1.57;
            profile.joints[2].limits.measured_position_margin_rad = 0.01;
            let measured = if margin.is_some() { 1.581 } else { 1.570901 };
            backend.state.write().joints[2].position_rev = measured / std::f32::consts::TAU;
            runtime.initialize().await.unwrap();
            let (session, _, _) = runtime.acquire("hand-guiding-boundary".into()).unwrap();
            assert!(runtime
                .set_mode(session, OperatingMode::Active)
                .await
                .is_err());
            assert_eq!(backend.inner.enable_calls.load(Ordering::Acquire), 0);
            runtime
                .start_gravity_comp(session, &[0.5; DOF])
                .await
                .unwrap();
            let sent = backend.last.read().unwrap()[2];
            assert!(
                (motor_position_to_ros(sent.position_rev, &runtime.profile.joints[2]) - 1.57).abs()
                    < 1.0e-6
            );
            assert!(sent.kp_nm_rev == 0.0 && sent.velocity_rev_s == 0.0);
            assert!(
                (sent.position_rev - backend.state.read().joints[2].position_rev).abs() <= 0.002
            );
            assert!((runtime.joint_state_proto().q[2] - measured).abs() < 1.0e-6);
            runtime.shutdown().await.unwrap();
        }
    }

    #[test]
    fn hand_guiding_position_margin_does_not_expand_other_modes_or_commands() {
        let mut runtime = runtime_for_safety_test();
        let profile = Arc::make_mut(&mut runtime.profile);
        profile.controller.hand_guiding_position_margin_rad = Some(0.012);
        for joint in &mut profile.joints {
            joint.limits.measured_position_margin_rad = 0.01;
        }
        for index in 0..DOF {
            for sign in [-1.0, 1.0] {
                let mut position = [0.0; DOF];
                position[index] = sign * 2.011;
                let feedback = feedback_from_ros(&runtime, position, [0.0; DOF]);
                assert!(runtime
                    .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                    .is_none());
                for mode in [
                    OperatingMode::Active,
                    OperatingMode::Disabled,
                    OperatingMode::Passive,
                ] {
                    assert_eq!(
                        runtime
                            .measured_feedback_fault_in_mode(&feedback, mode)
                            .unwrap()
                            .0,
                        FAULT_MEASURED_POSITION_LIMIT
                    );
                }
                let targets = runtime.gravity_comp_targets(&feedback, [0.5; DOF]);
                runtime.validate_targets(&targets).unwrap();
                assert_eq!(targets[index].position_rad, sign * 2.0);
                assert!(runtime
                    .validate_targets(&runtime.hold_targets(&feedback))
                    .is_err());
                let mut external = targets.clone();
                external[index].position_rad = position[index];
                assert!(runtime.validate_targets(&external).is_err());

                position[index] = sign * 2.013;
                let feedback = feedback_from_ros(&runtime, position, [0.0; DOF]);
                assert_eq!(
                    runtime
                        .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                        .unwrap()
                        .0,
                    FAULT_MEASURED_POSITION_LIMIT
                );
                assert!(runtime
                    .validate_targets(&runtime.gravity_comp_targets(&feedback, [0.5; DOF]))
                    .is_err());
            }
        }
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
            feedback.joints[2].position_rev = invalid;
            assert_eq!(
                runtime
                    .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                    .unwrap()
                    .0,
                FAULT_MEASURED_POSITION_LIMIT
            );
            assert!(runtime
                .validate_targets(&runtime.gravity_comp_targets(&feedback, [0.5; DOF]))
                .is_err());
        }
    }

    #[test]
    fn hand_guiding_boundary_target_keeps_actual_pose_for_gravity() {
        let (mut runtime, _) = runtime_with_startup_gravity();
        let profile = Arc::make_mut(&mut runtime.profile);
        profile.controller.hand_guiding_position_margin_rad = Some(0.012);
        profile.joints[0].limits.position_upper_rad = 1.57;
        let feedback = feedback_from_ros(&runtime, [1.578, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; DOF]);
        let actual_q = runtime.ros_joint_state(&feedback).0;
        let targets = runtime.gravity_comp_targets(&feedback, [0.5; DOF]);
        assert_eq!(targets[0].position_rad, 1.57);
        let gravity = runtime.data.read().gravity;
        let actual_torque = runtime.dynamics.gravity_torque_with(&actual_q, gravity)[0];
        let mut clamped_q = actual_q.clone();
        clamped_q[0] = 1.57;
        let clamped_torque = runtime.dynamics.gravity_torque_with(&clamped_q, gravity)[0];
        let joint = &runtime.profile.joints[0];
        let expected =
            joint.clamp_gravity_feedforward(actual_torque * joint.gravity_compensation_scale);
        assert!((targets[0].torque_nm - expected).abs() < 1.0e-6);
        assert!((actual_torque - clamped_torque).abs() > 1.0e-4);
        runtime.data.write().feedback = feedback;
        assert_eq!(runtime.joint_state_proto().q[0], actual_q[0]);
    }

    #[tokio::test]
    async fn hand_guiding_configured_speed_is_used_by_the_control_loop() {
        let backend = Arc::new(DampingBackend::new(false));
        let mut runtime = runtime_with_backend(backend.clone());
        Arc::make_mut(&mut runtime.profile)
            .controller
            .hand_guiding_velocity_limits_rad_s = Some([2.0; DOF]);
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-speed".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        // Independent feedback models hand motion without following zero
        // velocity targets, so the loop must really accept speeds above 0.2.
        for state in &mut backend.state.write().joints {
            state.velocity_rev_s = 0.8 / std::f32::consts::TAU;
        }
        *backend.last.write() = None;
        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(500), async {
            while backend.last.read().is_none() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("hand guiding did not output at the relaxed speed");
        assert_eq!(runtime.mode(), OperatingMode::GravityComp);
        assert!(backend.inner.enabled.load(Ordering::Acquire));
        assert!(backend
            .last
            .read()
            .unwrap()
            .iter()
            .all(|t| t.kp_nm_rev == 0.0 && t.velocity_rev_s == 0.0));
        runtime.gravity_comp_heartbeat(session, 1).unwrap();

        backend.state.write().joints[1].velocity_rev_s = 2.01 / std::f32::consts::TAU;
        tokio::time::timeout(Duration::from_millis(500), async {
            while runtime.mode() != OperatingMode::Fault
                || backend.inner.enabled.load(Ordering::Acquire)
            {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("configured hand-guiding overspeed must disable");
        let error = runtime
            .gravity_comp_heartbeat(session, 2)
            .unwrap_err()
            .to_string();
        assert!(error.contains("joint_2 measured velocity"));
        assert!(error.contains("exceeds 2.000000 rad/s"));
        runtime.shutdown().await.unwrap();
        loop_task.await.unwrap();
    }

    #[test]
    fn hand_guiding_speed_limits_do_not_change_other_modes_or_commands() {
        let mut runtime = runtime_for_safety_test();
        let limits = [0.8, 1.0, 1.2, 1.4, 1.6, 2.0];
        Arc::make_mut(&mut runtime.profile)
            .controller
            .hand_guiding_velocity_limits_rad_s = Some(limits);
        for index in 0..DOF {
            for sign in [-1.0, 1.0] {
                let mut velocity = [0.0; DOF];
                velocity[index] = sign * limits[index];
                let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocity);
                assert!(runtime
                    .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                    .is_none());
                for mode in [
                    OperatingMode::Active,
                    OperatingMode::Disabled,
                    OperatingMode::Passive,
                ] {
                    assert_eq!(
                        runtime
                            .measured_feedback_fault_in_mode(&feedback, mode)
                            .unwrap()
                            .0,
                        FAULT_MEASURED_OVERSPEED
                    );
                }
                let mut commands = targets([0.0; DOF]);
                commands[index].velocity_rad_s = velocity[index];
                assert!(runtime.validate_targets(&commands).is_err());

                velocity[index] = sign * (limits[index] + 0.001);
                let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocity);
                let (code, reason) = runtime
                    .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                    .unwrap();
                assert_eq!(code, FAULT_MEASURED_OVERSPEED);
                assert!(reason.contains(&runtime.profile.joints[index].name));
            }
            for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                let mut velocity = [0.0; DOF];
                velocity[index] = invalid;
                let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocity);
                assert_eq!(
                    runtime
                        .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                        .unwrap()
                        .0,
                    FAULT_MEASURED_OVERSPEED
                );
            }
        }
        let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.8; DOF]);
        feedback.joints[0].position_rev = 3.0 / std::f32::consts::TAU;
        assert_eq!(
            runtime
                .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                .unwrap()
                .0,
            FAULT_MEASURED_POSITION_LIMIT
        );
    }

    #[tokio::test]
    async fn hand_guiding_rejects_moving_entry_and_retains_overspeed_protection() {
        let mut backend = ShutdownOrderBackend::new();
        backend.feedback.joints[0].velocity_rev_s = 0.03 / std::f32::consts::TAU;
        let backend = Arc::new(backend);
        let mut runtime = runtime_with_backend(backend.clone());
        Arc::make_mut(&mut runtime.profile)
            .controller
            .hand_guiding_velocity_limits_rad_s = Some([2.0; DOF]);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("moving-entry".into()).unwrap();
        assert!(runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap_err()
            .to_string()
            .contains("entry speed"));
        assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);

        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("guiding-overspeed".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        let mut moved = backend.targets();
        moved[0].velocity_rev_s = 1.0;
        backend.set_targets(moved).await.unwrap();
        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(500), async {
            while backend.is_enabled() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .unwrap();
        assert_eq!(
            runtime.driver_state_proto().fault_code,
            FAULT_MEASURED_OVERSPEED
        );
        runtime.shutdown().await.unwrap();
        loop_task.await.unwrap();
    }

    #[tokio::test]
    async fn hand_guiding_rejects_invalid_gains_and_nonowners_before_enable() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-test".into()).unwrap();
        for invalid in [
            vec![],
            vec![1.0; 5],
            vec![0.0; DOF],
            vec![-1.0; DOF],
            vec![f32::NAN; DOF],
            vec![f32::INFINITY; DOF],
        ] {
            assert!(runtime.start_gravity_comp(session, &invalid).await.is_err());
            assert!(!backend.is_enabled());
        }
        assert!(runtime
            .start_gravity_comp(session + 1, &[1.0; DOF])
            .await
            .is_err());
        assert!(!backend.is_enabled());
    }

    #[tokio::test]
    async fn hand_guiding_ramps_gravity_without_any_position_stiffness() {
        let (runtime, backend) = runtime_with_startup_gravity();
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-ramp".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        let initial = backend.targets();
        assert!(initial
            .iter()
            .all(|t| t.kp_nm_rev == 0.0 && t.torque_nm == 0.0 && t.velocity_rev_s == 0.0));
        assert!((initial[0].kd_nm_s_rev - std::f32::consts::TAU * 0.5).abs() < 1.0e-5);
        let t0 = runtime
            .data
            .read()
            .gravity_comp
            .as_ref()
            .unwrap()
            .ramp
            .as_ref()
            .unwrap()
            .last_tick_ns;
        runtime
            .gravity_comp_tick(&backend.feedback(), t0 + 20_000_000)
            .await
            .unwrap();
        assert!((backend.targets()[0].torque_nm - 0.1).abs() < 1.0e-5);
        runtime
            .gravity_comp_tick(&backend.feedback(), t0 + 100_000_000)
            .await
            .unwrap();
        assert!((backend.targets()[0].torque_nm - 0.3).abs() < 1.0e-5);
        assert!(runtime
            .data
            .read()
            .gravity_comp
            .as_ref()
            .unwrap()
            .ramp
            .is_none());
        assert!(runtime
            .event_log_proto()
            .events
            .iter()
            .any(|e| e.code == "hand_guiding_ready"));

        // After the ramp, gravity follows the actual hand-moved pose directly.
        let feedback = feedback_from_ros(
            &runtime,
            [std::f32::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.1; DOF],
        );
        runtime
            .gravity_comp_tick(&feedback, t0 + 102_000_000)
            .await
            .unwrap();
        let targets = backend.targets();
        assert!(targets[0].torque_nm.abs() < 1.0e-5);
        assert!(targets
            .iter()
            .all(|t| t.kp_nm_rev == 0.0 && t.velocity_rev_s == 0.0));
        assert!((targets[0].position_rev - 0.25).abs() < 1.0e-5);
    }

    #[tokio::test]
    async fn hand_guiding_validates_eventual_gravity_before_zero_ramp_enable() {
        let (template, _) = runtime_with_startup_gravity();
        let backend = Arc::new(ShutdownOrderBackend::new());
        // This backend rejects the full target even though the zero ramp frame
        // alone would fit. No enable call may escape this preflight failure.
        backend
            .reject_nonzero_feedforward
            .store(true, Ordering::Release);
        let runtime = ArmRuntime::new(template.profile, backend.clone(), template.dynamics);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-budget".into()).unwrap();
        assert!(runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .is_err());
        assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
    }

    #[tokio::test]
    async fn hand_guiding_is_exclusive_and_cannot_become_a_trajectory() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-owner".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        assert_eq!(runtime.acquire("competitor".into()).unwrap().0, 0);
        assert!(runtime
            .submit_trajectory(streaming_command(session, vec![]))
            .is_err());
        assert!(runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .is_err());
        assert!(runtime
            .start_gravity_comp(session, &[1.0; DOF])
            .await
            .is_err());
        assert!(runtime.set_gravity(session, [0.0, 0.0, 9.81]).is_err());
        assert!(runtime.release(session + 1).await.is_err());
        assert!(backend.is_enabled());
        runtime.release(session).await.unwrap();
        assert!(!backend.is_enabled());
        assert!(runtime.data.read().gravity_comp.is_none());
        assert!(runtime.gravity_comp_heartbeat(session, 1).is_err());
        let (new_session, _, _) = runtime.acquire("next-owner".into()).unwrap();
        assert_ne!(new_session, session);
        runtime
            .set_mode(new_session, OperatingMode::Active)
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn hand_guiding_heartbeat_cannot_replay_or_revive_an_expired_lease() {
        let (runtime, _) = runtime_and_backend_for_safety_test();
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-heartbeat".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        assert!(runtime.gravity_comp_heartbeat(session + 1, 1).is_err());
        runtime.gravity_comp_heartbeat(session, 1).unwrap();
        let renewed = runtime
            .data
            .read()
            .gravity_comp
            .as_ref()
            .unwrap()
            .renewed_at;
        assert!(runtime.gravity_comp_heartbeat(session, 1).is_err());
        assert!(runtime.gravity_comp_heartbeat(session, 0).is_err());
        assert_eq!(
            runtime
                .data
                .read()
                .gravity_comp
                .as_ref()
                .unwrap()
                .renewed_at,
            renewed
        );
        runtime
            .data
            .write()
            .gravity_comp
            .as_mut()
            .unwrap()
            .renewed_at = Instant::now() - GRAVITY_COMP_LEASE;
        assert!(runtime
            .gravity_comp_heartbeat(session, 2)
            .unwrap_err()
            .to_string()
            .contains("expired"));
    }

    #[tokio::test]
    async fn hand_guiding_heartbeat_reports_the_original_joint_fault() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-fault-report".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        let mut overspeed = [crate::conversion::MotorTarget::default(); DOF];
        overspeed[1].velocity_rev_s = 1.0;
        backend.set_targets(overspeed).await.unwrap();
        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(500), async {
            while runtime.mode() != OperatingMode::Fault || backend.is_enabled() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .expect("hand-guiding overspeed must fault and disable");
        let state = runtime.driver_state_proto();
        assert_eq!(state.fault_code, FAULT_MEASURED_OVERSPEED);
        let error = runtime
            .gravity_comp_heartbeat(session, 1)
            .unwrap_err()
            .to_string();
        assert!(error.contains("0x1007"));
        assert!(error.contains("joint_2 measured velocity"));
        assert!(error.contains(&state.fault_reason));
        assert!(runtime.data.read().gravity_comp.is_none());
        runtime.shutdown().await.unwrap();
        loop_task.await.unwrap();
    }

    #[tokio::test]
    async fn hand_guiding_deadman_fault_retries_disable_without_position_hold() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-deadman".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        runtime
            .data
            .write()
            .gravity_comp
            .as_mut()
            .unwrap()
            .renewed_at = Instant::now() - GRAVITY_COMP_LEASE;
        backend.fail_next_disables(1);
        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(500), async {
            while backend.is_enabled() {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await
        .unwrap();
        assert_eq!(runtime.mode(), OperatingMode::Fault);
        assert_eq!(
            runtime.driver_state_proto().fault_code,
            FAULT_COMMAND_WATCHDOG
        );
        assert!(backend.disable_attempts() >= 2);
        assert!(backend.targets().iter().all(|t| t.kp_nm_rev == 0.0));
        assert!(runtime.gravity_comp_heartbeat(session, 1).is_err());
        runtime.shutdown().await.unwrap();
        loop_task.await.unwrap();
    }

    #[tokio::test]
    async fn hand_guiding_runs_with_heartbeats_and_stops_on_shutdown() {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-live".into()).unwrap();
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        // Survive both the ordinary 100 ms command watchdog and the 500 ms
        // guiding lease without sending any trajectory/position command.
        for sequence in 1..=14 {
            runtime.gravity_comp_heartbeat(session, sequence).unwrap();
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        assert_eq!(runtime.mode(), OperatingMode::GravityComp);
        assert!(runtime.driver_state_proto().command_age_s < 0.2);
        runtime.shutdown().await.unwrap();
        loop_task.await.unwrap();
        assert!(!backend.is_enabled());
        assert!(runtime.gravity_comp_heartbeat(session, 15).is_err());
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

    #[test]
    fn automatic_motion_feedforward_is_not_added_to_explicit_client_torque() {
        let template = runtime_for_safety_test();
        let mut profile = (*template.profile).clone();
        for j in &mut profile.joints {
            j.gravity_compensation_scale = 0.0;
        }
        profile.joints[0].motion_feedforward = Some(crate::profile::MotionFeedforward {
            positive_nm: 0.2,
            negative_nm: 0.4,
            velocity_scale_rad_s: 0.001,
        });
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            vec![(0.0, [0.0; 3]); DOF],
            [0.0, 0.0, -9.81],
        );
        let runtime = ArmRuntime::new(Arc::new(profile), Arc::new(MockBackend::new()), dynamics);
        let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        let mut output = targets([0.0; DOF]);
        output[0].velocity_rad_s = -0.01;
        runtime.apply_command_gravity_feedforward(&mut output, &feedback, true, 0);
        assert!((output[0].torque_nm + 0.4).abs() < 1e-6);
        runtime.apply_command_gravity_feedforward(&mut output, &feedback, true, 1);
        assert!(
            (output[0].torque_nm + 0.4).abs() < 1e-6,
            "must replace, not accumulate"
        );
        output[0].torque_nm = 0.125;
        runtime.apply_command_gravity_feedforward(&mut output, &feedback, false, 2);
        assert_eq!(output[0].torque_nm, 0.125);
        output[0].velocity_rad_s = 0.0;
        runtime.apply_command_gravity_feedforward(&mut output, &feedback, true, 3);
        assert_eq!(output[0].torque_nm, 0.0);
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
