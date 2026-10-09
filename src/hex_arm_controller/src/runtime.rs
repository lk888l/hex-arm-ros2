//! Runtime ownership: ArmRuntime alone owns RuntimeData and mode_gate.
//! Child modules borrow this owner; the control task privately owns interpolation.
mod command;
mod control;
mod gravity;
mod mode;
mod safety;
mod session;
mod shutdown;

use gravity::canonical_feedback_position;

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
use crate::profile::{validate_gravity_vector, HardwareProfile, MotorProtocol};
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
    /// Trace correlation only; command timing/admission remains unchanged.
    source_sequence: u64,
    targets: Vec<RosTarget>,
    duration_ns: u64,
    received_at: Instant,
    rebase_from_feedback: Option<Vec<RosTarget>>,
    /// An empty protocol `tau_ff` vector delegates gravity compensation to
    /// this controller. A non-empty vector remains an explicit client-owned
    /// feed-forward command (for example the legacy motor GUI).
    automatic_gravity_feedforward: bool,
}

/// Prepared off-lock; admission is rechecked atomically when committing.
struct PreparedTrajectory {
    session_id: u32,
    source_sequence: u64,
    targets: Vec<RosTarget>,
    duration_ns: u64,
    automatic_gravity_feedforward: bool,
    default_hold: bool,
}

#[derive(Debug)]
struct GravityStartupRamp {
    hold_targets: Vec<RosTarget>,
    output_nm: [f32; DOF],
    last_tick_ns: u64,
}

#[derive(Debug)]
struct MeowEntryHold {
    measured: Vec<f32>,
    bounded: Vec<f32>,
}

/// A control calculation belongs to one session and one successful activation.
/// Command generation changes within that activation do not invalidate its owner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ControlOwner {
    session_id: Option<u32>,
    epoch: u64,
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
    /// Incremented by mode commits, independently of incoming command updates.
    control_epoch: u64,
    /// A latched fault is not considered physically quiescent until the
    /// backend has confirmed that all six axes left Operation Enabled.
    disable_pending: bool,
    next_disable_retry_at: Option<Instant>,
    feedback: FeedbackSnapshot,
    motors: Vec<MotorIdentitySnapshot>,
    initialized: bool,
    gravity: [f32; 3],
    gravity_startup_ramp: Option<GravityStartupRamp>,
    /// ros2_control initially echoes the measured activation pose. Only that
    /// stationary echo may map to the bounded Meow hold, until motion starts.
    meow_entry_hold: Option<MeowEntryHold>,
    gravity_comp: Option<GravityCompLease>,
    events: VecDeque<pb::Event>,
    next_event_seq: u64,
}

impl RuntimeData {
    fn control_owner(&self) -> ControlOwner {
        ControlOwner {
            session_id: self.session.as_ref().map(|session| session.id),
            epoch: self.control_epoch,
        }
    }

    fn accepts_control(&self, owner: ControlOwner) -> bool {
        !self.closing
            && !self.damped_stopping
            && self.safety.mode == OperatingMode::Active
            && self.control_owner() == owner
    }
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
                control_epoch: 0,
                disable_pending: false,
                next_disable_retry_at: None,
                feedback: FeedbackSnapshot::default(),
                motors: Vec::new(),
                initialized: false,
                gravity,
                gravity_startup_ramp: None,
                meow_entry_hold: None,
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
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
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

    /// Identity lookup is independent of live feedback and never delays control.
    /// A physical rescan is serialized with mode changes and only allowed disabled.
    pub async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        {
            let data = self.data.read();
            anyhow::ensure!(
                !data.closing && !data.damped_stopping,
                "controller is shutting down"
            );
            anyhow::ensure!(data.initialized, "controller is not initialized");
            if !refresh {
                return Ok(data.motors.clone());
            }
            anyhow::ensure!(
                data.safety.mode == OperatingMode::Disabled && !data.disable_pending,
                "motor refresh requires confirmed DISABLED mode"
            );
        }
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        {
            let data = self.data.read();
            anyhow::ensure!(
                !data.closing && !data.damped_stopping,
                "controller is shutting down"
            );
            anyhow::ensure!(data.initialized, "controller is not initialized");
            anyhow::ensure!(
                data.safety.mode == OperatingMode::Disabled && !data.disable_pending,
                "motor refresh requires confirmed DISABLED mode"
            );
        }
        let motors = self.backend.discover(true).await?;
        let mut data = self.data.write();
        anyhow::ensure!(!data.closing, "controller is shutting down");
        data.motors = motors.clone();
        Ok(motors)
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
mod tests;
