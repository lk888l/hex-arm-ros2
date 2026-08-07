use std::array;
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use hex_arm_dynamics::ArmDynamics;
use parking_lot::RwLock;
use tokio::sync::Mutex;
use tokio::time::MissedTickBehavior;

use crate::backend::{FeedbackSnapshot, MotorBackend, MotorIdentitySnapshot, DOF};
use crate::conversion::{
    motor_position_to_ros, motor_torque_to_ros, motor_velocity_to_ros, ros_target_to_motor,
    RosTarget,
};
use crate::interpolation::Interpolator;
use crate::profile::HardwareProfile;
use crate::protocol::pb;
use crate::safety::{OperatingMode, SafetyState};

const FAULT_MOTOR: u32 = 0x1001;
const FAULT_FEEDBACK_TIMEOUT: u32 = 0x1002;
const FAULT_COMMAND_WATCHDOG: u32 = 0x1003;
const FAULT_TRANSPORT: u32 = 0x1004;
const FAULT_COMMAND: u32 = 0x1005;
const EVENT_CAPACITY: usize = 100;

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
}

#[derive(Debug)]
struct RuntimeData {
    safety: SafetyState,
    session: Option<SessionLease>,
    next_session_id: u32,
    command: Option<CommandEnvelope>,
    command_generation: u64,
    feedback: FeedbackSnapshot,
    motors: Vec<MotorIdentitySnapshot>,
    initialized: bool,
    gravity: [f32; 3],
    events: VecDeque<pb::Event>,
    next_event_seq: u64,
}

pub struct ArmRuntime {
    pub profile: Arc<HardwareProfile>,
    backend: Arc<dyn MotorBackend>,
    dynamics: ArmDynamics,
    data: RwLock<RuntimeData>,
    mode_gate: Mutex<()>,
    started_at: Instant,
}

impl ArmRuntime {
    pub fn new(
        profile: Arc<HardwareProfile>,
        backend: Arc<dyn MotorBackend>,
        dynamics: ArmDynamics,
    ) -> Self {
        Self {
            profile,
            backend,
            dynamics,
            data: RwLock::new(RuntimeData {
                safety: SafetyState::default(),
                session: None,
                next_session_id: 1,
                command: None,
                command_generation: 0,
                feedback: FeedbackSnapshot::default(),
                motors: Vec::new(),
                initialized: false,
                gravity: [0.0, 0.0, -9.81],
                events: VecDeque::with_capacity(EVENT_CAPACITY),
                next_event_seq: 1,
            }),
            mode_gate: Mutex::new(()),
            started_at: Instant::now(),
        }
    }

    pub async fn initialize(&self) -> Result<()> {
        self.backend.initialize_disabled().await?;
        let motors = self.backend.discover(false).await?;
        let feedback = self.backend.feedback();
        let mut data = self.data.write();
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
        let motors = self.backend.discover(refresh).await?;
        self.data.write().motors = motors.clone();
        Ok(motors)
    }

    pub fn acquire(&self, client_name: String) -> Result<(u32, u32, Option<String>)> {
        let mut data = self.data.write();
        let event_client_name = client_name.clone();
        if let Some(holder) = &data.session {
            return Ok((0, holder.id, Some(holder.client_name.clone())));
        }
        let id = data.next_session_id.max(1);
        data.next_session_id = data.next_session_id.wrapping_add(1).max(1);
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
        self.require_session(session_id)?;
        self.backend
            .disable_all()
            .await
            .context("disable while releasing session")?;
        let mut data = self.data.write();
        data.safety.mode = OperatingMode::Disabled;
        data.session = None;
        data.command = None;
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

        let active_targets = if requested == OperatingMode::Active {
            let targets = self.hold_targets(&feedback);
            Some((targets.clone(), self.motor_targets(&targets)?))
        } else {
            None
        };

        let hardware_result = match requested {
            OperatingMode::Disabled => self.backend.disable_all().await,
            OperatingMode::Active => {
                let (_, motor_targets) = active_targets.as_ref().expect("ACTIVE targets prepared");
                match self.backend.enable_compressed_mit().await {
                    Ok(()) => self.backend.set_targets(*motor_targets).await,
                    Err(error) => Err(error),
                }
            }
            OperatingMode::Passive => match self.backend.enable_compressed_mit().await {
                Ok(()) => self.backend.set_targets([Default::default(); DOF]).await,
                Err(error) => Err(error),
            },
            OperatingMode::GravityComp => self.backend.enable_compressed_mit().await,
            OperatingMode::Fault | OperatingMode::Calibrating => unreachable!(),
        };

        if let Err(error) = hardware_result {
            let disable_error = self.backend.disable_all().await.err();
            if let Some(disable_error) = disable_error {
                tracing::error!(%disable_error, "rollback disable after mode transition failure also failed");
            }
            anyhow::bail!("hardware rejected mode transition to {requested:?}: {error}");
        }

        let mut data = self.data.write();
        data.safety = next_safety;
        data.command = if let Some((targets, _)) = active_targets {
            data.command_generation = data.command_generation.wrapping_add(1);
            Some(CommandEnvelope {
                generation: data.command_generation,
                targets,
                duration_ns: 0,
                received_at: Instant::now(),
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
        Ok(())
    }

    pub fn submit_trajectory(&self, command: pb::JointTrajectory) -> Result<()> {
        self.require_session(command.session_id)?;
        let point = command
            .points
            .last()
            .context("trajectory chunk has no points")?;
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
        let duration_ns = command.t_from_start_ns.last().copied().unwrap_or(0).max(0) as u64;

        let mut data = self.data.write();
        anyhow::ensure!(
            data.safety.mode == OperatingMode::Active,
            "joint commands require ACTIVE mode"
        );
        data.command_generation = data.command_generation.wrapping_add(1);
        data.command = Some(CommandEnvelope {
            generation: data.command_generation,
            targets,
            duration_ns,
            received_at: Instant::now(),
        });
        Ok(())
    }

    pub async fn clear_fault(&self, session_id: u32) -> Result<()> {
        let _gate = self.mode_gate.lock().await;
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
        self.require_session(session_id)?;
        anyhow::ensure!(
            self.profile.calibrated,
            "gravity compensation is locked until zero calibration is complete"
        );
        anyhow::ensure!(
            gravity.iter().all(|value| value.is_finite()),
            "gravity vector is non-finite"
        );
        let norm = gravity
            .iter()
            .map(|value| value * value)
            .sum::<f32>()
            .sqrt();
        anyhow::ensure!(
            (8.0..=12.0).contains(&norm),
            "gravity magnitude must be within 8..12 m/s^2"
        );
        self.data.write().gravity = gravity;
        Ok(())
    }

    pub async fn run_control_loop(self: Arc<Self>) {
        let mut interval = tokio::time::interval(Duration::from_micros(1000));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let initial = self.hold_targets(&self.backend.feedback());
        let mut interpolator = Interpolator::hold(initial, self.monotonic_ns());
        let mut generation = 0;

        loop {
            interval.tick().await;
            let feedback = self.backend.feedback();
            {
                self.data.write().feedback = feedback.clone();
            }
            let mode = self.data.read().safety.mode;
            if matches!(
                mode,
                OperatingMode::Active | OperatingMode::GravityComp | OperatingMode::Passive
            ) {
                if self.backend.transport_failed() {
                    self.fault_and_disable(FAULT_TRANSPORT, "userspace gs_usb transport failed")
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
                        self.fault_and_disable(
                            FAULT_COMMAND_WATCHDOG,
                            "ROS command watchdog expired",
                        )
                        .await;
                        continue;
                    }
                    if command.generation != generation {
                        if let Err(error) = interpolator.retarget(
                            command.targets.clone(),
                            self.monotonic_ns(),
                            command.duration_ns,
                        ) {
                            self.fault_and_disable(FAULT_COMMAND, error.to_string())
                                .await;
                            continue;
                        }
                        generation = command.generation;
                    }
                    let targets = interpolator.sample(self.monotonic_ns());
                    match self.motor_targets(&targets) {
                        Ok(targets) => {
                            if let Err(error) = self.backend.set_targets(targets).await {
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
                    let q = self.ros_joint_state(&feedback).0;
                    let gravity = self.data.read().gravity;
                    let tau = self.dynamics.gravity_torque_with(&q, gravity);
                    let targets: Vec<_> = q
                        .iter()
                        .zip(tau)
                        .map(|(q, tau)| RosTarget {
                            position_rad: *q,
                            torque_nm: tau,
                            ..Default::default()
                        })
                        .collect();
                    if let Ok(targets) = self.motor_targets(&targets) {
                        if let Err(error) = self.backend.set_targets(targets).await {
                            self.fault_and_disable(FAULT_TRANSPORT, error.to_string())
                                .await;
                        }
                    }
                }
                OperatingMode::Passive => {
                    if let Err(error) = self.backend.set_targets([Default::default(); DOF]).await {
                        self.fault_and_disable(FAULT_TRANSPORT, error.to_string())
                            .await;
                    }
                }
                OperatingMode::Disabled | OperatingMode::Fault | OperatingMode::Calibrating => {}
            }
        }
    }

    pub async fn shutdown(&self) {
        let _ = self.backend.disable_all().await;
        let mut data = self.data.write();
        data.safety.mode = OperatingMode::Disabled;
        data.command = None;
        data.session = None;
    }

    pub fn joint_state_proto(&self) -> pb::JointState {
        let feedback = self.data.read().feedback.clone();
        let (q, dq, tau, temp) = self.ros_joint_state_full(&feedback);
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

    fn hold_targets(&self, feedback: &FeedbackSnapshot) -> Vec<RosTarget> {
        feedback
            .joints
            .iter()
            .zip(&self.profile.joints)
            .map(|(state, joint)| RosTarget {
                position_rad: motor_position_to_ros(state.position_rev, joint),
                velocity_rad_s: 0.0,
                torque_nm: 0.0,
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            })
            .collect()
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
        Ok(array::from_fn(|index| {
            ros_target_to_motor(targets[index], &self.profile.joints[index])
        }))
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

    async fn fault_and_disable(&self, code: u32, reason: impl Into<String>) {
        let reason = reason.into();
        {
            let mut data = self.data.write();
            if data.safety.mode == OperatingMode::Fault {
                return;
            }
            data.safety.latch_fault(code, reason.clone());
            data.command = None;
            self.push_event_locked(
                &mut data,
                pb::EventSeverity::Fatal,
                "whole_arm_fault",
                reason.clone(),
                &[("fault_code", format!("0x{code:04x}"))],
            );
        }
        tracing::error!(fault_code = code, %reason, "whole-arm fault latched");
        if let Err(error) = self.backend.disable_all().await {
            tracing::error!(%error, "best-effort fault disable failed");
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
