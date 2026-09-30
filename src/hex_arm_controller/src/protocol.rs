use std::future::IntoFuture;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use prost::Message;
use tokio::sync::watch;
use tokio::task::JoinHandle;
use zenoh::Session;

use crate::runtime::ArmRuntime;
use crate::safety::OperatingMode;

pub mod pb {
    include!(concat!(env!("OUT_DIR"), "/robot_api.rs"));
}

fn supported_timeout_behaviors() -> Vec<i32> {
    // HOLD and RAMP_STOP are wire-compatible enum values, but neither has a
    // commissioned real-time safety implementation in this driver. Advertising
    // only FAULT keeps capability discovery aligned with submit_trajectory.
    vec![pb::TimeoutBehavior::Fault as i32]
}

pub struct ProtocolTasks {
    handles: Vec<JoinHandle<()>>,
}

impl ProtocolTasks {
    /// Cancel and join all remaining Zenoh I/O tasks. The caller must first
    /// latch ArmRuntime closing and complete ArmRuntime::shutdown: aborting a
    /// query task while it owns an in-flight set_mode future would otherwise
    /// cancel a CAN transition halfway through. Normal control follows this
    /// ordering in main.rs; by this point tasks still alive are stuck only in
    /// transport declaration/reply/publication work.
    pub async fn cancel_and_join(self) -> Result<()> {
        for handle in &self.handles {
            handle.abort();
        }
        let mut failures = Vec::new();
        for handle in self.handles {
            if let Err(error) = handle.await {
                if !error.is_cancelled() {
                    failures.push(error.to_string());
                }
            }
        }
        anyhow::ensure!(
            failures.is_empty(),
            "one or more Zenoh protocol tasks failed: {}",
            failures.join("; ")
        );
        Ok(())
    }
}

pub async fn serve(
    session: Session,
    runtime: Arc<ArmRuntime>,
    urdf_xml: String,
) -> Result<ProtocolTasks> {
    anyhow::ensure!(!runtime.is_closing(), "controller is shutting down");

    // This is the only declaration performed before its task is spawned. Do
    // it first so a subscriber setup failure cannot leave a partially detached
    // API behind.
    let command_subscriber = spawn_command_subscriber(session.clone(), runtime.clone()).await?;
    let mut handles = vec![command_subscriber];
    handles.push(spawn_description(session.clone(), runtime.clone()));
    handles.push(spawn_arm_description(session.clone(), runtime.clone()));
    handles.extend(spawn_urdf(session.clone(), runtime.clone(), urdf_xml));
    handles.push(spawn_acquire(session.clone(), runtime.clone()));
    handles.push(spawn_release(session.clone(), runtime.clone()));
    handles.push(spawn_set_mode(session.clone(), runtime.clone()));
    handles.push(spawn_start_gravity_comp(session.clone(), runtime.clone()));
    handles.push(spawn_gravity_comp_heartbeat(
        session.clone(),
        runtime.clone(),
    ));
    handles.push(spawn_damped_stop(session.clone(), runtime.clone()));
    handles.push(spawn_clear_fault(session.clone(), runtime.clone()));
    handles.push(spawn_event_log(session.clone(), runtime.clone()));
    handles.push(spawn_discovery(session.clone(), runtime.clone()));
    handles.extend(spawn_set_gravity(session.clone(), runtime.clone()));
    handles.push(spawn_state_publisher(session, runtime.clone()));
    Ok(ProtocolTasks { handles })
}

async fn next_while_running<Operation>(
    closing: &mut watch::Receiver<bool>,
    operation: Operation,
) -> Option<Operation::Output>
where
    Operation: IntoFuture,
{
    if *closing.borrow() {
        return None;
    }
    tokio::select! {
        biased;
        _ = closing.changed() => None,
        output = operation.into_future() => Some(output),
    }
}

fn encode<M: Message>(message: &M) -> Vec<u8> {
    message.encode_to_vec()
}

fn decode<M: Message + Default>(query: &zenoh::query::Query) -> Result<M> {
    let bytes = query
        .payload()
        .map(|payload| payload.to_bytes())
        .unwrap_or_default();
    M::decode(bytes.as_ref()).context("decode robot_api query payload")
}

async fn reply<M: Message>(query: zenoh::query::Query, message: M) {
    let key = query.key_expr().clone();
    if let Err(error) = query.reply(key, encode(&message)).await {
        tracing::warn!(%error, "Zenoh query reply failed");
    }
}

fn generic(result: Result<()>) -> pb::GenericResponse {
    match result {
        Ok(()) => pb::GenericResponse {
            ok: true,
            error: None,
        },
        Err(error) => pb::GenericResponse {
            ok: false,
            error: Some(error.to_string()),
        },
    }
}

fn spawn_description(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/description", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare description queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let message = pb::RobotDescription {
                robot_index: runtime
                    .profile
                    .robot_prefix
                    .rsplit('/')
                    .next()
                    .unwrap_or("arm0")
                    .into(),
                kind: pb::RobotKind::Arm as i32,
                model: "firefly_y6".into(),
                api_version: Some(pb::ApiVersion {
                    major: 0,
                    minor: 4,
                    patch: 0,
                }),
                device_keys: vec![format!("{}/arm", runtime.profile.robot_prefix)],
                supported_modes: vec![
                    pb::OperatingMode::Disabled as i32,
                    pb::OperatingMode::Active as i32,
                    pb::OperatingMode::GravityComp as i32,
                ],
                urdf_key: Some(format!("{}/urdf", runtime.profile.robot_prefix)),
            };
            reply(query, message).await;
        }
    })
}

fn spawn_arm_description(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/arm/description", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare arm description queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let joints = &runtime.profile.joints;
            let message = pb::ArmDescription {
                dof: 6,
                joint_names: joints.iter().map(|joint| joint.name.clone()).collect(),
                pos_min: joints
                    .iter()
                    .map(|joint| joint.limits.position_lower_rad)
                    .collect(),
                pos_max: joints
                    .iter()
                    .map(|joint| joint.limits.position_upper_rad)
                    .collect(),
                vel_max: joints
                    .iter()
                    .map(|joint| joint.limits.velocity_rad_s)
                    .collect(),
                tau_max: joints.iter().map(|joint| joint.limits.torque_nm).collect(),
                default_kp: joints.iter().map(|joint| joint.default_kp).collect(),
                default_kd: joints.iter().map(|joint| joint.default_kd).collect(),
                motor_models: joints
                    .iter()
                    .map(|joint| joint.identity.model.clone())
                    .collect(),
                supported_timeouts: supported_timeout_behaviors(),
                total_power_max_w: None,
                thermal_derate_temp: None,
            };
            reply(query, message).await;
        }
    })
}

fn spawn_urdf(session: Session, runtime: Arc<ArmRuntime>, urdf_xml: String) -> Vec<JoinHandle<()>> {
    let mut handles = Vec::new();
    for suffix in ["urdf", "arm/urdf"] {
        let session = session.clone();
        let runtime = runtime.clone();
        let xml = urdf_xml.clone();
        let mut closing = runtime.closing_receiver();
        handles.push(tokio::spawn(async move {
            let key = format!("{}/{}", runtime.profile.robot_prefix, suffix);
            let Some(queryable) =
                next_while_running(&mut closing, session.declare_queryable(&key)).await
            else {
                return;
            };
            let queryable = queryable.expect("declare URDF queryable");
            loop {
                let Some(Ok(query)) =
                    next_while_running(&mut closing, queryable.recv_async()).await
                else {
                    break;
                };
                reply(
                    query,
                    pb::UrdfResource {
                        xml: xml.clone(),
                        root_link: "base_link".into(),
                        tip_link: "link_6".into(),
                        xml_gz: None,
                        mount_links: Vec::new(),
                    },
                )
                .await;
            }
        }));
    }
    handles
}

fn spawn_acquire(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/rpc/acquire_session", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare acquire queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let request: Result<pb::AcquireSessionRequest> = decode(&query);
            let response = match request.and_then(|request| {
                runtime.acquire(request.client_name.unwrap_or_else(|| "anonymous".into()))
            }) {
                Ok((session_id, holder, holder_name)) => pb::AcquireSessionResponse {
                    ok: session_id != 0,
                    session_id,
                    error: (session_id == 0).then(|| "robot is already controlled".into()),
                    current_holder: holder,
                    current_holder_name: holder_name,
                },
                Err(error) => pb::AcquireSessionResponse {
                    ok: false,
                    session_id: 0,
                    error: Some(error.to_string()),
                    current_holder: 0,
                    current_holder_name: None,
                },
            };
            reply(query, response).await;
        }
    })
}

fn spawn_release(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/rpc/release_session", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare release queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let result = match decode::<pb::ReleaseSessionRequest>(&query) {
                Ok(request) => runtime.release(request.session_id).await,
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_set_mode(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/rpc/set_mode", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare mode queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let result = match decode::<pb::SetModeRequest>(&query) {
                Ok(request) => match mode_from_wire(request.mode) {
                    Ok(mode) => runtime.set_mode(request.session_id, mode).await,
                    Err(error) => Err(error),
                },
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_start_gravity_comp(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/rpc/start_gravity_comp", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare hand-guiding queryable");
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
            let result = match decode::<pb::StartGravityCompRequest>(&query) {
                Ok(request) => {
                    runtime
                        .start_gravity_comp(request.session_id, &request.damping_nm_s_rad)
                        .await
                }
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_gravity_comp_heartbeat(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!(
            "{}/rpc/gravity_comp_heartbeat",
            runtime.profile.robot_prefix
        );
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare hand-guiding heartbeat queryable");
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
            let result = match decode::<pb::GravityCompHeartbeatRequest>(&query) {
                Ok(request) => runtime.gravity_comp_heartbeat(request.session_id, request.sequence),
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_damped_stop(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/rpc/damped_stop", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare damped stop queryable");
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
            let result = match decode::<pb::DampedStopRequest>(&query) {
                Ok(request) => runtime.damped_stop(request.session_id).await,
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_clear_fault(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/rpc/clear_fault", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare clear fault queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let result = match decode::<pb::ClearFaultRequest>(&query) {
                Ok(request) => runtime.clear_fault(request.session_id).await,
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_event_log(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/events/recent", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare event log queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            reply(query, runtime.event_log_proto()).await;
        }
    })
}

fn spawn_discovery(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let key = format!("{}/arm/rpc/discover", runtime.profile.robot_prefix);
        let Some(queryable) =
            next_while_running(&mut closing, session.declare_queryable(&key)).await
        else {
            return;
        };
        let queryable = queryable.expect("declare discovery queryable");
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            let response = match decode::<pb::DiscoverMotorsRequest>(&query) {
                Ok(request) => match runtime.discover(request.refresh).await {
                    Ok(motors) => pb::DiscoverMotorsResponse {
                        ok: true,
                        error: None,
                        motors: motors
                            .into_iter()
                            .map(|motor| pb::MotorIdentity {
                                node_id: motor.node_id as u32,
                                vendor_id: motor.vendor_id,
                                product_code: motor.product_code,
                                revision: motor.revision,
                                serial_number: motor.serial_number,
                                model: motor.model,
                                identity_verified: motor.identity_verified,
                            })
                            .collect(),
                    },
                    Err(error) => pb::DiscoverMotorsResponse {
                        ok: false,
                        error: Some(error.to_string()),
                        motors: Vec::new(),
                    },
                },
                Err(error) => pb::DiscoverMotorsResponse {
                    ok: false,
                    error: Some(error.to_string()),
                    motors: Vec::new(),
                },
            };
            reply(query, response).await;
        }
    })
}

fn spawn_set_gravity(session: Session, runtime: Arc<ArmRuntime>) -> Vec<JoinHandle<()>> {
    let mut handles = Vec::new();
    for suffix in ["rpc/set_gravity", "arm/rpc/set_gravity"] {
        let session = session.clone();
        let runtime = runtime.clone();
        let mut closing = runtime.closing_receiver();
        handles.push(tokio::spawn(async move {
            let key = format!("{}/{}", runtime.profile.robot_prefix, suffix);
            let Some(queryable) =
                next_while_running(&mut closing, session.declare_queryable(&key)).await
            else {
                return;
            };
            let queryable = queryable.expect("declare gravity queryable");
            loop {
                let Some(Ok(query)) =
                    next_while_running(&mut closing, queryable.recv_async()).await
                else {
                    break;
                };
                let result = decode::<pb::SetGravityRequest>(&query).and_then(|request| {
                    let gravity = request.gravity.context("gravity vector missing")?;
                    runtime.set_gravity(request.session_id, [gravity.x, gravity.y, gravity.z])
                });
                reply(query, generic(result)).await;
            }
        }));
    }
    handles
}

async fn spawn_command_subscriber(
    session: Session,
    runtime: Arc<ArmRuntime>,
) -> Result<JoinHandle<()>> {
    let key = format!("{}/arm/command", runtime.profile.robot_prefix);
    let subscriber = session
        .declare_subscriber(&key)
        .await
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    let mut closing = runtime.closing_receiver();
    Ok(tokio::spawn(async move {
        loop {
            let Some(Ok(sample)) = next_while_running(&mut closing, subscriber.recv_async()).await
            else {
                break;
            };
            let bytes = sample.payload().to_bytes();
            match pb::JointTrajectory::decode(bytes.as_ref()) {
                Ok(command) => {
                    if let Err(error) = runtime.submit_trajectory(command) {
                        tracing::warn!(%error, "joint command rejected");
                    }
                }
                Err(error) => tracing::warn!(%error, "malformed joint command rejected"),
            }
        }
    }))
}

fn spawn_state_publisher(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let period =
            Duration::from_secs_f64(1.0 / runtime.profile.controller.state_publish_hz as f64);
        let mut interval = tokio::time::interval(period);
        let mut last_event_seq = 0;
        loop {
            if next_while_running(&mut closing, interval.tick())
                .await
                .is_none()
            {
                break;
            }
            let prefix = &runtime.profile.robot_prefix;
            let joint = encode(&runtime.joint_state_proto());
            let driver = encode(&runtime.driver_state_proto());
            let status = encode(&runtime.robot_status_proto());
            if session
                .put(format!("{prefix}/arm/joint_state"), joint)
                .await
                .is_err()
                || session
                    .put(format!("{prefix}/driver_state"), driver)
                    .await
                    .is_err()
                || session
                    .put(format!("{prefix}/status"), status)
                    .await
                    .is_err()
            {
                tracing::warn!("one or more Zenoh state publications failed");
            }
            for event in runtime.events_after(last_event_seq) {
                if let Some(header) = &event.header {
                    last_event_seq = last_event_seq.max(header.seq);
                }
                if session
                    .put(format!("{prefix}/events"), encode(&event))
                    .await
                    .is_err()
                {
                    tracing::warn!("Zenoh event publication failed");
                }
            }
        }
    })
}

fn mode_from_wire(value: i32) -> Result<OperatingMode> {
    match pb::OperatingMode::try_from(value) {
        Ok(pb::OperatingMode::Disabled) => Ok(OperatingMode::Disabled),
        Ok(pb::OperatingMode::Active) => Ok(OperatingMode::Active),
        Ok(pb::OperatingMode::Passive) => Ok(OperatingMode::Passive),
        Ok(pb::OperatingMode::GravityComp) => Ok(OperatingMode::GravityComp),
        _ => anyhow::bail!("unsupported or controller-owned operating mode"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_capabilities_advertise_only_the_commissioned_fault_timeout() {
        assert_eq!(
            supported_timeout_behaviors(),
            vec![pb::TimeoutBehavior::Fault as i32]
        );
    }

    #[tokio::test]
    async fn protocol_waits_are_cancelled_by_the_shutdown_latch() {
        let (closing_tx, mut closing_rx) = watch::channel(false);
        let wait = tokio::spawn(async move {
            next_while_running(&mut closing_rx, std::future::pending::<()>()).await
        });
        tokio::task::yield_now().await;
        closing_tx.send_replace(true);

        let result = tokio::time::timeout(Duration::from_millis(100), wait)
            .await
            .expect("protocol receive remained detached after shutdown")
            .expect("protocol cancellation task panicked");
        assert!(result.is_none());
    }

    #[tokio::test]
    async fn stuck_zenoh_io_is_aborted_and_joined_after_backend_shutdown() {
        let tasks = ProtocolTasks {
            handles: vec![tokio::spawn(std::future::pending::<()>())],
        };

        tokio::time::timeout(Duration::from_millis(100), tasks.cancel_and_join())
            .await
            .expect("stuck protocol I/O prevented process shutdown")
            .expect("expected task cancellation is not a protocol failure");
    }
}
