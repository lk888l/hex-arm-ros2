use std::collections::HashMap;
use std::future::{poll_fn, IntoFuture};
use std::pin::Pin;
use std::sync::Arc;
use std::task::Poll;
use std::time::Duration;

use anyhow::{Context, Result};
use prost::Message;
use tokio::sync::watch;
use tokio::task::{JoinError, JoinHandle};
use tokio::time::MissedTickBehavior;
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

type QueryEndpoint =
    zenoh::query::Queryable<zenoh::handlers::FifoChannelHandler<zenoh::query::Query>>;
type CommandEndpoint =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;

struct ProtocolTask {
    name: &'static str,
    handle: JoinHandle<()>,
    outcome: Option<std::result::Result<(), JoinError>>,
}

pub struct ProtocolTasks {
    handles: Vec<ProtocolTask>,
}

impl ProtocolTasks {
    fn push(&mut self, name: &'static str, handle: JoinHandle<()>) {
        self.handles.push(ProtocolTask {
            name,
            handle,
            outcome: None,
        });
    }

    /// Polling JoinHandles is cancellation-safe: a signal can interrupt this
    /// monitor without cancelling an admitted hardware operation.
    pub async fn wait_for_failure(&mut self) -> anyhow::Error {
        poll_fn(|cx| {
            for task in &mut self.handles {
                if task.outcome.is_some() {
                    continue;
                }
                if let Poll::Ready(outcome) =
                    std::future::Future::poll(Pin::new(&mut task.handle), cx)
                {
                    let error = match &outcome {
                        Ok(()) => {
                            anyhow::anyhow!("protocol task {} exited unexpectedly", task.name)
                        }
                        Err(error) => {
                            anyhow::anyhow!("protocol task {} failed: {error}", task.name)
                        }
                    };
                    task.outcome = Some(outcome);
                    return Poll::Ready(error);
                }
            }
            Poll::Pending
        })
        .await
    }

    /// Call only after closing admission and completing backend shutdown.
    /// Cancelling an RPC before that point can interrupt a CAN transition.
    pub async fn cancel_and_join(self) -> Result<()> {
        for task in &self.handles {
            if task.outcome.is_none() {
                task.handle.abort();
            }
        }
        let mut failures = Vec::new();
        for task in self.handles {
            let outcome = match task.outcome {
                Some(outcome) => outcome,
                None => task.handle.await,
            };
            if let Err(error) = outcome {
                if !error.is_cancelled() {
                    failures.push(format!("{}: {error}", task.name));
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

// Declare every endpoint before spawning a handler. Failed setup drops all
// declarations while the arm is still disabled, with no detached RPC task.
const QUERY_ENDPOINTS: &[&str] = &[
    "description",
    "arm/description",
    "urdf",
    "arm/urdf",
    "rpc/acquire_session",
    "rpc/release_session",
    "rpc/set_mode",
    "rpc/start_gravity_comp",
    "rpc/gravity_comp_heartbeat",
    "rpc/damped_stop",
    "rpc/clear_fault",
    "events/recent",
    "arm/rpc/discover",
    "rpc/set_gravity",
    "arm/rpc/set_gravity",
];

pub async fn serve(
    session: Session,
    runtime: Arc<ArmRuntime>,
    urdf_xml: String,
) -> Result<ProtocolTasks> {
    anyhow::ensure!(!runtime.is_closing(), "controller is shutting down");
    let prefix = &runtime.profile.robot_prefix;
    let subscriber = session
        .declare_subscriber(format!("{prefix}/arm/command"))
        .await
        .map_err(|error| anyhow::anyhow!("declare command subscriber: {error}"))?;
    let mut endpoints = HashMap::new();
    for &suffix in QUERY_ENDPOINTS {
        let endpoint = session
            .declare_queryable(format!("{prefix}/{suffix}"))
            .await
            .map_err(|error| anyhow::anyhow!("declare {suffix}: {error}"))?;
        endpoints.insert(suffix, endpoint);
    }
    // No await follows the first spawn; cancellation during declaration can
    // therefore never leave a partially started API behind.
    let mut tasks = ProtocolTasks {
        handles: Vec::new(),
    };
    tasks.push(
        "command",
        spawn_command_subscriber(subscriber, runtime.clone()),
    );
    tasks.push(
        "description",
        spawn_description(endpoints.remove("description").unwrap(), runtime.clone()),
    );
    tasks.push(
        "arm_description",
        spawn_arm_description(
            endpoints.remove("arm/description").unwrap(),
            runtime.clone(),
        ),
    );
    for suffix in ["urdf", "arm/urdf"] {
        tasks.push(
            suffix,
            spawn_urdf(
                endpoints.remove(suffix).unwrap(),
                runtime.clone(),
                urdf_xml.clone(),
            ),
        );
    }
    tasks.push(
        "acquire",
        spawn_acquire(
            endpoints.remove("rpc/acquire_session").unwrap(),
            runtime.clone(),
        ),
    );
    tasks.push(
        "release",
        spawn_release(
            endpoints.remove("rpc/release_session").unwrap(),
            runtime.clone(),
        ),
    );
    tasks.push(
        "set_mode",
        spawn_set_mode(endpoints.remove("rpc/set_mode").unwrap(), runtime.clone()),
    );
    tasks.push(
        "start_gravity_comp",
        spawn_start_gravity_comp(
            endpoints.remove("rpc/start_gravity_comp").unwrap(),
            runtime.clone(),
        ),
    );
    tasks.push(
        "gravity_comp_heartbeat",
        spawn_gravity_comp_heartbeat(
            endpoints.remove("rpc/gravity_comp_heartbeat").unwrap(),
            runtime.clone(),
        ),
    );
    tasks.push(
        "damped_stop",
        spawn_damped_stop(
            endpoints.remove("rpc/damped_stop").unwrap(),
            runtime.clone(),
        ),
    );
    tasks.push(
        "clear_fault",
        spawn_clear_fault(
            endpoints.remove("rpc/clear_fault").unwrap(),
            runtime.clone(),
        ),
    );
    tasks.push(
        "events",
        spawn_event_log(endpoints.remove("events/recent").unwrap(), runtime.clone()),
    );
    tasks.push(
        "discovery",
        spawn_discovery(
            endpoints.remove("arm/rpc/discover").unwrap(),
            runtime.clone(),
        ),
    );
    for suffix in ["rpc/set_gravity", "arm/rpc/set_gravity"] {
        tasks.push(
            suffix,
            spawn_set_gravity(endpoints.remove(suffix).unwrap(), runtime.clone()),
        );
    }
    tasks.push("state", spawn_state_publisher(session, runtime));
    Ok(tasks)
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

fn spawn_description(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_arm_description(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_urdf(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>, xml: String) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
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
    })
}

fn spawn_acquire(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_release(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_set_mode(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_start_gravity_comp(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_gravity_comp_heartbeat(
    queryable: QueryEndpoint,
    runtime: Arc<ArmRuntime>,
) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
            let result = match decode::<pb::GravityCompHeartbeatRequest>(&query) {
                Ok(request) => runtime.gravity_comp_heartbeat(request.session_id, request.sequence),
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_damped_stop(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
            let result = match decode::<pb::DampedStopRequest>(&query) {
                Ok(request) => runtime.damped_stop(request.session_id).await,
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_clear_fault(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_event_log(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        loop {
            let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await
            else {
                break;
            };
            reply(query, runtime.event_log_proto()).await;
        }
    })
}

fn spawn_discovery(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
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

fn spawn_set_gravity(queryable: QueryEndpoint, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        while let Some(Ok(query)) = next_while_running(&mut closing, queryable.recv_async()).await {
            let result = decode::<pb::SetGravityRequest>(&query).and_then(|request| {
                let gravity = request.gravity.context("gravity vector missing")?;
                runtime.set_gravity(request.session_id, [gravity.x, gravity.y, gravity.z])
            });
            reply(query, generic(result)).await;
        }
    })
}

fn spawn_command_subscriber(
    subscriber: CommandEndpoint,
    runtime: Arc<ArmRuntime>,
) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        while let Some(Ok(sample)) = next_while_running(&mut closing, subscriber.recv_async()).await
        {
            let bytes = sample.payload().to_bytes();
            match pb::JointTrajectory::decode(bytes.as_ref()) {
                Ok(command) => {
                    crate::trace::record(
                        "rust_decode",
                        command.header.as_ref().map_or(0, |h| h.seq),
                        0,
                    );
                    if let Err(error) = runtime.submit_trajectory(command) {
                        tracing::warn!(%error, "joint command rejected");
                    }
                }
                Err(error) => tracing::warn!(%error, "malformed joint command rejected"),
            }
        }
    })
}

fn spawn_state_publisher(session: Session, runtime: Arc<ArmRuntime>) -> JoinHandle<()> {
    let mut closing = runtime.closing_receiver();
    tokio::spawn(async move {
        let period =
            Duration::from_secs_f64(1.0 / runtime.profile.controller.state_publish_hz as f64);
        let mut interval = tokio::time::interval(period);
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut last_event_seq = 0;
        let mut state_sequence = 0;
        loop {
            if next_while_running(&mut closing, interval.tick())
                .await
                .is_none()
            {
                break;
            }
            let prefix = &runtime.profile.robot_prefix;
            let mut joint_message = runtime.joint_state_proto();
            if crate::trace::enabled() {
                state_sequence += 1;
                if let Some(header) = &mut joint_message.header {
                    header.seq = state_sequence;
                }
                crate::trace::record("rust_state_publish", state_sequence, 0);
            }
            let joint = encode(&joint_message);
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
                return;
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
    async fn unexpected_protocol_exit_is_reported_without_cancelling_other_tasks() {
        let (entered_tx, entered_rx) = tokio::sync::oneshot::channel();
        let (finish_tx, finish_rx) = tokio::sync::oneshot::channel();
        let mut tasks = ProtocolTasks {
            handles: Vec::new(),
        };
        tasks.push(
            "admitted_hardware_rpc",
            tokio::spawn(async move {
                entered_tx.send(()).unwrap();
                finish_rx.await.unwrap();
            }),
        );
        entered_rx.await.unwrap();
        tasks.push("failed_subscriber", tokio::spawn(async {}));
        let failure = tokio::time::timeout(Duration::from_millis(100), tasks.wait_for_failure())
            .await
            .unwrap();
        assert!(failure.to_string().contains("failed_subscriber"));
        assert!(!tasks.handles[0].handle.is_finished());
        finish_tx.send(()).unwrap();
        tasks.handles[0].outcome = Some((&mut tasks.handles[0].handle).await);
        tasks.cancel_and_join().await.unwrap();
    }

    #[tokio::test]
    async fn protocol_task_panic_is_named_and_preserved_for_cleanup() {
        let mut tasks = ProtocolTasks {
            handles: Vec::new(),
        };
        tasks.push(
            "state_publisher",
            tokio::spawn(async { panic!("injected task failure") }),
        );
        let failure = tasks.wait_for_failure().await;
        assert!(failure.to_string().contains("state_publisher"));
        assert!(failure.to_string().contains("failed"));
        assert!(tasks
            .cancel_and_join()
            .await
            .unwrap_err()
            .to_string()
            .contains("state_publisher"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn closed_session_fails_before_any_protocol_handler_is_spawned() {
        let mut config = zenoh::Config::default();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        config.insert_json5("listen/endpoints", "[]").unwrap();
        let session = zenoh::open(config).await.unwrap();
        session.close().await.unwrap();
        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let profile = Arc::new(
            crate::profile::HardwareProfile::from_path_with_urdf(
                manifest.join("test/firefly_y6.mock.yaml"),
                Some(&manifest.join("../xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf")),
            )
            .unwrap(),
        );
        let dynamics = crate::payload_dynamics::load_profile_dynamics(&profile).unwrap();
        let runtime = Arc::new(ArmRuntime::new(
            profile,
            Arc::new(crate::motor::MockBackend::new()),
            dynamics,
        ));
        let failure = match serve(session, runtime.clone(), String::new()).await {
            Ok(_) => panic!("closed session accepted API startup"),
            Err(error) => error,
        };
        assert!(failure.to_string().contains("declare command subscriber"));
        assert!(!runtime.is_closing());
        runtime.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn stuck_zenoh_io_is_aborted_and_joined_after_backend_shutdown() {
        let tasks = ProtocolTasks {
            handles: vec![ProtocolTask {
                name: "stuck_io",
                handle: tokio::spawn(std::future::pending::<()>()),
                outcome: None,
            }],
        };

        tokio::time::timeout(Duration::from_millis(100), tasks.cancel_and_join())
            .await
            .expect("stuck protocol I/O prevented process shutdown")
            .expect("expected task cancellation is not a protocol failure");
    }
}
