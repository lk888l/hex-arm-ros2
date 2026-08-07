use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use prost::Message;
use zenoh::Session;

use crate::runtime::ArmRuntime;
use crate::safety::OperatingMode;

pub mod pb {
    include!(concat!(env!("OUT_DIR"), "/robot_api.rs"));
}

pub async fn serve(session: Session, runtime: Arc<ArmRuntime>, urdf_xml: String) -> Result<()> {
    spawn_description(session.clone(), runtime.clone());
    spawn_arm_description(session.clone(), runtime.clone());
    spawn_urdf(session.clone(), runtime.clone(), urdf_xml);
    spawn_acquire(session.clone(), runtime.clone());
    spawn_release(session.clone(), runtime.clone());
    spawn_set_mode(session.clone(), runtime.clone());
    spawn_clear_fault(session.clone(), runtime.clone());
    spawn_event_log(session.clone(), runtime.clone());
    spawn_discovery(session.clone(), runtime.clone());
    spawn_set_gravity(session.clone(), runtime.clone());
    spawn_command_subscriber(session.clone(), runtime.clone()).await?;
    spawn_state_publisher(session, runtime);
    Ok(())
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

fn spawn_description(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/description", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare description queryable");
        while let Ok(query) = queryable.recv_async().await {
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
                    minor: 3,
                    patch: 0,
                }),
                device_keys: vec![format!("{}/arm", runtime.profile.robot_prefix)],
                supported_modes: vec![
                    pb::OperatingMode::Disabled as i32,
                    pb::OperatingMode::Active as i32,
                    pb::OperatingMode::Passive as i32,
                    pb::OperatingMode::GravityComp as i32,
                ],
                urdf_key: Some(format!("{}/urdf", runtime.profile.robot_prefix)),
            };
            reply(query, message).await;
        }
    });
}

fn spawn_arm_description(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/arm/description", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare arm description queryable");
        while let Ok(query) = queryable.recv_async().await {
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
                supported_timeouts: vec![
                    pb::TimeoutBehavior::Hold as i32,
                    pb::TimeoutBehavior::RampStop as i32,
                    pb::TimeoutBehavior::Fault as i32,
                ],
                total_power_max_w: None,
                thermal_derate_temp: None,
            };
            reply(query, message).await;
        }
    });
}

fn spawn_urdf(session: Session, runtime: Arc<ArmRuntime>, urdf_xml: String) {
    for suffix in ["urdf", "arm/urdf"] {
        let session = session.clone();
        let runtime = runtime.clone();
        let xml = urdf_xml.clone();
        tokio::spawn(async move {
            let key = format!("{}/{}", runtime.profile.robot_prefix, suffix);
            let queryable = session
                .declare_queryable(&key)
                .await
                .expect("declare URDF queryable");
            while let Ok(query) = queryable.recv_async().await {
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
        });
    }
}

fn spawn_acquire(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/rpc/acquire_session", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare acquire queryable");
        while let Ok(query) = queryable.recv_async().await {
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
    });
}

fn spawn_release(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/rpc/release_session", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare release queryable");
        while let Ok(query) = queryable.recv_async().await {
            let result = match decode::<pb::ReleaseSessionRequest>(&query) {
                Ok(request) => runtime.release(request.session_id).await,
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    });
}

fn spawn_set_mode(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/rpc/set_mode", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare mode queryable");
        while let Ok(query) = queryable.recv_async().await {
            let result = match decode::<pb::SetModeRequest>(&query) {
                Ok(request) => match mode_from_wire(request.mode) {
                    Ok(mode) => runtime.set_mode(request.session_id, mode).await,
                    Err(error) => Err(error),
                },
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    });
}

fn spawn_clear_fault(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/rpc/clear_fault", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare clear fault queryable");
        while let Ok(query) = queryable.recv_async().await {
            let result = match decode::<pb::ClearFaultRequest>(&query) {
                Ok(request) => runtime.clear_fault(request.session_id).await,
                Err(error) => Err(error),
            };
            reply(query, generic(result)).await;
        }
    });
}

fn spawn_event_log(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/events/recent", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare event log queryable");
        while let Ok(query) = queryable.recv_async().await {
            reply(query, runtime.event_log_proto()).await;
        }
    });
}

fn spawn_discovery(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let key = format!("{}/arm/rpc/discover", runtime.profile.robot_prefix);
        let queryable = session
            .declare_queryable(&key)
            .await
            .expect("declare discovery queryable");
        while let Ok(query) = queryable.recv_async().await {
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
    });
}

fn spawn_set_gravity(session: Session, runtime: Arc<ArmRuntime>) {
    for suffix in ["rpc/set_gravity", "arm/rpc/set_gravity"] {
        let session = session.clone();
        let runtime = runtime.clone();
        tokio::spawn(async move {
            let key = format!("{}/{}", runtime.profile.robot_prefix, suffix);
            let queryable = session
                .declare_queryable(&key)
                .await
                .expect("declare gravity queryable");
            while let Ok(query) = queryable.recv_async().await {
                let result = decode::<pb::SetGravityRequest>(&query).and_then(|request| {
                    let gravity = request.gravity.context("gravity vector missing")?;
                    runtime.set_gravity(request.session_id, [gravity.x, gravity.y, gravity.z])
                });
                reply(query, generic(result)).await;
            }
        });
    }
}

async fn spawn_command_subscriber(session: Session, runtime: Arc<ArmRuntime>) -> Result<()> {
    let key = format!("{}/arm/command", runtime.profile.robot_prefix);
    let subscriber = session
        .declare_subscriber(&key)
        .await
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    tokio::spawn(async move {
        while let Ok(sample) = subscriber.recv_async().await {
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
    });
    Ok(())
}

fn spawn_state_publisher(session: Session, runtime: Arc<ArmRuntime>) {
    tokio::spawn(async move {
        let period =
            Duration::from_secs_f64(1.0 / runtime.profile.controller.state_publish_hz as f64);
        let mut interval = tokio::time::interval(period);
        let mut last_event_seq = 0;
        loop {
            interval.tick().await;
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
    });
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
