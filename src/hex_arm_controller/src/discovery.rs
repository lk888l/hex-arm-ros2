use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use async_trait::async_trait;
use can_transport::{CanBus, CanBusState, CanCapabilities, CanFilter, CanFrame, CanId, CanIoError};
use hex_meow_motor::canopen::{nmt, sdo};
use hex_meow_motor::types::MotorIdentity;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeRole {
    ExpectedJoint,
    Auxiliary,
    Unexpected,
}

impl std::fmt::Display for NodeRole {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ExpectedJoint => formatter.write_str("expected_joint"),
            Self::Auxiliary => formatter.write_str("auxiliary"),
            Self::Unexpected => formatter.write_str("unexpected"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DiscoveredNode {
    pub node_id: u8,
    pub nmt_state: nmt::NmtState,
    pub role: NodeRole,
    pub identity: Option<MotorIdentity>,
    pub identity_error: Option<String>,
}

#[derive(Debug, Clone)]
pub struct DiscoveryReport {
    pub nodes: Vec<DiscoveredNode>,
    pub missing_expected: Vec<u8>,
}

impl DiscoveryReport {
    pub fn has_failures(&self) -> bool {
        !self.missing_expected.is_empty()
            || self
                .nodes
                .iter()
                .any(|node| node.role == NodeRole::Unexpected || node.identity_error.is_some())
    }
}

#[derive(Debug, Clone)]
pub struct DiscoveryOptions {
    pub expected_node_ids: BTreeSet<u8>,
    pub auxiliary_node_ids: BTreeSet<u8>,
    pub observe_timeout: Duration,
    pub sdo_timeout: Duration,
}

impl DiscoveryOptions {
    pub fn validate(&self) -> Result<()> {
        anyhow::ensure!(
            !self.expected_node_ids.is_empty(),
            "at least one expected node id is required"
        );
        anyhow::ensure!(
            !self.observe_timeout.is_zero(),
            "discovery observation timeout must be positive"
        );
        anyhow::ensure!(!self.sdo_timeout.is_zero(), "SDO timeout must be positive");
        for node_id in self
            .expected_node_ids
            .iter()
            .chain(&self.auxiliary_node_ids)
        {
            anyhow::ensure!(
                (1..=127).contains(node_id),
                "CANopen node id {node_id} is outside 1..=127"
            );
        }
        anyhow::ensure!(
            self.expected_node_ids.is_disjoint(&self.auxiliary_node_ids),
            "expected and auxiliary node ids overlap"
        );
        Ok(())
    }
}

/// Restrict a CAN transport to SDO upload requests used by read-only discovery.
/// NMT, PDO, heartbeat production, SDO downloads, and shared motor commands are
/// rejected locally before they can reach the bus.
struct ReadOnlySdoBus {
    inner: Arc<dyn CanBus>,
}

impl ReadOnlySdoBus {
    fn new(inner: Arc<dyn CanBus>) -> Self {
        Self { inner }
    }

    fn upload_frame_allowed(frame: &CanFrame) -> bool {
        let CanId::Standard(cob_id) = frame.id() else {
            return false;
        };
        if !(0x601..=0x67f).contains(&cob_id) || frame.data().len() != 8 {
            return false;
        }
        // 0x40 initiates an upload, 0x60/0x70 request upload segments, and
        // 0x80 is the protocol abort emitted by the SDO client on a timeout.
        matches!(frame.data()[0], 0x40 | 0x60 | 0x70 | 0x80)
    }
}

#[async_trait]
impl CanBus for ReadOnlySdoBus {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        if !Self::upload_frame_allowed(&frame) {
            return Err(CanIoError::backend(std::io::Error::new(
                std::io::ErrorKind::PermissionDenied,
                "discover-only transport rejected a non-upload CAN frame",
            )));
        }
        self.inner.send(frame).await
    }

    async fn subscribe(
        &self,
        filter: CanFilter,
    ) -> Result<Box<dyn can_transport::CanRx>, CanIoError> {
        self.inner.subscribe(filter).await
    }

    fn capabilities(&self) -> CanCapabilities {
        self.inner.capabilities()
    }

    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        self.inner.bus_state().await
    }
}

pub async fn discover_read_only(
    bus: Arc<dyn CanBus>,
    options: &DiscoveryOptions,
) -> Result<DiscoveryReport> {
    options.validate()?;
    let read_only_bus: Arc<dyn CanBus> = Arc::new(ReadOnlySdoBus::new(bus));
    let mut heartbeat_rx = read_only_bus
        .subscribe(CanFilter::standard(0x700, 0x780))
        .await
        .context("subscribe to CANopen heartbeats")?;
    let deadline = tokio::time::Instant::now() + options.observe_timeout;
    let mut states = BTreeMap::new();

    loop {
        match tokio::time::timeout_at(deadline, heartbeat_rx.recv()).await {
            Ok(Ok(frame)) => {
                if let Some((node_id, state)) = nmt::parse_heartbeat(&frame) {
                    states.insert(node_id, state);
                }
            }
            Ok(Err(CanIoError::Lagged { .. })) => continue,
            Ok(Err(error)) => return Err(error).context("receive CANopen heartbeat"),
            Err(_) => break,
        }
    }

    let mut nodes = Vec::with_capacity(states.len());
    for (node_id, nmt_state) in states {
        let role = classify_node(
            node_id,
            &options.expected_node_ids,
            &options.auxiliary_node_ids,
        );
        let (identity, identity_error) =
            match read_identity(read_only_bus.as_ref(), node_id, options.sdo_timeout).await {
                Ok(identity) => (Some(identity), None),
                Err(error) => (None, Some(error.to_string())),
            };
        nodes.push(DiscoveredNode {
            node_id,
            nmt_state,
            role,
            identity,
            identity_error,
        });
    }

    let discovered: BTreeSet<_> = nodes.iter().map(|node| node.node_id).collect();
    let missing_expected = options
        .expected_node_ids
        .difference(&discovered)
        .copied()
        .collect();
    Ok(DiscoveryReport {
        nodes,
        missing_expected,
    })
}

fn classify_node(node_id: u8, expected: &BTreeSet<u8>, auxiliary: &BTreeSet<u8>) -> NodeRole {
    if expected.contains(&node_id) {
        NodeRole::ExpectedJoint
    } else if auxiliary.contains(&node_id) {
        NodeRole::Auxiliary
    } else {
        NodeRole::Unexpected
    }
}

async fn read_identity(bus: &dyn CanBus, node_id: u8, timeout: Duration) -> Result<MotorIdentity> {
    let timeout = Some(timeout);
    let vendor_id = sdo::upload_u32(bus, node_id, 0x1018, 1, timeout).await?;
    let product_code = sdo::upload_u32(bus, node_id, 0x1018, 2, timeout).await?;
    let revision_number = sdo::upload_u32(bus, node_id, 0x1018, 3, timeout).await?;
    let serial_number = sdo::upload_u32(bus, node_id, 0x1018, 4, timeout).await?;
    let product_name = sdo::upload_string(bus, node_id, 0x1008, 0, timeout)
        .await
        .ok();
    Ok(MotorIdentity {
        node_id,
        vendor_id,
        product_code,
        revision_number,
        serial_number,
        product_name,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    use can_transport::{CanControllerState, CanRx};
    use tokio::sync::mpsc;

    #[derive(Default)]
    struct TestBusState {
        subscribers: Vec<(CanFilter, mpsc::UnboundedSender<CanFrame>)>,
        sent: Vec<CanFrame>,
    }

    struct TestBus {
        heartbeats: Vec<CanFrame>,
        state: Mutex<TestBusState>,
    }

    impl TestBus {
        fn new(nodes: &[(u8, u8)]) -> Self {
            Self {
                heartbeats: nodes
                    .iter()
                    .map(|(node_id, state)| {
                        CanFrame::new_data(0x700 + *node_id as u16, &[*state]).unwrap()
                    })
                    .collect(),
                state: Mutex::new(TestBusState::default()),
            }
        }

        fn broadcast(&self, frame: CanFrame) {
            let state = self.state.lock().unwrap();
            for (filter, sender) in &state.subscribers {
                if filter.matches(&frame) {
                    let _ = sender.send(frame);
                }
            }
        }
    }

    #[async_trait]
    impl CanBus for TestBus {
        async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
            self.state.lock().unwrap().sent.push(frame);
            let CanId::Standard(cob_id) = frame.id() else {
                return Ok(());
            };
            if !(0x601..=0x67f).contains(&cob_id) || frame.data()[0] != 0x40 {
                return Ok(());
            }
            let node_id = (cob_id - 0x600) as u8;
            let index = u16::from_le_bytes([frame.data()[1], frame.data()[2]]);
            let subindex = frame.data()[3];
            let mut response = [0u8; 8];
            response[1..4].copy_from_slice(&frame.data()[1..4]);
            if index == 0x1018 {
                response[0] = 0x43;
                let value = match subindex {
                    1 => 0x0068_6578,
                    2 => 0x4342_5000 + node_id as u32,
                    3 => 1,
                    4 => node_id as u32,
                    _ => 0,
                };
                response[4..8].copy_from_slice(&value.to_le_bytes());
            } else {
                response[0] = 0x80;
                response[4..8].copy_from_slice(&0x0602_0000u32.to_le_bytes());
            }
            self.broadcast(CanFrame::new_data(0x580 + node_id as u16, &response)?);
            Ok(())
        }

        async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
            let (sender, receiver) = mpsc::unbounded_channel();
            for frame in &self.heartbeats {
                if filter.matches(frame) {
                    sender.send(*frame).unwrap();
                }
            }
            self.state
                .lock()
                .unwrap()
                .subscribers
                .push((filter, sender));
            Ok(Box::new(TestRx(receiver)))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }

        async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
            Ok(Some(CanBusState {
                state: Some(CanControllerState::ErrorActive),
                tx_errors: Some(0),
                rx_errors: Some(0),
            }))
        }
    }

    struct TestRx(mpsc::UnboundedReceiver<CanFrame>);

    #[async_trait]
    impl CanRx for TestRx {
        async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
            self.0.recv().await.ok_or(CanIoError::Disconnected)
        }

        fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
            match self.0.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }

    #[tokio::test]
    async fn discovery_reads_identity_without_nmt_pdo_or_downloads() {
        let bus = Arc::new(TestBus::new(&[(1, 0x7f), (15, 0x7f)]));
        let report = discover_read_only(
            bus.clone(),
            &DiscoveryOptions {
                expected_node_ids: BTreeSet::from([1]),
                auxiliary_node_ids: BTreeSet::from([15]),
                observe_timeout: Duration::from_millis(1),
                sdo_timeout: Duration::from_millis(20),
            },
        )
        .await
        .unwrap();

        assert_eq!(report.nodes.len(), 2);
        assert_eq!(report.nodes[0].role, NodeRole::ExpectedJoint);
        assert_eq!(report.nodes[1].role, NodeRole::Auxiliary);
        assert!(!report.has_failures());
        let sent = bus.state.lock().unwrap().sent.clone();
        assert_eq!(sent.len(), 10);
        assert!(sent.iter().all(|frame| {
            matches!(frame.id(), CanId::Standard(0x601..=0x67f))
                && matches!(frame.data()[0], 0x40 | 0x60 | 0x70 | 0x80)
        }));
    }

    #[tokio::test]
    async fn unexpected_node_is_reported_as_failure() {
        let bus = Arc::new(TestBus::new(&[(1, 0x7f), (42, 0x05)]));
        let report = discover_read_only(
            bus,
            &DiscoveryOptions {
                expected_node_ids: BTreeSet::from([1]),
                auxiliary_node_ids: BTreeSet::new(),
                observe_timeout: Duration::from_millis(1),
                sdo_timeout: Duration::from_millis(20),
            },
        )
        .await
        .unwrap();
        assert_eq!(report.nodes[1].role, NodeRole::Unexpected);
        assert!(report.has_failures());
    }

    #[tokio::test]
    async fn read_only_wrapper_rejects_control_and_sdo_download_frames() {
        let inner: Arc<dyn CanBus> = Arc::new(TestBus::new(&[]));
        let bus = ReadOnlySdoBus::new(inner);
        assert!(bus
            .send(CanFrame::new_data(0x000u16, &[0x01, 0x01]).unwrap())
            .await
            .is_err());
        assert!(bus
            .send(CanFrame::new_data(0x601u16, &[0x23, 0x40, 0x60, 0, 0, 0, 0, 0]).unwrap())
            .await
            .is_err());
    }
}
