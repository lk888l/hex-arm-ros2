//! Fail-closed, read-only validation for the field SocketCAN link.
//!
//! This module never changes link state or CAN bit timing. The full preflight
//! must complete before a `SocketCanBus` is constructed, so a mismatched
//! interface cannot receive NMT, SDO, PDO, or shared command traffic.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use serde::Deserialize;
use serde_json::Value;

use crate::profile::{ExpectedSocketCanLink, SocketCanAdapterFingerprint};

const SOCKETCAN_MTU: u32 = 72;
const SYS_CLASS_NET: &str = "/sys/class/net";
const DROP_COUNTER_SAMPLE_INTERVAL: Duration = Duration::from_millis(200);

/// Inspect and validate a SocketCAN link without opening a CAN socket or
/// transmitting a frame.
pub(crate) fn preflight_socketcan(
    interface: &str,
    expected: &ExpectedSocketCanLink,
) -> Result<SocketCanRuntimeSnapshot> {
    preflight_socketcan_with_xstats_expectation(
        interface,
        expected,
        CanXStatsExpectation::StrictZero,
    )
}

/// Exact, diagnostic-only acknowledgement of two historical SocketCAN
/// counters.  This is deliberately not part of the hardware profile: every
/// ordinary controller/commissioning open continues to require a zeroed CAN
/// error history.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoricalCanXStatsAcknowledgement {
    error_warning: u32,
    error_passive: u32,
}

impl HistoricalCanXStatsAcknowledgement {
    pub fn new(error_warning: u32, error_passive: u32) -> Self {
        Self {
            error_warning,
            error_passive,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CanXStatsExpectation {
    StrictZero,
    AcknowledgedHistorical(HistoricalCanXStatsAcknowledgement),
}

/// Narrow entry point used only by the bounded J2 high-torque diagnostic.
pub(crate) fn preflight_socketcan_for_diagnostic(
    interface: &str,
    expected: &ExpectedSocketCanLink,
    acknowledgement: HistoricalCanXStatsAcknowledgement,
) -> Result<SocketCanRuntimeSnapshot> {
    preflight_socketcan_with_xstats_expectation(
        interface,
        expected,
        CanXStatsExpectation::AcknowledgedHistorical(acknowledgement),
    )
}

fn preflight_socketcan_with_xstats_expectation(
    interface: &str,
    expected: &ExpectedSocketCanLink,
    xstats_expectation: CanXStatsExpectation,
) -> Result<SocketCanRuntimeSnapshot> {
    let before = inspect_link(interface)?;
    validate_link_with_xstats_expectation(&before, expected, xstats_expectation)
        .with_context(|| format!("SocketCAN link {interface} failed first preflight sample"))?;

    let adapter = read_adapter_snapshot(Path::new(SYS_CLASS_NET), interface)?;
    validate_adapter(&adapter, &expected.adapter)
        .with_context(|| format!("SocketCAN adapter for {interface} failed preflight"))?;

    thread::sleep(DROP_COUNTER_SAMPLE_INTERVAL);
    let after = inspect_link(interface)?;
    validate_link_with_xstats_expectation(&after, expected, xstats_expectation)
        .with_context(|| format!("SocketCAN link {interface} failed second preflight sample"))?;
    validate_drop_counters_stable(&before, &after)
        .with_context(|| format!("SocketCAN link {interface} failed dropped-frame check"))?;

    tracing::info!(
        interface,
        error_warning = after.linkinfo.info_xstats.error_warning,
        error_passive = after.linkinfo.info_xstats.error_passive,
        rx_dropped = after.stats64.rx.dropped,
        tx_dropped = after.stats64.tx.dropped,
        driver = %adapter.driver,
        usb_vid = format_args!("{:04x}", adapter.vendor_id),
        usb_pid = format_args!("{:04x}", adapter.product_id),
        serial = %adapter.serial,
        channel = adapter.channel,
        "SocketCAN fail-closed preflight passed"
    );
    Ok(SocketCanRuntimeSnapshot::from_link(&after))
}

fn inspect_link(interface: &str) -> Result<IpLink> {
    let output = Command::new("ip")
        .args([
            "-details",
            "-statistics",
            "-json",
            "link",
            "show",
            "dev",
            interface,
        ])
        .output()
        .context("run read-only SocketCAN inspection with iproute2")?;
    anyhow::ensure!(
        output.status.success(),
        "read-only SocketCAN inspection failed for {interface}: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );

    parse_ip_link(&output.stdout, interface)
}

/// Runtime health is intentionally narrower than the startup preflight. It
/// reuses the read-only iproute2 inspection, but does not repeat USB/sysfs
/// fingerprinting or timing validation once the strictly validated link is
/// open. Every kernel counter must remain exactly at its post-preflight
/// baseline; this both latches brief CAN error states after they recover and
/// prevents a counter reset from hiding later increments.
pub(crate) fn validate_runtime_socketcan(
    interface: &str,
    baseline: &SocketCanRuntimeSnapshot,
) -> Result<()> {
    inspect_and_validate_runtime(&IpRoute2RuntimeInspector, interface, baseline)
}

fn inspect_and_validate_runtime<I: RuntimeLinkInspector>(
    inspector: &I,
    interface: &str,
    baseline: &SocketCanRuntimeSnapshot,
) -> Result<()> {
    let current = inspector.inspect(interface)?;
    validate_runtime_snapshot(baseline, &current)
}

fn validate_runtime_snapshot(
    baseline: &SocketCanRuntimeSnapshot,
    current: &SocketCanRuntimeSnapshot,
) -> Result<()> {
    anyhow::ensure!(
        current.state == "ERROR-ACTIVE",
        "SocketCAN controller is not ERROR-ACTIVE: {}",
        current.state
    );
    anyhow::ensure!(
        current.tec == 0 && current.rec == 0,
        "SocketCAN TEC/REC are not both zero: tx={}, rx={}",
        current.tec,
        current.rec
    );

    for ((name, expected), (actual_name, actual)) in
        baseline.counters().into_iter().zip(current.counters())
    {
        debug_assert_eq!(name, actual_name);
        anyhow::ensure!(
            actual == expected,
            "SocketCAN cumulative counter {name} changed from baseline {expected} to {actual}"
        );
    }
    Ok(())
}

trait RuntimeLinkInspector {
    fn inspect(&self, interface: &str) -> Result<SocketCanRuntimeSnapshot>;
}

struct IpRoute2RuntimeInspector;

impl RuntimeLinkInspector for IpRoute2RuntimeInspector {
    fn inspect(&self, interface: &str) -> Result<SocketCanRuntimeSnapshot> {
        inspect_link(interface).map(|link| SocketCanRuntimeSnapshot::from_link(&link))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SocketCanRuntimeSnapshot {
    state: String,
    tec: u64,
    rec: u64,
    restarts: u64,
    bus_error: u64,
    arbitration_lost: u64,
    error_warning: u64,
    error_passive: u64,
    bus_off: u64,
    rx_errors: u64,
    rx_dropped: u64,
    rx_over_errors: u64,
    tx_errors: u64,
    tx_dropped: u64,
    tx_carrier_errors: u64,
    tx_collisions: u64,
}

impl SocketCanRuntimeSnapshot {
    fn from_link(link: &IpLink) -> Self {
        let can = &link.linkinfo.info_data;
        let xstats = &link.linkinfo.info_xstats;
        Self {
            state: can.state.clone(),
            tec: can.berr_counter.tx,
            rec: can.berr_counter.rx,
            restarts: xstats.restarts,
            bus_error: xstats.bus_error,
            arbitration_lost: xstats.arbitration_lost,
            error_warning: xstats.error_warning,
            error_passive: xstats.error_passive,
            bus_off: xstats.bus_off,
            rx_errors: link.stats64.rx.errors,
            rx_dropped: link.stats64.rx.dropped,
            rx_over_errors: link.stats64.rx.over_errors,
            tx_errors: link.stats64.tx.errors,
            tx_dropped: link.stats64.tx.dropped,
            tx_carrier_errors: link.stats64.tx.carrier_errors,
            tx_collisions: link.stats64.tx.collisions,
        }
    }

    fn counters(&self) -> [(&'static str, u64); 13] {
        [
            ("restarts", self.restarts),
            ("bus_error", self.bus_error),
            ("arbitration_lost", self.arbitration_lost),
            ("error_warning", self.error_warning),
            ("error_passive", self.error_passive),
            ("bus_off", self.bus_off),
            ("rx_errors", self.rx_errors),
            ("rx_dropped", self.rx_dropped),
            ("rx_over_errors", self.rx_over_errors),
            ("tx_errors", self.tx_errors),
            ("tx_dropped", self.tx_dropped),
            ("tx_carrier_errors", self.tx_carrier_errors),
            ("tx_collisions", self.tx_collisions),
        ]
    }
}

#[derive(Debug, Deserialize)]
struct IpLink {
    ifname: String,
    flags: Vec<String>,
    mtu: u32,
    operstate: String,
    link_type: String,
    linkinfo: IpLinkInfo,
    stats64: IpStats,
}

#[derive(Debug, Deserialize)]
struct IpLinkInfo {
    info_kind: String,
    info_data: IpCanData,
    info_xstats: IpCanXStats,
}

#[derive(Debug, Deserialize)]
struct IpCanData {
    ctrlmode: Vec<String>,
    state: String,
    berr_counter: IpErrorCounter,
    restart_ms: u32,
    bittiming: IpBitTiming,
    data_bittiming: IpBitTiming,
}

#[derive(Debug, Deserialize)]
struct IpErrorCounter {
    tx: u64,
    rx: u64,
}

#[derive(Debug, Deserialize)]
struct IpBitTiming {
    bitrate: u32,
    sample_point: Value,
    sjw: u16,
}

#[derive(Debug, Deserialize)]
struct IpCanXStats {
    restarts: u64,
    bus_error: u64,
    arbitration_lost: u64,
    error_warning: u64,
    error_passive: u64,
    bus_off: u64,
}

#[derive(Debug, Deserialize)]
struct IpStats {
    rx: IpRxStats,
    tx: IpTxStats,
}

#[derive(Debug, Deserialize)]
struct IpRxStats {
    errors: u64,
    dropped: u64,
    over_errors: u64,
}

#[derive(Debug, Deserialize)]
struct IpTxStats {
    errors: u64,
    dropped: u64,
    carrier_errors: u64,
    collisions: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct AdapterSnapshot {
    driver: String,
    vendor_id: u16,
    product_id: u16,
    serial: String,
    channel: u16,
    dev_id: u16,
}

fn parse_ip_link(bytes: &[u8], interface: &str) -> Result<IpLink> {
    let mut links: Vec<IpLink> = serde_json::from_slice(bytes)
        .with_context(|| format!("parse iproute2 JSON for {interface}"))?;
    anyhow::ensure!(
        links.len() == 1,
        "iproute2 returned {} links for {interface}, expected exactly one",
        links.len()
    );
    let link = links.pop().expect("length checked");
    anyhow::ensure!(
        link.ifname == interface,
        "iproute2 returned interface {}, expected {interface}",
        link.ifname
    );
    Ok(link)
}

#[cfg(test)]
fn validate_link(link: &IpLink, expected: &ExpectedSocketCanLink) -> Result<()> {
    validate_link_with_xstats_expectation(link, expected, CanXStatsExpectation::StrictZero)
}

fn validate_link_with_xstats_expectation(
    link: &IpLink,
    expected: &ExpectedSocketCanLink,
    xstats_expectation: CanXStatsExpectation,
) -> Result<()> {
    anyhow::ensure!(link.flags.iter().any(|flag| flag == "UP"), "link is not UP");
    anyhow::ensure!(
        link.flags.iter().any(|flag| flag == "LOWER_UP"),
        "link has no LOWER_UP carrier"
    );
    anyhow::ensure!(link.operstate == "UP", "operstate is {}", link.operstate);
    anyhow::ensure!(
        link.mtu == SOCKETCAN_MTU,
        "MTU is {}, expected 72",
        link.mtu
    );
    anyhow::ensure!(link.link_type == "can", "link type is not CAN");
    anyhow::ensure!(
        link.linkinfo.info_kind == "can",
        "link info kind is not CAN"
    );

    let can = &link.linkinfo.info_data;
    anyhow::ensure!(can.state == "ERROR-ACTIVE", "CAN state is {}", can.state);
    anyhow::ensure!(
        can.berr_counter.tx == 0 && can.berr_counter.rx == 0,
        "TEC/REC are not both zero: tx={}, rx={}",
        can.berr_counter.tx,
        can.berr_counter.rx
    );
    anyhow::ensure!(can.restart_ms == expected.restart_ms, "restart_ms mismatch");
    let fd_enabled = can.ctrlmode.iter().any(|mode| mode == "FD");
    anyhow::ensure!(fd_enabled == expected.fd, "CAN-FD ctrlmode mismatch");

    validate_timing(
        "nominal",
        &can.bittiming,
        expected.nominal_bitrate,
        expected.nominal_sample_point_permille,
        expected.nominal_sjw,
    )?;
    validate_timing(
        "data",
        &can.data_bittiming,
        expected.data_bitrate,
        expected.data_sample_point_permille,
        expected.data_sjw,
    )?;

    let x = &link.linkinfo.info_xstats;
    anyhow::ensure!(
        [x.restarts, x.bus_error, x.arbitration_lost, x.bus_off,]
            .iter()
            .all(|counter| *counter == 0),
        "CAN restarts/bus_error/arbitration_lost/bus_off counters are not all zero"
    );
    let (expected_warning, expected_passive) = match xstats_expectation {
        CanXStatsExpectation::StrictZero => (0, 0),
        CanXStatsExpectation::AcknowledgedHistorical(acknowledgement) => (
            u64::from(acknowledgement.error_warning),
            u64::from(acknowledgement.error_passive),
        ),
    };
    anyhow::ensure!(
        x.error_warning == expected_warning,
        "CAN error_warning counter is {}, expected exactly {expected_warning}",
        x.error_warning
    );
    anyhow::ensure!(
        x.error_passive == expected_passive,
        "CAN error_passive counter is {}, expected exactly {expected_passive}",
        x.error_passive
    );
    anyhow::ensure!(
        link.stats64.rx.errors == 0 && link.stats64.rx.over_errors == 0,
        "CAN RX error counters are nonzero"
    );
    anyhow::ensure!(
        link.stats64.tx.errors == 0
            && link.stats64.tx.carrier_errors == 0
            && link.stats64.tx.collisions == 0,
        "CAN TX error counters are nonzero"
    );
    Ok(())
}

fn validate_drop_counters_stable(before: &IpLink, after: &IpLink) -> Result<()> {
    anyhow::ensure!(
        after.stats64.rx.dropped == before.stats64.rx.dropped,
        "CAN RX dropped counter changed during the preflight sample: before={}, after={}",
        before.stats64.rx.dropped,
        after.stats64.rx.dropped
    );
    anyhow::ensure!(
        after.stats64.tx.dropped == before.stats64.tx.dropped,
        "CAN TX dropped counter changed during the preflight sample: before={}, after={}",
        before.stats64.tx.dropped,
        after.stats64.tx.dropped
    );
    Ok(())
}

fn validate_timing(
    phase: &str,
    actual: &IpBitTiming,
    bitrate: u32,
    sample_point_permille: u16,
    sjw: u16,
) -> Result<()> {
    anyhow::ensure!(
        actual.bitrate == bitrate,
        "{phase} bitrate is {}, expected {bitrate}",
        actual.bitrate
    );
    let actual_sample_point = parse_sample_point_permille(&actual.sample_point)?;
    anyhow::ensure!(
        actual_sample_point == sample_point_permille,
        "{phase} sample point is {actual_sample_point} permille, expected {sample_point_permille}"
    );
    anyhow::ensure!(
        actual.sjw == sjw,
        "{phase} SJW is {}, expected {sjw}",
        actual.sjw
    );
    Ok(())
}

fn parse_sample_point_permille(value: &Value) -> Result<u16> {
    let raw = match value {
        Value::String(value) => value.parse::<f64>().context("parse sample point string")?,
        Value::Number(value) => value.as_f64().context("parse numeric sample point")?,
        _ => anyhow::bail!("sample point is neither a string nor a number"),
    };
    let scaled = raw * 1000.0;
    let rounded = scaled.round();
    anyhow::ensure!(
        raw.is_finite() && (0.0..=1.0).contains(&raw) && (scaled - rounded).abs() < 0.000_001,
        "sample point {raw} cannot be represented as integer permille"
    );
    Ok(rounded as u16)
}

fn read_adapter_snapshot(sys_class_net: &Path, interface: &str) -> Result<AdapterSnapshot> {
    let netdev = sys_class_net.join(interface);
    let device = fs::canonicalize(netdev.join("device"))
        .with_context(|| format!("resolve sysfs device for {interface}"))?;
    let driver_path = fs::canonicalize(device.join("driver"))
        .with_context(|| format!("resolve sysfs driver for {interface}"))?;
    let driver = file_name(&driver_path, "adapter driver")?;

    let channel = read_sysfs_u16(&netdev.join("dev_port"), 10)
        .with_context(|| format!("read channel for {interface}"))?;
    let dev_id = read_sysfs_u16(&netdev.join("dev_id"), 0)
        .with_context(|| format!("read dev_id for {interface}"))?;

    let usb_device = find_usb_device(&device)?;
    Ok(AdapterSnapshot {
        driver,
        vendor_id: read_sysfs_u16(&usb_device.join("idVendor"), 16)?,
        product_id: read_sysfs_u16(&usb_device.join("idProduct"), 16)?,
        serial: read_trimmed(&usb_device.join("serial"))?,
        channel,
        dev_id,
    })
}

fn validate_adapter(
    actual: &AdapterSnapshot,
    expected: &SocketCanAdapterFingerprint,
) -> Result<()> {
    anyhow::ensure!(actual.driver == expected.driver, "driver mismatch");
    anyhow::ensure!(actual.vendor_id == expected.vendor_id, "USB VID mismatch");
    anyhow::ensure!(actual.product_id == expected.product_id, "USB PID mismatch");
    anyhow::ensure!(actual.serial == expected.serial, "USB serial mismatch");
    anyhow::ensure!(actual.channel == expected.channel, "USB channel mismatch");
    anyhow::ensure!(actual.dev_id == expected.channel, "USB dev_id mismatch");
    Ok(())
}

fn find_usb_device(device: &Path) -> Result<PathBuf> {
    let mut candidate = Some(device);
    while let Some(path) = candidate {
        if path.join("idVendor").is_file() && path.join("idProduct").is_file() {
            return Ok(path.to_path_buf());
        }
        candidate = path.parent();
    }
    anyhow::bail!("could not find a USB device ancestor in sysfs")
}

fn file_name(path: &Path, description: &str) -> Result<String> {
    path.file_name()
        .and_then(|value| value.to_str())
        .map(str::to_owned)
        .with_context(|| format!("read {description} from {}", path.display()))
}

fn read_trimmed(path: &Path) -> Result<String> {
    fs::read_to_string(path)
        .with_context(|| format!("read {}", path.display()))
        .map(|value| value.trim().to_owned())
}

fn read_sysfs_u16(path: &Path, radix: u32) -> Result<u16> {
    let value = read_trimmed(path)?;
    let value = if radix == 0 {
        value
            .strip_prefix("0x")
            .or_else(|| value.strip_prefix("0X"))
            .unwrap_or(&value)
    } else {
        &value
    };
    u16::from_str_radix(value, if radix == 0 { 16 } else { radix })
        .with_context(|| format!("parse integer from {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::os::unix::fs::symlink;

    const VALID_IP_JSON: &str = r#"[{
      "ifname":"can0",
      "flags":["NOARP","UP","LOWER_UP","ECHO"],
      "mtu":72,
      "operstate":"UP",
      "link_type":"can",
      "linkinfo":{
        "info_kind":"can",
        "info_data":{
          "ctrlmode":["FD"],
          "state":"ERROR-ACTIVE",
          "berr_counter":{"tx":0,"rx":0},
          "restart_ms":0,
          "bittiming":{"bitrate":1000000,"sample_point":"0.800","sjw":5},
          "data_bittiming":{"bitrate":4000000,"sample_point":"0.800","sjw":3}
        },
        "info_xstats":{"restarts":0,"bus_error":0,"arbitration_lost":0,"error_warning":0,"error_passive":0,"bus_off":0}
      },
      "stats64":{
        "rx":{"errors":0,"dropped":0,"over_errors":0},
        "tx":{"errors":0,"dropped":0,"carrier_errors":0,"collisions":0}
      }
    }]"#;

    fn expected() -> ExpectedSocketCanLink {
        ExpectedSocketCanLink {
            nominal_bitrate: 1_000_000,
            nominal_sample_point_permille: 800,
            nominal_sjw: 5,
            data_bitrate: 4_000_000,
            data_sample_point_permille: 800,
            data_sjw: 3,
            fd: true,
            restart_ms: 0,
            adapter: SocketCanAdapterFingerprint {
                driver: "gs_usb".into(),
                vendor_id: 0x1209,
                product_id: 0x2323,
                serial: "0123456789ABCDEF0123456789ABCDEF".into(),
                channel: 0,
            },
        }
    }

    fn valid_runtime_snapshot() -> SocketCanRuntimeSnapshot {
        let link = parse_ip_link(VALID_IP_JSON.as_bytes(), "can0").unwrap();
        SocketCanRuntimeSnapshot::from_link(&link)
    }

    #[derive(Clone)]
    struct FakeRuntimeInspector {
        snapshot: Option<SocketCanRuntimeSnapshot>,
    }

    impl RuntimeLinkInspector for FakeRuntimeInspector {
        fn inspect(&self, _interface: &str) -> Result<SocketCanRuntimeSnapshot> {
            self.snapshot
                .clone()
                .context("injected runtime inspection failure")
        }
    }

    #[test]
    fn accepts_the_exact_field_link_fixture() {
        let link = parse_ip_link(VALID_IP_JSON.as_bytes(), "can0").unwrap();
        validate_link(&link, &expected()).unwrap();
        assert_eq!(parse_sample_point_permille(&json!(0.8)).unwrap(), 800);
    }

    #[test]
    fn historical_warning_passive_ack_is_exact_and_diagnostic_only() {
        let mut fixture: Value = serde_json::from_str(VALID_IP_JSON).unwrap();
        *fixture
            .pointer_mut("/0/linkinfo/info_xstats/error_warning")
            .unwrap() = json!(10);
        *fixture
            .pointer_mut("/0/linkinfo/info_xstats/error_passive")
            .unwrap() = json!(10);
        let bytes = serde_json::to_vec(&fixture).unwrap();
        let first = parse_ip_link(&bytes, "can0").unwrap();

        // The ordinary preflight remains strict-zero.
        assert!(validate_link(&first, &expected()).is_err());

        let acknowledged = CanXStatsExpectation::AcknowledgedHistorical(
            HistoricalCanXStatsAcknowledgement::new(10, 10),
        );
        validate_link_with_xstats_expectation(&first, &expected(), acknowledged).unwrap();

        for expectation in [
            HistoricalCanXStatsAcknowledgement::new(9, 10),
            HistoricalCanXStatsAcknowledgement::new(10, 9),
            HistoricalCanXStatsAcknowledgement::new(11, 10),
            HistoricalCanXStatsAcknowledgement::new(10, 11),
        ] {
            assert!(validate_link_with_xstats_expectation(
                &first,
                &expected(),
                CanXStatsExpectation::AcknowledgedHistorical(expectation),
            )
            .is_err());
        }

        // A change between the two preflight samples cannot match the same
        // explicit acknowledgement and is therefore rejected.
        *fixture
            .pointer_mut("/0/linkinfo/info_xstats/error_warning")
            .unwrap() = json!(11);
        let bytes = serde_json::to_vec(&fixture).unwrap();
        let changed_second = parse_ip_link(&bytes, "can0").unwrap();
        assert!(
            validate_link_with_xstats_expectation(&changed_second, &expected(), acknowledged,)
                .is_err()
        );
    }

    #[test]
    fn historical_ack_never_relaxes_other_can_or_netdev_counters() {
        let acknowledged = CanXStatsExpectation::AcknowledgedHistorical(
            HistoricalCanXStatsAcknowledgement::new(10, 10),
        );
        for (pointer, replacement) in [
            ("/0/linkinfo/info_xstats/bus_error", json!(1)),
            ("/0/linkinfo/info_xstats/restarts", json!(1)),
            ("/0/stats64/rx/errors", json!(1)),
            ("/0/stats64/tx/carrier_errors", json!(1)),
        ] {
            let mut fixture: Value = serde_json::from_str(VALID_IP_JSON).unwrap();
            *fixture
                .pointer_mut("/0/linkinfo/info_xstats/error_warning")
                .unwrap() = json!(10);
            *fixture
                .pointer_mut("/0/linkinfo/info_xstats/error_passive")
                .unwrap() = json!(10);
            *fixture.pointer_mut(pointer).unwrap() = replacement;
            let bytes = serde_json::to_vec(&fixture).unwrap();
            let link = parse_ip_link(&bytes, "can0").unwrap();
            assert!(
                validate_link_with_xstats_expectation(&link, &expected(), acknowledged).is_err(),
                "historical acknowledgement unexpectedly accepted {pointer}"
            );
        }
    }

    #[test]
    fn rejects_every_safety_relevant_link_mismatch() {
        let cases = [
            ("/0/mtu", json!(16)),
            ("/0/operstate", json!("DOWN")),
            ("/0/linkinfo/info_data/state", json!("ERROR-PASSIVE")),
            ("/0/linkinfo/info_data/berr_counter/tx", json!(1)),
            ("/0/linkinfo/info_data/restart_ms", json!(100)),
            ("/0/linkinfo/info_data/bittiming/bitrate", json!(500000)),
            (
                "/0/linkinfo/info_data/bittiming/sample_point",
                json!("0.750"),
            ),
            ("/0/linkinfo/info_data/bittiming/sjw", json!(1)),
            (
                "/0/linkinfo/info_data/data_bittiming/bitrate",
                json!(5000000),
            ),
            (
                "/0/linkinfo/info_data/data_bittiming/sample_point",
                json!("0.750"),
            ),
            ("/0/linkinfo/info_data/data_bittiming/sjw", json!(1)),
            ("/0/linkinfo/info_xstats/bus_off", json!(1)),
            ("/0/stats64/rx/over_errors", json!(1)),
            ("/0/stats64/tx/carrier_errors", json!(1)),
        ];

        for (pointer, replacement) in cases {
            let mut fixture: Value = serde_json::from_str(VALID_IP_JSON).unwrap();
            *fixture.pointer_mut(pointer).unwrap() = replacement;
            let bytes = serde_json::to_vec(&fixture).unwrap();
            let link = parse_ip_link(&bytes, "can0").unwrap();
            assert!(
                validate_link(&link, &expected()).is_err(),
                "fixture mutation {pointer} was accepted"
            );
        }

        let mut fixture: Value = serde_json::from_str(VALID_IP_JSON).unwrap();
        *fixture
            .pointer_mut("/0/linkinfo/info_data/ctrlmode")
            .unwrap() = json!([]);
        let bytes = serde_json::to_vec(&fixture).unwrap();
        assert!(validate_link(&parse_ip_link(&bytes, "can0").unwrap(), &expected()).is_err());
    }

    #[test]
    fn accepts_stable_historical_drops_and_rejects_new_drops() {
        let mut historical: Value = serde_json::from_str(VALID_IP_JSON).unwrap();
        *historical.pointer_mut("/0/stats64/rx/dropped").unwrap() = json!(15_370);
        *historical.pointer_mut("/0/stats64/tx/dropped").unwrap() = json!(12);

        let bytes = serde_json::to_vec(&historical).unwrap();
        let before = parse_ip_link(&bytes, "can0").unwrap();
        let stable_after = parse_ip_link(&bytes, "can0").unwrap();
        validate_link(&before, &expected()).unwrap();
        validate_link(&stable_after, &expected()).unwrap();
        validate_drop_counters_stable(&before, &stable_after).unwrap();

        let mut new_rx_drop = historical.clone();
        *new_rx_drop.pointer_mut("/0/stats64/rx/dropped").unwrap() = json!(15_371);
        let bytes = serde_json::to_vec(&new_rx_drop).unwrap();
        let after = parse_ip_link(&bytes, "can0").unwrap();
        assert!(validate_drop_counters_stable(&before, &after).is_err());

        let mut new_tx_drop = historical;
        *new_tx_drop.pointer_mut("/0/stats64/tx/dropped").unwrap() = json!(13);
        let bytes = serde_json::to_vec(&new_tx_drop).unwrap();
        let after = parse_ip_link(&bytes, "can0").unwrap();
        assert!(validate_drop_counters_stable(&before, &after).is_err());
    }

    #[test]
    fn reads_and_validates_the_sysfs_adapter_fingerprint() {
        let temp = tempfile::tempdir().unwrap();
        let netdev = temp.path().join("class/net/can0");
        let usb_device = temp.path().join("devices/usb/3-2.2");
        let usb_interface = usb_device.join("3-2.2:1.0");
        let driver = temp.path().join("bus/usb/drivers/gs_usb");
        fs::create_dir_all(&netdev).unwrap();
        fs::create_dir_all(&usb_interface).unwrap();
        fs::create_dir_all(&driver).unwrap();
        fs::write(netdev.join("dev_port"), "0\n").unwrap();
        fs::write(netdev.join("dev_id"), "0x0\n").unwrap();
        fs::write(usb_device.join("idVendor"), "1209\n").unwrap();
        fs::write(usb_device.join("idProduct"), "2323\n").unwrap();
        fs::write(
            usb_device.join("serial"),
            "0123456789ABCDEF0123456789ABCDEF\n",
        )
        .unwrap();
        symlink(&usb_interface, netdev.join("device")).unwrap();
        symlink(&driver, usb_interface.join("driver")).unwrap();

        let actual = read_adapter_snapshot(&temp.path().join("class/net"), "can0").unwrap();
        validate_adapter(&actual, &expected().adapter).unwrap();

        let mut wrong = expected().adapter;
        wrong.channel = 1;
        assert!(validate_adapter(&actual, &wrong).is_err());
        wrong = expected().adapter;
        wrong.serial = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF".into();
        assert!(validate_adapter(&actual, &wrong).is_err());
    }

    #[test]
    fn runtime_inspection_is_fail_closed_without_accessing_can() {
        let mut baseline = valid_runtime_snapshot();
        baseline.rx_dropped = 15_370;
        baseline.tx_dropped = 12;

        let inspector = FakeRuntimeInspector {
            snapshot: Some(baseline.clone()),
        };
        inspect_and_validate_runtime(&inspector, "fake0", &baseline).unwrap();

        let unavailable = FakeRuntimeInspector { snapshot: None };
        assert!(inspect_and_validate_runtime(&unavailable, "fake0", &baseline).is_err());

        let mut passive = baseline.clone();
        passive.state = "ERROR-PASSIVE".into();
        assert!(validate_runtime_snapshot(&baseline, &passive).is_err());

        let mut tec = baseline.clone();
        tec.tec = 1;
        assert!(validate_runtime_snapshot(&baseline, &tec).is_err());

        let mut rec = baseline.clone();
        rec.rec = 1;
        assert!(validate_runtime_snapshot(&baseline, &rec).is_err());
    }

    #[test]
    fn runtime_latches_every_xstat_and_netdev_counter_change() {
        let mut baseline = valid_runtime_snapshot();
        baseline.rx_dropped = 15_370;
        baseline.tx_dropped = 12;

        type CounterMutation = (&'static str, fn(&mut SocketCanRuntimeSnapshot));
        let mutations: [CounterMutation; 13] = [
            ("restarts", |snapshot| snapshot.restarts += 1),
            ("bus_error", |snapshot| snapshot.bus_error += 1),
            ("arbitration_lost", |snapshot| {
                snapshot.arbitration_lost += 1
            }),
            ("error_warning", |snapshot| snapshot.error_warning += 1),
            ("error_passive", |snapshot| snapshot.error_passive += 1),
            ("bus_off", |snapshot| snapshot.bus_off += 1),
            ("rx_errors", |snapshot| snapshot.rx_errors += 1),
            ("rx_dropped", |snapshot| snapshot.rx_dropped += 1),
            ("rx_over_errors", |snapshot| snapshot.rx_over_errors += 1),
            ("tx_errors", |snapshot| snapshot.tx_errors += 1),
            ("tx_dropped", |snapshot| snapshot.tx_dropped += 1),
            ("tx_carrier_errors", |snapshot| {
                snapshot.tx_carrier_errors += 1
            }),
            ("tx_collisions", |snapshot| snapshot.tx_collisions += 1),
        ];

        for (name, mutate) in mutations {
            let mut current = baseline.clone();
            mutate(&mut current);
            let error = validate_runtime_snapshot(&baseline, &current).unwrap_err();
            assert!(
                error.to_string().contains(name),
                "counter {name} produced an unhelpful error: {error:#}"
            );
        }

        let mut reset_drop_counter = baseline.clone();
        reset_drop_counter.rx_dropped = 0;
        assert!(validate_runtime_snapshot(&baseline, &reset_drop_counter).is_err());
    }

    #[test]
    fn runtime_rejects_every_counter_decrease_from_a_nonzero_baseline() {
        let mut baseline = valid_runtime_snapshot();
        baseline.restarts = 2;
        baseline.bus_error = 2;
        baseline.arbitration_lost = 2;
        baseline.error_warning = 10;
        baseline.error_passive = 10;
        baseline.bus_off = 2;
        baseline.rx_errors = 2;
        baseline.rx_dropped = 2;
        baseline.rx_over_errors = 2;
        baseline.tx_errors = 2;
        baseline.tx_dropped = 2;
        baseline.tx_carrier_errors = 2;
        baseline.tx_collisions = 2;
        validate_runtime_snapshot(&baseline, &baseline).unwrap();

        type CounterMutation = (&'static str, fn(&mut SocketCanRuntimeSnapshot));
        let mutations: [CounterMutation; 13] = [
            ("restarts", |snapshot| snapshot.restarts -= 1),
            ("bus_error", |snapshot| snapshot.bus_error -= 1),
            ("arbitration_lost", |snapshot| {
                snapshot.arbitration_lost -= 1
            }),
            ("error_warning", |snapshot| snapshot.error_warning -= 1),
            ("error_passive", |snapshot| snapshot.error_passive -= 1),
            ("bus_off", |snapshot| snapshot.bus_off -= 1),
            ("rx_errors", |snapshot| snapshot.rx_errors -= 1),
            ("rx_dropped", |snapshot| snapshot.rx_dropped -= 1),
            ("rx_over_errors", |snapshot| snapshot.rx_over_errors -= 1),
            ("tx_errors", |snapshot| snapshot.tx_errors -= 1),
            ("tx_dropped", |snapshot| snapshot.tx_dropped -= 1),
            ("tx_carrier_errors", |snapshot| {
                snapshot.tx_carrier_errors -= 1
            }),
            ("tx_collisions", |snapshot| snapshot.tx_collisions -= 1),
        ];
        for (name, mutate) in mutations {
            let mut current = baseline.clone();
            mutate(&mut current);
            let error = validate_runtime_snapshot(&baseline, &current).unwrap_err();
            assert!(
                error.to_string().contains(name),
                "counter {name} decrease produced an unhelpful error: {error:#}"
            );
        }
    }

    #[test]
    fn runtime_latches_acknowledged_warning_passive_increment_or_reset() {
        let mut baseline = valid_runtime_snapshot();
        baseline.error_warning = 10;
        baseline.error_passive = 10;
        validate_runtime_snapshot(&baseline, &baseline).unwrap();

        for mutate in [
            (|snapshot: &mut SocketCanRuntimeSnapshot| snapshot.error_warning = 11)
                as fn(&mut SocketCanRuntimeSnapshot),
            |snapshot| snapshot.error_warning = 0,
            |snapshot| snapshot.error_passive = 11,
            |snapshot| snapshot.error_passive = 0,
        ] {
            let mut current = baseline.clone();
            mutate(&mut current);
            assert!(validate_runtime_snapshot(&baseline, &current).is_err());
        }
    }

    #[test]
    #[ignore = "requires the field gs_usb adapter on a native-Ubuntu SocketCAN host"]
    fn real_socketcan_preflight_remains_read_only() {
        let interface = std::env::var("HEX_ARM_CAN_IFACE").unwrap_or_else(|_| "can0".into());
        let serial = std::env::var("HEX_ARM_CAN_SERIAL")
            .expect("set HEX_ARM_CAN_SERIAL to the exact 32-digit USB serial");
        let mut expected = expected();
        expected.adapter.serial = serial;
        expected.adapter.channel = std::env::var("HEX_ARM_CAN_CHANNEL")
            .unwrap_or_else(|_| "0".into())
            .parse()
            .expect("HEX_ARM_CAN_CHANNEL must be numeric");
        let _baseline = preflight_socketcan(&interface, &expected).unwrap();
    }
}
