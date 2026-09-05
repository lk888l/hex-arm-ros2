//! 常驻 task：监听 TPDO1（0x180..=0x1FF）和 TPDO2（0x280..=0x2FF）帧。
//!
//! 每帧做三件事：
//!
//! 1. **liveness**：成功解码后分别更新 `last_tpdo1` / `last_tpdo2`；offline → online 时发
//!    [`Cia402Event::NodeOnline`]。
//! 2. **decode**（M4+）：用 [`super::codec::decode_tpdo1`] /
//!    [`super::codec::decode_tpdo2`] 把字节翻译成 `Measurements` 字段；每条
//!    有效 TPDO 都会保留两路 `0x603F` last-error 诊断；只有 TPDO2 的
//!    `0x6041` Fault bit 决定当前 [`Logic::Error`]。
//! 3. **error edge detection**：`logic` 从 非-Error 跳到 Error 时发一次
//!    [`Cia402Event::EnteredError`]。
//!
//! 设计取舍：原来设计是 per-motor runner，最终决定**全局监听器一把做完**。
//! 节省 N×2 路 subscription + 一个 task per motor，又方便单点维护
//! `last_tpdo`/`logic`/`measurements` 三者的一致性。详见 `DESIGN.md` §5。

use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use can_transport::{CanBus, CanFilter, CanFrame, CanId, CanIoError};
use tokio::sync::broadcast;
use tokio_util::sync::CancellationToken;

use super::codec::{decode_tpdo1, decode_tpdo2, status_word_to_logic, Tpdo1Frame, Tpdo2Frame};
use super::events::Cia402Event;
use super::motor_entry::{MotorEntry, MotorEntryInner};
use super::types::Logic;

/// function code 掩码：取出 11-bit ID 的高 4 位，匹配 0x180/0x280/0x380/...
const TPDO_FC_MASK: u16 = 0x780;
const TPDO1_BASE: u16 = 0x180;
const TPDO2_BASE: u16 = 0x280;

/// 哪条 TPDO 被收到了 —— 决定按 12B 还是 10B 解码。
#[derive(Copy, Clone)]
enum TpdoKind {
    Tpdo1,
    Tpdo2,
}

impl TpdoKind {
    fn tag(self) -> &'static str {
        match self {
            TpdoKind::Tpdo1 => "TPDO1",
            TpdoKind::Tpdo2 => "TPDO2",
        }
    }
}

pub(crate) async fn run_tpdo_listener(
    bus: Arc<dyn CanBus>,
    motors: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    velocity_window: Duration,
    cancel: CancellationToken,
) {
    let mut rx1 = match bus
        .subscribe(CanFilter::standard(TPDO1_BASE, TPDO_FC_MASK))
        .await
    {
        Ok(r) => r,
        Err(e) => {
            log::error!("TPDO listener: subscribe TPDO1 failed: {e}");
            return;
        }
    };
    let mut rx2 = match bus
        .subscribe(CanFilter::standard(TPDO2_BASE, TPDO_FC_MASK))
        .await
    {
        Ok(r) => r,
        Err(e) => {
            log::error!("TPDO listener: subscribe TPDO2 failed: {e}");
            return;
        }
    };

    loop {
        tokio::select! {
            _ = cancel.cancelled() => {
                log::debug!("TPDO listener cancelled");
                return;
            }
            res = rx1.recv() => handle_frame(res, TpdoKind::Tpdo1, &motors, &events_tx, velocity_window),
            res = rx2.recv() => handle_frame(res, TpdoKind::Tpdo2, &motors, &events_tx, velocity_window),
        }
    }
}

fn handle_frame(
    res: Result<CanFrame, CanIoError>,
    kind: TpdoKind,
    motors: &Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    events_tx: &broadcast::Sender<Cia402Event>,
    velocity_window: Duration,
) {
    let frame = match res {
        Ok(f) => f,
        Err(e) => {
            log::warn!("TPDO listener ({}) rx error: {e}", kind.tag());
            return;
        }
    };
    let CanId::Standard(cob_id) = frame.id() else {
        return;
    };
    let nid = (cob_id & 0x7F) as u8;
    if nid == 0 {
        return;
    }

    let entry = motors.read().unwrap().get(&nid).cloned();
    let Some(entry) = entry else {
        // TPDO 来自我们没记录过的 nid。通常不会发生 —— HB 会先到。
        // 不主动建条目（避免假数据污染列表）。
        return;
    };

    let now = Instant::now();

    // Decode before touching liveness.  A malformed frame must not keep a
    // motor online or make stale feedback appear fresh.
    let decoded = match kind {
        TpdoKind::Tpdo1 => decode_tpdo1(frame.data()).map(DecodedTpdo::Tpdo1),
        TpdoKind::Tpdo2 => decode_tpdo2(frame.data()).map(DecodedTpdo::Tpdo2),
    };
    let Some(decoded) = decoded else {
        log::warn!(
            "{} from nid 0x{nid:02X}: bad length {} (want >={})",
            kind.tag(),
            frame.data().len(),
            match kind {
                TpdoKind::Tpdo1 => 12,
                TpdoKind::Tpdo2 => 10,
            }
        );
        return;
    };

    // ===== update measurements / logic / independent freshness =====

    let (live_state, error_event, online_event) = {
        let mut inner = entry.inner.lock().unwrap();
        inner.last_tpdo = Some(now);
        match kind {
            TpdoKind::Tpdo1 => inner.last_tpdo1 = Some(now),
            TpdoKind::Tpdo2 => inner.last_tpdo2 = Some(now),
        }
        let became_online = !inner.online;
        inner.online = true;

        let was_error = matches!(inner.logic, Some(Logic::Error { .. }));
        match decoded {
            DecodedTpdo::Tpdo1(f) => {
                apply_tpdo1(&mut inner, f, velocity_window);
            }
            DecodedTpdo::Tpdo2(f) => {
                apply_tpdo2(&mut inner.measurements, f);
            }
        }

        let last_error = [
            inner.measurements.tpdo2_error_code,
            inner.measurements.tpdo1_error_code,
        ]
        .into_iter()
        .flatten()
        .find(|code| *code != 0);
        let new_logic = inner.measurements.status_word.map(|status_word| {
            status_word_to_logic(status_word, inner.target_mode, last_error.unwrap_or(0))
        });
        let is_error = matches!(new_logic, Some(Logic::Error { .. }));
        if let Some(new_logic) = new_logic {
            inner.logic = Some(new_logic);
        }
        let error_event = if !was_error && is_error {
            match inner.logic.clone() {
                Some(Logic::Error { kind, raw_code }) => Some(Cia402Event::EnteredError {
                    nid,
                    kind,
                    raw: raw_code,
                }),
                _ => None,
            }
        } else {
            None
        };

        let online_event = became_online.then_some(Cia402Event::NodeOnline { nid });
        // 把当前 inner 拍成 LiveState 给 publish 用 —— 锁还在；publish 在锁外做。
        (inner.build_live_state(now), error_event, online_event)
    };
    // ↑ 锁已 drop，可以发事件 / publish 了（都不能在持锁中做）

    if let Some(ev) = online_event {
        let _ = events_tx.send(ev);
    }
    if let Some(ev) = error_event {
        let _ = events_tx.send(ev);
    }
    // 把最新 measurements / logic / connection 推给 status() / subscribe_status()。
    entry.publish(live_state);
}

#[derive(Clone, Copy)]
enum DecodedTpdo {
    Tpdo1(Tpdo1Frame),
    Tpdo2(Tpdo2Frame),
}

fn apply_tpdo1(inner: &mut MotorEntryInner, f: Tpdo1Frame, velocity_window: Duration) {
    // 先取出标量再借 measurements，避免同时可变 + 不可变借 inner。
    let peak = inner.peak_torque_nm;
    // 滤波速度：用电机时间戳对单圈位置做解卷绕 + 滑动窗口最小二乘。
    let velocity = inner
        .vel_filter
        .update(f.timestamp_us, f.position_rev, velocity_window);

    let m = &mut inner.measurements;
    m.position_rev = Some(f.position_rev);
    m.timestamp_us = Some(f.timestamp_us);
    m.tpdo1_error_code = Some(f.error_code);
    // torque: i16 ‰ of peak → Nm；没缓存 peak_torque 时留空。
    m.torque_nm = peak.map(|p| f.torque_permille as f32 / 1000.0 * p);
    // 样本不足 / 刚重置时 velocity 为 None，保留上一帧的值不动。
    if let Some(v) = velocity {
        m.velocity_rev_per_s = Some(v);
    }
}

fn apply_tpdo2(m: &mut super::types::Measurements, f: Tpdo2Frame) {
    m.status_word = Some(f.status_word);
    m.control_word_readback = Some(f.control_word_readback);
    m.driver_temp_c = Some(f.driver_temp_c());
    m.motor_temp_c = Some(f.motor_temp_c());
    m.tpdo2_error_code = Some(f.error_code);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{MotorErrorKind, MotorMode};

    const NID: u8 = 1;
    type Fixture = (
        Arc<MotorEntry>,
        Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
        broadcast::Sender<Cia402Event>,
    );

    fn fixture() -> Fixture {
        let entry = Arc::new(MotorEntry::new(NID));
        let motors = Arc::new(RwLock::new(HashMap::from([(NID, entry.clone())])));
        let (events, _) = broadcast::channel(16);
        (entry, motors, events)
    }

    fn tpdo1(error_code: u16) -> CanFrame {
        let mut payload = [0u8; 12];
        payload[..4].copy_from_slice(&0.25f32.to_le_bytes());
        payload[4..8].copy_from_slice(&1234u32.to_le_bytes());
        payload[10..12].copy_from_slice(&error_code.to_le_bytes());
        CanFrame::new_fd(TPDO1_BASE + NID as u16, &payload, true).unwrap()
    }

    fn tpdo2_with_status(error_code: u16, status_word: u16) -> CanFrame {
        let mut payload = [0u8; 10];
        payload[..2].copy_from_slice(&status_word.to_le_bytes());
        payload[6..8].copy_from_slice(&0x000Fu16.to_le_bytes());
        payload[8..10].copy_from_slice(&error_code.to_le_bytes());
        CanFrame::new_fd(TPDO2_BASE + NID as u16, &payload, true).unwrap()
    }

    fn tpdo2(error_code: u16) -> CanFrame {
        tpdo2_with_status(error_code, 0x0027)
    }

    #[test]
    fn malformed_tpdos_do_not_refresh_liveness_or_snapshot() {
        let (entry, motors, events) = fixture();
        let short_frames = [
            (
                CanFrame::new_fd(TPDO1_BASE + NID as u16, &[0u8; 11], true).unwrap(),
                TpdoKind::Tpdo1,
            ),
            (
                CanFrame::new_fd(TPDO2_BASE + NID as u16, &[0u8; 9], true).unwrap(),
                TpdoKind::Tpdo2,
            ),
        ];
        for (frame, kind) in short_frames {
            handle_frame(Ok(frame), kind, &motors, &events, Duration::from_millis(15));
        }

        let inner = entry.inner.lock().unwrap();
        assert!(!inner.online);
        assert!(inner.last_tpdo.is_none());
        assert!(inner.last_tpdo1.is_none());
        assert!(inner.last_tpdo2.is_none());
        drop(inner);
        assert!(!entry.snapshot.load_full().connection.online);
    }

    #[test]
    fn tpdo_streams_have_independent_freshness() {
        let (entry, motors, events) = fixture();
        handle_frame(
            Ok(tpdo2(0)),
            TpdoKind::Tpdo2,
            &motors,
            &events,
            Duration::from_millis(15),
        );

        let snapshot = entry.snapshot.load_full();
        assert!(snapshot.connection.last_tpdo.is_some());
        assert!(snapshot.connection.last_tpdo1.is_none());
        assert!(snapshot.connection.last_tpdo2.is_some());
        assert_eq!(snapshot.measurements.control_word_readback, Some(0x000F));
        assert!(!snapshot
            .connection
            .required_tpdos_fresh(Instant::now(), Duration::from_secs(1)));
    }

    #[test]
    fn retained_last_error_does_not_create_current_fault_without_status_fault_bit() {
        let (entry, motors, events) = fixture();
        entry.inner.lock().unwrap().target_mode = Some(MotorMode::Mit);

        handle_frame(
            Ok(tpdo2(0)),
            TpdoKind::Tpdo2,
            &motors,
            &events,
            Duration::from_millis(15),
        );
        handle_frame(
            Ok(tpdo1(0x2310)),
            TpdoKind::Tpdo1,
            &motors,
            &events,
            Duration::from_millis(15),
        );
        assert_eq!(
            entry.snapshot.load_full().logic,
            Some(Logic::Enabled(MotorMode::Mit))
        );

        // 0x603F remains available as diagnostics but cannot override a clean
        // current status word.
        handle_frame(
            Ok(tpdo2(0)),
            TpdoKind::Tpdo2,
            &motors,
            &events,
            Duration::from_millis(15),
        );
        let snapshot = entry.snapshot.load_full();
        assert_eq!(snapshot.measurements.tpdo1_error_code, Some(0x2310));
        assert_eq!(snapshot.logic, Some(Logic::Enabled(MotorMode::Mit)));

        // The same exact last-error becomes the active fault cause only when a
        // fresh TPDO2 sets 0x6041 bit 3.
        handle_frame(
            Ok(tpdo2_with_status(0x2310, 0x0008)),
            TpdoKind::Tpdo2,
            &motors,
            &events,
            Duration::from_millis(15),
        );
        assert!(matches!(
            entry.snapshot.load_full().logic,
            Some(Logic::Error {
                kind: MotorErrorKind::OverCurrent,
                raw_code: 0x2310
            })
        ));
    }

    #[test]
    fn field_observed_8130_with_0231_is_disabled_last_error_not_current_fault() {
        let (entry, motors, events) = fixture();
        handle_frame(
            Ok(tpdo1(0x8130)),
            TpdoKind::Tpdo1,
            &motors,
            &events,
            Duration::from_millis(15),
        );
        handle_frame(
            Ok(tpdo2_with_status(0x8130, 0x0231)),
            TpdoKind::Tpdo2,
            &motors,
            &events,
            Duration::from_millis(15),
        );
        let snapshot = entry.snapshot.load_full();
        assert_eq!(snapshot.logic, Some(Logic::Disabled));
        assert_eq!(snapshot.measurements.tpdo1_error_code, Some(0x8130));
        assert_eq!(snapshot.measurements.tpdo2_error_code, Some(0x8130));
    }
}
