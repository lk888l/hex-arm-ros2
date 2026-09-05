//! `initialize()` 完整序列 + 默认 TPDO1/TPDO2 配方。详见 `DESIGN.md` §6。
//!
//! 步骤回顾：
//! 1. lifecycle → `Initializing`；发 `Cia402Event::Initializing`
//! 2. NMT `EnterPreOperational`（目标=该 nid）
//!    等 HB 反馈 NMT 状态变成 `PreOperational`（最多 2 × `motor_heartbeat_period`）
//! 3. SDO 读 `0x6041` 状态字探活（顺便确认 SDO 在 PreOp 仍然通）
//! 4. 写 `0x6040=0x0006` 并重读 `0x6041`，在任何 OD/PDO 改写前确认 non-OE
//! 5. 用 [`crate::canopen::tpdo_config::build_tpdo_config_writes`] 配 TPDO1（高速，1 ms）
//! 6. 同上配 TPDO2（低速，20 ms）
//! 7. best-effort 读厂家运行时常量（`0x6076` peak_torque / `0x2003:07`
//!    MIT factor）；并把 `0x2003:06` 预设为 1000
//! 8. NMT `StartRemoteNode` → Operational，等 HB 反馈变成 `Operational`
//! 9. 配置 `0x1016` 心跳监控，等待一个超时窗口后重读 `0x6041` 验证。
//!    初始化前后只要发现 CiA402 Fault 就立即失败；初始化**永远不会**写 fault-reset
//!    (`0x6040 = 0x80`)。排除物理原因后，必须由调用方显式执行 `clear_error()`。
//! 10. lifecycle → `Initialized`；发 `Cia402Event::Initialized`
//!
//! 失败时 [`LifecycleRollback`] 自动把 lifecycle 退回 `Identified`（如果
//! identity 已知）或 `Unknown`。普通 TPDO 配置不会回滚；但在第一次 CAN
//! 操作前会记录本会话触碰/所有权，使 manager/backend 能在错误或取消后的
//! shutdown 中先确认 non-OE，再安全撤销心跳消费者。调用方之后可以重试。
//!
//! ## 关于默认 TPDO 映射
//!
//! v0.1 的默认映射是为 HexMeow CiA402 电机量身做的：
//!
//! - **TPDO1（高速 1 ms）**：`0x6064` actual_position(32b) + `0x1013`
//!   high_res_timestamp(32b) + `0x6077` actual_torque(16b) + `0x603F`
//!   error_code(16b) = **12 字节**
//! - **TPDO2（低速 20 ms）**：`0x6041` status(16b) + `0x2204:01` drv_temp(16b)
//!   plus `0x2204:02` motor_temp(16b) + `0x6040` ctrl(16b) + `0x603F`
//!   error_code(16b) = **10 字节**
//!
//! 注意：
//! - 速度故意**不** map，由上位机用 (pos_now-pos_prev)/(ts_now-ts_prev) 算
//!   （HexMeow 的 `0x6064` 是单圈 f32，需要在 host 侧做多圈累积）。
//! - `0x1013`、`0x2204:01/02` 是 vendor-specific 实现，标准 CiA402 不保证
//!   有；其他厂家电机要走自定义 recipe，未来会暴露 `initialize_with_recipes()` API。

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use can_transport::CanBus;
use tokio::sync::broadcast;

use crate::canopen::{
    heartbeat::encode_consumer_heartbeat_entry,
    nmt::{self, NmtCommand, NmtState},
    sdo,
    tpdo_config::{build_tpdo_config_writes, TpdoCommParams, TpdoEntry, TpdoRecipe},
};
use crate::error::{Error, Result};

use super::events::Cia402Event;
use super::manager::Cia402ManagerOptions;
use super::motor_entry::MotorEntry;
use super::types::MotorLifecycle;

/// Verified nodes touched by this manager's initialization session, together
/// with the exact heartbeat consumer value the session may later arm.
///
/// The entry is installed before the first NMT/SDO CAN operation. That makes it
/// deliberately conservative: every partial/cancelled initialization gets a
/// confirmed Shutdown, including failures before the `0x1016` write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SessionHeartbeatConsumer {
    pub expected: u32,
    /// Set before attempting `0x1016 = 0`. If that destructive write or any
    /// post-write verification is interrupted, a later zero readback is not a
    /// fast-path success: Shutdown/non-OE must be proven again first.
    pub zero_requires_non_oe_confirmation: bool,
}

pub(crate) type SessionHeartbeatConsumers = Arc<Mutex<HashMap<u8, SessionHeartbeatConsumer>>>;

/// Safely remove one heartbeat consumer that this manager session attempted to
/// install.
///
/// `expected_consumer` is the exact value recorded before the initialize SDO
/// was awaited.  A zero readback means the attempted write never landed (or a
/// previous cleanup already completed) and is therefore already safe.  A
/// different non-zero value is never modified because it may belong to another
/// controller configuration.
///
/// A current CiA402 Fault is allowed as long as OperationEnabled is clear: the
/// fault remains latched and this cleanup performs no fault reset.  Conversely,
/// failure to acknowledge Shutdown or authoritatively prove non-OE prevents the
/// zero write. Any failure after zero may have landed triggers a best-effort
/// restore; tracking also remembers that a later actual=0 retry must prove
/// Shutdown/non-OE again.
pub(crate) async fn cleanup_session_heartbeat_consumer(
    bus: &dyn CanBus,
    nid: u8,
    session_heartbeat_consumers: &SessionHeartbeatConsumers,
    opts: &Cia402ManagerOptions,
) -> Result<()> {
    let tracked = session_heartbeat_consumers
        .lock()
        .unwrap()
        .get(&nid)
        .copied()
        .ok_or_else(|| {
            Error::Internal(format!(
                "nid 0x{nid:02X}: no session heartbeat consumer is tracked"
            ))
        })?;
    let expected_consumer = tracked.expected;
    let timeout = Some(opts.sdo_timeout);

    // Physical state is independent of heartbeat-consumer ownership. Every
    // verified node touched by initialization is disabled first, so even an
    // unreadable 0x1016 cannot bypass the non-OE exit contract.
    let disable_result = request_shutdown_and_confirm_non_oe(bus, nid, timeout).await;
    // The ownership read is safe even when disable confirmation failed. It is
    // used only to enrich the error; no 0x1016 write is permitted unless the
    // disable result was successful.
    let consumer_result = sdo::upload_u32(bus, nid, 0x1016, 1, timeout).await;
    let actual = match (disable_result, consumer_result) {
        (Ok(()), Ok(actual)) => actual,
        (Ok(()), Err(read_error)) => return Err(read_error),
        (Err(disable_error), Ok(actual)) if actual != 0 && actual != expected_consumer => {
            let ownership_error = heartbeat_ownership_error(nid, actual, expected_consumer);
            return Err(Error::Internal(format!(
                "{ownership_error}; touched-node confirmed Shutdown also failed: {disable_error}"
            )));
        }
        (Err(disable_error), Ok(_)) => return Err(disable_error),
        (Err(disable_error), Err(read_error)) => {
            return Err(Error::Internal(format!(
                "touched-node confirmed Shutdown failed: {disable_error}; authoritative \
                 0x1016 read also failed: {read_error}"
            )));
        }
    };
    if actual == 0 {
        if tracked.zero_requires_non_oe_confirmation {
            log::info!(
                "nid 0x{nid:02X}: zero heartbeat consumer from an interrupted cleanup was \
                 accepted only after a new Shutdown/non-OE confirmation"
            );
        } else {
            log::info!(
                "nid 0x{nid:02X}: attempted session heartbeat write did not land; \
                 consumer is zero and drive is newly confirmed non-OE"
            );
        }
        return Ok(());
    }
    if actual != expected_consumer {
        return Err(heartbeat_ownership_error(nid, actual, expected_consumer));
    }

    // Mark the destructive phase before awaiting the zero write. Cancellation
    // or a lost SDO response can therefore never turn a later actual=0 retry
    // into the benign "arm write never landed" fast path above.
    {
        let mut consumers = session_heartbeat_consumers.lock().unwrap();
        let record = consumers.get_mut(&nid).ok_or_else(|| {
            Error::Internal(format!(
                "nid 0x{nid:02X}: heartbeat cleanup tracking disappeared before zero write"
            ))
        })?;
        if record.expected != expected_consumer {
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: heartbeat cleanup ownership changed before zero write"
            )));
        }
        record.zero_requires_non_oe_confirmation = true;
    }

    let post_zero_result: Result<()> = async {
        sdo::download_u32(bus, nid, 0x1016, 1, 0, timeout).await?;
        let readback = sdo::upload_u32(bus, nid, 0x1016, 1, timeout).await?;
        if readback != 0 {
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: heartbeat consumer cleanup readback is \
                 0x{readback:08X}, expected 0"
            )));
        }
        let after = sdo::upload_u16(bus, nid, 0x6041, 0, timeout).await?;
        if !super::codec::status_word_is_confirmed_non_torque(after) {
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: drive is not in a confirmed non-torque state after heartbeat cleanup \
                 (0x6041=0x{after:04X})"
            )));
        }
        Ok(())
    }
    .await;

    if let Err(post_zero_error) = post_zero_result {
        // Once zero may have landed, restore the exact session-owned consumer
        // before reporting failure. This keeps heartbeat loss available as the
        // final safety action while the host broadcaster is still alive.
        let restore_result = async {
            sdo::download_u32(bus, nid, 0x1016, 1, expected_consumer, timeout).await?;
            let restored = sdo::upload_u32(bus, nid, 0x1016, 1, timeout).await?;
            if restored != expected_consumer {
                return Err(Error::Internal(format!(
                    "nid 0x{nid:02X}: heartbeat consumer restore readback is \
                     0x{restored:08X}, expected 0x{expected_consumer:08X}"
                )));
            }
            Ok(())
        }
        .await;
        if restore_result.is_ok() {
            if let Some(record) = session_heartbeat_consumers.lock().unwrap().get_mut(&nid) {
                if record.expected == expected_consumer {
                    record.zero_requires_non_oe_confirmation = false;
                }
            }
        }
        return match restore_result {
            Ok(()) => Err(Error::Internal(format!(
                "{post_zero_error}; restored this session's heartbeat consumer to \
                 0x{expected_consumer:08X}; cleanup remains failed"
            ))),
            Err(restore_error) => Err(Error::Internal(format!(
                "{post_zero_error}; restoring this session's heartbeat consumer also failed: \
                 {restore_error}"
            ))),
        };
    }
    log::info!("nid 0x{nid:02X}: session heartbeat consumer disarmed and drive confirmed non-OE");
    Ok(())
}

fn heartbeat_ownership_error(nid: u8, actual: u32, expected: u32) -> Error {
    Error::Internal(format!(
        "nid 0x{nid:02X}: refusing heartbeat cleanup: authoritative 0x1016:01 is \
         0x{actual:08X}, not this session's expected 0x{expected:08X}"
    ))
}

async fn request_shutdown_and_confirm_non_oe(
    bus: &dyn CanBus,
    nid: u8,
    timeout: Option<Duration>,
) -> Result<()> {
    sdo::download_u16(bus, nid, 0x6040, 0, 0x0006, timeout).await?;
    let status_word = sdo::upload_u16(bus, nid, 0x6041, 0, timeout).await?;
    if !super::codec::status_word_is_confirmed_non_torque(status_word) {
        return Err(Error::Internal(format!(
            "nid 0x{nid:02X}: refusing heartbeat cleanup: Shutdown was not confirmed in a non-torque state \
             (0x6041=0x{status_word:04X}); 0x1016 remains unchanged"
        )));
    }
    if super::codec::status_word_has_fault(status_word) {
        // This is intentional: no reset is sent, so the active fault remains a
        // fail-closed condition while we remove only the host-exit watchdog.
        log::warn!(
            "nid 0x{nid:02X}: heartbeat cleanup sees Fault after fault reaction completed \
             (0x6041=0x{status_word:04X}); fault remains latched"
        );
    }
    Ok(())
}

/// 默认 TPDO1（高速 1 ms）映射：位置 + 时间戳 + 力矩 + 错误码 = 12 字节。
/// 速度由上位机用 (pos_now-pos_prev)/(ts_now-ts_prev) 计算。
pub const DEFAULT_TPDO1_ENTRIES: &[TpdoEntry] = &[
    TpdoEntry {
        index: 0x6064,
        subindex: 0,
        bit_len: 32,
    }, // actual_position（HexMeow CiA402: 单圈 f32；标准 CiA402: i32 encoder pulse）
    TpdoEntry {
        index: 0x1013,
        subindex: 0,
        bit_len: 32,
    }, // high_resolution_time_stamp (us, u32)
    TpdoEntry {
        index: 0x6077,
        subindex: 0,
        bit_len: 16,
    }, // actual_torque (i16, ‰ of peak)
    TpdoEntry {
        index: 0x603F,
        subindex: 0,
        bit_len: 16,
    }, // error_code (u16)
];

/// 默认 TPDO2（低速 20 ms）映射：状态字 + 驱动器/电机温度 + 控制字 + 错误码 = 10 字节。
pub const DEFAULT_TPDO2_ENTRIES: &[TpdoEntry] = &[
    TpdoEntry {
        index: 0x6041,
        subindex: 0,
        bit_len: 16,
    }, // status_word
    TpdoEntry {
        index: 0x2204,
        subindex: 1,
        bit_len: 16,
    }, // driver_temperature_x10 (vendor-specific, i16 ×0.1 ℃)
    TpdoEntry {
        index: 0x2204,
        subindex: 2,
        bit_len: 16,
    }, // motor_temperature_x10 (vendor-specific, i16 ×0.1 ℃)
    TpdoEntry {
        index: 0x6040,
        subindex: 0,
        bit_len: 16,
    }, // control_word（回读当前 CW）
    TpdoEntry {
        index: 0x603F,
        subindex: 0,
        bit_len: 16,
    }, // error_code (u16) —— 和 TPDO1 重复一份，方便低速通道也能看到
];

/// 默认 TPDO1 通信参数：异步事件 + 0.5 ms inhibit + 1 ms 周期（1000 Hz）。
pub const DEFAULT_TPDO1_COMM: TpdoCommParams = TpdoCommParams {
    transmission_type: 255,
    inhibit_time_x100us: 5, // 0.5 ms
    event_timer_ms: 1,      // 1000 Hz
};

/// 默认 TPDO2 通信参数：异步事件 + 19 ms inhibit + 20 ms 周期（50 Hz）。
pub const DEFAULT_TPDO2_COMM: TpdoCommParams = TpdoCommParams {
    transmission_type: 255,
    inhibit_time_x100us: 190, // 19 ms
    event_timer_ms: 20,       // 50 Hz
};

/// 给指定 nid 构造默认 TPDO1 recipe (`cob_id = 0x180 + nid`)。
pub fn default_tpdo1_recipe(nid: u8) -> TpdoRecipe {
    TpdoRecipe {
        tpdo_index: 0,
        cob_id: 0x180 + nid as u16,
        entries: DEFAULT_TPDO1_ENTRIES.to_vec(),
        comm: DEFAULT_TPDO1_COMM,
    }
}

/// 给指定 nid 构造默认 TPDO2 recipe (`cob_id = 0x280 + nid`)。
pub fn default_tpdo2_recipe(nid: u8) -> TpdoRecipe {
    TpdoRecipe {
        tpdo_index: 1,
        cob_id: 0x280 + nid as u16,
        entries: DEFAULT_TPDO2_ENTRIES.to_vec(),
        comm: DEFAULT_TPDO2_COMM,
    }
}

fn fault_requires_explicit_recovery(nid: u8, status_word: u16, phase: &str) -> Error {
    Error::Internal(format!(
        "nid 0x{nid:02X}: initialization refused: CiA402 Fault is set during {phase} \
         (status_word=0x{status_word:04X}); resolve the physical cause, explicitly call \
         clear_error(), then retry initialize; process restart never resets motor faults"
    ))
}

/// 完整 initialize 序列。**调用方必须先把 lifecycle != Initializing 并保留
/// `inflight_ops` 中的标记**，详见 [`crate::cia402::manager::Cia402Manager::initialize`]。
pub(crate) async fn run_initialize(
    bus: &dyn CanBus,
    entry: Arc<MotorEntry>,
    events_tx: &broadcast::Sender<Cia402Event>,
    opts: &Cia402ManagerOptions,
    session_heartbeat_consumers: &SessionHeartbeatConsumers,
) -> Result<()> {
    let nid = entry.node_id;
    let sdo_timeout = Some(opts.sdo_timeout);

    // 1. lifecycle → Initializing；同时清掉 control-side 缓存（旧的 mode/logic
    //    在 motor 重新配过 OD 之后都失效了）
    {
        let mut inner = entry.inner.lock().unwrap();
        if matches!(inner.lifecycle, MotorLifecycle::Initializing) {
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: initialize already running"
            )));
        }
        inner.lifecycle = MotorLifecycle::Initializing;
        inner.target_mode = None;
        inner.logic = None;
        inner.peak_torque_nm = None;
        inner.mit_kp_kd_factor = None;
        inner.measurements = Default::default();
        inner.vel_filter = Default::default();
    }
    let _ = events_tx.send(Cia402Event::Initializing { nid });

    // 失败时自动回退 lifecycle 到 Identified / Unknown
    let mut rollback = LifecycleRollback::new(entry.clone());

    // 2. NMT EnterPreOperational + 等待 HB 反馈
    let preop_cmd = nmt::build_nmt_command(NmtCommand::EnterPreOperational, nid)?;
    // Record this verified node as initialization-touched before the first CAN
    // operation. Even if cancellation/failure happens before 0x1016 is ever
    // written, shutdown must issue CW=0x06 and authoritatively prove non-OE.
    let timeout_ms = opts
        .consumer_heartbeat_timeout
        .as_millis()
        .min(u16::MAX as u128) as u16;
    let consumer = encode_consumer_heartbeat_entry(opts.heartbeat_node_id, timeout_ms);
    session_heartbeat_consumers.lock().unwrap().insert(
        nid,
        SessionHeartbeatConsumer {
            expected: consumer,
            zero_requires_non_oe_confirmation: false,
        },
    );
    bus.send(preop_cmd).await?;
    wait_for_nmt_state(
        &entry,
        NmtState::PreOperational,
        opts.motor_heartbeat_period * 2,
    )
    .await?;
    log::info!("nid 0x{nid:02X}: NMT = PreOperational");

    // 3. SDO 探活：读 0x6041 status_word
    let sw = sdo::upload_u16(bus, nid, 0x6041, 0, sdo_timeout).await?;
    if super::codec::status_word_has_fault(sw) {
        return Err(fault_requires_explicit_recovery(
            nid,
            sw,
            "pre-operational probe",
        ));
    }

    // 4. NMT PreOperational does not imply CiA402 disabled. Establish and prove a
    // fresh Shutdown state before changing any PDO mapping or vendor OD value.
    request_shutdown_and_confirm_non_oe(bus, nid, sdo_timeout).await?;
    log::info!("nid 0x{nid:02X}: pre-configuration drive state confirmed non-OE");

    // 5. 配置 TPDO1（高速）
    apply_tpdo_recipe(bus, nid, &default_tpdo1_recipe(nid), sdo_timeout).await?;

    // 6. 配置 TPDO2（低速）
    apply_tpdo_recipe(bus, nid, &default_tpdo2_recipe(nid), sdo_timeout).await?;

    // 7. best-effort 读厂家运行时常量（HexMeow CiA402 vendor-specific）：
    //    - 0x6076 Motor Peak Torque (REAL32, mNm) —— 后面 Torque target 用
    //    - 0x2003:07 MIT KP/KD Factor (REAL32) —— 后面 Mit target 用
    //    - 0x2003:06 MIT KP/KD Limit (UNSIGNED16) 预设为 1000 (full PD authority)
    //    任意一条失败只 log warn，不影响 init 成功；用对应模式时再报错。
    read_runtime_constants(bus, &entry, sdo_timeout).await;
    let _ = sdo::download_u16(bus, nid, 0x2003, 0x06, 1000, sdo_timeout)
        .await
        .map_err(|e| {
            log::debug!(
                "nid 0x{nid:02X}: 0x2003:06 (MIT PD limit) not writable ({e}); \
                 Mit mode will rely on motor default"
            );
        });

    // 8. NMT StartRemoteNode → Operational（PDO 开始流；CiA402 故障与否都会发
    //    PDO 反馈，所以即便此刻仍带故障，上位机也已经能看到数据）。
    let op_cmd = nmt::build_nmt_command(NmtCommand::StartRemoteNode, nid)?;
    bus.send(op_cmd).await?;
    wait_for_nmt_state(
        &entry,
        NmtState::Operational,
        opts.motor_heartbeat_period * 2,
    )
    .await?;
    log::info!("nid 0x{nid:02X}: NMT = Operational");

    // 9. 配置心跳监控并等待一个完整超时窗口验证。这里故意不先关闭 0x1016，
    //    更不会写 0x6040 bit 7；启动/重启不能替操作者清除一个锁存故障。
    let verify_wait = opts.consumer_heartbeat_timeout + Duration::from_millis(100);
    sdo::download_u32(bus, nid, 0x1016, 1, consumer, sdo_timeout).await?;
    tokio::time::sleep(verify_wait).await;
    let sw = sdo::upload_u16(bus, nid, 0x6041, 0, sdo_timeout).await?;
    if super::codec::status_word_has_fault(sw) {
        return Err(fault_requires_explicit_recovery(
            nid,
            sw,
            "heartbeat-monitor verification",
        ));
    }
    log::info!(
        "nid 0x{nid:02X}: heartbeat monitor armed without fault reset \
         (sw=0x{sw:04X}, 0x1016=0x{consumer:08X})"
    );

    // 10. 标 Initialized + 拆除 rollback
    {
        let mut inner = entry.inner.lock().unwrap();
        inner.lifecycle = MotorLifecycle::Initialized;
    }
    rollback.disarm();
    let _ = events_tx.send(Cia402Event::Initialized { nid });
    Ok(())
}

/// Best-effort 读 0x6076 (Motor Peak Torque) + 0x2003:07 (MIT KP/KD Factor)，
/// 缓存到 [`MotorEntry`]。失败只 log，不返回 Error。
async fn read_runtime_constants(
    bus: &dyn CanBus,
    entry: &Arc<MotorEntry>,
    sdo_timeout: Option<Duration>,
) {
    let nid = entry.node_id;

    // 0x6076 Motor Peak Torque：REAL32，单位 **Nm**（huayi.md 明确：6076h
    // 峰值力矩，单位为Nm）。直接缓存，不做单位换算。
    match sdo::upload_f32(bus, nid, 0x6076, 0, sdo_timeout).await {
        Ok(nm) => {
            log::info!("nid 0x{nid:02X}: 0x6076 (Motor Peak Torque) = {nm} Nm");
            entry.inner.lock().unwrap().peak_torque_nm = Some(nm);
        }
        Err(e) => {
            log::warn!(
                "nid 0x{nid:02X}: 0x6076 (Motor Peak Torque) not readable ({e}); \
                 Torque-mode target writes will be unavailable"
            );
        }
    }

    // 0x2003:07 MIT KP/KD Factor：REAL32。物理 Kp [Nm/Rev] = kp_int × factor。
    match sdo::upload_f32(bus, nid, 0x2003, 0x07, sdo_timeout).await {
        Ok(factor) => {
            log::info!("nid 0x{nid:02X}: 0x2003:07 (MIT KP/KD Factor) = {factor}");
            entry.inner.lock().unwrap().mit_kp_kd_factor = Some(factor);
        }
        Err(e) => {
            log::warn!(
                "nid 0x{nid:02X}: 0x2003:07 (MIT KP/KD Factor) not readable ({e}); \
                 Mit-mode target writes will be unavailable"
            );
        }
    }
}

/// 一次性把 recipe 编译出的所有 SDO 写顺序下发给电机。
async fn apply_tpdo_recipe(
    bus: &dyn CanBus,
    nid: u8,
    recipe: &TpdoRecipe,
    sdo_timeout: Option<Duration>,
) -> Result<()> {
    let writes = build_tpdo_config_writes(recipe)?;
    log::debug!(
        "nid 0x{nid:02X}: TPDO{} cob_id=0x{:03X}: {} SDO ops ({} bytes/frame)",
        recipe.tpdo_index + 1,
        recipe.cob_id,
        writes.len(),
        recipe.total_bytes(),
    );
    for w in &writes {
        sdo::download(bus, nid, w.index, w.subindex, &w.data, sdo_timeout).await?;
    }
    Ok(())
}

/// 轮询 [`MotorEntry::nmt_state`]（由 discovery task 在每帧 HB 时写入）直到
/// 等于 `target` 或超时。
async fn wait_for_nmt_state(
    entry: &Arc<MotorEntry>,
    target: NmtState,
    timeout: Duration,
) -> Result<()> {
    // 先看一眼现在的状态，命中就直接返回
    {
        let inner = entry.inner.lock().unwrap();
        if inner.nmt_state == Some(target) {
            return Ok(());
        }
    }
    let deadline = Instant::now() + timeout;
    let poll_period = Duration::from_millis(20);
    loop {
        tokio::time::sleep(poll_period).await;
        {
            let inner = entry.inner.lock().unwrap();
            if inner.nmt_state == Some(target) {
                return Ok(());
            }
        }
        if Instant::now() >= deadline {
            let observed = entry.inner.lock().unwrap().nmt_state;
            return Err(Error::Internal(format!(
                "nid 0x{:02X}: timeout waiting NMT {:?} (last observed {:?})",
                entry.node_id, target, observed,
            )));
        }
    }
}

/// RAII：函数提前返回（错误 / panic）时把 lifecycle 退回。
struct LifecycleRollback {
    entry: Arc<MotorEntry>,
    armed: bool,
}

impl LifecycleRollback {
    fn new(entry: Arc<MotorEntry>) -> Self {
        Self { entry, armed: true }
    }

    /// 成功路径在最后调用一次。
    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for LifecycleRollback {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        let mut inner = self.entry.inner.lock().unwrap();
        // 仅当还卡在 Initializing 时回退（避免覆盖成功后的状态）
        if matches!(inner.lifecycle, MotorLifecycle::Initializing) {
            inner.lifecycle = if inner.identity.is_some() {
                MotorLifecycle::Identified
            } else {
                MotorLifecycle::Unknown
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Mutex;

    use async_trait::async_trait;
    use can_transport::{
        CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanRx, FrameKind,
    };

    struct MockRx {
        rx: broadcast::Receiver<CanFrame>,
    }

    #[async_trait]
    impl CanRx for MockRx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            match self.rx.recv().await {
                Ok(frame) => Ok(frame),
                Err(broadcast::error::RecvError::Closed) => Err(CanIoError::Disconnected),
                Err(broadcast::error::RecvError::Lagged(dropped)) => {
                    Err(CanIoError::Lagged { dropped })
                }
            }
        }

        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            match self.rx.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(broadcast::error::TryRecvError::Empty) => Ok(None),
                Err(broadcast::error::TryRecvError::Closed) => Err(CanIoError::Disconnected),
                Err(broadcast::error::TryRecvError::Lagged(dropped)) => {
                    Err(CanIoError::Lagged { dropped })
                }
            }
        }
    }

    struct MockBus {
        nid: u8,
        entry: Arc<MotorEntry>,
        responses: broadcast::Sender<CanFrame>,
        sent: Mutex<Vec<CanFrame>>,
        status_words: Mutex<VecDeque<u16>>,
        consumer_heartbeat: Mutex<u32>,
        block_consumer_arm: AtomicBool,
        consumer_arm_started: AtomicBool,
        fail_consumer_upload: AtomicBool,
    }

    impl MockBus {
        fn new(entry: Arc<MotorEntry>, status_words: impl IntoIterator<Item = u16>) -> Self {
            let (responses, _) = broadcast::channel(64);
            Self {
                nid: entry.node_id,
                entry,
                responses,
                sent: Mutex::new(Vec::new()),
                status_words: Mutex::new(status_words.into_iter().collect()),
                consumer_heartbeat: Mutex::new(0),
                block_consumer_arm: AtomicBool::new(false),
                consumer_arm_started: AtomicBool::new(false),
                fail_consumer_upload: AtomicBool::new(false),
            }
        }

        fn upload_payload(&self, index: u16, subindex: u8) -> Vec<u8> {
            match (index, subindex) {
                (0x6041, 0) => {
                    let mut words = self.status_words.lock().unwrap();
                    let value = words
                        .pop_front()
                        .or_else(|| words.back().copied())
                        .unwrap_or(0x0040);
                    value.to_le_bytes().to_vec()
                }
                (0x1016, 1) => self
                    .consumer_heartbeat
                    .lock()
                    .unwrap()
                    .to_le_bytes()
                    .to_vec(),
                (0x6076, 0) | (0x2003, 0x07) => 1.0_f32.to_le_bytes().to_vec(),
                _ => vec![0; 4],
            }
        }

        fn wrote_fault_reset(&self) -> bool {
            self.sent.lock().unwrap().iter().any(|frame| {
                matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                    && matches!(frame.kind(), FrameKind::Data)
                    && frame.data().len() == 8
                    && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == 0x6040
                    && frame.data()[3] == 0
                    && u16::from_le_bytes([frame.data()[4], frame.data()[5]]) == 0x0080
            })
        }

        fn consumer_heartbeat(&self) -> u32 {
            *self.consumer_heartbeat.lock().unwrap()
        }

        fn wrote_control_word(&self, value: u16) -> bool {
            self.sent.lock().unwrap().iter().any(|frame| {
                matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                    && matches!(frame.kind(), FrameKind::Data)
                    && frame.data().len() == 8
                    && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == 0x6040
                    && frame.data()[3] == 0
                    && u16::from_le_bytes([frame.data()[4], frame.data()[5]]) == value
            })
        }

        fn wrote_consumer_download(&self) -> bool {
            self.sent.lock().unwrap().iter().any(|frame| {
                matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                    && frame.data().len() == 8
                    && frame.data()[0] != 0x40
                    && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == 0x1016
                    && frame.data()[3] == 1
            })
        }

        fn first_control_word_position(&self, value: u16) -> Option<usize> {
            self.sent.lock().unwrap().iter().position(|frame| {
                matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                    && frame.data().len() == 8
                    && frame.data()[0] != 0x40
                    && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == 0x6040
                    && frame.data()[3] == 0
                    && u16::from_le_bytes([frame.data()[4], frame.data()[5]]) == value
            })
        }

        fn first_sdo_index_position(&self, index: u16) -> Option<usize> {
            self.sent.lock().unwrap().iter().position(|frame| {
                matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                    && frame.data().len() == 8
                    && frame.data()[0] != 0x40
                    && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == index
            })
        }

        fn block_consumer_arm_write(&self) {
            self.block_consumer_arm.store(true, Ordering::Release);
        }

        fn consumer_arm_write_started(&self) -> bool {
            self.consumer_arm_started.load(Ordering::Acquire)
        }

        fn fail_consumer_upload(&self) {
            self.fail_consumer_upload.store(true, Ordering::Release);
        }

        fn is_consumer_upload(&self, frame: &CanFrame) -> bool {
            matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                && frame.data().len() == 8
                && frame.data()[0] == 0x40
                && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == 0x1016
                && frame.data()[3] == 1
        }

        fn is_nonzero_consumer_download(&self, frame: &CanFrame) -> bool {
            matches!(frame.id(), CanId::Standard(id) if id == 0x600 + self.nid as u16)
                && frame.data().len() == 8
                && frame.data()[0] != 0x40
                && u16::from_le_bytes([frame.data()[1], frame.data()[2]]) == 0x1016
                && frame.data()[3] == 1
                && u32::from_le_bytes([
                    frame.data()[4],
                    frame.data()[5],
                    frame.data()[6],
                    frame.data()[7],
                ]) != 0
        }

        fn handle_nmt(&self, frame: &CanFrame) {
            if frame.id() != CanId::Standard(0) || frame.data().len() != 2 {
                return;
            }
            let state = match frame.data()[0] {
                value if value == NmtCommand::EnterPreOperational as u8 => {
                    Some(NmtState::PreOperational)
                }
                value if value == NmtCommand::StartRemoteNode as u8 => Some(NmtState::Operational),
                _ => None,
            };
            if let Some(state) = state {
                self.entry.inner.lock().unwrap().nmt_state = Some(state);
            }
        }

        fn handle_sdo(&self, frame: &CanFrame) {
            let CanId::Standard(cob_id) = frame.id() else {
                return;
            };
            if cob_id != 0x600 + self.nid as u16 || frame.data().len() != 8 {
                return;
            }
            let request = frame.data();
            let index = u16::from_le_bytes([request[1], request[2]]);
            let subindex = request[3];
            let mut response = [0_u8; 8];
            response[1..=3].copy_from_slice(&request[1..=3]);
            if request[0] == 0x40 {
                let payload = self.upload_payload(index, subindex);
                response[0] = 0x43 | (((4 - payload.len()) as u8) << 2);
                response[4..4 + payload.len()].copy_from_slice(&payload);
            } else {
                if (index, subindex) == (0x1016, 1) {
                    *self.consumer_heartbeat.lock().unwrap() =
                        u32::from_le_bytes([request[4], request[5], request[6], request[7]]);
                }
                response[0] = 0x60;
            }
            let response =
                CanFrame::new_data(CanId::Standard(0x580 + self.nid as u16), &response).unwrap();
            let _ = self.responses.send(response);
        }
    }

    #[async_trait]
    impl CanBus for MockBus {
        async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
            if self.block_consumer_arm.load(Ordering::Acquire)
                && self.is_nonzero_consumer_download(&frame)
            {
                self.consumer_arm_started.store(true, Ordering::Release);
                std::future::pending::<()>().await;
            }
            if self.fail_consumer_upload.load(Ordering::Acquire) && self.is_consumer_upload(&frame)
            {
                self.sent.lock().unwrap().push(frame);
                return Err(CanIoError::Disconnected);
            }
            self.sent.lock().unwrap().push(frame);
            self.handle_nmt(&frame);
            self.handle_sdo(&frame);
            Ok(())
        }

        async fn subscribe(
            &self,
            _filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            Ok(Box::new(MockRx {
                rx: self.responses.subscribe(),
            }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    async fn initialize_with_status_words(
        status_words: impl IntoIterator<Item = u16>,
    ) -> (Result<()>, Arc<MockBus>) {
        let entry = Arc::new(MotorEntry::new(0x21));
        entry.inner.lock().unwrap().lifecycle = MotorLifecycle::Identified;
        let bus = Arc::new(MockBus::new(entry.clone(), status_words));
        let (events, _) = broadcast::channel(8);
        let options = Cia402ManagerOptions {
            sdo_timeout: Duration::from_millis(50),
            motor_heartbeat_period: Duration::from_millis(5),
            consumer_heartbeat_timeout: Duration::from_millis(1),
            ..Default::default()
        };
        let session_heartbeat_consumers = Arc::new(Mutex::new(HashMap::new()));
        let result = run_initialize(
            bus.as_ref(),
            entry,
            &events,
            &options,
            &session_heartbeat_consumers,
        )
        .await;
        (result, bus)
    }

    fn tracked_consumer(
        nid: u8,
        expected: u32,
        zero_requires_non_oe_confirmation: bool,
    ) -> SessionHeartbeatConsumers {
        Arc::new(Mutex::new(HashMap::from([(
            nid,
            SessionHeartbeatConsumer {
                expected,
                zero_requires_non_oe_confirmation,
            },
        )])))
    }

    #[test]
    fn default_tpdo1_is_12_bytes_4_entries() {
        let r = default_tpdo1_recipe(0x10);
        assert_eq!(r.total_bytes(), 12);
        assert_eq!(r.entries.len(), 4);
        assert_eq!(r.cob_id, 0x190);
        assert_eq!(r.tpdo_index, 0);
        assert!(r.validate().is_ok());
    }

    #[test]
    fn default_tpdo2_is_10_bytes_5_entries() {
        let r = default_tpdo2_recipe(0x10);
        assert_eq!(r.total_bytes(), 10);
        assert_eq!(r.entries.len(), 5);
        assert_eq!(r.cob_id, 0x290);
        assert_eq!(r.tpdo_index, 1);
        assert!(r.validate().is_ok());
    }

    #[test]
    fn default_tpdo1_timing_is_high_speed() {
        assert_eq!(DEFAULT_TPDO1_COMM.transmission_type, 255);
        assert_eq!(DEFAULT_TPDO1_COMM.inhibit_time_x100us, 5);
        assert_eq!(DEFAULT_TPDO1_COMM.event_timer_ms, 1);
    }

    #[test]
    fn default_tpdo2_timing_is_low_speed() {
        assert_eq!(DEFAULT_TPDO2_COMM.transmission_type, 255);
        assert_eq!(DEFAULT_TPDO2_COMM.inhibit_time_x100us, 190);
        assert_eq!(DEFAULT_TPDO2_COMM.event_timer_ms, 20);
    }

    #[test]
    fn tpdo1_and_tpdo2_use_different_cob_and_index() {
        let r1 = default_tpdo1_recipe(0x21);
        let r2 = default_tpdo2_recipe(0x21);
        assert_eq!(r1.cob_id, 0x1A1);
        assert_eq!(r2.cob_id, 0x2A1);
        assert_ne!(r1.tpdo_index, r2.tpdo_index);
    }

    #[tokio::test]
    async fn default_initialize_never_writes_fault_reset() {
        let (result, bus) = initialize_with_status_words([0x0040, 0x0040]).await;
        result.unwrap();
        assert!(
            !bus.wrote_fault_reset(),
            "default initialize must never write 0x6040 = 0x0080"
        );
    }

    #[tokio::test]
    async fn initially_oe_drive_is_disabled_before_any_tpdo_remap() {
        let (result, bus) = initialize_with_status_words([0x0027, 0x0040, 0x0040]).await;
        result.unwrap();

        let shutdown = bus
            .first_control_word_position(0x0006)
            .expect("initialize must issue CiA402 Shutdown");
        let first_tpdo_write = bus
            .first_sdo_index_position(0x1800)
            .expect("initialize must configure TPDO1");
        assert!(
            shutdown < first_tpdo_write,
            "TPDO mapping changed before initial OE state was disabled"
        );
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn faulted_initialize_fails_closed_without_reset() {
        let (result, bus) = initialize_with_status_words([0x0008]).await;
        let error = result.unwrap_err().to_string();
        assert!(error.contains("explicitly call clear_error"), "{error}");
        assert!(
            error.contains("restart never resets motor faults"),
            "{error}"
        );
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn heartbeat_monitor_fault_fails_closed_without_reset() {
        let (result, bus) = initialize_with_status_words([0x0040, 0x0040, 0x0008]).await;
        let error = result.unwrap_err().to_string();
        assert!(error.contains("heartbeat-monitor verification"), "{error}");
        assert!(error.contains("explicitly call clear_error"), "{error}");
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn partial_initialize_fault_is_cleaned_while_fault_remains_non_oe() {
        let entry = Arc::new(MotorEntry::new(0x21));
        entry.inner.lock().unwrap().lifecycle = MotorLifecycle::Identified;
        // Probe clean, heartbeat verification Fault, then two authoritative
        // cleanup reads remain Fault but non-OE. Cleanup must not reset it.
        let bus = Arc::new(MockBus::new(
            entry.clone(),
            [0x0040, 0x0008, 0x0008, 0x0008],
        ));
        let (events, _) = broadcast::channel(8);
        let options = Cia402ManagerOptions {
            sdo_timeout: Duration::from_millis(50),
            motor_heartbeat_period: Duration::from_millis(5),
            consumer_heartbeat_timeout: Duration::from_millis(1),
            ..Default::default()
        };
        let tracked = Arc::new(Mutex::new(HashMap::new()));

        let error = run_initialize(bus.as_ref(), entry, &events, &options, &tracked)
            .await
            .unwrap_err()
            .to_string();
        assert!(error.contains("heartbeat-monitor verification"), "{error}");
        let expected = tracked.lock().unwrap().get(&0x21).unwrap().expected;
        assert_eq!(bus.consumer_heartbeat(), expected);

        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap();
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(bus.wrote_control_word(0x0006));
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn preop_probe_failure_is_tracked_before_any_consumer_write() {
        let entry = Arc::new(MotorEntry::new(0x21));
        entry.inner.lock().unwrap().lifecycle = MotorLifecycle::Identified;
        let bus = Arc::new(MockBus::new(entry.clone(), [0x0008, 0x0008]));
        let (events, _) = broadcast::channel(8);
        let options = Cia402ManagerOptions {
            sdo_timeout: Duration::from_millis(50),
            motor_heartbeat_period: Duration::from_millis(5),
            ..Default::default()
        };
        let tracked = Arc::new(Mutex::new(HashMap::new()));

        run_initialize(bus.as_ref(), entry, &events, &options, &tracked)
            .await
            .unwrap_err();
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(tracked.lock().unwrap().contains_key(&0x21));

        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap();
        assert!(bus.wrote_control_word(0x0006));
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn cancelled_initialize_records_and_cleans_landed_heartbeat_write() {
        let entry = Arc::new(MotorEntry::new(0x21));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.lifecycle = MotorLifecycle::Identified;
            inner.identity = Some(crate::types::MotorIdentity {
                node_id: 0x21,
                vendor_id: 1,
                product_code: 2,
                revision_number: 3,
                serial_number: 4,
                product_name: None,
            });
        }
        let bus = Arc::new(MockBus::new(entry.clone(), [0x0040, 0x0040, 0x0040]));
        let (events, _) = broadcast::channel(8);
        let options = Cia402ManagerOptions {
            sdo_timeout: Duration::from_millis(50),
            motor_heartbeat_period: Duration::from_millis(5),
            // Keep run_initialize in its post-write verification sleep long
            // enough to deterministically cancel it.
            consumer_heartbeat_timeout: Duration::from_secs(5),
            ..Default::default()
        };
        let tracked = Arc::new(Mutex::new(HashMap::new()));

        let task_bus = bus.clone();
        let task_entry = entry.clone();
        let task_options = options.clone();
        let task_tracked = tracked.clone();
        let task = tokio::spawn(async move {
            run_initialize(
                task_bus.as_ref(),
                task_entry,
                &events,
                &task_options,
                &task_tracked,
            )
            .await
        });

        tokio::time::timeout(Duration::from_secs(1), async {
            while bus.consumer_heartbeat() == 0 {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("initialize did not reach the 0x1016 write");
        task.abort();
        let _ = task.await;

        assert_eq!(
            entry.inner.lock().unwrap().lifecycle,
            MotorLifecycle::Identified,
            "cancellation must drop the lifecycle rollback before shutdown cleanup"
        );
        let expected = tracked.lock().unwrap().get(&0x21).unwrap().expected;
        assert_eq!(bus.consumer_heartbeat(), expected);
        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap();
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(bus.wrote_control_word(0x0006));
    }

    #[tokio::test]
    async fn cancelled_unlanded_consumer_write_still_disables_initially_oe_drive() {
        let entry = Arc::new(MotorEntry::new(0x21));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.lifecycle = MotorLifecycle::Identified;
            inner.identity = Some(crate::types::MotorIdentity {
                node_id: 0x21,
                vendor_id: 1,
                product_code: 2,
                revision_number: 3,
                serial_number: 4,
                product_name: None,
            });
        }
        // The pre-op probe observes OE (0x0027). After cancellation, the mock
        // reports the result of the cleanup Shutdown as non-OE (0x0040).
        let bus = Arc::new(MockBus::new(entry.clone(), [0x0027, 0x0040]));
        bus.block_consumer_arm_write();
        let (events, _) = broadcast::channel(8);
        let options = Cia402ManagerOptions {
            sdo_timeout: Duration::from_millis(50),
            motor_heartbeat_period: Duration::from_millis(5),
            consumer_heartbeat_timeout: Duration::from_millis(1),
            ..Default::default()
        };
        let tracked = Arc::new(Mutex::new(HashMap::new()));

        let task_bus = bus.clone();
        let task_entry = entry.clone();
        let task_options = options.clone();
        let task_tracked = tracked.clone();
        let task = tokio::spawn(async move {
            run_initialize(
                task_bus.as_ref(),
                task_entry,
                &events,
                &task_options,
                &task_tracked,
            )
            .await
        });
        tokio::time::timeout(Duration::from_secs(1), async {
            while !bus.consumer_arm_write_started() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("initialize did not reach the blocked consumer arm write");

        assert_eq!(
            bus.consumer_heartbeat(),
            0,
            "arm write must not have landed"
        );
        assert!(tracked.lock().unwrap().contains_key(&0x21));
        task.abort();
        let _ = task.await;
        assert_eq!(
            entry.inner.lock().unwrap().lifecycle,
            MotorLifecycle::Identified
        );

        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap();
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(
            bus.wrote_control_word(0x0006),
            "actual=0 must not bypass confirmed Shutdown for a touched node"
        );
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn no_landed_heartbeat_write_still_requires_confirmed_shutdown() {
        let entry = Arc::new(MotorEntry::new(0x21));
        let bus = Arc::new(MockBus::new(entry, []));
        let options = Cia402ManagerOptions::default();
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let tracked = tracked_consumer(0x21, expected, false);

        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap();
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(bus.wrote_control_word(0x0006));
    }

    #[tokio::test]
    async fn unreadable_consumer_still_gets_confirmed_shutdown_before_error() {
        let entry = Arc::new(MotorEntry::new(0x21));
        let bus = Arc::new(MockBus::new(entry, [0x0040]));
        bus.fail_consumer_upload();
        let options = Cia402ManagerOptions::default();
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let tracked = tracked_consumer(0x21, expected, false);

        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap_err();
        assert!(bus.wrote_control_word(0x0006));
        assert!(!bus.wrote_consumer_download());
        assert_eq!(bus.consumer_heartbeat(), 0);
    }

    #[tokio::test]
    async fn torque_capable_cleanup_states_are_rejected_and_keep_heartbeat_armed() {
        for status_word in [0x0027, 0x0007, 0x000F] {
            let entry = Arc::new(MotorEntry::new(0x21));
            let bus = Arc::new(MockBus::new(entry, [status_word]));
            let options = Cia402ManagerOptions::default();
            let expected = encode_consumer_heartbeat_entry(0x10, 250);
            let tracked = tracked_consumer(0x21, expected, false);
            *bus.consumer_heartbeat.lock().unwrap() = expected;

            let error = cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
                .await
                .unwrap_err()
                .to_string();
            assert!(
                error.contains("not confirmed in a non-torque state"),
                "{error}"
            );
            assert_eq!(bus.consumer_heartbeat(), expected);
            assert!(bus.wrote_control_word(0x0006));
            assert!(!bus.wrote_fault_reset());
        }
    }

    #[tokio::test]
    async fn post_zero_operation_enabled_restores_the_session_watchdog() {
        let entry = Arc::new(MotorEntry::new(0x21));
        // First status confirms Shutdown; the post-zero status unexpectedly
        // reports OE and must force restoration before the error is returned.
        let bus = Arc::new(MockBus::new(entry, [0x0040, 0x0027]));
        let options = Cia402ManagerOptions::default();
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let tracked = tracked_consumer(0x21, expected, false);
        *bus.consumer_heartbeat.lock().unwrap() = expected;

        let error = cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap_err()
            .to_string();
        assert!(
            error.contains("not in a confirmed non-torque state"),
            "{error}"
        );
        assert!(
            error.contains("restored this session's heartbeat"),
            "{error}"
        );
        assert_eq!(bus.consumer_heartbeat(), expected);
        assert!(
            !tracked
                .lock()
                .unwrap()
                .get(&0x21)
                .unwrap()
                .zero_requires_non_oe_confirmation
        );
        assert!(!bus.wrote_fault_reset());
    }

    #[tokio::test]
    async fn zero_from_interrupted_cleanup_requires_new_non_oe_confirmation() {
        let entry = Arc::new(MotorEntry::new(0x21));
        let bus = Arc::new(MockBus::new(entry, [0x0027, 0x0040]));
        let options = Cia402ManagerOptions::default();
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let tracked = tracked_consumer(0x21, expected, true);

        let first = cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap_err()
            .to_string();
        assert!(
            first.contains("not confirmed in a non-torque state"),
            "{first}"
        );
        assert_eq!(bus.consumer_heartbeat(), 0);

        cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap();
        assert_eq!(bus.consumer_heartbeat(), 0);
        assert!(bus.wrote_control_word(0x0006));
    }

    #[tokio::test]
    async fn foreign_nonzero_heartbeat_consumer_is_never_modified() {
        let entry = Arc::new(MotorEntry::new(0x21));
        let bus = Arc::new(MockBus::new(entry, []));
        let options = Cia402ManagerOptions::default();
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let foreign = encode_consumer_heartbeat_entry(0x11, 250);
        let tracked = tracked_consumer(0x21, expected, false);
        *bus.consumer_heartbeat.lock().unwrap() = foreign;

        let error = cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap_err()
            .to_string();
        assert!(error.contains("not this session's expected"), "{error}");
        assert_eq!(bus.consumer_heartbeat(), foreign);
        assert!(bus.wrote_control_word(0x0006));
    }

    #[tokio::test]
    async fn initially_oe_foreign_consumer_is_preserved_but_touched_axis_is_disabled() {
        let entry = Arc::new(MotorEntry::new(0x21));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.lifecycle = MotorLifecycle::Identified;
            inner.identity = Some(crate::types::MotorIdentity {
                node_id: 0x21,
                vendor_id: 1,
                product_code: 2,
                revision_number: 3,
                serial_number: 4,
                product_name: None,
            });
        }
        // Initialization observes OE. Its own 0x1016 write is cancelled before
        // landing; cleanup then observes the post-CW6 non-OE status.
        let bus = Arc::new(MockBus::new(entry.clone(), [0x0027, 0x0040]));
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let foreign = encode_consumer_heartbeat_entry(0x11, 250);
        *bus.consumer_heartbeat.lock().unwrap() = foreign;
        bus.block_consumer_arm_write();
        let (events, _) = broadcast::channel(8);
        let options = Cia402ManagerOptions {
            sdo_timeout: Duration::from_millis(50),
            motor_heartbeat_period: Duration::from_millis(5),
            consumer_heartbeat_timeout: Duration::from_millis(250),
            ..Default::default()
        };
        let tracked = Arc::new(Mutex::new(HashMap::new()));

        let task_bus = bus.clone();
        let task_entry = entry.clone();
        let task_options = options.clone();
        let task_tracked = tracked.clone();
        let task = tokio::spawn(async move {
            run_initialize(
                task_bus.as_ref(),
                task_entry,
                &events,
                &task_options,
                &task_tracked,
            )
            .await
        });
        tokio::time::timeout(Duration::from_secs(1), async {
            while !bus.consumer_arm_write_started() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("initialize did not reach blocked consumer write");
        task.abort();
        let _ = task.await;

        assert_eq!(
            tracked.lock().unwrap().get(&0x21).unwrap().expected,
            expected
        );
        let error = cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap_err()
            .to_string();
        assert!(error.contains("not this session's expected"), "{error}");
        assert_eq!(bus.consumer_heartbeat(), foreign);
        assert!(bus.wrote_control_word(0x0006));
    }

    #[tokio::test]
    async fn foreign_ownership_and_failed_non_oe_confirmation_are_both_reported() {
        let entry = Arc::new(MotorEntry::new(0x21));
        let bus = Arc::new(MockBus::new(entry, [0x0027]));
        let options = Cia402ManagerOptions::default();
        let expected = encode_consumer_heartbeat_entry(0x10, 250);
        let foreign = encode_consumer_heartbeat_entry(0x11, 250);
        let tracked = tracked_consumer(0x21, expected, false);
        *bus.consumer_heartbeat.lock().unwrap() = foreign;

        let error = cleanup_session_heartbeat_consumer(bus.as_ref(), 0x21, &tracked, &options)
            .await
            .unwrap_err()
            .to_string();
        assert!(error.contains("not this session's expected"), "{error}");
        assert!(error.contains("confirmed Shutdown also failed"), "{error}");
        assert!(
            error.contains("not confirmed in a non-torque state"),
            "{error}"
        );
        assert_eq!(bus.consumer_heartbeat(), foreign);
        assert!(bus.wrote_control_word(0x0006));
    }
}
