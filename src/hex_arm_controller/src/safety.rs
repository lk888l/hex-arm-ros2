#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum OperatingMode {
    Disabled = 1,
    Active = 2,
    Passive = 3,
    GravityComp = 4,
    Fault = 100,
    Calibrating = 101,
}

#[derive(Debug, Clone)]
pub struct SafetyState {
    pub mode: OperatingMode,
    pub fault_code: u32,
    pub fault_reason: String,
}

impl Default for SafetyState {
    fn default() -> Self {
        Self {
            mode: OperatingMode::Disabled,
            fault_code: 0,
            fault_reason: String::new(),
        }
    }
}

impl SafetyState {
    pub fn request_mode(
        &mut self,
        requested: OperatingMode,
        profile_valid: bool,
        calibrated: bool,
        six_online: bool,
        feedback_fresh: bool,
    ) -> anyhow::Result<()> {
        if self.mode == OperatingMode::Fault {
            if requested == OperatingMode::Disabled {
                // DISABLED is always safe to request, but it must not double as
                // a fault-reset operation.  Keep the software latch (including
                // its first fault code and reason) until clear_fault succeeds.
                return Ok(());
            }
            anyhow::bail!("fault is latched; clear_fault is required");
        }
        if matches!(
            requested,
            OperatingMode::Active | OperatingMode::Passive | OperatingMode::GravityComp
        ) && !(profile_valid && calibrated && six_online && feedback_fresh)
        {
            anyhow::bail!(
                "{requested:?} requires a valid calibrated profile and six fresh online motors"
            );
        }
        if requested == OperatingMode::Passive {
            anyhow::bail!(
                "PASSIVE is unavailable until torque-free operation is commissioned; use DISABLED"
            );
        }
        if requested == OperatingMode::GravityComp {
            anyhow::bail!(
                "GRAVITY_COMP is unavailable until a session deadman is implemented; use ACTIVE"
            );
        }
        self.mode = requested;
        Ok(())
    }

    pub fn disable_preserving_fault(&mut self) {
        if self.mode != OperatingMode::Fault {
            self.mode = OperatingMode::Disabled;
        }
    }

    pub fn latch_fault(&mut self, code: u32, reason: impl Into<String>) {
        if self.mode != OperatingMode::Fault {
            self.fault_code = code;
            self.fault_reason = reason.into();
        }
        self.mode = OperatingMode::Fault;
    }

    pub fn clear_fault(&mut self) {
        self.mode = OperatingMode::Disabled;
        self.fault_code = 0;
        self.fault_reason.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn startup_is_disabled_and_active_is_guarded() {
        let mut state = SafetyState::default();
        assert_eq!(state.mode, OperatingMode::Disabled);
        assert!(state
            .request_mode(OperatingMode::Active, false, false, true, true)
            .is_err());
        assert!(state
            .request_mode(OperatingMode::Active, true, false, true, true)
            .is_err());
        assert!(state
            .request_mode(OperatingMode::Active, true, true, true, true)
            .is_ok());
    }

    #[test]
    fn every_torque_capable_mode_requires_all_readiness_inputs() {
        for requested in [
            OperatingMode::Active,
            OperatingMode::Passive,
            OperatingMode::GravityComp,
        ] {
            for readiness in [
                [false, true, true, true],
                [true, false, true, true],
                [true, true, false, true],
                [true, true, true, false],
            ] {
                let mut state = SafetyState::default();
                let [profile_valid, calibrated, six_online, feedback_fresh] = readiness;
                let error = state
                    .request_mode(
                        requested,
                        profile_valid,
                        calibrated,
                        six_online,
                        feedback_fresh,
                    )
                    .unwrap_err();
                assert!(error.to_string().contains("valid calibrated profile"));
                assert_eq!(state.mode, OperatingMode::Disabled);
            }
        }
    }

    #[test]
    fn uncommissioned_modes_remain_fail_closed_when_all_readiness_inputs_are_true() {
        for (requested, expected_reason) in [
            (OperatingMode::Passive, "torque-free operation"),
            (OperatingMode::GravityComp, "session deadman"),
        ] {
            let mut state = SafetyState::default();
            let error = state
                .request_mode(requested, true, true, true, true)
                .unwrap_err();
            assert!(error.to_string().contains(expected_reason));
            assert_eq!(state.mode, OperatingMode::Disabled);
        }
    }

    #[test]
    fn fault_is_latched_until_explicit_clear() {
        let mut state = SafetyState::default();
        state.latch_fault(42, "feedback timeout");
        state.latch_fault(99, "later fault");
        assert_eq!(state.fault_code, 42);
        assert!(state
            .request_mode(OperatingMode::Disabled, true, true, true, true)
            .is_ok());
        assert_eq!(state.mode, OperatingMode::Fault);
        assert_eq!(state.fault_code, 42);
        assert_eq!(state.fault_reason, "feedback timeout");
        state.disable_preserving_fault();
        assert_eq!(state.mode, OperatingMode::Fault);
        assert!(state
            .request_mode(OperatingMode::Active, true, true, true, true)
            .is_err());
        state.clear_fault();
        assert_eq!(state.mode, OperatingMode::Disabled);
    }
}
