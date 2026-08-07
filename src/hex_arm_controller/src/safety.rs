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
        if self.mode == OperatingMode::Fault && requested != OperatingMode::Disabled {
            anyhow::bail!("fault is latched; clear_fault is required");
        }
        if requested == OperatingMode::Active
            && !(profile_valid && calibrated && six_online && feedback_fresh)
        {
            anyhow::bail!("ACTIVE requires a valid calibrated profile and six fresh online motors");
        }
        if requested == OperatingMode::GravityComp
            && !(profile_valid && calibrated && six_online && feedback_fresh)
        {
            anyhow::bail!("GRAVITY_COMP additionally requires completed zero calibration");
        }
        self.mode = requested;
        Ok(())
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
    fn fault_is_latched_until_explicit_clear() {
        let mut state = SafetyState::default();
        state.latch_fault(42, "feedback timeout");
        state.latch_fault(99, "later fault");
        assert_eq!(state.fault_code, 42);
        assert!(state
            .request_mode(OperatingMode::Active, true, true, true, true)
            .is_err());
        state.clear_fault();
        assert_eq!(state.mode, OperatingMode::Disabled);
    }
}
