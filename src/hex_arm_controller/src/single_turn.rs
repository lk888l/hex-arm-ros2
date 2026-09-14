//! Fail-closed handling for HexMeow's single-turn `0x6064` position.
//!
//! The drive reports a canonical position in `[-0.5, 0.5)` revolutions.  A
//! revolute joint, however, needs one continuous branch of that periodic
//! value.  The initial branch must be recovered from independently calibrated
//! joint limits (and, during commissioning, can additionally be checked
//! against a surveyed startup pose).  Subsequent samples are unwrapped only
//! when their step is small enough to be unambiguous.
//!
//! This module is deliberately independent of the CAN backend and profile
//! schema.  It sends no frames and is suitable for use before a drive is
//! enabled.

use std::fmt;

pub const SINGLE_TURN_MIN_REV: f32 = -0.5;
pub const SINGLE_TURN_MAX_REV: f32 = 0.5;

const HALF_TURN_AMBIGUITY_EPS_REV: f32 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BranchWindow {
    pub lower_rev: f32,
    pub upper_rev: f32,
    /// Small allowance for calibrated joint-limit and sensor noise.
    pub tolerance_rev: f32,
}

impl BranchWindow {
    pub fn validate(self) -> Result<Self, SingleTurnError> {
        if !self.lower_rev.is_finite()
            || !self.upper_rev.is_finite()
            || !self.tolerance_rev.is_finite()
            || self.lower_rev >= self.upper_rev
            || self.tolerance_rev < 0.0
        {
            return Err(SingleTurnError::InvalidBranchWindow);
        }

        // With a full-turn (or wider) window, one raw sample can represent two
        // allowed continuous positions.  No amount of local arithmetic can
        // recover the lost turn count, so reject that configuration.
        if self.upper_rev - self.lower_rev + 2.0 * self.tolerance_rev >= 1.0 {
            return Err(SingleTurnError::AmbiguousBranchWindow);
        }
        Ok(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StartupReference {
    pub position_rev: f32,
    pub max_error_rev: f32,
}

impl StartupReference {
    fn validate(self) -> Result<Self, SingleTurnError> {
        if !self.position_rev.is_finite()
            || !self.max_error_rev.is_finite()
            || self.max_error_rev < 0.0
            || self.max_error_rev >= 0.5
        {
            return Err(SingleTurnError::InvalidStartupReference);
        }
        Ok(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SingleTurnError {
    NonFiniteRawPosition,
    NonCanonicalRawPosition {
        raw_rev: f32,
    },
    InvalidBranchWindow,
    AmbiguousBranchWindow,
    NoBranchInWindow {
        raw_rev: f32,
    },
    AmbiguousInitialBranch {
        raw_rev: f32,
    },
    InvalidStartupReference,
    StartupReferenceMismatch {
        resolved_rev: f32,
        reference_rev: f32,
        max_error_rev: f32,
    },
    InvalidMaxStep,
    AmbiguousHalfTurnStep {
        delta_rev: f32,
    },
    StepTooLarge {
        delta_rev: f32,
        max_step_rev: f32,
    },
    PositionOutsideWindow {
        position_rev: f32,
    },
    Faulted,
    InvalidCommandWindow,
    CommandWindowTouchesWrapSeam,
}

impl fmt::Display for SingleTurnError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteRawPosition => write!(f, "single-turn position is not finite"),
            Self::NonCanonicalRawPosition { raw_rev } => write!(
                f,
                "single-turn position {raw_rev} rev is outside [-0.5, 0.5)"
            ),
            Self::InvalidBranchWindow => write!(f, "invalid continuous-position branch window"),
            Self::AmbiguousBranchWindow => write!(
                f,
                "continuous-position branch window spans one turn and is ambiguous"
            ),
            Self::NoBranchInWindow { raw_rev } => write!(
                f,
                "no periodic branch of raw position {raw_rev} rev lies in the joint window"
            ),
            Self::AmbiguousInitialBranch { raw_rev } => write!(
                f,
                "more than one branch of raw position {raw_rev} rev lies in the joint window"
            ),
            Self::InvalidStartupReference => write!(f, "invalid startup position reference"),
            Self::StartupReferenceMismatch {
                resolved_rev,
                reference_rev,
                max_error_rev,
            } => write!(
                f,
                "resolved position {resolved_rev} rev is not within {max_error_rev} rev of startup reference {reference_rev} rev"
            ),
            Self::InvalidMaxStep => write!(f, "maximum unwrap step must be finite and in (0, 0.5) rev"),
            Self::AmbiguousHalfTurnStep { delta_rev } => write!(
                f,
                "single-turn sample changed by {delta_rev} rev; direction is ambiguous"
            ),
            Self::StepTooLarge {
                delta_rev,
                max_step_rev,
            } => write!(
                f,
                "unwrapped step {delta_rev} rev exceeds maximum {max_step_rev} rev"
            ),
            Self::PositionOutsideWindow { position_rev } => write!(
                f,
                "unwrapped position {position_rev} rev is outside the calibrated joint window"
            ),
            Self::Faulted => write!(f, "single-turn unwrapper is faulted"),
            Self::InvalidCommandWindow => write!(f, "invalid compressed-MIT position window"),
            Self::CommandWindowTouchesWrapSeam => write!(
                f,
                "compressed-MIT position window reaches the single-turn wrap seam"
            ),
        }
    }
}

impl std::error::Error for SingleTurnError {}

fn validate_raw(raw_rev: f32) -> Result<(), SingleTurnError> {
    if !raw_rev.is_finite() {
        return Err(SingleTurnError::NonFiniteRawPosition);
    }
    if !(SINGLE_TURN_MIN_REV..SINGLE_TURN_MAX_REV).contains(&raw_rev) {
        return Err(SingleTurnError::NonCanonicalRawPosition { raw_rev });
    }
    Ok(())
}

/// Resolve an initial single-turn reading to its only allowed continuous
/// branch.  `window` should be derived from calibrated ROS joint limits,
/// direction and zero offset in motor-revolution units.
pub fn resolve_initial_branch(
    raw_rev: f32,
    window: BranchWindow,
    startup_reference: Option<StartupReference>,
) -> Result<f32, SingleTurnError> {
    validate_raw(raw_rev)?;
    let window = window.validate()?;
    let lower = (window.lower_rev - window.tolerance_rev) as f64;
    let upper = (window.upper_rev + window.tolerance_rev) as f64;
    let raw = raw_rev as f64;
    let first_turn = (lower - raw).ceil() as i64;
    let last_turn = (upper - raw).floor() as i64;

    if first_turn > last_turn {
        return Err(SingleTurnError::NoBranchInWindow { raw_rev });
    }
    if first_turn != last_turn {
        return Err(SingleTurnError::AmbiguousInitialBranch { raw_rev });
    }

    let resolved_rev = (raw + first_turn as f64) as f32;
    if let Some(reference) = startup_reference {
        let reference = reference.validate()?;
        if (resolved_rev - reference.position_rev).abs() > reference.max_error_rev {
            return Err(SingleTurnError::StartupReferenceMismatch {
                resolved_rev,
                reference_rev: reference.position_rev,
                max_error_rev: reference.max_error_rev,
            });
        }
    }
    Ok(resolved_rev)
}

/// Stateful unwrapping after the initial branch has been established.
///
/// Any bad sample permanently faults this instance.  The owner must latch the
/// corresponding controller fault and construct a new instance only as part
/// of an explicit recovery/reinitialization sequence.
#[derive(Debug, Clone)]
pub struct SingleTurnUnwrapper {
    window: BranchWindow,
    last_raw_rev: f32,
    position_rev: f32,
    faulted: bool,
}

impl SingleTurnUnwrapper {
    pub fn initialize(
        raw_rev: f32,
        window: BranchWindow,
        startup_reference: Option<StartupReference>,
    ) -> Result<Self, SingleTurnError> {
        let window = window.validate()?;
        let position_rev = resolve_initial_branch(raw_rev, window, startup_reference)?;
        Ok(Self {
            window,
            last_raw_rev: raw_rev,
            position_rev,
            faulted: false,
        })
    }

    pub fn position_rev(&self) -> f32 {
        self.position_rev
    }

    pub fn is_faulted(&self) -> bool {
        self.faulted
    }

    pub fn update(&mut self, raw_rev: f32, max_step_rev: f32) -> Result<f32, SingleTurnError> {
        if self.faulted {
            return Err(SingleTurnError::Faulted);
        }
        let result = self.update_inner(raw_rev, max_step_rev);
        if result.is_err() {
            self.faulted = true;
        }
        result
    }

    fn update_inner(&mut self, raw_rev: f32, max_step_rev: f32) -> Result<f32, SingleTurnError> {
        validate_raw(raw_rev)?;
        if !max_step_rev.is_finite() || max_step_rev <= 0.0 || max_step_rev >= 0.5 {
            return Err(SingleTurnError::InvalidMaxStep);
        }

        let raw_delta = raw_rev - self.last_raw_rev;
        if (raw_delta.abs() - 0.5).abs() <= HALF_TURN_AMBIGUITY_EPS_REV {
            return Err(SingleTurnError::AmbiguousHalfTurnStep {
                delta_rev: raw_delta,
            });
        }
        let delta_rev = if raw_delta > 0.5 {
            raw_delta - 1.0
        } else if raw_delta < -0.5 {
            raw_delta + 1.0
        } else {
            raw_delta
        };
        if delta_rev.abs() > max_step_rev {
            return Err(SingleTurnError::StepTooLarge {
                delta_rev,
                max_step_rev,
            });
        }

        let next = self.position_rev + delta_rev;
        let lower = self.window.lower_rev - self.window.tolerance_rev;
        let upper = self.window.upper_rev + self.window.tolerance_rev;
        if next < lower || next > upper {
            return Err(SingleTurnError::PositionOutsideWindow { position_rev: next });
        }

        self.last_raw_rev = raw_rev;
        self.position_rev = next;
        Ok(next)
    }
}

/// Reject a compressed-MIT command mapping that relies on undocumented
/// multi-turn target semantics or reaches the periodic seam.
///
/// Until the drive's position-error behavior at the seam is verified, a real
/// hardware profile should use a commissioning window that lies wholly inside
/// this canonical range.  A positive `seam_guard_rev` leaves room for
/// sampling/command skew near the wrap.
pub fn validate_single_turn_command_window(
    lower_rev: f32,
    upper_rev: f32,
    seam_guard_rev: f32,
) -> Result<(), SingleTurnError> {
    if !lower_rev.is_finite()
        || !upper_rev.is_finite()
        || !seam_guard_rev.is_finite()
        || lower_rev >= upper_rev
        || !(0.0..0.5).contains(&seam_guard_rev)
    {
        return Err(SingleTurnError::InvalidCommandWindow);
    }
    let safe_lower = SINGLE_TURN_MIN_REV + seam_guard_rev;
    let safe_upper = SINGLE_TURN_MAX_REV - seam_guard_rev;
    if lower_rev < safe_lower || upper_rev >= safe_upper {
        return Err(SingleTurnError::CommandWindowTouchesWrapSeam);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    fn window(lower_rev: f32, upper_rev: f32) -> BranchWindow {
        BranchWindow {
            lower_rev,
            upper_rev,
            tolerance_rev: 1.0e-4,
        }
    }

    #[test]
    fn unwraps_forward_and_reverse_across_boundary() {
        let mut forward = SingleTurnUnwrapper::initialize(0.49, window(0.4, 0.7), None).unwrap();
        assert!((forward.update(-0.49, 0.03).unwrap() - 0.51).abs() < 1.0e-6);

        let mut reverse = SingleTurnUnwrapper::initialize(-0.49, window(-0.7, -0.4), None).unwrap();
        assert!((reverse.update(0.49, 0.03).unwrap() + 0.51).abs() < 1.0e-6);
    }

    #[test]
    fn restart_recovers_noncanonical_continuous_branch_from_limits() {
        let resolved = resolve_initial_branch(-0.49, window(0.4, 0.7), None).unwrap();
        assert!((resolved - 0.51).abs() < 1.0e-6);
    }

    #[test]
    fn restart_reference_mismatch_fails_closed() {
        let error = resolve_initial_branch(
            -0.49,
            window(0.4, 0.7),
            Some(StartupReference {
                position_rev: 0.42,
                max_error_rev: 0.02,
            }),
        )
        .unwrap_err();
        assert!(matches!(
            error,
            SingleTurnError::StartupReferenceMismatch { .. }
        ));
    }

    #[test]
    fn one_turn_branch_window_is_rejected_as_ambiguous() {
        let error = resolve_initial_branch(0.1, window(0.0, 1.0), None).unwrap_err();
        assert_eq!(error, SingleTurnError::AmbiguousBranchWindow);
    }

    #[test]
    fn missing_initial_branch_fails_closed() {
        let error = resolve_initial_branch(0.3, window(0.1, 0.2), None).unwrap_err();
        assert!(matches!(error, SingleTurnError::NoBranchInWindow { .. }));
    }

    #[test]
    fn exact_half_turn_step_is_ambiguous_and_latches_fault() {
        let mut unwrap = SingleTurnUnwrapper::initialize(0.0, window(-0.4, 0.4), None).unwrap();
        let error = unwrap.update(-0.5, 0.49).unwrap_err();
        assert!(matches!(
            error,
            SingleTurnError::AmbiguousHalfTurnStep { .. }
        ));
        assert!(unwrap.is_faulted());
        assert_eq!(
            unwrap.update(0.0, 0.1).unwrap_err(),
            SingleTurnError::Faulted
        );
    }

    #[test]
    fn oversized_step_latches_fault_without_advancing_state() {
        let mut unwrap = SingleTurnUnwrapper::initialize(0.0, window(-0.4, 0.4), None).unwrap();
        let error = unwrap.update(0.02, 0.01).unwrap_err();
        assert!(matches!(error, SingleTurnError::StepTooLarge { .. }));
        assert_eq!(unwrap.position_rev(), 0.0);
        assert!(unwrap.is_faulted());
    }

    #[test]
    fn canonical_upper_bound_is_exclusive() {
        let error = resolve_initial_branch(0.5, window(-0.4, 0.4), None).unwrap_err();
        assert!(matches!(
            error,
            SingleTurnError::NonCanonicalRawPosition { .. }
        ));
    }

    #[test]
    fn per_axis_surveyed_candidates_resolve_to_expected_ros_angles() {
        let raw = [
            -0.004_082_441_3,
            0.251_499_35,
            0.253_775_12,
            -0.000_022_888_18,
            -0.004_293_829,
            -0.019_331_366,
        ];
        let direction = [-1.0, -1.0, 1.0, 1.0, 1.0, 1.0];
        let offset = [
            -0.025_651, 0.010_217, -0.024_516, 0.000_144, 0.026_979, 0.121_463,
        ];
        let ros_limits = [
            (-2.86, 2.86),
            (-1.57, 2.09),
            (-1.57, 1.57),
            (-1.57, 1.57),
            (-1.54, 1.54),
            (-2.79, 2.79),
        ];
        let expected = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0];

        for i in 0..6 {
            let motor_a = direction[i] * (ros_limits[i].0 - offset[i]) / TAU;
            let motor_b = direction[i] * (ros_limits[i].1 - offset[i]) / TAU;
            let resolved = resolve_initial_branch(
                raw[i],
                BranchWindow {
                    lower_rev: motor_a.min(motor_b),
                    upper_rev: motor_a.max(motor_b),
                    tolerance_rev: 0.001,
                },
                Some(StartupReference {
                    position_rev: direction[i] * (expected[i] - offset[i]) / TAU,
                    max_error_rev: 0.01,
                }),
            )
            .unwrap();
            let q = direction[i] * TAU * resolved + offset[i];
            assert!((q - expected[i]).abs() < 1.0e-4, "joint {} q={q}", i + 1);
        }
    }

    #[test]
    fn corrected_joint_2_full_range_avoids_the_single_turn_seam() {
        // With q2=-1.570 at the current raw~0.251499, the full URDF range maps inside one
        // canonical turn instead of crossing the seam as the old +1.570 fit did.
        assert!(validate_single_turn_command_window(-0.331_008, 0.251_499, 0.01).is_ok());

        let old_error =
            validate_single_turn_command_window(0.168_734, 0.751_241, 0.01).unwrap_err();
        assert_eq!(old_error, SingleTurnError::CommandWindowTouchesWrapSeam);
    }
}
