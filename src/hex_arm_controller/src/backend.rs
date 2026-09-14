//! Compatibility imports. New code uses the motor module.
#[cfg(feature = "legacy")]
pub(crate) use crate::motor::legacy::*;
pub use crate::motor::*;
