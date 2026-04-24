//! Motor state machine — replaces separate armed/running/old_routine/stepper_sine bools.
//!
//! Valid states form a linear progression:
//!   Disarmed → Armed → StepperSine → OldRoutine → Running
//!
//! Encoding as a single u8 allows atomic storage in SharedState.

/// Motor operating mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MotorMode {
    /// Not armed. Waiting for input signal detection + zero throttle timeout.
    Disarmed = 0,
    /// Armed, not running. Ready to accept throttle.
    Armed = 1,
    /// Sinusoidal startup (open-loop phase stepping).
    StepperSine = 2,
    /// BEMF polling mode (old_routine). Transitional startup phase.
    OldRoutine = 3,
    /// Normal running — interrupt-driven BEMF zero-cross detection.
    Running = 4,
}

impl MotorMode {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Disarmed,
            1 => Self::Armed,
            2 => Self::StepperSine,
            3 => Self::OldRoutine,
            4 => Self::Running,
            _ => Self::Disarmed,
        }
    }

    pub fn is_armed(self) -> bool { self as u8 >= Self::Armed as u8 }
    pub fn is_running(self) -> bool { self == Self::OldRoutine || self == Self::Running }
    pub fn is_old_routine(self) -> bool { self == Self::OldRoutine }
    pub fn is_stepper_sine(self) -> bool { self == Self::StepperSine }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_armed_flags() {
        assert!(!MotorMode::Disarmed.is_armed());
        assert!(MotorMode::Armed.is_armed());
        assert!(MotorMode::StepperSine.is_armed());
        assert!(MotorMode::OldRoutine.is_armed());
        assert!(MotorMode::Running.is_armed());
    }

    #[test]
    fn mode_running_flags() {
        assert!(!MotorMode::Disarmed.is_running());
        assert!(!MotorMode::Armed.is_running());
        assert!(!MotorMode::StepperSine.is_running());
        assert!(MotorMode::OldRoutine.is_running());
        assert!(MotorMode::Running.is_running());
    }

    #[test]
    fn roundtrip() {
        for v in 0..=4u8 {
            let mode = MotorMode::from_u8(v);
            assert_eq!(mode as u8, v);
        }
    }

    #[test]
    fn invalid_defaults_to_disarmed() {
        assert_eq!(MotorMode::from_u8(255), MotorMode::Disarmed);
    }
}
