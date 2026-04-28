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

    pub fn is_armed(self) -> bool {
        self as u8 >= Self::Armed as u8
    }
    pub fn is_running(self) -> bool {
        self == Self::OldRoutine || self == Self::Running
    }
    pub fn is_old_routine(self) -> bool {
        self == Self::OldRoutine
    }
    pub fn is_stepper_sine(self) -> bool {
        self == Self::StepperSine
    }
}

/// Motor state transition events.
/// Centralizes all valid state changes — call `MotorMode::transition()` instead
/// of imperatively setting individual flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorEvent {
    /// Input detected, zero throttle confirmed → Disarmed→Armed
    Arm,
    /// Safety disarm (timeout, error, LVC) → Any→Disarmed
    Disarm,
    /// Throttle applied → Armed→OldRoutine
    StartMotor,
    /// Throttle zero or desync → Running/OldRoutine→Armed
    StopMotor,
    /// Enter sinusoidal startup → Armed→StepperSine
    EnterSine,
    /// Sine changeover → StepperSine→OldRoutine
    ExitSine,
    /// Enough zero-crosses → OldRoutine→Running
    BemfLocked,
    /// Desync fallback → Running→OldRoutine
    DesyncFallback,
}

impl MotorMode {
    /// Apply a state transition event. Returns the new mode.
    /// Invalid transitions are silently ignored (returns self unchanged).
    pub fn transition(self, event: MotorEvent) -> Self {
        match (self, event) {
            (Self::Disarmed, MotorEvent::Arm) => Self::Armed,
            (_, MotorEvent::Disarm) => Self::Disarmed,
            (Self::Armed, MotorEvent::StartMotor) => Self::OldRoutine,
            (Self::OldRoutine | Self::Running, MotorEvent::StopMotor) => Self::Armed,
            (Self::Armed, MotorEvent::EnterSine) => Self::StepperSine,
            (Self::StepperSine, MotorEvent::ExitSine) => Self::OldRoutine,
            (Self::OldRoutine, MotorEvent::BemfLocked) => Self::Running,
            (Self::Running, MotorEvent::DesyncFallback) => Self::OldRoutine,
            _ => self, // invalid transition — no change
        }
    }
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

    // --- Transition tests ---

    #[test]
    fn transition_arm_from_disarmed() {
        assert_eq!(
            MotorMode::Disarmed.transition(MotorEvent::Arm),
            MotorMode::Armed
        );
    }

    #[test]
    fn transition_arm_ignored_when_armed() {
        assert_eq!(
            MotorMode::Armed.transition(MotorEvent::Arm),
            MotorMode::Armed
        );
    }

    #[test]
    fn transition_disarm_from_any() {
        assert_eq!(
            MotorMode::Running.transition(MotorEvent::Disarm),
            MotorMode::Disarmed
        );
        assert_eq!(
            MotorMode::Armed.transition(MotorEvent::Disarm),
            MotorMode::Disarmed
        );
        assert_eq!(
            MotorMode::StepperSine.transition(MotorEvent::Disarm),
            MotorMode::Disarmed
        );
    }

    #[test]
    fn transition_start_motor() {
        assert_eq!(
            MotorMode::Armed.transition(MotorEvent::StartMotor),
            MotorMode::OldRoutine
        );
        // Can't start from Disarmed
        assert_eq!(
            MotorMode::Disarmed.transition(MotorEvent::StartMotor),
            MotorMode::Disarmed
        );
    }

    #[test]
    fn transition_stop_motor() {
        assert_eq!(
            MotorMode::Running.transition(MotorEvent::StopMotor),
            MotorMode::Armed
        );
        assert_eq!(
            MotorMode::OldRoutine.transition(MotorEvent::StopMotor),
            MotorMode::Armed
        );
    }

    #[test]
    fn transition_bemf_locked() {
        assert_eq!(
            MotorMode::OldRoutine.transition(MotorEvent::BemfLocked),
            MotorMode::Running
        );
        // Not from Running
        assert_eq!(
            MotorMode::Running.transition(MotorEvent::BemfLocked),
            MotorMode::Running
        );
    }

    #[test]
    fn transition_sine_flow() {
        let m = MotorMode::Armed.transition(MotorEvent::EnterSine);
        assert_eq!(m, MotorMode::StepperSine);
        let m = m.transition(MotorEvent::ExitSine);
        assert_eq!(m, MotorMode::OldRoutine);
    }

    #[test]
    fn transition_desync_fallback() {
        assert_eq!(
            MotorMode::Running.transition(MotorEvent::DesyncFallback),
            MotorMode::OldRoutine
        );
    }

    #[test]
    fn transition_full_lifecycle() {
        let m = MotorMode::Disarmed;
        let m = m.transition(MotorEvent::Arm);
        assert_eq!(m, MotorMode::Armed);
        let m = m.transition(MotorEvent::StartMotor);
        assert_eq!(m, MotorMode::OldRoutine);
        let m = m.transition(MotorEvent::BemfLocked);
        assert_eq!(m, MotorMode::Running);
        let m = m.transition(MotorEvent::StopMotor);
        assert_eq!(m, MotorMode::Armed);
        let m = m.transition(MotorEvent::Disarm);
        assert_eq!(m, MotorMode::Disarmed);
    }
}
