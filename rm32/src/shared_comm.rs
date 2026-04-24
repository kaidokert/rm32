//! SharedComm trait — abstraction over ISR↔main shared state.
//!
//! On real hardware, this is implemented via atomics (SharedState).
//! For testing, TestShared implements it with Cell fields.

use crate::motor_mode::MotorMode;

/// Communication interface between ISR and main loop contexts.
/// Provides access to shared motor state that both contexts need.
pub trait SharedComm {
    // Motor mode (replaces armed/running/old_routine/stepper_sine bools)
    fn motor_mode(&self) -> MotorMode;
    fn set_motor_mode(&self, mode: MotorMode);

    // Convenience accessors derived from motor_mode
    fn armed(&self) -> bool { self.motor_mode().is_armed() }
    fn running(&self) -> bool { self.motor_mode().is_running() }
    fn old_routine(&self) -> bool { self.motor_mode().is_old_routine() }
    fn stepper_sine(&self) -> bool { self.motor_mode().is_stepper_sine() }

    // Convenience setters that translate to mode transitions
    fn set_armed(&self, v: bool) {
        if v && !self.armed() {
            self.set_motor_mode(MotorMode::Armed);
        } else if !v {
            self.set_motor_mode(MotorMode::Disarmed);
        }
    }
    fn set_running(&self, v: bool) {
        if v && !self.running() {
            self.set_motor_mode(MotorMode::OldRoutine);
        } else if !v && self.running() {
            self.set_motor_mode(MotorMode::Armed);
        }
    }
    fn set_old_routine(&self, v: bool) {
        if v && self.running() {
            self.set_motor_mode(MotorMode::OldRoutine);
        } else if !v && self.old_routine() {
            self.set_motor_mode(MotorMode::Running);
        }
    }
    fn set_stepper_sine(&self, v: bool) {
        if v {
            self.set_motor_mode(MotorMode::StepperSine);
        } else if self.stepper_sine() {
            self.set_motor_mode(MotorMode::Armed);
        }
    }

    fn input_set(&self) -> bool;
    fn set_input_set(&self, v: bool);
    fn dshot_telemetry(&self) -> bool;

    fn newinput(&self) -> u16;
    fn set_newinput(&self, v: u16);
    fn adjusted_input(&self) -> u16;
    fn set_adjusted_input(&self, v: u16);
    fn duty_cycle_setpoint(&self) -> u16;
    fn set_duty_cycle_setpoint(&self, v: u16);

    fn zero_crosses(&self) -> u32;
    fn set_zero_crosses(&self, v: u32);
    fn increment_zero_crosses(&self);
    fn commutation_interval(&self) -> u32;
    fn set_commutation_interval(&self, v: u32);
    fn e_com_time(&self) -> i32;

    fn signal_timeout(&self) -> u16;
    fn increment_signal_timeout(&self);
}
