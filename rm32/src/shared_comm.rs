//! SharedComm trait — abstraction over ISR↔main shared state.
//!
//! On real hardware, this is implemented via atomics (SharedState).
//! For testing, MotorState implements it with direct field access.

/// Communication interface between ISR and main loop contexts.
/// Provides access to shared motor state that both contexts need.
pub trait SharedComm {
    fn armed(&self) -> bool;
    fn set_armed(&self, v: bool);
    fn running(&self) -> bool;
    fn set_running(&self, v: bool);
    fn input_set(&self) -> bool;
    fn set_input_set(&self, v: bool);
    fn stepper_sine(&self) -> bool;
    fn set_stepper_sine(&self, v: bool);
    fn old_routine(&self) -> bool;
    fn set_old_routine(&self, v: bool);
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
