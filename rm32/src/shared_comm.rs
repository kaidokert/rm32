//! SharedComm trait — abstraction over ISR↔main shared state.
//!
//! On real hardware, this is implemented via atomics (SharedState).
//! For testing, TestShared implements it with Cell fields.

use crate::motor_mode::{MotorEvent, MotorMode};

/// Communication interface between ISR and main loop contexts.
/// Provides access to shared motor state that both contexts need.
pub trait SharedComm {
    // Motor mode (replaces armed/running/old_routine/stepper_sine bools)
    fn motor_mode(&self) -> MotorMode;
    fn set_motor_mode(&self, mode: MotorMode);

    /// Apply a state transition event atomically.
    fn transition(&self, event: MotorEvent) {
        let new = self.motor_mode().transition(event);
        if new != self.motor_mode() {
            self.set_motor_mode(new);
        }
    }

    // Convenience accessors derived from motor_mode
    fn armed(&self) -> bool {
        self.motor_mode().is_armed()
    }
    fn running(&self) -> bool {
        self.motor_mode().is_running()
    }
    fn old_routine(&self) -> bool {
        self.motor_mode().is_old_routine()
    }
    fn stepper_sine(&self) -> bool {
        self.motor_mode().is_stepper_sine()
    }

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

    /// Whether detected input is DShot (vs servo). ISR transfer handler sets this.
    fn is_dshot(&self) -> bool {
        false
    }
    fn set_is_dshot(&self, _v: bool) {}

    fn newinput(&self) -> u16;
    fn set_newinput(&self, v: u16);
    fn adjusted_input(&self) -> u16;
    fn set_adjusted_input(&self, v: u16);
    fn duty_cycle_setpoint(&self) -> u16;
    fn set_duty_cycle_setpoint(&self, v: u16);

    /// Current duty cycle (ISR writes each tick, main reads for bidir speed gate).
    fn duty_cycle(&self) -> u16 {
        0
    }
    fn set_duty_cycle(&self, _v: u16) {}

    /// Motor direction (ISR reads, main writes on bidir direction change).
    fn forward(&self) -> bool {
        true
    }
    fn set_forward(&self, _v: bool) {}

    fn zero_crosses(&self) -> u32;
    fn set_zero_crosses(&self, v: u32);
    fn increment_zero_crosses(&self);
    fn commutation_interval(&self) -> u32;
    fn set_commutation_interval(&self, v: u32);
    fn e_com_time(&self) -> i32;

    fn signal_timeout(&self) -> u16;
    fn increment_signal_timeout(&self);

    fn stall_protection_adjust(&self) -> u16 {
        0
    }
    fn set_stall_protection_adjust(&self, _v: u16) {}

    fn battery_voltage(&self) -> u16 {
        0
    }

    // --- Main→ISR published state (main computes, ISR reads) ---

    fn send_telemetry(&self) -> bool;
    fn set_send_telemetry(&self, v: bool);

    fn save_settings_flag(&self) -> bool {
        false
    }
    fn set_save_settings_flag(&self, _v: bool) {}
    fn send_esc_info_flag(&self) -> bool {
        false
    }
    fn set_send_esc_info_flag(&self, _v: bool) {}

    /// TIM1 auto-reload value (variable PWM). Main publishes, ISR applies.
    fn tim1_arr(&self) -> u16 {
        1999
    }
    fn set_tim1_arr(&self, _v: u16) {}

    /// Max duty cycle (eRPM/temperature limiting). Main publishes, ISR applies.
    fn duty_maximum(&self) -> u16 {
        2000
    }
    fn set_duty_maximum(&self, _v: u16) {}

    /// BEMF filter level. Main computes based on motor speed, ISR uses for ZC detection.
    fn filter_level(&self) -> u8 {
        5
    }
    fn set_filter_level(&self, _v: u8) {}

    /// Min BEMF counts for zero-cross acceptance. Main adjusts during startup.
    fn min_bemf_counts(&self) -> u8 {
        2
    }
    fn set_min_bemf_counts(&self, _v: u8) {}

    /// Auto advance level. Main computes from duty cycle, ISR uses for timing.
    fn auto_advance(&self) -> u8 {
        0
    }
    fn set_auto_advance(&self, _v: u8) {}

    // --- ISR→Main published state ---

    /// Interval timer count (ISR publishes, main reads for stall detection).
    /// When this exceeds ~45000 (22.5ms at 2MHz), no BEMF zero-cross has occurred.
    fn interval_timer_count(&self) -> u32 {
        0
    }
    fn set_interval_timer_count(&self, _v: u32) {}

    fn set_actual_current(&self, _v: i16) {}
    fn set_battery_voltage(&self, _v: u16) {}
    fn set_degrees_celsius(&self, _v: i16) {}
    fn set_e_com_time(&self, _v: i32) {}
}
