//! SharedComm test implementation using Cell for interior mutability.

use crate::motor_mode::MotorMode;
use crate::shared_comm::{IsrTiming, MainControl, MotorState, SharedComm};
use core::cell::Cell;

/// Test-friendly SharedComm that uses Cell for interior mutability.
pub struct TestShared {
    pub mode: Cell<MotorMode>,
    pub input_set: Cell<bool>,
    pub dshot_telemetry: Cell<bool>,
    pub is_dshot: Cell<bool>,
    pub newinput: Cell<u16>,
    pub adjusted_input: Cell<u16>,
    pub duty_cycle_setpoint: Cell<u16>,
    pub duty_cycle: Cell<u16>,
    pub forward: Cell<bool>,
    pub zero_crosses: Cell<u32>,
    pub commutation_interval: Cell<u32>,
    pub e_com_time: Cell<i32>,
    pub signal_timeout: Cell<u16>,
    pub send_telemetry: Cell<bool>,
    // Main→ISR published state
    pub tim1_arr: Cell<u16>,
    pub duty_maximum: Cell<u16>,
    pub filter_level: Cell<u8>,
    pub min_bemf_counts: Cell<u8>,
    pub auto_advance: Cell<u8>,
    pub interval_timer_count: Cell<u32>,
    pub prop_brake_active: Cell<bool>,
}

impl Default for TestShared {
    fn default() -> Self {
        Self::new()
    }
}

impl TestShared {
    pub fn new() -> Self {
        Self {
            mode: Cell::new(MotorMode::Disarmed),
            input_set: Cell::new(false),
            dshot_telemetry: Cell::new(false),
            is_dshot: Cell::new(false),
            newinput: Cell::new(0),
            adjusted_input: Cell::new(0),
            duty_cycle_setpoint: Cell::new(0),
            duty_cycle: Cell::new(0),
            forward: Cell::new(true),
            zero_crosses: Cell::new(0),
            commutation_interval: Cell::new(12500),
            e_com_time: Cell::new(0),
            signal_timeout: Cell::new(0),
            send_telemetry: Cell::new(false),
            tim1_arr: Cell::new(1999),
            duty_maximum: Cell::new(2000),
            filter_level: Cell::new(5),
            min_bemf_counts: Cell::new(2),
            auto_advance: Cell::new(0),
            interval_timer_count: Cell::new(0),
            prop_brake_active: Cell::new(false),
        }
    }
}

impl MotorState for TestShared {
    fn motor_mode(&self) -> MotorMode {
        self.mode.get()
    }
    fn set_motor_mode(&self, mode: MotorMode) {
        self.mode.set(mode);
    }
}

impl IsrTiming for TestShared {
    fn zero_crosses(&self) -> u32 {
        self.zero_crosses.get()
    }
    fn set_zero_crosses(&self, v: u32) {
        self.zero_crosses.set(v);
    }
    fn increment_zero_crosses(&self) {
        let v = self.zero_crosses.get();
        if v < 10000 {
            self.zero_crosses.set(v + 1);
        }
    }
    fn commutation_interval(&self) -> u32 {
        self.commutation_interval.get()
    }
    fn set_commutation_interval(&self, v: u32) {
        self.commutation_interval.set(v);
    }
    fn e_com_time(&self) -> i32 {
        self.e_com_time.get()
    }
    fn set_e_com_time(&self, v: i32) {
        self.e_com_time.set(v);
    }
    fn interval_timer_count(&self) -> u32 {
        self.interval_timer_count.get()
    }
    fn set_interval_timer_count(&self, v: u32) {
        self.interval_timer_count.set(v);
    }
    fn signal_timeout(&self) -> u16 {
        self.signal_timeout.get()
    }
    fn increment_signal_timeout(&self) {
        let v = self.signal_timeout.get();
        if v < u16::MAX {
            self.signal_timeout.set(v + 1);
        }
    }
    fn duty_cycle(&self) -> u16 {
        self.duty_cycle.get()
    }
    fn set_duty_cycle(&self, v: u16) {
        self.duty_cycle.set(v);
    }
    fn forward(&self) -> bool {
        self.forward.get()
    }
    fn set_forward(&self, v: bool) {
        self.forward.set(v);
    }
}

impl MainControl for TestShared {
    fn adjusted_input(&self) -> u16 {
        self.adjusted_input.get()
    }
    fn set_adjusted_input(&self, v: u16) {
        self.adjusted_input.set(v);
    }
    fn duty_cycle_setpoint(&self) -> u16 {
        self.duty_cycle_setpoint.get()
    }
    fn set_duty_cycle_setpoint(&self, v: u16) {
        self.duty_cycle_setpoint.set(v);
    }
    fn tim1_arr(&self) -> u16 {
        self.tim1_arr.get()
    }
    fn set_tim1_arr(&self, v: u16) {
        self.tim1_arr.set(v);
    }
    fn duty_maximum(&self) -> u16 {
        self.duty_maximum.get()
    }
    fn set_duty_maximum(&self, v: u16) {
        self.duty_maximum.set(v);
    }
    fn filter_level(&self) -> u8 {
        self.filter_level.get()
    }
    fn set_filter_level(&self, v: u8) {
        self.filter_level.set(v);
    }
    fn min_bemf_counts(&self) -> u8 {
        self.min_bemf_counts.get()
    }
    fn set_min_bemf_counts(&self, v: u8) {
        self.min_bemf_counts.set(v);
    }
    fn auto_advance(&self) -> u8 {
        self.auto_advance.get()
    }
    fn set_auto_advance(&self, v: u8) {
        self.auto_advance.set(v);
    }
    fn prop_brake_active(&self) -> bool {
        self.prop_brake_active.get()
    }
    fn set_prop_brake_active(&self, v: bool) {
        self.prop_brake_active.set(v);
    }
}

impl SharedComm for TestShared {
    fn input_set(&self) -> bool {
        self.input_set.get()
    }
    fn set_input_set(&self, v: bool) {
        self.input_set.set(v);
    }
    fn dshot_telemetry(&self) -> bool {
        self.dshot_telemetry.get()
    }
    fn is_dshot(&self) -> bool {
        self.is_dshot.get()
    }
    fn set_is_dshot(&self, v: bool) {
        self.is_dshot.set(v);
    }
    fn newinput(&self) -> u16 {
        self.newinput.get()
    }
    fn set_newinput(&self, v: u16) {
        self.newinput.set(v);
    }
    fn send_telemetry(&self) -> bool {
        self.send_telemetry.get()
    }
    fn set_send_telemetry(&self, v: bool) {
        self.send_telemetry.set(v);
    }
}
