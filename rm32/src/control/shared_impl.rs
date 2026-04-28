//! SharedComm test implementation using Cell for interior mutability.

use crate::motor_mode::MotorMode;
use crate::shared_comm::SharedComm;
use core::cell::Cell;

/// Test-friendly SharedComm that uses Cell for interior mutability.
pub struct TestShared {
    pub mode: Cell<MotorMode>,
    pub input_set: Cell<bool>,
    pub dshot_telemetry: Cell<bool>,
    pub newinput: Cell<u16>,
    pub adjusted_input: Cell<u16>,
    pub duty_cycle_setpoint: Cell<u16>,
    pub zero_crosses: Cell<u32>,
    pub commutation_interval: Cell<u32>,
    pub e_com_time: Cell<i32>,
    pub signal_timeout: Cell<u16>,
    pub send_telemetry: Cell<bool>,
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
            newinput: Cell::new(0),
            adjusted_input: Cell::new(0),
            duty_cycle_setpoint: Cell::new(0),
            zero_crosses: Cell::new(0),
            commutation_interval: Cell::new(12500),
            e_com_time: Cell::new(0),
            signal_timeout: Cell::new(0),
            send_telemetry: Cell::new(false),
        }
    }
}

impl SharedComm for TestShared {
    fn motor_mode(&self) -> MotorMode {
        self.mode.get()
    }
    fn set_motor_mode(&self, mode: MotorMode) {
        self.mode.set(mode);
    }

    fn input_set(&self) -> bool {
        self.input_set.get()
    }
    fn set_input_set(&self, v: bool) {
        self.input_set.set(v);
    }
    fn dshot_telemetry(&self) -> bool {
        self.dshot_telemetry.get()
    }

    fn newinput(&self) -> u16 {
        self.newinput.get()
    }
    fn set_newinput(&self, v: u16) {
        self.newinput.set(v);
    }
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

    fn signal_timeout(&self) -> u16 {
        self.signal_timeout.get()
    }
    fn increment_signal_timeout(&self) {
        let v = self.signal_timeout.get();
        if v < u16::MAX {
            self.signal_timeout.set(v + 1);
        }
    }

    fn send_telemetry(&self) -> bool {
        self.send_telemetry.get()
    }
    fn set_send_telemetry(&self, v: bool) {
        self.send_telemetry.set(v);
    }
}
