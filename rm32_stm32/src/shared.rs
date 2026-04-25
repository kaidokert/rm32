//! Shared state between ISR and main loop — all atomic, lock-free.
//!
//! On Cortex-M0+ (single core, aligned access), atomic loads/stores
//! compile to plain LDR/STR which are inherently atomic.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};
use rm32::motor_mode::MotorMode;

/// Relaxed ordering — sufficient for single-core Cortex-M0+.
/// No reordering concerns without a cache or second core.
const ORD: Ordering = Ordering::Relaxed;

/// Shared state accessed by both ISR and main loop contexts.
/// All fields are atomic — no locks or critical sections needed.
#[repr(C)]
pub struct SharedState {
    // Motor state machine (single atomic replaces armed/running/old_routine/stepper_sine)
    motor_mode: AtomicU8,
    input_set: AtomicBool,
    send_telemetry: AtomicBool,
    dshot: AtomicBool,
    servo_pwm: AtomicBool,
    dshot_telemetry: AtomicBool,
    save_settings_flag: AtomicBool,
    send_esc_info_flag: AtomicBool,

    // Timing (ISR writes, main reads)
    zero_crosses: AtomicU32,
    commutation_interval: AtomicU32,

    // Input (DMA ISR writes, main/tenKhz reads)
    newinput: AtomicU16,
    adjusted_input: AtomicU16,

    // Control (main writes setpoint, ISR reads)
    duty_cycle_setpoint: AtomicU16,
    signal_timeout: AtomicU16,
    zero_input_count: AtomicU16,

    // Telemetry (main computes, ISR reads for speed PID)
    e_com_time: AtomicU32, // stored as u32, interpreted as i32

    // Stall protection (main computes, ISR applies to duty)
    stall_protection_adjust: AtomicU16,

    // Measurements (main writes, ISR reads for EDT)
    actual_current: AtomicU16, // mA, stored as u16
    battery_voltage: AtomicU16, // mV
    degrees_celsius: AtomicU16, // stored as u16, interpreted as i16
}

impl SharedState {
    pub const fn new() -> Self {
        Self {
            motor_mode: AtomicU8::new(MotorMode::Disarmed as u8),
            input_set: AtomicBool::new(false),
            send_telemetry: AtomicBool::new(false),
            dshot: AtomicBool::new(false),
            servo_pwm: AtomicBool::new(false),
            dshot_telemetry: AtomicBool::new(false),
            save_settings_flag: AtomicBool::new(false),
            send_esc_info_flag: AtomicBool::new(false),
            zero_crosses: AtomicU32::new(0),
            commutation_interval: AtomicU32::new(12500),
            newinput: AtomicU16::new(0),
            adjusted_input: AtomicU16::new(0),
            duty_cycle_setpoint: AtomicU16::new(0),
            signal_timeout: AtomicU16::new(0),
            zero_input_count: AtomicU16::new(0),
            e_com_time: AtomicU32::new(0),
            stall_protection_adjust: AtomicU16::new(0),
            actual_current: AtomicU16::new(0),
            battery_voltage: AtomicU16::new(0),
            degrees_celsius: AtomicU16::new(0),
        }
    }

    // --- Motor mode ---

    pub fn motor_mode(&self) -> MotorMode { MotorMode::from_u8(self.motor_mode.load(ORD)) }
    pub fn set_motor_mode(&self, mode: MotorMode) { self.motor_mode.store(mode as u8, ORD); }

    // Convenience getters (delegate to motor_mode)
    pub fn armed(&self) -> bool { self.motor_mode().is_armed() }
    pub fn running(&self) -> bool { self.motor_mode().is_running() }
    pub fn old_routine(&self) -> bool { self.motor_mode().is_old_routine() }
    pub fn stepper_sine(&self) -> bool { self.motor_mode().is_stepper_sine() }

    // Convenience setters (mode transitions)
    pub fn set_armed(&self, v: bool) {
        if v && !self.armed() { self.set_motor_mode(MotorMode::Armed); }
        else if !v { self.set_motor_mode(MotorMode::Disarmed); }
    }
    pub fn set_running(&self, v: bool) {
        if v && !self.running() { self.set_motor_mode(MotorMode::OldRoutine); }
        else if !v && self.running() { self.set_motor_mode(MotorMode::Armed); }
    }
    pub fn set_old_routine(&self, v: bool) {
        if v && self.running() { self.set_motor_mode(MotorMode::OldRoutine); }
        else if !v && self.old_routine() { self.set_motor_mode(MotorMode::Running); }
    }
    pub fn set_stepper_sine(&self, v: bool) {
        if v { self.set_motor_mode(MotorMode::StepperSine); }
        else if self.stepper_sine() { self.set_motor_mode(MotorMode::Armed); }
    }

    // --- Bool accessors ---

    pub fn input_set(&self) -> bool { self.input_set.load(ORD) }
    pub fn set_input_set(&self, v: bool) { self.input_set.store(v, ORD); }

    pub fn send_telemetry(&self) -> bool { self.send_telemetry.load(ORD) }
    pub fn set_send_telemetry(&self, v: bool) { self.send_telemetry.store(v, ORD); }

    pub fn dshot(&self) -> bool { self.dshot.load(ORD) }
    pub fn set_dshot(&self, v: bool) { self.dshot.store(v, ORD); }

    pub fn servo_pwm(&self) -> bool { self.servo_pwm.load(ORD) }
    pub fn set_servo_pwm(&self, v: bool) { self.servo_pwm.store(v, ORD); }

    pub fn dshot_telemetry(&self) -> bool { self.dshot_telemetry.load(ORD) }
    pub fn set_dshot_telemetry(&self, v: bool) { self.dshot_telemetry.store(v, ORD); }

    pub fn save_settings_flag(&self) -> bool { self.save_settings_flag.load(ORD) }
    pub fn set_save_settings_flag(&self, v: bool) { self.save_settings_flag.store(v, ORD); }

    pub fn send_esc_info_flag(&self) -> bool { self.send_esc_info_flag.load(ORD) }
    pub fn set_send_esc_info_flag(&self, v: bool) { self.send_esc_info_flag.store(v, ORD); }

    // --- U32 accessors ---

    pub fn zero_crosses(&self) -> u32 { self.zero_crosses.load(ORD) }
    pub fn set_zero_crosses(&self, v: u32) { self.zero_crosses.store(v, ORD); }
    /// Increment zero_crosses, capped at 10000 (matches C behavior).
    /// Safe without RMW atomic: only called from ISR context (single writer).
    pub fn increment_zero_crosses(&self) {
        let v = self.zero_crosses.load(ORD);
        if v < 10000 {
            self.zero_crosses.store(v + 1, ORD);
        }
    }

    pub fn commutation_interval(&self) -> u32 { self.commutation_interval.load(ORD) }
    pub fn set_commutation_interval(&self, v: u32) { self.commutation_interval.store(v, ORD); }

    pub fn e_com_time(&self) -> i32 { self.e_com_time.load(ORD) as i32 }
    pub fn set_e_com_time(&self, v: i32) { self.e_com_time.store(v as u32, ORD); }

    // --- U16 accessors ---

    pub fn newinput(&self) -> u16 { self.newinput.load(ORD) }
    pub fn set_newinput(&self, v: u16) { self.newinput.store(v, ORD); }

    pub fn adjusted_input(&self) -> u16 { self.adjusted_input.load(ORD) }
    pub fn set_adjusted_input(&self, v: u16) { self.adjusted_input.store(v, ORD); }

    pub fn duty_cycle_setpoint(&self) -> u16 { self.duty_cycle_setpoint.load(ORD) }
    pub fn set_duty_cycle_setpoint(&self, v: u16) { self.duty_cycle_setpoint.store(v, ORD); }

    pub fn signal_timeout(&self) -> u16 { self.signal_timeout.load(ORD) }
    pub fn set_signal_timeout(&self, v: u16) { self.signal_timeout.store(v, ORD); }
    pub fn increment_signal_timeout(&self) {
        let cur = self.signal_timeout.load(ORD);
        if cur < u16::MAX {
            self.signal_timeout.store(cur + 1, ORD);
        }
    }

    pub fn zero_input_count(&self) -> u16 { self.zero_input_count.load(ORD) }
    pub fn set_zero_input_count(&self, v: u16) { self.zero_input_count.store(v, ORD); }

    // --- Measurement accessors (main writes, ISR reads for EDT) ---

    pub fn stall_protection_adjust(&self) -> u16 { self.stall_protection_adjust.load(ORD) }
    pub fn set_stall_protection_adjust(&self, v: u16) { self.stall_protection_adjust.store(v, ORD); }

    pub fn actual_current(&self) -> i16 { self.actual_current.load(ORD) as i16 }
    pub fn set_actual_current(&self, v: i16) { self.actual_current.store(v as u16, ORD); }

    pub fn battery_voltage(&self) -> u16 { self.battery_voltage.load(ORD) }
    pub fn set_battery_voltage(&self, v: u16) { self.battery_voltage.store(v, ORD); }

    pub fn degrees_celsius(&self) -> i16 { self.degrees_celsius.load(ORD) as i16 }
    pub fn set_degrees_celsius(&self, v: i16) { self.degrees_celsius.store(v as u16, ORD); }
}

impl rm32::shared_comm::SharedComm for SharedState {
    fn motor_mode(&self) -> MotorMode { self.motor_mode() }
    fn set_motor_mode(&self, mode: MotorMode) { self.set_motor_mode(mode); }

    fn input_set(&self) -> bool { self.input_set() }
    fn set_input_set(&self, v: bool) { self.set_input_set(v); }
    fn dshot_telemetry(&self) -> bool { self.dshot_telemetry() }

    fn newinput(&self) -> u16 { self.newinput() }
    fn set_newinput(&self, v: u16) { self.set_newinput(v); }
    fn adjusted_input(&self) -> u16 { self.adjusted_input() }
    fn set_adjusted_input(&self, v: u16) { self.set_adjusted_input(v); }
    fn duty_cycle_setpoint(&self) -> u16 { self.duty_cycle_setpoint() }
    fn set_duty_cycle_setpoint(&self, v: u16) { self.set_duty_cycle_setpoint(v); }

    fn zero_crosses(&self) -> u32 { self.zero_crosses() }
    fn set_zero_crosses(&self, v: u32) { self.set_zero_crosses(v); }
    fn increment_zero_crosses(&self) { self.increment_zero_crosses(); }
    fn commutation_interval(&self) -> u32 { self.commutation_interval() }
    fn set_commutation_interval(&self, v: u32) { self.set_commutation_interval(v); }
    fn e_com_time(&self) -> i32 { self.e_com_time() }

    fn signal_timeout(&self) -> u16 { self.signal_timeout() }
    fn increment_signal_timeout(&self) { self.increment_signal_timeout(); }

    fn stall_protection_adjust(&self) -> u16 { self.stall_protection_adjust() }
    fn set_stall_protection_adjust(&self, v: u16) { self.set_stall_protection_adjust(v); }
}
