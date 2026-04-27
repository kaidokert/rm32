//! Shared state between ISR and main loop — all atomic, lock-free.
//!
//! Uses Release (stores) / Acquire (loads) ordering for cross-context
//! data passing. While Relaxed is sufficient on single-core Cortex-M0+
//! (no cache, no reordering), Acquire/Release is future-proof for
//! dual-core targets (RP2040, STM32H7) and communicates intent.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};
use rm32::motor_mode::MotorMode;

/// Store ordering — ensures writes are visible to other contexts.
const REL: Ordering = Ordering::Release;
/// Load ordering — ensures we see all prior writes from other contexts.
const ACQ: Ordering = Ordering::Acquire;

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

impl Default for SharedState {
    fn default() -> Self {
        Self::new()
    }
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

    pub fn motor_mode(&self) -> MotorMode { MotorMode::from_u8(self.motor_mode.load(ACQ)) }
    pub fn set_motor_mode(&self, mode: MotorMode) { self.motor_mode.store(mode as u8, REL); }

    /// Atomic state transition under interrupt::free.
    pub fn transition(&self, event: rm32::motor_mode::MotorEvent) {
        cortex_m::interrupt::free(|_| {
            let cur = self.motor_mode();
            let new = cur.transition(event);
            if new != cur {
                self.motor_mode.store(new as u8, REL);
            }
        });
    }

    // Convenience getters (delegate to motor_mode)
    pub fn armed(&self) -> bool { self.motor_mode().is_armed() }
    pub fn running(&self) -> bool { self.motor_mode().is_running() }
    pub fn old_routine(&self) -> bool { self.motor_mode().is_old_routine() }
    pub fn stepper_sine(&self) -> bool { self.motor_mode().is_stepper_sine() }

    // Convenience setters — interrupt-free CAS for mode transitions.
    // On Cortex-M0+ (no CAS instruction), disabling interrupts is the
    // only way to make load-check-store atomic.
    pub fn set_armed(&self, v: bool) {
        cortex_m::interrupt::free(|_| {
            if v && !self.armed() {
                self.motor_mode.store(MotorMode::Armed as u8, REL);
            } else if !v {
                self.motor_mode.store(MotorMode::Disarmed as u8, REL);
            }
        });
    }
    pub fn set_running(&self, v: bool) {
        cortex_m::interrupt::free(|_| {
            if v && !self.running() {
                self.motor_mode.store(MotorMode::OldRoutine as u8, REL);
            } else if !v && self.running() {
                self.motor_mode.store(MotorMode::Armed as u8, REL);
            }
        });
    }
    pub fn set_old_routine(&self, v: bool) {
        cortex_m::interrupt::free(|_| {
            if v && self.running() {
                self.motor_mode.store(MotorMode::OldRoutine as u8, REL);
            } else if !v && self.old_routine() {
                self.motor_mode.store(MotorMode::Running as u8, REL);
            }
        });
    }
    pub fn set_stepper_sine(&self, v: bool) {
        cortex_m::interrupt::free(|_| {
            if v {
                self.motor_mode.store(MotorMode::StepperSine as u8, REL);
            } else if self.stepper_sine() {
                self.motor_mode.store(MotorMode::Armed as u8, REL);
            }
        });
    }

    // --- Bool accessors ---

    pub fn input_set(&self) -> bool { self.input_set.load(ACQ) }
    pub fn set_input_set(&self, v: bool) { self.input_set.store(v, REL); }

    pub fn send_telemetry(&self) -> bool { self.send_telemetry.load(ACQ) }
    pub fn set_send_telemetry(&self, v: bool) { self.send_telemetry.store(v, REL); }

    pub fn dshot(&self) -> bool { self.dshot.load(ACQ) }
    pub fn set_dshot(&self, v: bool) { self.dshot.store(v, REL); }

    pub fn servo_pwm(&self) -> bool { self.servo_pwm.load(ACQ) }
    pub fn set_servo_pwm(&self, v: bool) { self.servo_pwm.store(v, REL); }

    pub fn dshot_telemetry(&self) -> bool { self.dshot_telemetry.load(ACQ) }
    pub fn set_dshot_telemetry(&self, v: bool) { self.dshot_telemetry.store(v, REL); }

    pub fn save_settings_flag(&self) -> bool { self.save_settings_flag.load(ACQ) }
    pub fn set_save_settings_flag(&self, v: bool) { self.save_settings_flag.store(v, REL); }

    pub fn send_esc_info_flag(&self) -> bool { self.send_esc_info_flag.load(ACQ) }
    pub fn set_send_esc_info_flag(&self, v: bool) { self.send_esc_info_flag.store(v, REL); }

    // --- U32 accessors ---

    pub fn zero_crosses(&self) -> u32 { self.zero_crosses.load(ACQ) }
    pub fn set_zero_crosses(&self, v: u32) { self.zero_crosses.store(v, REL); }
    /// Increment zero_crosses, capped at 10000 (matches C behavior).
    /// Uses interrupt-free section to prevent ISR/main RMW race.
    pub fn increment_zero_crosses(&self) {
        cortex_m::interrupt::free(|_| {
            let v = self.zero_crosses.load(ACQ);
            if v < 10000 {
                self.zero_crosses.store(v + 1, REL);
            }
        });
    }

    pub fn commutation_interval(&self) -> u32 { self.commutation_interval.load(ACQ) }
    pub fn set_commutation_interval(&self, v: u32) { self.commutation_interval.store(v, REL); }

    pub fn e_com_time(&self) -> i32 { self.e_com_time.load(ACQ) as i32 }
    pub fn set_e_com_time(&self, v: i32) { self.e_com_time.store(v as u32, REL); }

    // --- U16 accessors ---

    pub fn newinput(&self) -> u16 { self.newinput.load(ACQ) }
    pub fn set_newinput(&self, v: u16) { self.newinput.store(v, REL); }

    pub fn adjusted_input(&self) -> u16 { self.adjusted_input.load(ACQ) }
    pub fn set_adjusted_input(&self, v: u16) { self.adjusted_input.store(v, REL); }

    pub fn duty_cycle_setpoint(&self) -> u16 { self.duty_cycle_setpoint.load(ACQ) }
    pub fn set_duty_cycle_setpoint(&self, v: u16) { self.duty_cycle_setpoint.store(v, REL); }

    pub fn signal_timeout(&self) -> u16 { self.signal_timeout.load(ACQ) }
    pub fn set_signal_timeout(&self, v: u16) { self.signal_timeout.store(v, REL); }
    pub fn increment_signal_timeout(&self) {
        cortex_m::interrupt::free(|_| {
            let v = self.signal_timeout.load(ACQ);
            if v < u16::MAX {
                self.signal_timeout.store(v + 1, REL);
            }
        });
    }

    pub fn zero_input_count(&self) -> u16 { self.zero_input_count.load(ACQ) }
    pub fn set_zero_input_count(&self, v: u16) { self.zero_input_count.store(v, REL); }

    // --- Measurement accessors (main writes, ISR reads for EDT) ---

    pub fn stall_protection_adjust(&self) -> u16 { self.stall_protection_adjust.load(ACQ) }
    pub fn set_stall_protection_adjust(&self, v: u16) { self.stall_protection_adjust.store(v, REL); }

    pub fn actual_current(&self) -> i16 { self.actual_current.load(ACQ) as i16 }
    pub fn set_actual_current(&self, v: i16) { self.actual_current.store(v as u16, REL); }

    pub fn battery_voltage(&self) -> u16 { self.battery_voltage.load(ACQ) }
    pub fn set_battery_voltage(&self, v: u16) { self.battery_voltage.store(v, REL); }

    pub fn degrees_celsius(&self) -> i16 { self.degrees_celsius.load(ACQ) as i16 }
    pub fn set_degrees_celsius(&self, v: i16) { self.degrees_celsius.store(v as u16, REL); }
}

impl rm32::shared_comm::SharedComm for SharedState {
    fn motor_mode(&self) -> MotorMode { self.motor_mode() }
    fn set_motor_mode(&self, mode: MotorMode) { self.set_motor_mode(mode); }

    /// Atomic transition: load-compute-store under interrupt::free.
    fn transition(&self, event: rm32::motor_mode::MotorEvent) {
        cortex_m::interrupt::free(|_| {
            let cur = self.motor_mode();
            let new = cur.transition(event);
            if new != cur {
                self.motor_mode.store(new as u8, REL);
            }
        });
    }

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

    fn battery_voltage(&self) -> u16 { self.battery_voltage() }
}
