//! Shared state between ISR and main loop — all atomic, lock-free.
//!
//! Uses `portable-atomic` for cross-architecture support:
//! - On Cortex-M4+ (G431, L431): hardware LDREX/STREX for lock-free CAS
//! - On Cortex-M0+ (G071, F051): automatic fallback to interrupt-free sections
//!
//! Acquire/Release ordering for cross-context data passing.

use crate::motor_mode::MotorMode;
use portable_atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

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
    duty_cycle: AtomicU16, // ISR writes, main reads (bidir speed gate)
    forward: AtomicBool,   // direction: ISR reads, main writes on bidir change
    signal_timeout: AtomicU16,
    zero_input_count: AtomicU16,

    // Telemetry (main computes, ISR reads for speed PID)
    e_com_time: AtomicU32, // stored as u32, interpreted as i32

    // Stall protection (main computes, ISR applies to duty)
    stall_protection_adjust: AtomicU16,
    current_limit_adjust: AtomicU16, // main PID publishes, ISR clamps duty

    // Measurements (main writes, ISR reads for EDT)
    actual_current: AtomicU16,  // mA, stored as u16
    battery_voltage: AtomicU16, // mV
    degrees_celsius: AtomicU16, // stored as u16, interpreted as i16

    // ISR→main: interval timer count for stall detection
    interval_timer_count: AtomicU32,

    // Main→ISR published control (main computes, ISR applies)
    tim1_arr: AtomicU16,           // variable PWM auto-reload
    duty_maximum: AtomicU16,       // eRPM/temperature throttle restriction
    filter_level: AtomicU8,        // BEMF comparator filter samples
    min_bemf_counts: AtomicU8,     // min zero-cross detection threshold
    auto_advance: AtomicU8,        // commutation timing advance level
    prop_brake_active: AtomicBool, // proportional brake engaged (main sets, ISR reads)
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
            duty_cycle: AtomicU16::new(0),
            forward: AtomicBool::new(true),
            signal_timeout: AtomicU16::new(0),
            zero_input_count: AtomicU16::new(0),
            e_com_time: AtomicU32::new(0),
            interval_timer_count: AtomicU32::new(0),
            stall_protection_adjust: AtomicU16::new(0),
            current_limit_adjust: AtomicU16::new(2000),
            actual_current: AtomicU16::new(0),
            battery_voltage: AtomicU16::new(0),
            degrees_celsius: AtomicU16::new(0),
            tim1_arr: AtomicU16::new(1999),
            duty_maximum: AtomicU16::new(2000),
            filter_level: AtomicU8::new(5),
            min_bemf_counts: AtomicU8::new(2),
            auto_advance: AtomicU8::new(0),
            prop_brake_active: AtomicBool::new(false),
        }
    }

    // --- Motor mode ---

    pub fn motor_mode(&self) -> MotorMode {
        MotorMode::from_u8(self.motor_mode.load(ACQ))
    }
    pub fn set_motor_mode(&self, mode: MotorMode) {
        self.motor_mode.store(mode as u8, REL);
    }

    /// Atomic state transition via CAS.
    /// On M4: hardware LDREX/STREX. On M0: portable-atomic disables interrupts.
    pub fn transition(&self, event: crate::motor_mode::MotorEvent) {
        self.motor_mode
            .fetch_update(REL, ACQ, |cur| {
                let mode = MotorMode::from_u8(cur);
                let new = mode.transition(event);
                if new != mode { Some(new as u8) } else { None }
            })
            .ok();
    }

    // Convenience getters (delegate to motor_mode)
    pub fn armed(&self) -> bool {
        self.motor_mode().is_armed()
    }
    pub fn running(&self) -> bool {
        self.motor_mode().is_running()
    }
    pub fn old_routine(&self) -> bool {
        self.motor_mode().is_old_routine()
    }
    pub fn stepper_sine(&self) -> bool {
        self.motor_mode().is_stepper_sine()
    }

    // Convenience setters — atomic CAS via portable-atomic.
    // Hardware LDREX/STREX on M4, interrupt-free fallback on M0.
    pub fn set_armed(&self, v: bool) {
        self.motor_mode
            .fetch_update(REL, ACQ, |cur| {
                let mode = MotorMode::from_u8(cur);
                if v && !mode.is_armed() {
                    Some(MotorMode::Armed as u8)
                } else if !v {
                    Some(MotorMode::Disarmed as u8)
                } else {
                    None
                }
            })
            .ok();
    }
    pub fn set_running(&self, v: bool) {
        self.motor_mode
            .fetch_update(REL, ACQ, |cur| {
                let mode = MotorMode::from_u8(cur);
                if v && !mode.is_running() {
                    Some(MotorMode::OldRoutine as u8)
                } else if !v && mode.is_running() {
                    Some(MotorMode::Armed as u8)
                } else {
                    None
                }
            })
            .ok();
    }
    pub fn set_old_routine(&self, v: bool) {
        self.motor_mode
            .fetch_update(REL, ACQ, |cur| {
                let mode = MotorMode::from_u8(cur);
                if v && mode.is_running() {
                    Some(MotorMode::OldRoutine as u8)
                } else if !v && mode.is_old_routine() {
                    Some(MotorMode::Running as u8)
                } else {
                    None
                }
            })
            .ok();
    }
    pub fn set_stepper_sine(&self, v: bool) {
        self.motor_mode
            .fetch_update(REL, ACQ, |cur| {
                let mode = MotorMode::from_u8(cur);
                if v {
                    Some(MotorMode::StepperSine as u8)
                } else if mode.is_stepper_sine() {
                    Some(MotorMode::Armed as u8)
                } else {
                    None
                }
            })
            .ok();
    }

    // --- Bool accessors ---

    pub fn input_set(&self) -> bool {
        self.input_set.load(ACQ)
    }
    pub fn set_input_set(&self, v: bool) {
        self.input_set.store(v, REL);
    }

    pub fn send_telemetry(&self) -> bool {
        self.send_telemetry.load(ACQ)
    }
    pub fn set_send_telemetry(&self, v: bool) {
        self.send_telemetry.store(v, REL);
    }

    pub fn dshot(&self) -> bool {
        self.dshot.load(ACQ)
    }
    pub fn set_dshot(&self, v: bool) {
        self.dshot.store(v, REL);
    }

    pub fn servo_pwm(&self) -> bool {
        self.servo_pwm.load(ACQ)
    }
    pub fn set_servo_pwm(&self, v: bool) {
        self.servo_pwm.store(v, REL);
    }

    pub fn dshot_telemetry(&self) -> bool {
        self.dshot_telemetry.load(ACQ)
    }
    pub fn set_dshot_telemetry(&self, v: bool) {
        self.dshot_telemetry.store(v, REL);
    }

    pub fn save_settings_flag(&self) -> bool {
        self.save_settings_flag.load(ACQ)
    }
    pub fn set_save_settings_flag(&self, v: bool) {
        self.save_settings_flag.store(v, REL);
    }

    pub fn send_esc_info_flag(&self) -> bool {
        self.send_esc_info_flag.load(ACQ)
    }
    pub fn set_send_esc_info_flag(&self, v: bool) {
        self.send_esc_info_flag.store(v, REL);
    }

    // --- U32 accessors ---

    pub fn zero_crosses(&self) -> u32 {
        self.zero_crosses.load(ACQ)
    }
    pub fn set_zero_crosses(&self, v: u32) {
        self.zero_crosses.store(v, REL);
    }
    /// Increment zero_crosses, capped at 10000 (matches C behavior).
    pub fn increment_zero_crosses(&self) {
        self.zero_crosses
            .fetch_update(REL, ACQ, |v| if v < 10000 { Some(v + 1) } else { None })
            .ok();
    }

    pub fn commutation_interval(&self) -> u32 {
        self.commutation_interval.load(ACQ)
    }
    pub fn set_commutation_interval(&self, v: u32) {
        self.commutation_interval.store(v, REL);
    }

    pub fn e_com_time(&self) -> i32 {
        self.e_com_time.load(ACQ) as i32
    }
    pub fn set_e_com_time(&self, v: i32) {
        self.e_com_time.store(v as u32, REL);
    }

    // --- U16 accessors ---

    pub fn newinput(&self) -> u16 {
        self.newinput.load(ACQ)
    }
    pub fn set_newinput(&self, v: u16) {
        self.newinput.store(v, REL);
    }

    pub fn adjusted_input(&self) -> u16 {
        self.adjusted_input.load(ACQ)
    }
    pub fn set_adjusted_input(&self, v: u16) {
        self.adjusted_input.store(v, REL);
    }

    pub fn duty_cycle_setpoint(&self) -> u16 {
        self.duty_cycle_setpoint.load(ACQ)
    }
    pub fn set_duty_cycle_setpoint(&self, v: u16) {
        self.duty_cycle_setpoint.store(v, REL);
    }

    pub fn duty_cycle(&self) -> u16 {
        self.duty_cycle.load(ACQ)
    }
    pub fn set_duty_cycle(&self, v: u16) {
        self.duty_cycle.store(v, REL);
    }

    pub fn forward(&self) -> bool {
        self.forward.load(ACQ)
    }
    pub fn set_forward(&self, v: bool) {
        self.forward.store(v, REL);
    }

    pub fn signal_timeout(&self) -> u16 {
        self.signal_timeout.load(ACQ)
    }
    pub fn set_signal_timeout(&self, v: u16) {
        self.signal_timeout.store(v, REL);
    }
    pub fn increment_signal_timeout(&self) {
        self.signal_timeout
            .fetch_update(REL, ACQ, |v| if v < u16::MAX { Some(v + 1) } else { None })
            .ok();
    }

    pub fn zero_input_count(&self) -> u16 {
        self.zero_input_count.load(ACQ)
    }
    pub fn set_zero_input_count(&self, v: u16) {
        self.zero_input_count.store(v, REL);
    }

    // --- Measurement accessors (main writes, ISR reads for EDT) ---

    pub fn stall_protection_adjust(&self) -> u16 {
        self.stall_protection_adjust.load(ACQ)
    }
    pub fn set_stall_protection_adjust(&self, v: u16) {
        self.stall_protection_adjust.store(v, REL);
    }

    pub fn current_limit_adjust(&self) -> u16 {
        self.current_limit_adjust.load(ACQ)
    }
    pub fn set_current_limit_adjust(&self, v: u16) {
        self.current_limit_adjust.store(v, REL);
    }

    pub fn actual_current(&self) -> i16 {
        self.actual_current.load(ACQ) as i16
    }
    pub fn set_actual_current(&self, v: i16) {
        self.actual_current.store(v as u16, REL);
    }

    pub fn battery_voltage(&self) -> u16 {
        self.battery_voltage.load(ACQ)
    }
    pub fn set_battery_voltage(&self, v: u16) {
        self.battery_voltage.store(v, REL);
    }

    pub fn degrees_celsius(&self) -> i16 {
        self.degrees_celsius.load(ACQ) as i16
    }
    pub fn set_degrees_celsius(&self, v: i16) {
        self.degrees_celsius.store(v as u16, REL);
    }

    pub fn interval_timer_count(&self) -> u32 {
        self.interval_timer_count.load(ACQ)
    }
    pub fn set_interval_timer_count(&self, v: u32) {
        self.interval_timer_count.store(v, REL);
    }

    // --- Main→ISR published control ---

    pub fn tim1_arr(&self) -> u16 {
        self.tim1_arr.load(ACQ)
    }
    pub fn set_tim1_arr(&self, v: u16) {
        self.tim1_arr.store(v, REL);
    }

    pub fn duty_maximum(&self) -> u16 {
        self.duty_maximum.load(ACQ)
    }
    pub fn set_duty_maximum(&self, v: u16) {
        self.duty_maximum.store(v, REL);
    }

    pub fn filter_level(&self) -> u8 {
        self.filter_level.load(ACQ)
    }
    pub fn set_filter_level(&self, v: u8) {
        self.filter_level.store(v, REL);
    }

    pub fn min_bemf_counts(&self) -> u8 {
        self.min_bemf_counts.load(ACQ)
    }
    pub fn set_min_bemf_counts(&self, v: u8) {
        self.min_bemf_counts.store(v, REL);
    }

    pub fn auto_advance(&self) -> u8 {
        self.auto_advance.load(ACQ)
    }
    pub fn set_auto_advance(&self, v: u8) {
        self.auto_advance.store(v, REL);
    }

    pub fn prop_brake_active(&self) -> bool {
        self.prop_brake_active.load(ACQ)
    }
    pub fn set_prop_brake_active(&self, v: bool) {
        self.prop_brake_active.store(v, REL);
    }
}

impl crate::shared_comm::MotorState for SharedState {
    fn motor_mode(&self) -> MotorMode {
        self.motor_mode()
    }
    fn set_motor_mode(&self, mode: MotorMode) {
        self.set_motor_mode(mode);
    }
    fn transition(&self, event: crate::motor_mode::MotorEvent) {
        SharedState::transition(self, event);
    }
    // Override convenience setters to use atomic CAS inherent methods
    // instead of the non-atomic trait defaults (load + store).
    fn set_armed(&self, v: bool) {
        SharedState::set_armed(self, v);
    }
    fn set_running(&self, v: bool) {
        SharedState::set_running(self, v);
    }
    fn set_old_routine(&self, v: bool) {
        SharedState::set_old_routine(self, v);
    }
    fn set_stepper_sine(&self, v: bool) {
        SharedState::set_stepper_sine(self, v);
    }
}

impl crate::shared_comm::IsrTiming for SharedState {
    fn zero_crosses(&self) -> u32 {
        self.zero_crosses()
    }
    fn set_zero_crosses(&self, v: u32) {
        self.set_zero_crosses(v);
    }
    fn increment_zero_crosses(&self) {
        self.increment_zero_crosses();
    }
    fn commutation_interval(&self) -> u32 {
        self.commutation_interval()
    }
    fn set_commutation_interval(&self, v: u32) {
        self.set_commutation_interval(v);
    }
    fn e_com_time(&self) -> i32 {
        self.e_com_time()
    }
    fn set_e_com_time(&self, v: i32) {
        SharedState::set_e_com_time(self, v);
    }
    fn interval_timer_count(&self) -> u32 {
        SharedState::interval_timer_count(self)
    }
    fn set_interval_timer_count(&self, v: u32) {
        SharedState::set_interval_timer_count(self, v);
    }
    fn signal_timeout(&self) -> u16 {
        self.signal_timeout()
    }
    fn increment_signal_timeout(&self) {
        self.increment_signal_timeout();
    }
    fn duty_cycle(&self) -> u16 {
        SharedState::duty_cycle(self)
    }
    fn set_duty_cycle(&self, v: u16) {
        SharedState::set_duty_cycle(self, v);
    }
    fn forward(&self) -> bool {
        SharedState::forward(self)
    }
    fn set_forward(&self, v: bool) {
        SharedState::set_forward(self, v);
    }
}

impl crate::shared_comm::MainControl for SharedState {
    fn adjusted_input(&self) -> u16 {
        SharedState::adjusted_input(self)
    }
    fn set_adjusted_input(&self, v: u16) {
        SharedState::set_adjusted_input(self, v);
    }
    fn duty_cycle_setpoint(&self) -> u16 {
        SharedState::duty_cycle_setpoint(self)
    }
    fn set_duty_cycle_setpoint(&self, v: u16) {
        SharedState::set_duty_cycle_setpoint(self, v);
    }
    fn stall_protection_adjust(&self) -> u16 {
        SharedState::stall_protection_adjust(self)
    }
    fn set_stall_protection_adjust(&self, v: u16) {
        SharedState::set_stall_protection_adjust(self, v);
    }
    fn current_limit_adjust(&self) -> u16 {
        SharedState::current_limit_adjust(self)
    }
    fn set_current_limit_adjust(&self, v: u16) {
        SharedState::set_current_limit_adjust(self, v);
    }
    fn prop_brake_active(&self) -> bool {
        SharedState::prop_brake_active(self)
    }
    fn set_prop_brake_active(&self, v: bool) {
        SharedState::set_prop_brake_active(self, v);
    }
    fn tim1_arr(&self) -> u16 {
        SharedState::tim1_arr(self)
    }
    fn set_tim1_arr(&self, v: u16) {
        SharedState::set_tim1_arr(self, v);
    }
    fn duty_maximum(&self) -> u16 {
        SharedState::duty_maximum(self)
    }
    fn set_duty_maximum(&self, v: u16) {
        SharedState::set_duty_maximum(self, v);
    }
    fn filter_level(&self) -> u8 {
        SharedState::filter_level(self)
    }
    fn set_filter_level(&self, v: u8) {
        SharedState::set_filter_level(self, v);
    }
    fn min_bemf_counts(&self) -> u8 {
        SharedState::min_bemf_counts(self)
    }
    fn set_min_bemf_counts(&self, v: u8) {
        SharedState::set_min_bemf_counts(self, v);
    }
    fn auto_advance(&self) -> u8 {
        SharedState::auto_advance(self)
    }
    fn set_auto_advance(&self, v: u8) {
        SharedState::set_auto_advance(self, v);
    }
    fn set_actual_current(&self, v: i16) {
        SharedState::set_actual_current(self, v);
    }
    fn set_battery_voltage(&self, v: u16) {
        SharedState::set_battery_voltage(self, v);
    }
    fn set_degrees_celsius(&self, v: i16) {
        SharedState::set_degrees_celsius(self, v);
    }
    fn battery_voltage(&self) -> u16 {
        SharedState::battery_voltage(self)
    }
}

impl crate::shared_comm::SharedComm for SharedState {
    fn input_set(&self) -> bool {
        SharedState::input_set(self)
    }
    fn set_input_set(&self, v: bool) {
        SharedState::set_input_set(self, v);
    }
    fn dshot_telemetry(&self) -> bool {
        SharedState::dshot_telemetry(self)
    }
    fn is_dshot(&self) -> bool {
        SharedState::dshot(self)
    }
    fn set_is_dshot(&self, v: bool) {
        SharedState::set_dshot(self, v);
    }
    fn newinput(&self) -> u16 {
        SharedState::newinput(self)
    }
    fn set_newinput(&self, v: u16) {
        SharedState::set_newinput(self, v);
    }
    fn send_telemetry(&self) -> bool {
        SharedState::send_telemetry(self)
    }
    fn set_send_telemetry(&self, v: bool) {
        SharedState::set_send_telemetry(self, v);
    }
    fn save_settings_flag(&self) -> bool {
        SharedState::save_settings_flag(self)
    }
    fn set_save_settings_flag(&self, v: bool) {
        SharedState::set_save_settings_flag(self, v);
    }
    fn send_esc_info_flag(&self) -> bool {
        SharedState::send_esc_info_flag(self)
    }
    fn set_send_esc_info_flag(&self, v: bool) {
        SharedState::set_send_esc_info_flag(self, v);
    }
}
