//! Motor controller state — replaces the ~130 globals from main.c.
//!
//! Decomposed into focused sub-structs that each own a coherent slice of state.

use crate::pid::Pid;

/// BEMF zero-cross detection state.
#[derive(Clone)]
pub struct BemfState {
    pub counter: u8,
    pub zc_found: bool,
    pub min_counts_up: u8,
    pub min_counts_down: u8,
    pub bad_count: u8,
    pub bad_count_threshold: u8,
    pub filter_level: u8,
    pub wait_time: u16,
    pub last_zc_time: u16,
    pub this_zc_time: u16,
    pub advance: u16,
    pub temp_advance: u8,
    pub auto_advance_level: u8,
}

/// Duty cycle and ramp control.
#[derive(Clone)]
pub struct DutyState {
    pub cycle: u16,
    pub setpoint: u16,
    pub maximum: u16,
    pub last: u16,
    pub adjusted: u16,
    pub min_startup: u16,
    pub startup_max: u16,
    pub minimum: u16,
    pub max_change: u8,
    pub ramp_count: u16,
    pub ramp_divider: u8,
    pub max_ramp_startup: u8,
    pub max_ramp_low_rpm: u8,
    pub max_ramp_high_rpm: u8,
}

/// Input signal state.
#[derive(Clone, Default)]
pub struct InputState {
    pub input: u16,
    pub adjusted: u16,
    pub newinput: u16,
    pub input_set: bool,
    pub dshot: bool,
    pub servo_pwm: bool,
    pub signal_timeout: u16,
    pub zero_input_count: u16,
    pub dshot_telemetry: bool,
    pub edt_armed: bool,
    pub edt_arm_enable: bool,
    pub servo_low_threshold: u16,
    pub servo_high_threshold: u16,
    pub servo_neutral: u16,
}

/// PID controllers and associated state.
///
/// Owns all three PID loops (current limit, stall protection, speed control)
/// and their output accumulators. `MainState` holds this as `pub pid: PidState`.
#[derive(Clone)]
pub struct PidState {
    pub current: Pid,
    pub speed: Pid,
    pub stall: Pid,
    /// Whether current limiting is active (from EEPROM config).
    pub use_current_limit: bool,
    /// Current limit duty ceiling (adjusted by PID). 2000 = no limit.
    pub current_limit_adjust: i16,
    /// Stall protection PID output accumulator.
    pub stall_adjust: i32,
    /// Stall protection target commutation interval.
    pub stall_protect_target_interval: u16,
    /// Whether closed-loop speed control is active.
    pub use_speed_control: bool,
    /// Speed PID output accumulator (throttle override).
    pub input_override: i32,
    /// Speed PID target e_com_time.
    pub target_e_com_time: u32,
}

/// Telemetry scheduling state.
#[derive(Clone, Default)]
pub struct TelemetryState {
    pub send_telemetry: bool,
    pub send_esc_info: bool,
    pub ms_count: u16,
}

/// Protection system state.
#[derive(Clone)]
pub struct ProtectionState {
    pub bemf_timeout_happened: u8,
    pub bemf_timeout: u8,
    pub low_voltage_count: u16,
    pub low_voltage_cutoff: bool,
    pub desync_happened: u32,
}

/// Sensor measurements.
#[derive(Clone, Default)]
pub struct Measurements {
    pub battery_voltage: crate::units::MilliVolts,
    pub actual_current: crate::units::MilliAmps,
    pub degrees_celsius: crate::units::DegreesCelsius,
    pub consumed_current: i32,
}

/// Timing and commutation interval tracking.
#[derive(Clone)]
pub struct TimingState {
    pub commutation_interval: u32,
    pub commutation_intervals: [u16; 6],
    pub average_interval: u32,
    pub last_average_interval: u32,
    pub zero_crosses: u32,
    pub e_com_time: i32,
    pub e_rpm: u16,
    pub polling_mode_changeover: u32,
}

impl Default for BemfState {
    fn default() -> Self {
        Self {
            counter: 0,
            zc_found: false,
            min_counts_up: 2,
            min_counts_down: 2,
            bad_count: 0,
            bad_count_threshold: 2,
            filter_level: 5,
            wait_time: 0,
            last_zc_time: 0,
            this_zc_time: 0,
            advance: 0,
            temp_advance: 0,
            auto_advance_level: 0,
        }
    }
}

impl Default for DutyState {
    fn default() -> Self {
        Self {
            cycle: 0,
            setpoint: 0,
            maximum: 2000,
            last: 0,
            adjusted: 0,
            min_startup: 120,
            startup_max: 200,
            minimum: 5, // DEAD_TIME
            max_change: 2,
            ramp_count: 0,
            ramp_divider: 0,
            max_ramp_startup: 2,
            max_ramp_low_rpm: 6,
            max_ramp_high_rpm: 16,
        }
    }
}

impl Default for PidState {
    fn default() -> Self {
        Self {
            current: Pid::new(400, 0, 1000, 20000, 100000),
            speed: Pid::new(10, 0, 100, 10000, 50000),
            stall: Pid::new(1, 0, 50, 10000, 50000),
            use_current_limit: false,
            current_limit_adjust: 2000,
            stall_adjust: 0,
            stall_protect_target_interval: 0,
            use_speed_control: false,
            input_override: 0,
            target_e_com_time: 0,
        }
    }
}

impl Default for TimingState {
    fn default() -> Self {
        Self {
            commutation_interval: 12500,
            commutation_intervals: [0; 6],
            average_interval: 0,
            last_average_interval: 0,
            zero_crosses: 0,
            e_com_time: 0,
            e_rpm: 0,
            polling_mode_changeover: 0,
        }
    }
}

impl Default for ProtectionState {
    fn default() -> Self {
        Self {
            bemf_timeout_happened: 0,
            bemf_timeout: 10,
            low_voltage_count: 0,
            low_voltage_cutoff: false,
            desync_happened: 0,
        }
    }
}
