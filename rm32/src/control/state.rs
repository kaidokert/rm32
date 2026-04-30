//! Motor controller state — replaces the ~130 globals from main.c.
//!
//! Decomposed into focused sub-structs that each own a coherent slice of state.

use crate::pid::Pid;

/// BEMF zero-cross detection state.
#[derive(Clone)]
pub struct BemfState {
    pub counter: u8,
    pub zc_found: bool,
    pub(crate) min_counts_up: u8,
    pub(crate) min_counts_down: u8,
    pub(crate) bad_count: u8,
    pub(crate) bad_count_threshold: u8,
    pub filter_level: u8,
    pub(crate) wait_time: u16,
    pub(crate) last_zc_time: u16,
    pub(crate) this_zc_time: u16,
    pub temp_advance: u8,
}

/// Duty cycle and ramp control.
#[derive(Clone)]
pub struct DutyState {
    pub cycle: u16,
    pub(crate) maximum: u16,
    pub last: u16,
    pub adjusted: u16,
    pub min_startup: u16,
    pub startup_max: u16,
    pub minimum: u16,
    pub(crate) max_change: u8,
    pub(crate) ramp_count: u16,
    pub(crate) ramp_divider: u8,
    pub(crate) max_ramp_startup: u8,
    pub(crate) max_ramp_low_rpm: u8,
    pub(crate) max_ramp_high_rpm: u8,
}

/// PID controllers and associated state.
///
/// Owns all three PID loops (current limit, stall protection, speed control)
/// and their output accumulators. `MainState` holds this as `pub pid: PidState`.
#[derive(Clone)]
pub struct PidState {
    current: Pid,
    speed: Pid,
    stall: Pid,
    use_current_limit: bool,
    current_limit_adjust: i16,
    stall_adjust: i32,
    stall_protect_target_interval: u16,
    use_speed_control: bool,
    input_override: i32,
    target_e_com_time: u32,
}

impl PidState {
    /// Create PidState with board-specific stall protection target interval.
    pub fn with_stall_target(stall_protect_target_interval: u16) -> Self {
        Self {
            stall_protect_target_interval,
            ..Self::default()
        }
    }

    /// Update current-limit PID gains from EEPROM-derived motor config.
    pub fn set_current_gains(&mut self, kp: u32, ki: u32, kd: u32) {
        self.current.set_gains(kp, ki, kd);
    }

    /// Set whether current limiting is active (from EEPROM config).
    pub fn set_use_current_limit(&mut self, v: bool) {
        self.use_current_limit = v;
    }

    /// Set whether closed-loop speed control is active.
    pub fn set_use_speed_control(&mut self, v: bool) {
        self.use_speed_control = v;
    }

    /// Stall protection PID tick. Returns stall boost value for ISR (0-150).
    pub(crate) fn tick_stall(&mut self, commutation_interval: i32) -> u16 {
        self.stall_adjust += self.stall.calculate(
            commutation_interval,
            self.stall_protect_target_interval as i32,
        );
        self.stall_adjust = self.stall_adjust.clamp(0, 150 * 10000);
        (self.stall_adjust / 10000) as u16
    }

    /// Current limit PID tick. Returns duty ceiling for ISR (clamped to min..2000).
    /// Returns 2000 (no limit) when current limiting is inactive or motor not running.
    pub(crate) fn tick_current_limit(
        &mut self,
        actual_current: i16,
        target: i32,
        min_duty: i16,
        running: bool,
    ) -> u16 {
        if self.use_current_limit && running {
            let adj = self.current.calculate(actual_current as i32, target) / 10000;
            self.current_limit_adjust -= adj as i16;
            self.current_limit_adjust = self.current_limit_adjust.clamp(min_duty, 2000);
            self.current_limit_adjust as u16
        } else {
            self.current_limit_adjust = 2000;
            2000
        }
    }

    /// Speed control PID tick. Returns throttle override if active, None otherwise.
    pub(crate) fn tick_speed_control(
        &mut self,
        e_com_time: i32,
        zero_crosses: u32,
        running: bool,
    ) -> Option<u16> {
        if !self.use_speed_control || !running {
            return None;
        }
        self.input_override += self
            .speed
            .calculate(e_com_time, self.target_e_com_time as i32);
        self.input_override = self.input_override.clamp(0, 2047 * 10000);
        if zero_crosses < 100 {
            self.speed.reset();
        }
        Some((self.input_override / 10000) as u16)
    }
}

/// Protection system state.
#[derive(Clone)]
pub struct ProtectionState {
    pub bemf_timeout_happened: u8,
    pub bemf_timeout: u8,
    pub low_voltage_count: u16,
    pub(crate) low_voltage_cutoff: bool,
}

/// Sensor measurements.
#[derive(Clone, Default)]
pub struct Measurements {
    pub battery_voltage: crate::units::MilliVolts,
    pub actual_current: crate::units::MilliAmps,
    pub degrees_celsius: crate::units::DegreesCelsius,
    pub(crate) consumed_current: i32,
    /// EWMA filter for ADC voltage readings.
    pub(crate) voltage_filter: crate::filter::EwmaPow2<3>,
    /// Multi-stage filter for ADC current readings.
    pub(crate) current_filter: crate::current::CurrentFilter,
}

/// Main-loop timing state — eRPM and commutation interval tracking.
///
/// Owns the main-loop side of timing computation. ISR-owned timing
/// (commutation_interval, zero_crosses, e_com_time) lives in SharedComm.
#[derive(Clone, Default)]
pub struct TimingState {
    pub average_interval: u32,
    pub last_average_interval: u32,
    pub e_rpm: u16,
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
            temp_advance: 0,
        }
    }
}

impl Default for DutyState {
    fn default() -> Self {
        Self {
            cycle: 0,
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

impl Default for ProtectionState {
    fn default() -> Self {
        Self {
            bemf_timeout_happened: 0,
            bemf_timeout: 10,
            low_voltage_count: 0,
            low_voltage_cutoff: false,
        }
    }
}
