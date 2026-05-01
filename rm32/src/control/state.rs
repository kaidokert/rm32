//! Motor controller state — replaces the ~130 globals from main.c.
//!
//! Decomposed into focused sub-structs that each own a coherent slice of state.

use crate::pid::Pid;

/// BEMF zero-cross detection state.
#[derive(Clone)]
pub struct BemfState {
    pub(crate) counter: u8,
    pub(crate) zc_found: bool,
    pub(crate) min_counts_up: u8,
    pub(crate) min_counts_down: u8,
    pub(crate) bad_count: u8,
    pub(crate) bad_count_threshold: u8,
    pub(crate) filter_level: u8,
    pub(crate) wait_time: u16,
    pub(crate) last_zc_time: u16,
    pub(crate) this_zc_time: u16,
    pub(crate) temp_advance: u8,
}

/// Duty cycle and ramp control.
#[derive(Clone)]
pub struct DutyState {
    cycle: u16,
    maximum: u16,
    last: u16,
    adjusted: u16,
    min_startup: u16,
    startup_max: u16,
    minimum: u16,
    max_change: u8,
    ramp_count: u16,
    ramp_divider: u8,
    max_ramp_startup: u8,
    max_ramp_low_rpm: u8,
    max_ramp_high_rpm: u8,
}

impl DutyState {
    /// Set duty limits from motor config. Called during ISR state init.
    pub fn set_duty_limits(&mut self, minimum: u16, min_startup: u16, startup_max: u16) {
        self.minimum = minimum;
        self.min_startup = min_startup;
        self.startup_max = startup_max;
    }

    /// Apply dead-time override to duty thresholds.
    pub fn apply_dead_time_override(&mut self, dead_time: u16) {
        self.min_startup += dead_time;
        self.minimum += dead_time;
        self.startup_max += dead_time;
    }

    /// Map throttle input to duty setpoint, with startup clamping.
    /// Returns the setpoint and whether this is a startup condition.
    pub(crate) fn compute_setpoint(
        &self,
        input: u16,
        zero_crosses: u32,
        stall_protection: u8,
    ) -> u16 {
        let setpoint = crate::functions::map(
            input as i32,
            crate::constants::THROTTLE_MIN_SIGNAL as i32,
            crate::constants::DSHOT_MAX_THROTTLE as i32,
            self.minimum as i32,
            crate::constants::DUTY_SCALE_MAX as i32,
        ) as u16;
        let safe_shift = stall_protection.min(5);
        if zero_crosses < (crate::constants::STARTUP_ZC_BASE >> safe_shift) {
            setpoint.clamp(self.min_startup, self.startup_max)
        } else {
            setpoint
        }
    }

    /// Apply ramp rate limiting to duty cycle.
    /// `battery_voltage`: mV for voltage-based ramp, `commutation_interval`: for speed check.
    pub(crate) fn ramp_limit(
        &mut self,
        battery_voltage: u16,
        commutation_interval: u32,
        zero_crosses: u32,
        voltage_based: bool,
    ) {
        if self.ramp_count > self.ramp_divider as u16 {
            self.ramp_count = 0;
            if voltage_based {
                let v_change =
                    crate::functions::map(battery_voltage as i32, 800, 2200, 10, 1) as u8;
                self.max_change = if commutation_interval > 200 {
                    v_change
                } else {
                    v_change.saturating_mul(3)
                };
            } else if zero_crosses < 150 || self.last < 150 {
                self.max_change = self.max_ramp_startup;
            } else if self.last > 500 {
                self.max_change = self.max_ramp_low_rpm;
            } else {
                self.max_change = self.max_ramp_high_rpm;
            }
            let change = self.max_change as u16;
            if self.cycle > self.last + change {
                self.cycle = self.last + change;
            }
            if self.last > self.cycle + change {
                self.cycle = self.last - change;
            }
        } else {
            self.cycle = self.last;
        }
    }

    /// Apply stall boost, duty maximum, and current limit ceilings.
    pub(crate) fn clamp_ceilings(
        &mut self,
        stall_boost: u16,
        duty_maximum: u16,
        current_limit: u16,
    ) {
        self.cycle = self.cycle.saturating_add(stall_boost);
        self.maximum = duty_maximum;
        if self.cycle > self.maximum {
            self.cycle = self.maximum;
        }
        if self.cycle > current_limit {
            self.cycle = current_limit;
        }
    }

    /// Set duty to startup value when motor first starts.
    pub(crate) fn start_motor(&mut self) {
        self.last = self.min_startup;
    }

    /// Increment ramp counter (called each ISR tick).
    pub(crate) fn increment_ramp_count(&mut self) {
        self.ramp_count += 1;
    }

    /// Set ramp divider (test setup).
    pub fn set_ramp_divider(&mut self, v: u8) {
        self.ramp_divider = v;
    }

    /// Compute PWM compare value from duty cycle and timer auto-reload.
    pub(crate) fn pwm_compare(&self, tim1_arr: u16) -> u16 {
        ((self.cycle as u32 * tim1_arr as u32) / crate::constants::DUTY_SCALE_MAX as u32 + 1) as u16
    }

    /// Compute PWM compare value for proportional brake mode.
    pub(crate) fn brake_compare(drag_brake_strength: u8, tim1_arr: u16) -> u16 {
        let brake_duty = drag_brake_strength as u32 * 200;
        tim1_arr.saturating_sub(
            (brake_duty * tim1_arr as u32 / crate::constants::DUTY_SCALE_MAX as u32) as u16,
        )
    }

    /// Finalize tick: store last duty, return current cycle for PWM output.
    pub(crate) fn finalize(&mut self) -> u16 {
        self.last = self.cycle;
        self.cycle
    }
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

    /// Set speed PID target e_com_time.
    pub fn set_target_e_com_time(&mut self, v: u32) {
        self.target_e_com_time = v;
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
            self.speed.clear_integral();
        }
        Some((self.input_override / 10000) as u16)
    }
}

/// Protection system state.
#[derive(Clone)]
pub struct ProtectionState {
    pub(crate) bemf_timeout_happened: u8,
    pub(crate) bemf_timeout: u8,
    pub(crate) low_voltage_count: u16,
    pub(crate) low_voltage_cutoff: bool,
}

impl ProtectionState {
    /// BEMF timeout event count — exceeds bemf_timeout when rotor is stuck.
    pub fn bemf_timeout_happened(&self) -> u8 {
        self.bemf_timeout_happened
    }

    /// BEMF timeout threshold.
    pub fn bemf_timeout(&self) -> u8 {
        self.bemf_timeout
    }

    /// Set BEMF timeout happened count (test injection).
    pub fn set_bemf_timeout_happened(&mut self, v: u8) {
        self.bemf_timeout_happened = v;
    }

    /// Set BEMF timeout threshold (test injection).
    pub fn set_bemf_timeout(&mut self, v: u8) {
        self.bemf_timeout = v;
    }
}

/// Sensor measurements.
#[derive(Clone, Default)]
pub struct Measurements {
    pub(crate) battery_voltage: crate::units::MilliVolts,
    pub(crate) actual_current: crate::units::MilliAmps,
    pub(crate) degrees_celsius: crate::units::DegreesCelsius,
    pub(crate) consumed_current: i32,
    /// EWMA filter for ADC voltage readings.
    pub(crate) voltage_filter: crate::filter::EwmaPow2<3>,
    /// Multi-stage filter for ADC current readings.
    pub(crate) current_filter: crate::current::CurrentFilter,
}

impl Measurements {
    /// Read battery voltage.
    pub fn battery_voltage(&self) -> crate::units::MilliVolts {
        self.battery_voltage
    }

    /// Set battery voltage.
    pub fn set_battery_voltage(&mut self, v: crate::units::MilliVolts) {
        self.battery_voltage = v;
    }

    /// Read actual current.
    pub fn actual_current(&self) -> crate::units::MilliAmps {
        self.actual_current
    }

    /// Set actual current.
    pub fn set_actual_current(&mut self, v: crate::units::MilliAmps) {
        self.actual_current = v;
    }

    /// Read temperature in degrees Celsius.
    pub fn degrees_celsius(&self) -> crate::units::DegreesCelsius {
        self.degrees_celsius
    }
}

/// Main-loop timing state — eRPM and commutation interval tracking.
///
/// Owns the main-loop side of timing computation. ISR-owned timing
/// (commutation_interval, zero_crosses, e_com_time) lives in SharedComm.
#[derive(Clone, Default)]
pub struct TimingState {
    pub(crate) average_interval: u32,
    pub(crate) last_average_interval: u32,
    pub(crate) e_rpm: u16,
}

impl BemfState {
    /// Read BEMF counter value.
    pub fn counter(&self) -> u8 {
        self.counter
    }

    /// Read zero-cross found flag.
    pub fn zc_found(&self) -> bool {
        self.zc_found
    }

    /// Read filter level.
    pub fn filter_level(&self) -> u8 {
        self.filter_level
    }

    /// Read temp advance.
    pub fn temp_advance(&self) -> u8 {
        self.temp_advance
    }

    /// Set temp advance (e.g. from EEPROM advance_level).
    pub fn set_temp_advance(&mut self, v: u8) {
        self.temp_advance = v;
    }
}

impl TimingState {
    /// Read average interval.
    pub fn average_interval(&self) -> u32 {
        self.average_interval
    }

    /// Set average interval.
    pub fn set_average_interval(&mut self, v: u32) {
        self.average_interval = v;
    }

    /// Read last average interval.
    pub fn set_last_average_interval(&mut self, v: u32) {
        self.last_average_interval = v;
    }

    /// Read eRPM.
    pub fn e_rpm(&self) -> u16 {
        self.e_rpm
    }
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

impl DutyState {
    /// Read duty cycle.
    pub fn cycle(&self) -> u16 {
        self.cycle
    }

    /// Set duty cycle.
    pub fn set_cycle(&mut self, v: u16) {
        self.cycle = v;
    }

    /// Read last duty cycle.
    pub fn last(&self) -> u16 {
        self.last
    }

    /// Set last duty cycle.
    pub fn set_last(&mut self, v: u16) {
        self.last = v;
    }

    /// Read adjusted duty cycle.
    pub fn adjusted(&self) -> u16 {
        self.adjusted
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

impl ProtectionState {
    /// Set low voltage count (for testing/harness).
    pub fn set_low_voltage_count(&mut self, v: u16) {
        self.low_voltage_count = v;
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

#[cfg(test)]
mod tests {
    use super::*;

    // These tests mirror the C Catch2 tests in tests/test_tenkhz.cpp.

    /// Mirrors C: "tenKhzRoutine current limit PID at 1kHz"
    /// C setup: currentPid.Kp=100, actual_current=5000, target=2000,
    ///          use_current_limit_adjust=2000
    /// C assert: use_current_limit_adjust < 2000
    #[test]
    fn current_limit_pid_reduces_ceiling_when_over_target() {
        let mut pid = PidState::default();
        pid.set_use_current_limit(true);
        pid.set_current_gains(100, 0, 0);

        let actual_current = 5000i16; // mA — above target
        let target = 10 * 200; // config.current_limit=10 → target=2000
        let min_duty = 50; // minimum_duty_cycle.min(50) * 10

        let ceiling = pid.tick_current_limit(actual_current, target, min_duty, true);
        assert!(ceiling < 2000, "ceiling={ceiling}, expected < 2000");
    }

    /// Verify ceiling doesn't go below min_duty floor.
    #[test]
    fn current_limit_pid_clamps_to_min_duty() {
        let mut pid = PidState::default();
        pid.set_use_current_limit(true);
        pid.set_current_gains(10000, 0, 0); // aggressive gain

        let ceiling = pid.tick_current_limit(30000, 1000, 500, true);
        assert!(
            ceiling >= 500,
            "ceiling={ceiling}, expected >= 500 (min_duty)"
        );
    }

    /// When not running or not enabled, ceiling resets to 2000.
    #[test]
    fn current_limit_pid_resets_when_inactive() {
        let mut pid = PidState::default();
        pid.set_use_current_limit(true);
        pid.set_current_gains(100, 0, 0);

        // Drive ceiling down
        pid.tick_current_limit(5000, 2000, 50, true);
        let reduced = pid.tick_current_limit(5000, 2000, 50, true);
        assert!(reduced < 2000);

        // Not running → resets
        let reset = pid.tick_current_limit(5000, 2000, 50, false);
        assert_eq!(reset, 2000);
    }

    /// Mirrors C: "tenKhzRoutine stall protection PID"
    /// C setup: stallPid.Kp=1, commutation_interval=8000, target=6500,
    ///          stall_protection_adjust=0
    /// C assert: stall_protection_adjust > 0
    #[test]
    fn stall_pid_boosts_when_interval_above_target() {
        let mut pid = PidState::with_stall_target(6500);

        let boost = pid.tick_stall(8000); // ci=8000 > target=6500
        assert!(boost > 0, "boost={boost}, expected > 0");
    }

    /// Stall adjust is clamped to 0-150 range.
    #[test]
    fn stall_pid_clamps_to_150() {
        let mut pid = PidState::with_stall_target(100);

        // Many ticks with huge error to saturate
        for _ in 0..10000 {
            pid.tick_stall(50000);
        }
        let boost = pid.tick_stall(50000);
        assert!(boost <= 150, "boost={boost}, expected <= 150");
    }

    /// Stall adjust doesn't go negative.
    #[test]
    fn stall_pid_clamps_to_zero() {
        let mut pid = PidState::with_stall_target(10000);

        // ci below target → negative error
        let boost = pid.tick_stall(1000);
        assert_eq!(boost, 0, "boost should be 0 when ci < target");
    }

    /// Speed control returns None when not active.
    #[test]
    fn speed_control_inactive_returns_none() {
        let mut pid = PidState::default();
        assert!(pid.tick_speed_control(5000, 200, true).is_none());
    }

    /// Speed control returns None when not running.
    #[test]
    fn speed_control_not_running_returns_none() {
        let mut pid = PidState::default();
        pid.set_use_speed_control(true);
        assert!(pid.tick_speed_control(5000, 200, false).is_none());
    }

    /// Speed control returns override when active and running.
    #[test]
    fn speed_control_active_returns_override() {
        let mut pid = PidState::default();
        pid.set_use_speed_control(true);

        // With default speed PID gains (kp=10), e_com > target → positive output
        let result = pid.tick_speed_control(5000, 200, true);
        assert!(result.is_some());
        assert!(result.unwrap() > 0);
    }

    /// Speed control with low zero_crosses still produces output
    /// (it resets integral but proportional term still works).
    #[test]
    fn speed_control_works_during_startup() {
        let mut pid = PidState::default();
        pid.set_use_speed_control(true);

        // Low zero_crosses triggers integral reset each tick,
        // but proportional output should still accumulate
        let v1 = pid.tick_speed_control(5000, 50, true).unwrap();
        let v2 = pid.tick_speed_control(5000, 50, true).unwrap();
        assert!(v2 >= v1, "override should accumulate: v1={v1}, v2={v2}");
    }

    // --- DutyState tests ---

    #[test]
    fn set_duty_limits_applies_values() {
        let mut d = DutyState::default();
        d.set_duty_limits(100, 200, 300);
        assert_eq!(d.minimum, 100);
        assert_eq!(d.min_startup, 200);
        assert_eq!(d.startup_max, 300);
    }

    #[test]
    fn apply_dead_time_override_shifts_all_thresholds() {
        let mut d = DutyState::default();
        d.set_duty_limits(5, 120, 200);
        d.apply_dead_time_override(160);
        assert_eq!(d.minimum, 165);
        assert_eq!(d.min_startup, 280);
        assert_eq!(d.startup_max, 360);
    }

    #[test]
    fn apply_dead_time_override_zero_is_noop() {
        let mut d = DutyState::default();
        d.set_duty_limits(5, 120, 200);
        d.apply_dead_time_override(0);
        assert_eq!(d.minimum, 5);
        assert_eq!(d.min_startup, 120);
        assert_eq!(d.startup_max, 200);
    }

    // --- ProtectionState tests ---

    #[test]
    fn protection_getters_match_setters() {
        let mut p = ProtectionState::default();
        assert_eq!(p.bemf_timeout(), 10); // default
        p.set_bemf_timeout(20);
        assert_eq!(p.bemf_timeout(), 20);
        p.set_bemf_timeout_happened(5);
        assert_eq!(p.bemf_timeout_happened(), 5);
    }
}
