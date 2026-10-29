//! Main loop exclusive state and logic.
//!
//! This runs in thread mode (non-ISR). Accesses shared state via atomics,
//! owns protection/telemetry/config exclusively.

use crate::config::EepromConfig;
use crate::constants::*;
use crate::control::state::{Measurements, PidState, ProtectionState, TimingState};
use crate::functions::get_abs_dif;
use crate::hal::{Adc, TelemetryUart};
use crate::shared_comm::IsrAction;
use crate::telemetry;
use crate::units::AdcCount;
use embedded_hal::digital::OutputPin;

use crate::shared_state::SharedState;

/// Compute variable PWM auto-reload value for mode 1 (interval-scaled).
pub(crate) fn variable_pwm_mode1<P: Timing>(commutation_interval: u32, timer1_max_arr: u16) -> u16 {
    let half = timer1_max_arr as i32 / 2;
    let full = timer1_max_arr as i32;
    let result = crate::functions::map(
        commutation_interval as i32,
        P::VARIABLE_PWM1_LO as i32,
        P::VARIABLE_PWM1_HI as i32,
        half,
        full,
    );
    result.clamp(half, full) as u16
}

/// Compute variable PWM auto-reload value for mode 2 (CPU-scaled).
pub(crate) fn variable_pwm_mode2<P: Timing>(average_interval: u32, cpu_mhz: u8) -> u16 {
    let bounded = if average_interval == 0 {
        P::VARIABLE_PWM2_HI
    } else {
        average_interval.clamp(P::VARIABLE_PWM2_LO, P::VARIABLE_PWM2_HI)
    };
    // PWM geometry uses the original 2 MHz interval scale, independent of
    // the port's interval timer. Bound first to keep very long intervals safe.
    let average_interval = (bounded as u64 * 2_000_000 / P::TIMER_HZ as u64) as u32;
    let scale = cpu_mhz as u32 / 9;
    (average_interval * scale) as u16
}

/// Compute duty ceiling from eRPM and temperature limits.
/// Returns the more restrictive of the two (or 2000 if neither applies).
pub(crate) fn duty_ceiling(
    e_com_time: i32,
    motor_kv: u16,
    motor_poles: u8,
    degrees_celsius: i16,
    temperature_limit: u8,
) -> u16 {
    let k_erpm = if e_com_time > 0 {
        (600000 / e_com_time) / 10
    } else {
        0
    };
    let poles = (motor_poles as i32).max(1);
    let erpm_max = if motor_kv < 300 {
        2000
    } else {
        let div = (32 / poles).max(1);
        let low_rpm = motor_kv as i32 / 100 / div;
        let high_rpm = motor_kv as i32 / 12 / div;
        if k_erpm > 0 && high_rpm > low_rpm {
            crate::functions::map(k_erpm, low_rpm, high_rpm, 400, 2000).clamp(1, 2000) as u16
        } else {
            2000
        }
    };

    let temp_max = if degrees_celsius > temperature_limit as i16 {
        crate::functions::map(
            degrees_celsius as i32,
            temperature_limit as i32 - 10,
            temperature_limit as i32 + 10,
            1000,
            1,
        )
        .clamp(1, 2000) as u16
    } else {
        2000
    };

    erpm_max.min(temp_max)
}

/// Marker type for boards without a custom LED.
pub struct NoLed;
impl OutputPin for NoLed {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
impl embedded_hal::digital::ErrorType for NoLed {
    type Error = core::convert::Infallible;
}

/// Main-loop exclusive state, generic over optional LED pin.
pub struct MainState<P: Timing, LED: OutputPin = NoLed> {
    _platform: core::marker::PhantomData<P>,
    pub protection: ProtectionState,
    pub(crate) measurements: Measurements<P>,
    pub config: EepromConfig,

    // PID controllers (main computes adjustments, ISR applies)
    pub(crate) pid: PidState,

    // Timing (main-loop side — ISR timing is in SharedComm)
    pub(crate) timing: TimingState,
    // Board-level hardware constants (set once from BoardConfig, never change)
    pub(crate) voltage_divider_x100: u16,
    pub(crate) millivolt_per_amp: u16,
    pub(crate) current_offset: i16,
    pub(crate) use_ntc: bool,
    /// CPU MHz for variable PWM mode 2 scaling
    pub(crate) cpu_mhz: u8,
    pub cell_count: u8,
    pub(crate) motor_kv: u16,
    pub(crate) minimum_duty: u16,
    pub(crate) target_min_bemf_counts: u8,
    pub(crate) low_cell_volt_cutoff: u16,
    pub(crate) desync_check: bool,
    pub(crate) last_armed: bool,
    /// Set on the tick when arming transition happens
    pub just_armed: bool,
    /// Set when signal timeout handling requests a firmware reset.
    pub needs_reset: bool,
    /// Custom LED pin (NoLed if board has no custom LED)
    pub(crate) led: LED,
    /// Position within the 200 ms LED period, in control ticks.
    pub(crate) led_phase: u32,
    /// TIM1 max auto-reload (from PWM frequency config, overwritten by apply_motor_config)
    pub(crate) timer1_max_arr: u16,
    /// Last observed control clock; None anchors the first main service.
    last_control_tick: Option<u32>,
}

/// MCU-specific constants — properties of the silicon, not the board PCB.
#[derive(Clone, Copy, Debug)]
pub struct ChipParams {
    /// TIM1 auto-reload at default PWM frequency.
    pub timer1_max_arr: u16,
    /// CPU frequency in MHz.
    pub cpu_mhz: u8,
}

impl<P: Timing> MainState<P, NoLed> {
    /// Construct MainState from board config and chip constants.
    ///
    /// Used by both firmware and harness to ensure identical initialization.
    /// PID tuning, motor_kv, and other EEPROM-derived values are applied
    /// later via `apply_motor_config()`.
    pub fn new(board: &crate::board::BoardConfig, chip: ChipParams) -> Self {
        Self::new_with_led(board, chip, NoLed)
    }
}

impl<P: Timing, LED: OutputPin> MainState<P, LED> {
    /// Construct main-loop state with an optional custom status LED output.
    pub fn new_with_led(board: &crate::board::BoardConfig, chip: ChipParams, led: LED) -> Self {
        assert!(
            board.min_bemf_counts <= u8::MAX / 2,
            "min_bemf_counts must fit derived startup thresholds"
        );
        let () = P::VALID;
        Self {
            _platform: core::marker::PhantomData,
            protection: ProtectionState::default(),
            measurements: Measurements::default(),
            config: EepromConfig::default(),
            pid: PidState::with_stall_target(board.stall_protect_interval),
            timing: TimingState::default(),
            timer1_max_arr: chip.timer1_max_arr,
            voltage_divider_x100: board.voltage_divider_x100,
            millivolt_per_amp: board.millivolt_per_amp,
            current_offset: board.current_offset,
            use_ntc: board.use_ntc,
            cpu_mhz: chip.cpu_mhz,
            cell_count: 0,
            motor_kv: 2000,
            minimum_duty: 0,
            target_min_bemf_counts: board.min_bemf_counts,
            low_cell_volt_cutoff: 330,
            desync_check: false,
            last_armed: false,
            just_armed: false,
            needs_reset: false,
            led,
            led_phase: 0,
            last_control_tick: None,
        }
    }
}

impl<P: Timing, LED: OutputPin> MainState<P, LED> {
    /// Read-only access to measurements.
    pub fn measurements(&self) -> &Measurements<P> {
        &self.measurements
    }

    /// Read-only access to timing state.
    pub fn timing(&self) -> &TimingState {
        &self.timing
    }

    /// Mutable access to timing state.
    pub fn timing_mut(&mut self) -> &mut TimingState {
        &mut self.timing
    }

    /// Read-only access to PID state.
    pub fn pid(&self) -> &PidState {
        &self.pid
    }

    // --- Harness config injection setters ---

    /// Set use_current_limit on the PID controller.
    pub fn set_use_current_limit(&mut self, v: bool) {
        self.pid.set_use_current_limit(v);
    }

    /// Set use_speed_control on the PID controller.
    pub fn set_use_speed_control(&mut self, v: bool) {
        self.pid.set_use_speed_control(v);
    }

    /// Set average_interval on timing state.
    pub fn set_average_interval(&mut self, v: u32) {
        self.timing.set_average_interval(v);
    }

    /// Set last_average_interval on timing state.
    pub fn set_last_average_interval(&mut self, v: u32) {
        self.timing.set_last_average_interval(v);
    }

    /// Set battery_voltage measurement.
    pub fn set_battery_voltage(&mut self, v: crate::units::CentiVolts) {
        self.measurements.set_battery_voltage(v);
    }

    /// Set actual_current measurement.
    pub fn set_actual_current(&mut self, v: crate::units::CentiAmps) {
        self.measurements.set_actual_current(v);
    }

    /// Read motor_kv.
    pub fn motor_kv(&self) -> u16 {
        self.motor_kv
    }

    /// Set motor_kv.
    pub fn set_motor_kv(&mut self, v: u16) {
        self.motor_kv = v;
    }

    /// Read desync_check flag.
    pub fn desync_check(&self) -> bool {
        self.desync_check
    }

    /// Set desync_check flag.
    pub fn set_desync_check(&mut self, v: bool) {
        self.desync_check = v;
    }

    /// Apply EEPROM-derived motor configuration.
    ///
    /// Called after loading config from flash (firmware) or after `load_eeprom`
    /// command (harness). Updates PID tuning, motor KV, voltage cutoff, and
    /// current limit flag from the derived `MotorConfig`.
    pub fn apply_motor_config(&mut self, motor_cfg: &crate::config::MotorConfig) {
        self.pid.set_current_gains(
            motor_cfg.current_kp,
            motor_cfg.current_ki,
            motor_cfg.current_kd,
        );
        self.motor_kv = motor_cfg.motor_kv;
        self.minimum_duty = motor_cfg.minimum_duty;
        self.low_cell_volt_cutoff = motor_cfg.low_cell_volt_cutoff;
        self.timer1_max_arr = motor_cfg.timer1_max_arr;
        self.pid.set_use_current_limit(
            self.config.current_limit > 0 && self.config.current_limit < 100,
        );
    }

    /// Main loop iteration. Reads shared atomics, updates main-exclusive state.
    pub(crate) fn tick(
        &mut self,
        shared: &SharedState,
        adc: &mut dyn Adc,
        telem: &mut dyn TelemetryUart,
    ) {
        // One clock snapshot for all periodic work. Hold the previous observed
        // current through this interval BEFORE the ADC path replaces it below.
        // Delayed servicing retains elapsed time but cannot recover unobserved
        // current changes. Service at least once per full u32 clock wrap.
        let now = shared.control_ticks();
        let elapsed = self
            .last_control_tick
            .map_or(0, |last| now.wrapping_sub(last));
        self.last_control_tick = Some(now);
        self.measurements
            .consumed_charge
            .accumulate(self.measurements.actual_current, elapsed);
        let led_period = P::LED_BLINK_HALF_PERIOD as u32 * 2;
        if elapsed != 0 {
            let phase = self.led_phase + elapsed % led_period;
            self.led_phase = if phase >= led_period {
                phase - led_period
            } else {
                phase
            };
        }

        // e_com_time: read from SharedComm (ISR computes from per-step intervals)
        let e_com_time = shared.e_com_time();

        // Average interval
        self.timing.average_interval =
            crate::platform::e_com_us_to_avg_ticks::<P>(e_com_time) as u32;

        // BEMF timeout clearing — check whether the user has released the throttle.
        // For unidirectional: newinput == 0 means stick centered.
        // For bidirectional: newinput near SERVO_CENTER means stick centered.
        // process_input zeros adjusted_input on latch, so we can't use it directly.
        let zc = shared.zero_crosses();
        let raw_input = shared.newinput();
        let stick_released = if self.config.bi_direction != 0 && shared.dshot() {
            // DShot bidir: zero means no throttle (commands are 1-47)
            raw_input == 0
        } else if self.config.bi_direction != 0 {
            // Servo bidir: dead band around center means stick released
            let db = (self.config.servo_dead_band as u16) << 1;
            let center = crate::constants::SERVO_CENTER;
            raw_input >= center.saturating_sub(db) && raw_input <= center + db
        } else {
            raw_input == 0
        };
        if zc > 1000 || stick_released {
            self.protection.bemf_timeout_happened = 0;
        }
        if zc > 100 && raw_input < 200 && !(self.config.bi_direction != 0 && shared.dshot()) {
            // Skip for DShot bidir: raw_input 48-199 is active reverse throttle
            self.protection.bemf_timeout_happened = 0;
        }
        if self.config.use_sine_start != 0
            && raw_input < crate::constants::SINE_BEMF_CLEAR_THROTTLE
            && !(self.config.bi_direction != 0 && shared.dshot())
        {
            self.protection.bemf_timeout_happened = 0;
        }
        // Stall detection: if interval timer exceeds threshold, motor has stalled.
        // C: if (INTERVAL_TIMER_COUNT > 45000 && running == 1)
        if shared.interval_timer_count() > P::BEMF_STALL_TIMER_THRESHOLD && shared.running() {
            // Only increment if not already latched (102 = confirmed stuck)
            if self.protection.bemf_timeout_happened != BEMF_FAULT_LATCHED {
                self.protection.bemf_timeout_happened =
                    self.protection.bemf_timeout_happened.saturating_add(1);
            }
            shared.set_old_routine(true);
            if shared.adjusted_input() < THROTTLE_MIN_SIGNAL {
                shared.request_isr_action(IsrAction::ResetIntervalTimer);
                shared.transition(crate::motor_mode::MotorEvent::StopMotor);
                shared.set_commutation_interval(P::DESYNC_RESET_INTERVAL);
            } else {
                shared.request_isr_action(IsrAction::CommutateKick);
            }
            shared.set_zero_crosses(0);
        }

        // Dynamic BEMF timeout threshold: lenient at low throttle
        if raw_input < BEMF_LENIENT_THROTTLE {
            self.protection.bemf_timeout = BEMF_TIMEOUT_LENIENT;
        } else {
            self.protection.bemf_timeout = BEMF_TIMEOUT_STRICT;
        }

        // Desync detection
        if self.desync_check && zc > 10 {
            let diff = get_abs_dif(
                self.timing.last_average_interval as i32,
                self.timing.average_interval as i32,
            );
            if diff > (self.timing.average_interval >> 1)
                && self.timing.average_interval < P::DESYNC_MAX_INTERVAL
            {
                // Reset interval to 5000 if motor was running (>100 ZCs)
                // Check before zeroing zero_crosses (C has this after, which is a bug)
                if zc > 100 {
                    self.timing.average_interval = P::DESYNC_RESET_INTERVAL;
                }
                shared.set_zero_crosses(0);
                if (self.config.bi_direction == 0 && shared.adjusted_input() > 47)
                    || shared.commutation_interval() > P::DESYNC_STOP_INTERVAL
                {
                    shared.transition(crate::motor_mode::MotorEvent::StopMotor);
                }
                shared.transition(crate::motor_mode::MotorEvent::DesyncFallback);
            }
            self.desync_check = false;
            self.timing.last_average_interval = self.timing.average_interval;
        }

        // Signal timeout thresholds fire only after the counter exceeds the limit.
        let signal_timeout = shared.signal_timeout();
        if shared.armed() {
            if signal_timeout > P::SIGNAL_TIMEOUT_DISARM {
                shared.transition(crate::motor_mode::MotorEvent::Disarm);
                shared.set_input_set(false);
                self.needs_reset = true;
            }
        } else if shared.input_set() && signal_timeout > P::SIGNAL_TIMEOUT_UNARMED {
            shared.set_input_set(false);
            self.needs_reset = true;
        }

        // eRPM
        if !shared.stepper_sine() && e_com_time > 0 {
            self.timing.e_rpm = if shared.running() {
                (600000 / e_com_time) as u16
            } else {
                0
            };
        }

        // Cell count auto-detection on arming transition
        let armed = shared.armed();
        self.just_armed = armed && !self.last_armed;
        if self.just_armed && self.cell_count == 0 && self.config.low_voltage_cut_off == 1 {
            self.cell_count = (self.measurements.battery_voltage.0 / 370) as u8;
        }
        self.last_armed = armed;

        if shared.one_khz_counter_check_and_reset(P::PID_LOOP_DIVIDER) {
            // ADC measurements — typed conversions via AdcCount
            let smoothed_v = AdcCount(self.measurements.voltage_filter.update(adc.raw_voltage()));
            let smoothed_c = AdcCount(self.measurements.current_filter.update(adc.raw_current()));
            self.measurements.battery_voltage = smoothed_v.to_centivolts(self.voltage_divider_x100);
            self.measurements.actual_current =
                smoothed_c.to_centiamps(self.current_offset, self.millivolt_per_amp);
            self.measurements.degrees_celsius = if self.use_ntc {
                crate::ntc::ntc_degrees(adc.raw_temperature())
            } else {
                adc.calc_temperature(adc.raw_temperature())
            };
            adc.start_conversion();

            // Publish measurements to shared state (ISR reads for EDT)
            shared.set_actual_current(self.measurements.actual_current);
            shared.set_battery_voltage(self.measurements.battery_voltage);
            shared.set_degrees_celsius(self.measurements.degrees_celsius.0);

            // Low voltage cutoff
            if self.config.low_voltage_cut_off != 0 {
                let below_cutoff = if self.config.low_voltage_cut_off == 2 {
                    let threshold = self.config.absolute_cutoff_centivolts();
                    threshold.0 > 0 && self.measurements.battery_voltage < threshold
                } else {
                    self.measurements
                        .battery_voltage
                        .below_cell_threshold(self.cell_count, self.low_cell_volt_cutoff)
                };
                if below_cutoff {
                    self.protection.low_voltage_count += 1;
                } else if !self.protection.low_voltage_cutoff {
                    self.protection.low_voltage_count = 0;
                }
                let lvc_limit = if shared.stepper_sine() {
                    LVC_STARTUP_THRESHOLD
                } else {
                    LVC_NORMAL_THRESHOLD
                };
                if self.protection.low_voltage_count > lvc_limit {
                    self.protection.low_voltage_cutoff = true;
                    shared.request_isr_action(IsrAction::AllOff);
                    shared.transition(crate::motor_mode::MotorEvent::Disarm);
                }
            }

            // Stall protection PID — boosts duty at low RPM for crawlers/RC cars
            if self.config.stall_protection != 0 && shared.running() {
                // Board targets and PID gains retain their established 0.5 us
                // units; changing the timer clock must not retune the controller.
                let interval =
                    (shared.commutation_interval() as u64 * 2_000_000 / P::TIMER_HZ as u64) as i32;
                let boost = self.pid.tick_stall(interval);
                shared.set_stall_protection_adjust(boost);
            }

            // Current limit PID — reduces duty when current exceeds limit
            {
                let target = self.config.current_limit as i32 * 200;
                let min_duty = self.minimum_duty.min(i16::MAX as u16) as i16;
                let ceiling = self.pid.tick_current_limit(
                    self.measurements.actual_current.0,
                    target,
                    min_duty,
                    shared.running(),
                );
                shared.set_current_limit_adjust(ceiling);
            }

            // Speed control PID — closed-loop RPM control
            if let Some(override_input) =
                self.pid
                    .tick_speed_control(shared.e_com_time(), zc, shared.running())
            {
                shared.set_newinput(override_input.clamp(48, 2047));
            }
        }

        // Telemetry send
        if shared.send_telemetry() {
            let mut pkt = [0u8; 10];
            let voltage_cv = self.measurements.battery_voltage.to_centivolts();
            let current_ca = self.measurements.actual_current.to_centiamps();
            telemetry::make_telem_package(
                &mut pkt,
                self.measurements.degrees_celsius.to_i8(),
                voltage_cv,
                current_ca,
                self.measurements.consumed_milliamp_hours(),
                self.timing.e_rpm, // already in units of 100 eRPM (600000/e_com_time)
            );
            telem.send_dma(&pkt);
            shared.set_send_telemetry(false);
        }

        // Variable PWM — adjust tim1_arr based on commutation speed
        if self.config.variable_pwm == 1 {
            shared.set_tim1_arr(variable_pwm_mode1::<P>(
                shared.commutation_interval(),
                self.timer1_max_arr,
            ));
        } else if self.config.variable_pwm == 2 {
            shared.set_tim1_arr(variable_pwm_mode2::<P>(
                self.timing.average_interval,
                self.cpu_mhz,
            ));
        } else {
            // variable_pwm=0: publish the EEPROM-derived ARR so ISR uses it
            shared.set_tim1_arr(self.timer1_max_arr);
        }

        // eRPM + temperature duty ceiling
        shared.set_duty_maximum(duty_ceiling(
            e_com_time,
            self.motor_kv,
            self.config.motor_poles,
            self.measurements.degrees_celsius.0,
            self.config.temperature_limit,
        ));

        // Require stronger BEMF confirmation during startup.
        if zc < 5 {
            let counts = if self.config.bi_direction != 0 {
                self.target_min_bemf_counts + 1
            } else {
                self.target_min_bemf_counts * 2
            };
            shared.set_min_bemf_counts(counts);
        } else {
            shared.set_min_bemf_counts(self.target_min_bemf_counts);
        }

        // Filter level — dynamic based on motor speed
        let filter = if zc < 100 && shared.commutation_interval() > P::FILTER_SLOW_INTERVAL {
            12u8
        } else if shared.commutation_interval() < P::FILTER_FAST_INTERVAL {
            2
        } else {
            crate::functions::map(
                self.timing.average_interval as i32,
                P::FILTER_MAP_LO_INTERVAL as i32,
                P::FILTER_MAP_HI_INTERVAL as i32,
                3,
                12,
            ) as u8
        };
        shared.set_filter_level(filter);

        // Auto advance scales with duty; otherwise publish the static EEPROM
        // advance mapping used by AM32.
        if self.config.auto_advance != 0 {
            let level =
                crate::functions::map(shared.duty_cycle_setpoint() as i32, 100, 2000, 13, 23) as u8;
            shared.set_auto_advance(level);
        } else {
            shared.set_auto_advance(self.config.temp_advance());
        }

        // Note: send_esc_info_flag is checked and cleared by firmware main.rs
        // after sending the actual packet. MainState does not own this flag.

        // Custom LED: exact 100 ms low/high phases, continuing while off or
        // solid. Modulo phase retains missed periods without service-time drift.
        {
            let input = shared.adjusted_input();
            if (47..1947).contains(&input) {
                if self.led_phase >= P::LED_BLINK_HALF_PERIOD as u32 {
                    let _ = self.led.set_high();
                } else {
                    let _ = self.led.set_low();
                }
            } else if input > crate::constants::LED_HIGH_THROTTLE {
                let _ = self.led.set_high();
            } else {
                let _ = self.led.set_low();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    type MainState<LED = NoLed> = super::MainState<crate::platform::TestPlatform, LED>;

    // --- Variable PWM mode 2 ---

    #[test]
    fn vpwm2_clamps_low() {
        // avg < 100 → floor at 100 * scale
        assert_eq!(
            variable_pwm_mode2::<crate::platform::TestPlatform>(50, 64),
            (100 * (64 / 9)) as u16
        );
    }

    #[test]
    fn vpwm2_clamps_high() {
        // avg >= 250 → ceiling at 250 * scale
        assert_eq!(
            variable_pwm_mode2::<crate::platform::TestPlatform>(300, 64),
            (250 * (64 / 9)) as u16
        );
    }

    #[test]
    fn vpwm2_scales_mid() {
        // 100 <= avg < 250 → avg * scale
        assert_eq!(
            variable_pwm_mode2::<crate::platform::TestPlatform>(150, 64),
            (150 * (64 / 9)) as u16
        );
    }

    #[test]
    fn vpwm2_zero_interval_clamps_high() {
        assert_eq!(
            variable_pwm_mode2::<crate::platform::TestPlatform>(0, 64),
            (250 * (64 / 9)) as u16
        );
    }

    // --- Variable PWM mode 1 ---

    #[test]
    fn vpwm1_fast_interval() {
        let arr = variable_pwm_mode1::<crate::platform::TestPlatform>(96, 1999);
        assert_eq!(arr, 999); // maps to max_arr/2
    }

    #[test]
    fn vpwm1_slow_interval() {
        let arr = variable_pwm_mode1::<crate::platform::TestPlatform>(200, 1999);
        assert_eq!(arr, 1999); // maps to max_arr
    }

    // --- Duty ceiling ---

    #[test]
    fn duty_ceiling_no_limits() {
        assert_eq!(duty_ceiling(0, 2000, 14, 25, 80), 2000);
    }

    #[test]
    fn duty_ceiling_matches_am32_rpm_limit_shape() {
        assert_eq!(duty_ceiling(1446, 2220, 14, 25, 141), 992);
        assert_eq!(duty_ceiling(600, 2220, 14, 25, 141), 2000);
    }

    #[test]
    fn duty_ceiling_skips_rpm_limit_for_low_kv_motors() {
        assert_eq!(duty_ceiling(1446, 280, 14, 25, 141), 2000);
    }

    #[test]
    fn duty_ceiling_temp_reduces() {
        let dc = duty_ceiling(0, 2000, 14, 85, 80);
        assert!(dc < 2000, "expected reduced duty, got {}", dc);
    }

    #[test]
    fn duty_ceiling_high_poles_no_panic() {
        // motor_poles > 32 must not divide by zero
        let dc = duty_ceiling(1000, 2000, 40, 25, 80);
        assert!(dc > 0);
    }

    #[test]
    fn duty_ceiling_takes_minimum() {
        // Both limits active → should return the lower one
        let dc = duty_ceiling(100, 2000, 14, 85, 80);
        assert!(dc < 2000);
    }

    // --- Stall detection (BEMF timeout increment) ---

    const ADC_FULL_SCALE: u16 = 4095;

    struct MockAdc {
        raw_voltage: u16,
        raw_current: u16,
    }
    impl MockAdc {
        fn new() -> Self {
            Self::with_raw(0, 0)
        }

        fn with_raw(raw_voltage: u16, raw_current: u16) -> Self {
            Self {
                raw_voltage,
                raw_current,
            }
        }

        fn with_raw_voltage(raw_voltage: u16) -> Self {
            Self::with_raw(raw_voltage, 0)
        }

        fn with_raw_current(raw_current: u16) -> Self {
            Self::with_raw(0, raw_current)
        }
    }
    impl crate::hal::Adc for MockAdc {
        fn start_conversion(&mut self) {}
        fn raw_voltage(&self) -> u16 {
            self.raw_voltage
        }
        fn raw_current(&self) -> u16 {
            self.raw_current
        }
        fn raw_temperature(&self) -> u16 {
            0
        }
        fn calc_temperature(&self, _: u16) -> crate::units::DegreesCelsius {
            crate::units::DegreesCelsius(25)
        }
    }

    struct MockTelem;
    impl crate::hal::TelemetryUart for MockTelem {
        fn send_dma(&mut self, _: &[u8]) {}
    }

    fn make_test_main_state() -> MainState {
        MainState::new(
            &crate::board::BoardConfig::DEFAULT,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 64,
            },
        )
    }

    #[test]
    #[should_panic(expected = "min_bemf_counts must fit derived startup thresholds")]
    fn rejects_min_bemf_counts_that_overflow_startup_thresholds() {
        let board = crate::board::BoardConfig {
            min_bemf_counts: 128,
            ..crate::board::BoardConfig::DEFAULT
        };
        let _ = MainState::new(
            &board,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 64,
            },
        );
    }

    fn trip_one_khz(shared: &SharedState) {
        for _ in 0..<crate::platform::TestPlatform as Timing>::PID_LOOP_DIVIDER {
            shared.one_khz_counter_inc();
        }
    }

    #[derive(Default)]
    struct CapturedTelemetry([u8; 10]);

    impl crate::hal::TelemetryUart for CapturedTelemetry {
        fn send_dma(&mut self, packet: &[u8]) {
            self.0.copy_from_slice(packet);
        }
    }

    #[derive(Default)]
    struct TestLed(bool);

    impl embedded_hal::digital::ErrorType for TestLed {
        type Error = core::convert::Infallible;
    }
    impl OutputPin for TestLed {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0 = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0 = true;
            Ok(())
        }
    }

    #[test]
    fn accounting_regression_main_calls_without_time_do_not_consume_charge() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        main.set_actual_current(crate::units::CentiAmps(1000)); // 10 A
        let mut telem = CapturedTelemetry::default();
        for _ in 0..20001 {
            main.tick(&shared, &mut MockAdc::new(), &mut telem);
        }
        shared.set_send_telemetry(true);
        main.tick(&shared, &mut MockAdc::new(), &mut telem);
        assert_eq!(u16::from_be_bytes([telem.0[5], telem.0[6]]), 0);
    }

    #[test]
    fn accounting_regression_main_calls_without_time_do_not_blink() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_adjusted_input(100);
        let mut main = MainState::new_with_led(
            &crate::board::BoardConfig::DEFAULT,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 64,
            },
            TestLed::default(),
        );
        for _ in 0..2001 {
            main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        }
        assert!(
            !main.led.0,
            "LED phase must not advance without control ticks"
        );
    }

    fn advance_control_clock(shared: &SharedState, ticks: u32) {
        for _ in 0..ticks {
            shared.increment_control_ticks();
        }
    }

    fn send_charge(main: &mut MainState, shared: &SharedState) -> u16 {
        let mut telem = CapturedTelemetry::default();
        shared.set_send_telemetry(true);
        main.tick(shared, &mut MockAdc::new(), &mut telem);
        u16::from_be_bytes([telem.0[5], telem.0[6]])
    }

    #[test]
    fn charge_first_service_anchors_without_backfilling_startup_time() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        advance_control_clock(&shared, 72000);
        let mut main = make_test_main_state();
        main.set_actual_current(crate::units::CentiAmps(100));
        assert_eq!(send_charge(&mut main, &shared), 0);
        advance_control_clock(&shared, 72000);
        assert_eq!(send_charge(&mut main, &shared), 1); // 1 A * 3.6 s
        assert_eq!(send_charge(&mut main, &shared), 1);
    }

    #[test]
    fn charge_and_kiss_are_independent_of_main_service_cadence() {
        for interval in [1, 20, 2000, 72000] {
            let shared = SharedState::new::<crate::platform::TestPlatform>();
            let mut main = make_test_main_state();
            main.set_actual_current(crate::units::CentiAmps(100));
            main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
            for _ in 0..72000 / interval {
                advance_control_clock(&shared, interval);
                main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
            }
            assert_eq!(send_charge(&mut main, &shared), 1, "interval={interval}");
        }
    }

    #[test]
    fn charge_holds_previous_sample_until_adc_observation() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        main.set_actual_current(crate::units::CentiAmps(100)); // 1 A held
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        // Make the next ADC observation exactly 165 A after current filtering.
        for _ in 0..50 {
            main.measurements.current_filter.update(4095);
        }
        advance_control_clock(&shared, 72000); // 3.6 s at the prior 1 A
        trip_one_khz(&shared);
        shared.set_send_telemetry(true);
        let mut telem = CapturedTelemetry::default();
        main.tick(&shared, &mut MockAdc::with_raw_current(4095), &mut telem);
        assert_eq!(u16::from_be_bytes([telem.0[5], telem.0[6]]), 1);
        assert_eq!(main.measurements.actual_current.0, 16500);
        advance_control_clock(&shared, 72000); // next 3.6 s at 165 A
        assert_eq!(send_charge(&mut main, &shared), 166);
    }

    #[test]
    fn charge_wrap_negative_current_and_wire_saturation() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        // 72000 elapsed ticks across a wrap: 1 A * 3.6 s = 1 mAh.
        main.last_control_tick = Some(0u32.wrapping_sub(72000));
        main.set_actual_current(crate::units::CentiAmps(100));
        assert_eq!(send_charge(&mut main, &shared), 1);
        main.set_actual_current(crate::units::CentiAmps(-100));
        advance_control_clock(&shared, 72000);
        assert_eq!(send_charge(&mut main, &shared), 1);
        main.set_actual_current(crate::units::CentiAmps(i16::MAX));
        main.last_control_tick = Some(shared.control_ticks().wrapping_add(1));
        assert_eq!(send_charge(&mut main, &shared), u16::MAX); // largest valid gap
        advance_control_clock(&shared, 20);
        assert_eq!(send_charge(&mut main, &shared), u16::MAX);
    }

    fn make_led_main() -> MainState<TestLed> {
        MainState::new_with_led(
            &crate::board::BoardConfig::DEFAULT,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 64,
            },
            TestLed::default(),
        )
    }

    #[test]
    fn led_exact_phase_boundaries_and_delayed_service() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_adjusted_input(100);
        let mut main = make_led_main();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        let mut previous = 0;
        for (ticks, high) in [
            (1999, false),
            (2000, true),
            (3999, true),
            (4000, false),
            (402001, true),
            (404000, false),
        ] {
            advance_control_clock(&shared, ticks - previous);
            main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
            assert_eq!(main.led.0, high, "tick={ticks}");
            previous = ticks;
        }
    }

    #[test]
    fn led_wrap_and_throttle_gates_preserve_continuous_phase() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_led_main();
        main.led_phase = 1999;
        main.last_control_tick = Some(u32::MAX - 1999);
        shared.set_adjusted_input(100);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert!(main.led.0); // 3999 ticks into the period after clock wrap
        shared.increment_control_ticks();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert!(!main.led.0); // exact period boundary
        // Phase advances even while throttle selects solid/off output.
        shared.set_adjusted_input(2000);
        advance_control_clock(&shared, 2000);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        for (input, high) in [
            (0, false),
            (46, false),
            (47, true),
            (1946, true),
            (1947, false),
            (1948, true),
        ] {
            shared.set_adjusted_input(input);
            main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
            assert_eq!(main.led.0, high, "input={input}");
        }
        shared.set_adjusted_input(0);
        advance_control_clock(&shared, 6000);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        shared.set_adjusted_input(100);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert!(!main.led.0);
    }

    #[test]
    fn measurement_regression_adc_to_kiss_voltage() {
        // A 3.3 V ADC pin behind the default 11:1 divider means 36.30 V.
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        let mut telem = CapturedTelemetry::default();
        shared.set_send_telemetry(true);
        trip_one_khz(&shared);
        main.tick(&shared, &mut MockAdc::with_raw_voltage(4095), &mut telem);
        assert_eq!(u16::from_be_bytes([telem.0[1], telem.0[2]]), 3630);
    }

    #[test]
    fn measurement_regression_adc_to_edt_voltage() {
        // EDT carries quarter-volts: 36.30 V truncates to 145 * 0.25 V.
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        trip_one_khz(&shared);
        main.tick(
            &shared,
            &mut MockAdc::with_raw_voltage(4095),
            &mut MockTelem,
        );
        let mut edt = crate::edt::EdtScheduler::default();
        edt.request_init();
        edt.next_frame(
            crate::units::CentiAmps::ZERO,
            crate::units::CentiVolts::ZERO,
            0,
        );
        let mut frame = crate::edt::EdtFrame::Erpm;
        for _ in 0..100 {
            frame = edt.next_frame(crate::units::CentiAmps::ZERO, shared.battery_voltage(), 25);
        }
        assert!(matches!(frame, crate::edt::EdtFrame::Extended(0x491)));
    }

    #[test]
    fn measurement_regression_adc_to_kiss_current() {
        // 3.3 V / (20 mV/A) = 165 A, or 16500 centiamps on KISS.
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        for _ in 0..50 {
            main.measurements.current_filter.update(4095);
        }
        let mut telem = CapturedTelemetry::default();
        shared.set_send_telemetry(true);
        trip_one_khz(&shared);
        main.tick(&shared, &mut MockAdc::with_raw_current(4095), &mut telem);
        assert_eq!(u16::from_be_bytes([telem.0[3], telem.0[4]]), 16500);
    }

    #[test]
    fn measurement_regression_adc_to_edt_current() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = make_test_main_state();
        for _ in 0..50 {
            main.measurements.current_filter.update(4095);
        }
        trip_one_khz(&shared);
        main.tick(
            &shared,
            &mut MockAdc::with_raw_current(4095),
            &mut MockTelem,
        );
        let mut edt = crate::edt::EdtScheduler::default();
        edt.request_init();
        edt.next_frame(
            crate::units::CentiAmps::ZERO,
            crate::units::CentiVolts::ZERO,
            0,
        );
        let mut frame = crate::edt::EdtFrame::Erpm;
        for _ in 0..40 {
            frame = edt.next_frame(shared.actual_current(), crate::units::CentiVolts::ZERO, 25);
        }
        assert!(matches!(frame, crate::edt::EdtFrame::Extended(0x6A5)));
    }

    #[test]
    fn measurement_regression_absolute_cutoff_half_volt_setting() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(crate::motor_mode::MotorMode::OldRoutine);
        let mut main = make_test_main_state();
        main.config.low_voltage_cut_off = 2;
        main.config.absolute_voltage_cutoff = 25; // 12.5 V
        main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);
        trip_one_khz(&shared);
        // One third of ADC full-scale: 1.1 V * 11 = 12.1 V, below cutoff.
        main.tick(
            &shared,
            &mut MockAdc::with_raw_voltage(1365),
            &mut MockTelem,
        );
        assert!(!shared.armed());
    }

    #[test]
    fn adc_cutoff_boundaries_preserve_strict_comparison_and_disabled_threshold() {
        // A 10:1 divider makes ADC=1365 exactly 11 V; 1229 truncates to 9.90 V.
        for (mode, absolute, raw, should_trip) in [
            (2, 22, 1364, true),
            (2, 22, 1365, false),
            (2, 22, 1367, false),
            (2, 0, 0, false),
            (0, 22, 0, false),
            (1, 0, 1228, true),
            (1, 0, 1229, false),
            (1, 0, 1230, false),
        ] {
            let shared = SharedState::new::<crate::platform::TestPlatform>();
            shared.set_motor_mode(crate::motor_mode::MotorMode::OldRoutine);
            let mut main = make_test_main_state();
            main.voltage_divider_x100 = 1000;
            main.config.low_voltage_cut_off = mode;
            main.config.absolute_voltage_cutoff = absolute;
            main.cell_count = 3;
            main.low_cell_volt_cutoff = 330;
            main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);
            trip_one_khz(&shared);
            main.tick(&shared, &mut MockAdc::with_raw_voltage(raw), &mut MockTelem);
            assert_eq!(!shared.armed(), should_trip, "mode={mode}, raw={raw}");
        }
    }

    #[test]
    fn cell_count_uses_nominal_cell_centivolts() {
        for (cv, cells) in [(369, 0), (370, 1), (1109, 2), (1110, 3), (1680, 4)] {
            let shared = SharedState::new::<crate::platform::TestPlatform>();
            shared.set_motor_mode(crate::motor_mode::MotorMode::OldRoutine);
            let mut main = make_test_main_state();
            main.config.low_voltage_cut_off = 1;
            main.set_battery_voltage(crate::units::CentiVolts(cv));
            main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
            assert_eq!(main.cell_count, cells);
        }
    }

    #[test]
    fn stall_detection_increments_timeout() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine); // running=true
        shared.set_interval_timer_count(50000); // > 45000 threshold
        shared.set_adjusted_input(100); // above throttle min

        let mut main = make_test_main_state();
        assert_eq!(main.protection.bemf_timeout_happened, 0);

        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert!(
            main.protection.bemf_timeout_happened > 0,
            "bemf_timeout_happened should increment on stall"
        );
    }

    #[test]
    fn stall_detection_does_not_trigger_below_threshold() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);
        shared.set_interval_timer_count(40000); // below 45000

        let mut main = make_test_main_state();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert_eq!(main.protection.bemf_timeout_happened, 0);
    }

    #[test]
    fn current_limit_uses_derived_minimum_duty() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);

        let mut main = make_test_main_state();
        main.config.minimum_duty_cycle = 5;
        main.config.current_limit = 1;
        main.config.current_p = 100;
        main.config.rc_car_reverse = 1;
        main.config.normalize_after_load();

        let motor_cfg = main.config.derive_motor_config(1999, 60, 1, false);
        main.apply_motor_config(&motor_cfg);

        let mut adc = MockAdc::with_raw_current(4095);
        for _ in 0..1000 {
            trip_one_khz(&shared);
            main.tick(&shared, &mut adc, &mut MockTelem);
        }

        assert_eq!(shared.current_limit_adjust(), motor_cfg.minimum_duty);
    }

    #[test]
    fn min_bemf_counts_are_stricter_during_unidirectional_startup() {
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_zero_crosses(4);

        let mut main = make_test_main_state();
        main.config.bi_direction = 0;
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert_eq!(shared.min_bemf_counts(), 6);
    }

    #[test]
    fn min_bemf_counts_are_stricter_during_bidirectional_startup() {
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_zero_crosses(4);

        let mut main = make_test_main_state();
        main.config.bi_direction = 1;
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert_eq!(shared.min_bemf_counts(), 4);
    }

    #[test]
    fn min_bemf_counts_use_target_after_startup() {
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_zero_crosses(5);

        let mut main = make_test_main_state();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert_eq!(shared.min_bemf_counts(), 3);
    }

    #[test]
    fn min_bemf_counts_follow_configured_target() {
        use crate::shared_state::SharedState;

        let board = crate::board::BoardConfig {
            min_bemf_counts: 5,
            ..crate::board::BoardConfig::DEFAULT
        };
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_zero_crosses(4);

        let mut main = MainState::new(
            &board,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 64,
            },
        );

        main.config.bi_direction = 0;
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert_eq!(shared.min_bemf_counts(), 10);

        main.config.bi_direction = 1;
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert_eq!(shared.min_bemf_counts(), 6);

        shared.set_zero_crosses(5);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert_eq!(shared.min_bemf_counts(), 5);
    }

    #[test]
    fn lvc_mode1_per_cell_triggers_disarm_and_all_off() {
        use crate::motor_mode::MotorMode;
        use crate::shared_comm::MainControl;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);

        let mut main = make_test_main_state();
        main.config.low_voltage_cut_off = 1;
        main.cell_count = 3;
        main.low_cell_volt_cutoff = 330;
        main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);

        trip_one_khz(&shared);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert!(!shared.armed());
        assert_eq!(MainControl::isr_action(&shared), IsrAction::AllOff);
    }

    #[test]
    fn lvc_does_not_run_before_one_khz_dispatch() {
        use crate::motor_mode::MotorMode;
        use crate::shared_comm::MainControl;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);

        let mut main = make_test_main_state();
        main.config.low_voltage_cut_off = 1;
        main.cell_count = 3;
        main.low_cell_volt_cutoff = 330;
        main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);

        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert!(shared.armed());
        assert_eq!(MainControl::isr_action(&shared), IsrAction::None);
    }

    #[test]
    fn lvc_mode2_absolute_cutoff() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);

        let mut main = make_test_main_state();
        main.config.low_voltage_cut_off = 2;
        main.config.absolute_voltage_cutoff = 20; // 10 V
        main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);

        trip_one_khz(&shared);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert!(!shared.armed());
    }

    #[test]
    fn lvc_mode2_above_threshold_does_not_disarm() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);

        let mut main = make_test_main_state();
        main.config.low_voltage_cut_off = 2;
        main.config.absolute_voltage_cutoff = 20; // 10 V
        main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);

        trip_one_khz(&shared);
        main.tick(
            &shared,
            &mut MockAdc::with_raw_voltage(ADC_FULL_SCALE),
            &mut MockTelem,
        );

        assert!(shared.armed());
    }

    #[test]
    fn lvc_latch_inhibits_counter_recovery() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);

        let mut main = make_test_main_state();
        main.config.low_voltage_cut_off = 2;
        main.config.absolute_voltage_cutoff = 20; // 10 V
        main.protection.set_low_voltage_count(LVC_NORMAL_THRESHOLD);

        trip_one_khz(&shared);
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert!(!shared.armed());
        let latched_count = main.protection.low_voltage_count();
        assert!(latched_count > LVC_NORMAL_THRESHOLD);

        // Let the voltage filter settle above the physical 10 V threshold.
        for _ in 0..50 {
            main.measurements.voltage_filter.update(ADC_FULL_SCALE);
        }
        shared.set_motor_mode(MotorMode::OldRoutine);
        trip_one_khz(&shared);
        main.tick(
            &shared,
            &mut MockAdc::with_raw_voltage(ADC_FULL_SCALE),
            &mut MockTelem,
        );

        assert_eq!(main.protection.low_voltage_count(), latched_count);
        assert!(!shared.armed());
    }

    #[test]
    fn signal_timeout_armed_requests_reset() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::OldRoutine);
        shared.set_input_set(true);
        for _ in 0..=<crate::platform::TestPlatform as Timing>::SIGNAL_TIMEOUT_DISARM {
            shared.increment_signal_timeout();
        }

        let mut main = make_test_main_state();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert!(!shared.armed());
        assert!(!shared.input_set());
        assert!(main.needs_reset);
    }

    #[test]
    fn signal_timeout_unarmed_resets_input_and_requests_reset() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::Disarmed);
        shared.set_input_set(true);
        for _ in 0..=<crate::platform::TestPlatform as Timing>::SIGNAL_TIMEOUT_UNARMED {
            shared.increment_signal_timeout();
        }

        let mut main = make_test_main_state();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert!(!shared.input_set());
        assert!(main.needs_reset);
    }

    #[test]
    fn signal_timeout_unarmed_without_prior_input_does_not_reset() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.set_motor_mode(MotorMode::Disarmed);
        assert!(!shared.input_set());
        for _ in 0..=<crate::platform::TestPlatform as Timing>::SIGNAL_TIMEOUT_UNARMED {
            shared.increment_signal_timeout();
        }

        let mut main = make_test_main_state();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);

        assert!(!main.needs_reset);
    }
}
