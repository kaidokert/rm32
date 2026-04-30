//! Main loop exclusive state and logic.
//!
//! This runs in thread mode (non-ISR). Accesses shared state via atomics,
//! owns protection/telemetry/config exclusively.

use crate::config::EepromConfig;
use crate::constants::*;
use crate::control::state::{Measurements, ProtectionState, TelemetryState};
use crate::current::CurrentFilter;
use crate::filter::EwmaPow2;
use crate::functions::get_abs_dif;
use crate::hal::{Adc, TelemetryUart};
use crate::pid::Pid;
use crate::telemetry;
use embedded_hal::digital::OutputPin;

use crate::shared_state::SharedState;

/// Compute variable PWM auto-reload value for mode 1 (interval-scaled).
pub fn variable_pwm_mode1(commutation_interval: u32, timer1_max_arr: u16) -> u16 {
    let half = timer1_max_arr as i32 / 2;
    let full = timer1_max_arr as i32;
    let result = crate::functions::map(commutation_interval as i32, 96, 200, half, full);
    result.clamp(half, full) as u16
}

/// Compute variable PWM auto-reload value for mode 2 (CPU-scaled).
pub fn variable_pwm_mode2(average_interval: u32, cpu_mhz: u8) -> u16 {
    let scale = cpu_mhz as u32 / 9;
    if average_interval < 100 && average_interval > 0 {
        (100 * scale) as u16
    } else if average_interval >= 250 || average_interval == 0 {
        (250 * scale) as u16
    } else {
        (average_interval * scale) as u16
    }
}

/// Compute duty ceiling from eRPM and temperature limits.
/// Returns the more restrictive of the two (or 2000 if neither applies).
pub fn duty_ceiling(
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
    let poles = motor_poles.max(2) as i32;
    let low_rpm = motor_kv as i32 * poles / 3200;
    let high_rpm = motor_kv as i32 * poles / 384;
    let erpm_max = if k_erpm > 0 && high_rpm > low_rpm {
        crate::functions::map(k_erpm, low_rpm, high_rpm, 600, 2000).clamp(1, 2000) as u16
    } else {
        2000
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
pub struct MainState<LED: OutputPin = NoLed> {
    pub protection: ProtectionState,
    pub measurements: Measurements,
    pub telemetry: TelemetryState,
    pub config: EepromConfig,

    // PID controllers (main computes adjustments, ISR applies)
    pub current_pid: Pid,
    pub speed_pid: Pid,
    pub stall_pid: Pid,

    // Derived values
    pub e_rpm: u16,
    pub average_interval: u32,
    pub last_average_interval: u32,
    pub commutation_intervals: [u16; 6],
    pub cell_count: u8,
    pub motor_kv: u16,
    pub low_cell_volt_cutoff: u16,
    pub voltage_divider: u16,
    pub millivolt_per_amp: u16,
    pub current_offset: i16,
    pub stall_protection_adjust: i32,
    /// Current limit duty ceiling (adjusted by PID). 2000 = no limit.
    pub current_limit_adjust: i16,
    /// Whether current limiting is active (set by loadEEpromSettings).
    pub use_current_limit: bool,
    pub stall_protect_target_interval: u16,
    pub use_speed_control_loop: bool,
    pub speed_input_override: i32,
    pub target_e_com_time: u32,
    pub desync_check: bool,
    pub current_filter: CurrentFilter,
    pub voltage_filter: EwmaPow2<3>,
    pub last_armed: bool,
    /// Set on the tick when arming transition happens
    pub just_armed: bool,
    /// Use external NTC thermistor instead of internal temp sensor
    pub use_ntc: bool,
    /// Custom LED pin (NoLed if board has no custom LED)
    pub led: LED,
    pub led_counter: u16,
    /// TIM1 max auto-reload (from PWM frequency config)
    pub timer1_max_arr: u16,
    /// CPU MHz for variable PWM mode 2 scaling
    pub cpu_mhz: u8,
    /// Main-loop tick counter for consumed current accumulation
    pub ten_khz_counter: u32,
}

impl<LED: OutputPin> MainState<LED> {
    /// Main loop iteration. Reads shared atomics, updates main-exclusive state.
    pub fn tick(&mut self, shared: &SharedState, adc: &mut dyn Adc, telem: &mut dyn TelemetryUart) {
        // e_com_time calculation
        let sum: u32 = self.commutation_intervals.iter().map(|&v| v as u32).sum();
        let e_com_time = ((sum + 4) >> 1) as i32;
        shared.set_e_com_time(e_com_time);

        // Average interval
        self.average_interval = (e_com_time / 3) as u32;

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
        if shared.interval_timer_count() > BEMF_STALL_TIMER_THRESHOLD && shared.running() {
            // Only increment if not already latched (102 = confirmed stuck)
            if self.protection.bemf_timeout_happened != BEMF_FAULT_LATCHED {
                self.protection.bemf_timeout_happened =
                    self.protection.bemf_timeout_happened.saturating_add(1);
            }
            shared.set_old_routine(true);
            if shared.adjusted_input() < THROTTLE_MIN_SIGNAL {
                shared.transition(crate::motor_mode::MotorEvent::StopMotor);
                shared.set_commutation_interval(DESYNC_RESET_INTERVAL);
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
                self.last_average_interval as i32,
                self.average_interval as i32,
            );
            if diff > (self.average_interval >> 1) && self.average_interval < DESYNC_MAX_INTERVAL {
                // Reset interval to 5000 if motor was running (>100 ZCs)
                // Check before zeroing zero_crosses (C has this after, which is a bug)
                if zc > 100 {
                    self.average_interval = DESYNC_RESET_INTERVAL;
                }
                shared.set_zero_crosses(0);
                self.protection.desync_happened += 1;
                if (self.config.bi_direction == 0 && shared.adjusted_input() > 47)
                    || shared.commutation_interval() > 1000
                {
                    shared.transition(crate::motor_mode::MotorEvent::StopMotor);
                }
                shared.transition(crate::motor_mode::MotorEvent::DesyncFallback);
            }
            self.desync_check = false;
            self.last_average_interval = self.average_interval;
        }

        // Signal timeout
        if shared.signal_timeout() > crate::constants::SIGNAL_TIMEOUT_DISARM && shared.armed() {
            shared.transition(crate::motor_mode::MotorEvent::Disarm);
            shared.set_input_set(false);
        }

        // eRPM
        if !shared.stepper_sine() && e_com_time > 0 {
            self.e_rpm = if shared.running() {
                (600000 / e_com_time) as u16
            } else {
                0
            };
        }

        // Low voltage cutoff
        // Stepper sine (startup) uses fast 0.1s timeout; normal uses 10s
        if self.config.low_voltage_cut_off != 0 {
            let threshold = self.cell_count as u16 * self.low_cell_volt_cutoff;
            if self.measurements.battery_voltage.0 < threshold && threshold > 0 {
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
                shared.transition(crate::motor_mode::MotorEvent::Disarm);
            }
        }

        // ADC measurements — typed conversions via AdcCount
        use crate::units::AdcCount;
        let smoothed_v = AdcCount(self.voltage_filter.update(adc.raw_voltage()));
        let smoothed_c = AdcCount(self.current_filter.update(adc.raw_current()));
        self.measurements.battery_voltage = smoothed_v.to_millivolts(self.voltage_divider);
        self.measurements.actual_current =
            smoothed_c.to_milliamps(self.current_offset, self.millivolt_per_amp);
        self.measurements.degrees_celsius = if self.use_ntc {
            crate::ntc::ntc_degrees(adc.raw_temperature())
        } else {
            adc.calc_temperature(adc.raw_temperature())
        };
        adc.start_conversion();

        // Publish measurements to shared state (ISR reads for EDT)
        shared.set_actual_current(self.measurements.actual_current.0);
        shared.set_battery_voltage(self.measurements.battery_voltage.0);
        shared.set_degrees_celsius(self.measurements.degrees_celsius.0);

        // Cell count auto-detection on arming transition
        let armed = shared.armed();
        self.just_armed = armed && !self.last_armed;
        if self.just_armed && self.cell_count == 0 && self.config.low_voltage_cut_off == 1 {
            self.cell_count = (self.measurements.battery_voltage.0 / 370) as u8;
        }
        self.last_armed = armed;

        // Stall protection PID — boosts duty at low RPM for crawlers/RC cars
        if self.config.stall_protection != 0 && shared.running() {
            let ci = shared.commutation_interval() as i32;
            let target = self.stall_protect_target_interval as i32;
            self.stall_protection_adjust += self.stall_pid.calculate(ci, target);
            self.stall_protection_adjust = self.stall_protection_adjust.clamp(0, 150 * 10000);
            // Publish to ISR via shared state (ISR adds to duty)
            shared.set_stall_protection_adjust((self.stall_protection_adjust / 10000) as u16);
        }

        // Current limit PID — reduces duty when current exceeds limit
        if self.use_current_limit && shared.running() {
            let target = self.config.current_limit as i32 * 200;
            let adj = self
                .current_pid
                .calculate(self.measurements.actual_current.0 as i32, target)
                / 10000;
            self.current_limit_adjust -= adj as i16;
            self.current_limit_adjust = self
                .current_limit_adjust
                .clamp(self.config.minimum_duty_cycle as i16 * 10, 2000);
            shared.set_current_limit_adjust(self.current_limit_adjust as u16);
        } else {
            // Reset ceiling when inactive to prevent stale cap on next start
            self.current_limit_adjust = 2000;
            shared.set_current_limit_adjust(2000);
        }

        // Speed control PID — closed-loop RPM control
        if self.use_speed_control_loop && shared.running() {
            let e_com = shared.e_com_time();
            self.speed_input_override += self
                .speed_pid
                .calculate(e_com, self.target_e_com_time as i32);
            self.speed_input_override = self.speed_input_override.clamp(0, 2047 * 10000);
            if shared.zero_crosses() < 100 {
                self.speed_pid.integral = 0;
            }
            // Override throttle input with PID output
            let override_input = (self.speed_input_override / 10000) as u16;
            shared.set_newinput(override_input.clamp(48, 2047));
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
                (self.measurements.consumed_current / 1000) as u16, // µAh → mAh
                self.e_rpm, // already in units of 100 eRPM (600000/e_com_time)
            );
            telem.send_dma(&pkt);
            shared.set_send_telemetry(false);
        }

        // Consumed current accumulation (1s interval at ~20kHz)
        // TODO: counter incremented in main loop (variable rate), not ISR.
        // Matches C firmware behavior but integration is approximate.
        self.ten_khz_counter += 1;
        if self.ten_khz_counter > 20000 {
            self.measurements.consumed_current += self.measurements.actual_current.0 as i32;
            self.ten_khz_counter = 0;
        }

        // Variable PWM — adjust tim1_arr based on commutation speed
        if self.config.variable_pwm == 1 {
            shared.set_tim1_arr(variable_pwm_mode1(
                shared.commutation_interval(),
                self.timer1_max_arr,
            ));
        } else if self.config.variable_pwm == 2 {
            shared.set_tim1_arr(variable_pwm_mode2(self.average_interval, self.cpu_mhz));
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

        // Min BEMF counts adjustment — more lenient during startup
        if zc < 5 {
            let counts = if self.config.bi_direction != 0 { 3 } else { 4 };
            shared.set_min_bemf_counts(counts);
        } else {
            shared.set_min_bemf_counts(2);
        }

        // Filter level — dynamic based on motor speed
        let filter = if zc < 100 && shared.commutation_interval() > 500 {
            12u8
        } else if shared.commutation_interval() < 50 {
            2
        } else {
            crate::functions::map(self.average_interval as i32, 100, 500, 3, 12) as u8
        };
        shared.set_filter_level(filter);

        // Auto advance — scales with duty cycle
        if self.config.auto_advance != 0 {
            let level =
                crate::functions::map(shared.duty_cycle_setpoint() as i32, 100, 2000, 13, 23) as u8;
            shared.set_auto_advance(level);
        }

        // Note: send_esc_info_flag is checked and cleared by firmware main.rs
        // after sending the actual packet. MainState does not own this flag.

        // Custom LED: blink with throttle, solid when high
        {
            let input = shared.adjusted_input();
            self.led_counter = self.led_counter.wrapping_add(1);
            if (47..1947).contains(&input) {
                if self.led_counter > crate::constants::LED_BLINK_HALF_PERIOD {
                    let _ = self.led.set_high();
                } else {
                    let _ = self.led.set_low();
                }
                if self.led_counter > crate::constants::LED_BLINK_HALF_PERIOD * 2 {
                    self.led_counter = 0;
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

    // --- Variable PWM mode 2 ---

    #[test]
    fn vpwm2_clamps_low() {
        // avg < 100 → floor at 100 * scale
        assert_eq!(variable_pwm_mode2(50, 64), (100 * (64 / 9)) as u16);
    }

    #[test]
    fn vpwm2_clamps_high() {
        // avg >= 250 → ceiling at 250 * scale
        assert_eq!(variable_pwm_mode2(300, 64), (250 * (64 / 9)) as u16);
    }

    #[test]
    fn vpwm2_scales_mid() {
        // 100 <= avg < 250 → avg * scale
        assert_eq!(variable_pwm_mode2(150, 64), (150 * (64 / 9)) as u16);
    }

    #[test]
    fn vpwm2_zero_interval_clamps_high() {
        assert_eq!(variable_pwm_mode2(0, 64), (250 * (64 / 9)) as u16);
    }

    // --- Variable PWM mode 1 ---

    #[test]
    fn vpwm1_fast_interval() {
        let arr = variable_pwm_mode1(96, 1999);
        assert_eq!(arr, 999); // maps to max_arr/2
    }

    #[test]
    fn vpwm1_slow_interval() {
        let arr = variable_pwm_mode1(200, 1999);
        assert_eq!(arr, 1999); // maps to max_arr
    }

    // --- Duty ceiling ---

    #[test]
    fn duty_ceiling_no_limits() {
        assert_eq!(duty_ceiling(0, 2000, 14, 25, 80), 2000);
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

    struct MockAdc;
    impl MockAdc {
        fn new() -> Self {
            Self
        }
    }
    impl crate::hal::Adc for MockAdc {
        fn start_conversion(&mut self) {}
        fn raw_voltage(&self) -> u16 {
            0
        }
        fn raw_current(&self) -> u16 {
            0
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
        MainState {
            protection: crate::control::state::ProtectionState::default(),
            measurements: crate::control::state::Measurements::default(),
            telemetry: crate::control::state::TelemetryState::default(),
            config: crate::config::EepromConfig::default(),
            current_pid: crate::pid::Pid::new(400, 0, 1000, 20000, 100000),
            speed_pid: crate::pid::Pid::new(10, 0, 100, 10000, 50000),
            stall_pid: crate::pid::Pid::new(1, 0, 50, 10000, 50000),
            e_rpm: 0,
            average_interval: 0,
            last_average_interval: 0,
            commutation_intervals: [0; 6],
            cell_count: 0,
            motor_kv: 2000,
            low_cell_volt_cutoff: 330,
            voltage_divider: 110,
            millivolt_per_amp: 20,
            current_offset: 0,
            stall_protection_adjust: 0,
            current_limit_adjust: 2000,
            use_current_limit: false,
            stall_protect_target_interval: 6500,
            use_speed_control_loop: false,
            speed_input_override: 0,
            target_e_com_time: 0,
            desync_check: false,
            current_filter: crate::current::CurrentFilter::new(),
            voltage_filter: crate::filter::EwmaPow2::new(),
            last_armed: false,
            just_armed: false,
            use_ntc: false,
            led: NoLed,
            led_counter: 0,
            timer1_max_arr: 1999,
            cpu_mhz: 64,
            ten_khz_counter: 0,
        }
    }

    #[test]
    fn stall_detection_increments_timeout() {
        use crate::motor_mode::MotorMode;
        use crate::shared_state::SharedState;

        let shared = SharedState::new();
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

        let shared = SharedState::new();
        shared.set_motor_mode(MotorMode::OldRoutine);
        shared.set_interval_timer_count(40000); // below 45000

        let mut main = make_test_main_state();
        main.tick(&shared, &mut MockAdc::new(), &mut MockTelem);
        assert_eq!(main.protection.bemf_timeout_happened, 0);
    }
}
