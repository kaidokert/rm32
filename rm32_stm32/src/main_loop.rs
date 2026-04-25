//! Main loop exclusive state and logic.
//!
//! This runs in thread mode (non-ISR). Accesses shared state via atomics,
//! owns protection/telemetry/config exclusively.

use rm32::config::EepromConfig;
use rm32::constants::*;
use rm32::control::state::{Measurements, ProtectionState, TelemetryState};
use rm32::current::CurrentFilter;
use rm32::filter::EwmaPow2;
use rm32::pid::Pid;
use rm32::functions::get_abs_dif;
use rm32::hal::{Adc, TelemetryUart};
use rm32::telemetry;

use crate::shared::SharedState;

/// Main-loop exclusive state.
pub struct MainState {
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
}

impl MainState {
    /// Main loop iteration. Reads shared atomics, updates main-exclusive state.
    pub fn tick(&mut self, shared: &SharedState, adc: &mut dyn Adc, telem: &mut dyn TelemetryUart) {
        // e_com_time calculation
        let sum: u32 = self.commutation_intervals.iter().map(|&v| v as u32).sum();
        let e_com_time = ((sum + 4) >> 1) as i32;
        shared.set_e_com_time(e_com_time);

        // Average interval
        self.average_interval = (e_com_time / 3) as u32;

        // BEMF timeout clearing — dynamic thresholds matching C
        let zc = shared.zero_crosses();
        let adj_input = shared.adjusted_input();
        if zc > 1000 || adj_input == 0 {
            self.protection.bemf_timeout_happened = 0;
        }
        if zc > 100 && adj_input < 200 {
            self.protection.bemf_timeout_happened = 0;
        }
        if self.config.use_sine_start != 0 && adj_input < 160 {
            self.protection.bemf_timeout_happened = 0;
        }
        // Dynamic BEMF timeout threshold: lenient at low throttle
        if adj_input < BEMF_LENIENT_THROTTLE {
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
                    shared.set_running(false);
                }
                shared.set_old_routine(true);
            }
            self.desync_check = false;
            self.last_average_interval = self.average_interval;
        }

        // Signal timeout
        if shared.signal_timeout() > 10000
            && shared.armed() {
                shared.set_armed(false);
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
            if self.measurements.battery_voltage < threshold && threshold > 0 {
                self.protection.low_voltage_count += 1;
            } else if !self.protection.low_voltage_cutoff {
                self.protection.low_voltage_count = 0;
            }
            let lvc_limit = if shared.stepper_sine() { LVC_STARTUP_THRESHOLD } else { LVC_NORMAL_THRESHOLD };
            if self.protection.low_voltage_count > lvc_limit {
                self.protection.low_voltage_cutoff = true;
                shared.set_armed(false);
                shared.set_running(false);
            }
        }

        // ADC measurements — typed conversions via AdcCount
        use rm32::units::AdcCount;
        let smoothed_v = AdcCount(self.voltage_filter.update(adc.raw_voltage()));
        let smoothed_c = AdcCount(self.current_filter.update(adc.raw_current()));
        self.measurements.battery_voltage = smoothed_v.to_millivolts(self.voltage_divider).0;
        self.measurements.actual_current = smoothed_c.to_milliamps(self.current_offset, self.millivolt_per_amp).0;
        self.measurements.degrees_celsius = adc.calc_temperature(adc.raw_temperature()).0;
        adc.start_conversion();

        // Publish measurements to shared state (ISR reads for EDT)
        shared.set_actual_current(self.measurements.actual_current);
        shared.set_battery_voltage(self.measurements.battery_voltage);
        shared.set_degrees_celsius(self.measurements.degrees_celsius);

        // Cell count auto-detection on arming transition
        let armed = shared.armed();
        self.just_armed = armed && !self.last_armed;
        if self.just_armed
            && self.cell_count == 0 && self.config.low_voltage_cut_off == 1 {
                self.cell_count = (self.measurements.battery_voltage / 370) as u8;
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

        // Speed control PID — closed-loop RPM control
        if self.use_speed_control_loop && shared.running() {
            let e_com = shared.e_com_time();
            self.speed_input_override += self.speed_pid.calculate(e_com, self.target_e_com_time as i32);
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
            let voltage_cv = self.measurements.battery_voltage / 10; // mV → centivolts
            let current_ca = (self.measurements.actual_current as u16) / 10; // mA → centiamps
            telemetry::make_telem_package(
                &mut pkt,
                self.measurements.degrees_celsius as i8,
                voltage_cv,
                current_ca,
                (self.measurements.consumed_current / 1000) as u16, // µAh → mAh
                self.e_rpm, // already in units of 100 eRPM (600000/e_com_time)
            );
            telem.send_dma(&pkt);
            shared.set_send_telemetry(false);
        }
    }
}
