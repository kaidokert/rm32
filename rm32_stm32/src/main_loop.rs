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
use rm32::functions::{get_abs_dif, map};
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
        if shared.signal_timeout() > 10000 {
            if shared.armed() {
                shared.set_armed(false);
                shared.set_input_set(false);
            }
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

        // ADC measurements
        let raw_v = adc.raw_voltage();
        let raw_c = adc.raw_current();
        let raw_t = adc.raw_temperature();
        let smoothed_v = self.voltage_filter.update(raw_v);
        self.measurements.battery_voltage =
            (smoothed_v as u32 * 3300 / 4095 * self.voltage_divider as u32 / 100) as u16;
        // C formula: actual_current = ((smoothed * 3300/41) - (CURRENT_OFFSET * 100)) / MILLIVOLT_PER_AMP
        let smoothed_c = self.current_filter.update(raw_c);
        let current_mv = (smoothed_c as i32) * 3300 / 41 - (self.current_offset as i32) * 100;
        self.measurements.actual_current = if self.millivolt_per_amp > 0 {
            (current_mv / self.millivolt_per_amp as i32) as i16
        } else {
            0
        };
        self.measurements.degrees_celsius = adc.calc_temperature(raw_t);
        adc.start_conversion();

        // Publish measurements to shared state (ISR reads for EDT)
        shared.set_actual_current(self.measurements.actual_current);
        shared.set_battery_voltage(self.measurements.battery_voltage);
        shared.set_degrees_celsius(self.measurements.degrees_celsius);

        // Cell count auto-detection on arming transition
        let armed = shared.armed();
        self.just_armed = armed && !self.last_armed;
        if self.just_armed {
            if self.cell_count == 0 && self.config.low_voltage_cut_off == 1 {
                self.cell_count = (self.measurements.battery_voltage / 370) as u8;
            }
        }
        self.last_armed = armed;

        // Stall protection PID — boosts duty at low RPM for crawlers/RC cars
        if self.config.stall_protection != 0 && shared.running() {
            let ci = shared.commutation_interval() as i32;
            let target = self.stall_protect_target_interval as i32;
            self.stall_protection_adjust += self.stall_pid.calculate(ci, target);
            self.stall_protection_adjust = self.stall_protection_adjust.clamp(0, 150 * 10000);
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
