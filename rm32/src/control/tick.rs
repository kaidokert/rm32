//! Control loop tick functions.
//!
//! These are the core methods that run the motor control state machine,
//! equivalent to tenKhzRoutine, main_loop, interruptRoutine, etc.

use crate::control::state::*;
use crate::functions::{get_abs_dif, map};
use crate::hal;

/// Hardware interface passed to control loop ticks.
/// Combines all HAL traits needed by the control loop into one bound.
pub trait ControlHal:
    hal::PwmOutput
    + hal::Comparator
    + hal::PhaseOutput
    + hal::IntervalTimer
    + hal::ComTimer
    + hal::System
{
}
impl<T> ControlHal for T where
    T: hal::PwmOutput
        + hal::Comparator
        + hal::PhaseOutput
        + hal::IntervalTimer
        + hal::ComTimer
        + hal::System
{
}

impl MotorState {
    /// Start motor if not already running. Equivalent to C `startMotor()`.
    pub fn start_motor(&mut self, hal: &mut impl ControlHal) {
        if !self.running {
            self.commutate(hal);
            self.timing.commutation_interval = crate::constants::INITIAL_COMMUTATION_INTERVAL;
            hal.set_count(5000);
            self.running = true;
        }
        hal.enable_interrupts();
    }

    /// Advance commutation one step. Equivalent to C `commutate()`.
    pub fn commutate(&mut self, hal: &mut impl ControlHal) {
        let step = self.commutation.advance();

        hal.disable_irq();
        if !self.prop_brake_active {
            hal.com_step(step);
        }
        hal.enable_irq();
        hal.change_input();

        if self.timing.average_interval > self.timing.polling_mode_changeover + 500 {
            self.old_routine = true;
        }

        self.bemf.counter = 0;
        self.bemf.zc_found = false;
        self.timing.commutation_intervals[(step - 1) as usize] =
            self.timing.commutation_interval as u16;
    }

    /// Process a zero-cross interrupt. Equivalent to C `interruptRoutine()`.
    /// Returns true if the zero-cross was accepted (passed filter).
    pub fn interrupt_routine(&mut self, hal: &mut impl ControlHal) -> bool {
        // Filter: check comparator multiple times
        for _ in 0..self.bemf.filter_level {
            if hal.output_level() == self.commutation.rising {
                return false; // false alarm
            }
        }

        hal.disable_irq();
        hal.mask_interrupts();
        self.bemf.last_zc_time = self.bemf.this_zc_time;
        self.bemf.this_zc_time = hal.count() as u16;
        hal.set_count(0);
        hal.set_and_enable(self.bemf.wait_time + 1);
        hal.enable_irq();
        true
    }

    /// Commutation timer callback. Equivalent to C `PeriodElapsedCallback()`.
    pub fn period_elapsed_callback(&mut self, hal: &mut impl ControlHal) {
        hal.disable_interrupt(); // disable COM timer interrupt
        self.commutate(hal);

        let zc_avg = (self.bemf.last_zc_time as u32 + self.bemf.this_zc_time as u32) >> 1;
        self.timing.commutation_interval = (self.timing.commutation_interval + zc_avg) >> 1;

        let advance = if self.config.auto_advance == 0 {
            (self.timing.commutation_interval * self.bemf.temp_advance as u32)
                >> crate::constants::ADVANCE_SHIFT
        } else {
            (self.timing.commutation_interval * self.bemf.auto_advance_level as u32)
                >> crate::constants::ADVANCE_SHIFT
        };
        self.bemf.advance = advance as u16;

        self.bemf.wait_time =
            (self.timing.commutation_interval as u16 >> 1).wrapping_sub(advance as u16);

        if !self.old_routine {
            hal.enable_interrupts();
        }
        if self.timing.zero_crosses < 10000 {
            self.timing.zero_crosses += 1;
        }
    }

    /// Process throttle input. Equivalent to C `setInput()`.
    pub fn set_input(&mut self, hal: &mut impl ControlHal) {
        // --- Bidirectional throttle mapping ---
        if self.config.bi_direction != 0 {
            if !self.input.dshot {
                // Servo bidirectional
                if self.config.rc_car_reverse != 0 {
                    self.set_input_servo_rc_car(hal);
                } else {
                    self.set_input_servo_bidir(hal);
                }
            } else {
                // DShot bidirectional
                if self.config.rc_car_reverse != 0 {
                    self.set_input_dshot_rc_car();
                } else {
                    self.set_input_dshot_bidir(hal);
                }
            }
        } else {
            self.input.adjusted = self.input.newinput;
        }

        // --- BEMF timeout protection ---
        if self.protection.bemf_timeout_happened > self.protection.bemf_timeout
            && self.config.stuck_rotor_protection != 0
        {
            hal.all_off();
            hal.mask_interrupts();
            self.input.input = 0;
            self.protection.bemf_timeout_happened = 102;
            return;
        }

        // --- Map adjusted_input to input ---
        if self.config.use_sine_start != 0 {
            let changeover = (self.config.sine_mode_changeover_throttle_level as u16) * 20;
            if self.input.adjusted < 30 {
                self.input.input = 0;
            } else if self.input.adjusted > 30 && self.input.adjusted < changeover {
                self.input.input =
                    map(self.input.adjusted as i32, 30, changeover as i32, 47, 160) as u16;
            } else if self.input.adjusted >= changeover {
                self.input.input = map(
                    self.input.adjusted as i32,
                    changeover as i32,
                    2047,
                    160,
                    2047,
                ) as u16;
            }
        } else if !self.pid.use_speed_control {
            self.input.input = self.input.adjusted;
        }

        // --- Motor start/stop/brake logic ---
        let threshold = 47 + (80 * self.config.use_sine_start as u16);
        if !self.stepper_sine && self.armed {
            if self.input.input >= threshold {
                if !self.running {
                    hal.all_off();
                    if !self.old_routine {
                        self.start_motor(hal);
                    }
                    self.running = true;
                    self.duty.last = self.duty.min_startup;
                }
                self.duty.setpoint = if self.config.use_sine_start != 0 {
                    map(
                        self.input.input as i32,
                        137,
                        2047,
                        self.duty.minimum as i32 + 40,
                        2000,
                    ) as u16
                } else {
                    map(
                        self.input.input as i32,
                        47,
                        2047,
                        self.duty.minimum as i32,
                        2000,
                    ) as u16
                };
                if self.config.rc_car_reverse == 0 {
                    self.prop_brake_active = false;
                }
            } else {
                // Low input path
                if self.play_tone_flag != 0 {
                    self.play_tone_flag = 0; // consumed (actual playback done by caller)
                }

                if self.config.comp_pwm == 0 {
                    self.duty.setpoint = 0;
                    if !self.running {
                        self.old_routine = true;
                        self.timing.zero_crosses = 0;
                        if self.config.brake_on_stop != 0 {
                            hal.full_brake();
                        } else if !self.prop_brake_active {
                            hal.all_off();
                        }
                    }
                } else {
                    // comp_pwm path
                    if !self.running {
                        self.old_routine = true;
                        self.timing.zero_crosses = 0;
                        if self.config.brake_on_stop > 0 {
                            if self.config.use_sine_start == 0 && self.config.brake_on_stop == 1 {
                                self.prop_brake_duty_cycle =
                                    self.config.drag_brake_strength as u16 * 200;
                                if self.prop_brake_duty_cycle >= 1999 {
                                    hal.full_brake();
                                } else {
                                    hal.proportional_brake();
                                    self.prop_brake_active = true;
                                }
                            }
                        } else {
                            hal.all_off();
                        }
                        self.duty.setpoint = 0;
                    }
                    if self.config.use_sine_start == 1 {
                        self.stepper_sine = true;
                    }
                    self.duty.setpoint = 0;
                }
            }

            // --- Startup duty floor/ceiling ---
            if !self.prop_brake_active {
                if self.input.input >= 47
                    && self.timing.zero_crosses < (30u32 >> self.config.stall_protection)
                {
                    if self.duty.setpoint < self.duty.min_startup {
                        self.duty.setpoint = self.duty.min_startup;
                    }
                    if self.duty.setpoint > self.duty.startup_max {
                        self.duty.setpoint = self.duty.startup_max;
                    }
                }
                if self.duty.setpoint > self.duty.maximum {
                    self.duty.setpoint = self.duty.maximum;
                }
                if self.pid.use_current_limit
                    && self.duty.setpoint > self.pid.current_limit_adjust as u16
                {
                    self.duty.setpoint = self.pid.current_limit_adjust as u16;
                }
            }
        }
    }

    // --- Bidirectional sub-methods ---

    fn set_input_dshot_bidir(&mut self, hal: &mut impl ControlHal) {
        let reversing_dead_band = 1u16;
        if self.input.newinput > 1047 {
            if self.commutation.forward == (self.config.dir_reversed != 0) {
                // Wrong direction: try to reverse
                if (self.timing.commutation_interval > self.reverse_speed_threshold as u32
                    && (self.duty.cycle < 200))
                    || self.stepper_sine
                {
                    self.commutation.forward = self.config.dir_reversed == 0;
                    self.timing.zero_crosses = 0;
                    self.old_routine = true;
                    hal.mask_interrupts();
                } else {
                    self.input.newinput = 0;
                }
            }
            self.input.adjusted = ((self.input.newinput.saturating_sub(1048)) * 2 + 47)
                .saturating_sub(reversing_dead_band);
        } else if self.input.newinput <= 1047 && self.input.newinput > 47 {
            if self.commutation.forward == (self.config.dir_reversed == 0) {
                if (self.timing.commutation_interval > self.reverse_speed_threshold as u32
                    && (self.duty.cycle < 200))
                    || self.stepper_sine
                {
                    self.timing.zero_crosses = 0;
                    self.old_routine = true;
                    self.commutation.forward = self.config.dir_reversed != 0;
                    hal.mask_interrupts();
                } else {
                    self.input.newinput = 0;
                }
            }
            self.input.adjusted = ((self.input.newinput.saturating_sub(48)) * 2 + 47)
                .saturating_sub(reversing_dead_band);
        } else {
            self.input.adjusted = 0;
        }
    }

    fn set_input_dshot_rc_car(&mut self) {
        let reversing_dead_band = 1u16;
        if self.input.newinput > 1047 {
            if self.commutation.forward == (self.config.dir_reversed != 0) {
                self.input.adjusted = 0;
                self.prop_brake_active = true;
                if self.return_to_center {
                    self.commutation.forward = self.config.dir_reversed == 0;
                    self.prop_brake_active = false;
                    self.return_to_center = false;
                }
            }
            if !self.prop_brake_active {
                self.return_to_center = false;
                self.input.adjusted =
                    ((self.input.newinput - 1048) * 2 + 47).saturating_sub(reversing_dead_band);
            }
        } else if self.input.newinput <= 1047 && self.input.newinput > 47 {
            if self.commutation.forward == (self.config.dir_reversed == 0) {
                self.input.adjusted = 0;
                self.prop_brake_active = true;
                if self.return_to_center {
                    self.commutation.forward = self.config.dir_reversed != 0;
                    self.prop_brake_active = false;
                    self.return_to_center = false;
                }
            }
            if !self.prop_brake_active {
                self.return_to_center = false;
                self.input.adjusted =
                    ((self.input.newinput - 48) * 2 + 47).saturating_sub(reversing_dead_band);
            }
        } else {
            self.input.adjusted = 0;
            if self.prop_brake_active {
                self.prop_brake_active = false;
                self.return_to_center = true;
            }
        }
    }

    fn set_input_servo_rc_car(&mut self, _hal: &mut impl ControlHal) {
        let dead_band = (self.config.servo_dead_band as u16) << 1;
        if self.input.newinput > 1000 + dead_band {
            if self.commutation.forward == (self.config.dir_reversed != 0) {
                self.input.adjusted = 0;
                self.prop_brake_active = true;
                if self.return_to_center {
                    self.commutation.forward = self.config.dir_reversed == 0;
                    self.prop_brake_active = false;
                    self.return_to_center = false;
                }
            }
            if !self.prop_brake_active {
                self.return_to_center = false;
                self.input.adjusted = map(
                    self.input.newinput as i32,
                    (1000 + dead_band) as i32,
                    2000,
                    47,
                    2047,
                ) as u16;
            }
        } else if self.input.newinput < 1000u16.saturating_sub(dead_band) {
            if self.commutation.forward == (self.config.dir_reversed == 0) {
                self.input.adjusted = 0;
                self.prop_brake_active = true;
                if self.return_to_center {
                    self.commutation.forward = self.config.dir_reversed != 0;
                    self.prop_brake_active = false;
                    self.return_to_center = false;
                }
            }
            if !self.prop_brake_active {
                self.return_to_center = false;
                self.input.adjusted = map(
                    self.input.newinput as i32,
                    0,
                    1000i32 - dead_band as i32,
                    2047,
                    47,
                ) as u16;
            }
        } else {
            self.input.adjusted = 0;
            if self.prop_brake_active {
                self.prop_brake_active = false;
                self.return_to_center = true;
            }
        }
    }

    fn set_input_servo_bidir(&mut self, hal: &mut impl ControlHal) {
        let dead_band = (self.config.servo_dead_band as u16) << 1;
        if self.input.newinput > 1000 + dead_band {
            if self.commutation.forward == (self.config.dir_reversed != 0) {
                if (self.timing.commutation_interval > self.reverse_speed_threshold as u32
                    && (self.duty.cycle < 200))
                    || self.stepper_sine
                {
                    self.commutation.forward = self.config.dir_reversed == 0;
                    self.timing.zero_crosses = 0;
                    self.old_routine = true;
                    hal.mask_interrupts();
                } else {
                    self.input.newinput = 1000;
                }
            }
            self.input.adjusted = map(
                self.input.newinput as i32,
                (1000 + dead_band) as i32,
                2000,
                47,
                2047,
            ) as u16;
        } else if self.input.newinput < 1000u16.saturating_sub(dead_band) {
            if self.commutation.forward == (self.config.dir_reversed == 0) {
                if (self.timing.commutation_interval > self.reverse_speed_threshold as u32
                    && (self.duty.cycle < 200))
                    || self.stepper_sine
                {
                    self.timing.zero_crosses = 0;
                    self.old_routine = true;
                    self.commutation.forward = self.config.dir_reversed != 0;
                    hal.mask_interrupts();
                } else {
                    self.input.newinput = 1000;
                }
            }
            self.input.adjusted = map(
                self.input.newinput as i32,
                0,
                1000i32 - dead_band as i32,
                2047,
                47,
            ) as u16;
        } else {
            self.input.adjusted = 0;
        }
    }

    /// 10kHz/20kHz control loop tick. Equivalent to C `tenKhzRoutine()`.
    pub fn ten_khz_tick(&mut self, hal: &mut impl ControlHal) {
        self.duty.cycle = self.duty.setpoint;
        self.ten_khz_counter += 1;
        self.input.signal_timeout += 1;
        self.duty.ramp_count += 1;
        self.one_khz_loop_counter += 1;

        // Telemetry interval
        if self.config.telemetry_on_interval != 0 {
            self.telemetry.ms_count += 1;
            let threshold = (self.telemetry_interval_ms as u16 - 1
                + self.config.telemetry_on_interval as u16)
                * 20;
            if self.telemetry.ms_count > threshold {
                self.telemetry.send_telemetry = true;
                self.telemetry.ms_count = 0;
            }
        }

        // Arming logic
        if !self.armed {
            if self.cell_count == 0 && self.input.input_set && self.input.adjusted == 0 {
                self.armed_timeout_count += 1;
                if self.armed_timeout_count > 20000 {
                    // LOOP_FREQUENCY_HZ
                    if self.input.zero_input_count > 30 {
                        self.armed = true;
                    } else {
                        self.input.input_set = false;
                        self.armed_timeout_count = 0;
                    }
                }
            } else {
                self.armed_timeout_count = 0;
            }
        }

        // 1kHz PID loops
        if self.one_khz_loop_counter > 20 {
            // PID_LOOP_DIVIDER
            self.one_khz_loop_counter = 0;

            if self.pid.use_current_limit && self.running {
                let target = self.config.current_limit as i32 * 200;
                let adj = self
                    .pid
                    .current
                    .calculate(self.measurements.actual_current as i32, target)
                    / 10000;
                self.pid.current_limit_adjust -= adj as i16;
                self.pid.current_limit_adjust = self
                    .pid
                    .current_limit_adjust
                    .clamp(self.duty.minimum as i16, 2000);
            }

            if self.config.stall_protection != 0 && self.running {
                let adj = self.pid.stall.calculate(
                    self.timing.commutation_interval as i32,
                    6500, // stall_protect_target_interval
                );
                self.pid.stall_adjust += adj;
                self.pid.stall_adjust = self.pid.stall_adjust.clamp(0, 150 * 10000);
            }
        }

        // Ramp rate limiting
        if self.duty.ramp_count > self.duty.ramp_divider as u16 {
            self.duty.ramp_count = 0;

            // Select ramp rate
            if self.timing.zero_crosses < 150 || self.duty.last < 150 {
                self.duty.max_change = self.duty.max_ramp_startup;
            } else if self.timing.average_interval > 500 {
                self.duty.max_change = self.duty.max_ramp_low_rpm;
            } else {
                self.duty.max_change = self.duty.max_ramp_high_rpm;
            }

            let change = self.duty.max_change as u16;
            if self.duty.cycle > self.duty.last + change {
                self.duty.cycle = self.duty.last + change;
            }
            if self.duty.last > self.duty.cycle + change {
                self.duty.cycle = self.duty.last - change;
            }
        } else {
            self.duty.cycle = self.duty.last;
        }

        // Adjusted duty cycle calculation
        if self.armed && self.running && self.input.input > 47 {
            self.duty.adjusted =
                ((self.duty.cycle as u32 * self.tim1_arr as u32) / 2000 + 1) as u16;
        } else if self.prop_brake_active {
            self.duty.adjusted = self.tim1_arr
                - ((self.prop_brake_duty_cycle as u32 * self.tim1_arr as u32) / 2000) as u16;
        } else {
            self.duty.adjusted = ((self.duty.cycle as u32 * self.tim1_arr as u32) / 2000) as u16;
        }

        self.duty.last = self.duty.cycle;
        hal.set_auto_reload(self.tim1_arr);
        hal.set_duty_all(self.duty.adjusted);
    }

    /// Main loop iteration. Equivalent to C `main_loop()`.
    pub fn main_loop_tick(&mut self) {
        // e_com_time calculation
        let sum: u32 = self
            .timing
            .commutation_intervals
            .iter()
            .map(|&v| v as u32)
            .sum();
        self.timing.e_com_time = ((sum + 4) >> 1) as i32;

        // Min BEMF counts adjustment
        if self.timing.zero_crosses < 5 {
            if self.config.bi_direction != 0 {
                self.bemf.min_counts_up = 2 + 1; // TARGET_MIN_BEMF_COUNTS + 1
                self.bemf.min_counts_down = 2 + 1;
            } else {
                self.bemf.min_counts_up = 2 * 2; // TARGET_MIN_BEMF_COUNTS * 2
                self.bemf.min_counts_down = 2 * 2;
            }
        } else {
            self.bemf.min_counts_up = 2; // TARGET_MIN_BEMF_COUNTS
            self.bemf.min_counts_down = 2;
        }

        // Variable PWM
        if self.config.variable_pwm == 1 {
            self.tim1_arr = map(
                self.timing.commutation_interval as i32,
                96,
                200,
                self.timer1_max_arr as i32 / 2,
                self.timer1_max_arr as i32,
            ) as u16;
        } else if self.config.variable_pwm == 2 {
            // Automatic: scale average_interval by CPU_MHZ/9, clamped to 100-250
            let avg = self.timing.average_interval;
            let scale = self.cpu_mhz as u32 / 9;
            self.tim1_arr = if avg < 100 && avg > 0 {
                (100 * scale) as u16
            } else if avg >= 250 || avg == 0 {
                (250 * scale) as u16
            } else {
                (avg * scale) as u16
            };
        }

        // Consumed current accumulation (1s interval)
        if self.ten_khz_counter > 20000 {
            // LOOP_FREQUENCY_HZ
            self.measurements.consumed_current += self.measurements.actual_current as i32;
            self.ten_khz_counter = 0;
        }

        // BEMF timeout clearing
        if self.timing.zero_crosses > 1000 || self.input.adjusted == 0 {
            self.protection.bemf_timeout_happened = 0;
        }

        // Average interval
        self.timing.average_interval = (self.timing.e_com_time / 3) as u32;

        // Desync detection
        if self.commutation.desync_check && self.timing.zero_crosses > 10 {
            let diff = get_abs_dif(
                self.timing.last_average_interval as i32,
                self.timing.average_interval as i32,
            );
            if diff > (self.timing.average_interval >> 1) && self.timing.average_interval < 2000 {
                self.timing.zero_crosses = 0;
                self.protection.desync_happened += 1;
                if (self.config.bi_direction == 0 && self.input.input > 47)
                    || self.timing.commutation_interval > 1000
                {
                    self.running = false;
                }
                self.old_routine = true;
                self.duty.last = self.duty.min_startup / 2;
            }
            self.commutation.desync_check = false;
            self.timing.last_average_interval = self.timing.average_interval;
        }

        // Signal timeout
        if self.input.signal_timeout > 10000 {
            // LOOP_FREQUENCY_HZ >> 1
            if self.armed {
                self.armed = false;
                self.input.input = 0;
                self.input.input_set = false;
                self.input.zero_input_count = 0;
            }
        }

        // Send telemetry in main loop
        if self.telemetry.send_telemetry {
            // Caller is responsible for actual packet send
            self.telemetry.send_telemetry = false;
        }
        if self.telemetry.send_esc_info {
            self.telemetry.send_esc_info = false;
        }

        // eRPM calculation
        if !self.stepper_sine && self.timing.e_com_time > 0 {
            self.timing.e_rpm = if self.running {
                (600000 / self.timing.e_com_time) as u16
            } else {
                0
            };
        }

        // Filter level
        if self.timing.zero_crosses < 100 && self.timing.commutation_interval > 500 {
            self.bemf.filter_level = 12;
        } else {
            self.bemf.filter_level =
                map(self.timing.average_interval as i32, 100, 500, 3, 12) as u8;
        }
        if self.timing.commutation_interval < 50 {
            self.bemf.filter_level = 2;
        }

        // Low voltage cutoff (checked at 1kHz rate via one_khz counter reset)
        if self.one_khz_loop_counter == 0 && self.config.low_voltage_cut_off != 0 {
            let threshold = if self.config.low_voltage_cut_off == 1 {
                self.cell_count as u16 * 330 // low_cell_volt_cutoff default
            } else {
                self.config.absolute_voltage_cutoff as u16
            };
            if self.measurements.battery_voltage < threshold && threshold > 0 {
                self.protection.low_voltage_count += 1;
            } else if !self.protection.low_voltage_cutoff {
                self.protection.low_voltage_count = 0;
            }
            if self.protection.low_voltage_count > 10000 {
                self.protection.low_voltage_cutoff = true;
                self.input.input = 0;
                self.running = false;
                self.input.zero_input_count = 0;
                self.armed = false;
            }
        }

        // eRPM-based throttle restriction (protects motor/ESC at extreme RPMs)
        {
            let k_erpm = if self.timing.e_com_time > 0 {
                (600000 / self.timing.e_com_time) / 10
            } else {
                0
            };
            let low_rpm = self.motor_kv as i32 / 100 / (32 / self.config.motor_poles.max(2) as i32);
            let high_rpm = self.motor_kv as i32 / 12 / (32 / self.config.motor_poles.max(2) as i32);
            if k_erpm > 0 && high_rpm > low_rpm {
                self.duty.maximum = map(k_erpm, low_rpm, high_rpm, 600, 2000) as u16;
            } else {
                self.duty.maximum = 2000;
            }
        }

        // Temperature limiting
        if self.measurements.degrees_celsius > self.config.temperature_limit as i16 {
            self.duty.maximum = map(
                self.measurements.degrees_celsius as i32,
                self.config.temperature_limit as i32 - 10,
                self.config.temperature_limit as i32 + 10,
                1000,
                1,
            ) as u16;
        }

        // Auto advance
        if self.config.auto_advance != 0 {
            self.bemf.auto_advance_level = map(self.duty.cycle as i32, 100, 2000, 13, 23) as u8;
        }
    }
}
