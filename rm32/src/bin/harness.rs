//! RM32 host-side test harness.
//!
//! Same stdin/stdout line protocol as the C am32_harness.
//! Python blackbox tests can verify both implementations produce identical output.

use rm32::control::state::MotorState;
use rm32::dshot;
use rm32::hal;
use rm32::signal;
use std::io::{self, BufRead, Write};

/// Mock HAL that records PWM outputs.
struct StdHal {
    timer_count: u32,
    pwm_duty: u16,
    pwm_arr: u16,
    pwm_duty_count: u32,
    comp_level: bool,
    mask_called: bool,
}

impl StdHal {
    fn new() -> Self {
        Self {
            timer_count: 0,
            pwm_duty: 0,
            pwm_arr: 0,
            pwm_duty_count: 0,
            comp_level: false,
            mask_called: false,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

impl hal::PwmOutput for StdHal {
    fn set_duty_all(&mut self, duty: u16) {
        self.pwm_duty = duty;
        self.pwm_duty_count += 1;
    }
    fn set_auto_reload(&mut self, arr: u16) {
        self.pwm_arr = arr;
    }
    fn set_prescaler(&mut self, _psc: u16) {}
    fn set_compare1(&mut self, _val: u16) {}
    fn set_compare2(&mut self, _val: u16) {}
    fn set_compare3(&mut self, _val: u16) {}
    fn generate_update_event(&mut self) {}
    fn set_dead_time_override(&mut self, _dtg: u16) {}
}

impl hal::Comparator for StdHal {
    fn output_level(&self) -> bool {
        self.comp_level
    }
    fn set_step(&mut self, _step: u8, _rising: bool) {}
    fn change_input(&mut self) {}
    fn enable_interrupts(&mut self) {}
    fn mask_interrupts(&mut self) {
        self.mask_called = true;
    }
}

impl hal::PhaseOutput for StdHal {
    fn com_step(&mut self, _step: u8) {}
    fn all_off(&mut self) {}
    fn full_brake(&mut self) {}
    fn all_pwm(&mut self) {}
    fn proportional_brake(&mut self) {}
}

impl hal::IntervalTimer for StdHal {
    fn count(&self) -> u32 {
        self.timer_count
    }
    fn set_count(&mut self, val: u32) {
        self.timer_count = val;
    }
}

impl hal::ComTimer for StdHal {
    fn set_and_enable(&mut self, _timeout: u16) {}
    fn disable_interrupt(&mut self) {}
    fn enable_interrupt(&mut self) {}
}

impl hal::System for StdHal {
    fn reset(&mut self) -> ! {
        std::process::exit(0)
    }
    fn enable_irq(&mut self) {}
    fn disable_irq(&mut self) {}
    fn start_watchdog(&mut self, _prescaler: u8, _reload: u16) {}
    fn reload_watchdog(&mut self) {}
    fn delay_micros(&mut self, _us: u32) {}
    fn delay_millis(&mut self, _ms: u32) {}
}

struct Harness {
    state: MotorState,
    hal: StdHal,
    tick_count: u32,
    has_throttle: bool,
    throttle_value: u16,
    do_transfer: bool,
    dma_buffer: [u32; 64],
}

impl Harness {
    fn new() -> Self {
        Self {
            state: MotorState::default(),
            hal: StdHal::new(),
            tick_count: 0,
            has_throttle: false,
            throttle_value: 0,
            do_transfer: false,
            dma_buffer: [0; 64],
        }
    }

    fn reset(&mut self) {
        self.state = MotorState::default();
        self.hal.reset();
        self.tick_count = 0;
        self.has_throttle = false;
        self.throttle_value = 0;
        self.do_transfer = false;
        self.dma_buffer = [0; 64];
    }

    /// Build a DShot frame in the DMA buffer.
    fn build_dshot_frame(&mut self, value: u16) {
        let mut bits = [0u8; 16];
        for i in 0..11 {
            bits[i] = ((value >> (10 - i)) & 1) as u8;
        }
        bits[11] = 0;
        let crc = (bits[0] ^ bits[4] ^ bits[8]) << 3
            | (bits[1] ^ bits[5] ^ bits[9]) << 2
            | (bits[2] ^ bits[6] ^ bits[10]) << 1
            | (bits[3] ^ bits[7] ^ bits[11]);
        bits[12] = (crc >> 3) & 1;
        bits[13] = (crc >> 2) & 1;
        bits[14] = (crc >> 1) & 1;
        bits[15] = crc & 1;

        let mut base = 1000u32;
        for i in 0..16 {
            self.dma_buffer[i * 2] = base;
            self.dma_buffer[i * 2 + 1] = base + if bits[i] != 0 { 22 } else { 10 };
            base += 32;
        }
    }

    /// Process a transfer complete (DShot or servo input).
    fn handle_transfer(&mut self) {
        if self.state.input.input_set {
            if self.state.input.dshot {
                // Decode DShot frame
                let buf: [u32; 32] = self.dma_buffer[..32].try_into().unwrap();
                let frame = dshot::decode_frame(&buf, 400, 600, false);
                match frame {
                    dshot::DshotFrame::Throttle { value, telemetry } => {
                        if self.state.input.edt_armed && value > 47 {
                            self.state.input.newinput = value;
                        } else if value == 0 {
                            self.state.input.newinput = 0;
                        }
                        if telemetry {
                            self.state.telemetry.send_telemetry = true;
                        }
                        self.state.input.signal_timeout = 0;
                    }
                    dshot::DshotFrame::Command { cmd, .. } => {
                        self.state.input.newinput = 0;
                        self.state.input.signal_timeout = 0;
                        // Process commands (direction, bidir, etc.)
                        // Simplified: just set forward based on cmd 7/8/20/21
                        match cmd {
                            7 => {
                                self.state.config.dir_reversed = 0;
                                self.state.commutation.forward = true;
                            }
                            8 => {
                                self.state.config.dir_reversed = 1;
                                self.state.commutation.forward = false;
                            }
                            9 => {
                                self.state.config.bi_direction = 0;
                            }
                            10 => {
                                self.state.config.bi_direction = 1;
                            }
                            20 => {
                                self.state.commutation.forward =
                                    self.state.config.dir_reversed == 0;
                            }
                            21 => {
                                self.state.commutation.forward =
                                    self.state.config.dir_reversed != 0;
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            } else if self.state.input.servo_pwm {
                // Servo: compute input from pulse width
                let pulse = self.dma_buffer[1].saturating_sub(self.dma_buffer[0]) as u16;
                if pulse > 800 && pulse < 2200 {
                    let val = signal::compute_servo_unidirectional(pulse, 1100, 1900);
                    self.state.input.newinput = val;
                    self.state.input.signal_timeout = 0;
                }
            }
        } else {
            // Detect input type
            let sig = signal::detect_input(&self.dma_buffer[..32], 48);
            match sig {
                signal::SignalType::Dshot600 | signal::SignalType::Dshot300 => {
                    self.state.input.dshot = true;
                    self.state.input.input_set = true;
                }
                signal::SignalType::ServoPwm => {
                    self.state.input.servo_pwm = true;
                    self.state.input.input_set = true;
                }
                _ => {}
            }
        }
    }

    fn do_tick(&mut self) {
        if self.has_throttle {
            self.state.input.newinput = self.throttle_value;
            self.state.input.signal_timeout = 0;
        }
        self.hal.timer_count += 1;
        if self.do_transfer {
            self.handle_transfer();
            self.do_transfer = false;
        }
        self.state.set_input(&mut self.hal);
        self.state.ten_khz_tick(&mut self.hal);
        self.state.main_loop_tick();
        self.tick_count += 1;
    }

    fn print_state(&self) {
        let s = &self.state;
        println!(
            "tick={} armed={} running={} step={} forward={} \
             duty_cycle={} duty_cycle_setpoint={} adjusted_duty_cycle={} \
             commutation_interval={} average_interval={} \
             e_com_time={} e_rpm={} zero_crosses={} \
             input={} adjusted_input={} newinput={} \
             bemfcounter={} zcfound={} rising={} \
             old_routine={} stepper_sine={} \
             signaltimeout={} armed_timeout_count={} \
             battery_voltage={} actual_current={} degrees_celsius={} \
             last_duty_cycle={} prop_brake_active={} \
             inputSet={} dshot={} servoPwm={} \
             pwm_duty={} pwm_arr={} pwm_duty_count={} \
             duty_cycle_maximum={} filter_level={} \
             send_telemetry={} send_esc_info_flag={}",
            self.tick_count,
            s.armed as i32,
            s.running as i32,
            s.commutation.step,
            s.commutation.forward as i32,
            s.duty.cycle,
            s.duty.setpoint,
            s.duty.adjusted,
            s.timing.commutation_interval,
            s.timing.average_interval,
            s.timing.e_com_time,
            s.timing.e_rpm,
            s.timing.zero_crosses,
            s.input.input,
            s.input.adjusted,
            s.input.newinput,
            s.bemf.counter,
            s.bemf.zc_found as i32,
            s.commutation.rising as i32,
            s.old_routine as i32,
            s.stepper_sine as i32,
            s.input.signal_timeout,
            s.armed_timeout_count,
            s.measurements.battery_voltage,
            s.measurements.actual_current,
            s.measurements.degrees_celsius,
            s.duty.last,
            s.prop_brake_active as i32,
            s.input.input_set as i32,
            s.input.dshot as i32,
            s.input.servo_pwm as i32,
            self.hal.pwm_duty,
            self.hal.pwm_arr,
            self.hal.pwm_duty_count,
            s.duty.maximum,
            s.bemf.filter_level,
            s.telemetry.send_telemetry as i32,
            s.telemetry.send_esc_info as i32,
        );
        io::stdout().flush().unwrap();
    }

    fn apply_kv(&mut self, key: &str, val: &str) {
        let v: i64 = val.parse().unwrap_or(0);
        match key {
            "throttle" => {
                if v < 0 {
                    self.has_throttle = false;
                } else {
                    self.throttle_value = v as u16;
                    self.has_throttle = true;
                    self.state.input.edt_armed = true;
                }
            }
            "comp" => self.hal.comp_level = v != 0,
            "transfer" => self.do_transfer = v != 0,
            "dshot_frame" => {
                self.build_dshot_frame(v as u16);
                self.do_transfer = true;
            }
            "zc" => {
                if v == 1 {
                    self.state.interrupt_routine(&mut self.hal);
                    self.state.period_elapsed_callback(&mut self.hal);
                }
            }
            "interval_timer" => self.hal.timer_count = v as u32,
            k if k.starts_with("dma_") => {
                if let Ok(idx) = k[4..].parse::<usize>()
                    && idx < 64
                {
                    self.dma_buffer[idx] = v as u32;
                }
            }
            "armed" => self.state.armed = v != 0,
            "running" => self.state.running = v != 0,
            "inputSet" => self.state.input.input_set = v != 0,
            "dshot" => self.state.input.dshot = v != 0,
            "servoPwm" => self.state.input.servo_pwm = v != 0,
            "forward" => self.state.commutation.forward = v != 0,
            "step" => self.state.commutation.step = v as u8,
            "old_routine" => self.state.old_routine = v != 0,
            "zero_crosses" => self.state.timing.zero_crosses = v as u32,
            "commutation_interval" => self.state.timing.commutation_interval = v as u32,
            "zero_input_count" => self.state.input.zero_input_count = v as u16,
            "EDT_ARMED" => self.state.input.edt_armed = v != 0,
            "EDT_ARM_ENABLE" => self.state.input.edt_arm_enable = v != 0,
            "dshot_telemetry" => self.state.input.dshot_telemetry = v != 0,
            "signaltimeout" => self.state.input.signal_timeout = v as u16,
            "cell_count" => self.state.cell_count = v as u8,
            "battery_voltage" => self.state.measurements.battery_voltage = v as u16,
            "degrees_celsius" => self.state.measurements.degrees_celsius = v as i16,
            "actual_current" => self.state.measurements.actual_current = v as i16,
            "bemf_timeout_happened" => self.state.protection.bemf_timeout_happened = v as u8,
            "prop_brake_active" => self.state.prop_brake_active = v != 0,
            "stepper_sine" => self.state.stepper_sine = v != 0,
            "last_duty_cycle" => self.state.duty.last = v as u16,
            "use_current_limit" => self.state.pid.use_current_limit = v != 0,
            "use_speed_control_loop" => self.state.pid.use_speed_control = v != 0,
            "send_esc_info_flag" => self.state.telemetry.send_esc_info = v != 0,
            "send_telemetry" => self.state.telemetry.send_telemetry = v != 0,
            "low_voltage_count" => self.state.protection.low_voltage_count = v as u16,
            "out_put" => {} // not used in Rust harness (bidir DMA direction)
            "calibration_required"
            | "high_calibration_set"
            | "high_calibration_counts"
            | "low_calibration_counts"
            | "servo_high_threshold"
            | "servo_low_threshold"
            | "enter_calibration_count"
            | "last_input"
            | "adjusted_input" => {
                // Servo calibration state - simplified in Rust
                // Some of these map to input state
                if key == "adjusted_input" {
                    self.state.input.adjusted = v as u16;
                }
            }
            "duty_cycle" => self.state.duty.cycle = v as u16,
            "bemf_timeout" => self.state.protection.bemf_timeout = v as u8,
            // eeprom
            "eeprom.bi_direction" => self.state.config.bi_direction = v as u8,
            "eeprom.dir_reversed" => self.state.config.dir_reversed = v as u8,
            "eeprom.rc_car_reverse" => self.state.config.rc_car_reverse = v as u8,
            "eeprom.use_sine_start" => self.state.config.use_sine_start = v as u8,
            "eeprom.comp_pwm" => self.state.config.comp_pwm = v as u8,
            "eeprom.variable_pwm" => self.state.config.variable_pwm = v as u8,
            "eeprom.brake_on_stop" => self.state.config.brake_on_stop = v as u8,
            "eeprom.stall_protection" => self.state.config.stall_protection = v as u8,
            "eeprom.stuck_rotor_protection" => self.state.config.stuck_rotor_protection = v as u8,
            "eeprom.sine_mode_changeover_thottle_level" => {
                self.state.config.sine_mode_changeover_throttle_level = v as u8
            }
            "eeprom.drag_brake_strength" => self.state.config.drag_brake_strength = v as u8,
            "eeprom.input_type" => self.state.config.input_type = v as u8,
            "eeprom.telemetry_on_interval" => self.state.config.telemetry_on_interval = v as u8,
            "eeprom.low_voltage_cut_off" => self.state.config.low_voltage_cut_off = v as u8,
            "eeprom.limits.temperature" => self.state.config.temperature_limit = v as u8,
            "eeprom.limits.current" => self.state.config.current_limit = v as u8,
            "eeprom.beep_volume" => self.state.config.beep_volume = v as u8,
            "eeprom.motor_kv" => self.state.config.motor_kv = v as u8,
            "eeprom.motor_poles" => self.state.config.motor_poles = v as u8,
            "eeprom.advance_level" => self.state.config.advance_level = v as u8,
            "eeprom.max_ramp" => self.state.config.max_ramp = v as u8,
            "eeprom.eeprom_version" => self.state.config.eeprom_version = v as u8,
            "eeprom.current_I" => self.state.config.current_i = v as u8,
            "eeprom.current_P" => self.state.config.current_p = v as u8,
            "eeprom.current_D" => self.state.config.current_d = v as u8,
            "eeprom.sine_mode_power" => self.state.config.sine_mode_power = v as u8,
            "eeprom.driving_brake_strength" => self.state.config.driving_brake_strength = v as u8,
            _ => eprintln!("harness: unknown key '{}'", key),
        }
    }

    fn parse_kvs(&mut self, args: &str) {
        for token in args.split_whitespace() {
            if let Some((k, v)) = token.split_once('=') {
                self.apply_kv(k, v);
            }
        }
    }
}

fn main() {
    let mut harness = Harness::new();
    let stdin = io::stdin();

    println!("ready");
    io::stdout().flush().unwrap();

    for line in stdin.lock().lines() {
        let line = line.unwrap();
        let line = line.trim();

        if line.starts_with("quit") {
            break;
        } else if line.starts_with("reset") {
            harness.reset();
            println!("reset");
            io::stdout().flush().unwrap();
        } else if line.starts_with("state") {
            harness.print_state();
        } else if line.starts_with("load_eeprom") {
            harness.state.load_settings();
            println!("ok");
            io::stdout().flush().unwrap();
        } else if line.starts_with("config ") {
            harness.parse_kvs(&line[7..]);
            println!("ok");
            io::stdout().flush().unwrap();
        } else if let Some(rest) = line.strip_prefix("ticks ") {
            let (n_str, kvs) = rest.split_once(' ').unwrap_or((rest, ""));
            let n: u32 = n_str.parse().unwrap_or(1);
            if !kvs.is_empty() {
                harness.parse_kvs(kvs);
            }
            for _ in 0..n {
                harness.do_tick();
            }
            harness.print_state();
        } else if line.starts_with("tick") {
            if line.len() > 4 && &line[4..5] == " " {
                harness.parse_kvs(&line[5..]);
            }
            harness.do_tick();
            harness.print_state();
        } else {
            eprintln!("harness: unknown command '{}'", line);
        }
    }
}
