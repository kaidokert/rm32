//! RM32 host-side test harness v2 — uses isr_logic path.
//!
//! Same stdin/stdout protocol as harness.rs (and C am32_harness).
//! Uses isr_logic::ten_khz_tick() + input_mapping + SharedComm
//! instead of the legacy MotorState/tick.rs path.

use rm32::commutation::Commutation;
use rm32::config::EepromConfig;
use rm32::control::context::MotorContext;
use rm32::control::isr_logic::{self, TickCounters};
use rm32::control::shared_impl::TestShared;
use rm32::control::state::{BemfState, DutyState};
use rm32::dshot;
use rm32::hal;
use rm32::input_mapping;
use rm32::motor_mode::MotorMode;
use rm32::shared_comm::SharedComm;
use rm32::signal;
use std::io::{self, BufRead, Write};

// --- Mock HAL (same as harness.rs) ---

struct MockPwm {
    duty: u16,
    arr: u16,
    duty_count: u32,
}
impl hal::PwmOutput for MockPwm {
    fn set_duty_all(&mut self, d: u16) {
        self.duty = d;
        self.duty_count += 1;
    }
    fn set_auto_reload(&mut self, arr: u16) {
        self.arr = arr;
    }
    fn set_prescaler(&mut self, _: u16) {}
    fn set_compare1(&mut self, _: u16) {}
    fn set_compare2(&mut self, _: u16) {}
    fn set_compare3(&mut self, _: u16) {}
    fn generate_update_event(&mut self) {}
    fn set_dead_time_override(&mut self, _: u16) {}
}

struct MockComp {
    level: bool,
}
impl hal::Comparator for MockComp {
    fn output_level(&self) -> bool {
        self.level
    }
    fn set_step(&mut self, _: u8, _: bool) {}
    fn change_input(&mut self) {}
    fn enable_interrupts(&mut self) {}
    fn mask_interrupts(&mut self) {}
}

struct MockPhase;
impl hal::PhaseOutput for MockPhase {
    fn com_step(&mut self, _: u8) {}
    fn all_off(&mut self) {}
    fn full_brake(&mut self) {}
    fn all_pwm(&mut self) {}
    fn proportional_brake(&mut self) {}
}

struct MockInterval {
    count: u32,
}
impl hal::IntervalTimer for MockInterval {
    fn count(&self) -> u32 {
        self.count
    }
    fn set_count(&mut self, v: u32) {
        self.count = v;
    }
}

struct MockComTimer;
impl hal::ComTimer for MockComTimer {
    fn set_and_enable(&mut self, _: u16) {}
    fn disable_interrupt(&mut self) {}
    fn enable_interrupt(&mut self) {}
}

struct MockMotorHal {
    pwm: MockPwm,
    comp: MockComp,
    phase: MockPhase,
    interval: MockInterval,
    com_timer: MockComTimer,
}

impl hal::MotorHal for MockMotorHal {
    type Pwm = MockPwm;
    type Comp = MockComp;
    type Phase = MockPhase;
    type Interval = MockInterval;
    type Com = MockComTimer;

    fn pwm(&mut self) -> &mut MockPwm {
        &mut self.pwm
    }
    fn comp(&mut self) -> &mut MockComp {
        &mut self.comp
    }
    fn phase(&mut self) -> &mut MockPhase {
        &mut self.phase
    }
    fn interval(&mut self) -> &mut MockInterval {
        &mut self.interval
    }
    fn com_timer(&mut self) -> &mut MockComTimer {
        &mut self.com_timer
    }
}

// --- Harness state ---

struct Harness {
    shared: TestShared,
    commutation: Commutation,
    bemf: BemfState,
    duty: DutyState,
    config: EepromConfig,
    counters: TickCounters,
    hal: MockMotorHal,

    // Harness-level state
    tick_count: u32,
    has_throttle: bool,
    throttle_value: u16,
    do_transfer: bool,
    dma_buffer: [u32; 64],

    // Input state (replaces MotorState.input)
    input: u16, // mapped throttle (after set_input mapping)
    dshot: bool,
    servo_pwm: bool,
    edt_armed: bool,

    // Bidirectional state
    prop_brake_active: bool,
    #[allow(dead_code)] // TODO: wire up RC-car reverse path
    return_to_center: bool,
}

impl Harness {
    fn new() -> Self {
        Self {
            shared: TestShared::new(),
            commutation: Commutation::new(),
            bemf: BemfState::default(),
            duty: DutyState::default(),
            config: EepromConfig::default(),
            counters: TickCounters {
                ten_khz_counter: 0,
                one_khz_loop_counter: 0,
                armed_timeout_count: 0,
                tim1_arr: 1999,
                voltage_based_ramp: false,
            },
            hal: MockMotorHal {
                pwm: MockPwm {
                    duty: 0,
                    arr: 0,
                    duty_count: 0,
                },
                comp: MockComp { level: false },
                phase: MockPhase,
                interval: MockInterval { count: 0 },
                com_timer: MockComTimer,
            },
            tick_count: 0,
            has_throttle: false,
            throttle_value: 0,
            do_transfer: false,
            dma_buffer: [0; 64],
            input: 0,
            dshot: false,
            servo_pwm: false,
            edt_armed: false,
            prop_brake_active: false,
            return_to_center: false,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn build_dshot_frame(&mut self, value: u16) {
        let mut bits = [0u8; 16];
        for (i, bit) in bits[..11].iter_mut().enumerate() {
            *bit = ((value >> (10 - i)) & 1) as u8;
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
        for (i, &bit) in bits.iter().enumerate() {
            self.dma_buffer[i * 2] = base;
            self.dma_buffer[i * 2 + 1] = base + if bit != 0 { 22 } else { 10 };
            base += 32;
        }
    }

    fn handle_transfer(&mut self) {
        if self.shared.input_set() {
            if self.dshot {
                let buf: [u32; 32] = self.dma_buffer[..32].try_into().unwrap();
                let frame = dshot::decode_frame(&buf, 400, 600, false);
                match frame {
                    dshot::DshotFrame::Throttle { value, telemetry } => {
                        if self.edt_armed && value > 47 {
                            self.shared.set_newinput(value);
                        } else if value == 0 {
                            self.shared.set_newinput(0);
                        }
                        if telemetry {
                            self.shared.set_send_telemetry(true);
                        }
                        // Reset signal timeout via shared
                        // (TestShared doesn't have direct set, use workaround)
                    }
                    dshot::DshotFrame::Command { cmd, .. } => {
                        self.shared.set_newinput(0);
                        match cmd {
                            7 => {
                                self.config.dir_reversed = 0;
                                self.commutation.forward = true;
                            }
                            8 => {
                                self.config.dir_reversed = 1;
                                self.commutation.forward = false;
                            }
                            9 => self.config.bi_direction = 0,
                            10 => self.config.bi_direction = 1,
                            20 => {
                                self.commutation.forward = self.config.dir_reversed == 0;
                            }
                            21 => {
                                self.commutation.forward = self.config.dir_reversed != 0;
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            } else if self.servo_pwm {
                let pulse = self.dma_buffer[1].saturating_sub(self.dma_buffer[0]) as u16;
                if pulse > 800 && pulse < 2200 {
                    let val = signal::compute_servo_unidirectional(pulse, 1100, 1900);
                    self.shared.set_newinput(val);
                }
            }
        } else {
            let sig = signal::detect_input(&self.dma_buffer[..32], 48);
            match sig {
                signal::SignalType::Dshot600 | signal::SignalType::Dshot300 => {
                    self.dshot = true;
                    self.shared.set_input_set(true);
                }
                signal::SignalType::ServoPwm => {
                    self.servo_pwm = true;
                    self.shared.set_input_set(true);
                }
                _ => {}
            }
        }
    }

    fn do_tick(&mut self) {
        // Apply persistent throttle
        if self.has_throttle {
            self.shared.set_newinput(self.throttle_value);
            self.shared.signal_timeout.set(0);
        }

        // Advance interval timer
        self.hal.interval.count += 1;

        // Handle transfer complete
        if self.do_transfer {
            self.handle_transfer();
            self.do_transfer = false;
        }

        // Bidirectional throttle mapping (replaces set_input bidir logic)
        let newinput = self.shared.newinput();
        if self.config.bi_direction != 0 {
            let r = input_mapping::dshot_bidir(
                newinput,
                self.commutation.forward,
                self.config.dir_reversed != 0,
                self.shared.commutation_interval(),
                self.duty.cycle,
                self.shared.stepper_sine(),
                1500, // reverse_speed_threshold
            );
            self.shared.set_adjusted_input(r.adjusted);
            if r.reverse {
                self.commutation.forward = !self.commutation.forward;
                self.shared.set_zero_crosses(0);
                self.shared.set_old_routine(true);
            }
        } else {
            self.shared.set_adjusted_input(newinput);
        }

        // Sine start throttle mapping
        if self.config.use_sine_start != 0 {
            let mapped = input_mapping::sine_start_map(
                self.shared.adjusted_input(),
                self.config.sine_mode_changeover_throttle_level,
            );
            self.input = mapped;
        } else {
            self.input = self.shared.adjusted_input();
        }

        // ISR tick (arming, throttle→setpoint, ramp, PWM output)
        let mut ctx = MotorContext {
            commutation: &mut self.commutation,
            bemf: &mut self.bemf,
            duty: &mut self.duty,
            config: &self.config,
            counters: &mut self.counters,
            shared: &self.shared,
            hal: &mut self.hal,
        };
        isr_logic::ten_khz_tick(&mut ctx);

        self.tick_count += 1;
    }

    fn print_state(&self) {
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
            self.shared.armed() as i32,
            self.shared.running() as i32,
            self.commutation.step,
            self.commutation.forward as i32,
            self.duty.cycle,
            self.shared.duty_cycle_setpoint(),
            0u16, // adjusted_duty_cycle not computed in isr_logic path
            self.shared.commutation_interval(),
            0u32, // average_interval computed in main_state
            self.shared.e_com_time(),
            0u16, // e_rpm computed in main_state
            self.shared.zero_crosses(),
            self.input,
            self.shared.adjusted_input(),
            self.shared.newinput(),
            self.bemf.counter,
            self.bemf.zc_found as i32,
            self.commutation.rising as i32,
            self.shared.old_routine() as i32,
            self.shared.stepper_sine() as i32,
            self.shared.signal_timeout(),
            self.counters.armed_timeout_count,
            0u16, // battery_voltage from main_state
            0i16, // actual_current from main_state
            0i16, // degrees_celsius from main_state
            self.duty.last,
            self.prop_brake_active as i32,
            self.shared.input_set() as i32,
            self.dshot as i32,
            self.servo_pwm as i32,
            self.hal.pwm.duty,
            self.hal.pwm.arr,
            self.hal.pwm.duty_count,
            self.duty.maximum,
            self.bemf.filter_level,
            self.shared.send_telemetry() as i32,
            0i32, // send_esc_info_flag
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
                    self.edt_armed = true;
                }
            }
            "comp" => self.hal.comp.level = v != 0,
            "transfer" => self.do_transfer = v != 0,
            "dshot_frame" => {
                self.build_dshot_frame(v as u16);
                self.do_transfer = true;
            }
            "zc" => {
                if v == 1 {
                    isr_logic::bemf_zero_cross(
                        &self.commutation,
                        &mut self.bemf,
                        &mut self.hal.comp,
                        &mut self.hal.interval,
                        &mut self.hal.com_timer,
                    );
                    isr_logic::commutation_timer_expired(
                        &mut self.commutation,
                        &mut self.bemf,
                        &self.shared,
                        &mut self.hal.com_timer,
                        &mut self.hal.comp,
                        &mut self.hal.phase,
                    );
                }
            }
            "interval_timer" => self.hal.interval.count = v as u32,
            k if k.starts_with("dma_") => {
                if let Ok(idx) = k[4..].parse::<usize>()
                    && idx < 64
                {
                    self.dma_buffer[idx] = v as u32;
                }
            }
            // Direct state overrides
            "armed" => {
                if v != 0 {
                    self.shared.mode.set(MotorMode::Armed);
                } else {
                    self.shared.mode.set(MotorMode::Disarmed);
                }
            }
            "running" => {
                if v != 0 {
                    self.shared.mode.set(MotorMode::OldRoutine);
                } else if self.shared.armed() {
                    self.shared.mode.set(MotorMode::Armed);
                }
            }
            "inputSet" => self.shared.set_input_set(v != 0),
            "dshot" => self.dshot = v != 0,
            "servoPwm" => self.servo_pwm = v != 0,
            "forward" => self.commutation.forward = v != 0,
            "step" => self.commutation.step = v as u8,
            "old_routine" => self.shared.set_old_routine(v != 0),
            "zero_crosses" => self.shared.set_zero_crosses(v as u32),
            "commutation_interval" => self.shared.set_commutation_interval(v as u32),
            "zero_input_count" => {} // not in TestShared
            "EDT_ARMED" => self.edt_armed = v != 0,
            "EDT_ARM_ENABLE" => {}
            "dshot_telemetry" => {}
            "signaltimeout" => self.shared.signal_timeout.set(v as u16),
            "cell_count" => {}
            "battery_voltage" => {}
            "degrees_celsius" => {}
            "actual_current" => {}
            "bemf_timeout_happened" => {}
            "bemf_timeout" => {}
            "prop_brake_active" => self.prop_brake_active = v != 0,
            "stepper_sine" => self.shared.set_stepper_sine(v != 0),
            "last_duty_cycle" => self.duty.last = v as u16,
            "use_current_limit" => {}
            "use_speed_control_loop" => {}
            "send_esc_info_flag" => {}
            "send_telemetry" => self.shared.set_send_telemetry(v != 0),
            "low_voltage_count" => {}
            "out_put" => {}
            "duty_cycle" => self.duty.cycle = v as u16,
            "adjusted_input" => self.shared.set_adjusted_input(v as u16),
            "desync_check" => {}
            "average_interval" => {}
            "last_average_interval" => {}
            "process_adc" => {}
            // Calibration state (not implemented in v2)
            "calibration_required"
            | "high_calibration_set"
            | "high_calibration_counts"
            | "low_calibration_counts"
            | "servo_high_threshold"
            | "servo_low_threshold"
            | "enter_calibration_count"
            | "last_input" => {}
            // EEPROM config
            "eeprom.bi_direction" => self.config.bi_direction = v as u8,
            "eeprom.dir_reversed" => self.config.dir_reversed = v as u8,
            "eeprom.rc_car_reverse" => self.config.rc_car_reverse = v as u8,
            "eeprom.use_sine_start" => self.config.use_sine_start = v as u8,
            "eeprom.comp_pwm" => self.config.comp_pwm = v as u8,
            "eeprom.variable_pwm" => self.config.variable_pwm = v as u8,
            "eeprom.brake_on_stop" => self.config.brake_on_stop = v as u8,
            "eeprom.stall_protection" => self.config.stall_protection = v as u8,
            "eeprom.stuck_rotor_protection" => self.config.stuck_rotor_protection = v as u8,
            "eeprom.sine_mode_changeover_thottle_level"
            | "eeprom.sine_mode_changeover_throttle_level" => {
                self.config.sine_mode_changeover_throttle_level = v as u8;
            }
            "eeprom.drag_brake_strength" => self.config.drag_brake_strength = v as u8,
            "eeprom.input_type" => self.config.input_type = v as u8,
            "eeprom.telemetry_on_interval" => self.config.telemetry_on_interval = v as u8,
            "eeprom.low_voltage_cut_off" => self.config.low_voltage_cut_off = v as u8,
            "eeprom.limits.temperature" => self.config.temperature_limit = v as u8,
            "eeprom.limits.current" => self.config.current_limit = v as u8,
            "eeprom.beep_volume" => self.config.beep_volume = v as u8,
            "eeprom.motor_kv" => self.config.motor_kv = v as u8,
            "eeprom.motor_poles" => self.config.motor_poles = v as u8,
            "eeprom.advance_level" => self.config.advance_level = v as u8,
            "eeprom.max_ramp" => self.config.max_ramp = v as u8,
            "eeprom.eeprom_version" => self.config.eeprom_version = v as u8,
            "eeprom.current_I" => self.config.current_i = v as u8,
            "eeprom.current_P" => self.config.current_p = v as u8,
            "eeprom.current_D" => self.config.current_d = v as u8,
            "eeprom.sine_mode_power" => self.config.sine_mode_power = v as u8,
            "eeprom.driving_brake_strength" => self.config.driving_brake_strength = v as u8,
            _ => eprintln!("harness2: unknown key '{}'", key),
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
            // No-op for v2 (config set directly)
            println!("ok");
            io::stdout().flush().unwrap();
        } else if let Some(rest) = line.strip_prefix("config ") {
            harness.parse_kvs(rest);
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
        } else if let Some(rest) = line.strip_prefix("gcr_encode ") {
            let mut com_time: u16 = 0;
            let mut padding: usize = 7;
            for token in rest.split_whitespace() {
                if let Some(val) = token.strip_prefix("padding=") {
                    padding = val.parse().unwrap_or(7);
                } else {
                    com_time = token.parse().unwrap_or(0);
                }
            }
            let running = harness.shared.running();
            let value = dshot::erpm_to_12bit(com_time, running);
            let mut gcr = [0u32; 37];
            dshot::encode_gcr_frame(value, &mut gcr, padding, dshot::GCR_SHIFT_F0);
            let mut csum = 0u16;
            let mut cd = value;
            for _ in 0..3 {
                csum ^= cd;
                cd >>= 4;
            }
            csum = !csum & 0xF;
            let dshot_full = (value << 4) | csum;
            let shift = if !running {
                7
            } else {
                let mut s = 0u8;
                for i in (9..=15).rev() {
                    if com_time >> i == 1 {
                        s = (i + 1 - 9) as u8;
                        break;
                    }
                }
                s
            };
            print!("gcr=");
            for (i, val) in gcr.iter().enumerate() {
                if i > 0 {
                    print!(",");
                }
                print!("{}", val);
            }
            println!(
                " shift={} dshot_full={} padding={}",
                shift, dshot_full, padding
            );
            io::stdout().flush().unwrap();
        } else {
            eprintln!("harness2: unknown command '{}'", line);
        }
    }
}
