//! EEPROM settings loading and application.

use crate::config::InputType;
use crate::control::state::MotorState;

const EEPROM_VERSION: u8 = 3;

impl MotorState {
    /// Load and apply settings from eeprom config. Equivalent to C `loadEEpromSettings()`.
    pub fn load_settings(&mut self) {
        let cfg = &mut self.config;

        // Apply defaults for old eeprom version
        if cfg.eeprom_version < EEPROM_VERSION {
            cfg.max_ramp = 160;
            cfg.minimum_duty_cycle = 1;
            cfg.disable_stick_calibration = 0;
            cfg.absolute_voltage_cutoff = 10;
            cfg.current_p = 100;
            cfg.current_i = 0;
            cfg.current_d = 100;
            cfg.active_brake_power = 0;
            cfg.reserved_eeprom_3 = [0; 4];
        }

        // Advance level conversion
        if cfg.advance_level > 42 || (cfg.advance_level < 10 && cfg.advance_level > 3) {
            self.bemf.temp_advance = 16;
        }
        if cfg.advance_level < 4 {
            self.bemf.temp_advance = cfg.advance_level << 3;
            cfg.advance_level = self.bemf.temp_advance + 10;
        }
        if cfg.advance_level < 43 && cfg.advance_level > 9 {
            self.bemf.temp_advance = cfg.advance_level - 10;
        }

        // Motor KV
        self.motor_kv = (cfg.motor_kv as u16) * 40 + 20;

        // Apply settings that require eeprom_version >= 1
        if cfg.eeprom_version > 0 {
            // Servo thresholds
            self.input.servo_low_threshold = (cfg.servo_low_threshold as u16) * 2 + 750;
            self.input.servo_high_threshold = (cfg.servo_high_threshold as u16) * 2 + 1750;
            self.input.servo_neutral = cfg.servo_neutral as u16 + 1374;

            // Current limit PID
            self.pid.current.kp = (cfg.current_p as u32) * 2;
            self.pid.current.ki = cfg.current_i as u32;
            self.pid.current.kd = (cfg.current_d as u32) * 2;

            if cfg.current_limit > 0 && cfg.current_limit < 100 {
                self.pid.use_current_limit = true;
            }

            // Input type
            match cfg.input_type() {
                InputType::Auto => {
                    self.input.dshot = false;
                    self.input.servo_pwm = false;
                    self.input.edt_armed = true;
                }
                InputType::Dshot => {
                    self.input.dshot = true;
                    self.input.edt_armed = true;
                }
                InputType::Servo => {
                    self.input.servo_pwm = true;
                }
                InputType::Serial => {}
                InputType::EdtArm => {
                    self.input.edt_arm_enable = true;
                    self.input.edt_armed = false;
                    self.input.dshot = true;
                }
                InputType::DroneCan => {}
            }

            // Sine mode
            if cfg.sine_mode_changeover_throttle_level < 5
                || cfg.sine_mode_changeover_throttle_level > 25
            {
                cfg.sine_mode_changeover_throttle_level = 5;
            }
            if cfg.sine_mode_power == 0 || cfg.sine_mode_power > 10 {
                cfg.sine_mode_power = 5;
            }

            // Temperature limit
            if cfg.temperature_limit < 70 || cfg.temperature_limit > 140 {
                cfg.temperature_limit = 255;
            }

            // Drag brake
            if cfg.drag_brake_strength == 0 || cfg.drag_brake_strength > 10 {
                cfg.drag_brake_strength = 10;
            }
            if cfg.driving_brake_strength == 0 || cfg.driving_brake_strength > 9 {
                cfg.driving_brake_strength = 10;
            }

            // Ramp
            if cfg.max_ramp < 10 {
                self.duty.ramp_divider = 9;
                self.duty.max_ramp_startup = cfg.max_ramp;
                self.duty.max_ramp_low_rpm = cfg.max_ramp;
                self.duty.max_ramp_high_rpm = cfg.max_ramp;
            } else {
                self.duty.ramp_divider = 0;
                let r = cfg.max_ramp / 10;
                if r < self.duty.max_ramp_startup {
                    self.duty.max_ramp_startup = r;
                }
                if r < self.duty.max_ramp_low_rpm {
                    self.duty.max_ramp_low_rpm = r;
                }
                if r < self.duty.max_ramp_high_rpm {
                    self.duty.max_ramp_high_rpm = r;
                }
            }
        }

        // Bidirectional polling changeover
        if cfg.bi_direction != 0 {
            self.timing.polling_mode_changeover = 1000; // POLLING_MODE_THRESHOLD / 2
        } else {
            self.timing.polling_mode_changeover = 2000; // POLLING_MODE_THRESHOLD
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup_valid_config(state: &mut MotorState) {
        let cfg = &mut state.config;
        cfg.eeprom_version = EEPROM_VERSION;
        cfg.motor_kv = 50;
        cfg.motor_poles = 14;
        cfg.advance_level = 16;
        cfg.beep_volume = 5;
        cfg.temperature_limit = 80;
        cfg.drag_brake_strength = 5;
        cfg.driving_brake_strength = 10;
        cfg.sine_mode_power = 5;
        cfg.sine_mode_changeover_throttle_level = 10;
        cfg.input_type = 0; // Auto
        cfg.max_ramp = 50;
    }

    #[test]
    fn applies_defaults_for_old_version() {
        let mut state = MotorState::default();
        state.config.eeprom_version = 0;
        state.load_settings();
        assert_eq!(state.config.max_ramp, 160);
        assert_eq!(state.config.minimum_duty_cycle, 1);
        assert_eq!(state.config.current_p, 100);
        assert_eq!(state.config.current_d, 100);
    }

    #[test]
    fn skips_defaults_for_current_version() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.max_ramp = 50;
        state.load_settings();
        assert_eq!(state.config.max_ramp, 50);
    }

    #[test]
    fn advance_level_old_format_conversion() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.advance_level = 2;
        state.load_settings();
        assert_eq!(state.bemf.temp_advance, 16); // 2 << 3
        assert_eq!(state.config.advance_level, 26); // 16 + 10
    }

    #[test]
    fn advance_level_new_format() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.advance_level = 20;
        state.load_settings();
        assert_eq!(state.bemf.temp_advance, 10); // 20 - 10
    }

    #[test]
    fn advance_level_out_of_range() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.advance_level = 50;
        state.load_settings();
        assert_eq!(state.bemf.temp_advance, 16); // default
    }

    #[test]
    fn input_type_auto() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.input_type = 0;
        state.load_settings();
        assert!(!state.input.dshot);
        assert!(!state.input.servo_pwm);
        assert!(state.input.edt_armed);
    }

    #[test]
    fn input_type_dshot() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.input_type = 1;
        state.load_settings();
        assert!(state.input.dshot);
        assert!(state.input.edt_armed);
    }

    #[test]
    fn input_type_servo() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.input_type = 2;
        state.load_settings();
        assert!(state.input.servo_pwm);
    }

    #[test]
    fn input_type_edtarm() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.input_type = 4;
        state.load_settings();
        assert!(state.input.edt_arm_enable);
        assert!(!state.input.edt_armed);
        assert!(state.input.dshot);
    }

    #[test]
    fn servo_thresholds() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.servo_low_threshold = 100;
        state.config.servo_high_threshold = 50;
        state.config.servo_neutral = 126;
        state.load_settings();
        assert_eq!(state.input.servo_low_threshold, 950); // 100*2 + 750
        assert_eq!(state.input.servo_high_threshold, 1850); // 50*2 + 1750
        assert_eq!(state.input.servo_neutral, 1500); // 126 + 1374
    }

    #[test]
    fn current_limit_pid() {
        let mut state = MotorState::default();
        setup_valid_config(&mut state);
        state.config.current_p = 80;
        state.config.current_i = 10;
        state.config.current_d = 40;
        state.config.current_limit = 50;
        state.load_settings();
        assert_eq!(state.pid.current.kp, 160);
        assert_eq!(state.pid.current.ki, 10);
        assert_eq!(state.pid.current.kd, 80);
        assert!(state.pid.use_current_limit);
    }
}
