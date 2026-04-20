//! EEPROM configuration structure and defaults.
//!
//! Mirrors the C `EEprom_t` union.

/// Input type selection
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum InputType {
    Auto = 0,
    Dshot = 1,
    Servo = 2,
    Serial = 3,
    EdtArm = 4,
    DroneCan = 5,
}

/// Persistent ESC settings (192 bytes, matches C EEPROM layout)
#[derive(Clone)]
#[repr(C)]
pub struct EepromConfig {
    pub reserved_0: u8,
    pub eeprom_version: u8,
    pub reserved_1: u8,
    pub version_major: u8,
    pub version_minor: u8,
    pub max_ramp: u8,
    pub minimum_duty_cycle: u8,
    pub disable_stick_calibration: u8,
    pub absolute_voltage_cutoff: u8,
    pub current_p: u8,
    pub current_i: u8,
    pub current_d: u8,
    pub active_brake_power: u8,
    pub reserved_eeprom_3: [u8; 4],
    pub dir_reversed: u8,
    pub bi_direction: u8,
    pub use_sine_start: u8,
    pub comp_pwm: u8,
    pub variable_pwm: u8,
    pub stuck_rotor_protection: u8,
    pub advance_level: u8,
    pub pwm_frequency: u8,
    pub startup_power: u8,
    pub motor_kv: u8,
    pub motor_poles: u8,
    pub brake_on_stop: u8,
    pub stall_protection: u8,
    pub beep_volume: u8,
    pub telemetry_on_interval: u8,
    pub servo_low_threshold: u8,
    pub servo_high_threshold: u8,
    pub servo_neutral: u8,
    pub servo_dead_band: u8,
    pub low_voltage_cut_off: u8,
    pub low_cell_volt_cutoff: u8,
    pub rc_car_reverse: u8,
    pub use_hall_sensors: u8,
    pub sine_mode_changeover_throttle_level: u8,
    pub drag_brake_strength: u8,
    pub driving_brake_strength: u8,
    pub temperature_limit: u8,
    pub current_limit: u8,
    pub sine_mode_power: u8,
    pub input_type: u8,
    pub auto_advance: u8,
    pub tune: [u8; 128],
    pub can_node: u8,
    pub esc_index: u8,
    pub can_require_arming: u8,
    pub can_telem_rate: u8,
    pub can_require_zero_throttle: u8,
    pub can_filter_hz: u8,
    pub can_debug_rate: u8,
    pub can_term_enable: u8,
    pub can_reserved: [u8; 8],
}

impl EepromConfig {
    pub const SIZE: usize = 192;

    pub fn as_bytes(&self) -> &[u8; Self::SIZE] {
        // Safe: repr(C), all fields are plain bytes
        unsafe { &*(self as *const Self as *const [u8; Self::SIZE]) }
    }

    pub fn as_bytes_mut(&mut self) -> &mut [u8; Self::SIZE] {
        unsafe { &mut *(self as *mut Self as *mut [u8; Self::SIZE]) }
    }

    pub fn input_type(&self) -> InputType {
        match self.input_type {
            0 => InputType::Auto,
            1 => InputType::Dshot,
            2 => InputType::Servo,
            3 => InputType::Serial,
            4 => InputType::EdtArm,
            5 => InputType::DroneCan,
            _ => InputType::Auto,
        }
    }
}

impl Default for EepromConfig {
    fn default() -> Self {
        // Zero-init matches C behavior for fresh EEPROM
        unsafe { core::mem::zeroed() }
    }
}
