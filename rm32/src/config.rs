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

/// Persistent ESC settings (192 bytes, matches C EEPROM layout).
/// `Pod` + `Zeroable` derived via bytemuck — guarantees safe byte-level access.
#[derive(Clone, bytemuck::Pod, bytemuck::Zeroable, Copy)]
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

// Compile-time check: struct size must match EEPROM layout exactly.
// If a field is added/removed/resized, this fails to compile.
const _: () = {
    if core::mem::size_of::<EepromConfig>() != 192 {
        panic!("EepromConfig size must be exactly 192 bytes");
    }
};

impl EepromConfig {
    pub const SIZE: usize = 192;

    /// View config as raw bytes. Safe via bytemuck::Pod — no unsafe needed.
    pub fn as_bytes(&self) -> &[u8] {
        bytemuck::bytes_of(self)
    }

    /// View config as mutable raw bytes.
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        bytemuck::bytes_of_mut(self)
    }

    /// Create config from raw bytes (e.g. flash read).
    pub fn from_bytes(bytes: &[u8; Self::SIZE]) -> Self {
        *bytemuck::from_bytes(bytes)
    }

    // --- Typed accessors for boolean-like fields ---

    pub fn is_dir_reversed(&self) -> bool {
        self.dir_reversed != 0
    }
    pub fn is_bidirectional(&self) -> bool {
        self.bi_direction != 0
    }
    pub fn use_sine_start(&self) -> bool {
        self.use_sine_start != 0
    }
    pub fn use_comp_pwm(&self) -> bool {
        self.comp_pwm != 0
    }
    pub fn has_stuck_rotor_protection(&self) -> bool {
        self.stuck_rotor_protection != 0
    }
    pub fn has_stall_protection(&self) -> bool {
        self.stall_protection != 0
    }
    pub fn has_low_voltage_cutoff(&self) -> bool {
        self.low_voltage_cut_off != 0
    }
    pub fn is_lvc_per_cell(&self) -> bool {
        self.low_voltage_cut_off == 1
    }
    pub fn is_lvc_absolute(&self) -> bool {
        self.low_voltage_cut_off == 2
    }
    pub fn disable_stick_cal(&self) -> bool {
        self.disable_stick_calibration != 0
    }
    pub fn is_rc_car_reverse(&self) -> bool {
        self.rc_car_reverse != 0
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

pub const EEPROM_VERSION: u8 = 3;

impl EepromConfig {
    /// Check if loaded EEPROM data is valid (not blank/corrupt).
    /// Blank flash (all 0xFF) will have eeprom_version=255 which fails.
    pub fn is_valid(&self) -> bool {
        // Version 0 is a fresh zero-init (valid but needs defaults applied)
        // Version > EEPROM_VERSION suggests corrupt/blank flash
        self.eeprom_version <= EEPROM_VERSION
    }

    /// Apply defaults for fields added in newer EEPROM versions.
    /// Uses VERSION_DEFAULTS as the reference — changing a default in one place
    /// automatically propagates to the migration logic.
    pub fn apply_version_defaults(&mut self) {
        if self.eeprom_version < EEPROM_VERSION {
            let d = &VERSION_DEFAULTS;
            self.max_ramp = d.max_ramp;
            self.minimum_duty_cycle = d.minimum_duty_cycle;
            self.disable_stick_calibration = d.disable_stick_calibration;
            self.absolute_voltage_cutoff = d.absolute_voltage_cutoff;
            self.current_p = d.current_p;
            self.current_i = d.current_i;
            self.current_d = d.current_d;
            self.active_brake_power = d.active_brake_power;
            self.reserved_eeprom_3 = d.reserved_eeprom_3;
        }
        self.eeprom_version = EEPROM_VERSION;
    }
}

/// Canonical defaults for version-migrated fields.
/// Single source of truth — used by both apply_version_defaults and tests.
const VERSION_DEFAULTS: EepromConfig = {
    let mut c = EepromConfig::ZEROED;
    c.max_ramp = 160;
    c.minimum_duty_cycle = 1;
    c.absolute_voltage_cutoff = 10;
    c.current_p = 100;
    c.current_d = 100;
    c
};

impl EepromConfig {
    /// Const zero-init.
    /// SAFETY: bytemuck::Zeroable derive proves all-zeros is a valid bit pattern.
    /// Using mem::zeroed() because Zeroable::zeroed() is not const fn.
    const ZEROED: Self = unsafe { core::mem::zeroed() };
}

impl Default for EepromConfig {
    fn default() -> Self {
        Self::ZEROED
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blank_flash_is_invalid() {
        // All 0xFF simulates erased flash
        let mut cfg = EepromConfig::default();
        for b in cfg.as_bytes_mut().iter_mut() {
            *b = 0xFF;
        }
        assert!(!cfg.is_valid()); // eeprom_version=255 > EEPROM_VERSION
    }

    #[test]
    fn zero_init_is_valid() {
        let cfg = EepromConfig::default();
        assert!(cfg.is_valid()); // eeprom_version=0 <= EEPROM_VERSION
    }

    #[test]
    fn current_version_is_valid() {
        let mut cfg = EepromConfig::default();
        cfg.eeprom_version = EEPROM_VERSION;
        assert!(cfg.is_valid());
    }

    #[test]
    fn future_version_is_invalid() {
        let mut cfg = EepromConfig::default();
        cfg.eeprom_version = EEPROM_VERSION + 1;
        assert!(!cfg.is_valid());
    }

    #[test]
    fn version_defaults_applied_for_old_config() {
        let mut cfg = EepromConfig::default(); // version=0
        cfg.apply_version_defaults();
        assert_eq!(cfg.eeprom_version, EEPROM_VERSION);
        assert_eq!(cfg.max_ramp, 160);
        assert_eq!(cfg.minimum_duty_cycle, 1);
        assert_eq!(cfg.current_p, 100);
        assert_eq!(cfg.current_d, 100);
        assert_eq!(cfg.absolute_voltage_cutoff, 10);
    }

    #[test]
    fn version_defaults_not_applied_for_current_config() {
        let mut cfg = EepromConfig::default();
        cfg.eeprom_version = EEPROM_VERSION;
        cfg.max_ramp = 42; // custom value
        cfg.apply_version_defaults();
        assert_eq!(cfg.max_ramp, 42); // should NOT be overwritten
    }

    #[test]
    fn blank_flash_fallback_produces_safe_defaults() {
        // Simulate the full main.rs load path
        let mut cfg = EepromConfig::default();
        for b in cfg.as_bytes_mut().iter_mut() {
            *b = 0xFF;
        }
        if !cfg.is_valid() {
            cfg = EepromConfig::default();
        }
        cfg.apply_version_defaults();
        assert_eq!(cfg.eeprom_version, EEPROM_VERSION);
        assert_eq!(cfg.max_ramp, 160);
        assert_eq!(cfg.motor_kv, 0); // zero-init default
    }
}
