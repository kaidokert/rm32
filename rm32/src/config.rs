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
    /// Reserved — hall sensor commutation is not implemented in C or Rust firmware.
    pub reserved_hall_sensors: u8,
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

/// Derived motor configuration — computed from EepromConfig + BoardConfig.
/// All the math that was in main.rs, now testable on host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorConfig {
    /// Base minimum duty cycle (EEPROM minimum_duty_cycle * 10)
    pub minimum_duty: u16,
    /// Minimum startup duty (minimum_duty + startup_power)
    pub min_startup_duty: u16,
    /// Maximum duty during startup ramp
    pub startup_max_duty: u16,
    /// TIM1 auto-reload value for requested PWM frequency
    pub timer1_max_arr: u16,
    /// Dead-time override from driving_brake_strength (0 = no override)
    pub dead_time_override: u16,
    /// Current PID gains (scaled from EEPROM)
    pub current_kp: u32,
    pub current_ki: u32,
    pub current_kd: u32,
    /// Motor KV (scaled from EEPROM, adjusted by board KV divider)
    pub motor_kv: u16,
    /// Low cell voltage cutoff in millivolts
    pub low_cell_volt_cutoff: u16,
    /// Servo calibration
    pub servo_low: u16,
    pub servo_high: u16,
    pub servo_neutral: u16,
}

impl EepromConfig {
    /// Derive motor configuration from EEPROM settings and board hardware.
    ///
    /// `default_arr`: TIM1 auto-reload at default 24kHz (MCU-specific: CPU_MHZ*1e6/24000-1)
    /// `dead_time`: board dead-time from YAML
    /// `kv_divider`: board KV divider (1=normal, 2=3-cell max, 16=1-2 cell max)
    /// `startup_boost`: board flag for heavy-prop startup boost
    pub fn derive_motor_config(
        &self,
        default_arr: u16,
        dead_time: u8,
        kv_divider: u8,
        startup_boost: bool,
    ) -> MotorConfig {
        // Base minimum duty from EEPROM
        let mdc = self.minimum_duty_cycle;
        let minimum_duty_base = if mdc > 0 && mdc < 51 {
            mdc as u16 * 10
        } else {
            0
        };

        // Startup power adds to minimum duty
        let sp = self.startup_power;
        let min_startup_base = if sp > 49 && sp < 151 {
            minimum_duty_base + sp as u16
        } else {
            minimum_duty_base
        };

        // Startup boost: extra duty for heavy props
        let (min_startup_duty, minimum_duty, startup_max_duty) = if startup_boost {
            let pf = self.pwm_frequency;
            (
                min_startup_base + 200 + (pf as u16 * 100 / 24),
                minimum_duty_base + 50 + (pf as u16 * 50 / 24),
                minimum_duty_base + 400,
            )
        } else {
            (min_startup_base, minimum_duty_base, minimum_duty_base + 400)
        };

        // PWM frequency → timer1_max_arr
        let pf = self.pwm_frequency;
        let timer1_max_arr = if pf > 7 && pf < 145 {
            let divider = pf as u32 * 100 / 6;
            (default_arr as u32 * 400 / divider) as u16
        } else {
            default_arr
        };

        // Dead-time override from driving_brake_strength
        let dead_time_override = {
            let mut dbs = self.driving_brake_strength;
            if dbs == 0 || dbs > 9 {
                dbs = 10;
            }
            if dbs < 10 {
                let dto = dead_time as u16 + (150 - dbs as u16 * 10);
                dto.min(200)
            } else {
                0
            }
        };

        // PID gains
        let kv_div = kv_divider.max(1) as u16;
        MotorConfig {
            minimum_duty,
            min_startup_duty,
            startup_max_duty,
            timer1_max_arr,
            dead_time_override,
            current_kp: (self.current_p as u32) * 2,
            current_ki: self.current_i as u32,
            current_kd: (self.current_d as u32) * 2,
            motor_kv: ((self.motor_kv as u16) * 40 + 20) / kv_div,
            low_cell_volt_cutoff: self.low_cell_volt_cutoff as u16 + 250,
            servo_low: (self.servo_low_threshold as u16) * 2 + 750,
            servo_high: (self.servo_high_threshold as u16) * 2 + 1750,
            servo_neutral: self.servo_neutral as u16 + 1374,
        }
    }
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

    // --- MotorConfig derivation tests ---

    #[test]
    fn motor_config_default_eeprom() {
        let cfg = EepromConfig::default();
        let mc = cfg.derive_motor_config(2999, 60, 1, false);
        // Zero EEPROM → minimum_duty=0, startup_power=0 → all duty=0
        assert_eq!(mc.minimum_duty, 0);
        assert_eq!(mc.min_startup_duty, 0);
        assert_eq!(mc.startup_max_duty, 400);
        // pwm_frequency=0 → default ARR
        assert_eq!(mc.timer1_max_arr, 2999);
        // driving_brake_strength=0 → dbs=10 → no override
        assert_eq!(mc.dead_time_override, 0);
    }

    #[test]
    fn motor_config_typical_values() {
        let mut cfg = EepromConfig::default();
        cfg.minimum_duty_cycle = 5; // 5*10 = 50
        cfg.startup_power = 100; // 50+100 = 150
        cfg.pwm_frequency = 24; // 24kHz default → ARR unchanged
        let mc = cfg.derive_motor_config(2999, 60, 1, false);
        assert_eq!(mc.minimum_duty, 50);
        assert_eq!(mc.min_startup_duty, 150);
        assert_eq!(mc.timer1_max_arr, 2999);
    }

    #[test]
    fn motor_config_startup_boost() {
        let mut cfg = EepromConfig::default();
        cfg.minimum_duty_cycle = 5;
        cfg.startup_power = 100;
        cfg.pwm_frequency = 24;
        let mc = cfg.derive_motor_config(2999, 60, 1, true);
        // With boost: extra 200 + pf*100/24 = 200+100 = 300 added to startup
        assert!(mc.min_startup_duty > 150);
        assert!(mc.minimum_duty > 50);
    }

    #[test]
    fn motor_config_pwm_frequency_scaling() {
        let mut cfg = EepromConfig::default();
        cfg.pwm_frequency = 48; // 48kHz → ARR should be ~half
        let mc = cfg.derive_motor_config(2999, 60, 1, false);
        assert!(mc.timer1_max_arr < 2999);
        assert!(mc.timer1_max_arr > 1000);
    }

    #[test]
    fn motor_config_dead_time_override() {
        let mut cfg = EepromConfig::default();
        cfg.driving_brake_strength = 5;
        let mc = cfg.derive_motor_config(2999, 60, 1, false);
        // dbs=5 → dto = 60 + (150 - 50) = 160
        assert_eq!(mc.dead_time_override, 160);
    }

    #[test]
    fn motor_config_kv_divider() {
        let mut cfg = EepromConfig::default();
        cfg.motor_kv = 50; // 50*40+20 = 2020
        let mc1 = cfg.derive_motor_config(2999, 60, 1, false);
        let mc2 = cfg.derive_motor_config(2999, 60, 2, false);
        assert_eq!(mc1.motor_kv, 2020);
        assert_eq!(mc2.motor_kv, 1010);
    }

    #[test]
    fn motor_config_out_of_range_mdc_is_zero() {
        let mut cfg = EepromConfig::default();
        cfg.minimum_duty_cycle = 55; // > 50 → clamped to 0
        let mc = cfg.derive_motor_config(2999, 60, 1, false);
        assert_eq!(mc.minimum_duty, 0);
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
