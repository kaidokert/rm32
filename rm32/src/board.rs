//! Board target configuration.
//!
//! Each ESC board has specific hardware characteristics: dead-time,
//! voltage divider ratio, current sensor scaling, ADC channel assignments.
//! These are compile-time constants in the C firmware (from targets.h).

/// Board-specific hardware configuration.
#[derive(Clone, Copy)]
pub struct BoardConfig {
    /// Firmware name (max 12 chars)
    pub name: &'static str,
    /// PWM dead-time in TIM1 DTG units
    pub dead_time: u8,
    /// Voltage divider ratio (e.g. 110 = 11:1 divider × 10)
    pub voltage_divider: u16,
    /// Current sensor millivolts per amp
    pub millivolt_per_amp: u16,
    /// Current sensor offset (ADC counts)
    pub current_offset: i16,
    /// ADC channel for current sense (STM32 channel number)
    pub current_adc_channel: u8,
    /// ADC channel for voltage sense
    pub voltage_adc_channel: u8,
    /// Stall protection target interval
    pub stall_protect_interval: u16,
    /// Minimum BEMF filter counts for zero-cross detection (varies by target)
    pub min_bemf_counts: u8,
    /// Whether this board has a WS2812 LED
    pub has_led: bool,
    /// WS2812 LED pin number on GPIOB (e.g. 8 for PB8)
    pub led_pin: Option<u8>,
}

impl BoardConfig {
    /// Default values matching C firmware defaults from targets.h
    pub const DEFAULT: Self = Self {
        name: "Generic",
        dead_time: 60,
        voltage_divider: 110,
        millivolt_per_amp: 20,
        current_offset: 0,
        current_adc_channel: 4,  // PA4
        voltage_adc_channel: 6,  // PA6
        stall_protect_interval: 6500,
        has_led: false,
        led_pin: None,
        min_bemf_counts: 2, // TARGET_MIN_BEMF_COUNTS default
    };
}

// --- Pre-defined board configs for G071 targets ---

pub const GEN_64K_G071: BoardConfig = BoardConfig {
    name: "G071 64kESC",
    dead_time: 60,
    has_led: true,
    led_pin: Some(8), // PB8
    ..BoardConfig::DEFAULT
};

pub const TBS_4IN1_G071: BoardConfig = BoardConfig {
    name: "TBS 4N1 G071",
    dead_time: 60,
    current_adc_channel: 5,  // PA5
    voltage_adc_channel: 6,  // PA6
    ..BoardConfig::DEFAULT
};

pub const TBS_12S_G071: BoardConfig = BoardConfig {
    name: "TBS 12S G071",
    dead_time: 80,
    voltage_divider: 210,
    millivolt_per_amp: 25,
    ..BoardConfig::DEFAULT
};

pub const FLYCOLOR_G071: BoardConfig = BoardConfig {
    name: "Flycolor G071",
    dead_time: 40,
    ..BoardConfig::DEFAULT
};

pub const MAMBA_F55_G071: BoardConfig = BoardConfig {
    name: "Mamba F55 G071",
    dead_time: 40,
    ..BoardConfig::DEFAULT
};

pub const IFLIGHT_12S_G071: BoardConfig = BoardConfig {
    name: "iFlight 12S",
    dead_time: 80,
    voltage_divider: 210,
    millivolt_per_amp: 25,
    ..BoardConfig::DEFAULT
};

// --- F051 boards ---

/// Default F051 board (HARDWARE_GROUP_F0_A)
/// ADC channels differ from G071: current=PA6/CH6, voltage=PA3/CH3
pub const SISKIN_F051: BoardConfig = BoardConfig {
    name: "SISKIN PA2",
    dead_time: 45,
    current_adc_channel: 6,
    voltage_adc_channel: 3,
    ..BoardConfig::DEFAULT
};

pub const WRAITH32V2_F051: BoardConfig = BoardConfig {
    name: "Wraith32 V2",
    dead_time: 45,
    current_adc_channel: 6,
    voltage_adc_channel: 3,
    ..BoardConfig::DEFAULT
};

pub const IFLIGHT_F051: BoardConfig = BoardConfig {
    name: "iFlight F051",
    dead_time: 45,
    millivolt_per_amp: 30,
    current_adc_channel: 6,
    voltage_adc_channel: 3,
    ..BoardConfig::DEFAULT
};

// --- L431 boards ---

pub const NEUTRON_L431: BoardConfig = BoardConfig {
    name: "L431 Neutron",
    dead_time: 60,
    voltage_divider: 210,
    millivolt_per_amp: 16,
    current_offset: 498,
    current_adc_channel: 8,
    voltage_adc_channel: 11,
    ..BoardConfig::DEFAULT
};

pub const VIMDRONES_L431: BoardConfig = BoardConfig {
    name: "VimDrones L4",
    dead_time: 60,
    voltage_divider: 210,
    millivolt_per_amp: 8,
    current_offset: 498,
    current_adc_channel: 8,
    voltage_adc_channel: 11,
    ..BoardConfig::DEFAULT
};
