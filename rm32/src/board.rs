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
    /// Use external NTC thermistor instead of internal temp sensor
    pub use_ntc: bool,
    /// Inverted input signal polarity
    pub inverted_input: bool,
    /// KV divider for limited cell count boards (1=normal, 2=THREE_CELL_MAX, 16=ONE_TWO_CELL_MAX)
    pub kv_divider: u8,
    /// Enable startup boost (extra initial torque for heavy props)
    pub startup_boost: bool,
    /// Enable voltage-based ramp rate scaling
    pub voltage_based_ramp: bool,
    /// Enable RPM pulse output on commutation step 1/4 (debug)
    pub pulse_output: bool,
    /// Enable dual ADC conversion triggering
    pub dual_adc: bool,
    /// PWM/enable bridge mode (low-side pins are enable, not complementary PWM)
    pub bridge_enable: bool,
    /// Custom LED on PB3: blinks with throttle position, solid when armed
    pub custom_led: bool,
}

impl BoardConfig {
    /// Default values matching C firmware defaults from targets.h
    pub const DEFAULT: Self = Self {
        name: "Generic",
        dead_time: 60,
        voltage_divider: 110,
        millivolt_per_amp: 20,
        current_offset: 0,
        current_adc_channel: 4, // PA4
        voltage_adc_channel: 6, // PA6
        stall_protect_interval: 6500,
        has_led: false,
        led_pin: None,
        min_bemf_counts: 2,
        use_ntc: false,
        inverted_input: false,
        kv_divider: 1,
        startup_boost: false,
        voltage_based_ramp: false,
        pulse_output: false,
        dual_adc: false,
        bridge_enable: false,
        custom_led: false,
    };
}
