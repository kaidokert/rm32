//! MCU-specific configuration and PAC re-exports.
//!
//! Each MCU implements `ChipConfig` with associated constants.
//! Drivers can be generic over `T: ChipConfig` instead of reading `config::*`.

// --- PAC re-exports ---
#[cfg(feature = "stm32g071")]
pub use stm32g0xx_hal::stm32 as pac;
#[cfg(feature = "stm32g071")]
pub use stm32g0xx_hal as hal_impl;

#[cfg(feature = "stm32l431")]
pub use stm32l4xx_hal::pac;
#[cfg(feature = "stm32l431")]
pub use stm32l4xx_hal as hal_impl;

#[cfg(feature = "stm32g431")]
pub use stm32g4::stm32g431 as pac;

#[cfg(feature = "stm32f051")]
pub use stm32f0xx_hal::pac;
#[cfg(feature = "stm32f051")]
pub use stm32f0xx_hal as hal_impl;

// --- ChipConfig trait ---

/// MCU-specific constants. Replaces flat `mod config` with a trait
/// that drivers can be generic over.
pub trait ChipConfig {
    const CPU_FREQUENCY_MHZ: u32;
    const EEPROM_START: u32;
    const FLASH_PAGE_SIZE: u32;
    const TIMER_PSC: u16;
    const GCR_SHIFT: u8;
    const COMP_EXTI_LINE: u32;
    const INPUT_DMA_CHANNEL: usize;
    const ADC_CURRENT_CHANNEL: u8;
    const ADC_VOLTAGE_CHANNEL: u8;
    const TIM1_AUTORELOAD: u16 = ((Self::CPU_FREQUENCY_MHZ * 1_000_000 / 24_000) - 1) as u16;
}

// --- Per-MCU implementations ---

#[cfg(feature = "stm32g071")]
pub struct Chip;
#[cfg(feature = "stm32g071")]
impl ChipConfig for Chip {
    const CPU_FREQUENCY_MHZ: u32 = 64;
    const EEPROM_START: u32 = 0x0800_F800;
    const FLASH_PAGE_SIZE: u32 = 0x800;
    const TIMER_PSC: u16 = 31;
    const GCR_SHIFT: u8 = 7;
    const COMP_EXTI_LINE: u32 = 18;
    const INPUT_DMA_CHANNEL: usize = 0;
    const ADC_CURRENT_CHANNEL: u8 = 4;
    const ADC_VOLTAGE_CHANNEL: u8 = 6;
}

#[cfg(feature = "stm32l431")]
pub struct Chip;
#[cfg(feature = "stm32l431")]
impl ChipConfig for Chip {
    const CPU_FREQUENCY_MHZ: u32 = 80;
    const EEPROM_START: u32 = 0x0800_F800;
    const FLASH_PAGE_SIZE: u32 = 0x800;
    const TIMER_PSC: u16 = 39;
    const GCR_SHIFT: u8 = 7;
    const COMP_EXTI_LINE: u32 = 22;
    const INPUT_DMA_CHANNEL: usize = 4;
    const ADC_CURRENT_CHANNEL: u8 = 8;
    const ADC_VOLTAGE_CHANNEL: u8 = 11;
}

#[cfg(feature = "stm32g431")]
pub struct Chip;
#[cfg(feature = "stm32g431")]
impl ChipConfig for Chip {
    const CPU_FREQUENCY_MHZ: u32 = 170;
    const EEPROM_START: u32 = 0x0800_F800;
    const FLASH_PAGE_SIZE: u32 = 0x800;
    const TIMER_PSC: u16 = 84;
    const GCR_SHIFT: u8 = 7;
    const COMP_EXTI_LINE: u32 = 21;
    const INPUT_DMA_CHANNEL: usize = 0;
    const ADC_CURRENT_CHANNEL: u8 = 5;
    const ADC_VOLTAGE_CHANNEL: u8 = 13;
}

#[cfg(feature = "stm32f051")]
pub struct Chip;
#[cfg(feature = "stm32f051")]
impl ChipConfig for Chip {
    const CPU_FREQUENCY_MHZ: u32 = 48;
    const EEPROM_START: u32 = 0x0800_7C00;
    const FLASH_PAGE_SIZE: u32 = 0x400;
    const TIMER_PSC: u16 = 23;
    const GCR_SHIFT: u8 = 6;
    const COMP_EXTI_LINE: u32 = 21;
    const INPUT_DMA_CHANNEL: usize = 4;
    const ADC_CURRENT_CHANNEL: u8 = 6;
    const ADC_VOLTAGE_CHANNEL: u8 = 3;
}

// --- Backward-compatible `config` module (delegates to Chip) ---
// Existing code uses `config::CPU_FREQUENCY_MHZ` etc. — this preserves that.

pub mod config {
    use super::{Chip, ChipConfig};
    pub const CPU_FREQUENCY_MHZ: u32 = Chip::CPU_FREQUENCY_MHZ;
    pub const EEPROM_START: u32 = Chip::EEPROM_START;
    pub const FLASH_PAGE_SIZE: u32 = Chip::FLASH_PAGE_SIZE;
    pub const TIMER_PSC: u16 = Chip::TIMER_PSC;
    pub const GCR_SHIFT: u8 = Chip::GCR_SHIFT;
    pub const COMP_EXTI_LINE: u32 = Chip::COMP_EXTI_LINE;
    pub const INPUT_DMA_CHANNEL: usize = Chip::INPUT_DMA_CHANNEL;
    pub const ADC_CURRENT_CHANNEL: u8 = Chip::ADC_CURRENT_CHANNEL;
    pub const ADC_VOLTAGE_CHANNEL: u8 = Chip::ADC_VOLTAGE_CHANNEL;
    pub const TIM1_AUTORELOAD: u16 = Chip::TIM1_AUTORELOAD;
}
