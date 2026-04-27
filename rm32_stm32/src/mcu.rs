//! MCU-specific configuration and PAC re-exports.
//!
//! Feature-gated: exactly one of `stm32g071` or `stm32f051` must be enabled.

// --- STM32G071 ---
#[cfg(feature = "stm32g071")]
pub use stm32g0xx_hal::stm32 as pac;
#[cfg(feature = "stm32g071")]
pub use stm32g0xx_hal as hal_impl;

#[cfg(feature = "stm32g071")]
pub mod config {
    pub const CPU_FREQUENCY_MHZ: u32 = 64;
    pub const EEPROM_START: u32 = 0x0800_F800;
    pub const FLASH_PAGE_SIZE: u32 = 0x800; // 2KB
    pub const TIMER_PSC: u16 = 31; // 64MHz / 32 = 2MHz
    pub const GCR_SHIFT: u8 = 7;   // ×128 for G0
    pub const COMP_EXTI_LINE: u32 = 18;
    pub const INPUT_DMA_CHANNEL: usize = 0; // DMA1_CH1
    pub const ADC_CURRENT_CHANNEL: u8 = 4;  // PA4
    pub const ADC_VOLTAGE_CHANNEL: u8 = 6;  // PA6
    pub const TIM1_AUTORELOAD: u16 = ((CPU_FREQUENCY_MHZ * 1_000_000 / 24_000) - 1) as u16;
}

// --- STM32L431 ---
#[cfg(feature = "stm32l431")]
pub use stm32l4xx_hal::pac;
#[cfg(feature = "stm32l431")]
pub use stm32l4xx_hal as hal_impl;

#[cfg(feature = "stm32l431")]
pub mod config {
    pub const CPU_FREQUENCY_MHZ: u32 = 80;
    pub const EEPROM_START: u32 = 0x0800_F800;
    pub const FLASH_PAGE_SIZE: u32 = 0x800;
    pub const TIMER_PSC: u16 = 39; // 80MHz / 40 = 2MHz
    pub const GCR_SHIFT: u8 = 7;
    pub const COMP_EXTI_LINE: u32 = 22; // COMP2 on EXTI22
    pub const INPUT_DMA_CHANNEL: usize = 4;
    pub const ADC_CURRENT_CHANNEL: u8 = 8;
    pub const ADC_VOLTAGE_CHANNEL: u8 = 11;
    pub const TIM1_AUTORELOAD: u16 = ((CPU_FREQUENCY_MHZ * 1_000_000 / 24_000) - 1) as u16;
}

// --- STM32G431 ---
#[cfg(feature = "stm32g431")]
pub use stm32g4::stm32g431 as pac;

#[cfg(feature = "stm32g431")]
pub mod config {
    pub const CPU_FREQUENCY_MHZ: u32 = 170;
    pub const EEPROM_START: u32 = 0x0800_F800;
    pub const FLASH_PAGE_SIZE: u32 = 0x800; // 2KB
    pub const TIMER_PSC: u16 = 84; // 170MHz / 85 = 2MHz
    pub const GCR_SHIFT: u8 = 7;
    pub const COMP_EXTI_LINE: u32 = 21; // COMP1 on EXTI21
    pub const INPUT_DMA_CHANNEL: usize = 0; // DMA1_CH1
    pub const ADC_CURRENT_CHANNEL: u8 = 5;  // PA4
    pub const ADC_VOLTAGE_CHANNEL: u8 = 13; // PA5
    pub const TIM1_AUTORELOAD: u16 = ((CPU_FREQUENCY_MHZ * 1_000_000 / 24_000) - 1) as u16;
}

// --- STM32F051 ---
#[cfg(feature = "stm32f051")]
pub use stm32f0xx_hal::pac;
#[cfg(feature = "stm32f051")]
pub use stm32f0xx_hal as hal_impl;

#[cfg(feature = "stm32f051")]
pub mod config {
    pub const CPU_FREQUENCY_MHZ: u32 = 48;
    pub const EEPROM_START: u32 = 0x0800_7C00;
    pub const FLASH_PAGE_SIZE: u32 = 0x400; // 1KB
    pub const TIMER_PSC: u16 = 23; // 48MHz / 24 = 2MHz
    pub const GCR_SHIFT: u8 = 6;   // ×64 for F0
    pub const COMP_EXTI_LINE: u32 = 21;
    pub const INPUT_DMA_CHANNEL: usize = 4; // DMA1_CH5
    pub const ADC_CURRENT_CHANNEL: u8 = 6;  // PA6
    pub const ADC_VOLTAGE_CHANNEL: u8 = 3;  // PA3
    pub const TIM1_AUTORELOAD: u16 = ((CPU_FREQUENCY_MHZ * 1_000_000 / 24_000) - 1) as u16;
}
