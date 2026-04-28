//! MCU configuration gateway — trait definition + re-exports from active MCU.
//!
//! Zero cfg blocks for individual MCUs. Each `mcu_xxx/chip.rs` provides
//! the PAC re-export, HAL re-export, and ChipConfig implementation.

/// MCU-specific constants. Drivers can be generic over `T: ChipConfig`.
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
    const WDG_PRESCALER: u8;
    const WDG_RELOAD: u16;
}

// Re-export pac, hal_impl, and Chip from the active MCU directory.
#[cfg(feature = "stm32f051")]
pub use crate::mcu_f051::chip::*;
#[cfg(feature = "stm32g071")]
pub use crate::mcu_g071::chip::*;
#[cfg(feature = "stm32g431")]
pub use crate::mcu_g431::chip::*;
#[cfg(feature = "stm32l431")]
pub use crate::mcu_l431::chip::*;
