//! RM32 STM32 HAL implementation.
//!
//! Supports STM32G071, STM32F051, STM32L431, STM32G431 via feature flags.
//! MCU-specific configuration is centralized in `mcu.rs`.

#![no_std]

pub mod mcu;
pub use mcu::pac;

// --- Shared across all MCUs (zero cfg) ---
pub mod adc_generic;
pub mod adc_hal;
pub mod capture_generic;
pub mod capture_hal;
pub mod comp_hal;
pub mod comparator;
pub mod dma_buf;
pub mod emergency;
pub mod flash;
pub mod gpio_pin;
pub mod gpio_regs;
pub mod init;
pub mod isr;
pub mod isr_handlers;
#[cfg(not(test))]
mod panic;
pub mod phase;
pub mod regs;
pub mod stub;
pub mod telem_hal;
pub mod timer;
pub mod ws2812_hal;

// --- MCU-specific peripheral modules (one cfg per chip) ---
#[cfg(feature = "stm32f051")]
pub mod mcu_f051;
#[cfg(feature = "stm32g071")]
pub mod mcu_g071;
#[cfg(feature = "stm32g431")]
pub mod mcu_g431;
#[cfg(feature = "stm32l431")]
pub mod mcu_l431;
