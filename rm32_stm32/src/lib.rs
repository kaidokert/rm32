//! RM32 STM32 HAL implementation.
//!
//! Supports STM32G071, STM32F051, STM32L431, STM32G431 via feature flags.
//! MCU-specific configuration is centralized in `mcu.rs`.

#![no_std]

pub mod mcu;
pub use mcu::pac;
pub use mcu::config;

// --- Shared across all MCUs (zero cfg) ---
pub mod comparator;
pub mod timer;
pub mod phase;
pub mod shared;
pub mod main_loop;
pub mod flash;
pub mod regs;
pub mod emergency;
pub mod gpio_regs;
pub mod gpio_pin;
pub mod dma_buf;
pub mod capture_hal;
pub mod capture_generic;
pub mod comp_hal;
pub mod adc_hal;
pub mod adc_generic;
pub mod telem_hal;
pub mod isr;
pub mod isr_handlers;
pub mod init;
pub mod stub;
pub mod ws2812_hal;

// --- MCU-specific peripheral modules (one cfg per chip) ---
#[cfg(feature = "stm32g071")]
pub mod mcu_g071;
#[cfg(feature = "stm32f051")]
pub mod mcu_f051;
#[cfg(feature = "stm32l431")]
pub mod mcu_l431;
#[cfg(feature = "stm32g431")]
pub mod mcu_g431;
