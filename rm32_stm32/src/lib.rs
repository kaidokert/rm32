//! RM32 STM32 HAL implementation.
//!
//! Supports STM32G071 and STM32F051 via feature flags.

#![no_std]

pub mod mcu;
pub use mcu::pac;
pub use mcu::config;

// Shared across MCUs
pub mod comparator;
pub mod timer;
pub mod phase;
pub mod shared;
pub mod main_loop;
pub mod flash;
pub mod regs;
pub mod isr;
pub mod isr_handlers;
pub mod init;
pub mod stub;
pub mod ws2812_hal;

// MCU-specific ISR vectors
#[cfg(feature = "stm32g071")]
pub mod interrupts_g071;
#[cfg(feature = "stm32f051")]
pub mod interrupts_f051;
#[cfg(feature = "stm32l431")]
pub mod interrupts_l431;

// G071-specific peripheral modules
#[cfg(feature = "stm32g071")]
pub mod pwm;
#[cfg(feature = "stm32g071")]
pub mod system;
#[cfg(feature = "stm32g071")]
pub mod input_capture;
#[cfg(feature = "stm32g071")]
pub mod comp_init;
#[cfg(feature = "stm32g071")]
pub mod telemetry_uart;
#[cfg(feature = "stm32g071")]
pub mod adc;

// F051-specific peripheral modules
#[cfg(feature = "stm32f051")]
pub mod input_capture_f051;
#[cfg(feature = "stm32f051")]
pub mod comp_init_f051;
#[cfg(feature = "stm32f051")]
pub mod telemetry_uart_f051;
#[cfg(feature = "stm32f051")]
pub mod adc_f051;

// L431-specific peripheral modules
#[cfg(feature = "stm32l431")]
pub mod input_capture_l431;
#[cfg(feature = "stm32l431")]
pub mod comp_init_l431;
#[cfg(feature = "stm32l431")]
pub mod telemetry_uart_l431;
#[cfg(feature = "stm32l431")]
pub mod adc_l431;
