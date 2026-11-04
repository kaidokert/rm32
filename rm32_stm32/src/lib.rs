//! RM32 STM32 HAL implementation.
//!
//! Supports STM32G071, STM32F051, STM32L431, STM32G431 via feature flags.
//! MCU-specific configuration is centralized in `mcu.rs`.

#![no_std]

/// Firmware debug print.
#[macro_export]
macro_rules! dprintln {
    ($($arg:tt)*) => {{
        $crate::dline(format_args!($($arg)*));
    }};
}

/// Format `args` into RTT outside a critical section.
///
/// `rtt_target::rprintln!` runs `core::fmt::write` inside
/// `critical_section::with` (disabling all interrupts, including priority 0).
/// Calling `write_fmt` here runs formatting with interrupts enabled; each
/// already-formatted fragment goes to RTT under its own short critical section
/// (a buffer copy), followed by the trailing newline.
pub fn dline(args: core::fmt::Arguments) {
    struct Sinks;
    impl core::fmt::Write for Sinks {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            fragment(s);
            Ok(())
        }
    }
    fn fragment(s: &str) {
        rtt_target::print_impl::write_str(0, s);
    }
    use core::fmt::Write as _;
    let _ = Sinks.write_fmt(args);
    fragment("\n");
}

pub mod board_ext;
pub mod mcu;
pub use mcu::pac;

// --- Shared across all MCUs (zero cfg) ---
pub mod adc_generic;
pub mod adc_hal;
pub mod bridge;
pub mod capture_generic;
pub mod capture_hal;
pub mod comp_hal;
pub mod comparator;
pub mod delay;
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

pub mod platform;
