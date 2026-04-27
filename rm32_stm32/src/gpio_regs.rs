//! Unified GPIO register access — safe, static, zero-cost.
//!
//! Each GPIO port is a zero-sized type implementing `GpioPort`.
//! All methods are static (no `&self`) — addresses are compile-time constants
//! after monomorphization. No `unsafe` leaks to call sites.
//!
//! Bridges the PAC accessor divergence (methods vs fields) in one place.

use crate::pac;

/// Safe GPIO port operations. Implemented per-port as a ZST.
/// All methods are static — the port address is baked in at compile time.
pub trait GpioPort {
    /// Atomic bit-set/reset via BSRR (write-only, no RMW needed).
    fn write_bsrr(val: u32);
    /// Read-modify-write MODER register.
    /// SAFETY is internal: single-core ISR at fixed priority = no preemption.
    fn modify_moder(f: impl FnOnce(u32) -> u32);
    /// Read ODR.
    fn read_odr() -> u32;
    /// Write ODR (for XOR toggle).
    fn write_odr(val: u32);
}

/// Macro to define a port ZST + GpioPort impl, bridging PAC accessor style.
macro_rules! define_port {
    // Method-accessor PACs (stm32g0-staging, stm32g4): gpio.bsrr()
    (method, $name:ident, $pac_periph:path) => {
        pub struct $name;
        impl GpioPort for $name {
            #[inline(always)]
            fn write_bsrr(val: u32) {
                // SAFETY: BSRR is write-only, bit-atomic — safe from any context.
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { gpio.bsrr().as_ptr().write_volatile(val); }
            }
            #[inline(always)]
            fn modify_moder(f: impl FnOnce(u32) -> u32) {
                // SAFETY: Single-core, ISR at fixed priority — no concurrent RMW.
                let gpio = unsafe { &*<$pac_periph>::PTR };
                let ptr = gpio.moder().as_ptr();
                unsafe { ptr.write_volatile(f(ptr.read_volatile())); }
            }
            #[inline(always)]
            fn read_odr() -> u32 {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { gpio.odr().as_ptr().read_volatile() }
            }
            #[inline(always)]
            fn write_odr(val: u32) {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { (gpio.odr().as_ptr() as *mut u32).write_volatile(val); }
            }
        }
    };
    // Field-accessor PACs (stm32f0, stm32l4): gpio.bsrr
    (field, $name:ident, $pac_periph:path) => {
        pub struct $name;
        impl GpioPort for $name {
            #[inline(always)]
            fn write_bsrr(val: u32) {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { (gpio.bsrr.as_ptr() as *mut u32).write_volatile(val); }
            }
            #[inline(always)]
            fn modify_moder(f: impl FnOnce(u32) -> u32) {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                let ptr = gpio.moder.as_ptr();
                unsafe { (ptr as *mut u32).write_volatile(f(ptr.read_volatile())); }
            }
            #[inline(always)]
            fn read_odr() -> u32 {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { gpio.odr.as_ptr().read_volatile() }
            }
            #[inline(always)]
            fn write_odr(val: u32) {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { (gpio.odr.as_ptr() as *mut u32).write_volatile(val); }
            }
        }
    };
}

// --- Port definitions per PAC style ---

#[cfg(any(feature = "stm32g071", feature = "stm32g431"))]
define_port!(method, PortA, pac::GPIOA);
#[cfg(any(feature = "stm32g071", feature = "stm32g431"))]
define_port!(method, PortB, pac::GPIOB);

#[cfg(any(feature = "stm32f051", feature = "stm32l431"))]
define_port!(field, PortA, pac::GPIOA);
#[cfg(any(feature = "stm32f051", feature = "stm32l431"))]
define_port!(field, PortB, pac::GPIOB);
