//! Unified GPIO register access — safe, static, zero-cost.
//!
//! Each GPIO port is a zero-sized type implementing `GpioPort`.
//! All methods are static (no `&self`) — addresses are compile-time constants
//! after monomorphization. No `unsafe` leaks to call sites.
//!
//! Bridges the PAC accessor divergence (methods vs fields) in one place.
//! Port definitions (PortA, PortB) are in mcu_xxx/chip.rs via define_port! macro.

/// Safe GPIO port operations. Implemented per-port as a ZST.
/// All methods are static — the port address is baked in at compile time.
pub trait GpioPort {
    fn write_bsrr(val: u32);
    fn modify_moder(f: impl FnOnce(u32) -> u32);
    fn read_odr() -> u32;
    fn write_odr(val: u32);
}

/// Define a port ZST + GpioPort impl. Uses fully qualified paths so it can
/// be invoked from any module (mcu_xxx/chip.rs).
#[macro_export]
macro_rules! define_port {
    (method, $name:ident, $pac_periph:path) => {
        pub struct $name;
        impl $crate::gpio_regs::GpioPort for $name {
            #[inline(always)]
            fn write_bsrr(val: u32) {
                let gpio = unsafe { &*<$pac_periph>::PTR };
                unsafe { gpio.bsrr().as_ptr().write_volatile(val); }
            }
            #[inline(always)]
            fn modify_moder(f: impl FnOnce(u32) -> u32) {
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
    (field, $name:ident, $pac_periph:path) => {
        pub struct $name;
        impl $crate::gpio_regs::GpioPort for $name {
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
