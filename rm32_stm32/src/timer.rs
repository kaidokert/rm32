//! Timer implementations.
//!
//! TIM2: Interval timer — free-running at 2MHz
//! TIM14: Commutation timer — one-shot at 2MHz
//! PSC derived from MCU config to achieve 2MHz regardless of clock speed.

use rm32::hal::{ComTimer, IntervalTimer};

/// PAC-based timer register access. Bridges method vs field accessor styles.
/// G071/G431 use method accessors: tim.cr1(), tim.sr(), etc.
/// F051/L431 use field accessors: tim.cr1, tim.sr, etc.

#[macro_export]
macro_rules! define_timer_ops {
    (method, $mod_name:ident, $pac_periph:path) => {
        pub mod $mod_name {
            // SAFETY: All unsafe is encapsulated here. Callers see safe functions.
            macro_rules! tim {
                () => {
                    unsafe { &*<$pac_periph>::PTR }
                };
            }
            #[inline]
            pub fn read_cr1() -> u32 {
                tim!().cr1().read().bits()
            }
            #[inline]
            pub fn write_cr1(v: u32) {
                unsafe {
                    tim!().cr1().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn modify_cr1(f: impl FnOnce(u32) -> u32) {
                write_cr1(f(read_cr1()));
            }
            #[inline]
            pub fn read_cnt() -> u32 {
                tim!().cnt().read().bits()
            }
            #[inline]
            pub fn write_cnt(v: u32) {
                unsafe {
                    tim!().cnt().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_psc(v: u32) {
                unsafe {
                    tim!().psc().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_arr(v: u32) {
                unsafe {
                    tim!().arr().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_egr(v: u32) {
                unsafe {
                    tim!().egr().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_sr(v: u32) {
                unsafe {
                    tim!().sr().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn read_dier() -> u32 {
                tim!().dier().read().bits()
            }
            #[inline]
            pub fn write_dier(v: u32) {
                unsafe {
                    tim!().dier().write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn modify_dier(f: impl FnOnce(u32) -> u32) {
                write_dier(f(read_dier()));
            }
        }
    };
    (field, $mod_name:ident, $pac_periph:path) => {
        pub mod $mod_name {
            macro_rules! tim {
                () => {
                    unsafe { &*<$pac_periph>::PTR }
                };
            }
            #[inline]
            pub fn read_cr1() -> u32 {
                tim!().cr1.read().bits()
            }
            #[inline]
            pub fn write_cr1(v: u32) {
                unsafe {
                    tim!().cr1.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn modify_cr1(f: impl FnOnce(u32) -> u32) {
                write_cr1(f(read_cr1()));
            }
            #[inline]
            pub fn read_cnt() -> u32 {
                tim!().cnt.read().bits()
            }
            #[inline]
            pub fn write_cnt(v: u32) {
                unsafe {
                    tim!().cnt.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_psc(v: u32) {
                unsafe {
                    tim!().psc.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_arr(v: u32) {
                unsafe {
                    tim!().arr.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_egr(v: u32) {
                unsafe {
                    tim!().egr.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn write_sr(v: u32) {
                unsafe {
                    tim!().sr.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn read_dier() -> u32 {
                tim!().dier.read().bits()
            }
            #[inline]
            pub fn write_dier(v: u32) {
                unsafe {
                    tim!().dier.write(|w| w.bits(v));
                }
            }
            #[inline]
            pub fn modify_dier(f: impl FnOnce(u32) -> u32) {
                write_dier(f(read_dier()));
            }
        }
    };
}

// Timer ops (tim2_ops, com_tim_ops) defined in mcu_xxx/chip.rs, re-exported via mcu::*.
use crate::mcu::{com_tim_ops, tim2_ops};

/// TIM2 as free-running interval timer (2MHz, 0.5us/tick).
pub struct Tim2Interval {
    _private: (),
}

impl Default for Tim2Interval {
    fn default() -> Self {
        Self::new()
    }
}

impl Tim2Interval {
    pub fn new() -> Self {
        // Enable TIM2 clock
        crate::mcu::enable_tim2_clock();

        tim2_ops::modify_cr1(|v| v & !(1 << 0)); // CEN=0
        tim2_ops::write_psc(crate::config::TIMER_PSC as u32);
        tim2_ops::write_arr(0xFFFF_FFFF);
        tim2_ops::write_egr(1); // UG
        tim2_ops::write_cnt(0);
        tim2_ops::modify_cr1(|v| v | (1 << 0)); // CEN=1
        Self { _private: () }
    }
}

impl IntervalTimer for Tim2Interval {
    fn count(&self) -> u32 {
        tim2_ops::read_cnt()
    }

    fn set_count(&mut self, val: u32) {
        tim2_ops::write_cnt(val);
    }
}

/// TIM14 as one-shot commutation timer (2MHz, 0.5us/tick).
pub struct Tim14Com {
    _private: (),
}

impl Default for Tim14Com {
    fn default() -> Self {
        Self::new()
    }
}

impl Tim14Com {
    pub fn new() -> Self {
        crate::mcu::enable_com_timer_clock();

        com_tim_ops::write_psc(crate::config::TIMER_PSC as u32);
        com_tim_ops::write_arr(0xFFFF);
        com_tim_ops::write_egr(1);
        Self { _private: () }
    }
}

impl ComTimer for Tim14Com {
    fn set_and_enable(&mut self, timeout: u16) {
        com_tim_ops::modify_cr1(|v| v & !(1 << 0));
        com_tim_ops::write_cnt(0);
        com_tim_ops::write_arr(timeout as u32);
        com_tim_ops::write_sr(0);
        com_tim_ops::modify_dier(|v| v | 1);
        com_tim_ops::modify_cr1(|v| v | (1 << 0));
    }

    fn disable_interrupt(&mut self) {
        com_tim_ops::modify_dier(|v| v & !1);
    }

    fn enable_interrupt(&mut self) {
        com_tim_ops::modify_dier(|v| v | 1);
    }
}
