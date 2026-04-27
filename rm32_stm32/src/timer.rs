//! Timer implementations.
//!
//! TIM2: Interval timer — free-running at 2MHz
//! TIM14: Commutation timer — one-shot at 2MHz
//! PSC derived from MCU config to achieve 2MHz regardless of clock speed.

use crate::pac;
use crate::pac::RCC;
use rm32::hal::{IntervalTimer, ComTimer};

/// PAC-based timer register access. Bridges method vs field accessor styles.
/// G071/G431 use method accessors: tim.cr1(), tim.sr(), etc.
/// F051/L431 use field accessors: tim.cr1, tim.sr, etc.

macro_rules! define_timer_ops {
    (method, $mod_name:ident, $pac_periph:path) => {
        mod $mod_name {
            use super::pac;
            macro_rules! tim { () => { unsafe { &*<$pac_periph>::PTR } } }
            #[inline(always)]
            pub unsafe fn read_cr1() -> u32 { tim!().cr1().read().bits() }
            #[inline(always)]
            pub unsafe fn write_cr1(v: u32) { tim!().cr1().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn modify_cr1(f: impl FnOnce(u32) -> u32) { write_cr1(f(read_cr1())); }
            #[inline(always)]
            pub unsafe fn read_cnt() -> u32 { tim!().cnt().read().bits() }
            #[inline(always)]
            pub unsafe fn write_cnt(v: u32) { tim!().cnt().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_psc(v: u32) { tim!().psc().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_arr(v: u32) { tim!().arr().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_egr(v: u32) { tim!().egr().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_sr(v: u32) { tim!().sr().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn read_dier() -> u32 { tim!().dier().read().bits() }
            #[inline(always)]
            pub unsafe fn write_dier(v: u32) { tim!().dier().write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn modify_dier(f: impl FnOnce(u32) -> u32) { write_dier(f(read_dier())); }
        }
    };
    (field, $mod_name:ident, $pac_periph:path) => {
        mod $mod_name {
            use super::pac;
            macro_rules! tim { () => { unsafe { &*<$pac_periph>::PTR } } }
            #[inline(always)]
            pub unsafe fn read_cr1() -> u32 { tim!().cr1.read().bits() }
            #[inline(always)]
            pub unsafe fn write_cr1(v: u32) { tim!().cr1.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn modify_cr1(f: impl FnOnce(u32) -> u32) { write_cr1(f(read_cr1())); }
            #[inline(always)]
            pub unsafe fn read_cnt() -> u32 { tim!().cnt.read().bits() }
            #[inline(always)]
            pub unsafe fn write_cnt(v: u32) { tim!().cnt.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_psc(v: u32) { tim!().psc.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_arr(v: u32) { tim!().arr.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_egr(v: u32) { tim!().egr.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn write_sr(v: u32) { tim!().sr.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn read_dier() -> u32 { tim!().dier.read().bits() }
            #[inline(always)]
            pub unsafe fn write_dier(v: u32) { tim!().dier.write(|w| w.bits(v)); }
            #[inline(always)]
            pub unsafe fn modify_dier(f: impl FnOnce(u32) -> u32) { write_dier(f(read_dier())); }
        }
    };
}

// TIM2: method-style on G071/G431, field-style on F051/L431
#[cfg(any(feature = "stm32g071", feature = "stm32g431"))]
define_timer_ops!(method, tim2_ops, pac::TIM2);
#[cfg(any(feature = "stm32f051", feature = "stm32l431"))]
define_timer_ops!(field, tim2_ops, pac::TIM2);

// Commutation timer: TIM14 on G071/F051, TIM16 on L431/G431
#[cfg(feature = "stm32g071")]
define_timer_ops!(method, com_tim_ops, pac::TIM14);
#[cfg(feature = "stm32g431")]
define_timer_ops!(method, com_tim_ops, pac::TIM16);
#[cfg(feature = "stm32f051")]
define_timer_ops!(field, com_tim_ops, pac::TIM14);
#[cfg(feature = "stm32l431")]
define_timer_ops!(field, com_tim_ops, pac::TIM16);

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
        #[allow(unused_variables)] // Used via PAC on all MCU variants
        let rcc = unsafe { &*RCC::ptr() };
        #[cfg(feature = "stm32g071")]
        rcc.apbenr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
        #[cfg(feature = "stm32f051")]
        rcc.apb1enr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
        #[cfg(feature = "stm32l431")]
        rcc.apb1enr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN

        unsafe {
            tim2_ops::modify_cr1(|v| v & !(1 << 0)); // CEN=0
            tim2_ops::write_psc(crate::config::TIMER_PSC as u32);
            tim2_ops::write_arr(0xFFFF_FFFF);
            tim2_ops::write_egr(1); // UG
            tim2_ops::write_cnt(0);
            tim2_ops::modify_cr1(|v| v | (1 << 0)); // CEN=1
        }
        Self { _private: () }
    }
}

impl IntervalTimer for Tim2Interval {
    fn count(&self) -> u32 {
        unsafe { tim2_ops::read_cnt() }
    }

    fn set_count(&mut self, val: u32) {
        unsafe { tim2_ops::write_cnt(val); }
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
        // Enable TIM14 clock
        #[allow(unused_variables)] // Used via PAC on all MCU variants
        let rcc = unsafe { &*RCC::ptr() };
        #[cfg(feature = "stm32g071")]
        rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 15)) }); // TIM14EN
        #[cfg(feature = "stm32f051")]
        rcc.apb1enr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 8)) }); // TIM14EN
        #[cfg(feature = "stm32l431")]
        rcc.apb2enr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 17)) }); // TIM16EN

        unsafe {
            com_tim_ops::write_psc(crate::config::TIMER_PSC as u32);
            com_tim_ops::write_arr(0xFFFF);
            com_tim_ops::write_egr(1);
        }
        Self { _private: () }
    }
}

impl ComTimer for Tim14Com {
    fn set_and_enable(&mut self, timeout: u16) {
        unsafe {
            com_tim_ops::modify_cr1(|v| v & !(1 << 0));
            com_tim_ops::write_cnt(0);
            com_tim_ops::write_arr(timeout as u32);
            com_tim_ops::write_sr(0);
            com_tim_ops::modify_dier(|v| v | 1);
            com_tim_ops::modify_cr1(|v| v | (1 << 0));
        }
    }

    fn disable_interrupt(&mut self) {
        unsafe { com_tim_ops::modify_dier(|v| v & !1); }
    }

    fn enable_interrupt(&mut self) {
        unsafe { com_tim_ops::modify_dier(|v| v | 1); }
    }
}
