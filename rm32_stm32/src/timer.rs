//! Timer implementations.
//!
//! TIM2: Interval timer — free-running at 2MHz
//! TIM14: Commutation timer — one-shot at 2MHz
//! PSC derived from MCU config to achieve 2MHz regardless of clock speed.

use crate::pac::RCC;
use rm32::hal::{IntervalTimer, ComTimer};

// Timer register offsets (same for all STM32 timers)
const CR1: u32 = 0x00;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;

#[cfg(feature = "stm32g071")]
const TIM2_BASE: u32 = 0x4000_0000;
#[cfg(feature = "stm32g071")]
const TIM14_BASE: u32 = 0x4000_2000;

#[cfg(feature = "stm32f051")]
const TIM2_BASE: u32 = 0x4000_0000;
#[cfg(feature = "stm32f051")]
const TIM14_BASE: u32 = 0x4000_2000;

#[cfg(feature = "stm32l431")]
const TIM2_BASE: u32 = 0x4000_0000;
// L431 uses TIM16 in place of TIM14 (TIM16 base = 0x4001_4400)
#[cfg(feature = "stm32l431")]
const TIM14_BASE: u32 = 0x4001_4400;

use crate::regs::{write_off as write_reg, read_off as read_reg, modify_off as modify_reg};

/// TIM2 as free-running interval timer (2MHz, 0.5µs/tick).
pub struct Tim2Interval {
    _private: (),
}

impl Tim2Interval {
    pub fn new() -> Self {
        // Enable TIM2 clock
        let rcc = unsafe { &*RCC::ptr() };
        #[cfg(feature = "stm32g071")]
        rcc.apbenr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
        #[cfg(feature = "stm32f051")]
        unsafe {
            // APB1ENR at RCC base + 0x1C on F0
            let apb1enr = (RCC::ptr() as u32 + 0x1C) as *mut u32;
            apb1enr.write_volatile(apb1enr.read_volatile() | (1 << 0)); // TIM2EN
        }
        #[cfg(feature = "stm32l431")]
        unsafe {
            // APB1ENR1 at RCC base + 0x58 on L4
            let apb1enr1 = (RCC::ptr() as u32 + 0x58) as *mut u32;
            apb1enr1.write_volatile(apb1enr1.read_volatile() | (1 << 0)); // TIM2EN
        }

        unsafe {
            modify_reg(TIM2_BASE, CR1, |v| v & !(1 << 0)); // CEN=0
            write_reg(TIM2_BASE, PSC, crate::config::TIMER_PSC as u32);
            write_reg(TIM2_BASE, ARR, 0xFFFF_FFFF);
            write_reg(TIM2_BASE, EGR, 1); // UG
            write_reg(TIM2_BASE, CNT, 0);
            modify_reg(TIM2_BASE, CR1, |v| v | (1 << 0)); // CEN=1
        }
        Self { _private: () }
    }
}

impl IntervalTimer for Tim2Interval {
    fn count(&self) -> u32 {
        unsafe { read_reg(TIM2_BASE, CNT) }
    }

    fn set_count(&mut self, val: u32) {
        unsafe { write_reg(TIM2_BASE, CNT, val); }
    }
}

/// TIM14 as one-shot commutation timer (2MHz, 0.5µs/tick).
pub struct Tim14Com {
    _private: (),
}

impl Tim14Com {
    pub fn new() -> Self {
        // Enable TIM14 clock
        let rcc = unsafe { &*RCC::ptr() };
        #[cfg(feature = "stm32g071")]
        rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 15)) }); // TIM14EN (APB2)
        #[cfg(feature = "stm32f051")]
        unsafe {
            let apb1enr = (RCC::ptr() as u32 + 0x1C) as *mut u32;
            apb1enr.write_volatile(apb1enr.read_volatile() | (1 << 8)); // TIM14EN
        }
        #[cfg(feature = "stm32l431")]
        unsafe {
            // TIM16EN is bit 17 in APB2ENR (RCC base + 0x60 on L4)
            let apb2enr = (RCC::ptr() as u32 + 0x60) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 17)); // TIM16EN
        }

        unsafe {
            write_reg(TIM14_BASE, PSC, crate::config::TIMER_PSC as u32);
            write_reg(TIM14_BASE, ARR, 0xFFFF);
            write_reg(TIM14_BASE, EGR, 1);
        }
        Self { _private: () }
    }
}

impl ComTimer for Tim14Com {
    fn set_and_enable(&mut self, timeout: u16) {
        unsafe {
            modify_reg(TIM14_BASE, CR1, |v| v & !(1 << 0));
            write_reg(TIM14_BASE, CNT, 0);
            write_reg(TIM14_BASE, ARR, timeout as u32);
            write_reg(TIM14_BASE, SR, 0);
            modify_reg(TIM14_BASE, DIER, |v| v | 1);
            modify_reg(TIM14_BASE, CR1, |v| v | (1 << 0));
        }
    }

    fn disable_interrupt(&mut self) {
        unsafe { modify_reg(TIM14_BASE, DIER, |v| v & !1); }
    }

    fn enable_interrupt(&mut self) {
        unsafe { modify_reg(TIM14_BASE, DIER, |v| v | 1); }
    }
}
