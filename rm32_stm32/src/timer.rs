//! Timer implementations.
//!
//! TIM2: Interval timer — free-running at 2MHz
//! TIM14/TIM16: Commutation timer — one-shot at 2MHz
//! PSC derived from MCU config to achieve 2MHz regardless of clock speed.
//!
//! Each MCU uses `define_raw_timer!` to generate a zero-sized struct implementing
//! `RawTimer`. `Tim2Interval` and `ComTimerImpl` are generic over `T: RawTimer`,
//! giving IDEs and linters full visibility into the timer API.

use crate::mcu::ChipConfig;
use rm32::hal::{ComTimer, IntervalTimer};

/// Low-level timer register access — implemented per-MCU via `define_raw_timer!`.
///
/// All methods are `&self` on a zero-sized type — the struct carries no data,
/// it just selects which PAC peripheral the methods operate on.
pub trait RawTimer {
    fn read_cr1(&self) -> u32;
    fn write_cr1(&self, v: u32);
    fn read_cnt(&self) -> u32;
    fn write_cnt(&self, v: u32);
    fn write_psc(&self, v: u32);
    fn write_arr(&self, v: u32);
    fn write_egr(&self, v: u32);
    fn read_sr(&self) -> u32;
    fn write_sr(&self, v: u32);
    fn read_dier(&self) -> u32;
    fn write_dier(&self, v: u32);

    #[inline]
    fn modify_cr1(&self, f: impl FnOnce(u32) -> u32) {
        self.write_cr1(f(self.read_cr1()));
    }

    #[inline]
    fn modify_dier(&self, f: impl FnOnce(u32) -> u32) {
        self.write_dier(f(self.read_dier()));
    }
}

/// Generates a zero-sized struct implementing `RawTimer` for a PAC timer peripheral.
///
/// Two variants handle PAC accessor differences:
/// - `method`: G071/G431 PACs use `tim.cr1()`, `tim.sr()`, etc.
/// - `field`: F051/L431 PACs use `tim.cr1`, `tim.sr`, etc.
#[macro_export]
macro_rules! define_raw_timer {
    (method, $name:ident, $pac_periph:path) => {
        pub struct $name;

        impl $crate::timer::RawTimer for $name {
            // SAFETY: Each method accesses a single PAC peripheral via its singleton PTR.
            // Safe wrappers around unsafe PAC register access.
            #[inline]
            fn read_cr1(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.cr1().read().bits()
            }
            #[inline]
            fn write_cr1(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).cr1().write(|w| w.bits(v));
                }
            }
            #[inline]
            fn read_cnt(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.cnt().read().bits()
            }
            #[inline]
            fn write_cnt(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).cnt().write(|w| w.bits(v));
                }
            }
            #[inline]
            fn write_psc(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).psc().write(|w| w.bits(v));
                }
            }
            #[inline]
            fn write_arr(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).arr().write(|w| w.bits(v));
                }
            }
            #[inline]
            fn write_egr(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).egr().write(|w| w.bits(v));
                }
            }
            #[inline]
            fn read_sr(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.sr().read().bits()
            }
            #[inline]
            fn write_sr(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).sr().write(|w| w.bits(v));
                }
            }
            #[inline]
            fn read_dier(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.dier().read().bits()
            }
            #[inline]
            fn write_dier(&self, v: u32) {
                unsafe {
                    (&*<$pac_periph>::PTR).dier().write(|w| w.bits(v));
                }
            }
        }
    };
    (field, $name:ident, $pac_periph:path) => {
        pub struct $name;

        impl $crate::timer::RawTimer for $name {
            #[inline]
            fn read_cr1(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.cr1.read().bits()
            }
            #[inline]
            fn write_cr1(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).cr1.write(|w| w.bits(v)) }
            }
            #[inline]
            fn read_cnt(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.cnt.read().bits()
            }
            #[inline]
            fn write_cnt(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).cnt.write(|w| w.bits(v)) }
            }
            #[inline]
            fn write_psc(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).psc.write(|w| w.bits(v)) }
            }
            #[inline]
            fn write_arr(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).arr.write(|w| w.bits(v)) }
            }
            #[inline]
            fn write_egr(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).egr.write(|w| w.bits(v)) }
            }
            #[inline]
            fn read_sr(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.sr.read().bits()
            }
            #[inline]
            fn write_sr(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).sr.write(|w| w.bits(v)) }
            }
            #[inline]
            fn read_dier(&self) -> u32 {
                unsafe { &*<$pac_periph>::PTR }.dier.read().bits()
            }
            #[inline]
            fn write_dier(&self, v: u32) {
                unsafe { (&*<$pac_periph>::PTR).dier.write(|w| w.bits(v)) }
            }
        }
    };
}

// Raw timer types defined in mcu_xxx/chip.rs, re-exported via mcu::*.
use crate::mcu::{ComTimerRaw, Tim2Raw};

/// Reject a stale NVIC invocation after commutation was cancelled or rearmed.
/// TIM14/TIM16 uses UIF/UIE (bit zero); other status flags are left intact.
pub fn take_commutation_interrupt() -> bool {
    let raw = ComTimerRaw;
    if raw.read_sr() & raw.read_dier() & 1 == 0 {
        return false;
    }
    raw.write_sr(!1);
    true
}

/// TIM2 as free-running interval timer (2MHz, 0.5us/tick).
pub struct Tim2Interval {
    raw: Tim2Raw,
}

impl Default for Tim2Interval {
    fn default() -> Self {
        Self::new()
    }
}

impl Tim2Interval {
    pub fn new() -> Self {
        crate::mcu::enable_tim2_clock();

        let raw = Tim2Raw;
        raw.modify_cr1(|v| v & !(1 << 0)); // CEN=0
        raw.write_psc(crate::mcu::Chip::TIMER_PSC as u32);
        // ARR = 0xFFFF (16-bit wrap, AM32-matched). TIM2 is 32-bit on L4/G4,
        // but AM32's HAL_TIM_Base_Init writes only the low 16 bits, leaving
        // the upper half at 0. Commutation interval measurement uses u16
        // anyway. Matching this avoids any cross-wrap arithmetic surprises.
        raw.write_arr(0xFFFF);
        raw.write_egr(1); // UG
        raw.write_cnt(0);
        raw.modify_cr1(|v| v | (1 << 0)); // CEN=1
        Self { raw }
    }
}

impl IntervalTimer for Tim2Interval {
    fn count(&self) -> u32 {
        self.raw.read_cnt()
    }

    fn set_count(&mut self, val: u32) {
        self.raw.write_cnt(val);
    }
}

/// Free-running commutation timer (`TIM14` on G071/F051, `TIM16` on L431/G431).
///
/// The counter runs continuously from boot with `CR1 = CEN` (`ARPE = 0`) so
/// `set_and_enable` never stops or restarts `CEN`, eliminating stop/start
/// prescaler jitter. Keeping `ARPE = 0` ensures each arm's `ARR = timeout`
/// takes effect immediately for the current countdown (whereas `ARPE = 1`
/// without `UG` leaves the new `ARR` in preload until the previous reload
/// value wraps, which would stall startup on the `0xFFFF` boot reload).
pub struct Tim14Com {
    raw: ComTimerRaw,
}

impl Default for Tim14Com {
    fn default() -> Self {
        Self::new()
    }
}

impl Tim14Com {
    pub fn new() -> Self {
        crate::mcu::enable_com_timer_clock();

        let raw = ComTimerRaw;
        raw.write_psc(crate::mcu::Chip::TIMER_PSC as u32);
        raw.write_arr(0xFFFF);
        raw.write_egr(1); // UG — latch PSC into active registers
        // CR1 = CEN only, free-running. DELIBERATE divergence from
        // AM32's CR1=0x81 (ARPE on): with ARPE, an arm's ARR write sits
        // in preload until the PREVIOUS active ARR wraps — AM32 fires
        // one commutation stale, and the boot value (0xFFFF = 32 ms)
        // dead-starts rm32's polling path, which unlike AM32's inline
        // zcfoundroutine commutates through this timer's ISR. ARPE off
        // makes every arm's ARR immediately active: fires at exactly
        // the requested wait for both polling and interrupt modes.
        raw.modify_cr1(|v| v | (1 << 0));
        Self { raw }
    }
}

impl ComTimer for Tim14Com {
    fn set_and_enable(&mut self, timeout: u16) {
        // AM32-matched SET_AND_ENABLE_COM_INT (peripherals.h:19-21):
        // CNT=0, ARR=timeout, SR=0, UIE on, with no EGR.UG and no CR1.CEN
        // stop/start toggle. Firing EGR.UG here would assert UIF in the
        // prescaled timer clock domain after a back-to-back CPU `SR = 0`
        // write completes, leaving UIF set when UIE is enabled and firing
        // the commutation ISR prematurely.
        self.raw.write_cnt(0);
        self.raw.write_arr(timeout as u32);
        self.raw.write_sr(0);
        self.raw.modify_dier(|v| v | 1); // UIE
    }

    fn disable_interrupt(&mut self) {
        self.raw.modify_dier(|v| v & !1);
    }

    fn enable_interrupt(&mut self) {
        self.raw.modify_dier(|v| v | 1);
    }
}
