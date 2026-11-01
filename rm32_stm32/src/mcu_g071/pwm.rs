//! TIM1 3-phase PWM output implementation.
//!
//! Uses stm32g0xx-hal's Pwm + PwmPin for init and per-channel duty.
//! Dead-time / complementary outputs and runtime ARR changes use raw PAC
//! (HAL doesn't expose these).

use crate::bridge::TimerPwm;
use crate::pac::TIM1;
use stm32g0xx_hal::gpio::{DefaultMode, gpioa::*};
use stm32g0xx_hal::rcc::Rcc;
use stm32g0xx_hal::time::Hertz;
use stm32g0xx_hal::timer::Channel;
use stm32g0xx_hal::timer::pwm::{Pwm, PwmExt, PwmPin};

type Channel1 = Channel<0>;
type Channel2 = Channel<1>;
type Channel3 = Channel<2>;

/// TIM1 3-phase PWM with complementary outputs.
/// Holds 3 PwmPin objects for type-safe per-channel duty cycle control.
pub struct Tim1Pwm {
    _pwm: Pwm<TIM1>,
    ch1: PwmPin<TIM1, Channel1>,
    ch2: PwmPin<TIM1, Channel2>,
    ch3: PwmPin<TIM1, Channel3>,
}

impl Tim1Pwm {
    /// Initialize TIM1 PWM at the given frequency, binding PA8/PA9/PA10.
    ///
    /// `freq` sets the base PWM frequency (typically 24kHz for ESC).
    /// `dead_time` configures the DTG field in BDTR for FET dead-time insertion.
    pub fn new(
        tim1: TIM1,
        pa8: PA8<DefaultMode>,
        pa9: PA9<DefaultMode>,
        pa10: PA10<DefaultMode>,
        freq: Hertz,
        rcc: &mut Rcc,
        dead_time: u8,
    ) -> Self {
        // HAL handles: clock enable, reset, PSC/ARR from freq, counter start
        let pwm = tim1.pwm(freq, rcc);

        // bind_pin: configures GPIO alternate function, returns typed PwmPin
        let ch1 = pwm.bind_pin(pa8);
        let ch2 = pwm.bind_pin(pa9);
        let ch3 = pwm.bind_pin(pa10);

        // Advanced features not covered by HAL: dead-time, MOE, complementary outputs
        let tim = unsafe { &*TIM1::ptr() };
        tim.bdtr()
            .modify(|_, w| unsafe { w.dtg().bits(dead_time).moe().set_bit() });
        // Complete TIM1 output-compare setup not performed by `stm32g0xx-hal`'s
        // `bind_pin` (which leaves CCMR1/CCMR2 at 0 = Frozen and only sets
        // complementary CCxNE bits in CCER, preventing the high-side channels
        // from switching). Matches AM32 G071 MX_TIM1_Init: PWM mode 1 + OCxPE
        // on CH1..CH4, ARPE enabled, and CCxE + CCxNE on CH1..CH3.
        tim.ccmr1_output().write(|w| unsafe { w.bits(0x6868) });
        tim.ccmr2_output().write(|w| unsafe { w.bits(0x6868) });
        tim.ccer().write(|w| unsafe { w.bits(0x0555) });
        tim.cr1()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 7)) }); // ARPE
        tim.egr().write(|w| w.ug().set_bit()); // load preloaded ARR/CCR

        // Low-side pins PA7 (CH1N), PB0 (CH2N), PB1 (CH3N): select AF2
        // (TIM1) in AFRL. The phase driver only flips MODER between output
        // and alternate per step — without this the "alternate" low side
        // would route AF0 (SPI1/TIM14) to the gate input. Then park all six
        // gate pins as push-pull outputs driven low until the first step.
        // SAFETY: boot-time single-owner GPIOA/GPIOB configuration.
        unsafe {
            let gpioa = &*crate::pac::GPIOA::ptr();
            let gpiob = &*crate::pac::GPIOB::ptr();
            gpioa
                .afrl()
                .modify(|r, w| w.bits((r.bits() & !(0xF << 28)) | (2 << 28)));
            gpiob
                .afrl()
                .modify(|r, w| w.bits((r.bits() & !0xFF) | 2 | (2 << 4)));
            gpioa.bsrr().write(|w| {
                w.bits((1 << (7 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << (10 + 16)))
            });
            gpiob.bsrr().write(|w| w.bits((1 << 16) | (1 << 17)));
            gpioa.moder().modify(|r, w| {
                let m = (0b11 << 14) | (0b11 << 16) | (0b11 << 18) | (0b11 << 20);
                w.bits((r.bits() & !m) | (0b01 << 14) | (0b01 << 16) | (0b01 << 18) | (0b01 << 20))
            });
            gpiob
                .moder()
                .modify(|r, w| w.bits((r.bits() & !0b1111) | 0b0101));
        }

        Self {
            _pwm: pwm,
            ch1,
            ch2,
            ch3,
        }
    }
}

/// TIM1 main output enable (BDTR.MOE) state.
#[inline]
pub fn moe() -> bool {
    let tim = unsafe { &*TIM1::ptr() };
    tim.bdtr().read().moe().bit_is_set()
}

/// Set/clear TIM1 BDTR.MOE. With MOE = 0 (OSSI = 0) every TIM1 output is
/// disabled regardless of CCER, so a pin left in alternate mode cannot
/// switch a gate.
#[inline]
pub fn set_moe(on: bool) {
    let tim = unsafe { &*TIM1::ptr() };
    tim.bdtr().modify(|_, w| w.moe().bit(on));
}

impl TimerPwm for Tim1Pwm {
    fn set_duty_all(&mut self, duty: u16) {
        // PwmPin::set_duty writes to the channel's CCR register
        self.ch1.set_duty(duty);
        self.ch2.set_duty(duty);
        self.ch3.set_duty(duty);
    }

    fn set_auto_reload(&mut self, arr: u16) {
        // Runtime ARR change (variable PWM) — not in HAL's Pwm API
        let tim = unsafe { &*TIM1::ptr() };
        tim.arr().write(|w| unsafe { w.arr().bits(arr) });
    }

    fn set_prescaler(&mut self, psc: u16) {
        // Runtime PSC change — not in HAL
        let tim = unsafe { &*TIM1::ptr() };
        tim.psc().write(|w| unsafe { w.psc().bits(psc) });
    }

    fn set_compare1(&mut self, val: u16) {
        self.ch1.set_duty(val);
    }

    fn set_compare2(&mut self, val: u16) {
        self.ch2.set_duty(val);
    }

    fn set_compare3(&mut self, val: u16) {
        self.ch3.set_duty(val);
    }

    fn generate_update_event(&mut self) {
        let tim = unsafe { &*TIM1::ptr() };
        tim.egr().write(|w| w.ug().set_bit());
    }

    fn set_dead_time_override(&mut self, dtg: u16) {
        let tim = unsafe { &*TIM1::ptr() };
        tim.bdtr()
            .modify(|r, w| unsafe { w.bits(r.bits() | dtg as u32) });
    }
}
