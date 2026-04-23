//! TIM1 3-phase PWM output implementation.
//!
//! Uses stm32g0xx-hal's Pwm + PwmPin for init and per-channel duty.
//! Dead-time / complementary outputs and runtime ARR changes use raw PAC
//! (HAL doesn't expose these).

use crate::pac::TIM1;
use stm32g0xx_hal::gpio::{gpioa::*, DefaultMode};
use stm32g0xx_hal::rcc::Rcc;
use stm32g0xx_hal::time::Hertz;
use stm32g0xx_hal::timer::pwm::{Pwm, PwmExt, PwmPin};
use stm32g0xx_hal::timer::Channel;
use rm32::hal::PwmOutput;

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
        tim.bdtr().modify(|_, w| unsafe { w.dtg().bits(dead_time).moe().set_bit() });
        tim.ccer().modify(|_, w| {
            w.cc1ne().set_bit()
             .cc2ne().set_bit()
             .cc3ne().set_bit()
        });

        Self { _pwm: pwm, ch1, ch2, ch3 }
    }
}

impl PwmOutput for Tim1Pwm {
    fn set_duty_all(&mut self, duty: u16) {
        // PwmPin::set_duty writes to the channel's CCR register
        self.ch1.set_duty(duty );
        self.ch2.set_duty(duty );
        self.ch3.set_duty(duty );
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
        self.ch1.set_duty(val );
    }

    fn set_compare2(&mut self, val: u16) {
        self.ch2.set_duty(val );
    }

    fn set_compare3(&mut self, val: u16) {
        self.ch3.set_duty(val );
    }

    fn generate_update_event(&mut self) {
        // Force update event — not in HAL
        let tim = unsafe { &*TIM1::ptr() };
        tim.egr().write(|w| w.ug().set_bit());
    }
}
