//! TIM1 3-phase PWM output for STM32L431.

use rm32::hal::PwmOutput;

pub struct Pwm { _private: () }

impl Pwm {
    pub fn new() -> Self { Self { _private: () } }
}

impl PwmOutput for Pwm {
    fn set_duty_all(&mut self, d: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.ccr1.write(|w| unsafe { w.bits(d as u32) });
        tim1.ccr2.write(|w| unsafe { w.bits(d as u32) });
        tim1.ccr3.write(|w| unsafe { w.bits(d as u32) });
    }
    fn set_auto_reload(&mut self, a: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.arr.write(|w| unsafe { w.bits(a as u32) });
    }
    fn set_prescaler(&mut self, p: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.psc.write(|w| unsafe { w.bits(p as u32) });
    }
    fn set_compare1(&mut self, v: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.ccr1.write(|w| unsafe { w.bits(v as u32) });
    }
    fn set_compare2(&mut self, v: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.ccr2.write(|w| unsafe { w.bits(v as u32) });
    }
    fn set_compare3(&mut self, v: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.ccr3.write(|w| unsafe { w.bits(v as u32) });
    }
    fn generate_update_event(&mut self) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.egr.write(|w| w.ug().set_bit());
    }
    fn set_dead_time_override(&mut self, dtg: u16) {
        let tim1 = unsafe { &*stm32l4xx_hal::pac::TIM1::ptr() };
        tim1.bdtr.modify(|r, w| unsafe { w.bits(r.bits() | dtg as u32) });
    }
}
