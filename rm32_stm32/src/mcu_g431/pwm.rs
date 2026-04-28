//! TIM1 3-phase PWM output for STM32G431.

use rm32::hal::PwmOutput;

pub struct Pwm {
    _private: (),
}

impl Pwm {
    pub fn new() -> Self {
        Self { _private: () }
    }
}

impl PwmOutput for Pwm {
    fn set_prescaler(&mut self, psc: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.psc().write(|w| w.bits(psc as u32));
        }
    }
    fn set_auto_reload(&mut self, arr: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.arr().write(|w| w.bits(arr as u32));
        }
    }
    fn set_duty_all(&mut self, duty: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.ccr1().write(|w| w.bits(duty as u32));
            tim1.ccr2().write(|w| w.bits(duty as u32));
            tim1.ccr3().write(|w| w.bits(duty as u32));
        }
    }
    fn set_compare1(&mut self, val: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.ccr1().write(|w| w.bits(val as u32));
        }
    }
    fn set_compare2(&mut self, val: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.ccr2().write(|w| w.bits(val as u32));
        }
    }
    fn set_compare3(&mut self, val: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.ccr3().write(|w| w.bits(val as u32));
        }
    }
    fn generate_update_event(&mut self) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.egr().write(|w| w.ug().set_bit());
        }
    }
    fn set_dead_time_override(&mut self, dead_time: u16) {
        let tim1 = unsafe { &*stm32g4xx_hal::stm32::TIM1::PTR };
        unsafe {
            tim1.bdtr()
                .modify(|r, w| w.bits((r.bits() & !0xFF) | (dead_time as u32 & 0xFF)));
        }
    }
}
