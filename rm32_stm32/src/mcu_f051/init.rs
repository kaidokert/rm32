//! STM32F051 initialization.

use crate::init::InitResult;
use crate::phase::G0APhaseDriver;
use crate::timer::{Tim2Interval, Tim14Com};

pub fn init(dead_time: u8) -> InitResult<super::system::System, super::adc::F051Adc, super::telemetry_uart::F051TelemUart> {
    use stm32f0xx_hal::prelude::*;
    use stm32f0xx_hal::pac;

    let mut dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // Clock: 48MHz PLL from HSI
    let _rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut dp.FLASH);

    let rcc_pac = unsafe { &*pac::RCC::ptr() };
    unsafe {
        // Enable GPIO A/B clocks
        rcc_pac.ahbenr.modify(|_, w| w.iopaen().set_bit().iopben().set_bit());
        // Enable TIM1 (APB2ENR bit 11)
        rcc_pac.apb2enr.modify(|_, w| w.tim1en().set_bit());

        // PA8/9/10 as AF2 (TIM1_CH1/2/3)
        let gpioa = &*pac::GPIOA::ptr();
        gpioa.moder.modify(|r, w| {
            w.bits((r.bits() & !(0b11<<16 | 0b11<<18 | 0b11<<20)) | (0b10<<16 | 0b10<<18 | 0b10<<20))
        });
        gpioa.afrh.modify(|r, w| {
            w.bits((r.bits() & !(0xFFF)) | (2 | 2<<4 | 2<<8))
        });
    }

    // TIM1 PWM: 48MHz/2000 = 24kHz
    unsafe {
        let tim1 = &*pac::TIM1::ptr();
        tim1.psc.write(|w| w.bits(0));
        tim1.arr.write(|w| w.bits(1999));
        tim1.ccmr1_output().write(|w| w.bits(0x6868));   // OC1/2 PWM mode 1
        tim1.ccmr2_output().write(|w| w.bits(0x0068));   // OC3 PWM mode 1
        tim1.ccer.write(|w| w.bits(0x555));               // CC1-3 + CC1N-3N enable
        tim1.bdtr.write(|w| w.bits(dead_time as u32 | (1 << 15))); // DT + MOE
        tim1.cr1.write(|w| w.cen().set_bit());
    }
    let pwm = super::pwm::Pwm::new();
    let phase = G0APhaseDriver::new(false); // same pins for F0_A

    // COMP1 init
    super::comp_init::init_comp1();
    let comp = super::comparator::new_comparator();

    // Timers
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new();

    // Input capture (TIM15 + DMA1_CH5)
    super::input_capture::init_f051();
    let mut input = super::input_capture::new_capture();
    use rm32::hal::InputCapture;
    input.receive_dshot_dma();

    // ADC
    let adc = super::adc::new_adc();
    let _ = adc.init();

    // UART telemetry
    let telem = super::telemetry_uart::F051TelemUart::init().unwrap_or_else(|_| super::telemetry_uart::F051TelemUart::post_init());

    // TIM6: 48MHz/2400 = 20kHz
    unsafe {
        rcc_pac.apb1enr.modify(|_, w| w.tim6en().set_bit());
        let tim6 = &*pac::TIM6::ptr();
        tim6.psc.write(|w| w.bits(0));
        tim6.arr.write(|w| w.bits(2399));
        tim6.egr.write(|w| w.ug().set_bit());
        tim6.sr.write(|w| w.uif().clear_bit());
        tim6.dier.write(|w| w.uie().set_bit());
        tim6.cr1.write(|w| w.cen().set_bit());
    }

    // NVIC
    unsafe {
        use pac::{Interrupt, NVIC};
        NVIC::unmask(Interrupt::TIM6_DAC);
        NVIC::unmask(Interrupt::TIM14);
        NVIC::unmask(Interrupt::ADC_COMP);
        NVIC::unmask(Interrupt::DMA1_CH4_5_6_7_DMA2_CH3_4_5);
        NVIC::unmask(Interrupt::EXTI4_15);
    }

    // Enable EXTI line 15 (software-triggered by DMA TC)
    unsafe {
        let exti = &*pac::EXTI::ptr();
        exti.imr.modify(|r, w| w.bits(r.bits() | (1 << 15)));
    }

    let sys = super::system::System::new();

    let hal = crate::isr::TargetIsrHal { pwm, input, comp, interval, com_timer, phase };
    InitResult { hal, sys, adc, telem }
}
