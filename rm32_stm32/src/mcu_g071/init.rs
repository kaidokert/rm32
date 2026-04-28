//! STM32G071 initialization.

use crate::init::InitResult;
use crate::phase::G0APhaseDriver;
use crate::timer::{Tim2Interval, Tim14Com};

pub fn init(dead_time: u8) -> InitResult<
    super::system::SystemControl,
    super::adc::AdcReader,
    super::telemetry_uart::TelemUart,
> {
    use stm32g0xx_hal::prelude::*;
    use stm32g0xx_hal::stm32;
    use stm32g0xx_hal::rcc::Config as RccConfig;
    use stm32g0xx_hal::time::Hertz;

    let dp = stm32::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(RccConfig::pll());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let _gpiob = dp.GPIOB.split(&mut rcc);

    let pwm = super::pwm::Tim1Pwm::new(
        dp.TIM1, gpioa.pa8, gpioa.pa9, gpioa.pa10,
        Hertz::from_raw(24_000), &mut rcc,
        dead_time,
    );
    let phase = G0APhaseDriver::new(false);
    let sys = super::system::SystemControl::new(dp.IWDG);
    super::comp_init::init_comp2();
    let comp = super::comparator::new_comparator();
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new();

    // DShot input capture
    super::input_capture::init_g071();
    let mut input = super::input_capture::new_capture();
    use rm32::hal::InputCapture;
    input.receive_dshot_dma();

    let adc = super::adc::new_adc();
    let _ = adc.init();
    let telem = super::telemetry_uart::TelemUart::init().unwrap_or_else(|_| super::telemetry_uart::TelemUart::post_init());

    // TIM6: 20kHz
    {
        let rcc_raw = unsafe { &*stm32::RCC::ptr() };
        rcc_raw.apbenr1().modify(|_, w| w.tim6en().set_bit());
        let tim6 = unsafe { &*stm32::TIM6::ptr() };
        tim6.psc().write(|w| unsafe { w.bits(0) });
        tim6.arr().write(|w| unsafe { w.bits(3199) });
        tim6.egr().write(|w| w.ug().set_bit());
        tim6.sr().write(|w| w.uif().clear_bit());
        tim6.dier().write(|w| w.uie().set_bit());
        tim6.cr1().write(|w| w.cen().set_bit());
    }

    unsafe {
        use stm32::{Interrupt, NVIC};
        NVIC::unmask(Interrupt::TIM6_DAC_LPTIM1);
        NVIC::unmask(Interrupt::TIM14);
        NVIC::unmask(Interrupt::ADC_COMP);
        NVIC::unmask(Interrupt::DMA1_CHANNEL1);
        NVIC::unmask(Interrupt::EXTI4_15);
    }
    let exti = unsafe { &*stm32::EXTI::ptr() };
    exti.imr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 15)) });

    let hal = crate::isr::TargetIsrHal { pwm, input, comp, interval, com_timer, phase };
    InitResult { hal, sys, adc, telem }
}
