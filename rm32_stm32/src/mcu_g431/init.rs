//! STM32G431 initialization.

use crate::init::InitResult;
use crate::phase::G0APhaseDriver;
use crate::timer::{Tim2Interval, Tim14Com};

pub fn init(dead_time: u8) -> InitResult<super::system::System, super::adc::G431Adc, super::telemetry_uart::G431TelemUart> {
    use stm32g4xx_hal::stm32 as pac;

    let rcc = unsafe { &*pac::RCC::PTR };
    let flash = unsafe { &*pac::FLASH::PTR };
    let gpioa = unsafe { &*pac::GPIOA::PTR };
    let tim1 = unsafe { &*pac::TIM1::PTR };
    let tim6 = unsafe { &*pac::TIM6::PTR };
    let exti = unsafe { &*pac::EXTI::PTR };

    // Clock: 170MHz via PLL from HSI16 (M=4, N=85, R=2 -> 16/4*85/2 = 170MHz)
    unsafe {
        // Flash latency = 4 wait states for 170MHz
        flash.acr().modify(|_, w| w.latency().bits(4));
        while flash.acr().read().latency().bits() != 4 {}

        // Configure PLL: PLLSRC=HSI16, M=4(3), N=85, R=2(0), PLLREN
        rcc.pllcfgr().write(|w| {
            w.pllsrc().bits(0b10)  // HSI16
             .pllm().bits(3)       // M=4 (M-1)
             .plln().bits(85)
             .pllr().bits(0)       // R=2 (00=/2)
             .pllren().set_bit()
        });

        // Enable PLL
        rcc.cr().modify(|_, w| w.pllon().set_bit());
        while rcc.cr().read().pllrdy().bit_is_clear() {}

        // Switch system clock to PLL
        rcc.cfgr().modify(|_, w| w.sw().bits(0b11));
        while rcc.cfgr().read().sws().bits() != 0b11 {}

        // Enable peripheral clocks
        rcc.ahb2enr().modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit());
        rcc.apb2enr().modify(|_, w| w.tim1en().set_bit().tim15en().set_bit().tim16en().set_bit());
        rcc.apb1enr1().modify(|_, w| w.tim2en().set_bit().tim6en().set_bit());

        // PA8/9/10 as AF6 (TIM1_CH1/2/3)
        gpioa.moder().modify(|_, w| {
            w.moder8().bits(0b10).moder9().bits(0b10).moder10().bits(0b10)
        });
        gpioa.afrh().modify(|_, w| {
            w.afrh8().bits(6).afrh9().bits(6).afrh10().bits(6)
        });
    }

    // TIM1 PWM: 170MHz / (ARR+1) = 24kHz -> ARR = 7082
    unsafe {
        tim1.psc().write(|w| w.psc().bits(0));
        tim1.arr().write(|w| w.arr().bits(crate::config::TIM1_AUTORELOAD as u32));
        tim1.ccmr1_output().write(|w| w.bits(0x6868));  // OC1/2 PWM mode 1
        tim1.ccmr2_output().write(|w| w.bits(0x0068));  // OC3 PWM mode 1
        tim1.ccer().write(|w| w.bits(0x555));            // CC1-3 + CC1N-3N enable
        tim1.bdtr().write(|w| w.bits(dead_time as u32 | (1 << 15))); // DT + MOE
        tim1.cr1().write(|w| w.cen().set_bit());
    }
    let pwm = super::pwm::Pwm::new();
    let phase = G0APhaseDriver::new(false);

    // COMP1+COMP2 init
    super::comp_init::init_comp();
    let comp = super::comparator::new_comparator();
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new();

    // Input capture (TIM15 + DMA1_CH1)
    super::input_capture::init_g431();
    let mut input = super::input_capture::new_capture();
    use rm32::hal::InputCapture;
    input.receive_dshot_dma();

    // ADC
    let adc = super::adc::new_adc();
    let _ = adc.init();

    // UART telemetry
    let telem = super::telemetry_uart::G431TelemUart::init().unwrap_or_else(|_| super::telemetry_uart::G431TelemUart::post_init());

    // TIM6: 170MHz / 8500 = 20kHz
    unsafe {
        tim6.psc().write(|w| w.psc().bits(0));
        tim6.arr().write(|w| w.arr().bits(8499));
        tim6.egr().write(|w| w.ug().set_bit());
        tim6.sr().write(|w| w.bits(0));
        tim6.dier().write(|w| w.uie().set_bit());
        tim6.cr1().write(|w| w.cen().set_bit());
    }

    // NVIC
    unsafe {
        use pac::{Interrupt, NVIC};
        NVIC::unmask(Interrupt::TIM6_DACUNDER);
        NVIC::unmask(Interrupt::TIM1_UP_TIM16);
        NVIC::unmask(Interrupt::COMP1_2_3);
        NVIC::unmask(Interrupt::DMA1_CH1);
        NVIC::unmask(Interrupt::EXTI15_10);
    }

    // EXTI line 15 (software-triggered by DMA TC)
    unsafe {
        exti.imr1().modify(|_, w| w.im15().set_bit());
    }

    let sys = super::system::System::new();
    let hal = crate::isr::TargetIsrHal { pwm, input, comp, interval, com_timer, phase };
    InitResult { hal, sys, adc, telem }
}
