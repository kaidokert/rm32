//! STM32L431 initialization.

use crate::init::InitResult;
use crate::mcu::ChipConfig;
use crate::phase::G0APhaseDriver;
use crate::timer::{Tim2Interval, Tim14Com};

pub fn init(
    dead_time: u8,
) -> InitResult<super::system::System, super::adc::L431Adc, super::telemetry_uart::L431TelemUart> {
    use stm32l4xx_hal::pac;
    use stm32l4xx_hal::prelude::*;

    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // Clock: 80MHz from MSI via PLL
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80_000_000u32.Hz())
        .freeze(&mut flash.acr, &mut pwr);
    let _ = clocks;

    let rcc_pac = unsafe { &*pac::RCC::ptr() };
    unsafe {
        // Enable GPIOA, GPIOB (AHB2ENR bits 0, 1)
        rcc_pac
            .ahb2enr
            .modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit());
        // Enable TIM1 (APB2ENR bit 11)
        rcc_pac.apb2enr.modify(|_, w| w.tim1en().set_bit());

        // PA8/9/10 as AF1 (TIM1_CH1/2/3 on L4 = AF1, not AF2)
        let gpioa = &*pac::GPIOA::ptr();
        gpioa.moder.modify(|r, w| {
            w.bits(
                (r.bits() & !(0b11 << 16 | 0b11 << 18 | 0b11 << 20))
                    | (0b10 << 16 | 0b10 << 18 | 0b10 << 20),
            )
        });
        gpioa.afrh.modify(|r, w| {
            w.bits((r.bits() & !(0xFFF)) | (1 | 1 << 4 | 1 << 8)) // AF1
        });
    }

    // TIM1 PWM: 80MHz / (ARR+1) = 24kHz -> ARR = 3332
    unsafe {
        let tim1 = &*pac::TIM1::ptr();
        tim1.psc.write(|w| w.bits(0));
        tim1.arr
            .write(|w| w.bits(super::chip::Chip::TIM1_AUTORELOAD as u32));
        tim1.ccmr1_output().write(|w| w.bits(0x6868)); // OC1/2 PWM mode 1
        tim1.ccmr2_output().write(|w| w.bits(0x0068)); // OC3 PWM mode 1
        tim1.ccer.write(|w| w.bits(0x555)); // CC1-3 + CC1N-3N enable
        tim1.bdtr.write(|w| w.bits(dead_time as u32 | (1 << 15))); // DT + MOE
        tim1.cr1.write(|w| w.cen().set_bit());
    }
    let pwm = super::pwm::Pwm::new();
    let phase = G0APhaseDriver::new(false); // same pins for L4_N

    // COMP2 init
    super::comp_init::init_comp2();
    let comp = super::comparator::new_comparator();
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new(); // L431 uses TIM16, but TIM14 struct works (same register layout)

    // Input capture (TIM15 + DMA1_CH5)
    super::input_capture::init_l431();
    let mut input = super::input_capture::new_capture();
    use rm32::hal::InputCapture;
    input.receive_dshot_dma();

    // ADC
    let adc = super::adc::new_adc();
    let _ = adc.init();

    // UART telemetry
    let telem = super::telemetry_uart::L431TelemUart::init()
        .unwrap_or_else(|_| super::telemetry_uart::L431TelemUart::post_init());

    // TIM6: 80MHz / 4000 = 20kHz
    unsafe {
        // Enable TIM6 (APB1ENR1 bit 4)
        rcc_pac.apb1enr1.modify(|_, w| w.tim6en().set_bit());
        let tim6 = &*pac::TIM6::ptr();
        tim6.psc.write(|w| w.bits(0));
        tim6.arr.write(|w| w.bits(3999));
        tim6.egr.write(|w| w.ug().set_bit());
        tim6.sr.write(|w| w.uif().clear_bit());
        tim6.dier.write(|w| w.uie().set_bit());
        tim6.cr1.write(|w| w.cen().set_bit());
    }

    // NVIC
    unsafe {
        use pac::{Interrupt, NVIC};
        NVIC::unmask(Interrupt::TIM6_DACUNDER);
        NVIC::unmask(Interrupt::TIM1_UP_TIM16);
        NVIC::unmask(Interrupt::COMP);
        NVIC::unmask(Interrupt::DMA1_CH5);
        NVIC::unmask(Interrupt::EXTI15_10);
    }

    // Enable EXTI line 15 (software-triggered by DMA TC)
    unsafe {
        let exti = &*pac::EXTI::ptr();
        exti.imr1.modify(|r, w| w.bits(r.bits() | (1 << 15)));
    }

    let sys = super::system::System::new();
    let hal = crate::isr::TargetIsrHal {
        pwm,
        input,
        comp,
        interval,
        com_timer,
        phase,
    };
    InitResult {
        hal,
        sys,
        adc,
        telem,
    }
}
