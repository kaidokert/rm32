//! STM32L431 initialization.

use crate::init::InitResult;
use crate::mcu::ChipConfig;
use crate::phase::G0APhaseDriver;
use crate::timer::{Tim2Interval, Tim14Com};

pub fn init(
    dead_time: u8,
    bemf_pins: rm32::board::BemfPins,
) -> InitResult<super::system::System, super::adc::L431Adc, super::telemetry_uart::L431TelemUart> {
    use stm32l4xx_hal::pac;

    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // Clock tree: AM32-matched 80 MHz SYSCLK = HSI16 / 2 * 20 / 2 (M=2, N=20, R=2).
    // Ensure PWR regulator is in Range 1 (VOS = 0b01, required for up to 80 MHz per
    // RM0394 §5.1.8) and configure 4 Flash wait states, prefetch, and instruction/data
    // caches before raising SYSCLK above 16 MHz.
    dp.RCC.apb1enr1.modify(|_, w| w.pwren().set_bit());
    dp.PWR.cr1.modify(|_, w| unsafe { w.vos().bits(0b01) });
    let _ = crate::regs::wait_for(
        || dp.PWR.sr2.read().vosf().bit_is_clear(),
        1_000_000,
        "PWR VOS1",
    );

    dp.FLASH.acr.write(|w| unsafe {
        w.latency()
            .bits(4)
            .prften()
            .set_bit()
            .icen()
            .set_bit()
            .dcen()
            .set_bit()
    });
    let _ = crate::regs::wait_for(
        || dp.FLASH.acr.read().latency().bits() == 4,
        1_000_000,
        "FLASH latency",
    );

    dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    let _ = crate::regs::wait_for(
        || dp.RCC.cr.read().hsirdy().bit_is_set(),
        1_000_000,
        "HSI16 ready",
    );

    // RM0394 §6.4.4: STM32L4 RCC_PLLCFGR has no PLLRGE field (bits [3:2] are reserved);
    // VCO input frequency is HSI16 / PLLM = 16 MHz / 2 = 8 MHz (within L4's 4–16 MHz spec).
    dp.RCC.pllcfgr.write(|w| unsafe {
        w.pllsrc()
            .bits(2) // HSI16
            .pllm()
            .bits(1) // /2
            .plln()
            .bits(20)
            .pllr()
            .bits(0) // /2
            .pllren()
            .set_bit()
    });

    dp.RCC.cr.modify(|_, w| w.pllon().set_bit());
    if crate::regs::wait_for(
        || dp.RCC.cr.read().pllrdy().bit_is_set(),
        1_000_000,
        "PLL ready",
    )
    .is_ok()
    {
        dp.RCC.cfgr.modify(|_, w| unsafe { w.sw().bits(0b11) });
        let _ = crate::regs::wait_for(
            || dp.RCC.cfgr.read().sws().bits() == 0b11,
            1_000_000,
            "SYSCLK PLL switch",
        );
    }

    // NVIC priority grouping = 3 (4 bits preempt / 0 subpriority on Cortex-M4),
    // matching AM32's HAL_NVIC_SetPriorityGrouping configuration.
    unsafe {
        core::ptr::write_volatile(0xE000_ED0C as *mut u32, 0x05FA_0300);
    }

    let rcc_pac = unsafe { &*pac::RCC::ptr() };
    unsafe {
        // Enable GPIOA, GPIOB (AHB2ENR bits 0, 1)
        rcc_pac
            .ahb2enr
            .modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit());
        // Enable TIM1 (APB2ENR bit 11)
        rcc_pac.apb2enr.modify(|_, w| w.tim1en().set_bit());

        // PA8/9/10 as AF1 (TIM1_CH1/2/3 on L4 = AF1, not AF2);
        // PA7 = TIM1_CH1N (AF1), parked output-low until the first commutation step.
        let gpioa = &*pac::GPIOA::ptr();
        gpioa.bsrr.write(|w| w.bits(1 << (7 + 16)));
        gpioa.afrl.modify(|_, w| w.afrl7().bits(1));
        gpioa.afrh.modify(|r, w| {
            w.bits((r.bits() & !(0xFFF)) | (1 | 1 << 4 | 1 << 8)) // AF1
        });
        gpioa.moder.modify(|r, w| {
            w.bits(
                (r.bits() & !(0b11 << 14 | 0b11 << 16 | 0b11 << 18 | 0b11 << 20))
                    | (0b01 << 14 | 0b10 << 16 | 0b10 << 18 | 0b10 << 20),
            )
        });
        // PB0 = TIM1_CH2N (AF1), PB1 = TIM1_CH3N (AF1), parked output-low until armed;
        // PB6 = USART1_TX (AF7, pull-up). Clear PB4's reset NJTRST pull-up so COMP2 INP is unbiased.
        let gpiob = &*pac::GPIOB::ptr();
        gpiob.bsrr.write(|w| w.bits((1 << 16) | (1 << 17)));
        gpiob
            .afrl
            .modify(|_, w| w.afrl0().bits(1).afrl1().bits(1).afrl6().bits(7));
        gpiob.moder.modify(|_, w| {
            w.moder0()
                .bits(0b01)
                .moder1()
                .bits(0b01)
                .moder6()
                .bits(0b10)
        });
        gpiob
            .pupdr
            .modify(|_, w| w.pupdr4().bits(0b00).pupdr6().bits(0b01));
    }

    // TIM1 PWM: 80MHz / (ARR+1) = 24kHz -> ARR = 3332, AM32-matched fields:
    // - CCMR1/CCMR2 = 0x6868 (OC1..OC4 PWM mode 1 with OCxPE preload)
    // - CCER = 0x1555 (CC1..CC3 complementary + CC4E for OC4REF trigger)
    // - CCR4 = 0x64 (100)
    // - BDTR = DTG | BKP | MOE
    // - CR1  = ARPE | CEN
    unsafe {
        let tim1 = &*pac::TIM1::ptr();
        tim1.psc.write(|w| w.psc().bits(0));
        tim1.arr
            .write(|w| w.arr().bits(super::chip::Chip::TIM1_AUTORELOAD));
        tim1.ccmr1_output().write(|w| {
            w.oc1m()
                .bits(0b110)
                .oc1pe()
                .set_bit()
                .oc2m()
                .bits(0b110)
                .oc2pe()
                .set_bit()
        });
        tim1.ccmr2_output().write(|w| {
            w.oc3m()
                .bits(0b110)
                .oc3pe()
                .set_bit()
                .oc4m()
                .bits(0b110)
                .oc4pe()
                .set_bit()
        });
        tim1.ccer.write(|w| {
            w.cc1e()
                .set_bit()
                .cc1ne()
                .set_bit()
                .cc2e()
                .set_bit()
                .cc2ne()
                .set_bit()
                .cc3e()
                .set_bit()
                .cc3ne()
                .set_bit()
                .cc4e()
                .set_bit()
        });
        tim1.ccr4.write(|w| w.ccr().bits(0x64));
        tim1.bdtr
            .write(|w| w.moe().set_bit().bkp().set_bit().dtg().bits(dead_time));
        tim1.cr1.write(|w| w.arpe().set_bit().cen().set_bit());
    }
    let pwm = super::pwm::Pwm::new();
    let phase = G0APhaseDriver::new(false); // same pins for L4_N

    // COMP2 init
    super::comp_init::init_comp2(bemf_pins.phase_a);
    let comp = super::comparator::new_comparator(bemf_pins);
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

    const CONTROL_RELOAD: u16 = (80_000_000
        / <crate::platform::StmPlatform as rm32::platform::Platform>::TICK_HZ
        - 1) as u16;
    // TIM6: 80MHz / 4000 = 20kHz
    unsafe {
        // Enable TIM6 (APB1ENR1 bit 4)
        rcc_pac.apb1enr1.modify(|_, w| w.tim6en().set_bit());
        let tim6 = &*pac::TIM6::ptr();
        tim6.psc.write(|w| w.bits(0));
        tim6.arr.write(|w| w.bits(CONTROL_RELOAD as u32));
        tim6.egr.write(|w| w.ug().set_bit());
        tim6.sr.write(|w| w.uif().clear_bit());
        tim6.dier.write(|w| w.uie().set_bit());
        tim6.cr1.write(|w| w.cen().set_bit());
    }

    // NVIC priorities — AM32-matched. STM32L4 implements 4 priority bits in the
    // upper nibble of each IPR byte (PRIGROUP = 3: 4 preempt bits, 0 subpriority).
    // `cortex_m::peripheral::NVIC::set_priority` writes the raw byte, so priority
    // level N must be passed as `N << 4`:
    //   COMP             0  — BEMF zero-cross, must preempt tenKhzRoutine
    //   TIM1_UP_TIM16    0  — commutation timer, same urgency as COMP
    //   DMA1_CH5         1  — DShot/PWM input capture
    //   EXTI15_10        2  — software-triggered frame processing
    //   TIM6_DACUNDER    3  — 20 kHz control loop (lowest priority)
    unsafe {
        use pac::{Interrupt, NVIC};
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(Interrupt::COMP, 0 << 4);
        nvic.set_priority(Interrupt::TIM1_UP_TIM16, 0 << 4);
        nvic.set_priority(Interrupt::DMA1_CH5, 1 << 4);
        nvic.set_priority(Interrupt::EXTI15_10, 2 << 4);
        nvic.set_priority(Interrupt::TIM6_DACUNDER, 3 << 4);
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
