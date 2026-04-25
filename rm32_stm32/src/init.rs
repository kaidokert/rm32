//! MCU initialization — returns configured peripherals.
//!
//! Single `init()` function per MCU. The binary `main()` is MCU-independent.

use rm32::hal::{PwmOutput, System};

/// Start IWDG — shared across all MCUs (same register layout at 0x4000_3000).
#[allow(dead_code)] // Used by F051/L431 but not G071 (which uses HAL)
fn iwdg_start(prescaler: u8, reload: u16) {
    const IWDG: u32 = 0x4000_3000;
    unsafe {
        (IWDG as *mut u32).write_volatile(0x5555);
        ((IWDG + 4) as *mut u32).write_volatile(prescaler as u32);
        ((IWDG + 8) as *mut u32).write_volatile(reload as u32);
        while ((IWDG + 0x0C) as *const u32).read_volatile() & 0x03 != 0 {}
        (IWDG as *mut u32).write_volatile(0xCCCC);
        (IWDG as *mut u32).write_volatile(0xAAAA);
    }
}

#[cfg(feature = "stm32g071")]
use crate::comparator::g071::G071BemfComparator as BemfComp;
#[cfg(feature = "stm32f051")]
use crate::comparator::f051::F051BemfComparator as BemfComp;
#[cfg(feature = "stm32l431")]
use crate::comparator::l431::L431BemfComparator as BemfComp;
use crate::timer::{Tim2Interval, Tim14Com};
use crate::phase::G0APhaseDriver;

/// Result of MCU initialization.
pub struct InitResult<PWM: PwmOutput, SYS: System> {
    pub pwm: PWM,
    pub comp: BemfComp,
    pub interval: Tim2Interval,
    pub com_timer: Tim14Com,
    pub phase: G0APhaseDriver,
    pub sys: SYS,
}

// ============================================================
// STM32G071
// ============================================================
#[cfg(feature = "stm32g071")]
pub fn init() -> InitResult<crate::pwm::Tim1Pwm, crate::system::SystemControl> {
    use stm32g0xx_hal::prelude::*;
    use stm32g0xx_hal::stm32;
    use stm32g0xx_hal::rcc::Config as RccConfig;
    use stm32g0xx_hal::time::Hertz;

    let dp = stm32::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(RccConfig::pll());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let _gpiob = dp.GPIOB.split(&mut rcc);

    let pwm = crate::pwm::Tim1Pwm::new(
        dp.TIM1, gpioa.pa8, gpioa.pa9, gpioa.pa10,
        Hertz::from_raw(24_000), &mut rcc,
        rm32::board::BoardConfig::DEFAULT.dead_time,
    );
    let phase = G0APhaseDriver::new(false);
    let sys = crate::system::SystemControl::new(dp.IWDG);
    crate::comp_init::init_comp2();
    let comp = crate::comparator::g071::new_comparator();
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new();

    // DShot input capture
    {
        crate::input_capture::init_g071();
        let mut input = crate::input_capture::new_capture();
        // Hardware init done by init_*() above
        use rm32::hal::InputCapture;
        input.receive_dshot_dma();
    }
    let adc = crate::adc::new_adc();
    let _ = adc.init();
    let _ = crate::telemetry_uart::TelemUart::init();

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

    InitResult { pwm, comp, interval, com_timer, phase, sys }
}

// ============================================================
// STM32F051
// ============================================================
#[cfg(feature = "stm32f051")]
const TIM1_BASE: u32 = 0x4001_2C00;

#[cfg(feature = "stm32f051")]
pub struct F051Pwm { _private: () }

#[cfg(feature = "stm32f051")]
impl PwmOutput for F051Pwm {
    fn set_duty_all(&mut self, d: u16) {
        unsafe {
            ((TIM1_BASE + 0x34) as *mut u32).write_volatile(d as u32);
            ((TIM1_BASE + 0x38) as *mut u32).write_volatile(d as u32);
            ((TIM1_BASE + 0x3C) as *mut u32).write_volatile(d as u32);
        }
    }
    fn set_auto_reload(&mut self, a: u16) { unsafe { ((TIM1_BASE+0x2C) as *mut u32).write_volatile(a as u32); } }
    fn set_prescaler(&mut self, p: u16) { unsafe { ((TIM1_BASE+0x28) as *mut u32).write_volatile(p as u32); } }
    fn set_compare1(&mut self, v: u16) { unsafe { ((TIM1_BASE+0x34) as *mut u32).write_volatile(v as u32); } }
    fn set_compare2(&mut self, v: u16) { unsafe { ((TIM1_BASE+0x38) as *mut u32).write_volatile(v as u32); } }
    fn set_compare3(&mut self, v: u16) { unsafe { ((TIM1_BASE+0x3C) as *mut u32).write_volatile(v as u32); } }
    fn generate_update_event(&mut self) { unsafe { ((TIM1_BASE+0x14) as *mut u32).write_volatile(1); } }
    fn set_dead_time_override(&mut self, dtg: u16) {
        unsafe { crate::regs::modify(TIM1_BASE + 0x44, |v| v | dtg as u32); }
    }
}

#[cfg(feature = "stm32f051")]
pub struct F051System { _private: () }

#[cfg(feature = "stm32f051")]
impl System for F051System {
    fn reset(&mut self) -> ! { cortex_m::peripheral::SCB::sys_reset() }
    fn enable_irq(&mut self) { unsafe { cortex_m::interrupt::enable() }; }
    fn disable_irq(&mut self) { cortex_m::interrupt::disable(); }
    fn start_watchdog(&mut self, prescaler: u8, reload: u16) { iwdg_start(prescaler, reload); }
    fn reload_watchdog(&mut self) { unsafe { crate::regs::write(0x4000_3000, 0xAAAA); } }
    fn delay_micros(&mut self, us: u32) { cortex_m::asm::delay(us * 48); }
    fn delay_millis(&mut self, ms: u32) { for _ in 0..ms { self.delay_micros(1000); } }
}

#[cfg(feature = "stm32f051")]
pub fn init() -> InitResult<F051Pwm, F051System> {
    use stm32f0xx_hal::prelude::*;
    use stm32f0xx_hal::pac;

    let mut dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // Clock: 48MHz PLL from HSI
    let _rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut dp.FLASH);

    let rcc_base = 0x4002_1000u32;
    unsafe {
        // Enable GPIO A/B clocks
        let ahbenr = (rcc_base + 0x14) as *mut u32;
        ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 17) | (1 << 18));
        // Enable TIM1 (APB2ENR bit 11)
        let apb2enr = (rcc_base + 0x18) as *mut u32;
        apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 11));

        // PA8/9/10 as AF2 (TIM1_CH1/2/3)
        let gpioa_moder = crate::periph_addr::GPIOA as *mut u32;
        let gpioa_afrh = (crate::periph_addr::GPIOA + 0x24) as *mut u32;
        let m = gpioa_moder.read_volatile();
        gpioa_moder.write_volatile(
            (m & !(0b11<<16 | 0b11<<18 | 0b11<<20)) | (0b10<<16 | 0b10<<18 | 0b10<<20)
        );
        let a = gpioa_afrh.read_volatile();
        gpioa_afrh.write_volatile((a & !(0xFFF)) | (2 | 2<<4 | 2<<8));
    }

    // TIM1 PWM: 48MHz/2000 = 24kHz
    let dead_time = rm32::board::SISKIN_F051.dead_time;
    unsafe {
        ((TIM1_BASE+0x28) as *mut u32).write_volatile(0);    // PSC
        ((TIM1_BASE+0x2C) as *mut u32).write_volatile(1999); // ARR
        ((TIM1_BASE+0x18) as *mut u32).write_volatile(0x6868); // CCMR1: PWM1
        ((TIM1_BASE+0x1C) as *mut u32).write_volatile(0x0068); // CCMR2: PWM1
        ((TIM1_BASE+0x20) as *mut u32).write_volatile(0x555);  // CCER: all channels + complementary
        ((TIM1_BASE+0x44) as *mut u32).write_volatile(dead_time as u32 | (1<<15)); // BDTR: DTG+MOE
        (TIM1_BASE as *mut u32).write_volatile(1);    // CR1: CEN
    }
    let pwm = F051Pwm { _private: () };
    let phase = G0APhaseDriver::new(false); // same pins for F0_A

    // COMP1 init
    crate::comp_init_f051::init_comp1();
    let comp = crate::comparator::f051::new_comparator();

    // Timers
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new();

    // Input capture (TIM15 + DMA1_CH5)
    {
        crate::input_capture_f051::init_f051();
        let mut input = crate::input_capture_f051::new_capture();
        // Hardware init done by init_*() above
        use rm32::hal::InputCapture;
        input.receive_dshot_dma();
    }

    // ADC
    let adc = crate::adc_f051::new_adc();
    let _ = adc.init();

    // UART telemetry
    let _ = crate::telemetry_uart_f051::F051TelemUart::init();

    // TIM6: 48MHz/2400 = 20kHz
    unsafe {
        let apb1enr = (rcc_base + 0x1C) as *mut u32;
        apb1enr.write_volatile(apb1enr.read_volatile() | (1 << 4)); // TIM6EN
        let tim6 = 0x4000_1000u32;
        ((tim6+0x28) as *mut u32).write_volatile(0);    // PSC
        ((tim6+0x2C) as *mut u32).write_volatile(2399); // ARR
        ((tim6+0x14) as *mut u32).write_volatile(1);    // EGR.UG
        ((tim6+0x10) as *mut u32).write_volatile(0);    // SR clear
        ((tim6+0x0C) as *mut u32).write_volatile(1);    // DIER.UIE
        (tim6 as *mut u32).write_volatile(1);    // CR1.CEN
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
        let imr = 0x4001_0400 as *mut u32;
        imr.write_volatile(imr.read_volatile() | (1 << 15));
    }

    let sys = F051System { _private: () };

    InitResult { pwm, comp, interval, com_timer, phase, sys }
}

// ============================================================
// STM32L431
// ============================================================
#[cfg(feature = "stm32l431")]
const TIM1_BASE_L4: u32 = 0x4001_2C00;

#[cfg(feature = "stm32l431")]
pub struct L431Pwm { _private: () }

#[cfg(feature = "stm32l431")]
impl PwmOutput for L431Pwm {
    fn set_duty_all(&mut self, d: u16) {
        unsafe {
            ((TIM1_BASE_L4 + 0x34) as *mut u32).write_volatile(d as u32);
            ((TIM1_BASE_L4 + 0x38) as *mut u32).write_volatile(d as u32);
            ((TIM1_BASE_L4 + 0x3C) as *mut u32).write_volatile(d as u32);
        }
    }
    fn set_auto_reload(&mut self, a: u16) { unsafe { ((TIM1_BASE_L4+0x2C) as *mut u32).write_volatile(a as u32); } }
    fn set_prescaler(&mut self, p: u16) { unsafe { ((TIM1_BASE_L4+0x28) as *mut u32).write_volatile(p as u32); } }
    fn set_compare1(&mut self, v: u16) { unsafe { ((TIM1_BASE_L4+0x34) as *mut u32).write_volatile(v as u32); } }
    fn set_compare2(&mut self, v: u16) { unsafe { ((TIM1_BASE_L4+0x38) as *mut u32).write_volatile(v as u32); } }
    fn set_compare3(&mut self, v: u16) { unsafe { ((TIM1_BASE_L4+0x3C) as *mut u32).write_volatile(v as u32); } }
    fn generate_update_event(&mut self) { unsafe { ((TIM1_BASE_L4+0x14) as *mut u32).write_volatile(1); } }
    fn set_dead_time_override(&mut self, dtg: u16) {
        unsafe { crate::regs::modify(TIM1_BASE_L4 + 0x44, |v| v | dtg as u32); }
    }
}

#[cfg(feature = "stm32l431")]
pub struct L431System { _private: () }

#[cfg(feature = "stm32l431")]
impl System for L431System {
    fn reset(&mut self) -> ! { cortex_m::peripheral::SCB::sys_reset() }
    fn enable_irq(&mut self) { unsafe { cortex_m::interrupt::enable() }; }
    fn disable_irq(&mut self) { cortex_m::interrupt::disable(); }
    fn start_watchdog(&mut self, prescaler: u8, reload: u16) { iwdg_start(prescaler, reload); }
    fn reload_watchdog(&mut self) { unsafe { crate::regs::write(0x4000_3000, 0xAAAA); } }
    fn delay_micros(&mut self, us: u32) { cortex_m::asm::delay(us * 80); }
    fn delay_millis(&mut self, ms: u32) { for _ in 0..ms { self.delay_micros(1000); } }
}

#[cfg(feature = "stm32l431")]
pub fn init() -> InitResult<L431Pwm, L431System> {
    use stm32l4xx_hal::prelude::*;
    use stm32l4xx_hal::pac;

    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // Clock: 80MHz from MSI via PLL
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80_000_000u32.Hz()).freeze(&mut flash.acr, &mut pwr);
    let _ = clocks;

    let rcc_base = 0x4002_1000u32;
    unsafe {
        // Enable GPIOA, GPIOB (AHB2ENR bits 0, 1)
        let ahb2enr = (rcc_base + 0x4C) as *mut u32;
        ahb2enr.write_volatile(ahb2enr.read_volatile() | (1 << 0) | (1 << 1));
        // Enable TIM1 (APB2ENR bit 11)
        let apb2enr = (rcc_base + 0x60) as *mut u32;
        apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 11));

        // PA8/9/10 as AF1 (TIM1_CH1/2/3 on L4 = AF1, not AF2)
        let gpioa_moder = crate::periph_addr::GPIOA as *mut u32;
        let gpioa_afrh = (crate::periph_addr::GPIOA + 0x24) as *mut u32;
        let m = gpioa_moder.read_volatile();
        gpioa_moder.write_volatile(
            (m & !(0b11<<16 | 0b11<<18 | 0b11<<20)) | (0b10<<16 | 0b10<<18 | 0b10<<20)
        );
        let a = gpioa_afrh.read_volatile();
        gpioa_afrh.write_volatile((a & !(0xFFF)) | (1 | 1<<4 | 1<<8)); // AF1
    }

    // TIM1 PWM: 80MHz / (ARR+1) = 24kHz → ARR = 3332
    let dead_time = rm32::board::NEUTRON_L431.dead_time;
    unsafe {
        ((TIM1_BASE_L4+0x28) as *mut u32).write_volatile(0);
        ((TIM1_BASE_L4+0x2C) as *mut u32).write_volatile(crate::config::TIM1_AUTORELOAD as u32);
        ((TIM1_BASE_L4+0x18) as *mut u32).write_volatile(0x6868);
        ((TIM1_BASE_L4+0x1C) as *mut u32).write_volatile(0x0068);
        ((TIM1_BASE_L4+0x20) as *mut u32).write_volatile(0x555);
        ((TIM1_BASE_L4+0x44) as *mut u32).write_volatile(dead_time as u32 | (1<<15));
        ((TIM1_BASE_L4+0x00) as *mut u32).write_volatile(1);
    }
    let pwm = L431Pwm { _private: () };
    let phase = G0APhaseDriver::new(false); // same pins for L4_N

    // COMP2 init
    crate::comp_init_l431::init_comp2();
    let comp = crate::comparator::l431::new_comparator();
    let interval = Tim2Interval::new();
    let com_timer = Tim14Com::new(); // L431 uses TIM16, but TIM14 struct works (same register layout)

    // Input capture (TIM15 + DMA1_CH5)
    {
        crate::input_capture_l431::init_l431();
        let mut input = crate::input_capture_l431::new_capture();
        // Hardware init done by init_*() above
        use rm32::hal::InputCapture;
        input.receive_dshot_dma();
    }

    // ADC
    let adc = crate::adc_l431::new_adc();
    let _ = adc.init();

    // UART telemetry
    let _ = crate::telemetry_uart_l431::L431TelemUart::init();

    // TIM6: 80MHz / 4000 = 20kHz
    unsafe {
        // Enable TIM6 (APB1ENR1 bit 4)
        let apb1enr1 = (rcc_base + 0x58) as *mut u32;
        apb1enr1.write_volatile(apb1enr1.read_volatile() | (1 << 4));
        let tim6 = 0x4000_1000u32;
        ((tim6+0x28) as *mut u32).write_volatile(0);
        ((tim6+0x2C) as *mut u32).write_volatile(3999); // 80MHz/4000=20kHz
        ((tim6+0x14) as *mut u32).write_volatile(1);
        ((tim6+0x10) as *mut u32).write_volatile(0);
        ((tim6+0x0C) as *mut u32).write_volatile(1);
        ((tim6+0x00) as *mut u32).write_volatile(1);
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
        let imr = 0x4001_0400 as *mut u32; // EXTI IMR1
        imr.write_volatile(imr.read_volatile() | (1 << 15));
    }

    let sys = L431System { _private: () };
    InitResult { pwm, comp, interval, com_timer, phase, sys }
}
