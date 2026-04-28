pub use stm32f0xx_hal as hal_impl;
pub use stm32f0xx_hal::pac;

use crate::mcu::ChipConfig;

pub struct Chip;

impl ChipConfig for Chip {
    const CPU_FREQUENCY_MHZ: u32 = 48;
    const EEPROM_START: u32 = 0x0800_7C00;
    const FLASH_PAGE_SIZE: u32 = 0x400;
    const TIMER_PSC: u16 = 23;
    const GCR_SHIFT: u8 = 6;
    const COMP_EXTI_LINE: u32 = 21;
    const INPUT_DMA_CHANNEL: usize = 4;
    const ADC_CURRENT_CHANNEL: u8 = 6;
    const ADC_VOLTAGE_CHANNEL: u8 = 3;
    const WDG_PRESCALER: u8 = 2;
    const WDG_RELOAD: u16 = 4000;
}
pub use super::flash::FlashStorage;

/// Enable TIM2 peripheral clock.
pub fn enable_tim2_clock() {
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb1enr
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
}

/// Enable commutation timer (TIM14) peripheral clock.
pub fn enable_com_timer_clock() {
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb1enr
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 8)) }); // TIM14EN
}

/// Adjust IRQ priorities based on motor speed. No-op on F051.
pub fn adjust_irq_priorities(_interval: u32, _dshot_telem: bool) {}

pub type TargetIsrHal = crate::isr::IsrHal<
    super::pwm::Pwm,
    super::input_capture::F051DshotCapture,
    super::comparator::F051BemfComparator,
    crate::timer::Tim2Interval,
    crate::timer::Tim14Com,
    crate::phase::G0APhaseDriver,
>;
pub use super::comparator::F051BemfComparator as BemfComp;
pub use super::init::init as init_mcu;

crate::define_port!(field, PortA, crate::pac::GPIOA);
crate::define_port!(field, PortB, crate::pac::GPIOB);
crate::define_raw_timer!(field, Tim2Raw, crate::pac::TIM2);
crate::define_raw_timer!(field, ComTimerRaw, crate::pac::TIM14);
