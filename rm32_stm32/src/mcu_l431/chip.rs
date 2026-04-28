pub use stm32l4xx_hal::pac;
pub use stm32l4xx_hal as hal_impl;

use crate::mcu::ChipConfig;

pub struct Chip;

impl ChipConfig for Chip {
    const CPU_FREQUENCY_MHZ: u32 = 80;
    const EEPROM_START: u32 = 0x0800_F800;
    const FLASH_PAGE_SIZE: u32 = 0x800;
    const TIMER_PSC: u16 = 39;
    const GCR_SHIFT: u8 = 7;
    const COMP_EXTI_LINE: u32 = 22;
    const INPUT_DMA_CHANNEL: usize = 4;
    const ADC_CURRENT_CHANNEL: u8 = 8;
    const ADC_VOLTAGE_CHANNEL: u8 = 11;
    const WDG_PRESCALER: u8 = 2;
    const WDG_RELOAD: u16 = 4000;
}
pub use super::flash::FlashStorage;

/// Enable TIM2 peripheral clock.
pub fn enable_tim2_clock() {
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb1enr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
}

/// Enable commutation timer (TIM16) peripheral clock.
pub fn enable_com_timer_clock() {
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb2enr.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 17)) }); // TIM16EN
}

/// Adjust IRQ priorities based on motor speed.
/// Low eRPM: DShot DMA > commutation (don't drop input frames)
/// High eRPM: commutation > DShot (don't miss commutation steps)
pub fn adjust_irq_priorities(interval: u32, dshot_telem: bool) {
    use pac::Interrupt;
    const DSHOT_PRIORITY_THRESHOLD: u32 = 60;
    let nvic = unsafe { &mut *(cortex_m::peripheral::NVIC::PTR as *mut cortex_m::peripheral::NVIC) };
    if dshot_telem && interval > DSHOT_PRIORITY_THRESHOLD {
        unsafe {
            nvic.set_priority(Interrupt::DMA1_CH5, 0);
            nvic.set_priority(Interrupt::TIM1_UP_TIM16, 1);
            nvic.set_priority(Interrupt::COMP, 1);
        }
    } else {
        unsafe {
            nvic.set_priority(Interrupt::DMA1_CH5, 1);
            nvic.set_priority(Interrupt::TIM1_UP_TIM16, 0);
            nvic.set_priority(Interrupt::COMP, 0);
        }
    }
}

pub type TargetIsrHal = crate::isr::IsrHal<
    super::pwm::Pwm,
    super::input_capture::L431DshotCapture,
    super::comparator::L431BemfComparator,
    crate::timer::Tim2Interval, crate::timer::Tim14Com, crate::phase::G0APhaseDriver,
>;
pub use super::comparator::L431BemfComparator as BemfComp;
pub use super::init::init as init_mcu;

crate::define_port!(field, PortA, crate::pac::GPIOA);
crate::define_port!(field, PortB, crate::pac::GPIOB);
crate::define_timer_ops!(field, tim2_ops, crate::pac::TIM2);
crate::define_timer_ops!(field, com_tim_ops, crate::pac::TIM16);
