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
    // SAFETY: Single-core MCU, called during init before interrupts are enabled.
    // RCC is a single-owner peripheral; no concurrent access.
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb1enr
        // SAFETY: Setting only the TIM2EN bit (bit 0); other bits preserved via read-modify-write.
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
}

/// Enable commutation timer (TIM14) peripheral clock.
pub fn enable_com_timer_clock() {
    // SAFETY: Single-core MCU, called during init before interrupts are enabled.
    // RCC is a single-owner peripheral; no concurrent access.
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb1enr
        // SAFETY: Setting only the TIM14EN bit (bit 8); other bits preserved via read-modify-write.
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 8)) }); // TIM14EN
}

/// Adjust IRQ priorities based on motor speed. No-op on F051.
pub fn adjust_irq_priorities(_interval: u32, _dshot_telem: bool) {}

/// The input tier's IRQs (DShot capture DMA transfer-complete and the
/// software-triggered EXTI frame handler): the only interrupts that
/// borrow `INPUT_LOCAL`. Masked in the NVIC while an ISR holds that cell,
/// so the priority-0 comparator and commutation timer stay live.
pub const INPUT_IRQS: [pac::Interrupt; 2] = [
    pac::Interrupt::DMA1_CH4_5_6_7_DMA2_CH3_4_5,
    pac::Interrupt::EXTI4_15,
];

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
