pub use stm32l4xx_hal as hal_impl;
pub use stm32l4xx_hal::pac;

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
    // SAFETY: Single-core MCU, called during init before interrupts are enabled.
    // RCC is a single-owner peripheral; no concurrent access.
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb1enr1
        // SAFETY: Setting only the TIM2EN bit (bit 0); other bits preserved via read-modify-write.
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // TIM2EN
}

/// Enable commutation timer (TIM16) peripheral clock.
pub fn enable_com_timer_clock() {
    // SAFETY: Single-core MCU, called during init before interrupts are enabled.
    // RCC is a single-owner peripheral; no concurrent access.
    let rcc = unsafe { &*pac::RCC::PTR };
    rcc.apb2enr
        // SAFETY: Setting only the TIM16EN bit (bit 17); other bits preserved via read-modify-write.
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 17)) }); // TIM16EN
}

/// Adjust IRQ priorities based on motor speed.
/// Low eRPM: DShot DMA > commutation (don't drop input frames)
/// High eRPM: commutation > DShot (don't miss commutation steps)
///
/// NOTE: STM32L4 NVIC has 4 priority bits in the UPPER nibble of each IPR
/// byte. cortex_m's `set_priority` writes the raw byte, so priority levels
/// must be passed as `level << 4`. Prior versions of this function passed
/// raw 0/1 which both collapsed to level 0 — the priority swap was a no-op.
pub fn adjust_irq_priorities(interval: u32, dshot_telem: bool) {
    use pac::Interrupt;
    const DSHOT_PRIORITY_THRESHOLD: u32 = 60;
    // SAFETY: NVIC is a core peripheral with a fixed address. We are the only
    // code adjusting these priorities; the main loop calls this outside of ISR context.
    //
    // STM32L4 implements 4 priority bits in the upper nibble of each IPR byte.
    // `cortex_m::peripheral::NVIC::set_priority` writes the raw byte, so priority
    // level N must be passed as `N << 4` (writing unshifted `1` lands in the
    // unimplemented low nibble and reads back as `0`, making all three IRQs equal
    // priority).
    // Mask interrupts across the multi-vector priority update so `TIM1_UP_TIM16`
    // and `COMP` never observe transiently different preemption priorities (which
    // would allow one to preempt the other while borrowing `COM_LOCAL`).
    cortex_m::interrupt::free(|_| {
        let nvic =
            unsafe { &mut *(cortex_m::peripheral::NVIC::PTR as *mut cortex_m::peripheral::NVIC) };
        if dshot_telem && interval > DSHOT_PRIORITY_THRESHOLD {
            // SAFETY: Setting valid priority values (0-1) for valid interrupt numbers
            // inside a critical section.
            unsafe {
                nvic.set_priority(Interrupt::DMA1_CH5, 0 << 4);
                nvic.set_priority(Interrupt::TIM1_UP_TIM16, 1 << 4);
                nvic.set_priority(Interrupt::COMP, 1 << 4);
            }
        } else {
            // SAFETY: Same as above — valid priority values for valid interrupt numbers.
            unsafe {
                nvic.set_priority(Interrupt::DMA1_CH5, 1 << 4);
                nvic.set_priority(Interrupt::TIM1_UP_TIM16, 0 << 4);
                nvic.set_priority(Interrupt::COMP, 0 << 4);
            }
        }
    });
}

/// The input tier's IRQs (DShot capture DMA transfer-complete and the
/// software-triggered EXTI frame handler): the only interrupts that
/// borrow `INPUT_LOCAL`. Masked in the NVIC while an ISR holds that cell,
/// so the priority-0 comparator and commutation timer stay live.
pub const INPUT_IRQS: [pac::Interrupt; 2] = [pac::Interrupt::DMA1_CH5, pac::Interrupt::EXTI15_10];

pub type TargetIsrHal = crate::isr::IsrHal<
    super::pwm::Pwm,
    super::input_capture::L431DshotCapture,
    super::comparator::L431BemfComparator,
    crate::timer::Tim2Interval,
    crate::timer::Tim14Com,
    crate::phase::G0APhaseDriver,
>;
pub use super::comparator::L431BemfComparator as BemfComp;
pub use super::init::init as init_mcu;

crate::define_port!(field, PortA, crate::pac::GPIOA);
crate::define_port!(field, PortB, crate::pac::GPIOB);
crate::define_raw_timer!(field, Tim2Raw, crate::pac::TIM2);
crate::define_raw_timer!(field, ComTimerRaw, crate::pac::TIM16);
