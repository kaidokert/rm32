//! F051 interrupt vectors — thin wrappers calling shared handlers.

use stm32f0xx_hal::pac::interrupt;
use crate::pac;
use crate::isr_handlers;

#[interrupt]
fn TIM6_DAC() {
    // Clear UIF
    let tim6 = unsafe { &*pac::TIM6::PTR };
    unsafe { tim6.sr.write(|w| w.bits(0)); }
    isr_handlers::handle_tim6();
}

#[interrupt]
fn TIM14() {
    isr_handlers::handle_tim14();
}

#[interrupt]
fn ADC_COMP() {
    isr_handlers::handle_comp();
}

// F051 uses DMA1_CH4_5 for TIM15 input capture
#[interrupt]
fn DMA1_CH4_5_6_7_DMA2_CH3_4_5() {
    let dma = unsafe { &*pac::DMA1::PTR };
    let dma_isr = dma.isr.read().bits();
    // Channel 5 TC flag = bit 17
    if dma_isr & (1 << 17) != 0 {
        unsafe { dma.ifcr.write(|w| w.bits(1 << 16)); } // CGIF5
        // Disable DMA CH5 (CCR5)
        unsafe {
            dma.ch5.cr.modify(|r, w| w.bits(r.bits() & !1));
        }
        isr_handlers::handle_dma_tc();
        // Trigger software EXTI15
        let exti = unsafe { &*pac::EXTI::PTR };
        unsafe { exti.swier.write(|w| w.bits(1 << 15)); }
    }
}

#[interrupt]
fn EXTI4_15() {
    // Clear EXTI15 pending
    let exti = unsafe { &*pac::EXTI::PTR };
    unsafe { exti.pr.write(|w| w.bits(1 << 15)); }
    isr_handlers::handle_exti_frame();

    // Re-enable DMA CH5 for next frame
    let shared = crate::isr::shared();
    let sz = if shared.servo_pwm() { 2u32 } else { 32 };
    let dma = unsafe { &*pac::DMA1::PTR };
    unsafe {
        dma.ch5.ndtr.write(|w| w.bits(sz));
        // Enable CH5
        dma.ch5.cr.modify(|r, w| w.bits(r.bits() | 1));
    }
    // TIM15 CR1.CEN
    let tim15 = unsafe { &*pac::TIM15::PTR };
    unsafe {
        tim15.cr1.modify(|r, w| w.bits(r.bits() | 1));
    }
}
