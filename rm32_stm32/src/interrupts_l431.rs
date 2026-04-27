//! L431 interrupt vectors — thin wrappers calling shared handlers.
//! L431 uses TIM16 for commutation (shared IRQ with TIM1_UP).

use stm32l4xx_hal::pac::interrupt;
use crate::pac;
use crate::isr_handlers;

#[interrupt]
fn TIM6_DACUNDER() {
    let tim6 = unsafe { &*pac::TIM6::PTR };
    unsafe { tim6.sr.write(|w| w.bits(0)); }
    isr_handlers::handle_tim6();
}

#[interrupt]
fn TIM1_UP_TIM16() {
    // TIM16 is the commutation timer on L431
    let tim16 = unsafe { &*pac::TIM16::PTR };
    unsafe { tim16.sr.write(|w| w.bits(0)); }
    isr_handlers::handle_tim14(); // same logic, different timer
}

#[interrupt]
fn COMP() {
    isr_handlers::handle_comp();
}

// DMA1 Channel 5: input capture transfer complete
#[interrupt]
fn DMA1_CH5() {
    let dma = unsafe { &*pac::DMA1::PTR };
    let dma_isr = dma.isr.read().bits();
    // Channel 5 TC flag = bit 17
    if dma_isr & (1 << 17) != 0 {
        unsafe { dma.ifcr.write(|w| w.bits(1 << 16)); } // CGIF5
        // Disable DMA CH5
        unsafe {
            dma.ccr5.modify(|r, w| w.bits(r.bits() & !1));
        }
        isr_handlers::handle_dma_tc();
        // Trigger software EXTI15
        let exti = unsafe { &*pac::EXTI::PTR };
        unsafe { exti.swier1.write(|w| w.bits(1 << 15)); }
    }
}

#[interrupt]
fn EXTI15_10() {
    let exti = unsafe { &*pac::EXTI::PTR };
    unsafe { exti.pr1.write(|w| w.bits(1 << 15)); }
    isr_handlers::handle_exti_frame();

    // Re-enable DMA CH5 for next frame
    let shared = crate::isr::shared();
    let sz = if shared.servo_pwm() { 2u32 } else { 32 };
    let dma = unsafe { &*pac::DMA1::PTR };
    unsafe {
        dma.cndtr5.write(|w| w.bits(sz));
        dma.ccr5.modify(|r, w| w.bits(r.bits() | 1)); // Enable CH5
    }
    // TIM15 CR1.CEN
    let tim15 = unsafe { &*pac::TIM15::PTR };
    unsafe {
        tim15.cr1.modify(|r, w| w.bits(r.bits() | 1));
    }
}
