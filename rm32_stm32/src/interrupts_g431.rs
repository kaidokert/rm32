//! G431 interrupt vectors — thin wrappers calling shared handlers.
//! G431 uses TIM16 for commutation (shared IRQ with TIM1_UP).

use stm32g4::stm32g431::interrupt;
use crate::pac;
use crate::isr_handlers;

#[interrupt]
fn TIM6_DACUNDER() {
    let tim6 = unsafe { &*pac::TIM6::PTR };
    unsafe { tim6.sr().write(|w| w.bits(0)); }
    isr_handlers::handle_tim6();
}

#[interrupt]
fn TIM1_UP_TIM16() {
    let tim16 = unsafe { &*pac::TIM16::PTR };
    unsafe { tim16.sr().write(|w| w.bits(0)); }
    isr_handlers::handle_tim14();
}

#[interrupt]
fn COMP1_2_3() {
    isr_handlers::handle_comp();
}

// DMA1 Channel 1: input capture transfer complete
#[interrupt]
fn DMA1_CH1() {
    let dma = unsafe { &*pac::DMA1::PTR };
    let dma_isr = dma.isr().read().bits();
    // Channel 1 TC flag = bit 1
    if dma_isr & (1 << 1) != 0 {
        unsafe {
            dma.ifcr().write(|w| w.bits(1 << 0)); // CGIF1
            let ch1 = dma.ch1();
            ch1.cr().modify(|r, w| w.bits(r.bits() & !1)); // Disable CH1
        }
        isr_handlers::handle_dma_tc();
        // Trigger software EXTI15
        let exti = unsafe { &*pac::EXTI::PTR };
        unsafe { exti.swier1().write(|w| w.bits(1 << 15)); }
    }
}

#[interrupt]
fn EXTI15_10() {
    let exti = unsafe { &*pac::EXTI::PTR };
    unsafe { exti.pr1().write(|w| w.bits(1 << 15)); }
    isr_handlers::handle_exti_frame();

    // Re-enable DMA CH1 for next frame
    let shared = crate::isr::shared();
    let sz = if shared.servo_pwm() { 2u32 } else { 32 };
    let dma = unsafe { &*pac::DMA1::PTR };
    let ch1 = dma.ch1();
    unsafe {
        ch1.ndtr().write(|w| w.bits(sz));
        ch1.cr().modify(|r, w| w.bits(r.bits() | 1)); // Enable CH1
    }
    // TIM15 CR1.CEN
    let tim15 = unsafe { &*pac::TIM15::PTR };
    unsafe { tim15.cr1().modify(|r, w| w.bits(r.bits() | 1)); }
}
