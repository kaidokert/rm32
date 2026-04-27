//! G431 interrupt vectors — thin wrappers calling shared handlers.
//! G431 uses TIM16 for commutation (shared IRQ with TIM1_UP).

use stm32g4::stm32g431::interrupt;
use crate::isr_handlers;

const TIM6_SR: u32 = 0x4000_1010;
const TIM16_SR: u32 = 0x4001_4410;
const DMA1_ISR: u32 = 0x4002_0000;
const DMA1_IFCR: u32 = 0x4002_0004;
const DMA1_CCR1: u32 = 0x4002_0008;
const EXTI_PR1: u32 = 0x4001_0414;
const EXTI_SWIER1: u32 = 0x4001_0410;
const DMA1_CNDTR1: u32 = 0x4002_000C;

#[interrupt]
fn TIM6_DACUNDER() {
    unsafe { (TIM6_SR as *mut u32).write_volatile(0); }
    isr_handlers::handle_tim6();
}

#[interrupt]
fn TIM1_UP_TIM16() {
    unsafe { (TIM16_SR as *mut u32).write_volatile(0); }
    isr_handlers::handle_tim14();
}

#[interrupt]
fn COMP1_2_3() {
    isr_handlers::handle_comp();
}

// DMA1 Channel 1: input capture transfer complete
#[interrupt]
fn DMA1_CH1() {
    let dma_isr = unsafe { (DMA1_ISR as *const u32).read_volatile() };
    // Channel 1 TC flag = bit 1
    if dma_isr & (1 << 1) != 0 {
        unsafe {
            (DMA1_IFCR as *mut u32).write_volatile(1 << 0); // CGIF1
            let ccr = DMA1_CCR1 as *mut u32;
            ccr.write_volatile(ccr.read_volatile() & !1); // Disable CH1
        }
        isr_handlers::handle_dma_tc();
        // Trigger software EXTI15
        unsafe { (EXTI_SWIER1 as *mut u32).write_volatile(1 << 15); }
    }
}

#[interrupt]
fn EXTI15_10() {
    unsafe { (EXTI_PR1 as *mut u32).write_volatile(1 << 15); }
    isr_handlers::handle_exti_frame();

    // Re-enable DMA CH1 for next frame
    let shared = crate::isr::shared();
    let sz = if shared.servo_pwm() { 2u32 } else { 32 };
    unsafe {
        (DMA1_CNDTR1 as *mut u32).write_volatile(sz);
        let ccr = DMA1_CCR1 as *mut u32;
        ccr.write_volatile(ccr.read_volatile() | 1); // Enable CH1
        // TIM15 CR1.CEN
        let cr1 = 0x4001_4000 as *mut u32;
        cr1.write_volatile(cr1.read_volatile() | 1);
    }
}
