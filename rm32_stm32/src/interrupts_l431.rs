//! L431 interrupt vectors — thin wrappers calling shared handlers.
//! L431 uses TIM16 for commutation (shared IRQ with TIM1_UP).

use stm32l4xx_hal::pac::interrupt;
use crate::isr_handlers;

#[interrupt]
fn TIM6_DACUNDER() {
    unsafe { ((0x4000_1010u32) as *mut u32).write_volatile(0); } // TIM6 SR clear
    isr_handlers::handle_tim6();
}

#[interrupt]
fn TIM1_UP_TIM16() {
    // TIM16 is the commutation timer on L431
    // Clear TIM16 UIF (TIM16 base = 0x4001_4400)
    unsafe { ((0x4001_4410u32) as *mut u32).write_volatile(0); }
    isr_handlers::handle_tim14(); // same logic, different timer
}

#[interrupt]
fn COMP() {
    isr_handlers::handle_comp();
}

// DMA1 Channel 5: input capture transfer complete
#[interrupt]
fn DMA1_CH5() {
    // DMA1 ISR at 0x4002_0000, IFCR at 0x4002_0004
    let dma_isr = unsafe { (0x4002_0000 as *const u32).read_volatile() };
    // Channel 5 TC flag = bit 17
    if dma_isr & (1 << 17) != 0 {
        unsafe { (0x4002_0004 as *mut u32).write_volatile(1 << 16); } // CGIF5
        // Disable DMA CH5
        unsafe {
            let ccr = 0x4002_0058 as *mut u32;
            ccr.write_volatile(ccr.read_volatile() & !1);
        }
        isr_handlers::handle_dma_tc();
        // Trigger software EXTI15
        unsafe { (0x4001_0410 as *mut u32).write_volatile(1 << 15); } // EXTI SWIER1
    }
}

#[interrupt]
fn EXTI15_10() {
    unsafe { (0x4001_0414 as *mut u32).write_volatile(1 << 15); } // EXTI PR1
    isr_handlers::handle_exti_frame();

    // Re-enable DMA CH5 for next frame
    let shared = crate::isr::shared();
    let sz = if shared.servo_pwm() { 2u32 } else { 32 };
    unsafe {
        (0x4002_005C as *mut u32).write_volatile(sz); // DMA CH5 CNDTR
        let ccr = 0x4002_0058 as *mut u32;
        ccr.write_volatile(ccr.read_volatile() | 1); // Enable CH5
    }
    // TIM15 CR1.CEN
    unsafe {
        let cr1 = 0x4001_4000 as *mut u32;
        cr1.write_volatile(cr1.read_volatile() | 1);
    }
}
