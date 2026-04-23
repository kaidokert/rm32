//! F051 interrupt vectors — thin wrappers calling shared handlers.

use stm32f0xx_hal::pac::interrupt;
use crate::isr_handlers;

#[interrupt]
fn TIM6_DAC() {
    // Clear UIF
    unsafe { ((0x4000_1010u32) as *mut u32).write_volatile(0); } // TIM6 SR
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
    // DMA1 ISR at 0x4002_0000, IFCR at 0x4002_0004
    let dma_isr = unsafe { (0x4002_0000 as *const u32).read_volatile() };
    // Channel 5 TC flag = bit 17
    if dma_isr & (1 << 17) != 0 {
        unsafe { (0x4002_0004 as *mut u32).write_volatile(1 << 16); } // CGIF5
        // Disable DMA CH5 (CCR at base + 0x58)
        unsafe {
            let ccr = (0x4002_0058) as *mut u32;
            ccr.write_volatile(ccr.read_volatile() & !1);
        }
        isr_handlers::handle_dma_tc();
        // Trigger software EXTI15
        unsafe { (0x4001_0410 as *mut u32).write_volatile(1 << 15); } // EXTI SWIER
    }
}

#[interrupt]
fn EXTI4_15() {
    // Clear EXTI15 pending (PR at 0x4001_0414)
    unsafe { (0x4001_0414 as *mut u32).write_volatile(1 << 15); }
    isr_handlers::handle_exti_frame();

    // Re-enable DMA CH5 for next frame
    let shared = crate::isr::shared();
    let sz = if shared.servo_pwm() { 2u32 } else { 32 };
    unsafe {
        // DMA CH5 CNDTR at 0x4002_005C
        (0x4002_005C as *mut u32).write_volatile(sz);
        // Enable CH5
        let ccr = 0x4002_0058 as *mut u32;
        ccr.write_volatile(ccr.read_volatile() | 1);
    }
    // TIM15 CR1.CEN
    unsafe {
        let cr1 = 0x4001_4000 as *mut u32; // TIM15 base
        cr1.write_volatile(cr1.read_volatile() | 1);
    }
}
