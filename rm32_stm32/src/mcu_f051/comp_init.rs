//! COMP1 initialization for BEMF zero-cross detection on STM32F051.
//!
//! F051 uses COMP1 (not COMP2 like G071).
//!   INP: PA1 (hardwired on COMP1)
//!   INM: switched per step — PA5, PA4, PA0
//!   EXTI line 21

use crate::pac::{COMP, EXTI, GPIOA, RCC};

/// Initialize COMP1 for BEMF sensing.
pub fn init_comp1() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    let comp = unsafe { &*COMP::ptr() };
    let exti = unsafe { &*EXTI::ptr() };

    unsafe {
        // PA0, PA1, PA4, PA5 as analog mode
        gpioa.moder.modify(|_, w| {
            w.moder0()
                .analog()
                .moder1()
                .analog()
                .moder4()
                .analog()
                .moder5()
                .analog()
        });

        // Enable SYSCFG/COMP clock (APB2ENR bit 0)
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

        // Configure COMP1: PA5 as INM (INMSEL=101), enabled, high speed
        comp.csr.modify(|r, w| {
            // Clear COMP1 bits (lower 16), keep COMP2 bits (upper 16)
            let comp2_bits = r.bits() & 0xFFFF_0000;
            w.bits(comp2_bits)
                .comp1en()
                .enabled()
                .comp1insel()
                .bits(0b101) // PA5
                .comp1mode()
                .high_speed()
        });

        // Wait for startup (~5us at 48MHz)
        cortex_m::asm::delay(240);

        // EXTI line 21: start with interrupts disabled, both edges
        exti.imr.modify(|r, w| w.bits(r.bits() & !(1 << 21)));
        exti.rtsr.modify(|r, w| w.bits(r.bits() | (1 << 21)));
        exti.ftsr.modify(|r, w| w.bits(r.bits() | (1 << 21)));
    }
}
