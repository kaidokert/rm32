//! COMP2 initialization for BEMF zero-cross detection.
//!
//! For HARDWARE_GROUP_G0_A (G071):
//!   Uses COMP2 (not COMP1)
//!   INP: PA3 (IO3)
//!   INM: switched per step — PB7 (IO2), PB3 (IO1), PA2 (IO3)
//!   EXTI line 18

use crate::pac::{COMP, EXTI, GPIOA, GPIOB, RCC};

/// Initialize COMP2 for BEMF sensing.
pub fn init_comp2() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    let gpiob = unsafe { &*GPIOB::ptr() };
    let comp = unsafe { &*COMP::ptr() };
    let exti = unsafe { &*EXTI::ptr() };

    // Enable SYSCFG clock
    rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // SYSCFGEN

    // Configure analog pins:
    // PA2 (COMP2_INM IO3), PA3 (COMP2_INP IO3) as analog
    gpioa.moder().modify(|r, w| unsafe {
        w.bits(r.bits() | (0b11 << 4) | (0b11 << 6)) // PA2, PA3 = analog
    });
    // PB3 (COMP2_INM IO1), PB7 (COMP2_INM IO2) as analog
    gpiob.moder().modify(|r, w| unsafe {
        w.bits(r.bits() | (0b11 << 6) | (0b11 << 14)) // PB3, PB7 = analog
    });

    // Configure COMP2:
    //   INMSEL = IO3 (PA2, initial — switched per step by change_input)
    //   INPSEL = IO3 (PA3)
    //   Hysteresis = none
    //   Polarity = non-inverted
    //   Power mode = high speed
    comp.comp2_csr().write(|w| unsafe {
        w.bits(
            ((0b1000 << 4)   // INMSEL = IO3 (PA2)
            | (0b10 << 8))  // power mode high speed
            | (1 << 0)      // EN = enable
        )
    });

    // Wait for startup (~5µs)
    cortex_m::asm::delay(320);

    // EXTI line 18: start with interrupts disabled
    exti.imr1().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 18)) });
    // Both edges initially (change_input will select rising or falling)
    exti.rtsr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 18)) });
    exti.ftsr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 18)) });
}
