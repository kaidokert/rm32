//! COMP2 initialization for BEMF zero-cross detection on STM32L431.
//!
//! L431 NEUTRON uses COMP2:
//!   INP: PB4 (IO1)
//!   INM: switched per step — PB7 (IO2), PA5 (IO5), PA4 (IO4)
//!   EXTI line 22

use crate::pac::{COMP, EXTI, GPIOA, GPIOB, RCC};

/// Initialize COMP2 for BEMF sensing on L431.
pub fn init_comp2() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    let gpiob = unsafe { &*GPIOB::ptr() };
    let comp = unsafe { &*COMP::ptr() };
    let exti = unsafe { &*EXTI::ptr() };

    unsafe {
        // Enable GPIOA, GPIOB clocks (AHB2ENR bits 0, 1)
        rcc.ahb2enr
            .modify(|_, w| w.gpioaen().set_bit().gpioben().set_bit());

        // PA4, PA5 as analog (INM inputs)
        gpioa
            .moder
            .modify(|_, w| w.moder4().bits(0b11).moder5().bits(0b11));
        // PB4 as analog (INP), PB7 as analog (INM)
        gpiob
            .moder
            .modify(|_, w| w.moder4().bits(0b11).moder7().bits(0b11));

        // Configure COMP2 via PAC
        comp.comp2_csr.write(|w| {
            w.comp2_inmsel()
                .bits(0b101) // PB7 = IO2
                .comp2_inpsel()
                .bits(0b00) // IO1 = PB4
                .comp2_pwrmode()
                .bits(0b00) // high speed
                .comp2_en()
                .set_bit()
        });

        // Wait for startup (~5us at 80MHz)
        cortex_m::asm::delay(400);

        // EXTI line 22 via PAC
        exti.imr1.modify(|r, w| w.bits(r.bits() & !(1 << 22)));
        exti.rtsr1.modify(|r, w| w.bits(r.bits() | (1 << 22)));
        exti.ftsr1.modify(|r, w| w.bits(r.bits() | (1 << 22)));
    }
}
