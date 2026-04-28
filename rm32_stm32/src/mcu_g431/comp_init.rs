//! COMP1+COMP2 initialization for BEMF zero-cross detection on STM32G431.
//!
//! G431 uses dual comparators switched per commutation step:
//!   COMP1: INP=PA1(IO1), INM per step — EXTI21
//!   COMP2: INP=PA3(IO2), INM per step — EXTI22
//! BEMF inputs: PA0(INM IO2), PA4(INM IO1), PA5(INM IO1)

use crate::pac;

/// Initialize COMP1 and COMP2 for BEMF sensing on G431.
pub fn init_comp() {
    let rcc = unsafe { &*pac::RCC::PTR };
    let gpioa = unsafe { &*pac::GPIOA::PTR };
    let comp = unsafe { &*pac::COMP::PTR };
    let exti = unsafe { &*pac::EXTI::PTR };

    unsafe {
        // Enable GPIOA clock
        rcc.ahb2enr().modify(|_, w| w.gpioaen().set_bit());

        // PA0, PA1, PA3, PA4, PA5 as analog
        gpioa.moder().modify(|_, w| {
            w.moder0()
                .bits(0b11)
                .moder1()
                .bits(0b11)
                .moder3()
                .bits(0b11)
                .moder4()
                .bits(0b11)
                .moder5()
                .bits(0b11)
        });

        // COMP1: INP=PA1(IO1), INM=PA4(IO1=0b000), enable
        comp.c1csr().write(|w| {
            w.inmsel()
                .bits(0b000)
                .inpsel()
                .bit(false) // IO1 = PA1
                .en()
                .set_bit()
        });

        // COMP2: INP=PA3(IO2), INM=PA5(IO1=0b000), enable
        comp.c2csr().write(|w| {
            w.inmsel()
                .bits(0b000)
                .inpsel()
                .bit(true) // IO2 = PA3
                .en()
                .set_bit()
        });

        // Wait for comparator startup (~5us at 170MHz)
        cortex_m::asm::delay(850);

        // EXTI lines 21 (COMP1) and 22 (COMP2)
        exti.imr1()
            .modify(|_, w| w.im21().clear_bit().im22().clear_bit());
        exti.rtsr1()
            .modify(|_, w| w.rt21().set_bit().rt22().set_bit());
        exti.ftsr1()
            .modify(|_, w| w.ft21().set_bit().ft22().set_bit());
    }
}
