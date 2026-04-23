//! COMP2 initialization for BEMF zero-cross detection on STM32L431.
//!
//! L431 NEUTRON uses COMP2:
//!   INP: PB4 (IO1)
//!   INM: switched per step — PB7 (IO2), PA5 (IO5), PA4 (IO4)
//!   EXTI line 22
//!
//! COMP2_CSR at 0x4001_0204

const RCC_BASE: u32 = 0x4002_1000;
const GPIOA_BASE: u32 = 0x4800_0000;
const GPIOB_BASE: u32 = 0x4800_0400;
const COMP2_CSR: u32 = 0x4001_0204;
const EXTI_BASE: u32 = 0x4001_0400;

// L4 EXTI register offsets: IMR1=0x00, EMR1=0x04, RTSR1=0x08, FTSR1=0x0C, SWIER1=0x10, PR1=0x14

#[inline(always)]
unsafe fn modify_reg(addr: u32, f: impl FnOnce(u32) -> u32) {
    let ptr = addr as *mut u32;
    ptr.write_volatile(f(ptr.read_volatile()));
}

/// Initialize COMP2 for BEMF sensing on L431.
pub fn init_comp2() {
    unsafe {
        // Enable GPIOA, GPIOB clocks (AHB2ENR bits 0, 1)
        modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 0) | (1 << 1));

        // PA4, PA5 as analog (INM inputs)
        modify_reg(GPIOA_BASE, |v| v | (0b11 << 8) | (0b11 << 10));
        // PB4 as analog (INP), PB7 as analog (INM)
        modify_reg(GPIOB_BASE, |v| v | (0b11 << 8) | (0b11 << 14));

        // Configure COMP2_CSR:
        //   INMSEL = 0b101 (PB7 = IO2, initial — switched per step by change_input)
        //   INPSEL = 0b00 (IO1 = PB4)
        //   PWRMODE = 0b00 (high speed)
        //   Polarity = non-inverted
        //   EN = 1
        (COMP2_CSR as *mut u32).write_volatile(
            (0b101 << 4)    // INMSEL = IO2 (PB7)
            | (0b00 << 7)   // INPSEL = IO1 (PB4)
            | (0b00 << 2)   // PWRMODE = high speed
            | (1 << 0)      // EN
        );

        // Wait for startup (~5us at 80MHz)
        cortex_m::asm::delay(400);

        // EXTI line 22: start with interrupts disabled
        modify_reg(EXTI_BASE, |v| v & !(1 << 22)); // IMR1
        // Both edges initially
        modify_reg(EXTI_BASE + 0x08, |v| v | (1 << 22)); // RTSR1
        modify_reg(EXTI_BASE + 0x0C, |v| v | (1 << 22)); // FTSR1
    }
}
