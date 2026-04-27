//! COMP1+COMP2 initialization for BEMF zero-cross detection on STM32G431.
//!
//! G431 uses dual comparators switched per commutation step:
//!   COMP1 (0x4001_0200): INP=PA1(IO1), INM per step — EXTI21
//!   COMP2 (0x4001_0204): INP=PA3(IO2), INM per step — EXTI22
//! BEMF inputs: PA0(INM IO2), PA4(INM IO1), PA5(INM IO1)

const RCC: u32 = 0x4002_1000;
const COMP1_CSR: u32 = 0x4001_0200;
const COMP2_CSR: u32 = 0x4001_0204;
const EXTI: u32 = 0x4000_0400;
const GPIOA: u32 = 0x4800_0000;

/// Initialize COMP1 and COMP2 for BEMF sensing on G431.
pub fn init_comp() {
    unsafe {
        use crate::regs::{modify as modify_reg, write};

        // Enable GPIOA clock (AHB2ENR bit 0)
        modify_reg(RCC + 0x4C, |v| v | (1 << 0));

        // PA0, PA1, PA3, PA4, PA5 as analog
        let moder = GPIOA as *mut u32;
        modify_reg(GPIOA, |v| {
            v | (0b11 << 0)   // PA0
              | (0b11 << 2)   // PA1
              | (0b11 << 6)   // PA3
              | (0b11 << 8)   // PA4
              | (0b11 << 10)  // PA5
        });

        // COMP1: INP=PA1(IO1=0b00), INM=PA4(IO1=0b000), high-speed, enable
        // CSR: [22]=POLARITY(0), [15:12]=BLANKING(0), [8:7]=HYST(00),
        //      [6:4]=INMSEL(000=PA4), [3:2]=INPSEL(00=PA1), [0]=EN
        write(COMP1_CSR, (0b000 << 4) | (0b00 << 2) | (1 << 0));

        // COMP2: INP=PA3(IO2=0b01), INM=PA5(IO1=0b000), high-speed, enable
        write(COMP2_CSR, (0b000 << 4) | (0b01 << 2) | (1 << 0));

        // Wait for comparator startup (~5us at 170MHz)
        cortex_m::asm::delay(850);

        // EXTI lines 21 (COMP1) and 22 (COMP2): enable both edge triggers
        // IMR1: don't enable yet (ISR logic enables when ready)
        modify_reg(EXTI + 0x00, |v| v & !((1 << 21) | (1 << 22))); // IMR1: mask both
        modify_reg(EXTI + 0x08, |v| v | (1 << 21) | (1 << 22));    // RTSR1: rising
        modify_reg(EXTI + 0x0C, |v| v | (1 << 21) | (1 << 22));    // FTSR1: falling
    }
}
