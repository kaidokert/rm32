//! COMP1 initialization for BEMF zero-cross detection on STM32F051.
//!
//! F051 uses COMP1 (not COMP2 like G071).
//!   INP: PA1 (hardwired on COMP1)
//!   INM: switched per step — PA5, PA4, PA0
//!   EXTI line 21
//!
//! COMP CSR register on F051 is a single 32-bit register at COMP base + 0x1C
//! controlling COMP1 (bits [15:0]) and COMP2 (bits [31:16]).

const GPIOA_BASE: u32 = 0x4800_0000;
const GPIOA_MODER: u32 = GPIOA_BASE; // offset 0x00

const RCC_BASE: u32 = 0x4002_1000;
// F0 doesn't have a separate SYSCFG clock enable — COMP registers are memory-mapped

/// Initialize COMP1 for BEMF sensing.
pub fn init_comp1() {
    unsafe {
        // PA0, PA1, PA4, PA5 as analog mode (MODER = 0b11)
        let moder = GPIOA_MODER as *mut u32;
        let m = moder.read_volatile();
        let analog_bits = (0b11 << 0)   // PA0
            | (0b11 << 2)               // PA1
            | (0b11 << 8)               // PA4
            | (0b11 << 10);             // PA5
        moder.write_volatile(m | analog_bits);

        // Enable SYSCFG/COMP clock (APB2ENR bit 0)
        let apb2enr = (RCC_BASE + 0x18) as *mut u32;
        apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 0));

        // Configure COMP1 via CSR register (COMP base = 0x4001_001C on F0)
        // Initial config: PA5 as INM, COMP1 enabled, high speed
        // COMP_PA5 = 0b1010001 = INMSEL=101(PA5) | EN=1
        let comp_csr = 0x4001_001C as *mut u32;
        let v = comp_csr.read_volatile();
        // Clear COMP1 bits (lower 16 bits), keep COMP2 bits
        comp_csr.write_volatile((v & 0xFFFF_0000) | 0x51); // PA5, enabled

        // Wait for startup (~5us at 48MHz)
        cortex_m::asm::delay(240);

        // EXTI line 21: start with interrupts disabled
        // EXTI IMR at 0x4001_0400
        let imr = 0x4001_0400 as *mut u32;
        imr.write_volatile(imr.read_volatile() & !(1 << 21));
        // Both edges initially
        let rtsr = 0x4001_0408 as *mut u32;
        let ftsr = 0x4001_040C as *mut u32;
        rtsr.write_volatile(rtsr.read_volatile() | (1 << 21));
        ftsr.write_volatile(ftsr.read_volatile() | (1 << 21));
    }
}
