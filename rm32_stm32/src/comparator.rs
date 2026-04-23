//! BEMF comparator — MCU-specific.
//!
//! G071: COMP2 on EXTI18
//! F051: COMP1 on EXTI21

use crate::pac::{COMP, EXTI};
use rm32::hal::Comparator as CompTrait;

const EXTI_LINE: u32 = 1 << crate::config::COMP_EXTI_LINE;

#[cfg(feature = "stm32g071")]
mod inmsel {
    pub const PHASE_A: u32 = 0b0110;
    pub const PHASE_B: u32 = 0b0111;
    pub const PHASE_C: u32 = 0b1000;
    pub const INP: u32 = 0b10;
}

#[cfg(feature = "stm32l431")]
mod inmsel {
    // COMP2 INMSEL for L431
    pub const PHASE_A: u32 = 0b0101; // IO2 = PB7  (INM5)
    pub const PHASE_B: u32 = 0b0100; // IO5 = PA5  (INM4)
    pub const PHASE_C: u32 = 0b0011; // IO4 = PA4  (INM3)
    pub const INP: u32 = 0b00;       // IO1
}

#[cfg(feature = "stm32f051")]
mod inmsel {
    pub const PHASE_A: u32 = 0b1010001; // PA5
    pub const PHASE_B: u32 = 0b1000001; // PA4
    pub const PHASE_C: u32 = 0b1100001; // PA0
}

pub struct BemfComparator {
    step: u8,
    rising: bool,
}

impl BemfComparator {
    pub fn new() -> Self {
        Self { step: 1, rising: true }
    }

    pub fn set_step(&mut self, step: u8, rising: bool) {
        self.step = step;
        self.rising = rising;
    }
}

impl CompTrait for BemfComparator {
    fn set_step(&mut self, step: u8, rising: bool) {
        self.step = step;
        self.rising = rising;
    }

    fn output_level(&self) -> bool {
        let comp = unsafe { &*COMP::ptr() };
        #[cfg(feature = "stm32g071")]
        { return comp.comp2_csr().read().bits() & (1 << 30) != 0; }
        #[cfg(feature = "stm32f051")]
        {
            let csr = unsafe { *((COMP::ptr() as u32 + 0x1C) as *const u32) };
            return csr & (1 << 30) != 0;
        }
        #[cfg(feature = "stm32l431")]
        {
            // COMP2_CSR at 0x4001_0204 on L4
            let csr = unsafe { *(0x4001_0204 as *const u32) };
            return csr & (1 << 30) != 0;
        }
    }

    fn change_input(&mut self) {
        let comp = unsafe { &*COMP::ptr() };
        let _exti = unsafe { &*EXTI::ptr() };

        let phase = match self.step {
            1 | 4 => inmsel::PHASE_C,
            2 | 5 => inmsel::PHASE_A,
            3 | 6 => inmsel::PHASE_B,
            _ => inmsel::PHASE_C,
        };

        #[cfg(feature = "stm32g071")]
        comp.comp2_csr().modify(|r, w| unsafe {
            let cleared = r.bits() & !(0xF << 4 | 0x3 << 8);
            w.bits(cleared | (phase << 4) | (inmsel::INP << 8))
        });

        #[cfg(feature = "stm32l431")]
        unsafe {
            let csr = 0x4001_0204 as *mut u32; // COMP2_CSR on L4
            let v = csr.read_volatile();
            csr.write_volatile((v & !(0xF << 4 | 0x3 << 8)) | (phase << 4) | (inmsel::INP << 8));
        }

        #[cfg(feature = "stm32f051")]
        unsafe {
            let csr = (COMP::ptr() as u32 + 0x1C) as *mut u32;
            csr.write_volatile(phase);
        }

        // EXTI edge trigger
        #[cfg(feature = "stm32g071")]
        {
            let exti = unsafe { &*EXTI::ptr() };
            if self.rising {
                exti.rtsr1().modify(|r, w| unsafe { w.bits(r.bits() & !EXTI_LINE) });
                exti.ftsr1().modify(|r, w| unsafe { w.bits(r.bits() | EXTI_LINE) });
            } else {
                exti.rtsr1().modify(|r, w| unsafe { w.bits(r.bits() | EXTI_LINE) });
                exti.ftsr1().modify(|r, w| unsafe { w.bits(r.bits() & !EXTI_LINE) });
            }
        }
        #[cfg(feature = "stm32l431")]
        unsafe {
            // L4 EXTI: IMR1=0x00, EMR1=0x04, RTSR1=0x08, FTSR1=0x0C
            let rtsr1 = 0x4001_0408 as *mut u32;
            let ftsr1 = 0x4001_040C as *mut u32;
            if self.rising {
                rtsr1.write_volatile(rtsr1.read_volatile() & !EXTI_LINE);
                ftsr1.write_volatile(ftsr1.read_volatile() | EXTI_LINE);
            } else {
                rtsr1.write_volatile(rtsr1.read_volatile() | EXTI_LINE);
                ftsr1.write_volatile(ftsr1.read_volatile() & !EXTI_LINE);
            }
        }
        #[cfg(feature = "stm32f051")]
        unsafe {
            // EXTI RTSR at offset 0x08, FTSR at 0x0C from EXTI base (0x40010400)
            let rtsr = 0x4001_0408 as *mut u32;
            let ftsr = 0x4001_040C as *mut u32;
            if self.rising {
                rtsr.write_volatile(rtsr.read_volatile() & !EXTI_LINE);
                ftsr.write_volatile(ftsr.read_volatile() | EXTI_LINE);
            } else {
                rtsr.write_volatile(rtsr.read_volatile() | EXTI_LINE);
                ftsr.write_volatile(ftsr.read_volatile() & !EXTI_LINE);
            }
        }
    }

    fn enable_interrupts(&mut self) {
        #[cfg(feature = "stm32g071")]
        {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr1().modify(|r, w| unsafe { w.bits(r.bits() | EXTI_LINE) });
        }
        #[cfg(feature = "stm32l431")]
        unsafe {
            // IMR1 at EXTI base + 0x00 on L4 is actually at 0x4001_0400 + offset
            // L4 EXTI IMR1 is at offset 0x00 from EXTI base
            let imr1 = (EXTI::ptr() as u32) as *mut u32;
            imr1.write_volatile(imr1.read_volatile() | EXTI_LINE);
        }
        #[cfg(feature = "stm32f051")]
        unsafe {
            let imr = 0x4001_0400 as *mut u32;
            imr.write_volatile(imr.read_volatile() | EXTI_LINE);
        }
    }

    fn mask_interrupts(&mut self) {
        #[cfg(feature = "stm32g071")]
        {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr1().modify(|r, w| unsafe { w.bits(r.bits() & !EXTI_LINE) });
            exti.rpr1().write(|w| unsafe { w.bits(EXTI_LINE) });
            exti.fpr1().write(|w| unsafe { w.bits(EXTI_LINE) });
        }
        #[cfg(feature = "stm32l431")]
        unsafe {
            let exti_base = EXTI::ptr() as u32;
            let imr1 = exti_base as *mut u32;
            imr1.write_volatile(imr1.read_volatile() & !EXTI_LINE);
            // L4 PR1 at offset 0x14
            let pr1 = (exti_base + 0x14) as *mut u32;
            pr1.write_volatile(EXTI_LINE);
        }
        #[cfg(feature = "stm32f051")]
        unsafe {
            let imr = 0x4001_0400 as *mut u32;
            let pr = 0x4001_0414 as *mut u32;
            imr.write_volatile(imr.read_volatile() & !EXTI_LINE);
            pr.write_volatile(EXTI_LINE);
        }
    }
}
