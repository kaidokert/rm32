//! Generic BEMF comparator — MCU details via CompOps + ExtiOps traits.

use rm32::hal::Comparator as CompTrait;
use crate::comp_hal::{CompOps, ExtiOps, InmselMap};

/// Generic BEMF comparator. Zero cfg blocks — MCU differences in trait impls.
pub struct BemfComparator<C: CompOps, E: ExtiOps> {
    step: u8,
    rising: bool,
    comp: C,
    exti: E,
    inmsel: InmselMap,
}

impl<C: CompOps, E: ExtiOps> BemfComparator<C, E> {
    pub fn new(comp: C, exti: E, inmsel: InmselMap) -> Self {
        Self { step: 1, rising: true, comp, exti, inmsel }
    }
}

impl<C: CompOps, E: ExtiOps> CompTrait for BemfComparator<C, E> {
    fn set_step(&mut self, step: u8, rising: bool) {
        self.step = step;
        self.rising = rising;
    }

    fn output_level(&self) -> bool {
        self.comp.output()
    }

    fn change_input(&mut self) {
        let phase = match self.step {
            1 | 4 => self.inmsel.phase_c,
            2 | 5 => self.inmsel.phase_a,
            3 | 6 => self.inmsel.phase_b,
            _ => self.inmsel.phase_c,
        };
        self.comp.set_inmsel(phase);

        if self.rising {
            self.exti.set_falling_edge();
        } else {
            self.exti.set_rising_edge();
        }
    }

    fn enable_interrupts(&mut self) {
        self.exti.enable_interrupt();
    }

    fn mask_interrupts(&mut self) {
        self.exti.mask_and_clear();
    }
}

// ============================================================
// G431: Dual COMP1 (EXTI21) + COMP2 (EXTI22), switched per step
// ============================================================
#[cfg(feature = "stm32g431")]
pub mod g431 {
    use super::*;

    const COMP1_CSR: u32 = 0x4001_0200;
    const COMP2_CSR: u32 = 0x4001_0204;
    const EXTI: u32 = 0x4000_0400;
    const LINE_21: u32 = 1 << 21;
    const LINE_22: u32 = 1 << 22;

    // SAFETY: ISR-local shared state — only accessed from COMP ISR and commutation
    // ISR handlers which run at the same NVIC priority (no preemption between them).
    // Single-core Cortex-M guarantees no concurrent access.
    static mut ACTIVE_CSR: u32 = COMP2_CSR;
    static mut ACTIVE_LINE: u32 = LINE_22;

    /// G431 dual-comparator. Tracks active comp per commutation step.
    pub struct G431Comp;
    impl G431Comp { pub fn new() -> Self { Self } }

    impl CompOps for G431Comp {
        fn output(&self) -> bool {
            unsafe {
                let csr = ACTIVE_CSR as *const u32;
                csr.read_volatile() & (1 << 30) != 0
            }
        }
        fn set_inmsel(&self, phase: u32) {
            // phase encodes: [31:16]=INM/INP config bits, [15:0]=COMP CSR address
            let comp_base = phase & 0xFFFF;
            let config = phase >> 16;
            unsafe {
                ACTIVE_CSR = comp_base;
                ACTIVE_LINE = if comp_base == COMP1_CSR { LINE_21 } else { LINE_22 };
                let csr = comp_base as *mut u32;
                let v = csr.read_volatile();
                let cleared = v & !(0b111 << 4 | 0b11 << 2);
                csr.write_volatile(cleared | config | (1 << 0));
            }
        }
    }

    /// G431 EXTI — manages both lines 21 and 22.
    pub struct G431Exti;
    impl G431Exti { pub fn new() -> Self { Self } }

    impl ExtiOps for G431Exti {
        fn set_rising_edge(&self) {
            unsafe {
                let line = ACTIVE_LINE;
                crate::regs::modify(EXTI + 0x08, |v| v & !(LINE_21 | LINE_22));
                crate::regs::modify(EXTI + 0x0C, |v| v | line);
            }
        }
        fn set_falling_edge(&self) {
            unsafe {
                let line = ACTIVE_LINE;
                crate::regs::modify(EXTI + 0x08, |v| v | line);
                crate::regs::modify(EXTI + 0x0C, |v| v & !(LINE_21 | LINE_22));
            }
        }
        fn enable_interrupt(&self) {
            unsafe { crate::regs::modify(EXTI + 0x00, |v| v | ACTIVE_LINE); }
        }
        fn mask_and_clear(&self) {
            unsafe {
                crate::regs::modify(EXTI + 0x00, |v| v & !(LINE_21 | LINE_22));
                ((EXTI + 0x14) as *mut u32).write_volatile(LINE_21 | LINE_22);
            }
        }
    }

    // Phase mapping for G4_B (PROTONDRIVE):
    // Each value encodes [31:16]=INMSEL+INPSEL bits, [15:0]=COMP base address.
    // Steps 1,4 (phase C): COMP1, INM=PA0(IO2=0b001), INP=PA1(IO1=0b00)
    // Steps 2,5 (phase A): COMP2, INM=PA4(IO1=0b000), INP=PA3(IO2=0b01)
    // Steps 3,6 (phase B): COMP1, INM=PA5(IO1=0b000), INP=PA1(IO1=0b00)
    //
    // But the generic BemfComparator calls change_input() which calls set_inmsel()
    // with the phase value, then calls exti edge set. We need the EXTI to know
    // which line is active. We'll handle this by storing the EXTI line in the
    // phase value and using a custom wrapper.

    // Simplified: encode comp_addr in lower 16 bits
    pub const INMSEL: InmselMap = InmselMap {
        // phase_a: COMP1, INM=PA4(000), INP=PA1(00) → config = 0b000_00 << 2 = 0
        phase_a: ((0b000 << 4 | 0b00 << 2) << 16) as u32 | COMP1_CSR,
        // phase_b: COMP2, INM=PA4(000), INP=PA3(01) → config = 0b000_01 << 2
        phase_b: ((0b000 << 4 | 0b01 << 2) << 16) as u32 | COMP2_CSR,
        // phase_c: COMP1, INM=PA0(001), INP=PA1(00) → config = 0b001_00 << 2
        phase_c: ((0b001 << 4 | 0b00 << 2) << 16) as u32 | COMP1_CSR,
    };

    pub type G431BemfComparator = BemfComparator<G431Comp, G431Exti>;

    pub fn new_comparator() -> G431BemfComparator {
        BemfComparator::new(G431Comp::new(), G431Exti::new(), INMSEL)
    }
}

// ============================================================
// G071: COMP2 on EXTI18
// ============================================================
#[cfg(feature = "stm32g071")]
pub mod g071 {
    use super::*;
    use crate::pac::{COMP, EXTI};

    pub struct G071Comp;
    impl CompOps for G071Comp {
        fn output(&self) -> bool {
            let comp = unsafe { &*COMP::ptr() };
            comp.comp2_csr().read().bits() & (1 << 30) != 0
        }
        fn set_inmsel(&self, phase: u32) {
            let comp = unsafe { &*COMP::ptr() };
            comp.comp2_csr().modify(|r, w| unsafe {
                let cleared = r.bits() & !(0xF << 4 | 0x3 << 8);
                w.bits(cleared | (phase << 4) | (0b10 << 8)) // INP = IO3
            });
        }
    }

    pub struct G071Exti;
    const LINE: u32 = 1 << 18;
    impl ExtiOps for G071Exti {
        fn set_rising_edge(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.rtsr1().modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
            exti.ftsr1().modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
        }
        fn set_falling_edge(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.rtsr1().modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
            exti.ftsr1().modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        }
        fn enable_interrupt(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr1().modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        }
        fn mask_and_clear(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr1().modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
            exti.rpr1().write(|w| unsafe { w.bits(LINE) });
            exti.fpr1().write(|w| unsafe { w.bits(LINE) });
        }
    }

    pub const INMSEL: InmselMap = InmselMap {
        phase_a: 0b0110, phase_b: 0b0111, phase_c: 0b1000,
    };

    pub type G071BemfComparator = BemfComparator<G071Comp, G071Exti>;

    pub fn new_comparator() -> G071BemfComparator {
        BemfComparator::new(G071Comp, G071Exti, INMSEL)
    }
}

// ============================================================
// F051: COMP1 on EXTI21
// ============================================================
#[cfg(feature = "stm32f051")]
pub mod f051 {
    use super::*;
    use crate::pac::{COMP, EXTI};

    pub struct F051Comp;
    impl CompOps for F051Comp {
        fn output(&self) -> bool {
            let comp = unsafe { &*COMP::ptr() };
            comp.csr.read().bits() & (1 << 30) != 0
        }
        fn set_inmsel(&self, phase: u32) {
            // F051 COMP1: whole CSR lower 16 bits = phase value
            let comp = unsafe { &*COMP::ptr() };
            let upper = comp.csr.read().bits() & 0xFFFF_0000;
            comp.csr.write(|w| unsafe { w.bits(upper | phase) });
        }
    }

    pub struct F051Exti;
    const LINE: u32 = 1 << 21;
    impl ExtiOps for F051Exti {
        fn set_rising_edge(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
        }
        fn set_falling_edge(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.rtsr.modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
            exti.ftsr.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        }
        fn enable_interrupt(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        }
        fn mask_and_clear(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr.modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
            exti.pr.write(|w| unsafe { w.bits(LINE) });
        }
    }

    pub const INMSEL: InmselMap = InmselMap {
        phase_a: 0b1010001, phase_b: 0b1000001, phase_c: 0b1100001,
    };

    pub type F051BemfComparator = BemfComparator<F051Comp, F051Exti>;

    pub fn new_comparator() -> F051BemfComparator {
        BemfComparator::new(F051Comp, F051Exti, INMSEL)
    }
}

// ============================================================
// L431: COMP2 on EXTI22
// ============================================================
#[cfg(feature = "stm32l431")]
pub mod l431 {
    use super::*;
    use crate::pac::{COMP, EXTI};

    pub struct L431Comp;
    impl CompOps for L431Comp {
        fn output(&self) -> bool {
            let comp = unsafe { &*COMP::ptr() };
            comp.comp2_csr.read().bits() & (1 << 30) != 0
        }
        fn set_inmsel(&self, phase: u32) {
            let comp = unsafe { &*COMP::ptr() };
            let v = comp.comp2_csr.read().bits();
            comp.comp2_csr.write(|w| unsafe {
                w.bits((v & !(0xF << 4 | 0x3 << 8)) | (phase << 4))
            });
        }
    }

    pub struct L431Exti;
    const LINE: u32 = 1 << 22;
    impl ExtiOps for L431Exti {
        fn set_rising_edge(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
            exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
        }
        fn set_falling_edge(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
            exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        }
        fn enable_interrupt(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr1.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        }
        fn mask_and_clear(&self) {
            let exti = unsafe { &*EXTI::ptr() };
            exti.imr1.modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
            // L4: PR1 at offset 0x14
            exti.pr1.write(|w| unsafe { w.bits(LINE) });
        }
    }

    pub const INMSEL: InmselMap = InmselMap {
        phase_a: 0b0101, phase_b: 0b0100, phase_c: 0b0011,
    };

    pub type L431BemfComparator = BemfComparator<L431Comp, L431Exti>;

    pub fn new_comparator() -> L431BemfComparator {
        BemfComparator::new(L431Comp, L431Exti, INMSEL)
    }
}
