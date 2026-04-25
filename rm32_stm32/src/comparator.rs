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
