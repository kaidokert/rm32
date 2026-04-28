use crate::comparator::BemfComparator;
use crate::comp_hal::{CompOps, ExtiOps, InmselMap};
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
