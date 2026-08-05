use crate::comp_hal::{CompOps, ExtiOps};
use crate::comparator::BemfComparator;
use crate::pac::{COMP, EXTI};
use rm32::board::BemfPins;

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

pub type F051BemfComparator = BemfComparator<F051Comp, F051Exti>;

pub fn new_comparator(bemf_pins: BemfPins) -> F051BemfComparator {
    BemfComparator::new(F051Comp, F051Exti, bemf_pins)
}
