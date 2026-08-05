use crate::comp_hal::{CompOps, ExtiOps};
use crate::comparator::BemfComparator;
use crate::pac::{COMP, EXTI};
use rm32::board::BemfPins;

pub struct L431Comp;
impl CompOps for L431Comp {
    fn output(&self) -> bool {
        let comp = unsafe { &*COMP::ptr() };
        comp.comp2_csr.read().bits() & (1 << 30) != 0
    }
    fn set_inmsel(&self, phase: u32) {
        // L431 COMP2 phase values are packed as (INMESEL << 8) | INMSEL.
        let comp = unsafe { &*COMP::ptr() };
        let inmsel = phase & 0x7;
        let inmesel = (phase >> 8) & 0x3;
        let v = comp.comp2_csr.read().bits();
        let cleared = v & !(0x7 << 4 | 0x3 << 25);
        comp.comp2_csr
            .write(|w| unsafe { w.bits(cleared | (inmsel << 4) | (inmesel << 25)) });
    }
}

pub struct L431Exti;
const LINE: u32 = 1 << 22;
impl ExtiOps for L431Exti {
    fn set_rising_edge(&self) {
        let exti = unsafe { &*EXTI::ptr() };
        exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() | LINE) });
        exti.ftsr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
    }
    fn set_falling_edge(&self) {
        let exti = unsafe { &*EXTI::ptr() };
        exti.rtsr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !LINE) });
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

pub type L431BemfComparator = BemfComparator<L431Comp, L431Exti>;

pub fn new_comparator(bemf_pins: BemfPins) -> L431BemfComparator {
    BemfComparator::new(L431Comp, L431Exti, bemf_pins)
}
