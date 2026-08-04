use crate::comp_hal::{CompOps, ExtiOps};
use crate::comparator::BemfComparator;
use crate::pac::{COMP, EXTI};
use rm32::board::BemfPins;

const LINE_21: u32 = 1 << 21;
const LINE_22: u32 = 1 << 22;

/// Which comparator is currently active: false = COMP1, true = COMP2.
// SAFETY: ISR-local shared state — only accessed from COMP ISR and commutation
// ISR handlers which run at the same NVIC priority (no preemption between them).
// Single-core Cortex-M guarantees no concurrent access.
static mut ACTIVE_IS_COMP2: bool = true;

macro_rules! comp {
    () => {
        &*COMP::PTR
    };
}
macro_rules! exti {
    () => {
        &*EXTI::PTR
    };
}
#[inline]
fn active_line() -> u32 {
    unsafe { if ACTIVE_IS_COMP2 { LINE_22 } else { LINE_21 } }
}

/// G431 dual-comparator. Tracks active comp per commutation step.
pub struct G431Comp;
impl G431Comp {
    pub fn new() -> Self {
        Self
    }
}

impl CompOps for G431Comp {
    fn output(&self) -> bool {
        unsafe {
            if ACTIVE_IS_COMP2 {
                comp!().c2csr().read().bits() & (1 << 30) != 0
            } else {
                comp!().c1csr().read().bits() & (1 << 30) != 0
            }
        }
    }
    fn set_inmsel(&self, phase: u32) {
        // phase encodes: [31:16]=INM/INP config bits, [15:0]=comp selector
        // Lower bit 0 of [15:0]: 0 = COMP1 (c1csr addr), 1 = COMP2 (c2csr addr)
        let is_comp2 = (phase & 1) != 0;
        let config = phase >> 16;
        unsafe {
            ACTIVE_IS_COMP2 = is_comp2;
            if is_comp2 {
                let v = comp!().c2csr().read().bits();
                let cleared = v & !(0b111 << 4 | 0b11 << 2);
                comp!()
                    .c2csr()
                    .write(|w| w.bits(cleared | config | (1 << 0)));
            } else {
                let v = comp!().c1csr().read().bits();
                let cleared = v & !(0b111 << 4 | 0b11 << 2);
                comp!()
                    .c1csr()
                    .write(|w| w.bits(cleared | config | (1 << 0)));
            }
        }
    }
}

/// G431 EXTI — manages both lines 21 and 22.
pub struct G431Exti;
impl G431Exti {
    pub fn new() -> Self {
        Self
    }
}

impl ExtiOps for G431Exti {
    fn set_rising_edge(&self) {
        let line = active_line();
        unsafe {
            exti!()
                .rtsr1()
                .modify(|r, w| w.bits(r.bits() & !(LINE_21 | LINE_22)));
            exti!().ftsr1().modify(|r, w| w.bits(r.bits() | line));
        }
    }
    fn set_falling_edge(&self) {
        let line = active_line();
        unsafe {
            exti!().rtsr1().modify(|r, w| w.bits(r.bits() | line));
            exti!()
                .ftsr1()
                .modify(|r, w| w.bits(r.bits() & !(LINE_21 | LINE_22)));
        }
    }
    fn enable_interrupt(&self) {
        unsafe {
            exti!()
                .imr1()
                .modify(|r, w| w.bits(r.bits() | active_line()));
        }
    }
    fn mask_and_clear(&self) {
        unsafe {
            exti!()
                .imr1()
                .modify(|r, w| w.bits(r.bits() & !(LINE_21 | LINE_22)));
            exti!().pr1().write(|w| w.bits(LINE_21 | LINE_22));
        }
    }
}

pub type G431BemfComparator = BemfComparator<G431Comp, G431Exti>;

pub fn new_comparator(bemf_pins: BemfPins) -> G431BemfComparator {
    BemfComparator::new(G431Comp::new(), G431Exti::new(), bemf_pins)
}
