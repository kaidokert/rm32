use crate::comparator::BemfComparator;
use crate::comp_hal::{CompOps, ExtiOps, InmselMap};
use crate::pac::{COMP, EXTI};

const LINE_21: u32 = 1 << 21;
const LINE_22: u32 = 1 << 22;

/// Which comparator is currently active: false = COMP1, true = COMP2.
// SAFETY: ISR-local shared state — only accessed from COMP ISR and commutation
// ISR handlers which run at the same NVIC priority (no preemption between them).
// Single-core Cortex-M guarantees no concurrent access.
static mut ACTIVE_IS_COMP2: bool = true;

macro_rules! comp { () => { unsafe { &*COMP::PTR } } }
macro_rules! exti { () => { unsafe { &*EXTI::PTR } } }
#[inline(always)]
fn active_line() -> u32 { unsafe { if ACTIVE_IS_COMP2 { LINE_22 } else { LINE_21 } } }

/// G431 dual-comparator. Tracks active comp per commutation step.
pub struct G431Comp;
impl G431Comp { pub fn new() -> Self { Self } }

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
                comp!().c2csr().write(|w| w.bits(cleared | config | (1 << 0)));
            } else {
                let v = comp!().c1csr().read().bits();
                let cleared = v & !(0b111 << 4 | 0b11 << 2);
                comp!().c1csr().write(|w| w.bits(cleared | config | (1 << 0)));
            }
        }
    }
}

/// G431 EXTI — manages both lines 21 and 22.
pub struct G431Exti;
impl G431Exti { pub fn new() -> Self { Self } }

impl ExtiOps for G431Exti {
    fn set_rising_edge(&self) {
        let line = active_line();
        unsafe {
            exti!().rtsr1().modify(|r, w| w.bits(r.bits() & !(LINE_21 | LINE_22)));
            exti!().ftsr1().modify(|r, w| w.bits(r.bits() | line));
        }
    }
    fn set_falling_edge(&self) {
        let line = active_line();
        unsafe {
            exti!().rtsr1().modify(|r, w| w.bits(r.bits() | line));
            exti!().ftsr1().modify(|r, w| w.bits(r.bits() & !(LINE_21 | LINE_22)));
        }
    }
    fn enable_interrupt(&self) {
        unsafe { exti!().imr1().modify(|r, w| w.bits(r.bits() | active_line())); }
    }
    fn mask_and_clear(&self) {
        unsafe {
            exti!().imr1().modify(|r, w| w.bits(r.bits() & !(LINE_21 | LINE_22)));
            exti!().pr1().write(|w| w.bits(LINE_21 | LINE_22));
        }
    }
}

// Phase mapping for G4_B (PROTONDRIVE):
// Each value encodes [31:16]=INMSEL+INPSEL bits, [0]=comp selector (0=COMP1, 1=COMP2).
// Steps 1,4 (phase C): COMP1, INM=PA0(IO2=0b001), INP=PA1(IO1=0b00)
// Steps 2,5 (phase A): COMP2, INM=PA4(IO1=0b000), INP=PA3(IO2=0b01)
// Steps 3,6 (phase B): COMP1, INM=PA5(IO1=0b000), INP=PA1(IO1=0b00)
pub const INMSEL: InmselMap = InmselMap {
    // phase_a: COMP1, INM=PA4(000), INP=PA1(00) -> config = 0b000_00 << 2 = 0
    phase_a: ((0b000 << 4 | 0b00 << 2) << 16) as u32 | 0, // COMP1
    // phase_b: COMP2, INM=PA4(000), INP=PA3(01) -> config = 0b000_01 << 2
    phase_b: ((0b000 << 4 | 0b01 << 2) << 16) as u32 | 1, // COMP2
    // phase_c: COMP1, INM=PA0(001), INP=PA1(00) -> config = 0b001_00 << 2
    phase_c: ((0b001 << 4 | 0b00 << 2) << 16) as u32 | 0, // COMP1
};

pub type G431BemfComparator = BemfComparator<G431Comp, G431Exti>;

pub fn new_comparator() -> G431BemfComparator {
    BemfComparator::new(G431Comp::new(), G431Exti::new(), INMSEL)
}
