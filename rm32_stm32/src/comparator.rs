//! Generic BEMF comparator — MCU details via CompOps + ExtiOps traits.

use crate::comp_hal::{CompOps, ExtiOps, InmselMap};
use rm32::hal::Comparator as CompTrait;

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
        Self {
            step: 1,
            rising: true,
            comp,
            exti,
            inmsel,
        }
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
