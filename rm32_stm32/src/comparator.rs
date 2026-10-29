//! Generic BEMF comparator — MCU details via CompOps + ExtiOps traits.

use crate::comp_hal::{CompOps, ExtiOps};
use rm32::board::BemfPins;
use rm32::hal::{BemfSensor, Phase};

/// Generic BEMF comparator. Zero cfg blocks — MCU differences in trait impls.
pub struct BemfComparator<C: CompOps, E: ExtiOps> {
    comp: C,
    exti: E,
    bemf_pins: BemfPins,
}

impl<C: CompOps, E: ExtiOps> BemfComparator<C, E> {
    pub fn new(comp: C, exti: E, bemf_pins: BemfPins) -> Self {
        Self {
            comp,
            exti,
            bemf_pins,
        }
    }
}

impl<C: CompOps, E: ExtiOps> BemfSensor for BemfComparator<C, E> {
    fn level(&self) -> bool {
        self.comp.output()
    }

    #[inline(always)]
    fn select_phase(&mut self, phase: Phase, rising: bool) {
        let pin = match phase {
            Phase::A => self.bemf_pins.phase_a,
            Phase::B => self.bemf_pins.phase_b,
            Phase::C => self.bemf_pins.phase_c,
        };
        self.comp.set_inmsel(pin);

        if rising {
            self.exti.set_falling_edge();
        } else {
            self.exti.set_rising_edge();
        }
    }

    fn enable_irq(&mut self) {
        self.exti.enable_interrupt();
    }

    fn mask_irq(&mut self) {
        self.exti.mask_and_clear();
    }

    #[inline(always)]
    fn pending(&self) -> bool {
        self.exti.pending()
    }

    #[inline(always)]
    fn clear_pending(&mut self) {
        self.exti.clear_pending();
    }

    #[inline(always)]
    fn repend(&mut self) {
        self.exti.repend();
    }
}
