//! Generic BEMF comparator — MCU details via CompOps + ExtiOps traits.

use crate::board_ext::BemfPins;
use crate::comp_hal::{CompOps, ExtiOps};
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

    #[allow(unexpected_cfgs)]
    fn filter_read(&self) -> bool {
        // STM32G071: match AM32's persistence-filter sampling window. AM32's
        // non-inlined `interruptRoutine` -> `getCompOutputLevel` loop takes
        // ~30.6 CPU cycles per sample at 64 MHz (with `FLASH.ACR.PRFTEN` set),
        // whereas an inlined `COMP2.CSR` read takes ~12.6 cycles, halving the
        // filter time window and letting sub-1.5 us switching-noise pulses pass.
        // 14 single-cycle NOPs pad only the persistence-filter reads (`level()`
        // stays unpadded for fast gate/camping and polling checks).
        #[cfg(all(feature = "stm32g071", target_arch = "arm"))]
        unsafe {
            core::arch::asm!(
                ".rept 14",
                "nop",
                ".endr",
                options(nomem, nostack, preserves_flags)
            );
        }
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
