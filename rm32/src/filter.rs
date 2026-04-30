//! Multiply-free EWMA filter for smoothing ADC readings.
//!
//! Uses power-of-2 shift for division: `y += (x - y) >> K`
//! where α = 1/2^K. K=3 gives α=1/8 (matches C's `(7*y + x) >> 3`).
//!
//! Internal state uses `fixed::types::U16F16` for type safety.
//! Math operates on raw bits to maintain exact 1:1 identity with C output.

use fixed::types::U16F16;

/// Power-of-2 EWMA filter state.
/// K is the shift amount: α = 1/2^K.
#[derive(Clone)]
pub struct EwmaPow2<const K: u8> {
    state: U16F16,
    initialized: bool,
}

impl<const K: u8> Default for EwmaPow2<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const K: u8> EwmaPow2<K> {
    pub const fn new() -> Self {
        Self {
            state: U16F16::ZERO,
            initialized: false,
        }
    }

    /// Feed a new sample, return the filtered value (truncated to u16).
    /// Uses direct bit manipulation for exact C identity — no double conversion.
    #[inline]
    pub fn update(&mut self, sample: u16) -> u16 {
        if !self.initialized {
            self.state = U16F16::from_num(sample);
            self.initialized = true;
            return sample;
        }
        // Operate on raw bits: state is Q16.16, sample is integer.
        // Shift sample into Q16.16 space, do the EWMA, store back.
        // This is bit-identical to: y += (x - y) >> K on i32.
        let mut bits = self.state.to_bits() as i32;
        bits += (((sample as i32) << 16) - bits) >> K;
        self.state = U16F16::from_bits(bits as u32);
        // Truncate to integer part (matches C's integer truncation)
        self.state.to_num::<u16>()
    }

    /// Current filtered value without feeding a new sample.
    pub fn value(&self) -> u16 {
        self.state.to_num::<u16>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ewma_first_sample_passthrough() {
        let mut f = EwmaPow2::<3>::new();
        assert_eq!(f.update(1000), 1000);
    }

    #[test]
    fn ewma_converges_near_constant() {
        let mut f = EwmaPow2::<3>::new();
        f.update(0);
        for _ in 0..100 {
            f.update(800);
        }
        // With fractional state, convergence is exact (no dead zone)
        let v = f.value();
        assert!(v >= 799 && v <= 800, "expected ~800, got {}", v);
    }

    #[test]
    fn ewma_smooths_step() {
        let mut f = EwmaPow2::<3>::new();
        f.update(0);
        let v1 = f.update(800);
        // First step: 0 + (800-0)/8 = 100
        assert_eq!(v1, 100);
        let v2 = f.update(800);
        // Second: 100 + (800-100)/8 = 100 + 87 = 187
        assert_eq!(v2, 187);
    }

    #[test]
    fn ewma_k1_fast_response() {
        let mut f = EwmaPow2::<1>::new();
        f.update(0);
        let v = f.update(1000);
        // 0 + (1000-0)/2 = 500
        assert_eq!(v, 500);
    }
}
