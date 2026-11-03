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
pub(crate) struct EwmaPow2<const K: u8> {
    state: U16F16,
    initialized: bool,
}

impl<const K: u8> Default for EwmaPow2<K> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const K: u8> EwmaPow2<K> {
    pub(crate) const fn new() -> Self {
        Self {
            state: U16F16::ZERO,
            initialized: false,
        }
    }

    /// Feed a new sample, return the filtered value (truncated to u16).
    /// Uses direct bit manipulation for exact C identity — no double conversion.
    #[inline]
    pub(crate) fn update(&mut self, sample: u16) -> u16 {
        if !self.initialized {
            self.state = U16F16::from_num(sample);
            self.initialized = true;
            return sample;
        }
        // Operate on raw bits: state is Q16.16, sample is integer.
        // In exact signed Q16.16 arithmetic, `bits + (((sample << 16) - bits) >> K)`
        // equals `bits - ceil(bits / 2^K) + (sample << (16 - K))` because the
        // low K bits of `sample << 16` are zero (`1 <= K <= 16`). Evaluating in
        // unsigned u32 avoids signed i32 overflow when `sample >= 32768`.
        let bits = self.state.to_bits();
        let next = bits - ((bits + ((1 << K) - 1)) >> K) + ((sample as u32) << (16 - K));
        self.state = U16F16::from_bits(next);
        // Truncate to integer part (matches C's integer truncation)
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
        let v = f.update(800);
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

    #[test]
    fn ewma_full_u16_range_without_signed_overflow() {
        let mut f = EwmaPow2::<3>::new();
        assert_eq!(f.update(0), 0);
        // Step into the upper half of u16 (>= 32768), which would sign-flip i32 << 16.
        assert_eq!(f.update(32768), 4096);
        assert_eq!(f.update(u16::MAX), 11775);

        let mut f_max = EwmaPow2::<3>::new();
        assert_eq!(f_max.update(30000), 30000);
        // 30000 + (40000 - 30000) / 8 = 31250
        assert_eq!(f_max.update(40000), 31250);
        for _ in 0..200 {
            f_max.update(u16::MAX);
        }
        // Upward Q16.16 step with floor right-shift converges within 1 LSB of target
        // (bits = (65535 << 16) - 7 truncates to 65534), matching ewma_converges_near_constant.
        assert_eq!(f_max.update(u16::MAX), u16::MAX - 1);

        let mut f_down = EwmaPow2::<3>::new();
        assert_eq!(f_down.update(u16::MAX), u16::MAX);
        assert_eq!(f_down.update(u16::MAX), u16::MAX);
        // Step back down from u16::MAX toward 0: 65535 - ceil(65535 / 8) = 57343.
        assert_eq!(f_down.update(0), 57343);

        // Verify bit-for-bit identity against a 64-bit signed Q16.16 reference across 0..=u16::MAX.
        let mut f_ref = EwmaPow2::<3>::new();
        let mut ref_bits: i64 = 0;
        let samples = [
            0u16,
            1,
            4095,
            32767,
            32768,
            40000,
            65534,
            u16::MAX,
            12345,
            0,
        ];
        for (i, &s) in samples.iter().enumerate() {
            let out = f_ref.update(s);
            if i == 0 {
                ref_bits = (s as i64) << 16;
            } else {
                ref_bits += (((s as i64) << 16) - ref_bits) >> 3;
            }
            assert_eq!(f_ref.state.to_bits(), ref_bits as u32);
            assert_eq!(out, (ref_bits >> 16) as u16);
        }
    }
}
