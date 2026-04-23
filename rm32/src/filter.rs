//! Multiply-free EWMA filter for smoothing ADC readings.
//!
//! Uses power-of-2 shift for division: `y += (x - y) >> K`
//! where α = 1/2^K. K=3 gives α=1/8 (matches C's `(7*y + x) >> 3`).

/// Power-of-2 EWMA filter state.
/// K is the shift amount: α = 1/2^K.
pub struct EwmaPow2<const K: u8> {
    state: u32,
    initialized: bool,
}

impl<const K: u8> EwmaPow2<K> {
    pub const fn new() -> Self {
        Self { state: 0, initialized: false }
    }

    /// Feed a new sample, return the filtered value.
    #[inline]
    pub fn update(&mut self, sample: u16) -> u16 {
        if !self.initialized {
            self.state = sample as u32;
            self.initialized = true;
            return sample;
        }
        // y += (x - y) >> K  (using i32 to handle negative difference)
        let x = sample as i32;
        let y = self.state as i32;
        let next = y + ((x - y) >> K);
        self.state = next as u32;
        next as u16
    }

    /// Current filtered value without feeding a new sample.
    pub fn value(&self) -> u16 {
        self.state as u16
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
        // Integer EWMA truncation: steady-state error up to 2^K - 1
        let v = f.value();
        assert!(v >= 793 && v <= 800, "expected ~800, got {}", v);
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
