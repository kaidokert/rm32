//! Current smoothing (moving average filter).

const NUM_READINGS: usize = 50;

pub struct CurrentFilter {
    readings: [u16; NUM_READINGS],
    index: usize,
    total: u32,
}

impl CurrentFilter {
    pub const fn new() -> Self {
        Self {
            readings: [0; NUM_READINGS],
            index: 0,
            total: 0,
        }
    }

    /// Feed a new raw ADC current reading, returns smoothed value.
    pub fn update(&mut self, raw: u16) -> u16 {
        self.total -= self.readings[self.index] as u32;
        self.readings[self.index] = raw;
        self.total += raw as u32;
        self.index += 1;
        if self.index >= NUM_READINGS {
            self.index = 0;
        }
        (self.total / NUM_READINGS as u32) as u16
    }

    pub fn reset(&mut self) {
        self.readings = [0; NUM_READINGS];
        self.index = 0;
        self.total = 0;
    }
}

impl Default for CurrentFilter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returns_running_average() {
        let mut f = CurrentFilter::new();
        for _ in 0..50 {
            f.update(1000);
        }
        assert_eq!(f.update(1000), 1000);
    }

    #[test]
    fn wraps_index() {
        let mut f = CurrentFilter::new();
        for _ in 0..60 {
            f.update(500);
        }
        assert_eq!(f.index, 10); // 60 % 50
    }

    #[test]
    fn starts_at_zero() {
        let f = CurrentFilter::new();
        assert_eq!(f.total, 0);
        assert_eq!(f.index, 0);
    }

    #[test]
    fn reset_clears() {
        let mut f = CurrentFilter::new();
        for _ in 0..30 {
            f.update(500);
        }
        f.reset();
        assert_eq!(f.total, 0);
        assert_eq!(f.index, 0);
        assert_eq!(f.update(0), 0);
    }

    #[test]
    fn averages_mixed_values() {
        let mut f = CurrentFilter::new();
        // Fill half with 0, half with 1000
        for _ in 0..25 {
            f.update(0);
        }
        for _ in 0..25 {
            f.update(1000);
        }
        assert_eq!(f.update(1000), 520); // (25*0 + 26*1000) / 50 = 520
    }
}
