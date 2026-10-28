//! Current smoothing and consumed-charge accounting.

use crate::units::CentiAmps;

/// A main-loop-owned integral of current over control ticks. Fractional mAh
/// remain in the accumulator; only telemetry truncates to whole mAh.
#[derive(Clone, Default)]
pub(crate) struct ConsumedCharge {
    centiamp_ticks: u64,
}

impl ConsumedCharge {
    // 1 mAh = 3600 mA*s = 360 cA*s.
    const TICKS_PER_MAH: u64 = 360 * crate::constants::CONTROL_TICK_HZ as u64;
    const MAX: u64 = u16::MAX as u64 * Self::TICKS_PER_MAH;

    /// Integrate the last observed current over the entire elapsed interval.
    /// Negative readings do not subtract consumed charge. Once the KISS u16
    /// range is full, latch at 65535 mAh until the state is reinitialized.
    pub(crate) fn accumulate(&mut self, current: CentiAmps, elapsed_ticks: u32) {
        if elapsed_ticks == 0 || current.0 <= 0 || self.centiamp_ticks == Self::MAX {
            return;
        }
        // u32 ticks * positive i16 centiamps plus MAX fits comfortably in u64.
        // No division in this per-service path; the state is main-only.
        let increment = current.0 as u64 * elapsed_ticks as u64;
        self.centiamp_ticks = (self.centiamp_ticks + increment).min(Self::MAX);
    }

    pub(crate) fn milliamp_hours(&self) -> u16 {
        (self.centiamp_ticks / Self::TICKS_PER_MAH) as u16
    }
}

const NUM_READINGS: usize = 50;

#[derive(Clone)]
pub(crate) struct CurrentFilter {
    readings: [u16; NUM_READINGS],
    index: usize,
    total: u32,
}

impl CurrentFilter {
    pub(crate) const fn new() -> Self {
        Self {
            readings: [0; NUM_READINGS],
            index: 0,
            total: 0,
        }
    }

    /// Feed a new raw ADC current reading, returns smoothed value.
    pub(crate) fn update(&mut self, raw: u16) -> u16 {
        self.total -= self.readings[self.index] as u32;
        self.readings[self.index] = raw;
        self.total += raw as u32;
        self.index += 1;
        if self.index >= NUM_READINGS {
            self.index = 0;
        }
        (self.total / NUM_READINGS as u32) as u16
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
    fn charge_physical_values_and_fractional_ticks() {
        let mut charge = ConsumedCharge::default();
        // 1 A for 3.6 s = 1 mAh. One control tick is 50 us.
        charge.accumulate(CentiAmps(100), 71999);
        assert_eq!(charge.milliamp_hours(), 0);
        charge.accumulate(CentiAmps(100), 1);
        assert_eq!(charge.milliamp_hours(), 1);
        // 36 A for 1 s = 10 mAh; a delayed service must keep the whole second.
        charge.accumulate(CentiAmps(3600), 20000);
        assert_eq!(charge.milliamp_hours(), 11);
        // Even 10 mA increments are retained: 360 seconds = 1 mAh.
        charge.accumulate(CentiAmps(1), 7_200_000);
        assert_eq!(charge.milliamp_hours(), 12);
    }

    #[test]
    fn charge_partitioning_does_not_discard_fractional_consumption() {
        let mut single = ConsumedCharge::default();
        let mut split = ConsumedCharge::default();
        single.accumulate(CentiAmps(123), 72000);
        for _ in 0..72000 {
            split.accumulate(CentiAmps(123), 1);
        }
        assert_eq!(single.milliamp_hours(), 1);
        assert_eq!(split.centiamp_ticks, single.centiamp_ticks);
        // Add 0.77 mAh: the retained 0.23 mAh must complete another whole mAh.
        split.accumulate(CentiAmps(77), 72000);
        assert_eq!(split.milliamp_hours(), 2);
    }

    #[test]
    fn charge_zero_negative_and_saturation_boundaries() {
        let mut charge = ConsumedCharge::default();
        charge.accumulate(CentiAmps(100), 72000);
        for current in [0, -1, i16::MIN] {
            charge.accumulate(CentiAmps(current), u32::MAX);
        }
        charge.accumulate(CentiAmps(i16::MAX), 0);
        assert_eq!(charge.milliamp_hours(), 1);

        // One centiamp-tick below the largest wire value.
        charge.centiamp_ticks = 65535 * 7_200_000 - 1;
        assert_eq!(charge.milliamp_hours(), 65534);
        charge.accumulate(CentiAmps(1), 1);
        assert_eq!(charge.milliamp_hours(), 65535);
        charge.accumulate(CentiAmps(i16::MAX), u32::MAX);
        charge.accumulate(CentiAmps(-100), u32::MAX);
        assert_eq!(charge.milliamp_hours(), 65535);

        let mut large_gap = ConsumedCharge::default();
        large_gap.accumulate(CentiAmps(i16::MAX), u32::MAX);
        assert_eq!(large_gap.milliamp_hours(), 65535);
    }

    #[test]
    fn charge_matches_independent_milliamp_microsecond_reference() {
        let mut charge = ConsumedCharge::default();
        let mut milliamp_us = 0u128;
        for i in 0..2000u32 {
            let current = ((i * 97) % 12000) as i16 - 500;
            let ticks = (i * 397) % 30001;
            charge.accumulate(CentiAmps(current), ticks);
            milliamp_us += current.max(0) as u128 * 10 * ticks as u128 * 50;
            // 3600 seconds per hour, 1,000,000 microseconds per second.
            let mah = (milliamp_us / 3_600_000_000).min(65535);
            assert_eq!(charge.milliamp_hours() as u128, mah);
        }
    }

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
