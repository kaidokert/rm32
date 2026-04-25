//! Zero-cost newtype wrappers for physical units.
//!
//! Prevents mixing up ADC counts with millivolts, or timer ticks with microseconds.
//! All types are `Copy` and transparent — zero runtime overhead.

/// Battery voltage in millivolts.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct MilliVolts(pub u16);

impl MilliVolts {
    pub const ZERO: Self = Self(0);

    /// Convert to centivolts for KISS telemetry.
    pub fn to_centivolts(self) -> u16 { self.0 / 10 }

    /// Per-cell check: voltage < cell_count * per_cell_mv
    pub fn below_cell_threshold(self, cell_count: u8, per_cell_mv: u16) -> bool {
        let threshold = cell_count as u16 * per_cell_mv;
        threshold > 0 && self.0 < threshold
    }
}

/// Motor current in milliamps.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct MilliAmps(pub i16);

impl MilliAmps {
    pub const ZERO: Self = Self(0);

    /// Convert to centiamps for KISS telemetry.
    pub fn to_centiamps(self) -> u16 { (self.0 / 10) as u16 }
}

/// Temperature in degrees Celsius.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct DegreesCelsius(pub i16);

impl DegreesCelsius {
    /// As i8 for KISS telemetry packet.
    pub fn to_i8(self) -> i8 { self.0 as i8 }
}

/// Timer ticks (0.5µs resolution at 2MHz timer clock).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct TimerTicks(pub u32);

impl TimerTicks {
    pub const ZERO: Self = Self(0);

    /// Convert to eRPM (in units of 100 eRPM).
    /// Formula: 600000 / ticks (when ticks = e_com_time)
    pub fn to_erpm_100(self) -> u16 {
        if self.0 > 0 { (600000 / self.0) as u16 } else { 0 }
    }
}

/// Raw ADC count (12-bit, 0-4095).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[repr(transparent)]
pub struct AdcCount(pub u16);

impl AdcCount {
    /// Convert to millivolts given a voltage divider ratio.
    /// Formula: count * 3300 / 4095 * divider / 100
    pub fn to_millivolts(self, voltage_divider: u16) -> MilliVolts {
        MilliVolts((self.0 as u32 * 3300 / 4095 * voltage_divider as u32 / 100) as u16)
    }

    /// Convert to milliamps given offset and sensitivity.
    /// Formula: (count * 3300/41 - offset*100) / mv_per_amp
    pub fn to_milliamps(self, current_offset: i16, mv_per_amp: u16) -> MilliAmps {
        let mv = (self.0 as i32) * 3300 / 41 - (current_offset as i32) * 100;
        if mv_per_amp > 0 {
            MilliAmps((mv / mv_per_amp as i32) as i16)
        } else {
            MilliAmps::ZERO
        }
    }
}

/// Pure temperature calculation from raw ADC and calibration values (testable on host).
pub fn calc_temperature_pure(
    raw: u16, ts_cal1: u16, ts_cal2: u16, cal1_temp: i32, cal2_temp: i32,
) -> DegreesCelsius {
    let c1 = ts_cal1 as i32;
    let c2 = ts_cal2 as i32;
    if c2 == c1 { return DegreesCelsius(25); }
    DegreesCelsius(((cal2_temp - cal1_temp) * (raw as i32 - c1) / (c2 - c1) + cal1_temp) as i16)
}

/// Calculate temperature from raw ADC and ROM calibration addresses (hardware-only).
pub fn calc_temperature_from_cal(
    raw: u16, cal1_addr: u32, cal2_addr: u32, cal1_temp: i32, cal2_temp: i32,
) -> DegreesCelsius {
    let ts_cal1 = unsafe { *(cal1_addr as *const u16) };
    let ts_cal2 = unsafe { *(cal2_addr as *const u16) };
    calc_temperature_pure(raw, ts_cal1, ts_cal2, cal1_temp, cal2_temp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn millivolts_centivolts() {
        assert_eq!(MilliVolts(16800).to_centivolts(), 1680);
    }

    #[test]
    fn millivolts_cell_threshold() {
        let v = MilliVolts(10000); // 10V
        assert!(v.below_cell_threshold(3, 3700)); // 3 * 3.7V = 11.1V > 10V
        assert!(!v.below_cell_threshold(2, 3700)); // 2 * 3.7V = 7.4V < 10V
    }

    #[test]
    fn adc_to_millivolts() {
        let count = AdcCount(2048); // mid-range
        let mv = count.to_millivolts(110); // 11:1 divider
        assert!(mv.0 > 1500 && mv.0 < 2000);
    }

    #[test]
    fn adc_to_milliamps() {
        let count = AdcCount(2048);
        let ma = count.to_milliamps(498, 20);
        assert!(ma.0 > 5000 && ma.0 < 6000); // ~5750mA
    }

    #[test]
    fn timer_ticks_to_erpm() {
        assert_eq!(TimerTicks(600).to_erpm_100(), 1000); // 100k eRPM
        assert_eq!(TimerTicks(0).to_erpm_100(), 0);
    }

    // --- Temperature calculation tests (pure math, no hardware) ---

    #[test]
    fn temp_at_cal1_returns_cal1_temp() {
        // raw == ts_cal1 → should return cal1_temp exactly
        assert_eq!(calc_temperature_pure(1000, 1000, 1500, 30, 130), DegreesCelsius(30));
    }

    #[test]
    fn temp_at_cal2_returns_cal2_temp() {
        assert_eq!(calc_temperature_pure(1500, 1000, 1500, 30, 130), DegreesCelsius(130));
    }

    #[test]
    fn temp_midpoint() {
        // Midpoint between cal1 and cal2
        let mid = calc_temperature_pure(1250, 1000, 1500, 30, 130);
        assert_eq!(mid, DegreesCelsius(80)); // (30+130)/2 = 80
    }

    #[test]
    fn temp_equal_cals_returns_25() {
        assert_eq!(calc_temperature_pure(1234, 1000, 1000, 30, 130), DegreesCelsius(25));
    }

    #[test]
    fn temp_f051_range() {
        // F051: cal1=30C, cal2=110C. Typical cal values ~700, ~900
        let t = calc_temperature_pure(800, 700, 900, 30, 110);
        assert_eq!(t, DegreesCelsius(70)); // 50% of range = (30+110)/2 = 70
    }

    #[test]
    fn adc_zero_offset_current() {
        // Zero offset, 20 mv/A: mid-range ADC = ~4024 mA
        let ma = AdcCount(2048).to_milliamps(0, 20);
        assert!(ma.0 > 8000, "expected high mA with zero offset, got {}", ma.0);
    }

    #[test]
    fn adc_zero_mvperamp_returns_zero() {
        assert_eq!(AdcCount(2048).to_milliamps(498, 0), MilliAmps::ZERO);
    }
}
