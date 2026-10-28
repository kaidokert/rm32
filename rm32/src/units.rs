//! Zero-cost newtype wrappers for physical units.
//!
//! Prevents mixing up ADC counts with centivolts, or timer ticks with microseconds.
//! All types are `Copy` and transparent — zero runtime overhead.

/// Battery voltage in centivolts (0.01 V), also the KISS wire unit.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct CentiVolts(pub u16);

impl CentiVolts {
    pub const ZERO: Self = Self(0);

    pub fn to_centivolts(self) -> u16 {
        self.0
    }

    /// Strict per-cell cutoff; a zero threshold disables the check.
    pub fn below_cell_threshold(self, cell_count: u8, per_cell_cv: u16) -> bool {
        let threshold = cell_count as u32 * per_cell_cv as u32;
        threshold > 0 && (self.0 as u32) < threshold
    }
}

/// Motor current in centiamps (0.01 A). Signed to retain sensor offset error.
/// The representable range is -327.68 A through 327.67 A.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct CentiAmps(pub i16);

impl CentiAmps {
    pub const ZERO: Self = Self(0);

    /// KISS has an unsigned current field: negative readings encode as zero.
    pub fn to_centiamps(self) -> u16 {
        self.0.max(0) as u16
    }
}

/// Temperature in degrees Celsius.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[repr(transparent)]
pub struct DegreesCelsius(pub i16);

impl DegreesCelsius {
    /// As i8 for KISS telemetry packet.
    pub fn to_i8(self) -> i8 {
        self.0 as i8
    }
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
        (600000u32.checked_div(self.0).unwrap_or(0)) as u16
    }
}

/// Raw ADC count (12-bit, 0-4095).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[repr(transparent)]
pub struct AdcCount(pub u16);

impl AdcCount {
    /// Convert a 12-bit, 3.3 V ADC reading to battery centivolts.
    /// `voltage_divider_x100` is the battery/pin voltage ratio times 100:
    /// 1100 means 11:1. Truncate once, after scaling the pin voltage.
    /// Out-of-range counts clamp to full scale; output saturates at 655.35 V.
    pub fn to_centivolts(self, voltage_divider_x100: u16) -> CentiVolts {
        // raw * 3300 * ratio / (4095 * 100 * 10), reduced to fit u32
        // even at raw=4095 and ratio=u16::MAX (product < 2^32).
        let cv = self.0.min(4095) as u32 * voltage_divider_x100 as u32 * 11 / 13650;
        CentiVolts(cv.min(u16::MAX as u32) as u16)
    }

    /// Convert a 12-bit, 3.3 V ADC reading to centiamps.
    /// Offset is in millivolts; sensitivity is millivolts per amp.
    /// A zero sensitivity disables the sensor. Truncate toward zero once,
    /// then saturate to the signed current range instead of wrapping.
    pub fn to_centiamps(self, current_offset_mv: i16, mv_per_amp: u16) -> CentiAmps {
        if mv_per_amp == 0 {
            return CentiAmps::ZERO;
        }
        // 3300/4095 = 220/273. Keeping this rational until the final
        // division avoids losing resolution before subtracting the offset.
        // All products fit i32 for every i16 offset and u16 sensitivity.
        let numerator = (self.0.min(4095) as i32 * 220 - current_offset_mv as i32 * 273) * 100;
        let ca = numerator / (273 * mv_per_amp as i32);
        CentiAmps(ca.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
}

/// Pure temperature calculation from raw ADC and calibration values (testable on host).
pub fn calc_temperature_pure(
    raw: u16,
    ts_cal1: u16,
    ts_cal2: u16,
    cal1_temp: i32,
    cal2_temp: i32,
) -> DegreesCelsius {
    let c1 = ts_cal1 as i32;
    let c2 = ts_cal2 as i32;
    if c2 == c1 {
        return DegreesCelsius(25);
    }
    DegreesCelsius(((cal2_temp - cal1_temp) * (raw as i32 - c1) / (c2 - c1) + cal1_temp) as i16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn measurement_regression_negative_current_is_zero_on_unsigned_wire() {
        assert_eq!(CentiAmps(-100).to_centiamps(), 0);
    }

    #[test]
    fn centivolts_wire_units() {
        assert_eq!(CentiVolts(1680).to_centivolts(), 1680);
    }

    #[test]
    fn centivolts_cell_threshold() {
        let v = CentiVolts(1000); // 10V
        assert!(v.below_cell_threshold(3, 370)); // 3 * 3.7V = 11.1V > 10V
        assert!(!v.below_cell_threshold(2, 370)); // 2 * 3.7V = 7.4V < 10V
    }

    #[test]
    fn adc_to_centivolts() {
        let count = AdcCount(2048); // mid-range
        let cv = count.to_centivolts(1100); // 11:1 divider
        assert_eq!(cv.0, 1815); // 1.6504 V at the pin -> 18.15 V
    }

    #[test]
    fn adc_to_centiamps() {
        let count = AdcCount(2048);
        let ca = count.to_centiamps(498, 20);
        assert_eq!(ca.0, 5762); // (1.6504 V - 0.498 V) / 0.020 V/A
    }

    #[test]
    fn adc_physical_voltage_values() {
        // One third full scale is exactly 1.1 V at the ADC pin.
        assert_eq!(AdcCount(1365).to_centivolts(100), CentiVolts(110));
        assert_eq!(AdcCount(1365).to_centivolts(1100), CentiVolts(1210));
        assert_eq!(AdcCount(1365).to_centivolts(2100), CentiVolts(2310));
        // Fractional divider: 12.34:1 gives 13.574 V, truncated to 13.57 V.
        assert_eq!(AdcCount(1365).to_centivolts(1234), CentiVolts(1357));
        assert_eq!(AdcCount(4095).to_centivolts(2100), CentiVolts(6930));
        assert_eq!(AdcCount(0).to_centivolts(1100), CentiVolts::ZERO);
    }

    #[test]
    fn adc_voltage_matches_wide_physical_reference() {
        for divider in [0, 1, 100, 1100, 1234, 2100, u16::MAX] {
            for raw in 0..=4095 {
                // Independent unreduced formula in microvolts, using u64.
                let battery_uv = raw as u64 * 3_300_000 * divider as u64;
                let expected_cv = (battery_uv / (4095 * 100 * 10_000)).min(65535);
                assert_eq!(AdcCount(raw).to_centivolts(divider).0 as u64, expected_cv);
            }
        }
        assert_eq!(AdcCount(u16::MAX).to_centivolts(1100), CentiVolts(3630));
        assert_eq!(AdcCount(4095).to_centivolts(u16::MAX), CentiVolts(u16::MAX));
    }

    #[test]
    fn adc_physical_current_values() {
        // 1.1 V pin, 500 mV zero offset, 20 mV/A: exactly 30 A.
        assert_eq!(AdcCount(1365).to_centiamps(500, 20), CentiAmps(3000));
        assert_eq!(AdcCount(1365).to_centiamps(500, 30), CentiAmps(2000));
        assert_eq!(AdcCount(1365).to_centiamps(1100, 20), CentiAmps::ZERO);
        assert_eq!(AdcCount(1365).to_centiamps(1200, 20), CentiAmps(-500));
        assert_eq!(AdcCount(0).to_centiamps(-100, 20), CentiAmps(500));
        assert_eq!(AdcCount(4095).to_centiamps(0, 20), CentiAmps(16500));
        assert_eq!(AdcCount(4095).to_centiamps(0, 8), CentiAmps(i16::MAX));
        assert_eq!(AdcCount(0).to_centiamps(i16::MAX, 1), CentiAmps(i16::MIN));
    }

    #[test]
    fn adc_current_matches_wide_physical_reference() {
        for offset in [i16::MIN, -498, 0, 498, 1100, i16::MAX] {
            for sensitivity in [1, 8, 16, 20, 25, 30, u16::MAX] {
                for raw in 0..=4095 {
                    let pin_uv = raw as i64 * 3_300_000;
                    let offset_uv = offset as i64 * 1000 * 4095;
                    let ca = (pin_uv - offset_uv) / (4095 * 10 * sensitivity as i64);
                    let expected = ca.clamp(i16::MIN as i64, i16::MAX as i64);
                    assert_eq!(
                        AdcCount(raw).to_centiamps(offset, sensitivity).0 as i64,
                        expected
                    );
                }
            }
        }
        assert_eq!(AdcCount(u16::MAX).to_centiamps(0, 20), CentiAmps(16500));
    }

    #[test]
    fn cell_threshold_boundaries_and_wide_products() {
        assert!(CentiVolts(989).below_cell_threshold(3, 330));
        assert!(!CentiVolts(990).below_cell_threshold(3, 330));
        assert!(!CentiVolts(991).below_cell_threshold(3, 330));
        assert!(!CentiVolts(0).below_cell_threshold(0, 330));
        assert!(!CentiVolts(0).below_cell_threshold(3, 0));
        assert!(CentiVolts(u16::MAX).below_cell_threshold(u8::MAX, u16::MAX));
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
        assert_eq!(
            calc_temperature_pure(1000, 1000, 1500, 30, 130),
            DegreesCelsius(30)
        );
    }

    #[test]
    fn temp_at_cal2_returns_cal2_temp() {
        assert_eq!(
            calc_temperature_pure(1500, 1000, 1500, 30, 130),
            DegreesCelsius(130)
        );
    }

    #[test]
    fn temp_midpoint() {
        // Midpoint between cal1 and cal2
        let mid = calc_temperature_pure(1250, 1000, 1500, 30, 130);
        assert_eq!(mid, DegreesCelsius(80)); // (30+130)/2 = 80
    }

    #[test]
    fn temp_equal_cals_returns_25() {
        assert_eq!(
            calc_temperature_pure(1234, 1000, 1000, 30, 130),
            DegreesCelsius(25)
        );
    }

    #[test]
    fn temp_f051_range() {
        // F051: cal1=30C, cal2=110C. Typical cal values ~700, ~900
        let t = calc_temperature_pure(800, 700, 900, 30, 110);
        assert_eq!(t, DegreesCelsius(70)); // 50% of range = (30+110)/2 = 70
    }

    #[test]
    fn adc_zero_offset_current() {
        // Zero offset, 20 mV/A: mid-range ADC = ~82.52 A
        let ca = AdcCount(2048).to_centiamps(0, 20);
        assert!(
            ca.0 > 8000,
            "expected high centiamps with zero offset, got {}",
            ca.0
        );
    }

    #[test]
    fn adc_zero_mvperamp_returns_zero() {
        assert_eq!(AdcCount(2048).to_centiamps(498, 0), CentiAmps::ZERO);
    }
}
