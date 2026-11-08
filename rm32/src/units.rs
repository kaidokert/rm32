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

use crate::platform::Platform;

/// Raw ADC count (nominally 12-bit, 0-4095, or wider `u16` encoded readings).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[repr(transparent)]
pub struct AdcCount(pub u16);

/// Largest reading the `Adc` trait can return. Conversions are exact across the
/// full `u16` range, not only a 12-bit ADC range.
const ADC_MAX: u32 = u16::MAX as u32;

/// Greatest common divisor of `a` and `b` via Euclid's algorithm.
const fn gcd_u64(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let r = a % b;
        a = b;
        b = r;
    }
    a
}

/// `floor((a * x + b) / d)` for every `x` in `0..=x_max` with a multiply,
/// add and shift and no divide (cores without hardware divide would
/// otherwise call `__aeabi_*div`). [`AffineDiv::new`] runs at compile time,
/// checks every `x` against the exact quotient and fails the build if no
/// exact form exists.
///
/// Three forms, first exact one wins (`a = A*d + a1`, `a2 = d - a1`):
/// direct `floor((a x + b) / d)`; `A x + floor((a1 x + b) / d)`; and
/// `(A + 1) x - ceil((a2 x - b) / d)`. Peeling off the integer part keeps the
/// multiplier small enough for the precision a large `d` needs. A 32-bit
/// multiply is preferred; only when none of the three has an exact 32-bit
/// form over the whole domain does it take a 64-bit multiply (on M0 a
/// constant-time `__aeabi_lmul`, still no divide).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct AffineDiv {
    lin: i32,
    core: MulShift,
    negate: bool,
    /// The domain [`AffineDiv::new`] proved exact; `apply` beyond it wraps.
    x_max: u32,
}

/// `((x * mul + add) >> shift) - sub` = `floor((alpha x + beta) / d)`, in
/// 32-bit arithmetic, or 64-bit when `wide`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MulShift {
    mul: u64,
    add: u64,
    shift: u32,
    sub: i32,
    wide: bool,
}

impl MulShift {
    const ZERO: Self = Self {
        mul: 0,
        add: 0,
        shift: 0,
        sub: 0,
        wide: false,
    };

    /// Exact form of `floor((alpha x + beta) / d)` for `x <= x_max`, if one
    /// exists with `x * mul + add` inside u32 (u64 when `wide`).
    const fn find(alpha: i32, beta: i32, d: i32, x_max: u32, wide: bool) -> Option<Self> {
        // beta = d * t + r with 0 <= r < d: floor((alpha x + r) / d) + t.
        let t = beta.div_euclid(d);
        let r = beta.rem_euclid(d);
        let limit = if wide {
            u64::MAX as u128
        } else {
            u32::MAX as u128
        };
        let mut shift = if wide { 63 } else { 31 };
        loop {
            let mul_lo = ((alpha as u128) << shift) / d as u128;
            let add_lo = ((r as u128) << shift) / d as u128;
            let mut k = 0;
            while k < 4 {
                let mul = mul_lo + (k & 1);
                let add = add_lo + (k >> 1);
                if x_max as u128 * mul + add <= limit {
                    let c = Self {
                        mul: mul as u64,
                        add: add as u64,
                        shift,
                        sub: -t,
                        wide,
                    };
                    if c.exact(alpha, r, d, x_max) {
                        return Some(c);
                    }
                }
                k += 1;
            }
            if shift == 0 {
                return None;
            }
            shift -= 1;
        }
    }

    /// Verify that `self.apply(x) + self.sub` equals `floor((alpha * x + r) / d)`
    /// for every `x` in `0..=x_max`.
    const fn exact(self, alpha: i32, r: i32, d: i32, x_max: u32) -> bool {
        let mut x = 0;
        while x <= x_max {
            let want = ((alpha as i64 * x as i64 + r as i64) / d as i64) as i32;
            if self.apply(x) + self.sub != want {
                return false;
            }
            x += 1;
        }
        true
    }

    /// Evaluate `((x * mul + add) >> shift) - sub`.
    #[inline(always)]
    const fn apply(self, x: u32) -> i32 {
        let q = if self.wide {
            ((x as u64 * self.mul + self.add) >> self.shift) as u32
        } else {
            (x * self.mul as u32 + self.add as u32) >> self.shift
        };
        q as i32 - self.sub
    }
}

impl AffineDiv {
    /// Always 0 (a board without the sensor).
    pub const ZERO: Self = Self {
        lin: 0,
        core: MulShift::ZERO,
        negate: false,
        x_max: u32::MAX,
    };

    /// Exact divide-free form of `floor((a * x + b) / d)`, `x <= x_max`.
    pub const fn new(a: i32, b: i32, d: i32, x_max: u32) -> Self {
        assert!(a >= 0 && d > 0);
        let a1 = a % d;
        let mut wide = false;
        loop {
            if let Some(core) = MulShift::find(a, b, d, x_max, wide) {
                return Self {
                    lin: 0,
                    core,
                    negate: false,
                    x_max,
                };
            }
            if let Some(core) = MulShift::find(a1, b, d, x_max, wide) {
                return Self {
                    lin: a / d,
                    core,
                    negate: false,
                    x_max,
                };
            }
            // ceil((a2 x - b) / d) = floor((a2 x - b + d - 1) / d)
            if let Some(core) = MulShift::find(d - a1, d - 1 - b, d, x_max, wide) {
                return Self {
                    lin: a / d + 1,
                    core,
                    negate: true,
                    x_max,
                };
            }
            if wide {
                panic!("no exact multiply-shift form for this board constant");
            }
            wide = true;
        }
    }

    /// The quotient for `x <= x_max` (debug builds assert it; beyond it
    /// the 32-bit multiply wraps).
    #[inline(always)]
    pub const fn apply(self, x: u32) -> i32 {
        debug_assert!(x <= self.x_max);
        let c = self.core.apply(x);
        self.lin * x as i32 + if self.negate { -c } else { c }
    }
}

/// Compile-time ADC conversions of a platform's board constants.
/// Blanket-implemented; read through [`AdcCount::centivolts`] and
/// [`AdcCount::centiamps`].
pub(crate) trait AdcScale: Platform + Sized {
    /// Largest raw ADC count before the battery voltage conversion reaches
    /// `u16::MAX` centivolts (655.35 V).
    const VOLTAGE_RAW_MAX: u32 = {
        let num0 = Self::ADC_VREF_MV as u64 * Self::VOLTAGE_DIVIDER_X100 as u64;
        let den0 = Self::ADC_FULL_SCALE as u64 * 1000;
        if num0 == 0 {
            ADC_MAX
        } else {
            let limit = (u16::MAX as u64 * den0).div_ceil(num0);
            if limit < ADC_MAX as u64 {
                limit as u32
            } else {
                ADC_MAX
            }
        }
    };

    /// Whether any `u16` ADC reading can reach or exceed `u16::MAX` centivolts.
    const VOLTAGE_SATURATES: bool = {
        let num0 = Self::ADC_VREF_MV as u64 * Self::VOLTAGE_DIVIDER_X100 as u64;
        let den0 = Self::ADC_FULL_SCALE as u64 * 1000;
        (ADC_MAX as u64 * num0) / den0 >= u16::MAX as u64
    };

    /// Raw ADC count -> battery centivolts:
    /// `floor(raw * ADC_VREF_MV * VOLTAGE_DIVIDER_X100 / (ADC_FULL_SCALE * 1000))`.
    const VOLTAGE_DIV: AffineDiv = {
        let num0 = Self::ADC_VREF_MV as u64 * Self::VOLTAGE_DIVIDER_X100 as u64;
        let den0 = Self::ADC_FULL_SCALE as u64 * 1000;
        if num0 == 0 {
            AffineDiv::ZERO
        } else {
            let g = gcd_u64(num0, den0);
            AffineDiv::new(
                (num0 / g) as i32,
                0,
                (den0 / g) as i32,
                Self::VOLTAGE_RAW_MAX,
            )
        }
    };

    /// First raw count with a non-negative current numerator
    /// `raw * ADC_VREF_MV - CURRENT_OFFSET * ADC_FULL_SCALE >= 0`.
    const CURRENT_NEG_BELOW: u32 = if Self::MILLIVOLT_PER_AMP == 0 || Self::CURRENT_OFFSET <= 0 {
        0
    } else {
        let num = Self::CURRENT_OFFSET as u64 * Self::ADC_FULL_SCALE as u64;
        let den = Self::ADC_VREF_MV as u64;
        let threshold = num.div_ceil(den);
        if threshold <= ADC_MAX as u64 {
            threshold as u32
        } else {
            ADC_MAX + 1
        }
    };

    /// Non-negative current branch (`raw >= CURRENT_NEG_BELOW`):
    /// `floor((raw * ADC_VREF_MV - CURRENT_OFFSET * ADC_FULL_SCALE) * 100 / (ADC_FULL_SCALE * MILLIVOLT_PER_AMP))`.
    const CURRENT_POS: AffineDiv = if Self::MILLIVOLT_PER_AMP == 0 {
        AffineDiv::ZERO
    } else {
        let a0 = Self::ADC_VREF_MV as u64 * 100;
        let b0_abs =
            (Self::CURRENT_OFFSET as i64).unsigned_abs() * Self::ADC_FULL_SCALE as u64 * 100;
        let d0 = Self::ADC_FULL_SCALE as u64 * Self::MILLIVOLT_PER_AMP as u64;
        let g = gcd_u64(gcd_u64(a0, b0_abs), d0);
        let a = (a0 / g) as i32;
        let b_mag = (b0_abs / g) as i32;
        let b = if Self::CURRENT_OFFSET >= 0 {
            -b_mag
        } else {
            b_mag
        };
        let d = (d0 / g) as i32;
        AffineDiv::new(a, b, d, ADC_MAX)
    };

    /// Negative current branch (`raw < CURRENT_NEG_BELOW`), truncated toward zero:
    /// `ceil(n / d) = floor((n + d - 1) / d)`.
    const CURRENT_NEG: AffineDiv = if Self::MILLIVOLT_PER_AMP == 0 || Self::CURRENT_OFFSET <= 0 {
        AffineDiv::ZERO
    } else {
        let a0 = Self::ADC_VREF_MV as u64 * 100;
        let b0_abs =
            (Self::CURRENT_OFFSET as i64).unsigned_abs() * Self::ADC_FULL_SCALE as u64 * 100;
        let d0 = Self::ADC_FULL_SCALE as u64 * Self::MILLIVOLT_PER_AMP as u64;
        let g = gcd_u64(gcd_u64(a0, b0_abs), d0);
        let a = (a0 / g) as i32;
        let b_mag = (b0_abs / g) as i32;
        let d = (d0 / g) as i32;
        let b = -b_mag + d - 1;
        AffineDiv::new(a, b, d, Self::CURRENT_NEG_BELOW.saturating_sub(1))
    };

    /// Whether any `u16` ADC reading can exceed the signed `i16` centiamp range.
    const CURRENT_SATURATES: bool = Self::MILLIVOLT_PER_AMP != 0
        && (Self::CURRENT_POS.apply(ADC_MAX) > i16::MAX as i32
            || (Self::CURRENT_NEG_BELOW > 0 && Self::CURRENT_NEG.apply(0) < i16::MIN as i32));
}

impl<P: Platform> AdcScale for P {}

/// Convert a raw ADC reading to battery centivolts using `P`'s compile-time
/// board constants.
#[inline]
pub fn adc_to_centivolts<P: Platform>(raw: AdcCount) -> CentiVolts {
    raw.centivolts::<P>()
}

/// Convert a raw ADC reading to motor current in centiamps using `P`'s
/// compile-time board constants.
#[inline]
pub fn adc_to_centiamps<P: Platform>(raw: AdcCount) -> CentiAmps {
    raw.centiamps::<P>()
}

impl AdcCount {
    /// Battery voltage in centivolts from `P`'s compile-time ADC reference,
    /// full-scale count, and voltage divider ratio. Truncates once after
    /// scaling the pin voltage and saturates at `u16::MAX` centivolts.
    #[inline]
    pub fn centivolts<P: Platform>(self) -> CentiVolts {
        if <P as AdcScale>::VOLTAGE_SATURATES {
            let raw = (self.0 as u32).min(<P as AdcScale>::VOLTAGE_RAW_MAX);
            CentiVolts(<P as AdcScale>::VOLTAGE_DIV.apply(raw).min(u16::MAX as i32) as u16)
        } else {
            CentiVolts(<P as AdcScale>::VOLTAGE_DIV.apply(self.0 as u32) as u16)
        }
    }

    /// Motor current in centiamps from `P`'s compile-time ADC reference,
    /// full-scale count, offset (mV), and sensitivity (mV/A). Truncates toward
    /// zero once and saturates to `i16::MIN..=i16::MAX` (0 without a sensor).
    #[inline]
    pub fn centiamps<P: Platform>(self) -> CentiAmps {
        let raw = self.0 as u32;
        let q = if raw < <P as AdcScale>::CURRENT_NEG_BELOW {
            <P as AdcScale>::CURRENT_NEG.apply(raw)
        } else {
            <P as AdcScale>::CURRENT_POS.apply(raw)
        };
        if <P as AdcScale>::CURRENT_SATURATES {
            CentiAmps(q.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
        } else {
            CentiAmps(q as i16)
        }
    }

    /// Convert a 3.3 V ADC reading (nominally 4095 full scale) to battery
    /// centivolts across the full `u16` input range.
    /// `voltage_divider_x100` is the battery/pin voltage ratio times 100:
    /// 1100 means 11:1. Truncate once, after scaling the pin voltage;
    /// output saturates at 655.35 V (`u16::MAX` centivolts).
    pub fn to_centivolts(self, voltage_divider_x100: u16) -> CentiVolts {
        // raw * 3300 * ratio / (4095 * 100 * 10) = (raw * ratio * 11) / 13650.
        // Clamping `raw * ratio` at 81_322_978 (the smallest product where
        // `prod * 11 / 13650` reaches `u16::MAX = 65535`) keeps `prod * 11`
        // within `u32` across the entire `u16 x u16` domain without 64-bit
        // division.
        let prod = (self.0 as u32 * voltage_divider_x100 as u32).min(81_322_978);
        CentiVolts((prod * 11 / 13650) as u16)
    }

    /// Convert a 3.3 V ADC reading (nominally 4095 full scale) to centiamps
    /// across the full `u16` input range.
    /// Offset is in millivolts; sensitivity is millivolts per amp.
    /// A zero sensitivity disables the sensor. Truncate toward zero once,
    /// then saturate to the signed current range instead of wrapping.
    pub fn to_centiamps(self, current_offset_mv: i16, mv_per_amp: u16) -> CentiAmps {
        if mv_per_amp == 0 {
            return CentiAmps::ZERO;
        }
        // 3300/4095 = 220/273. Keeping this rational until the final
        // division avoids losing resolution before subtracting the offset.
        // `diff` spans `-8_945_391..=23_363_364` across all `u16` counts and
        // `i16` offsets, so `diff.unsigned_abs() * 100 <= 2_336_336_400` fits
        // in `u32` without 64-bit division.
        let diff = self.0 as i32 * 220 - current_offset_mv as i32 * 273;
        let mag = ((diff.unsigned_abs() * 100) / (273 * mv_per_amp as u32)).min(i16::MAX as u32 + 1)
            as i32;
        let ca = if diff < 0 { -mag } else { mag };
        CentiAmps(ca.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
}

/// Pure temperature calculation from raw ADC and calibration values
/// (testable on host).
///
/// `vdda_mv` / `cal_vref_mv`: the factory TS_CAL points are measured
/// at a per-family reference supply (L4/G0/G4: 3.0 V; F0: 3.3 V),
/// while the board runs the ADC from VDDA (3.3 V here). The raw
/// reading must be rescaled into the calibration frame first —
/// exactly what ST's `__LL_ADC_CALC_TEMPERATURE(vdda, raw, res)`
/// does, and what AM32 calls (`main.c`: `(3300, ADC_raw_temp, 12B)`).
pub fn calc_temperature_pure(
    raw: u16,
    ts_cal1: u16,
    ts_cal2: u16,
    cal1_temp: i32,
    cal2_temp: i32,
    vdda_mv: i32,
    cal_vref_mv: i32,
) -> DegreesCelsius {
    let c1 = ts_cal1 as i32;
    let c2 = ts_cal2 as i32;
    if c2 == c1 || cal_vref_mv <= 0 || vdda_mv <= 0 {
        return DegreesCelsius(25);
    }
    let raw_scaled = raw as i32 * vdda_mv / cal_vref_mv;
    DegreesCelsius(((cal2_temp - cal1_temp) * (raw_scaled - c1) / (c2 - c1) + cal1_temp) as i16)
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
            for raw in 0..=u16::MAX {
                // Independent unreduced formula in microvolts, using u64.
                let battery_uv = raw as u64 * 3_300_000 * divider as u64;
                let expected_cv = (battery_uv / (4095 * 100 * 10_000)).min(65535);
                assert_eq!(AdcCount(raw).to_centivolts(divider).0 as u64, expected_cv);
            }
        }
        // Readings above 4095 scale monotonically instead of clamping at 4095,
        // and saturate at u16::MAX centivolts (655.35 V).
        assert_eq!(AdcCount(4095).to_centivolts(1100), CentiVolts(3630));
        assert_eq!(AdcCount(8190).to_centivolts(1100), CentiVolts(7260));
        assert_eq!(AdcCount(u16::MAX).to_centivolts(1100), CentiVolts(58093));
        assert_eq!(AdcCount(4095).to_centivolts(u16::MAX), CentiVolts(u16::MAX));
        assert_eq!(
            AdcCount(u16::MAX).to_centivolts(u16::MAX),
            CentiVolts(u16::MAX)
        );
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
                for raw in
                    (0..=4095).chain([4096, 5000, 8190, 10_000, 20_000, 32_768, 50_000, u16::MAX])
                {
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
        // Readings above 4095 continue scaling past 12-bit full scale up to
        // i16::MAX centiamps (327.67 A) without clamping at 4095.
        assert_eq!(AdcCount(4095).to_centiamps(0, 20), CentiAmps(16500));
        assert_eq!(AdcCount(5460).to_centiamps(0, 20), CentiAmps(22000));
        assert_eq!(AdcCount(8190).to_centiamps(0, 20), CentiAmps(i16::MAX));
        assert_eq!(AdcCount(u16::MAX).to_centiamps(0, 20), CentiAmps(i16::MAX));
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
            calc_temperature_pure(1000, 1000, 1500, 30, 130, 3000, 3000),
            DegreesCelsius(30)
        );
    }

    #[test]
    fn temp_at_cal2_returns_cal2_temp() {
        assert_eq!(
            calc_temperature_pure(1500, 1000, 1500, 30, 130, 3000, 3000),
            DegreesCelsius(130)
        );
    }

    #[test]
    fn temp_midpoint() {
        // Midpoint between cal1 and cal2
        let mid = calc_temperature_pure(1250, 1000, 1500, 30, 130, 3000, 3000);
        assert_eq!(mid, DegreesCelsius(80)); // (30+130)/2 = 80
    }

    #[test]
    fn temp_equal_cals_returns_25() {
        assert_eq!(
            calc_temperature_pure(1234, 1000, 1000, 30, 130, 3300, 3000),
            DegreesCelsius(25)
        );
        assert_eq!(
            calc_temperature_pure(1234, 1000, 1500, 30, 130, 3300, 0),
            DegreesCelsius(25)
        );
        assert_eq!(
            calc_temperature_pure(1234, 1000, 1500, 30, 130, 3300, -3000),
            DegreesCelsius(25)
        );
        assert_eq!(
            calc_temperature_pure(1234, 1000, 1500, 30, 130, 0, 3000),
            DegreesCelsius(25)
        );
    }

    #[test]
    fn temp_f051_range() {
        // F051: cal1=30C, cal2=110C. Typical cal values ~700, ~900
        let t = calc_temperature_pure(800, 700, 900, 30, 110, 3300, 3300);
        assert_eq!(t, DegreesCelsius(70)); // 50% of range = (30+110)/2 = 70
    }

    /// L431/G071/G431 TS_CAL points are measured at 3.0 V while VDDA is 3.3 V.
    /// With typical L431 calibration values (cal1=1034 @30 C, cal2=1381 @130 C),
    /// a ~21 C die produces raw=912 at 3.3 V. Unscaled interpolation reads ~-5 C;
    /// rescaling by VDDA/3000 matches ST's `__LL_ADC_CALC_TEMPERATURE` (22 C).
    #[test]
    fn temp_l431_vdda_rescale_matches_am32() {
        let unscaled = calc_temperature_pure(912, 1034, 1381, 30, 130, 3000, 3000);
        let scaled = calc_temperature_pure(912, 1034, 1381, 30, 130, 3300, 3000);
        assert_eq!(unscaled, DegreesCelsius(-5));
        assert_eq!(scaled, DegreesCelsius(22));
        // Exact 30 C die at 3.3 V VDDA produces 1000 counts when cal1 = 1100 at 3.0 V.
        assert_eq!(
            calc_temperature_pure(1000, 1100, 1500, 30, 130, 3300, 3000),
            DegreesCelsius(30)
        );
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

    macro_rules! board {
        ($name:ident, $div:expr, $mvpa:expr, $off:expr) => {
            board!($name, $div, $mvpa, $off, 4095, 3300);
        };
        ($name:ident, $div:expr, $mvpa:expr, $off:expr, $full:expr, $vref:expr) => {
            struct $name;
            impl Platform for $name {
                const TICK_HZ: u32 = 20_000;
                const TIMER_HZ: u32 = 2_000_000;
                const CAPTURE_HZ: u32 = 64_000_000;
                const GCR_SHIFT: u8 = 7;
                const VOLTAGE_DIVIDER_X100: u16 = $div;
                const MILLIVOLT_PER_AMP: u16 = $mvpa;
                const CURRENT_OFFSET: i16 = $off;
                const USE_NTC: bool = false;
                const ADC_FULL_SCALE: u16 = $full;
                const ADC_VREF_MV: u16 = $vref;
                const PWM_BASE_PERIOD: u16 = 1999;
            }
        };
    }

    board!(Board1194, 1194, 80, 1600);
    board!(Neutron, 933, 3, 0);
    board!(Am32Default, 1100, 20, 0);
    board!(TwelveS, 2100, 25, 0);
    board!(IflightF051, 1100, 30, 0);
    board!(Offset498, 1100, 20, 498);
    board!(NegOffset, 1100, 20, -100);
    board!(NoSensor, 1100, 0, 498);
    board!(ZeroDivider, 0, 20, 0);
    board!(MaxDivider, u16::MAX, 1, i16::MAX);
    board!(Adc10Bit3V0, 1100, 20, 0, 1023, 3000);

    #[test]
    fn mul_shift_covers_common_divisors() {
        assert_eq!(gcd_u64(3_300 * 1_100, 4_095 * 1_000), 15_000);
        assert_eq!(gcd_u64(0, 42), 42);
        for (a, b, d, x_max) in [
            (80, 0, 3, 4095),
            (242, 0, 273, 65535),
            (11, 0, 15, 65535),
            (220, -498 * 273, 273 * 20, 65535),
            (4400, -450_450, 3549, 65535),
            (933, 0, 1000, 4095),
            (1194, 0, 1000, 4095),
        ] {
            let f = AffineDiv::new(a, b, d, x_max);
            for x in 0..=x_max {
                let want = ((a as i64 * x as i64 + b as i64).div_euclid(d as i64)) as i32;
                assert_eq!(f.apply(x), want, "a={a} b={b} d={d} x={x}");
            }
        }
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic]
    fn apply_beyond_proved_domain_asserts() {
        let f = AffineDiv::new(933, 0, 1000, 4095);
        assert_eq!(f.apply(4095), 933 * 4095 / 1000);
        let _ = f.apply(4096);
    }

    #[test]
    fn affine_div_matches_division() {
        // No single u32 multiply-shift is exact for (raw * 80) / 3; the affine
        // form on raw peels off the integer quotient and is exact across 0..=4095.
        let f = AffineDiv::new(80, 0, 3, 4095);
        for raw in 0..=4095 {
            assert_eq!(f.apply(raw), (raw as i32 * 80) / 3);
        }
    }

    /// Check `AdcCount::centivolts::<P>` and `adc_to_centivolts::<P>` across `0..=u16::MAX`.
    fn verify_platform_voltage<P: Platform>() {
        for raw in 0..=u16::MAX {
            let expected = AdcCount(raw).to_centivolts(P::VOLTAGE_DIVIDER_X100);
            assert_eq!(
                AdcCount(raw).centivolts::<P>(),
                expected,
                "cV raw {raw} div {}",
                P::VOLTAGE_DIVIDER_X100
            );
            assert_eq!(super::adc_to_centivolts::<P>(AdcCount(raw)), expected);
        }
    }

    /// Check `AdcCount::centiamps::<P>` and `adc_to_centiamps::<P>` across `0..=u16::MAX`.
    fn verify_platform_current<P: Platform>() {
        for raw in 0..=u16::MAX {
            let expected = AdcCount(raw).to_centiamps(P::CURRENT_OFFSET, P::MILLIVOLT_PER_AMP);
            assert_eq!(
                AdcCount(raw).centiamps::<P>(),
                expected,
                "cA raw {raw} mVpA {} off {}",
                P::MILLIVOLT_PER_AMP,
                P::CURRENT_OFFSET
            );
            assert_eq!(super::adc_to_centiamps::<P>(AdcCount(raw)), expected);
        }
    }

    #[test]
    fn voltage_matches_reference_across_full_adc_range() {
        verify_platform_voltage::<Board1194>();
        verify_platform_voltage::<Neutron>();
        verify_platform_voltage::<Am32Default>();
        verify_platform_voltage::<TwelveS>();
        verify_platform_voltage::<IflightF051>();
        verify_platform_voltage::<Offset498>();
        verify_platform_voltage::<NoSensor>();
        verify_platform_voltage::<ZeroDivider>();
        verify_platform_voltage::<MaxDivider>();
        verify_platform_voltage::<crate::platform::TestPlatform>();
    }

    #[test]
    fn current_matches_reference_across_full_adc_range() {
        verify_platform_current::<Board1194>();
        verify_platform_current::<Neutron>();
        verify_platform_current::<Am32Default>();
        verify_platform_current::<TwelveS>();
        verify_platform_current::<IflightF051>();
        verify_platform_current::<Offset498>();
        verify_platform_current::<NegOffset>();
        verify_platform_current::<NoSensor>();
        verify_platform_current::<MaxDivider>();
        verify_platform_current::<crate::platform::TestPlatform>();
    }

    #[test]
    fn adc_scale_follows_the_platform() {
        // A 10-bit 3.0 V platform scales from its own ADC_FULL_SCALE and ADC_VREF_MV:
        // full scale (1023 counts) is 3.000 V at the pin -> 33.00 V (3300 cV) at 11:1.
        assert_eq!(AdcCount(1023).centivolts::<Adc10Bit3V0>(), CentiVolts(3300));
        assert_eq!(AdcCount(4095).centivolts::<Am32Default>(), CentiVolts(3630));
        assert_eq!(AdcCount(1023).centiamps::<Adc10Bit3V0>(), CentiAmps(15000));
    }

    #[test]
    fn ntc_linear_segment_matches_reference() {
        // Verify AffineDiv on the three linear NTC interpolation segments
        // (200..500, 500..1500, 1500..3000) and the lookup table across 0..=4095.
        let seg1 = AffineDiv::new(50, -200 * 50, 300, 499);
        for adc in 200..500u32 {
            assert_eq!(
                150 - seg1.apply(adc),
                150 - (adc as i32 - 200) * 50 / 300,
                "seg1 adc={adc}"
            );
        }
        let seg2 = AffineDiv::new(60, -500 * 60, 1000, 1499);
        for adc in 500..1500u32 {
            assert_eq!(
                100 - seg2.apply(adc),
                100 - (adc as i32 - 500) * 60 / 1000,
                "seg2 adc={adc}"
            );
        }
        let seg3 = AffineDiv::new(40, -1500 * 40, 1500, 2999);
        for adc in 1500..3000u32 {
            assert_eq!(
                40 - seg3.apply(adc),
                40 - (adc as i32 - 1500) * 40 / 1500,
                "seg3 adc={adc}"
            );
        }
        for raw in 0..=4095u16 {
            let adc = ((raw >> 4) as i32) * 16;
            let expected = if adc < 200 {
                150
            } else if adc < 500 {
                150 - (adc - 200) * 50 / 300
            } else if adc < 1500 {
                100 - (adc - 500) * 60 / 1000
            } else if adc < 3000 {
                40 - (adc - 1500) * 40 / 1500
            } else {
                0
            } as i16;
            assert_eq!(crate::ntc::ntc_degrees(raw), DegreesCelsius(expected));
        }
    }
}

/// The conversions against a literal transcription of AM32's C
/// (`Src/main.c:2558-2560`), every 12-bit count, on representative board
/// configurations and AM32's default constants. Neither form is AM32's
/// exactly: current here is the exact 3300 * 100 / 4095 per count, AM32's
/// `raw * 3300 / 41` reads up to 0.12 % lower; voltage truncates once, AM32
/// truncates the pin mV first. These bound the gap so a change in either
/// shows up.
#[cfg(test)]
mod am32_parity_tests {
    use super::*;
    use crate::platform::Platform;

    macro_rules! board {
        ($name:ident, $div:expr, $mvpa:expr, $off:expr) => {
            struct $name;
            impl Platform for $name {
                const TICK_HZ: u32 = 20_000;
                const TIMER_HZ: u32 = 2_000_000;
                const CAPTURE_HZ: u32 = 64_000_000;
                const GCR_SHIFT: u8 = 7;
                const VOLTAGE_DIVIDER_X100: u16 = $div;
                const MILLIVOLT_PER_AMP: u16 = $mvpa;
                const CURRENT_OFFSET: i16 = $off;
                const USE_NTC: bool = false;
                const ADC_FULL_SCALE: u16 = 4095;
                const ADC_VREF_MV: u16 = 3300;
                const PWM_BASE_PERIOD: u16 = 1999;
            }
        };
    }
    board!(Neutron, 933, 3, 0);
    board!(Board1194, 1194, 80, 1600);
    board!(Am32Default, 1100, 20, 0);

    /// AM32: `((raw * 3300 / 41) - (CURRENT_OFFSET * 100)) / MILLIVOLT_PER_AMP`,
    /// C truncation, negative clamped to 0.
    fn am32_current(raw: u16, off: i16, mvpa: u16) -> i32 {
        ((raw as i32 * 3300 / 41 - off as i32 * 100) / mvpa as i32).max(0)
    }

    /// AM32: `ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER / 100` (VOLTAGE_DIVIDER x10).
    fn am32_voltage(raw: u16, div_x10: u16) -> i32 {
        (raw as i32 * 3300 / 4095) * div_x10 as i32 / 100
    }

    /// Verify `AdcCount::centiamps::<P>` tracks AM32's C current formula within
    /// the bounded per-count `3300/4095` vs `3300/41` truncation difference
    /// across all 4,096 12-bit counts, including negative-offset and saturated
    /// regions.
    fn current_within<P: Platform>() {
        for raw in 0..=4095u16 {
            let am = am32_current(raw, P::CURRENT_OFFSET, P::MILLIVOLT_PER_AMP);
            let ours = AdcCount(raw).centiamps::<P>().0 as i32;
            // The per-count gap (3300 * 100 / 4095 - 3300 / 41 = 0.098) over
            // raw counts, plus one for the two truncations.
            let slack = raw as i32 / (10 * P::MILLIVOLT_PER_AMP as i32) + 1;
            if ours < 0 {
                let signed_am = (raw as i32 * 3300 / 41 - P::CURRENT_OFFSET as i32 * 100)
                    / P::MILLIVOLT_PER_AMP as i32;
                assert_eq!(
                    am, 0,
                    "raw {raw}: negative ours {ours} must clamp to 0 in AM32"
                );
                assert!(
                    ours >= signed_am && ours - signed_am <= slack,
                    "raw {raw}: ours {ours} signed AM32 {signed_am}"
                );
                continue;
            }
            if ours >= i16::MAX as i32 {
                assert_eq!(ours, i16::MAX as i32);
                assert!(
                    am + slack >= i16::MAX as i32,
                    "raw {raw}: saturated ours {ours} must be within slack {slack} of i16::MAX in AM32 ({am})"
                );
                continue;
            }
            assert!(
                ours >= am && ours - am <= slack,
                "raw {raw}: ours {ours} AM32 {am}"
            );
        }
    }

    /// Verify current conversion parity against AM32 across representative boards.
    #[test]
    fn current_tracks_am32_per_count_gap() {
        current_within::<Neutron>();
        current_within::<Board1194>();
        current_within::<Am32Default>();
    }

    /// Verify voltage conversion stays within 1 centivolt of AM32 across all 12-bit counts.
    #[test]
    fn voltage_tracks_am32_within_one_centivolt() {
        for raw in 0..=4095u16 {
            let am = am32_voltage(raw, 110);
            let ours = AdcCount(raw).centivolts::<Am32Default>().0 as i32;
            assert!(
                (0..=1).contains(&(ours - am)),
                "raw {raw}: ours {ours} AM32 {am}"
            );
        }
    }
}
