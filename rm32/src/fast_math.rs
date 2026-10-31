//! Exact constant-divisor arithmetic for motor-control paths.
//!
//! Cortex-M0 targets do not have integer divide instructions. These bounded
//! helpers preserve Rust's truncating integer-division results while replacing
//! divide operations in the control and PWM paths with shifts, multiplies, and
//! bounded corrections.

const PWM_NUMERATOR_MAX: u32 = crate::constants::DUTY_SCALE_MAX as u32 * u16::MAX as u32;
const DIV5_INPUT_MAX: u32 = u8::MAX as u32 * u16::MAX as u32;

/// Exact signed division by three, truncating toward zero over the full `i32`
/// domain.
#[inline(always)]
pub(crate) const fn div3_i32(n: i32) -> i32 {
    if n < 0 {
        -(div3_u32(n.unsigned_abs()) as i32)
    } else {
        div3_u32(n as u32) as i32
    }
}

/// Exact unsigned division by three over the full `u32` domain.
#[inline(never)]
const fn div3_u32(n: u32) -> u32 {
    let mut q = (n >> 2) + (n >> 4);
    q += q >> 4;
    q += q >> 8;
    q += q >> 16;
    let r = n - q * 3;
    q + ((11 * r) >> 5)
}

/// Exact unsigned division by five for values formed from a `u8` setting and a
/// `u16` timer value.
#[inline(never)]
const fn div5_u8_timer_product(n: u32) -> u32 {
    debug_assert!(n <= DIV5_INPUT_MAX);
    let mut q = (n >> 1) + (n >> 2);
    q += q >> 4;
    q += q >> 8;
    q += q >> 16;
    q >>= 2;
    let mut r = n - q * 5;
    if r >= 5 {
        q += 1;
        r -= 5;
    }
    if r >= 5 {
        q += 1;
        r -= 5;
    }
    if r >= 5 {
        q += 1;
        r -= 5;
    }
    debug_assert!(r < 5);
    q
}

/// Exact unsigned division by ten for a product of a `u8` setting and a `u16`
/// timer value. `floor(floor(n / 5) / 2) == floor(n / 10)`.
#[inline(never)]
pub(crate) const fn div10_u8_timer_product(n: u32) -> u32 {
    div5_u8_timer_product(n) >> 1
}

/// Exact truncating division by [`crate::constants::DUTY_SCALE_MAX`] for
/// products formed from a duty value in `0..=DUTY_SCALE_MAX` and a `u16` timer
/// auto-reload value. Supports the full timer range, including high configured
/// PWM periods.
#[inline(never)]
pub(crate) const fn div2000_pwm(n: u32) -> u32 {
    debug_assert!(n <= PWM_NUMERATOR_MAX);
    if n > 16_000_000 {
        return div5_u8_timer_product(div5_u8_timer_product(div5_u8_timer_product(n >> 4)));
    }
    let reduced = n >> 4;
    let mut q = (reduced * 4195) >> 19;
    if q * 125 > reduced {
        q -= 1;
    }
    if q * 125 > reduced {
        q -= 1;
    }
    q
}

/// Exact division by 1400 for the voltage-ramp numerator, whose maximum is
/// `(RAMP_VOLTAGE_HIGH_CV - RAMP_VOLTAGE_LOW_CV) * 9`.
#[inline(never)]
pub(crate) const fn div1400_ramp(n: u32) -> u32 {
    debug_assert!(n <= 12_600);
    let mut q = (n * 11_984) >> 24;
    if q * 1_400 > n {
        q -= 1;
    }
    q
}

/// Exact division by 1910 for the sine-start duty map. The input span is at
/// most 1910 and the duty span is at most `DUTY_SCALE_MAX`.
#[inline(never)]
pub(crate) const fn div1910_sine(n: u32) -> u32 {
    debug_assert!(n <= 1910 * crate::constants::DUTY_SCALE_MAX as u32);
    let mut q = (n * 1098) >> 21;
    if q * 1910 > n {
        q -= 1;
    }
    q
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn div3_matches_signed_division_over_motor_range_and_i32_edges() {
        for n in -1_000_000..=1_000_000 {
            assert_eq!(div3_i32(n), n / 3, "n={n}");
        }
        for n in [
            i32::MIN,
            i32::MIN + 1,
            -4,
            -3,
            -2,
            -1,
            0,
            1,
            2,
            3,
            4,
            i32::MAX,
        ] {
            assert_eq!(div3_i32(n), n / 3, "n={n}");
        }
    }

    #[test]
    fn div3_u32_matches_division_over_its_full_domain() {
        for wide in 0..=u32::MAX as u64 {
            let n = wide as u32;
            assert_eq!(div3_u32(n), n / 3, "n={n}");
        }
    }

    #[test]
    fn div5_matches_division_over_its_full_setting_timer_domain() {
        for n in 0..=DIV5_INPUT_MAX {
            assert_eq!(div5_u8_timer_product(n), n / 5, "n={n}");
            assert_eq!(div10_u8_timer_product(n), n / 10, "n={n}");
        }
    }

    #[test]
    fn div2000_matches_division_over_full_duty_and_u16_timer_domain() {
        for n in 0..=PWM_NUMERATOR_MAX {
            assert_eq!(
                div2000_pwm(n),
                n / crate::constants::DUTY_SCALE_MAX as u32,
                "n={n}"
            );
        }
    }

    #[test]
    fn div1400_matches_division_over_the_full_ramp_domain() {
        for n in 0..=12_600 {
            assert_eq!(div1400_ramp(n), n / 1400, "n={n}");
        }
    }

    #[test]
    fn div1910_matches_division_over_the_full_sine_map_domain() {
        for n in 0..=1910 * crate::constants::DUTY_SCALE_MAX as u32 {
            assert_eq!(div1910_sine(n), n / 1910, "n={n}");
        }
    }
}
