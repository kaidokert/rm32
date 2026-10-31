//! Static clock facts supplied by a hardware port.
//!
//! Each port names its clocks explicitly. Board ADC calibration and motor
//! policies remain separate from this timing contract.

/// Timer rates and PWM geometry used by the control core.
///
/// Control rates must be whole kHz in 1..=230 kHz (the 1 kHz dispatcher
/// and maximum telemetry slot must fit their counters). Interval timers
/// support 1..=2 MHz; their stall threshold fits a 16-bit counter. Capture
/// clocks are whole MHz in 1..=255 MHz. These ranges describe arithmetic
/// support, not hardware qualification or motor-loop tuning at other rates.
pub trait Platform {
    /// Control interrupt frequency in Hz.
    const TICK_HZ: u32;
    /// Interval and commutation timer frequency in Hz.
    const TIMER_HZ: u32;
    /// Input capture timer frequency at prescaler zero, in Hz.
    const CAPTURE_HZ: u32;
    /// Bidirectional DShot bit width is `1 << GCR_SHIFT` capture counts.
    const GCR_SHIFT: u8;
    /// Base PWM auto-reload value, before configuration is applied.
    const PWM_BASE_PERIOD: u16;
}

/// Microseconds to interval/commutation timer counts, truncating.
pub const fn us_to_ticks<P: Platform>(us: u32) -> u32 {
    validate::<P>();
    ((us as u64 * P::TIMER_HZ as u64) / 1_000_000) as u32
}

/// Milliseconds to control ticks, truncating.
pub const fn ms_to_control_ticks<P: Platform>(ms: u32) -> u32 {
    validate::<P>();
    ((ms as u64 * P::TICK_HZ as u64) / 1000) as u32
}

/// Check the ranges required by the core's counters and capture protocols.
pub const fn validate<P: Platform>() {
    assert!(P::TICK_HZ >= 1000 && P::TICK_HZ <= 230_000);
    assert!(P::TICK_HZ % 1000 == 0);
    assert!(P::TIMER_HZ >= 1_000_000 && P::TIMER_HZ <= 2_000_000);
    assert!(P::CAPTURE_HZ >= 1_000_000 && P::CAPTURE_HZ <= 255_000_000);
    assert!(P::CAPTURE_HZ % 1_000_000 == 0);
    assert!(P::GCR_SHIFT < 16);
    assert!(P::PWM_BASE_PERIOD > 0);
}

/// Sum of six step intervals to electrical period in microseconds.
/// Preserve the established +4 bias at 2 MHz; other rates round to nearest.
#[inline(always)]
pub fn six_intervals_to_us<P: Platform>(sum: u32) -> i32 {
    if P::TIMER_HZ == 2_000_000 {
        ((sum + 4) >> 1) as i32
    } else {
        ((sum as u64 * 1_000_000 + P::TIMER_HZ as u64 / 2) / P::TIMER_HZ as u64) as i32
    }
}

/// Platform timer ticks to AM32's 2 MHz timer ticks. AM32 states several
/// `map()` ranges in 2 MHz ticks; `map()` bisects, so its rounding depends on
/// the endpoints themselves, and the same curve at another clock must convert
/// the input rather than scale the endpoints. Free at 2 MHz.
#[inline(always)]
pub fn ticks_to_am32<P: Platform>(ticks: u32) -> u32 {
    const AM32_TIMER_HZ: u32 = 2_000_000;
    if P::TIMER_HZ == AM32_TIMER_HZ {
        ticks
    } else if AM32_TIMER_HZ.is_multiple_of(P::TIMER_HZ) {
        ticks.saturating_mul(AM32_TIMER_HZ / P::TIMER_HZ)
    } else {
        (ticks as u64 * AM32_TIMER_HZ as u64 / P::TIMER_HZ as u64).min(u32::MAX as u64) as u32
    }
}

/// AM32 2 MHz timer ticks to platform timer ticks (truncating); the inverse
/// of [`ticks_to_am32`] for thresholds AM32 derives at configuration time.
#[inline(always)]
pub fn am32_ticks_to<P: Platform>(am32_ticks: u32) -> u32 {
    const AM32_TIMER_HZ: u32 = 2_000_000;
    if P::TIMER_HZ == AM32_TIMER_HZ {
        am32_ticks
    } else {
        (am32_ticks as u64 * P::TIMER_HZ as u64 / AM32_TIMER_HZ as u64).min(u32::MAX as u64) as u32
    }
}

/// Electrical period in microseconds to average step interval in timer counts.
/// At a 2 MHz timer this is AM32's `average_interval = e_com_time / 3`.
#[inline(always)]
pub fn e_com_us_to_avg_ticks<P: Platform>(e_com_us: i32) -> i32 {
    if P::TIMER_HZ == 2_000_000 {
        crate::fast_math::div3_i32(e_com_us)
    } else {
        (e_com_us as i64 * P::TIMER_HZ as i64 / 6_000_000) as i32
    }
}

#[cfg(test)]
pub(crate) struct TestPlatform;
#[cfg(test)]
impl Platform for TestPlatform {
    const TICK_HZ: u32 = 20_000;
    const TIMER_HZ: u32 = 2_000_000;
    const CAPTURE_HZ: u32 = 64_000_000;
    const GCR_SHIFT: u8 = 7;
    const PWM_BASE_PERIOD: u16 = 1999;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// At 2 MHz both conversions are AM32's literal arithmetic, for every
    /// value the six-slot u16 sum and its e_com can take.
    #[test]
    fn two_mhz_conversions_are_am32_arithmetic() {
        for sum in (0..=6 * u16::MAX as u32)
            .step_by(7)
            .chain([6 * u16::MAX as u32])
        {
            assert_eq!(
                six_intervals_to_us::<TestPlatform>(sum),
                ((sum + 4) >> 1) as i32
            );
        }
        for e in (-200_000..=200_000).chain([i32::MIN + 1, i32::MAX]) {
            assert_eq!(e_com_us_to_avg_ticks::<TestPlatform>(e), e / 3);
        }
    }
}
