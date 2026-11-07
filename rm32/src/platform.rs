//! Static facts a port supplies about its chip and board.
//!
//! One implementation per port (a zero-sized type). No associated const has
//! a default: a default equal to one chip's value is exactly the coupling
//! this trait exists to remove.

/// Clock and board facts the control core is written against.
pub trait Platform {
    /// Control-tick rate (AM32 `tenKhzRoutine`, `LOOP_FREQUENCY_HZ`), Hz.
    const TICK_HZ: u32;
    /// Interval / commutation timer count rate, Hz.
    const TIMER_HZ: u32;
    /// Input-capture timer count rate at prescaler 0, Hz. Protocol
    /// detection sets the prescaler from it (AM32: `CPU_MHZ / 6` for the
    /// detection frame, `CPU_MHZ - 1` for servo).
    const CAPTURE_HZ: u32;
    /// Bidirectional DShot GCR output: one bit is `1 << GCR_SHIFT` timer
    /// counts (AM32: 6 on F051-class parts, 7 on G071 / L431 / G431).
    const GCR_SHIFT: u8;

    /// Bus voltage divider as its true ratio x100 (11:1 = 1100).
    const VOLTAGE_DIVIDER_X100: u16;
    /// Current sense gain, AM32 `MILLIVOLT_PER_AMP`; 0 = no current sensor.
    const MILLIVOLT_PER_AMP: u16;
    /// Current sense offset, AM32 `CURRENT_OFFSET`.
    const CURRENT_OFFSET: i16;
    /// Temperature from an external NTC instead of the MCU's sensor.
    const USE_NTC: bool;
    /// Largest ADC count (AM32's 12-bit STM32 ADCs: 4095).
    const ADC_FULL_SCALE: u16;
    /// ADC reference voltage, mV (AM32: 3300).
    const ADC_VREF_MV: u16;

    /// Motor PWM timer period (auto-reload) at the base carrier, AM32
    /// `TIMER1_MAX_ARR`. Shared state starts with it until the main loop
    /// publishes the configured period.
    const PWM_BASE_PERIOD: u16;
}

/// Microseconds to interval/commutation timer ticks (truncating). Use in a
/// const context (an associated const of [`crate::constants::Timing`]) so it
/// folds at compile time: the 64-bit arithmetic must never reach a core
/// without hardware divide.
pub const fn us_to_ticks<P: Platform>(us: u32) -> u32 {
    validate::<P>();
    ((us as u64 * P::TIMER_HZ as u64) / 1_000_000) as u32
}

/// Milliseconds to control ticks (truncating); const-context only, as
/// [`us_to_ticks`].
pub const fn ms_to_control_ticks<P: Platform>(ms: u32) -> u32 {
    validate::<P>();
    ((ms as u64 * P::TICK_HZ as u64) / 1000) as u32
}

/// Check the ranges the core's counters and capture protocols require.
/// Const-context only, like the converters that call it: an unsupported
/// port fails to compile instead of truncating a counter.
pub const fn validate<P: Platform>() {
    assert!(P::TICK_HZ >= 1000 && P::TICK_HZ <= 230_000);
    assert!(P::TICK_HZ % 1000 == 0);
    assert!(P::TIMER_HZ >= 1_000_000 && P::TIMER_HZ <= 2_000_000);
    assert!(P::CAPTURE_HZ >= 1_000_000 && P::CAPTURE_HZ <= 255_000_000);
    assert!(P::CAPTURE_HZ % 1_000_000 == 0);
    assert!(P::GCR_SHIFT < 16);
    assert!(P::PWM_BASE_PERIOD > 0);
}

/// Sum of the last six commutation intervals (timer ticks) to `e_com_time`,
/// the electrical period in microseconds. At a 2 MHz timer this is AM32's
/// `(sum + 4) >> 1` (the +4 is AM32's rounding bias, kept); other clocks
/// round to nearest. The branch is on a const and folds away.
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

/// AM32's clocks (20 kHz control tick, 2 MHz timer) and default board
/// (`BoardConfig::DEFAULT`) for the core's own unit tests. Not a default:
/// ports and the harness name their own platform.
#[cfg(test)]
pub(crate) struct TestPlatform;

#[cfg(test)]
impl Platform for TestPlatform {
    const TICK_HZ: u32 = 20_000;
    const TIMER_HZ: u32 = 2_000_000;
    const CAPTURE_HZ: u32 = 64_000_000;
    const GCR_SHIFT: u8 = 7;
    const VOLTAGE_DIVIDER_X100: u16 = 1100;
    const MILLIVOLT_PER_AMP: u16 = 20;
    const CURRENT_OFFSET: i16 = 0;
    const USE_NTC: bool = false;
    const ADC_FULL_SCALE: u16 = 4095;
    const ADC_VREF_MV: u16 = 3300;
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

    #[test]
    fn non_divisor_timer_hz_converts_ticks_without_overflow() {
        struct FractionalTimerPlatform;
        impl Platform for FractionalTimerPlatform {
            const TICK_HZ: u32 = 20_000;
            const TIMER_HZ: u32 = 1_500_000;
            const CAPTURE_HZ: u32 = 64_000_000;
            const GCR_SHIFT: u8 = 7;
            const VOLTAGE_DIVIDER_X100: u16 = 1100;
            const MILLIVOLT_PER_AMP: u16 = 20;
            const CURRENT_OFFSET: i16 = 0;
            const USE_NTC: bool = false;
            const ADC_FULL_SCALE: u16 = 4095;
            const ADC_VREF_MV: u16 = 3300;
            const PWM_BASE_PERIOD: u16 = 1999;
        }
        assert_eq!(ticks_to_am32::<FractionalTimerPlatform>(150), 200);
        assert_eq!(am32_ticks_to::<FractionalTimerPlatform>(200), 150);
    }
}
