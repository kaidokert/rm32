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

/// Electrical period in microseconds to average step interval in timer counts.
#[inline(always)]
#[cfg(any(feature = "m0-exact-arithmetic", test))]
pub fn e_com_us_to_avg_ticks<P: Platform>(e_com_us: i32) -> i32 {
    if P::TIMER_HZ == 2_000_000 {
        crate::fast_math::div3_i32(e_com_us)
    } else {
        (e_com_us as i64 * P::TIMER_HZ as i64 / 6_000_000) as i32
    }
}

/// Electrical period in microseconds to average step interval in timer counts.
#[inline(always)]
#[cfg(not(any(feature = "m0-exact-arithmetic", test)))]
pub fn e_com_us_to_avg_ticks<P: Platform>(e_com_us: i32) -> i32 {
    if P::TIMER_HZ == 2_000_000 {
        e_com_us / 3
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
