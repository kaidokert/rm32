//! Semantic constants replacing magic numbers throughout the codebase.
//!
//! Each constant documents what it represents, its units, and why that value
//! was chosen. Prevents "what does 2000 mean here?" questions.

/// DShot throttle resolution (11-bit): valid range 0-2047.
/// DShot frames encode throttle as an 11-bit value where 48-2047 = active throttle.
pub const DSHOT_MAX_THROTTLE: u16 = 2047;

/// Minimum throttle signal that counts as "motor should spin" (DShot value 48+).
/// Values 0-47 are reserved for DShot commands; 48+ = throttle.
pub const THROTTLE_MIN_SIGNAL: u16 = 47;

/// Maximum duty cycle in the PWM scaling system.
/// Maps to 100% of TIM1_ARR. The actual PWM compare value is
/// `(duty_cycle * tim1_arr) / DUTY_SCALE_MAX`.
pub const DUTY_SCALE_MAX: u16 = 2000;

/// Base interval for periodic KISS telemetry, in milliseconds.
pub const TELEMETRY_INTERVAL_MS: u16 = 30;

/// Zero-cross counter cap. Prevents overflow; after 10000 ZCs the motor
/// is considered reliably running.
pub const ZERO_CROSS_CAP: u32 = 10000;

/// Minimum zero-cross count before exiting old_routine.
/// Ensures enough successful commutations before trusting interrupt-driven mode.
pub const OLD_ROUTINE_EXIT_ZC: u32 = 20;

/// Servo PWM neutral/center position (pulse width mapped to 0-2047 scale).
/// Used as the center point for servo bidirectional dead band calculations.
pub const SERVO_CENTER: u16 = 1000;

/// Bidir DShot midpoint. Values 0-1047 = reverse, 1048-2047 = forward.
pub const BIDIR_MIDPOINT: u16 = 1048;

/// Low voltage cutoff counter threshold (normal mode).
/// Counted at 1 kHz: 10000 counts = 10 seconds sustained low voltage.
pub const LVC_NORMAL_THRESHOLD: u16 = 10000;

/// Low voltage cutoff counter threshold during stepper_sine startup.
/// Counted at 1 kHz: 100 counts = 0.1 second.
pub const LVC_STARTUP_THRESHOLD: u16 = 100;

/// Consecutive inverted-CRC DShot frames required before committing
/// auto-detected bidirectional DShot mode.
pub const BIDIR_CONFIRM_FRAMES: u8 = 4;

/// Consecutive high input-pin samples required before probing inverted-CRC DShot.
pub const BIDIR_IDLE_HIGH_FRAMES: u8 = 100;

/// BEMF timeout threshold at low throttle (< 150). Lenient to avoid false desync
/// when motor is barely spinning and BEMF signal is weak.
pub const BEMF_TIMEOUT_LENIENT: u8 = 100;

/// BEMF timeout threshold at high throttle (>= 150). Strict because at high RPM
/// a missed BEMF event indicates a real problem.
pub const BEMF_TIMEOUT_STRICT: u8 = 10;

/// Throttle level below which the lenient BEMF timeout is used.
pub const BEMF_LENIENT_THROTTLE: u16 = 150;

/// BEMF timeout fault latch value. When bemf_timeout_happened exceeds the
/// threshold, it's set to this sentinel (> any threshold) to indicate a
/// latched stuck-rotor fault that persists until motor conditions clear it.
pub const BEMF_FAULT_LATCHED: u8 = 102;

/// Base zero-cross count for startup phase. Below `STARTUP_ZC_BASE >> stall_protection`
/// zero-crosses, startup duty limits (min_startup/startup_max) are enforced.
/// Higher stall_protection narrows the window (15 for stall=1, 7 for stall=2, etc.).
pub const STARTUP_ZC_BASE: u32 = 30;

/// Fixed-point shift for commutation advance timing.
/// `advance = (temp_advance * commutation_interval) >> ADVANCE_SHIFT`
/// With ADVANCE_SHIFT=6, each unit of temp_advance ≈ 360°/64 ≈ 5.6° of advance.
pub const ADVANCE_SHIFT: u32 = 6;

/// Minimum zero-crosses before applying advance timing in old_routine BEMF polling.
/// Below this count, commutate immediately without waiting for advance delay.
pub const MIN_ZC_FOR_ADVANCE: u32 = 5;

/// Sine startup: throttle below which BEMF timeout is cleared.
pub const SINE_BEMF_CLEAR_THROTTLE: u16 = 160;

/// Sine mode: throttle below which slow stepping is used (vs changeover acceleration).
pub const SINE_SLOW_STEP_THROTTLE: u16 = 137;

/// Six-step drive yields to sine startup below this mapped throttle.
pub const SINE_ENTRY_THROTTLE: u16 = THROTTLE_MIN_SIGNAL + 80;

/// Sine mode: throttle above which changeover to BLDC may occur.
pub const SINE_CHANGEOVER_THROTTLE: u16 = 200;

/// Sine startup changeover commutation step.
pub const SINE_CHANGEOVER_STEP: u8 = 5;

/// Sine mode step delay at high throttle (µs).
pub const SINE_FAST_STEP_DELAY: u16 = 80;

/// Sine mode step delay at medium throttle (µs).
pub const SINE_MEDIUM_STEP_DELAY: u16 = 120;

/// Custom LED high-throttle threshold (above this = solid on).
pub const LED_HIGH_THROTTLE: u16 = 1947;

/// DShot bidir: throttle boundary for reverse braking.
pub const DSHOT_BIDIR_BRAKE_LIMIT: u16 = 1047;

/// Calibration entry: minimum high-stick hold ticks before entering calibration.
pub const CALIBRATION_ENTRY_COUNT: u8 = 50;

/// Calibration entry: minimum throttle to start counting.
pub const CALIBRATION_MIN_THROTTLE: u16 = 1500;

/// Calibration entry: maximum jitter between readings.
pub const CALIBRATION_MAX_JITTER: u32 = 50;

// --- Ramp rate limiting ---

/// Voltage ramp: minimum battery voltage for mapping (8.0V).
pub const RAMP_VOLTAGE_LOW_CV: i32 = 800;
/// Voltage ramp: maximum battery voltage for mapping (22.0V).
pub const RAMP_VOLTAGE_HIGH_CV: i32 = 2200;
/// Voltage ramp: max duty change at low voltage.
pub const RAMP_VOLTAGE_CHANGE_MAX: i32 = 10;
/// Voltage ramp: min duty change at high voltage.
pub const RAMP_VOLTAGE_CHANGE_MIN: i32 = 1;
/// Zero-crosses / duty threshold for startup ramp profile.
pub const RAMP_STARTUP_THRESHOLD: u16 = 150;
// --- Brake ---

/// Brake strength per-unit scale factor (DUTY_SCALE_MAX / 10).
pub const BRAKE_STRENGTH_SCALE: u32 = 200;

use crate::platform::{Platform, ms_to_control_ticks, us_to_ticks};

/// Physical durations expressed in the selected port's timer units.
/// Blanket implementation keeps all thresholds tied to the same clocks.
pub trait Timing: Platform + Sized {
    const VALID: () = crate::platform::validate::<Self>();
    const ARMING_TIMEOUT_TICKS: u32 = ms_to_control_ticks::<Self>(1000);

    const PID_LOOP_DIVIDER: u8 = ms_to_control_ticks::<Self>(1) as u8;

    const CONTROL_TICKS_PER_MS: u16 = ms_to_control_ticks::<Self>(1) as u16;

    const SIGNAL_TIMEOUT_DISARM: u32 = ms_to_control_ticks::<Self>(500);
    const SIGNAL_TIMEOUT_UNARMED: u32 = Self::SIGNAL_TIMEOUT_DISARM * 4;

    const INITIAL_COMMUTATION_INTERVAL: u32 = us_to_ticks::<Self>(6250);

    const STARTUP_COMMUTATION_INTERVAL: u32 = us_to_ticks::<Self>(5000);

    const SINE_CHANGEOVER_INTERVAL: u32 = us_to_ticks::<Self>(4500);

    const OLD_ROUTINE_EXIT_INTERVAL: u32 = us_to_ticks::<Self>(1000);

    const OLD_ROUTINE_REENTRY_MARGIN: u32 = us_to_ticks::<Self>(250);

    const DESYNC_RESET_INTERVAL: u32 = us_to_ticks::<Self>(2500);

    const DESYNC_MAX_INTERVAL: u32 = us_to_ticks::<Self>(1000);

    const DESYNC_STOP_INTERVAL: u32 = us_to_ticks::<Self>(500);

    const BEMF_STALL_TIMER_THRESHOLD: u32 = us_to_ticks::<Self>(22_500);

    const RAMP_FAST_COMMUTATION_THRESHOLD: u32 = us_to_ticks::<Self>(100);
    const RAMP_LOW_RPM_INTERVAL: u32 = us_to_ticks::<Self>(250);

    const FILTER_SLOW_INTERVAL: u32 = us_to_ticks::<Self>(250);
    const FILTER_FAST_INTERVAL: u32 = us_to_ticks::<Self>(25);

    const VARIABLE_PWM2_LO: u32 = us_to_ticks::<Self>(50);
    const VARIABLE_PWM2_HI: u32 = us_to_ticks::<Self>(125);

    const LED_BLINK_HALF_PERIOD: u16 = ms_to_control_ticks::<Self>(100) as u16;
}

impl<P: Platform> Timing for P {}

/// Interval-telemetry period in control ticks for config slot
/// `config_interval` (AM32: `(30 - 1 + telemetry_on_interval)` ms).
pub const fn telemetry_interval_ticks<P: Platform>(config_interval: u8) -> u16 {
    (TELEMETRY_INTERVAL_MS - 1 + config_interval as u16) * <P as Timing>::CONTROL_TICKS_PER_MS
}
