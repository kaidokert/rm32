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

/// Default TIM1 auto-reload value.
/// At 64MHz (G071): 64MHz / (1999+1) = 32kHz PWM frequency.
/// At 48MHz (F051): 48MHz / (1999+1) = 24kHz.
pub const TIM1_DEFAULT_ARR: u16 = 1999;

/// Arming timeout in 20kHz ticks (20000 ticks = 1.0 second).
/// ESC arms after receiving zero throttle for this duration.
pub const ARMING_TIMEOUT_TICKS: u32 = 20000;

/// Default initial commutation interval in timer ticks (0.5µs each).
/// 10000 ticks = 5ms between commutations = very slow startup.
pub const INITIAL_COMMUTATION_INTERVAL: u32 = 10000;

/// Zero-cross counter cap. Prevents overflow; after 10000 ZCs the motor
/// is considered reliably running.
pub const ZERO_CROSS_CAP: u32 = 10000;

/// Commutation interval threshold (timer ticks) to exit old_routine polling mode
/// and switch to interrupt-driven BEMF detection. Lower = higher RPM.
pub const OLD_ROUTINE_EXIT_INTERVAL: u32 = 2000;

/// Minimum zero-cross count before exiting old_routine.
/// Ensures enough successful commutations before trusting interrupt-driven mode.
pub const OLD_ROUTINE_EXIT_ZC: u32 = 20;

/// Bidir DShot midpoint. Values 0-1047 = reverse, 1048-2047 = forward.
pub const BIDIR_MIDPOINT: u16 = 1048;

/// Low voltage cutoff counter threshold (normal mode).
/// At 10kHz main loop rate: 10000 counts = 1.0 second sustained low voltage.
pub const LVC_NORMAL_THRESHOLD: u16 = 10000;

/// Low voltage cutoff counter threshold during stepper_sine startup.
/// Fast cutoff (0.1s) to protect batteries under heavy startup current draw.
pub const LVC_STARTUP_THRESHOLD: u16 = 1000;

/// Desync recovery: average_interval is reset to this value (5ms between commutations).
/// Provides a safe slow-speed starting point after desync event.
pub const DESYNC_RESET_INTERVAL: u32 = 5000;

/// Desync detection: only triggers when average_interval < this value.
/// Prevents false desync detection at very low RPM where intervals are naturally large.
pub const DESYNC_MAX_INTERVAL: u32 = 2000;

/// BEMF timeout threshold at low throttle (< 150). Lenient to avoid false desync
/// when motor is barely spinning and BEMF signal is weak.
pub const BEMF_TIMEOUT_LENIENT: u8 = 100;

/// BEMF timeout threshold at high throttle (>= 150). Strict because at high RPM
/// a missed BEMF event indicates a real problem.
pub const BEMF_TIMEOUT_STRICT: u8 = 10;

/// Throttle level below which the lenient BEMF timeout is used.
pub const BEMF_LENIENT_THROTTLE: u16 = 150;

/// Interval timer threshold for stall detection (timer ticks at 2MHz).
/// 45000 ticks = 22.5ms without a BEMF zero-cross → motor is stalled.
pub const BEMF_STALL_TIMER_THRESHOLD: u32 = 45000;

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

/// Signal timeout threshold (20kHz ticks). 10000 = 0.5 second with no valid input.
pub const SIGNAL_TIMEOUT_DISARM: u16 = 10000;

/// Sine startup: throttle below which BEMF timeout is cleared.
pub const SINE_BEMF_CLEAR_THROTTLE: u16 = 160;

/// Sine mode: throttle below which slow stepping is used (vs changeover acceleration).
pub const SINE_SLOW_STEP_THROTTLE: u16 = 137;

/// Sine mode: throttle above which changeover to BLDC may occur.
pub const SINE_CHANGEOVER_THROTTLE: u16 = 200;

/// Sine mode step delay at high throttle (µs).
pub const SINE_FAST_STEP_DELAY: u16 = 80;

/// Sine mode step delay at medium throttle (µs).
pub const SINE_MEDIUM_STEP_DELAY: u16 = 120;

/// Custom LED blink half-period (main loop ticks).
pub const LED_BLINK_HALF_PERIOD: u16 = 2000;

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
