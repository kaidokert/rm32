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

/// Fixed-point shift for commutation advance timing.
/// `advance = (temp_advance * commutation_interval) >> ADVANCE_SHIFT`
/// With ADVANCE_SHIFT=6, each unit of temp_advance ≈ 360°/64 ≈ 5.6° of advance.
pub const ADVANCE_SHIFT: u32 = 6;
