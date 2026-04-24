//! Semantic constants replacing magic numbers throughout the codebase.

/// DShot throttle resolution (11-bit): 0-2047
pub const DSHOT_MAX_THROTTLE: u16 = 2047;

/// Minimum throttle signal that counts as "not zero" (DShot value 48+)
pub const THROTTLE_MIN_SIGNAL: u16 = 47;

/// Maximum duty cycle value in the PWM scaling system
pub const DUTY_SCALE_MAX: u16 = 2000;

/// Default TIM1 auto-reload value (24kHz PWM at typical clock)
pub const TIM1_DEFAULT_ARR: u16 = 1999;

/// Arming timeout: ticks at zero throttle before arming (20000 = 1s at 20kHz)
pub const ARMING_TIMEOUT_TICKS: u32 = 20000;

/// Default initial commutation interval (slow startup)
pub const INITIAL_COMMUTATION_INTERVAL: u32 = 10000;

/// Zero-cross counter cap (matches C behavior)
pub const ZERO_CROSS_CAP: u32 = 10000;

/// Commutation interval threshold to exit old_routine polling mode
pub const OLD_ROUTINE_EXIT_INTERVAL: u32 = 2000;

/// Zero-cross count threshold to exit old_routine
pub const OLD_ROUTINE_EXIT_ZC: u32 = 20;

/// Bidir DShot midpoint (below = reverse, above = forward)
pub const BIDIR_MIDPOINT: u16 = 1048;

/// Low voltage cutoff counter threshold (normal mode, ~10s at tick rate)
pub const LVC_NORMAL_THRESHOLD: u16 = 10000;

/// Low voltage cutoff counter threshold (stepper_sine startup, ~0.1s)
pub const LVC_STARTUP_THRESHOLD: u16 = 1000;

/// Desync recovery: reset average_interval to this value
pub const DESYNC_RESET_INTERVAL: u32 = 5000;

/// Desync detection: minimum average_interval to trigger
pub const DESYNC_MAX_INTERVAL: u32 = 2000;

/// BEMF timeout: lenient threshold at low throttle
pub const BEMF_TIMEOUT_LENIENT: u8 = 100;

/// BEMF timeout: strict threshold at high throttle
pub const BEMF_TIMEOUT_STRICT: u8 = 10;

/// Throttle level below which BEMF timeout is lenient
pub const BEMF_LENIENT_THROTTLE: u16 = 150;

/// Fixed-point shift for commutation advance timing.
/// advance = (temp_advance * commutation_interval) >> ADVANCE_SHIFT
/// Each unit of temp_advance represents ~1.4 degrees (360 / 256).
pub const ADVANCE_SHIFT: u32 = 6;
