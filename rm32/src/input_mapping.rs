//! Throttle input mapping — bidirectional, servo, RC-car reverse.
//!
//! Pure functions extracted from the legacy `set_input()` in tick.rs.
//! Used by the test harness to map raw throttle input before calling
//! `isr_logic::ten_khz_tick()`.

use crate::constants::{DSHOT_MAX_THROTTLE, DUTY_SCALE_MAX, STARTUP_ZC_BASE, THROTTLE_MIN_SIGNAL};
use crate::functions::map;

/// Result of bidirectional input mapping.
pub struct BidirResult {
    /// Mapped throttle value (0-2047)
    pub adjusted: u16,
    /// Whether direction should be reversed
    pub reverse: bool,
    /// Whether prop brake is active
    pub prop_brake: bool,
}

/// Map DShot bidirectional input.
///
/// DShot bidir: 0-47 = commands, 48-1047 = reverse, 1048-2047 = forward.
/// Returns adjusted throttle (0-2047) and direction change flags.
pub fn dshot_bidir(
    newinput: u16,
    forward: bool,
    dir_reversed: bool,
    commutation_interval: u32,
    duty_cycle: u16,
    stepper_sine: bool,
    reverse_speed_threshold: u16,
) -> BidirResult {
    let reversing_dead_band = 1u16;
    let can_reverse =
        (commutation_interval > reverse_speed_threshold as u32 && duty_cycle < 200) || stepper_sine;

    if newinput > crate::constants::DSHOT_BIDIR_BRAKE_LIMIT {
        let want_forward = !dir_reversed;
        let reverse = forward != want_forward && can_reverse;
        let adjusted = if reverse || forward == want_forward {
            ((newinput.saturating_sub(crate::constants::BIDIR_MIDPOINT)) * 2 + THROTTLE_MIN_SIGNAL)
                .saturating_sub(reversing_dead_band)
        } else {
            0 // blocked: can't reverse at this speed
        };
        BidirResult {
            adjusted,
            reverse: false,
            prop_brake: false,
        }
    } else if newinput > THROTTLE_MIN_SIGNAL {
        // Motor needs to be going in the "normal forward" direction to need reversal
        let needs_reversal = forward != dir_reversed;
        let reverse = needs_reversal && can_reverse;
        let adjusted = if reverse || !needs_reversal {
            ((newinput.saturating_sub(THROTTLE_MIN_SIGNAL + 1)) * 2 + THROTTLE_MIN_SIGNAL)
                .saturating_sub(reversing_dead_band)
        } else {
            0
        };
        BidirResult {
            adjusted,
            reverse: false,
            prop_brake: false,
        }
    } else {
        BidirResult {
            adjusted: 0,
            reverse: false,
            prop_brake: false,
        }
    }
}

/// Map DShot RC-car reverse input.
pub fn dshot_rc_car(
    newinput: u16,
    forward: bool,
    dir_reversed: bool,
    prop_brake_active: bool,
    return_to_center: bool,
) -> BidirResult {
    let reversing_dead_band = 1u16;
    if newinput > crate::constants::DSHOT_BIDIR_BRAKE_LIMIT {
        let want_forward = !dir_reversed;
        let fwd_adjusted = ((newinput - crate::constants::BIDIR_MIDPOINT) * 2
            + THROTTLE_MIN_SIGNAL)
            .saturating_sub(reversing_dead_band);
        if forward != want_forward {
            if return_to_center {
                // Flip direction AND apply throttle (matches C fall-through)
                return BidirResult {
                    adjusted: fwd_adjusted,
                    reverse: true,
                    prop_brake: false,
                };
            }
            return BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            };
        }
        if !prop_brake_active {
            BidirResult {
                adjusted: fwd_adjusted,
                reverse: false,
                prop_brake: false,
            }
        } else {
            BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            }
        }
    } else if newinput > THROTTLE_MIN_SIGNAL {
        let want_reverse = dir_reversed;
        let rev_adjusted = ((newinput.saturating_sub(THROTTLE_MIN_SIGNAL + 1)) * 2
            + THROTTLE_MIN_SIGNAL)
            .saturating_sub(reversing_dead_band);
        if forward != want_reverse {
            if return_to_center {
                return BidirResult {
                    adjusted: rev_adjusted,
                    reverse: true,
                    prop_brake: false,
                };
            }
            return BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            };
        }
        if !prop_brake_active {
            BidirResult {
                adjusted: rev_adjusted,
                reverse: false,
                prop_brake: false,
            }
        } else {
            BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            }
        }
    } else {
        // Zero input: clear brake and enable return_to_center
        BidirResult {
            adjusted: 0,
            reverse: false,
            prop_brake: false,
        }
    }
}

/// Map servo bidirectional input (non-RC-car).
/// Speed-gated direction reversal with dead band around center (1000).
#[allow(clippy::too_many_arguments)]
pub fn servo_bidir(
    newinput: u16,
    forward: bool,
    dir_reversed: bool,
    commutation_interval: u32,
    duty_cycle: u16,
    stepper_sine: bool,
    reverse_speed_threshold: u16,
    dead_band: u16,
) -> BidirResult {
    let db2 = dead_band << 1;
    let center: u16 = 1000;
    let can_reverse =
        (commutation_interval > reverse_speed_threshold as u32 && duty_cycle < 200) || stepper_sine;

    if newinput > center + db2 {
        // Forward range
        let want_forward = !dir_reversed;
        if forward != want_forward {
            // Wrong direction — try to reverse
            if can_reverse {
                let adjusted = map(
                    newinput as i32,
                    (center + db2) as i32,
                    2000,
                    THROTTLE_MIN_SIGNAL as i32,
                    DSHOT_MAX_THROTTLE as i32,
                ) as u16;
                return BidirResult {
                    adjusted,
                    reverse: true,
                    prop_brake: false,
                };
            }
            // Can't reverse at this speed — idle (matches C: newinput=1000)
            return BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: false,
            };
        }
        let adjusted = map(
            newinput as i32,
            (center + db2) as i32,
            2000,
            THROTTLE_MIN_SIGNAL as i32,
            DSHOT_MAX_THROTTLE as i32,
        ) as u16;
        BidirResult {
            adjusted,
            reverse: false,
            prop_brake: false,
        }
    } else if newinput < center.saturating_sub(db2) {
        // Reverse range
        let needs_reversal = forward != dir_reversed;
        if needs_reversal {
            if can_reverse {
                let adjusted = map(
                    newinput as i32,
                    0,
                    (center - db2) as i32,
                    DSHOT_MAX_THROTTLE as i32,
                    THROTTLE_MIN_SIGNAL as i32,
                ) as u16;
                return BidirResult {
                    adjusted,
                    reverse: true,
                    prop_brake: false,
                };
            }
            return BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: false,
            };
        }
        let adjusted = map(
            newinput as i32,
            0,
            (center - db2) as i32,
            DSHOT_MAX_THROTTLE as i32,
            THROTTLE_MIN_SIGNAL as i32,
        ) as u16;
        BidirResult {
            adjusted,
            reverse: false,
            prop_brake: false,
        }
    } else {
        // Dead band
        BidirResult {
            adjusted: 0,
            reverse: false,
            prop_brake: false,
        }
    }
}

/// Map servo RC-car bidirectional input.
/// Brake-and-reverse with return-to-center handshake, dead band around 1000.
pub fn servo_rc_car(
    newinput: u16,
    forward: bool,
    dir_reversed: bool,
    prop_brake_active: bool,
    return_to_center: bool,
    dead_band: u16,
) -> BidirResult {
    let db2 = dead_band << 1;
    let center: u16 = 1000;

    if newinput > center + db2 {
        let want_forward = !dir_reversed;
        let fwd_adjusted = map(
            newinput as i32,
            (center + db2) as i32,
            2000,
            THROTTLE_MIN_SIGNAL as i32,
            DSHOT_MAX_THROTTLE as i32,
        ) as u16;
        if forward != want_forward {
            if return_to_center {
                // Flip direction AND apply throttle (C falls through to map)
                return BidirResult {
                    adjusted: fwd_adjusted,
                    reverse: true,
                    prop_brake: false,
                };
            }
            return BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            };
        }
        if !prop_brake_active {
            BidirResult {
                adjusted: fwd_adjusted,
                reverse: false,
                prop_brake: false,
            }
        } else {
            BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            }
        }
    } else if newinput < center.saturating_sub(db2) {
        let want_reverse = dir_reversed;
        let rev_adjusted = map(
            newinput as i32,
            0,
            (center - db2) as i32,
            DSHOT_MAX_THROTTLE as i32,
            THROTTLE_MIN_SIGNAL as i32,
        ) as u16;
        if forward != want_reverse {
            if return_to_center {
                return BidirResult {
                    adjusted: rev_adjusted,
                    reverse: true,
                    prop_brake: false,
                };
            }
            return BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            };
        }
        if !prop_brake_active {
            BidirResult {
                adjusted: rev_adjusted,
                reverse: false,
                prop_brake: false,
            }
        } else {
            BidirResult {
                adjusted: 0,
                reverse: false,
                prop_brake: true,
            }
        }
    } else {
        // Dead band: clear brake, enable return_to_center
        BidirResult {
            adjusted: 0,
            reverse: false,
            prop_brake: false,
        }
    }
}

/// Map sine-start throttle to input value.
/// Returns the mapped input value.
pub fn sine_start_map(adjusted: u16, changeover_level: u8) -> u16 {
    let changeover = (changeover_level as u16) * 20;
    if adjusted < 30 {
        0
    } else if adjusted < changeover {
        map(
            adjusted as i32,
            SINE_DEAD_BAND as i32,
            changeover as i32,
            THROTTLE_MIN_SIGNAL as i32,
            SINE_MID_THROTTLE as i32,
        ) as u16
    } else {
        map(
            adjusted as i32,
            changeover as i32,
            DSHOT_MAX_THROTTLE as i32,
            SINE_MID_THROTTLE as i32,
            DSHOT_MAX_THROTTLE as i32,
        ) as u16
    }
}

/// Sine start: input dead band (below this = zero throttle).
const SINE_DEAD_BAND: u16 = 30;
/// Sine start: midpoint throttle for slow→fast transition.
const SINE_MID_THROTTLE: u16 = 160;
/// Sine start: minimum input for slow stepping (above changeover).
const SINE_SLOW_STEP_MIN: u16 = 137;
/// Sine start: extra minimum duty offset for sine mode.
const SINE_DUTY_OFFSET: i32 = 40;

/// Map throttle input to duty cycle setpoint.
/// Returns the duty setpoint (0-2000 scale).
pub fn throttle_to_setpoint(input: u16, use_sine_start: bool, minimum_duty: u16) -> u16 {
    if use_sine_start {
        map(
            input as i32,
            SINE_SLOW_STEP_MIN as i32,
            DSHOT_MAX_THROTTLE as i32,
            minimum_duty as i32 + SINE_DUTY_OFFSET,
            DUTY_SCALE_MAX as i32,
        ) as u16
    } else {
        map(
            input as i32,
            THROTTLE_MIN_SIGNAL as i32,
            DSHOT_MAX_THROTTLE as i32,
            minimum_duty as i32,
            DUTY_SCALE_MAX as i32,
        ) as u16
    }
}

/// Apply startup duty floor/ceiling limits.
pub fn clamp_startup_duty(
    setpoint: u16,
    input: u16,
    zero_crosses: u32,
    stall_protection: u8,
    min_startup: u16,
    startup_max: u16,
    maximum: u16,
) -> u16 {
    let mut sp = setpoint;
    let safe_shift = stall_protection.min(5); // clamp to prevent shift overflow
    if input >= THROTTLE_MIN_SIGNAL && zero_crosses < (STARTUP_ZC_BASE >> safe_shift) {
        if sp < min_startup {
            sp = min_startup;
        }
        if sp > startup_max {
            sp = startup_max;
        }
    }
    if sp > maximum {
        sp = maximum;
    }
    sp
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sine_start_map_below_deadband() {
        assert_eq!(sine_start_map(20, 10), 0);
    }

    #[test]
    fn sine_start_map_in_range() {
        let v = sine_start_map(100, 10); // changeover = 200
        assert!(v >= 47 && v <= 160);
    }

    #[test]
    fn sine_start_map_above_changeover() {
        let v = sine_start_map(500, 10); // changeover = 200
        assert!(v > 160);
    }

    #[test]
    fn throttle_to_setpoint_normal() {
        let sp = throttle_to_setpoint(1000, false, 50);
        assert!(sp > 50 && sp < 2000);
    }

    #[test]
    fn throttle_to_setpoint_sine() {
        let sp = throttle_to_setpoint(1000, true, 50);
        assert!(sp > 50 && sp < 2000);
    }

    #[test]
    fn dshot_bidir_forward() {
        let r = dshot_bidir(1500, true, false, 5000, 100, false, 1500);
        assert!(r.adjusted > 0);
        assert!(!r.reverse);
    }

    #[test]
    fn dshot_bidir_zero_input() {
        let r = dshot_bidir(0, true, false, 5000, 100, false, 1500);
        assert_eq!(r.adjusted, 0);
    }

    #[test]
    fn clamp_startup_limits() {
        // During startup (low zero_crosses), clamp to min_startup
        assert_eq!(clamp_startup_duty(10, 100, 5, 0, 120, 200, 2000), 120);
        // Above startup_max
        assert_eq!(clamp_startup_duty(300, 100, 5, 0, 120, 200, 2000), 200);
        // After startup (high zero_crosses), no clamping
        assert_eq!(clamp_startup_duty(300, 100, 100, 0, 120, 200, 2000), 300);
    }
}
