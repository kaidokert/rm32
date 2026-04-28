//! Throttle input mapping — bidirectional, servo, RC-car reverse.
//!
//! Pure functions extracted from the legacy `set_input()` in tick.rs.
//! Used by the test harness to map raw throttle input before calling
//! `isr_logic::ten_khz_tick()`.

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

    if newinput > 1047 {
        let want_forward = !dir_reversed;
        let reverse = forward != want_forward && can_reverse;
        let adjusted = if reverse || forward == want_forward {
            ((newinput.saturating_sub(1048)) * 2 + 47).saturating_sub(reversing_dead_band)
        } else {
            0 // blocked: can't reverse at this speed
        };
        BidirResult {
            adjusted,
            reverse,
            prop_brake: false,
        }
    } else if newinput > 47 {
        let want_reverse = dir_reversed;
        let reverse = forward == want_reverse && can_reverse;
        let adjusted = if reverse || forward == want_reverse {
            ((newinput.saturating_sub(48)) * 2 + 47).saturating_sub(reversing_dead_band)
        } else {
            0
        };
        BidirResult {
            adjusted,
            reverse,
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
    if newinput > 1047 {
        let want_forward = !dir_reversed;
        if forward != want_forward {
            // Wrong direction — brake or reverse on center return
            if return_to_center {
                return BidirResult {
                    adjusted: 0,
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
                adjusted: ((newinput - 1048) * 2 + 47).saturating_sub(reversing_dead_band),
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
    } else if newinput > 47 {
        let want_reverse = dir_reversed;
        if forward != want_reverse {
            if return_to_center {
                return BidirResult {
                    adjusted: 0,
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
                adjusted: ((newinput - 48) * 2 + 47).saturating_sub(reversing_dead_band),
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
        BidirResult {
            adjusted: 0,
            reverse: false,
            prop_brake: !prop_brake_active && !return_to_center,
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
        map(adjusted as i32, 30, changeover as i32, 47, 160) as u16
    } else {
        map(adjusted as i32, changeover as i32, 2047, 160, 2047) as u16
    }
}

/// Map throttle input to duty cycle setpoint.
/// Returns the duty setpoint (0-2000 scale).
pub fn throttle_to_setpoint(input: u16, use_sine_start: bool, minimum_duty: u16) -> u16 {
    if use_sine_start {
        map(input as i32, 137, 2047, minimum_duty as i32 + 40, 2000) as u16
    } else {
        map(input as i32, 47, 2047, minimum_duty as i32, 2000) as u16
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
    if input >= 47 && zero_crosses < (30u32 >> stall_protection) {
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
