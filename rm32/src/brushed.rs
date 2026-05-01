//! Brushed motor control mode.
//!
//! Drives a brushed motor using two-channel PWM (forward/reverse).
//! No BEMF sensing — just duty cycle control with direction.

use crate::functions::map;

/// Brushed motor direction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BrushedDirection {
    Forward,
    Reverse,
}

/// Brushed motor control state.
pub struct BrushedState {
    direction: BrushedDirection,
    direction_set: bool,
}

impl Default for BrushedState {
    fn default() -> Self {
        Self {
            direction: BrushedDirection::Forward,
            direction_set: false,
        }
    }
}

/// Result of a brushed control tick.
#[allow(dead_code)]
pub struct BrushedOutput {
    /// Duty cycle for active channels (0-2000 scale)
    pub(crate) duty: u16,
    pub(crate) direction: BrushedDirection,
}

/// Run one tick of the brushed control loop.
///
/// `input`: throttle 0-2047 (47+ = active)
/// `bidirectional`: if true, input 0-1047 = reverse, 1048-2047 = forward
/// `max_duty`: maximum allowed duty (from temperature/current limiting)
pub fn brushed_tick(
    state: &mut BrushedState,
    input: u16,
    bidirectional: bool,
    max_duty: u16,
) -> BrushedOutput {
    if input < 48 {
        // Below threshold: set direction on next throttle-up
        state.direction_set = false;
        return BrushedOutput {
            duty: 0,
            direction: state.direction,
        };
    }

    if bidirectional {
        if input >= 1048 {
            if !state.direction_set {
                state.direction = BrushedDirection::Forward;
                state.direction_set = true;
            }
            let duty = map(input as i32, 1048, 2047, 0, max_duty as i32) as u16;
            BrushedOutput {
                duty,
                direction: BrushedDirection::Forward,
            }
        } else {
            if !state.direction_set {
                state.direction = BrushedDirection::Reverse;
                state.direction_set = true;
            }
            let duty = map(input as i32, 1047, 48, 0, max_duty as i32) as u16;
            BrushedOutput {
                duty,
                direction: BrushedDirection::Reverse,
            }
        }
    } else {
        // Unidirectional: 48-2047 → 0-95% duty (reserve 5% headroom)
        let duty = map(input as i32, 48, 2047, 0, (max_duty as i32 * 95) / 100) as u16;
        BrushedOutput {
            duty,
            direction: BrushedDirection::Forward,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_throttle_no_duty() {
        let mut state = BrushedState::default();
        let out = brushed_tick(&mut state, 0, false, 2000);
        assert_eq!(out.duty, 0);
    }

    #[test]
    fn mid_throttle_unidirectional() {
        let mut state = BrushedState::default();
        let out = brushed_tick(&mut state, 1047, false, 2000);
        assert!(out.duty > 800 && out.duty < 1000);
    }

    #[test]
    fn bidirectional_forward() {
        let mut state = BrushedState::default();
        let out = brushed_tick(&mut state, 1500, true, 2000);
        assert_eq!(out.direction, BrushedDirection::Forward);
        assert!(out.duty > 0);
    }

    #[test]
    fn bidirectional_reverse() {
        let mut state = BrushedState::default();
        let out = brushed_tick(&mut state, 500, true, 2000);
        assert_eq!(out.direction, BrushedDirection::Reverse);
        assert!(out.duty > 0);
    }
}
