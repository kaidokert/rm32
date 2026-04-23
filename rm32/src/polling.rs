//! Polling-mode zero-cross detection (zcfoundroutine equivalent).
//!
//! Used during the first few commutations at startup before the
//! interrupt-driven BEMF detection is reliable. This is a blocking
//! routine that busy-waits for the commutation timing delay.

use crate::control::state::MotorState;
use crate::control::tick::ControlHal;
use crate::hal;

impl MotorState {
    /// Polling-mode zero-cross found routine.
    /// Blocking: busy-waits until the interval timer exceeds waitTime.
    /// Called from the old_routine BEMF polling path in tenKhzRoutine.
    pub fn zc_found_routine(&mut self, hal: &mut impl ControlHal) {
        // Read current interval timer and reset
        self.bemf.this_zc_time = hal.count() as u16;
        hal.set_count(0);

        // Update commutation interval (exponential moving average, 75/25 split)
        let ci = self.timing.commutation_interval;
        self.timing.commutation_interval =
            (self.bemf.this_zc_time as u32 + 3 * ci) / 4;

        // Calculate advance and wait time
        let advance = (self.bemf.temp_advance as u32 * self.timing.commutation_interval) >> 6;
        self.bemf.wait_time =
            (self.timing.commutation_interval as u16 / 2).wrapping_sub(advance as u16);

        // Blocking wait for commutation timing
        // Early exit if zero_crosses < 5 (startup — don't stall)
        while (hal.count() as u16) < self.bemf.wait_time {
            if self.timing.zero_crosses < 5 {
                break;
            }
        }

        // Commutate
        self.commutate(hal);
        self.bemf.counter = 0;
        self.bemf.bad_count = 0;

        // Increment zero crosses
        if self.timing.zero_crosses < 10000 {
            self.timing.zero_crosses += 1;
        }

        // Check for transition from polling to interrupt mode
        if self.config.stall_protection != 0 || self.config.rc_car_reverse != 0 {
            if self.timing.zero_crosses >= 20 && self.timing.commutation_interval <= 2000 {
                self.old_routine = false;
                hal.enable_interrupts();
            }
        } else {
            if self.timing.commutation_interval < self.timing.polling_mode_changeover as u32 {
                self.old_routine = false;
                hal.enable_interrupts();
            }
        }
    }
}
