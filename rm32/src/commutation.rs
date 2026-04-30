//! 6-step BLDC commutation logic.

/// Commutation state
pub struct Commutation {
    pub step: u8, // 1-6
    pub forward: bool,
    pub rising: bool,
    pub desync_check: bool,
    /// Per-step commutation intervals for e_com_time averaging.
    /// Written by ISR on each step advance, read by main loop.
    pub intervals: [u16; 6],
}

impl Commutation {
    pub fn new() -> Self {
        Self {
            step: 1,
            forward: true,
            rising: true,
            desync_check: false,
            intervals: [0; 6],
        }
    }

    /// Record the current commutation interval for this step.
    /// Called by ISR after each step advance, matching C:
    /// `commutation_intervals[step - 1] = commutation_interval`
    pub fn record_interval(&mut self, commutation_interval: u16) {
        self.intervals[(self.step - 1) as usize] = commutation_interval;
    }

    /// Advance one commutation step. Returns the new step number.
    /// Sets `desync_check` on step wrap.
    pub fn advance(&mut self) -> u8 {
        if self.forward {
            self.step += 1;
            if self.step > 6 {
                self.step = 1;
                self.desync_check = true;
            }
            self.rising = !self.step.is_multiple_of(2);
        } else {
            self.step -= 1;
            if self.step < 1 {
                self.step = 6;
                self.desync_check = true;
            }
            self.rising = self.step.is_multiple_of(2);
        }
        self.step
    }
}

impl Default for Commutation {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_wraps_6_to_1() {
        let mut c = Commutation::new();
        c.step = 6;
        c.advance();
        assert_eq!(c.step, 1);
        assert!(c.desync_check);
    }

    #[test]
    fn reverse_wraps_1_to_6() {
        let mut c = Commutation::new();
        c.forward = false;
        c.step = 1;
        c.advance();
        assert_eq!(c.step, 6);
        assert!(c.desync_check);
    }

    #[test]
    fn forward_rising_parity() {
        let mut c = Commutation::new();
        c.step = 1;
        c.advance(); // step=2
        assert!(!c.rising); // 2%2==0
        c.advance(); // step=3
        assert!(c.rising); // 3%2==1
    }

    #[test]
    fn reverse_rising_parity() {
        let mut c = Commutation::new();
        c.forward = false;
        c.step = 4;
        c.advance(); // step=3
        // C code: rising = !(step % 2) which is step%2==0
        assert_eq!(c.rising, c.step % 2 == 0);
    }

    #[test]
    fn forward_full_cycle() {
        let mut c = Commutation::new();
        c.step = 1;
        for expected in [2, 3, 4, 5, 6, 1] {
            c.advance();
            assert_eq!(c.step, expected);
        }
        assert!(c.desync_check); // set on wrap
    }

    #[test]
    fn reverse_full_cycle() {
        let mut c = Commutation::new();
        c.forward = false;
        c.step = 6;
        for expected in [5, 4, 3, 2, 1, 6] {
            c.advance();
            assert_eq!(c.step, expected);
        }
        assert!(c.desync_check);
    }

    #[test]
    fn desync_check_only_on_wrap() {
        let mut c = Commutation::new();
        c.step = 3;
        c.desync_check = false;
        c.advance(); // 3->4, no wrap
        assert!(!c.desync_check);
    }

    #[test]
    fn multiple_wraps_desync_resets() {
        let mut c = Commutation::new();
        c.step = 6;
        c.advance(); // wraps, desync_check = true
        assert!(c.desync_check);
        c.desync_check = false;
        // Another full cycle
        for _ in 0..6 {
            c.advance();
        }
        assert!(c.desync_check);
    }

    #[test]
    fn rising_alternates_forward() {
        let mut c = Commutation::new();
        c.step = 1;
        let mut risings = [false; 6];
        for i in 0..6 {
            c.advance();
            risings[i] = c.rising;
        }
        // Steps 2,3,4,5,6,1 -> rising: false,true,false,true,false,true
        assert_eq!(risings, [false, true, false, true, false, true]);
    }
}
