//! 6-step BLDC commutation logic.

/// Commutation state
pub struct Commutation {
    pub(crate) step: u8, // 1-6
    pub(crate) forward: bool,
    pub(crate) rising: bool,
    pub(crate) desync_check: bool,
    /// Per-step commutation intervals for e_com_time averaging.
    /// Written by ISR on each step advance via record_interval().
    intervals: [u16; 8],
    /// Sum of `intervals` (slots 6 and 7 stay 0).
    interval_sum: u32,
}

impl Commutation {
    pub fn new() -> Self {
        Self {
            step: 1,
            forward: true,
            rising: true,
            desync_check: false,
            intervals: [0; 8],
            interval_sum: 0,
        }
    }

    /// Record the current commutation interval for this step and recompute e_com_time.
    /// Called by ISR after each step advance, matching C:
    /// `commutation_intervals[step - 1] = commutation_interval`
    ///
    /// Returns the updated e_com_time for publishing to SharedComm.
    #[inline(always)]
    pub(crate) fn record_interval<P: crate::platform::Platform>(
        &mut self,
        commutation_interval: u16,
    ) -> i32 {
        // Running sum of the six slots avoids re-summing all six elements on
        // every priority-0 commutation ISR entry. `step` is 1..=6 by
        // construction; `& 7` and the 8-slot array keep the index in range
        // without a bounds-check panic path on the ISR hot path.
        debug_assert!((1..=6).contains(&self.step), "step {}", self.step);
        let k = (self.step.wrapping_sub(1) & 7) as usize;
        let old = self.intervals[k];
        self.intervals[k] = commutation_interval;
        self.interval_sum = self.interval_sum - old as u32 + commutation_interval as u32;
        crate::platform::six_intervals_to_us::<P>(self.interval_sum)
    }

    /// Advance one commutation step. Returns the new step number.
    /// Sets `desync_check` on step wrap.
    #[inline(always)]
    pub(crate) fn advance(&mut self) -> u8 {
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

impl Commutation {
    /// Read current step (1-6).
    pub fn step(&self) -> u8 {
        self.step
    }

    /// Set current step (clamped to valid six-step sector `1..=6`).
    pub fn set_step(&mut self, v: u8) {
        let v = v.clamp(1, 6);
        self.step = v;
        self.rising = if self.forward {
            !v.is_multiple_of(2)
        } else {
            v.is_multiple_of(2)
        };
    }

    /// Read forward direction flag.
    pub fn forward(&self) -> bool {
        self.forward
    }

    /// Set forward direction flag.
    pub fn set_forward(&mut self, v: bool) {
        self.forward = v;
    }

    /// Read rising edge flag.
    pub fn rising(&self) -> bool {
        self.rising
    }

    /// Read desync_check flag.
    pub fn desync_check(&self) -> bool {
        self.desync_check
    }

    /// Set desync_check flag.
    pub fn set_desync_check(&mut self, v: bool) {
        self.desync_check = v;
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

    #[test]
    fn record_interval_running_sum_matches_six_slot_sum_across_wraps() {
        use crate::platform::TestPlatform;
        let mut c = Commutation::new();
        let mut ref_slots = [0u16; 6];
        let samples = [
            120u16, 340, 560, 780, 910, 1024, 65535, 0, 42, 999, 12345, 54321, 17, 888,
        ];
        for (idx, &sample) in samples.iter().enumerate() {
            if idx == 7 {
                c.set_forward(false);
            }
            c.advance();
            ref_slots[(c.step() - 1) as usize] = sample;
            let expected_sum: u32 = ref_slots.iter().map(|&v| v as u32).sum();
            let got = c.record_interval::<TestPlatform>(sample);
            let expected = crate::platform::six_intervals_to_us::<TestPlatform>(expected_sum);
            assert_eq!(got, expected, "idx={idx}, step={}", c.step());
        }

        // Out-of-range set_step inputs clamp to 1..=6 so slots 6..8 are never touched.
        c.set_step(0);
        assert_eq!(c.step(), 1);
        ref_slots[0] = 500;
        let expected_sum: u32 = ref_slots.iter().map(|&v| v as u32).sum();
        assert_eq!(
            c.record_interval::<TestPlatform>(500),
            crate::platform::six_intervals_to_us::<TestPlatform>(expected_sum)
        );

        c.set_step(7);
        assert_eq!(c.step(), 6);
        ref_slots[5] = 700;
        let expected_sum: u32 = ref_slots.iter().map(|&v| v as u32).sum();
        assert_eq!(
            c.record_interval::<TestPlatform>(700),
            crate::platform::six_intervals_to_us::<TestPlatform>(expected_sum)
        );
    }
}
