//! BEMF zero-cross detection logic.

use crate::control::state::BemfState;

impl BemfState {
    /// Process one comparator sample. `comp_level` is the raw comparator output
    /// (true = high). The polarity is inverted internally (matches C: `!getCompOutputLevel()`).
    /// `rising` indicates the expected zero-cross direction.
    pub fn update(&mut self, comp_level: bool, rising: bool) {
        let current_state = !comp_level; // polarity reversed, matches C

        if rising {
            if current_state {
                self.counter += 1;
            } else {
                self.bad_count += 1;
                if self.bad_count > self.bad_count_threshold {
                    self.counter = 0;
                }
            }
        } else if !current_state {
            self.counter += 1;
        } else {
            self.bad_count += 1;
            if self.bad_count > self.bad_count_threshold {
                self.counter = 0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_bemf() -> BemfState {
        BemfState {
            counter: 0,
            zc_found: false,
            min_counts_up: 2,
            min_counts_down: 2,
            bad_count: 0,
            bad_count_threshold: 10,
            filter_level: 5,
            wait_time: 0,
            last_zc_time: 0,
            this_zc_time: 0,
            advance: 0,
            temp_advance: 0,
            auto_advance_level: 0,
        }
    }

    #[test]
    fn increments_counter_on_correct_polarity_rising() {
        let mut b = new_bemf();
        // rising=true, comp_level=false -> !false=true=current_state -> counter++
        b.update(false, true);
        assert_eq!(b.counter, 1);
        b.update(false, true);
        assert_eq!(b.counter, 2);
    }

    #[test]
    fn increments_bad_count_on_wrong_polarity_rising() {
        let mut b = new_bemf();
        b.counter = 5;
        // rising=true, comp_level=true -> !true=false=current_state -> bad_count++
        b.update(true, true);
        assert_eq!(b.bad_count, 1);
        assert_eq!(b.counter, 5); // unchanged
    }

    #[test]
    fn resets_counter_when_bad_count_exceeds_threshold() {
        let mut b = new_bemf();
        b.counter = 10;
        b.bad_count = 10; // at threshold
        b.bad_count_threshold = 10;

        // One more bad reading pushes over
        b.update(true, true);
        assert_eq!(b.bad_count, 11);
        assert_eq!(b.counter, 0); // reset!
    }

    #[test]
    fn falling_edge_correct_polarity() {
        let mut b = new_bemf();
        // rising=false, comp_level=true -> !true=false=current_state -> !false -> counter++
        // Wait: rising=false, if !current_state -> counter++
        // current_state = !comp_level = !true = false
        // !current_state = !false = true -> counter++
        b.update(true, false);
        assert_eq!(b.counter, 1);
    }

    #[test]
    fn falling_edge_wrong_polarity() {
        let mut b = new_bemf();
        // rising=false, comp_level=false -> !false=true=current_state
        // !current_state = false -> bad_count++
        b.update(false, false);
        assert_eq!(b.bad_count, 1);
        assert_eq!(b.counter, 0);
    }

    #[test]
    fn bad_count_does_not_reset_below_threshold() {
        let mut b = new_bemf();
        b.counter = 5;
        b.bad_count = 5;
        b.bad_count_threshold = 10;

        b.update(true, true); // bad
        assert_eq!(b.bad_count, 6);
        assert_eq!(b.counter, 5); // NOT reset, still below threshold
    }
}
