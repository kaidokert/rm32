//! BEMF zero-cross detection tests.

#[cfg(test)]
mod tests {
    use crate::control::state::BemfState;

    #[test]
    fn increments_counter_on_correct_polarity_rising() {
        let mut b = BemfState::default();
        b.update(false, true);
        assert_eq!(b.counter(), 1);
        b.update(false, true);
        assert_eq!(b.counter(), 2);
    }

    #[test]
    fn increments_bad_count_on_wrong_polarity_rising() {
        let mut b = BemfState::default();
        for _ in 0..5 {
            b.update(false, true);
        }
        assert_eq!(b.counter(), 5);
        b.update(true, true);
        assert_eq!(b.bad_count(), 1);
        assert_eq!(b.counter(), 5);
    }

    #[test]
    fn resets_counter_when_bad_count_exceeds_threshold() {
        let mut b = BemfState::default();
        // Default bad_count_threshold=2
        for _ in 0..10 {
            b.update(false, true);
        }
        assert_eq!(b.counter(), 10);
        // 3 bad readings exceed threshold (2)
        b.update(true, true);
        b.update(true, true);
        b.update(true, true);
        assert_eq!(b.counter(), 0);
    }

    #[test]
    fn falling_edge_correct_polarity() {
        let mut b = BemfState::default();
        b.update(true, false);
        assert_eq!(b.counter(), 1);
    }

    #[test]
    fn falling_edge_wrong_polarity() {
        let mut b = BemfState::default();
        b.update(false, false);
        assert_eq!(b.bad_count(), 1);
        assert_eq!(b.counter(), 0);
    }

    #[test]
    fn bad_count_does_not_reset_below_threshold() {
        let mut b = BemfState::default();
        for _ in 0..5 {
            b.update(false, true);
        }
        b.update(true, true);
        assert_eq!(b.bad_count(), 1);
        assert_eq!(b.counter(), 5);
    }

    #[test]
    fn zero_cross_detected_after_threshold() {
        let mut b = BemfState::default();
        // Default min_counts_up=2, need counter > 2
        b.update(false, true);
        b.update(false, true);
        assert!(!b.zero_cross_detected(true)); // counter=2, not > 2
        b.update(false, true);
        assert!(b.zero_cross_detected(true)); // counter=3 > 2
    }

    #[test]
    fn record_zero_cross_returns_filtered_ci() {
        let mut b = BemfState::default();
        // new_ci = (this_zc_time + 3*ci) / 4
        let new_ci = b.record_zero_cross(1000, 2000);
        // = (1000 + 6000) / 4 = 1750
        assert_eq!(new_ci, 1750);
    }
}
