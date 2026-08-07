//! BEMF zero-cross detection tests.

#[cfg(test)]
mod tests {
    use crate::control::state::BemfState;

    const SATURATION_STEPS: usize = u8::MAX as usize + 10;

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
    fn bad_count_saturates_on_repeated_wrong_polarity() {
        let mut b = BemfState::default();
        for _ in 0..SATURATION_STEPS {
            b.update(false, false);
        }
        assert_eq!(b.bad_count(), u8::MAX);
        assert_eq!(b.counter(), 0);
    }

    #[test]
    fn rising_bad_count_saturates_on_repeated_wrong_polarity() {
        let mut b = BemfState::default();
        for _ in 0..SATURATION_STEPS {
            b.update(true, true);
        }
        assert_eq!(b.bad_count(), u8::MAX);
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

    #[test]
    fn record_zero_cross_clamps_wait_time_underflow() {
        let mut b = BemfState::default();
        b.set_temp_advance(u8::MAX);

        let new_ci = b.record_zero_cross(18_000, 18_000);

        assert_eq!(new_ci, 18_000);
        assert_eq!(b.com_timer_delay(), 1);
    }

    #[test]
    fn update_timing_from_timer_clamps_wait_time_underflow() {
        let mut b = BemfState::default();
        b.set_temp_advance(u8::MAX);
        b.record_zc_timing(18_000);
        b.record_zc_timing(18_000);

        let new_ci = b.update_timing_from_timer(18_000);

        assert_eq!(new_ci, 18_000);
        assert_eq!(b.com_timer_delay(), 1);
    }
}
