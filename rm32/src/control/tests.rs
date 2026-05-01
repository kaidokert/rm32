//! Integration tests for control loop — uses isr_logic path exclusively.
//!
//! Legacy MotorState/tick.rs tests have been removed. Behavioral coverage
//! is now provided by blackbox test vectors (tests/blackbox/vectors/).

#[cfg(test)]
mod tests {
    use crate::hal;

    // =================================================================
    // ISR logic tests (platform-independent, using TestShared + MockHal)
    // =================================================================

    use crate::control::isr_logic;
    use crate::control::shared_impl::TestShared;
    use crate::shared_comm::{IsrTiming as _, MainControl as _, MotorState as _, SharedComm as _};

    fn make_armed_timeout() -> u32 {
        0
    }

    struct MockPwm {
        last_duty: u16,
    }
    impl hal::PwmOutput for MockPwm {
        fn set_duty_all(&mut self, d: u16) {
            self.last_duty = d;
        }
        fn set_auto_reload(&mut self, _: u16) {}
        fn set_prescaler(&mut self, _: u16) {}
        fn set_compare1(&mut self, _: u16) {}
        fn set_compare2(&mut self, _: u16) {}
        fn set_compare3(&mut self, _: u16) {}
        fn generate_update_event(&mut self) {}
        fn set_dead_time_override(&mut self, _dtg: u16) {}
    }
    struct MockComp {
        level: bool,
        mask_called: bool,
    }
    impl hal::Comparator for MockComp {
        fn output_level(&self) -> bool {
            self.level
        }
        fn set_step(&mut self, _: u8, _: bool) {}
        fn change_input(&mut self) {}
        fn enable_interrupts(&mut self) {}
        fn mask_interrupts(&mut self) {
            self.mask_called = true;
        }
    }
    struct MockPhase;
    impl hal::PhaseOutput for MockPhase {
        fn com_step(&mut self, _: u8) {}
        fn all_off(&mut self) {}
        fn full_brake(&mut self) {}
        fn all_pwm(&mut self) {}
        fn proportional_brake(&mut self) {}
    }
    struct MockInterval {
        count: u32,
    }
    impl hal::IntervalTimer for MockInterval {
        fn count(&self) -> u32 {
            self.count
        }
        fn set_count(&mut self, v: u32) {
            self.count = v;
        }
    }
    struct MockComTimer;
    impl hal::ComTimer for MockComTimer {
        fn set_and_enable(&mut self, _: u16) {}
        fn disable_interrupt(&mut self) {}
        fn enable_interrupt(&mut self) {}
    }

    struct MockMotorHal {
        pwm: MockPwm,
        comp: MockComp,
        phase: MockPhase,
        interval: MockInterval,
        com_timer: MockComTimer,
    }
    impl hal::MotorHal for MockMotorHal {
        type Pwm = MockPwm;
        type Comp = MockComp;
        type Phase = MockPhase;
        type Interval = MockInterval;
        type Com = MockComTimer;

        fn pwm(&mut self) -> &mut MockPwm {
            &mut self.pwm
        }
        fn comp(&mut self) -> &mut MockComp {
            &mut self.comp
        }
        fn phase(&mut self) -> &mut MockPhase {
            &mut self.phase
        }
        fn interval(&mut self) -> &mut MockInterval {
            &mut self.interval
        }
        fn com_timer(&mut self) -> &mut MockComTimer {
            &mut self.com_timer
        }
    }
    impl MockMotorHal {
        fn new() -> Self {
            Self {
                pwm: MockPwm { last_duty: 0 },
                comp: MockComp {
                    level: false,
                    mask_called: false,
                },
                phase: MockPhase,
                interval: MockInterval { count: 0 },
                com_timer: MockComTimer,
            }
        }
    }

    #[test]
    fn isr_tick_throttle_maps_to_setpoint() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut armed_timeout = make_armed_timeout();
        let shared = TestShared::new();
        let mut hal = MockMotorHal::new();

        shared.mode.set(crate::motor_mode::MotorMode::Armed);
        shared.newinput.set(1000);
        shared.adjusted_input.set(1000);

        isr_logic::ten_khz_tick(&mut crate::control::context::MotorContext {
            commutation: &mut comm,
            bemf: &mut bemf,
            duty: &mut duty,
            config: &config,
            armed_timeout_count: &mut armed_timeout,
            voltage_based_ramp: false,
            shared: &shared,
            hal: &mut hal,
        });

        assert!(shared.duty_cycle_setpoint() > 0);
        assert_eq!(shared.adjusted_input(), 1000);
    }

    #[test]
    fn isr_tick_zero_throttle_no_setpoint() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut armed_timeout = make_armed_timeout();
        let shared = TestShared::new();
        let mut hal = MockMotorHal::new();

        shared.mode.set(crate::motor_mode::MotorMode::Armed);
        shared.newinput.set(0);

        isr_logic::ten_khz_tick(&mut crate::control::context::MotorContext {
            commutation: &mut comm,
            bemf: &mut bemf,
            duty: &mut duty,
            config: &config,
            armed_timeout_count: &mut armed_timeout,
            voltage_based_ramp: false,
            shared: &shared,
            hal: &mut hal,
        });

        assert_eq!(shared.duty_cycle_setpoint(), 0);
    }

    #[test]
    fn isr_tick_arming_sequence() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut armed_timeout = make_armed_timeout();
        let shared = TestShared::new();
        let mut hal = MockMotorHal::new();

        shared.input_set.set(true);
        shared.newinput.set(0);

        for _ in 0..20000 {
            isr_logic::ten_khz_tick(&mut crate::control::context::MotorContext {
                commutation: &mut comm,
                bemf: &mut bemf,
                duty: &mut duty,
                config: &config,
                armed_timeout_count: &mut armed_timeout,
                voltage_based_ramp: false,
                shared: &shared,
                hal: &mut hal,
            });
        }
        assert!(!shared.armed());

        isr_logic::ten_khz_tick(&mut crate::control::context::MotorContext {
            commutation: &mut comm,
            bemf: &mut bemf,
            duty: &mut duty,
            config: &config,
            armed_timeout_count: &mut armed_timeout,
            voltage_based_ramp: false,
            shared: &shared,
            hal: &mut hal,
        });
        assert!(shared.armed());
    }

    #[test]
    fn isr_tick_signal_timeout_increments() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut armed_timeout = make_armed_timeout();
        let shared = TestShared::new();
        let mut hal = MockMotorHal::new();

        isr_logic::ten_khz_tick(&mut crate::control::context::MotorContext {
            commutation: &mut comm,
            bemf: &mut bemf,
            duty: &mut duty,
            config: &config,
            armed_timeout_count: &mut armed_timeout,
            voltage_based_ramp: false,
            shared: &shared,
            hal: &mut hal,
        });

        assert_eq!(shared.signal_timeout(), 1);
    }

    #[test]
    fn isr_tick_ramp_limits_large_step() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut armed_timeout = make_armed_timeout();
        let shared = TestShared::new();
        let mut hal = MockMotorHal::new();

        shared.mode.set(crate::motor_mode::MotorMode::Armed);
        shared.newinput.set(2047);
        shared.adjusted_input.set(2047);
        duty.set_last(100);
        duty.set_ramp_divider(0);

        isr_logic::ten_khz_tick(&mut crate::control::context::MotorContext {
            commutation: &mut comm,
            bemf: &mut bemf,
            duty: &mut duty,
            config: &config,
            armed_timeout_count: &mut armed_timeout,
            voltage_based_ramp: false,
            shared: &shared,
            hal: &mut hal,
        });

        assert!(
            duty.cycle() < 2000,
            "duty should be ramp-limited, got {}",
            duty.cycle()
        );
        assert!(
            duty.cycle() > 100,
            "duty should increase from 100, got {}",
            duty.cycle()
        );
    }

    #[test]
    fn isr_commutation_timer_advances_step() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let shared = TestShared::new();
        let mut com_timer = MockComTimer;
        let mut comp = MockComp {
            level: false,
            mask_called: false,
        };
        let mut phase = MockPhase;

        let step_before = comm.step;
        isr_logic::commutation_timer_expired(
            &mut comm,
            &mut bemf,
            &shared,
            &mut com_timer,
            &mut comp,
            &mut phase,
        );

        assert_ne!(comm.step, step_before);
        assert_eq!(shared.zero_crosses(), 1);
        assert!(!bemf.zc_found());
    }

    #[test]
    fn isr_bemf_zero_cross_detected() {
        let comm = crate::commutation::Commutation::new(); // rising=true
        let mut bemf = crate::control::state::BemfState::default();
        let mut comp = MockComp {
            level: false,
            mask_called: false,
        };
        let mut interval = MockInterval { count: 500 };
        let mut com_timer = MockComTimer;

        bemf.set_filter_level(2);
        bemf.set_wait_time(500);

        isr_logic::bemf_zero_cross(&comm, &mut bemf, &mut comp, &mut interval, &mut com_timer);

        assert!(comp.mask_called);
    }

    #[test]
    fn isr_bemf_zero_cross_filtered_out() {
        let comm = crate::commutation::Commutation::new(); // rising=true
        let mut bemf = crate::control::state::BemfState::default();
        let mut comp = MockComp {
            level: true,
            mask_called: false,
        };
        let mut interval = MockInterval { count: 0 };
        let mut com_timer = MockComTimer;

        bemf.set_filter_level(2);

        isr_logic::bemf_zero_cross(&comm, &mut bemf, &mut comp, &mut interval, &mut com_timer);

        assert!(!comp.mask_called);
    }

    // =================================================================
    // Pure math tests (no MotorState dependency)
    // =================================================================

    #[test]
    fn current_scaling_formula() {
        let smoothed: u16 = 2048;
        let offset: i16 = 498;
        let mv_per_amp: u16 = 20;
        let current_mv = (smoothed as i32) * 3300 / 41 - (offset as i32) * 100;
        let actual_current = current_mv / mv_per_amp as i32;
        assert!(
            actual_current > 5700 && actual_current < 5800,
            "expected ~5750, got {}",
            actual_current
        );
    }
}
