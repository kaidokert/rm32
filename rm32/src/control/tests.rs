//! Integration tests for control loop methods.

#[cfg(test)]
mod tests {
    use crate::control::state::MotorState;
    use crate::control::tick::ControlHal;
    use crate::hal;

    /// Mock HAL for testing - records calls and returns configurable values.
    struct MockHal {
        pub comp_level: bool,
        pub timer_count: u32,
        pub last_duty_all: u16,
        pub duty_all_count: u32,
        pub last_com_step: u8,
        pub com_step_count: u32,
        pub all_off_called: bool,
        pub mask_called: bool,
        pub enable_comp_called: bool,
    }

    impl MockHal {
        fn new() -> Self {
            Self {
                comp_level: false,
                timer_count: 0,
                last_duty_all: 0,
                duty_all_count: 0,
                last_com_step: 0,
                com_step_count: 0,
                all_off_called: false,
                mask_called: false,
                enable_comp_called: false,
            }
        }
    }

    impl hal::PwmOutput for MockHal {
        fn set_duty_all(&mut self, duty: u16) { self.last_duty_all = duty; self.duty_all_count += 1; }
        fn set_auto_reload(&mut self, _arr: u16) {}
        fn set_prescaler(&mut self, _psc: u16) {}
        fn set_compare1(&mut self, _val: u16) {}
        fn set_compare2(&mut self, _val: u16) {}
        fn set_compare3(&mut self, _val: u16) {}
        fn generate_update_event(&mut self) {}
    }

    impl hal::Comparator for MockHal {
        fn output_level(&self) -> bool { self.comp_level }
        fn set_step(&mut self, _step: u8, _rising: bool) {}
        fn change_input(&mut self) {}
        fn enable_interrupts(&mut self) { self.enable_comp_called = true; }
        fn mask_interrupts(&mut self) { self.mask_called = true; }
    }

    impl hal::PhaseOutput for MockHal {
        fn com_step(&mut self, step: u8) { self.last_com_step = step; self.com_step_count += 1; }
        fn all_off(&mut self) { self.all_off_called = true; }
        fn full_brake(&mut self) {}
        fn all_pwm(&mut self) {}
        fn proportional_brake(&mut self) {}
    }

    impl hal::IntervalTimer for MockHal {
        fn count(&self) -> u32 { self.timer_count }
        fn set_count(&mut self, val: u32) { self.timer_count = val; }
    }

    impl hal::ComTimer for MockHal {
        fn set_and_enable(&mut self, _timeout: u16) {}
        fn disable_interrupt(&mut self) {}
        fn enable_interrupt(&mut self) {}
    }

    impl hal::System for MockHal {
        fn reset(&mut self) -> ! { panic!("reset called") }
        fn enable_irq(&mut self) {}
        fn disable_irq(&mut self) {}
        fn reload_watchdog(&mut self) {}
        fn delay_micros(&mut self, _us: u32) {}
        fn delay_millis(&mut self, _ms: u32) {}
    }

    // =========================================================
    // startMotor
    // =========================================================

    #[test]
    fn start_motor_when_stopped() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.running = false;

        state.start_motor(&mut hal);

        assert!(state.running);
        assert_eq!(state.timing.commutation_interval, 10000);
        assert!(hal.enable_comp_called);
    }

    #[test]
    fn start_motor_already_running() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.running = true;
        state.timing.commutation_interval = 500;

        state.start_motor(&mut hal);

        assert_eq!(state.timing.commutation_interval, 500); // unchanged
        assert_eq!(hal.com_step_count, 0); // no commutate
    }

    // =========================================================
    // commutate
    // =========================================================

    #[test]
    fn commutate_advances_step() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.commutation.step = 3;

        state.commutate(&mut hal);

        assert_eq!(state.commutation.step, 4);
        assert_eq!(hal.last_com_step, 4);
        assert_eq!(hal.com_step_count, 1);
    }

    #[test]
    fn commutate_skips_comstep_when_braking() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.prop_brake_active = true;

        state.commutate(&mut hal);

        assert_eq!(hal.com_step_count, 0);
    }

    #[test]
    fn commutate_clears_bemf() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.bemf.counter = 10;
        state.bemf.zc_found = true;

        state.commutate(&mut hal);

        assert_eq!(state.bemf.counter, 0);
        assert!(!state.bemf.zc_found);
    }

    // =========================================================
    // interruptRoutine
    // =========================================================

    #[test]
    fn interrupt_passes_when_comp_differs_from_rising() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.commutation.rising = true;
        hal.comp_level = false; // != rising -> passes filter
        state.bemf.filter_level = 3;
        hal.timer_count = 500;
        state.bemf.this_zc_time = 100;

        let accepted = state.interrupt_routine(&mut hal);

        assert!(accepted);
        assert_eq!(state.bemf.last_zc_time, 100);
        assert_eq!(state.bemf.this_zc_time, 500);
    }

    #[test]
    fn interrupt_rejects_false_alarm() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.commutation.rising = true;
        hal.comp_level = true; // == rising -> reject
        state.bemf.filter_level = 1;

        let accepted = state.interrupt_routine(&mut hal);

        assert!(!accepted);
        assert!(!hal.mask_called);
    }

    #[test]
    fn interrupt_filter_level_0_always_passes() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.bemf.filter_level = 0;
        hal.timer_count = 1000;

        let accepted = state.interrupt_routine(&mut hal);

        assert!(accepted);
        assert_eq!(state.bemf.this_zc_time, 1000);
    }

    // =========================================================
    // PeriodElapsedCallback
    // =========================================================

    #[test]
    fn period_elapsed_increments_zero_crosses() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.timing.zero_crosses = 0;
        state.timing.commutation_interval = 2000;

        state.period_elapsed_callback(&mut hal);

        assert_eq!(state.timing.zero_crosses, 1);
    }

    #[test]
    fn period_elapsed_caps_at_10000() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.timing.zero_crosses = 10000;
        state.timing.commutation_interval = 2000;

        state.period_elapsed_callback(&mut hal);

        assert_eq!(state.timing.zero_crosses, 10000);
    }

    #[test]
    fn period_elapsed_enables_comp_when_not_old_routine() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.old_routine = false;
        state.timing.commutation_interval = 2000;

        state.period_elapsed_callback(&mut hal);

        assert!(hal.enable_comp_called);
    }

    #[test]
    fn period_elapsed_skips_comp_when_old_routine() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.old_routine = true;
        state.timing.commutation_interval = 2000;

        state.period_elapsed_callback(&mut hal);

        assert!(!hal.enable_comp_called);
    }

    // =========================================================
    // setInput
    // =========================================================

    #[test]
    fn set_input_unidirectional_passthrough() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 1000;
        state.armed = true;

        state.set_input(&mut hal);

        assert_eq!(state.input.adjusted, 1000);
        assert_eq!(state.input.input, 1000);
    }

    #[test]
    fn set_input_bidir_dshot_forward() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.input.dshot = true;
        state.input.newinput = 1200;
        state.armed = true;

        state.set_input(&mut hal);

        // (1200 - 1048) * 2 + 47 - 1 = 350
        assert_eq!(state.input.adjusted, 350);
    }

    #[test]
    fn set_input_bidir_dshot_zero() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.input.dshot = true;
        state.input.newinput = 10;
        state.armed = true;

        state.set_input(&mut hal);

        assert_eq!(state.input.adjusted, 0);
    }

    #[test]
    fn set_input_bemf_timeout_cuts_throttle() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.stuck_rotor_protection = 1;
        state.protection.bemf_timeout_happened = 20;
        state.protection.bemf_timeout = 10;
        state.input.newinput = 500;
        state.armed = true;

        state.set_input(&mut hal);

        assert_eq!(state.input.input, 0);
        assert!(hal.all_off_called);
    }

    // =========================================================
    // tenKhzTick
    // =========================================================

    #[test]
    fn ten_khz_increments_counters() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();

        state.ten_khz_tick(&mut hal);

        assert_eq!(state.ten_khz_counter, 1);
        assert_eq!(state.input.signal_timeout, 1);
    }

    #[test]
    fn ten_khz_ramp_limits_increase() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.armed = true;
        state.running = true;
        state.input.input = 500;
        state.duty.setpoint = 1000;
        state.duty.last = 500;
        state.duty.max_ramp_high_rpm = 16;
        state.timing.average_interval = 200;
        state.timing.zero_crosses = 200;

        state.ten_khz_tick(&mut hal);

        assert_eq!(state.duty.cycle, 516); // 500 + 16
    }

    #[test]
    fn ten_khz_arms_after_timeout() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.armed = false;
        state.input.input_set = true;
        state.input.adjusted = 0;
        state.armed_timeout_count = 20000;
        state.input.zero_input_count = 31;

        state.ten_khz_tick(&mut hal);

        assert!(state.armed);
    }

    // =========================================================
    // mainLoopTick
    // =========================================================

    #[test]
    fn main_loop_calculates_e_com_time() {
        let mut state = MotorState::default();
        state.timing.commutation_intervals = [100, 200, 300, 100, 200, 300];

        state.main_loop_tick();

        assert_eq!(state.timing.e_com_time, 602); // (1200+4)>>1
    }

    #[test]
    fn main_loop_clears_bemf_timeout() {
        let mut state = MotorState::default();
        state.timing.zero_crosses = 1001;
        state.protection.bemf_timeout_happened = 50;

        state.main_loop_tick();

        assert_eq!(state.protection.bemf_timeout_happened, 0);
    }

    #[test]
    fn main_loop_signal_timeout_disarms() {
        let mut state = MotorState::default();
        state.armed = true;
        state.input.signal_timeout = 10001;

        state.main_loop_tick();

        assert!(!state.armed);
    }

    #[test]
    fn main_loop_desync_detection() {
        let mut state = MotorState::default();
        state.commutation.desync_check = true;
        state.timing.zero_crosses = 20;
        state.timing.commutation_intervals = [100, 100, 100, 100, 100, 100];
        state.timing.last_average_interval = 5000; // wildly different
        state.timing.commutation_interval = 2000;
        state.input.input = 100;

        state.main_loop_tick();

        assert_eq!(state.timing.zero_crosses, 0);
        assert_eq!(state.protection.desync_happened, 1);
        assert!(!state.running);
        assert!(state.old_routine);
    }

    #[test]
    fn main_loop_erpm_calculation() {
        let mut state = MotorState::default();
        state.running = true;
        state.timing.commutation_intervals = [333, 333, 333, 333, 333, 333];

        state.main_loop_tick();

        // e_com_time = (1998+4)>>1 = 1001
        // e_rpm = 600000/1001 = 599
        assert!(state.timing.e_rpm > 0);
    }

    // =========================================================
    // setInput — extended branches
    // =========================================================

    #[test]
    fn set_input_zero_input() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 0;
        state.armed = true;
        state.set_input(&mut hal);
        assert_eq!(state.input.adjusted, 0);
        assert_eq!(state.input.input, 0);
    }

    #[test]
    fn set_input_starts_motor() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 500;
        state.armed = true;
        state.running = false;
        state.set_input(&mut hal);
        assert!(state.running);
        assert!(state.duty.setpoint >= state.duty.minimum);
    }

    #[test]
    fn set_input_not_armed_no_start() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 500;
        state.armed = false;
        state.set_input(&mut hal);
        assert!(!state.running);
    }

    #[test]
    fn set_input_bidir_dshot_reverse() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.input.dshot = true;
        state.commutation.forward = false;
        state.input.newinput = 500; // reverse range
        state.armed = true;
        state.set_input(&mut hal);
        // (500-48)*2+47-1 = 950
        assert_eq!(state.input.adjusted, 950);
    }

    #[test]
    fn set_input_bidir_dshot_reverses_when_slow() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.dir_reversed = 0;
        state.input.dshot = true;
        state.commutation.forward = false; // currently reverse
        state.input.newinput = 1200; // requesting forward
        state.timing.commutation_interval = 10000; // slow
        state.duty.cycle = 100; // < 200
        state.reverse_speed_threshold = 500;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(state.commutation.forward); // changed
        assert_eq!(state.timing.zero_crosses, 0);
    }

    #[test]
    fn set_input_clamps_duty_max() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 2047;
        state.armed = true;
        state.duty.maximum = 500;
        state.set_input(&mut hal);
        assert!(state.duty.setpoint <= 500);
    }

    #[test]
    fn set_input_startup_duty_floor() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 48;
        state.armed = true;
        state.running = false;
        state.timing.zero_crosses = 0;
        state.duty.min_startup = 200;
        state.set_input(&mut hal);
        assert!(state.duty.setpoint >= state.duty.min_startup);
    }

    #[test]
    fn set_input_sine_start_maps_input() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.use_sine_start = 1;
        state.config.sine_mode_changeover_throttle_level = 10;
        state.input.newinput = 100;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(state.input.input >= 47 && state.input.input <= 160);
    }

    #[test]
    fn set_input_sine_start_dead_band() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.use_sine_start = 1;
        state.config.sine_mode_changeover_throttle_level = 10;
        state.input.newinput = 20;
        state.armed = true;
        state.set_input(&mut hal);
        assert_eq!(state.input.input, 0);
    }

    #[test]
    fn set_input_plays_tone_flag_consumed() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.input.newinput = 0;
        state.armed = true;
        state.play_tone_flag = 3;
        state.set_input(&mut hal);
        assert_eq!(state.play_tone_flag, 0);
    }

    #[test]
    fn set_input_rc_car_servo_brakes_wrong_direction() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 1;
        state.config.dir_reversed = 0;
        state.config.servo_dead_band = 50;
        state.input.dshot = false;
        state.commutation.forward = false; // wrong for forward input
        state.input.newinput = 1200; // > 1000 + 100
        state.armed = true;
        state.set_input(&mut hal);
        assert!(state.prop_brake_active);
        assert_eq!(state.input.adjusted, 0);
    }

    #[test]
    fn set_input_rc_car_servo_return_to_center() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 1;
        state.config.servo_dead_band = 50;
        state.input.dshot = false;
        state.input.newinput = 1000; // dead band
        state.prop_brake_active = true;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(!state.prop_brake_active);
        assert!(state.return_to_center);
    }

    #[test]
    fn set_input_rc_car_dshot_forward() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 1;
        state.config.dir_reversed = 0;
        state.input.dshot = true;
        state.commutation.forward = true;
        state.input.newinput = 1200;
        state.armed = true;
        state.set_input(&mut hal);
        assert_eq!(state.input.adjusted, 350); // (1200-1048)*2+47-1
    }

    #[test]
    fn set_input_rc_car_dshot_zero_clears_brake() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 1;
        state.input.dshot = true;
        state.input.newinput = 10;
        state.prop_brake_active = true;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(!state.prop_brake_active);
        assert!(state.return_to_center);
        assert_eq!(state.input.adjusted, 0);
    }

    #[test]
    fn set_input_comp_pwm_drag_brake() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.comp_pwm = 1;
        state.config.brake_on_stop = 1;
        state.config.drag_brake_strength = 5;
        state.input.newinput = 0;
        state.armed = true;
        state.running = false;
        state.set_input(&mut hal);
        assert_eq!(state.duty.setpoint, 0);
        assert!(state.prop_brake_active);
    }

    #[test]
    fn set_input_comp_pwm_sine_enters_stepper() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.comp_pwm = 1;
        state.config.use_sine_start = 1;
        state.config.sine_mode_changeover_throttle_level = 10;
        state.input.newinput = 0;
        state.armed = true;
        state.running = false;
        state.stepper_sine = false;
        state.set_input(&mut hal);
        assert!(state.stepper_sine);
    }

    #[test]
    fn set_input_servo_bidir_blocks_reverse_at_speed() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 0;
        state.config.dir_reversed = 0;
        state.config.servo_dead_band = 50;
        state.input.dshot = false;
        state.commutation.forward = false;
        state.input.newinput = 1200;
        state.timing.commutation_interval = 100; // too fast
        state.duty.cycle = 500; // >= 200
        state.reverse_speed_threshold = 500;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(!state.commutation.forward); // NOT changed
    }

    #[test]
    fn set_input_low_input_brake_on_stop() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.brake_on_stop = 1;
        state.input.newinput = 0;
        state.armed = true;
        state.running = false;
        state.set_input(&mut hal);
        assert_eq!(state.input.input, 0);
        assert_eq!(state.duty.setpoint, 0);
    }

    #[test]
    fn set_input_rc_car_prop_brake_duty() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 1;
        state.config.servo_dead_band = 50;
        state.config.dir_reversed = 0;
        state.input.dshot = false;
        state.commutation.forward = false;
        state.input.newinput = 1200;
        state.prop_brake_active = true;
        state.return_to_center = false;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(state.prop_brake_active);
    }

    // =========================================================
    // tenKhzTick — expanded
    // =========================================================

    #[test]
    fn ten_khz_does_not_arm_wrong_input() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.armed = false;
        state.input.input_set = true;
        state.input.adjusted = 100; // nonzero
        state.armed_timeout_count = 50000;
        state.ten_khz_tick(&mut hal);
        assert!(!state.armed);
        assert_eq!(state.armed_timeout_count, 0);
    }

    #[test]
    fn ten_khz_telemetry_interval() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.telemetry_on_interval = 1;
        state.telemetry_interval_ms = 30;
        // threshold = (30 - 1 + 1) * 20 = 600
        state.telemetry.ms_count = 600;
        state.ten_khz_tick(&mut hal);
        // ms_count was > threshold, so send_telemetry set and ms_count reset
        // But main_loop_tick clears send_telemetry... here we only call ten_khz_tick
        assert!(state.telemetry.send_telemetry);
        assert_eq!(state.telemetry.ms_count, 0);
    }

    #[test]
    fn ten_khz_ramp_decrease() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.armed = true;
        state.running = true;
        state.input.input = 500;
        state.duty.setpoint = 100;
        state.duty.last = 500;
        state.duty.max_ramp_high_rpm = 16;
        state.timing.average_interval = 200;
        state.timing.zero_crosses = 200;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.cycle, 484); // 500 - 16
    }

    #[test]
    fn ten_khz_holds_duty_when_ramp_below_divider() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.duty.ramp_divider = 9;
        state.duty.setpoint = 1000;
        state.duty.last = 500;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.cycle, 500);
    }

    #[test]
    fn ten_khz_duty_cycle_from_setpoint() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.duty.setpoint = 1000;
        state.duty.last = 1000;
        state.duty.ramp_divider = 9;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.cycle, 1000);
    }

    #[test]
    fn ten_khz_adjusted_duty_when_running() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.duty.ramp_divider = 9;
        state.duty.setpoint = 1000;
        state.duty.last = 1000;
        state.armed = true;
        state.running = true;
        state.input.input = 500;
        state.tim1_arr = 1999;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.adjusted, 1000); // (1000*1999/2000)+1
    }

    #[test]
    fn ten_khz_prop_brake_duty() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.duty.ramp_divider = 9;
        state.duty.setpoint = 0;
        state.duty.last = 0;
        state.armed = true;
        state.running = false;
        state.input.input = 0;
        state.prop_brake_active = true;
        state.prop_brake_duty_cycle = 1000;
        state.tim1_arr = 1999;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.adjusted, 1000); // 1999 - (1000*1999/2000)
    }

    #[test]
    fn ten_khz_startup_ramp_rate() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.timing.zero_crosses = 50;
        state.duty.last = 100;
        state.duty.setpoint = 500;
        state.duty.max_ramp_startup = 2;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.cycle, 102); // 100 + 2
    }

    #[test]
    fn ten_khz_high_rpm_ramp_rate() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.timing.zero_crosses = 200;
        state.duty.last = 500;
        state.duty.setpoint = 1000;
        state.timing.average_interval = 200;
        state.duty.max_ramp_high_rpm = 16;
        state.ten_khz_tick(&mut hal);
        assert_eq!(state.duty.cycle, 516); // 500 + 16
    }

    #[test]
    fn ten_khz_stall_protection_pid() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.one_khz_loop_counter = 21;
        state.config.stall_protection = 1;
        state.running = true;
        state.timing.commutation_interval = 8000;
        state.pid.stall.kp = 1;
        state.pid.stall.output_limit = 50000;
        state.pid.stall.integral_limit = 10000;
        state.pid.stall_adjust = 0;
        state.ten_khz_tick(&mut hal);
        assert!(state.pid.stall_adjust > 0);
    }

    #[test]
    fn ten_khz_current_limit_pid() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.one_khz_loop_counter = 21;
        state.pid.use_current_limit = true;
        state.running = true;
        state.measurements.actual_current = 5000;
        state.config.current_limit = 10;
        state.pid.current.kp = 100;
        state.pid.current.output_limit = 50000;
        state.pid.current.integral_limit = 10000;
        state.pid.current_limit_adjust = 2000;
        state.ten_khz_tick(&mut hal);
        assert!(state.pid.current_limit_adjust < 2000);
    }

    // =========================================================
    // mainLoopTick — expanded
    // =========================================================

    #[test]
    fn main_loop_bemf_counts_startup_bidir() {
        let mut state = MotorState::default();
        state.timing.zero_crosses = 3;
        state.config.bi_direction = 1;
        state.main_loop_tick();
        assert_eq!(state.bemf.min_counts_up, 3); // TARGET + 1
    }

    #[test]
    fn main_loop_bemf_counts_normal() {
        let mut state = MotorState::default();
        state.timing.zero_crosses = 100;
        state.main_loop_tick();
        assert_eq!(state.bemf.min_counts_up, 2);
    }

    #[test]
    fn main_loop_variable_pwm_maps() {
        let mut state = MotorState::default();
        state.config.variable_pwm = 1;
        state.timing.commutation_interval = 150;
        state.timer1_max_arr = 1999;
        state.main_loop_tick();
        assert!(state.tim1_arr >= state.timer1_max_arr / 2);
        assert!(state.tim1_arr <= state.timer1_max_arr);
    }

    #[test]
    fn main_loop_consumed_current() {
        let mut state = MotorState::default();
        state.ten_khz_counter = 20001;
        state.measurements.actual_current = 1000;
        state.measurements.consumed_current = 0;
        state.main_loop_tick();
        assert!(state.measurements.consumed_current > 0);
        assert_eq!(state.ten_khz_counter, 0);
    }

    #[test]
    fn main_loop_bemf_timeout_clear_zero_input() {
        let mut state = MotorState::default();
        state.input.adjusted = 0;
        state.protection.bemf_timeout_happened = 50;
        state.main_loop_tick();
        assert_eq!(state.protection.bemf_timeout_happened, 0);
    }

    #[test]
    fn main_loop_esc_info_flag_cleared() {
        let mut state = MotorState::default();
        state.telemetry.send_esc_info = true;
        state.main_loop_tick();
        assert!(!state.telemetry.send_esc_info);
    }

    #[test]
    fn main_loop_temperature_limits_duty() {
        let mut state = MotorState::default();
        state.running = true;
        state.config.temperature_limit = 80;
        state.measurements.degrees_celsius = 85;
        state.timing.commutation_intervals = [333; 6];
        state.main_loop_tick();
        assert!(state.duty.maximum < 2000);
    }

    #[test]
    fn main_loop_filter_high_during_startup() {
        let mut state = MotorState::default();
        state.timing.zero_crosses = 50;
        state.timing.commutation_interval = 600;
        state.timing.commutation_intervals = [333; 6];
        state.main_loop_tick();
        assert_eq!(state.bemf.filter_level, 12);
    }

    #[test]
    fn main_loop_filter_low_at_speed() {
        let mut state = MotorState::default();
        state.timing.zero_crosses = 200;
        state.timing.commutation_interval = 30;
        state.timing.commutation_intervals = [20; 6];
        state.main_loop_tick();
        assert_eq!(state.bemf.filter_level, 2);
    }

    #[test]
    fn set_input_servo_bidir_allows_reverse_when_slow() {
        let mut state = MotorState::default();
        let mut hal = MockHal::new();
        state.config.bi_direction = 1;
        state.config.rc_car_reverse = 0;
        state.config.dir_reversed = 0;
        state.config.servo_dead_band = 50;
        state.input.dshot = false;
        state.commutation.forward = false;
        state.input.newinput = 1200;
        state.timing.commutation_interval = 10000; // slow
        state.duty.cycle = 100; // < 200
        state.reverse_speed_threshold = 500;
        state.armed = true;
        state.set_input(&mut hal);
        assert!(state.commutation.forward); // changed
        assert_eq!(state.timing.zero_crosses, 0);
        assert!(hal.mask_called);
    }

    // =================================================================
    // ISR logic tests (platform-independent, using TestShared + MockHal)
    // =================================================================

    use crate::control::isr_logic::{self, TickCounters};
    use crate::control::shared_impl::TestShared;
    use crate::shared_comm::SharedComm as _;

    fn make_counters() -> TickCounters {
        TickCounters { ten_khz_counter: 0, one_khz_loop_counter: 0, armed_timeout_count: 0, tim1_arr: 1999 }
    }

    // Separate mocks for isr_logic (needs 4 distinct &mut references)
    struct MockPwm { last_duty: u16 }
    impl hal::PwmOutput for MockPwm {
        fn set_duty_all(&mut self, d: u16) { self.last_duty = d; }
        fn set_auto_reload(&mut self, _: u16) {}
        fn set_prescaler(&mut self, _: u16) {}
        fn set_compare1(&mut self, _: u16) {}
        fn set_compare2(&mut self, _: u16) {}
        fn set_compare3(&mut self, _: u16) {}
        fn generate_update_event(&mut self) {}
    }
    struct MockComp { level: bool, mask_called: bool }
    impl hal::Comparator for MockComp {
        fn output_level(&self) -> bool { self.level }
        fn set_step(&mut self, _: u8, _: bool) {}
        fn change_input(&mut self) {}
        fn enable_interrupts(&mut self) {}
        fn mask_interrupts(&mut self) { self.mask_called = true; }
    }
    struct MockPhase;
    impl hal::PhaseOutput for MockPhase {
        fn com_step(&mut self, _: u8) {}
        fn all_off(&mut self) {}
        fn full_brake(&mut self) {}
        fn all_pwm(&mut self) {}
        fn proportional_brake(&mut self) {}
    }
    struct MockInterval { count: u32 }
    impl hal::IntervalTimer for MockInterval {
        fn count(&self) -> u32 { self.count }
        fn set_count(&mut self, v: u32) { self.count = v; }
    }
    struct MockComTimer;
    impl hal::ComTimer for MockComTimer {
        fn set_and_enable(&mut self, _: u16) {}
        fn disable_interrupt(&mut self) {}
        fn enable_interrupt(&mut self) {}
    }

    #[test]
    fn isr_tick_throttle_maps_to_setpoint() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut counters = make_counters();
        let shared = TestShared::new();
        let mut pwm = MockPwm { last_duty: 0 };
        let mut comp = MockComp { level: false, mask_called: false };
        let mut phase = MockPhase;
        let mut interval = MockInterval { count: 0 };

        shared.armed.set(true);
        shared.newinput.set(1000);

        isr_logic::ten_khz_tick(
            &mut comm, &mut bemf, &mut duty, &config, &mut counters,
            &shared, &mut pwm, &mut comp, &mut phase, &mut interval,
        );

        assert!(shared.duty_cycle_setpoint() > 0);
        assert_eq!(shared.adjusted_input(), 1000);
    }

    #[test]
    fn isr_tick_zero_throttle_no_setpoint() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut counters = make_counters();
        let shared = TestShared::new();
        let mut pwm = MockPwm { last_duty: 0 };
        let mut comp = MockComp { level: false, mask_called: false };
        let mut phase = MockPhase;
        let mut interval = MockInterval { count: 0 };

        shared.armed.set(true);
        shared.newinput.set(0);

        isr_logic::ten_khz_tick(
            &mut comm, &mut bemf, &mut duty, &config, &mut counters,
            &shared, &mut pwm, &mut comp, &mut phase, &mut interval,
        );

        assert_eq!(shared.duty_cycle_setpoint(), 0);
    }

    #[test]
    fn isr_tick_arming_sequence() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut counters = make_counters();
        let shared = TestShared::new();
        let mut pwm = MockPwm { last_duty: 0 };
        let mut comp = MockComp { level: false, mask_called: false };
        let mut phase = MockPhase;
        let mut interval = MockInterval { count: 0 };

        shared.input_set.set(true);
        shared.newinput.set(0);

        for _ in 0..20000 {
            isr_logic::ten_khz_tick(
                &mut comm, &mut bemf, &mut duty, &config, &mut counters,
                &shared, &mut pwm, &mut comp, &mut phase, &mut interval,
            );
        }
        assert!(!shared.armed());

        isr_logic::ten_khz_tick(
            &mut comm, &mut bemf, &mut duty, &config, &mut counters,
            &shared, &mut pwm, &mut comp, &mut phase, &mut interval,
        );
        assert!(shared.armed());
    }

    #[test]
    fn isr_tick_signal_timeout_increments() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut counters = make_counters();
        let shared = TestShared::new();
        let mut pwm = MockPwm { last_duty: 0 };
        let mut comp = MockComp { level: false, mask_called: false };
        let mut phase = MockPhase;
        let mut interval = MockInterval { count: 0 };

        isr_logic::ten_khz_tick(
            &mut comm, &mut bemf, &mut duty, &config, &mut counters,
            &shared, &mut pwm, &mut comp, &mut phase, &mut interval,
        );

        assert_eq!(shared.signal_timeout(), 1);
    }

    #[test]
    fn isr_tick_ramp_limits_large_step() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let mut duty = crate::control::state::DutyState::default();
        let config = crate::config::EepromConfig::default();
        let mut counters = make_counters();
        let shared = TestShared::new();
        let mut pwm = MockPwm { last_duty: 0 };
        let mut comp = MockComp { level: false, mask_called: false };
        let mut phase = MockPhase;
        let mut interval = MockInterval { count: 0 };

        shared.armed.set(true);
        shared.newinput.set(2047);
        duty.last = 100;
        duty.ramp_divider = 0;

        isr_logic::ten_khz_tick(
            &mut comm, &mut bemf, &mut duty, &config, &mut counters,
            &shared, &mut pwm, &mut comp, &mut phase, &mut interval,
        );

        assert!(duty.cycle < 2000, "duty should be ramp-limited, got {}", duty.cycle);
        assert!(duty.cycle > 100, "duty should increase from 100, got {}", duty.cycle);
    }

    #[test]
    fn isr_commutation_timer_advances_step() {
        let mut comm = crate::commutation::Commutation::new();
        let mut bemf = crate::control::state::BemfState::default();
        let shared = TestShared::new();
        let mut com_timer = MockComTimer;
        let mut comp = MockComp { level: false, mask_called: false };
        let mut phase = MockPhase;

        let step_before = comm.step;
        isr_logic::commutation_timer_expired(
            &mut comm, &mut bemf, &shared,
            &mut com_timer, &mut comp, &mut phase,
        );

        assert_ne!(comm.step, step_before);
        assert_eq!(shared.zero_crosses(), 1);
        assert!(!bemf.zc_found);
    }

    #[test]
    fn isr_bemf_zero_cross_detected() {
        let comm = crate::commutation::Commutation::new(); // rising=true
        let mut bemf = crate::control::state::BemfState::default();
        let mut comp = MockComp { level: false, mask_called: false };
        let mut interval = MockInterval { count: 500 };
        let mut com_timer = MockComTimer;

        bemf.filter_level = 2;
        bemf.wait_time = 500;
        // comp_level=false, rising=true → false != true → filter passes

        isr_logic::bemf_zero_cross(
            &comm, &mut bemf, &mut comp, &mut interval, &mut com_timer,
        );

        assert!(comp.mask_called);
    }

    #[test]
    fn isr_bemf_zero_cross_filtered_out() {
        let comm = crate::commutation::Commutation::new(); // rising=true
        let mut bemf = crate::control::state::BemfState::default();
        let mut comp = MockComp { level: true, mask_called: false };
        let mut interval = MockInterval { count: 0 };
        let mut com_timer = MockComTimer;

        bemf.filter_level = 2;
        // comp_level=true, rising=true → true == true → filter rejects (early return)

        isr_logic::bemf_zero_cross(
            &comm, &mut bemf, &mut comp, &mut interval, &mut com_timer,
        );

        assert!(!comp.mask_called);
    }

    // =================================================================
    // Variable PWM mode 2 tests
    // =================================================================

    #[test]
    fn variable_pwm_mode2_clamps_low() {
        let mut state = MotorState::default();
        state.config.variable_pwm = 2;
        state.cpu_mhz = 64;
        state.timing.average_interval = 50; // below 100 floor
        state.timer1_max_arr = 1999;
        state.main_loop_tick();
        // scale = cpu_mhz/9 = 64/9 = 7, then 100 * 7 = 700
        assert_eq!(state.tim1_arr, 700);
    }

    #[test]
    fn variable_pwm_mode2_clamps_high() {
        let mut state = MotorState::default();
        state.config.variable_pwm = 2;
        state.cpu_mhz = 64;
        state.timing.average_interval = 300; // above 250 ceiling
        state.main_loop_tick();
        assert_eq!(state.tim1_arr, 250 * (64 / 9));
    }

    #[test]
    fn variable_pwm_mode2_scales_mid() {
        let mut state = MotorState::default();
        state.config.variable_pwm = 2;
        state.cpu_mhz = 64;
        state.timing.average_interval = 150;
        state.main_loop_tick();
        assert_eq!(state.tim1_arr, 150 * (64 / 9));
    }

    #[test]
    fn variable_pwm_mode0_unchanged() {
        let mut state = MotorState::default();
        state.config.variable_pwm = 0;
        state.tim1_arr = 1999;
        state.timing.average_interval = 150;
        state.main_loop_tick();
        assert_eq!(state.tim1_arr, 1999);
    }

    // =================================================================
    // Current scaling test
    // =================================================================

    #[test]
    fn current_scaling_formula() {
        // C formula: actual_current = ((smoothed * 3300/41) - (CURRENT_OFFSET * 100)) / MILLIVOLT_PER_AMP
        let smoothed: u16 = 2048; // mid-range ADC
        let offset: i16 = 498;
        let mv_per_amp: u16 = 20;
        let current_mv = (smoothed as i32) * 3300 / 41 - (offset as i32) * 100;
        let actual_current = current_mv / mv_per_amp as i32;
        // (2048 * 3300 / 41) = 164878 (integer), - 49800 = 115078
        // 115078 / 20 = 5753 (integer)
        assert!(actual_current > 5700 && actual_current < 5800,
            "expected ~5750, got {}", actual_current);
    }
}
