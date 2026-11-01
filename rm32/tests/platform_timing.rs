//! Physical-unit checks through the public main-loop and port APIs.
use rm32::{
    board::BoardConfig,
    constants::Timing,
    hal::{Adc, TelemetryUart},
    main_state::{ChipParams, MainState},
    motor_mode::MotorEvent,
    platform::{Platform, e_com_us_to_avg_ticks, six_intervals_to_us},
    shared_state::SharedState,
    system::SystemTick,
    units::{CentiAmps, DegreesCelsius},
};
use std::{cell::Cell, rc::Rc};

struct Port<const TICK: u32, const TIMER: u32>;
impl<const TICK: u32, const TIMER: u32> Platform for Port<TICK, TIMER> {
    const TICK_HZ: u32 = TICK;
    const TIMER_HZ: u32 = TIMER;
    const CAPTURE_HZ: u32 = 48_000_000;
    const GCR_SHIFT: u8 = 6;
    const PWM_BASE_PERIOD: u16 = 2399;
}
type Standard = Port<20_000, 2_000_000>;
type Alternate = Port<40_000, 1_000_000>;

#[test]
fn unsupported_clock_ranges_are_rejected_before_counter_truncation() {
    for validate in [
        rm32::platform::validate::<Port<0, 2_000_000>>,
        rm32::platform::validate::<Port<20_001, 2_000_000>>,
        rm32::platform::validate::<Port<231_000, 2_000_000>>,
        rm32::platform::validate::<Port<20_000, 0>>,
        rm32::platform::validate::<Port<20_000, 3_000_000>>,
    ] {
        assert!(std::panic::catch_unwind(validate).is_err());
    }
    rm32::platform::validate::<Port<230_000, 2_000_000>>();
    assert_eq!(
        rm32::constants::telemetry_interval_ticks::<Port<230_000, 2_000_000>>(255),
        65_320
    );
}

/// AM32 reverses only above `map(motor_kv, 300, 3000, 1000, 500)` 2 MHz
/// ticks (main.c:818, applied at EEPROM load); the 1500 initializer
/// (main.c:388) never survives boot.
#[test]
fn reversal_speed_gate_follows_motor_kv_for_dshot_and_servo() {
    fn reverse<P: Platform>(dshot: bool, motor_kv: u16, interval: u32) -> bool {
        let shared = SharedState::new::<P>();
        shared.transition(MotorEvent::Arm);
        shared.set_dshot(dshot);
        shared.set_newinput(500); // reverse half in both input encodings
        shared.set_duty_cycle(100);
        shared.set_commutation_interval(interval);
        let mut main = main_state::<P>();
        main.config.bi_direction = 1;
        main.config.rc_car_reverse = 0;
        main.set_motor_kv(motor_kv);
        SystemTick::<P>::new().tick_input(&shared, &mut main);
        !shared.forward()
    }
    // (kv, AM32 threshold in 2 MHz ticks) from the C map().
    for (kv, threshold) in [
        (100, 1000),
        (1020, 866),
        (2000, 685),
        (2020, 681),
        (5000, 500),
    ] {
        for dshot in [false, true] {
            for t in threshold - 3..=threshold + 3 {
                assert_eq!(
                    reverse::<Standard>(dshot, kv, t),
                    t > threshold,
                    "kv {kv}, {t}"
                );
                if t % 2 == 0 {
                    assert_eq!(
                        reverse::<Alternate>(dshot, kv, t / 2),
                        t > threshold,
                        "kv {kv}, {t}"
                    );
                }
            }
        }
    }
}

#[test]
fn stall_pid_keeps_same_physical_target_and_output_trajectory() {
    fn trajectory<P: Platform>(counts_per_us: u32) -> Vec<u16> {
        let shared = SharedState::new::<P>();
        shared.transition(MotorEvent::Arm);
        shared.transition(MotorEvent::StartMotor);
        shared.set_zero_crosses(200);
        let mut main = main_state::<P>();
        main.config.stall_protection = 1;
        let mut adc = AdcProbe::default();
        let mut telem = Telemetry::default();
        let system = SystemTick::<P>::new();
        let mut outputs = Vec::new();
        for us in [4000, 4000, 4000, 3000, 2000, 5000, 5000, 1000] {
            shared.set_commutation_interval(us * counts_per_us);
            for _ in 0..P::TICK_HZ / 1000 {
                shared.one_khz_counter_inc();
            }
            system.tick_main(&shared, &mut main, &mut adc, &mut telem);
            outputs.push(shared.stall_protection_adjust());
        }
        outputs
    }
    let standard = trajectory::<Standard>(2);
    assert!(standard.iter().any(|&boost| boost > 0));
    assert_eq!(standard, trajectory::<Alternate>(1));
}

#[derive(Default)]
struct AdcProbe {
    conversions: usize,
}
impl Adc for AdcProbe {
    fn start_conversion(&mut self) {
        self.conversions += 1;
    }
    fn raw_voltage(&self) -> u16 {
        2000
    }
    fn raw_current(&self) -> u16 {
        0
    }
    fn raw_temperature(&self) -> u16 {
        0
    }
    fn calc_temperature(&self, _: u16) -> DegreesCelsius {
        DegreesCelsius(25)
    }
}
#[derive(Default)]
struct Telemetry(Vec<u8>);
impl TelemetryUart for Telemetry {
    fn send_dma(&mut self, bytes: &[u8]) {
        self.0 = bytes.to_vec();
    }
}
struct Led(Rc<Cell<bool>>);
impl embedded_hal::digital::ErrorType for Led {
    type Error = std::convert::Infallible;
}
impl embedded_hal::digital::OutputPin for Led {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set(false);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set(true);
        Ok(())
    }
}
fn main_state<P: Platform>() -> MainState<P> {
    MainState::new(
        &BoardConfig::DEFAULT,
        ChipParams {
            timer1_max_arr: 2399,
            cpu_mhz: 48,
        },
    )
}

#[test]
fn port_initialization_and_capture_prescalers_use_declared_clocks() {
    let shared = SharedState::new::<Alternate>();
    assert_eq!(shared.commutation_interval(), 6250);
    assert_eq!(shared.tim1_arr(), 2399);
    assert_eq!(
        rm32::transfer::CaptureConfig::dshot_detection::<Alternate>().prescaler,
        Some(8)
    );
    assert_eq!(
        rm32::transfer::CaptureConfig::servo_detected::<Alternate>().prescaler,
        Some(47)
    );
    assert_eq!(Alternate::SIGNAL_TIMEOUT_UNARMED, 80_000);
    assert_eq!(
        rm32::constants::telemetry_interval_ticks::<Alternate>(255),
        11_360
    );
}

#[test]
fn conversion_matches_physical_reference_and_preserves_two_mhz_rounding() {
    for sum in (0..=6 * u16::MAX as u32).step_by(7) {
        assert_eq!(six_intervals_to_us::<Standard>(sum), ((sum + 4) / 2) as i32);
        assert_eq!(six_intervals_to_us::<Alternate>(sum), sum as i32);
    }
    for us in (-200_000..=200_000).chain([i32::MIN, i32::MAX]) {
        assert_eq!(e_com_us_to_avg_ticks::<Standard>(us), us / 3);
        assert_eq!(e_com_us_to_avg_ticks::<Alternate>(us), us / 6);
    }
}

#[test]
fn signal_counter_reaches_two_seconds_at_40khz_and_saturates_at_u32_max() {
    let shared = SharedState::new::<Alternate>();
    for _ in 0..80_001 {
        shared.increment_signal_timeout();
    }
    assert_eq!(shared.signal_timeout(), 80_001);
    shared.set_signal_timeout(u32::MAX);
    shared.increment_signal_timeout();
    assert_eq!(shared.signal_timeout(), u32::MAX);
}

#[test]
fn alternate_clock_signal_timeout_keeps_half_second_and_two_second_boundaries() {
    for (armed, limit) in [(true, 20_000), (false, 80_000)] {
        let shared = SharedState::new::<Alternate>();
        shared.set_input_set(true);
        if armed {
            shared.transition(MotorEvent::Arm);
        }
        let mut main = main_state::<Alternate>();
        let system = SystemTick::<Alternate>::new();
        let mut adc = AdcProbe::default();
        let mut telem = Telemetry::default();
        shared.set_signal_timeout(limit);
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        assert!(shared.input_set());
        assert!(!main.needs_reset);
        shared.increment_signal_timeout();
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        assert!(!shared.armed());
        assert!(main.needs_reset);
        assert!(!shared.input_set());
    }
}

#[test]
fn pid_adc_dispatch_stays_one_khz() {
    let shared = SharedState::new::<Alternate>();
    let mut main = main_state::<Alternate>();
    let system = SystemTick::<Alternate>::new();
    let mut adc = AdcProbe::default();
    let mut telem = Telemetry::default();
    for tick in 1..=400 {
        shared.one_khz_counter_inc();
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        assert_eq!(adc.conversions, tick / 40);
    }
}

#[test]
fn charge_and_kiss_keep_physical_units_across_delayed_service() {
    fn check<P: Platform>() {
        let shared = SharedState::new::<P>();
        let mut main = main_state::<P>();
        let system = SystemTick::<P>::new();
        let mut adc = AdcProbe::default();
        let mut telem = Telemetry::default();
        // 36 A for one second must be 10 mAh.
        main.set_actual_current(CentiAmps(3600));
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        for _ in 0..P::TICK_HZ {
            shared.increment_control_ticks();
        }
        shared.set_send_telemetry(true);
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        assert_eq!(main.measurements().consumed_milliamp_hours(), 10);
        assert_eq!(u16::from_be_bytes([telem.0[5], telem.0[6]]), 10);
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        assert_eq!(main.measurements().consumed_milliamp_hours(), 10);
    }
    check::<Standard>();
    check::<Alternate>();
}

#[test]
fn led_uses_elapsed_100ms_phases_at_alternate_control_rate() {
    let shared = SharedState::new::<Alternate>();
    shared.set_adjusted_input(500);
    let high = Rc::new(Cell::new(false));
    let mut main = MainState::<Alternate, _>::new_with_led(
        &BoardConfig::DEFAULT,
        ChipParams {
            timer1_max_arr: 2399,
            cpu_mhz: 48,
        },
        Led(high.clone()),
    );
    let system = SystemTick::<Alternate>::new();
    let mut adc = AdcProbe::default();
    let mut telem = Telemetry::default();
    for (tick, expected) in [
        (0, false),
        (3999, false),
        (4000, true),
        (7999, true),
        (8000, false),
        (20_001, true),
    ] {
        while shared.control_ticks() < tick {
            shared.increment_control_ticks();
        }
        system.tick_main(&shared, &mut main, &mut adc, &mut telem);
        assert_eq!(high.get(), expected, "tick {tick}");
    }
}

#[test]
fn variable_pwm_carrier_is_independent_of_interval_timer_clock() {
    fn arr<P: Platform>(mode: u8, period_us: i32, interval: u32) -> u16 {
        let shared = SharedState::new::<P>();
        shared.set_e_com_time(period_us);
        shared.set_commutation_interval(interval);
        let mut main = main_state::<P>();
        main.config.variable_pwm = mode;
        SystemTick::<P>::new().tick_main(
            &shared,
            &mut main,
            &mut AdcProbe::default(),
            &mut Telemetry::default(),
        );
        shared.tim1_arr()
    }
    for step_us in 1..=300 {
        for mode in [1, 2] {
            assert_eq!(
                arr::<Standard>(mode, step_us * 6, step_us as u32 * 2),
                arr::<Alternate>(mode, step_us * 6, step_us as u32),
                "mode {mode}, step {step_us} us"
            );
        }
    }
}

#[test]
fn filter_level_is_independent_of_interval_timer_clock() {
    fn filter<P: Platform>(period_us: i32, interval: u32) -> u8 {
        let shared = SharedState::new::<P>();
        shared.set_zero_crosses(200);
        shared.set_e_com_time(period_us);
        shared.set_commutation_interval(interval);
        let mut main = main_state::<P>();
        SystemTick::<P>::new().tick_main(
            &shared,
            &mut main,
            &mut AdcProbe::default(),
            &mut Telemetry::default(),
        );
        shared.filter_level()
    }
    for step_us in 1..=300 {
        assert_eq!(
            filter::<Standard>(step_us * 6, step_us as u32 * 2),
            filter::<Alternate>(step_us * 6, step_us as u32),
            "step {step_us} us"
        );
    }
}
