//! Exercise the main/ISR boundary using real shared state and observable peripherals.
use std::{
    cell::{Cell, RefCell},
    rc::Rc,
};

use rm32::{
    board::BoardConfig,
    commutation::Commutation,
    control::{
        context::MotorContext,
        isr_logic,
        state::{BemfState, DutyState},
    },
    hal::*,
    main_state::{ChipParams, MainState},
    motor_mode::MotorEvent,
    shared_state::SharedState,
    system::SystemTick,
};

type Log = Rc<RefCell<Vec<&'static str>>>;
struct Pwm {
    active: Rc<Cell<[u16; 3]>>,
    values: [u16; 3],
    log: Log,
}
impl PwmOutput for Pwm {
    fn set_duty_all(&mut self, v: u16) {
        self.values = [v; 3];
        self.log.borrow_mut().push("duty");
    }
    fn set_compare1(&mut self, v: u16) {
        self.values[0] = v;
        self.log.borrow_mut().push("a");
    }
    fn set_compare2(&mut self, v: u16) {
        self.values[1] = v;
        self.log.borrow_mut().push("b");
    }
    fn set_compare3(&mut self, v: u16) {
        self.values[2] = v;
        self.log.borrow_mut().push("c");
    }
    fn generate_update_event(&mut self) {
        self.active.set(self.values);
        self.log.borrow_mut().push("latch");
    }
    fn set_auto_reload(&mut self, _: u16) {}
    fn set_prescaler(&mut self, _: u16) {}
    fn set_dead_time_override(&mut self, _: u16) {}
}
struct Phase {
    active: Rc<Cell<[u16; 3]>>,
    sector_active: [u16; 3],
    log: Log,
    step: u8,
}
impl PhaseOutput for Phase {
    fn com_step(&mut self, step: u8) {
        self.sector_active = self.active.get();
        self.step = step;
        self.log.borrow_mut().push("sector");
    }
    fn all_off(&mut self) {
        self.log.borrow_mut().push("off");
    }
    fn all_pwm(&mut self) {
        self.log.borrow_mut().push("sine");
    }
    fn full_brake(&mut self) {
        self.log.borrow_mut().push("full_brake");
    }
    fn proportional_brake(&mut self) {
        self.log.borrow_mut().push("prop_brake");
    }
}
struct Comp {
    log: Log,
    enabled: bool,
    step: u8,
    rising: bool,
}
impl Comparator for Comp {
    fn output_level(&self) -> bool {
        false
    }
    fn set_step(&mut self, step: u8, rising: bool) {
        self.step = step;
        self.rising = rising;
        self.log.borrow_mut().push("route");
    }
    fn change_input(&mut self) {
        self.log.borrow_mut().push("mux");
    }
    fn enable_interrupts(&mut self) {
        self.enabled = true;
        self.log.borrow_mut().push("comp_on");
    }
    fn mask_interrupts(&mut self) {
        self.enabled = false;
        self.log.borrow_mut().push("comp_off");
    }
}
struct Interval {
    count: u32,
    log: Log,
}
impl IntervalTimer for Interval {
    fn count(&self) -> u32 {
        self.count
    }
    fn set_count(&mut self, v: u32) {
        self.count = v;
        self.log.borrow_mut().push("interval");
    }
}
struct Com {
    delay: u16,
    enabled: bool,
    log: Log,
}
impl ComTimer for Com {
    fn set_and_enable(&mut self, v: u16) {
        self.delay = v;
        self.enabled = true;
        self.log.borrow_mut().push("com_on");
    }
    fn disable_interrupt(&mut self) {
        self.enabled = false;
        self.log.borrow_mut().push("com_off");
    }
    fn enable_interrupt(&mut self) {
        self.enabled = true;
    }
}
struct Hal {
    pwm: Pwm,
    phase: Phase,
    comp: Comp,
    interval: Interval,
    com: Com,
}
impl MotorHal for Hal {
    type Pwm = Pwm;
    type Phase = Phase;
    type Comp = Comp;
    type Interval = Interval;
    type Com = Com;
    fn pwm(&mut self) -> &mut Pwm {
        &mut self.pwm
    }
    fn phase(&mut self) -> &mut Phase {
        &mut self.phase
    }
    fn comp(&mut self) -> &mut Comp {
        &mut self.comp
    }
    fn interval(&mut self) -> &mut Interval {
        &mut self.interval
    }
    fn com_timer(&mut self) -> &mut Com {
        &mut self.com
    }
}
struct Rig {
    shared: SharedState,
    system: SystemTick,
    main: MainState,
    comm: Commutation,
    bemf: BemfState,
    duty: DutyState,
    hal: Hal,
    log: Log,
}
impl Rig {
    fn new() -> Self {
        let log: Log = Rc::default();
        let active = Rc::new(Cell::new([0; 3]));
        let shared = SharedState::new();
        shared.transition(MotorEvent::Arm);
        shared.set_tim1_arr(1999);
        shared.set_duty_maximum(2000);
        shared.set_current_limit_adjust(2000);
        let mut main = MainState::new(
            &BoardConfig::DEFAULT,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 48,
            },
        );
        main.config.use_sine_start = 1;
        main.config.sine_mode_power = 5;
        let mut duty = DutyState::default();
        duty.set_duty_limits(100, 0, 2000);
        Self {
            shared,
            main,
            system: SystemTick::new(),
            comm: Commutation::new(),
            bemf: BemfState::default(),
            duty,
            hal: Hal {
                pwm: Pwm {
                    active: active.clone(),
                    values: [0; 3],
                    log: log.clone(),
                },
                phase: Phase {
                    active,
                    sector_active: [0; 3],
                    log: log.clone(),
                    step: 0,
                },
                comp: Comp {
                    log: log.clone(),
                    enabled: false,
                    step: 0,
                    rising: false,
                },
                interval: Interval {
                    count: 0,
                    log: log.clone(),
                },
                com: Com {
                    delay: 0,
                    enabled: false,
                    log: log.clone(),
                },
            },
            log,
        }
    }
    fn tick(&mut self, mapped_input: u16) {
        self.shared.set_adjusted_input(mapped_input);
        isr_logic::ten_khz_tick(&mut MotorContext {
            commutation: &mut self.comm,
            bemf: &mut self.bemf,
            duty: &mut self.duty,
            config: &self.main.config,
            armed_timeout_count: &mut 0,
            voltage_based_ramp: false,
            shared: &self.shared,
            hal: &mut self.hal,
        });
    }
    fn sine(&mut self) -> Option<u16> {
        self.system.service_sine(
            &self.shared,
            &mut self.main,
            &mut self.hal,
            &mut self.duty,
            self.comm.step(),
            60,
        )
    }
}

#[test]
fn entry_threshold_and_direct_start_policy() {
    for input in [0, 47, 48, 126, 127, 136, 137, 200, 201, 2047] {
        let mut r = Rig::new();
        r.tick(input);
        assert_eq!(r.shared.stepper_sine(), input < 127, "input {input}");
        assert_eq!(r.shared.running(), input >= 127, "input {input}");
    }
    let mut r = Rig::new();
    r.shared.transition(MotorEvent::Disarm);
    r.tick(80);
    assert!(!r.shared.stepper_sine());
    assert_eq!(r.sine(), None);
    let mut r = Rig::new();
    r.main.config.use_sine_start = 0;
    r.tick(80);
    assert!(r.shared.running());
    assert!(!r.shared.stepper_sine());
}

#[test]
fn sine_duty_map_matches_endpoints_and_linear_reference() {
    for input in 127..=2047 {
        let mut r = Rig::new();
        r.shared.set_zero_crosses(1000);
        r.tick(input);
        let expected = 140 + (u32::from(input.max(137) - 137) * 1860 / 1910) as u16;
        assert_eq!(r.shared.duty_cycle_setpoint(), expected, "input {input}");
    }
}

#[test]
fn sine_entry_uses_actual_sector_and_direction_once() {
    for step in 1..=6 {
        for forward in [false, true] {
            let mut r = Rig::new();
            r.comm.set_step(step);
            r.shared.set_forward(forward);
            r.tick(80);
            r.log.borrow_mut().clear();
            assert!(r.sine().is_some());
            let a = ((i32::from(step) - 1) * 60 + 180 + if forward { -1 } else { 1 }) % 360;
            let expected = [a, (a + 119) % 360, (a + 239) % 360].map(|p| {
                (((i32::from(rm32::sine::PWM_SIN[p as usize]) + 60) * 1999 / 2000) * 5 / 10) as u16
            });
            assert_eq!(r.hal.pwm.values, expected);
            assert_eq!(
                *r.log.borrow(),
                [
                    "com_off", "comp_off", "duty", "latch", "sine", "a", "b", "c"
                ]
            );
            r.log.borrow_mut().clear();
            r.sine();
            assert_eq!(*r.log.borrow(), ["a", "b", "c"]);
            let values = r.hal.pwm.values;
            for _ in 0..4 {
                r.tick(80);
            }
            assert_eq!(r.hal.pwm.values, values);
        }
    }
}

#[test]
fn handoff_is_ordered_single_use_and_direction_correct() {
    for forward in [false, true] {
        let mut r = Rig::new();
        r.tick(0);
        r.shared.set_adjusted_input(201);
        r.shared.set_forward(forward);
        r.sine();
        r.hal.pwm.generate_update_event(); // Simulate a PWM period loading sine compares.
        assert!(r.shared.stepper_sine());
        assert_eq!(r.shared.changeover_step(), 5);
        r.log.borrow_mut().clear();
        r.sine();
        assert!(r.log.borrow().is_empty());
        r.tick(201);
        assert!(r.shared.old_routine());
        assert_eq!(r.shared.changeover_step(), 0);
        assert_eq!(r.comm.step(), 5);
        assert_eq!(r.hal.phase.sector_active, [0; 3]);
        assert_eq!(r.hal.comp.step, 5);
        assert_eq!(r.hal.comp.rising, forward);
        assert!(
            !r.hal.comp.enabled,
            "polling and comparator IRQ must be exclusive"
        );
        assert_eq!(r.hal.interval.count, 9000);
        assert_eq!(r.hal.com.delay, 9000);
        assert!(r.hal.com.enabled);
        assert_eq!(r.shared.zero_crosses(), 20);
        let log = r.log.borrow();
        let at = |s| log.iter().position(|v| *v == s).unwrap();
        assert!(at("com_off") < at("sector"));
        assert!(at("duty") < at("sector"));
        assert!(at("latch") < at("sector"));
        assert!(at("latch") < at("interval"));
        assert!(at("interval") < at("route"));
        assert!(at("mux") < at("com_on"));
        drop(log);
        r.log.borrow_mut().clear();
        r.tick(201);
        assert!(!r.log.borrow().contains(&"sector"));
    }
}

#[test]
fn stale_handoff_cannot_restart_after_disarm_release_or_all_off() {
    for cancel in 0..3 {
        let mut r = Rig::new();
        r.tick(0);
        r.shared.set_adjusted_input(201);
        r.sine();
        if cancel == 0 {
            r.shared.transition(MotorEvent::Disarm);
        }
        if cancel == 2 {
            r.shared
                .request_isr_action(rm32::shared_comm::IsrAction::AllOff);
        }
        r.log.borrow_mut().clear();
        r.tick(if cancel == 1 { 0 } else { 201 });
        assert_eq!(r.shared.changeover_step(), 0);
        assert!(!r.hal.com.enabled);
        assert!(!r.log.borrow().contains(&"sector"));
        assert_eq!(r.hal.pwm.values, [0; 3]);
    }
}

#[test]
fn idle_never_writes_sine_pwm_and_restart_reseeds() {
    let mut r = Rig::new();
    r.tick(80);
    r.sine();
    r.tick(0);
    r.log.borrow_mut().clear();
    r.sine();
    assert!(r.log.borrow().is_empty());
    assert_eq!(r.hal.pwm.values, [0; 3]);
    r.comm.set_step(3);
    r.tick(80);
    r.sine();
    let restarted = r.hal.pwm.values;
    let mut fresh = Rig::new();
    fresh.comm.set_step(3);
    fresh.tick(80);
    fresh.sine();
    assert_eq!(restarted, fresh.hal.pwm.values);
}

#[test]
fn sine_idle_preserves_configured_brake_output() {
    for mode in 0..=2 {
        let mut r = Rig::new();
        r.main.config.brake_on_stop = mode;
        r.main.config.drag_brake_strength = 5;
        r.main.config.active_brake_power = 50;
        r.tick(0);
        r.sine();
        r.tick(0);
        let (phase, duty) = match mode {
            1 => ("prop_brake", 1000),
            2 => ("sector", 490),
            _ => ("off", 0),
        };
        assert_eq!(r.hal.pwm.values, [duty; 3]);
        let log = r.log.borrow();
        assert!(
            log.iter().position(|v| *v == phase).unwrap()
                < log.iter().rposition(|v| *v == "duty").unwrap()
        );
    }
}

#[test]
fn restart_reseeds_even_if_main_misses_the_idle_interval() {
    let mut r = Rig::new();
    r.tick(80);
    r.sine();
    r.tick(0);
    // No main service here: the ISR alone observes the release and restart.
    r.comm.set_step(4);
    r.tick(80);
    r.log.borrow_mut().clear();
    r.sine();
    assert!(r.log.borrow().contains(&"sine"));
    let mut fresh = Rig::new();
    fresh.comm.set_step(4);
    fresh.tick(80);
    fresh.sine();
    assert_eq!(r.hal.pwm.values, fresh.hal.pwm.values);
}

#[test]
fn running_to_sine_neutralizes_outputs_before_main_runs() {
    let mut r = Rig::new();
    r.tick(500);
    r.shared.transition(MotorEvent::BemfLocked);
    r.hal.pwm.set_duty_all(1200);
    r.hal.pwm.generate_update_event();
    r.hal.com.enabled = true;
    r.hal.comp.enabled = true;
    r.log.borrow_mut().clear();
    r.tick(80);
    assert!(r.shared.stepper_sine());
    assert!(r.log.borrow().contains(&"off"));
    assert_eq!(r.hal.pwm.active.get(), [0; 3]);
    assert!(!r.hal.com.enabled);
    assert!(!r.hal.comp.enabled);
    r.sine();
    assert!(r.log.borrow().contains(&"sine"));
}

#[test]
fn pending_comparator_interrupt_cannot_drive_sine_or_polling() {
    for mode in [
        rm32::motor_mode::MotorMode::Disarmed,
        rm32::motor_mode::MotorMode::Armed,
        rm32::motor_mode::MotorMode::StepperSine,
        rm32::motor_mode::MotorMode::OldRoutine,
    ] {
        let mut r = Rig::new();
        r.shared.set_motor_mode(mode);
        r.hal.interval.count = 1234;
        let accepted = isr_logic::bemf_zero_cross(
            &r.shared,
            &r.comm,
            &mut r.bemf,
            &mut r.hal.comp,
            &mut r.hal.interval,
            &mut r.hal.com,
        );
        assert!(!accepted);
        assert_eq!(r.hal.interval.count, 1234);
        assert!(!r.hal.com.enabled);
        assert_eq!(*r.log.borrow(), ["comp_off"]);
    }
}

#[test]
fn steady_active_braking_does_not_repeatedly_latch_zero() {
    let mut r = Rig::new();
    r.main.config.brake_on_stop = 2;
    r.main.config.active_brake_power = 50;
    r.tick(0);
    r.hal.pwm.generate_update_event(); // Natural PWM period loads brake compares.
    assert_eq!(r.hal.pwm.active.get(), [490; 3]);
    r.log.borrow_mut().clear();
    for _ in 0..3 {
        r.tick(0);
    }
    assert_eq!(r.hal.pwm.active.get(), [490; 3]);
    assert!(!r.log.borrow().contains(&"latch"));
    assert!(!r.log.borrow().contains(&"sector"));
}

#[test]
fn brief_main_sine_service_invalidates_active_brake_topology() {
    let mut r = Rig::new();
    r.main.config.brake_on_stop = 2;
    r.main.config.active_brake_power = 50;
    r.tick(0);
    r.shared.set_adjusted_input(80);
    r.sine();
    // Main saw active throttle, but the ISR next sees idle again.
    r.log.borrow_mut().clear();
    r.tick(0);
    assert!(r.log.borrow().contains(&"sector"));
    assert_eq!(r.hal.phase.step, 2);
    assert_eq!(r.hal.phase.sector_active, [0; 3]);
    assert_eq!(r.hal.pwm.values, [490; 3]);
}
