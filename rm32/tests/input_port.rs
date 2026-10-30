//! Execute the production STM32 ISR/input adapters on a serialized host.
//! This checks live-state ownership and call ordering, not hardware IRQ timing.
#![allow(dead_code)]
extern crate self as cortex_m;

use rm32::hal::*;
use std::cell::UnsafeCell;
use std::sync::Mutex as HostMutex;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BridgeEvent {
    Duty(u16),
    Period(u16),
    Divider(u16),
    AllOff,
    ComStep(u8),
}
static BRIDGE_EVENTS: HostMutex<Vec<BridgeEvent>> = HostMutex::new(Vec::new());
fn event(value: BridgeEvent) {
    BRIDGE_EVENTS.lock().unwrap().push(value);
}
fn take_events() -> Vec<BridgeEvent> {
    core::mem::take(&mut *BRIDGE_EVENTS.lock().unwrap())
}

#[path = "../../rm32_stm32/src/bridge.rs"]
mod bridge;
#[path = "../../rm32_stm32/src/isr.rs"]
mod isr;
#[path = "../../rm32_stm32/src/isr_handlers.rs"]
mod isr_handlers;

// Model single-core interrupt exclusion, including nested critical sections.
pub mod interrupt {
    use std::{cell::Cell, sync::Mutex as HostMutex};
    static LOCK: HostMutex<()> = HostMutex::new(());
    thread_local! { static DEPTH: Cell<u32> = const { Cell::new(0) }; }
    thread_local! { static ENTRIES: Cell<usize> = const { Cell::new(0) }; }
    pub struct CriticalSection;
    pub struct Mutex<T>(T);
    // Access is serialized by free(); this is the same contract as the port API.
    unsafe impl<T: Send> Sync for Mutex<T> {}
    impl<T> Mutex<T> {
        pub const fn new(value: T) -> Self {
            Self(value)
        }
        pub fn borrow<'a>(&'a self, _: &'a CriticalSection) -> &'a T {
            &self.0
        }
    }
    pub fn masked() -> bool {
        DEPTH.with(|d| d.get() != 0)
    }
    pub fn entries() -> usize {
        ENTRIES.with(Cell::get)
    }
    pub fn free<R>(f: impl FnOnce(&CriticalSection) -> R) -> R {
        if !masked() {
            ENTRIES.with(|n| n.set(n.get() + 1));
        }
        let _guard = if masked() {
            None
        } else {
            Some(LOCK.lock().unwrap())
        };
        struct Depth;
        impl Drop for Depth {
            fn drop(&mut self) {
                DEPTH.with(|d| d.set(d.get() - 1));
            }
        }
        DEPTH.with(|d| d.set(d.get() + 1));
        let _depth = Depth;
        f(&CriticalSection)
    }
}
mod platform {
    pub struct StmPlatform;
    impl rm32::platform::Platform for StmPlatform {
        const TICK_HZ: u32 = 20_000;
        const TIMER_HZ: u32 = 2_000_000;
        const CAPTURE_HZ: u32 = 48_000_000;
        const GCR_SHIFT: u8 = 6;
        const PWM_BASE_PERIOD: u16 = 1999;
    }
}
mod mcu {
    pub type TargetIsrHal = super::isr::IsrHal<
        super::Pwm,
        super::Input,
        super::Comp,
        super::Interval,
        super::Com,
        super::Phase,
    >;
}
mod timer {
    pub fn take_commutation_interrupt() -> bool {
        false
    }
}

struct Dma(UnsafeCell<[u32; 64]>);
// Only one test uses this DMA substitute, and every read/write is inside free().
// No buffer reference escapes a completed input-handler call.
unsafe impl Sync for Dma {}
static DMA: Dma = Dma(UnsafeCell::new([0; 64]));
struct Input {
    gcr: [u32; 37],
}
impl InputCapture for Input {
    fn receive_dshot_dma(&mut self) {}
    fn send_dshot_dma(&mut self) {}
    fn input_pin_state(&self) -> bool {
        false
    }
    fn set_pull_up(&mut self) {}
    fn set_pull_down(&mut self) {}
    fn set_pull_none(&mut self) {}
    fn dma_buffer(&self) -> &[u32; 64] {
        assert!(interrupt::masked());
        // See DMA's serialized access contract above.
        unsafe { &*DMA.0.get() }
    }
    fn gcr_buffer(&mut self) -> &mut [u32; 37] {
        &mut self.gcr
    }
    fn is_output(&self) -> bool {
        false
    }
    fn set_output_prescaler(&mut self, _: DshotOutputPrescaler) {
        assert!(interrupt::masked());
    }
}
struct Pwm;
impl bridge::TimerPwm for Pwm {
    fn set_duty_all(&mut self, value: u16) {
        event(BridgeEvent::Duty(value));
    }
    fn set_auto_reload(&mut self, value: u16) {
        event(BridgeEvent::Period(value));
    }
    fn set_prescaler(&mut self, value: u16) {
        event(BridgeEvent::Divider(value));
    }
    fn set_compare1(&mut self, _: u16) {}
    fn set_compare2(&mut self, _: u16) {}
    fn set_compare3(&mut self, _: u16) {}
    fn generate_update_event(&mut self) {}
    fn set_dead_time_override(&mut self, _: u16) {}
}
struct Phase;
impl PhaseOutput for Phase {
    fn com_step(&mut self, step: u8) {
        event(BridgeEvent::ComStep(step));
    }
    fn all_off(&mut self) {
        event(BridgeEvent::AllOff);
    }
    fn full_brake(&mut self) {}
    fn all_pwm(&mut self) {}
    fn proportional_brake(&mut self) {}
}
struct Comp;
impl BemfSensor for Comp {
    fn select_phase(&mut self, _: rm32::hal::Phase, _: bool) {}
    fn enable_irq(&mut self) {}
    fn mask_irq(&mut self) {}
    fn level(&self) -> bool {
        false
    }
    fn pending(&self) -> bool {
        false
    }
    fn clear_pending(&mut self) {}
    fn repend(&mut self) {}
}
struct Interval;
impl IntervalTimer for Interval {
    fn count(&self) -> u32 {
        0
    }
    fn set_count(&mut self, _: u32) {}
}
struct Com;
impl ComTimer for Com {
    fn set_and_enable(&mut self, _: u16) {}
    fn disable_interrupt(&mut self) {}
    fn enable_interrupt(&mut self) {}
}

fn raw_frame(value: u16) {
    interrupt::free(|_| {
        let payload = value * 2;
        let crc = ((payload & 15) ^ ((payload / 16) & 15) ^ ((payload / 256) & 15)) & 15;
        let packet = payload * 16 + crc;
        // A test producer updates DMA only between complete handler invocations.
        let buffer = unsafe { &mut *DMA.0.get() };
        for bit in 0..16 {
            buffer[bit * 2] = bit as u32 * 100;
            buffer[bit * 2 + 1] = buffer[bit * 2]
                + if packet & (1 << (15 - bit)) != 0 {
                    75
                } else {
                    25
                };
        }
        let next = isr_handlers::handle_exti_frame();
        assert_eq!(next.ndtr, 32);
        assert_eq!(next.prescaler, None);
    });
}
fn command(value: u16) {
    for _ in 0..6 {
        raw_frame(value);
    }
}

#[test]
fn firmware_saves_and_recovers_configuration_from_live_state_after_first_irq() {
    type Main = rm32::main_state::MainState<platform::StmPlatform>;
    let config = rm32::config::EepromConfig::default();
    isr::init_isr_state(isr::IsrState {
        commutation: rm32::commutation::Commutation::new(),
        bemf: rm32::control::state::BemfState::default(),
        duty: rm32::control::state::DutyState::default(),
        hal: isr::IsrHal {
            pwm: Pwm,
            input: Input { gcr: [0; 37] },
            comp: Comp,
            interval: Interval,
            com_timer: Com,
            phase: Phase,
        },
        cmd: rm32::dshot_commands::CommandProcessor::default(),
        tone: rm32::tone::ToneScheduler::default(),
        edt: rm32::edt::EdtScheduler::default(),
        crsf: rm32::crsf::CrsfParser::new(),
        transfer: rm32::transfer::TransferState::default(),
        config,
        forward: true,
        edt_armed: false,
        edt_arm_enable: false,
        frametime_low: 1400,
        frametime_high: 1650,
        armed_timeout_count: 0,
        voltage_based_ramp: false,
    });
    let mut main = Main::new(
        &rm32::board::BoardConfig::DEFAULT,
        rm32::main_state::ChipParams {
            timer1_max_arr: 1999,
            cpu_mhz: 48,
        },
    );
    let shared = isr::shared();
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    shared.set_input_set(true);
    shared.set_dshot(true);
    raw_frame(0); // First interrupt moves state out of the initialization cell.
    let mut visited = false;
    isr::with_isr_state(|_| visited = true);
    assert!(
        !visited,
        "fixture must exercise state after ownership transfer"
    );

    for i in 0..12u16 {
        command(36);
        raw_frame(48 + i);
        raw_frame(101 + i);
        raw_frame(37);
    }
    assert!(shared.config_write_overflowed());
    let entries = interrupt::entries();
    isr_handlers::resync_input_config(&mut main);
    assert_eq!(interrupt::entries(), entries + 1);
    assert!(!interrupt::masked());
    for i in 0..12 {
        assert_eq!(main.config.as_bytes()[48 + i], 101 + i as u8);
    }
    assert!(shared.pop_config_write().is_none());
    assert!(!shared.config_write_overflowed());

    command(8);
    command(12);
    let entries = interrupt::entries();
    assert!(isr_handlers::prepare_input_save(&mut main));
    assert_eq!(interrupt::entries(), entries + 1);
    assert!(!interrupt::masked()); // The caller may now perform flash I/O.
    assert_eq!(main.config.dir_reversed, 1);
    assert!(!isr_handlers::prepare_input_save(&mut main));
    let persisted = main.config;
    command(7);
    command(12);
    assert!(shared.save_settings_flag());
    assert_eq!(persisted.dir_reversed, 1);
    assert!(isr_handlers::prepare_input_save(&mut main));
    assert_eq!(main.config.dir_reversed, 0);

    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    shared.set_tim1_arr(1777);
    command(1);
    take_events();
    isr_handlers::handle_tim6();
    let start = take_events();
    assert!(
        start.windows(4).any(|w| w
            == [
                BridgeEvent::Period(1999),
                BridgeEvent::Divider(50),
                BridgeEvent::Duty(15),
                BridgeEvent::ComStep(2),
            ]),
        "{start:?}"
    );
    assert!(shared.tone_active());

    isr_handlers::handle_tim6();
    let held = take_events();
    assert!(!held.iter().any(|e| matches!(
        e,
        BridgeEvent::AllOff | BridgeEvent::Duty(_) | BridgeEvent::Period(_)
    )));

    for _ in 0..6000 {
        isr_handlers::handle_tim6();
    }
    let completed = take_events();
    assert!(completed.contains(&BridgeEvent::AllOff));
    assert!(completed.contains(&BridgeEvent::Divider(0)));
    assert!(completed.contains(&BridgeEvent::Period(1777)));
    assert!(!shared.tone_active());

    command(2);
    isr_handlers::handle_tim6();
    assert!(shared.tone_active());
    shared.set_motor_mode(rm32::motor_mode::MotorMode::OldRoutine);
    take_events();
    isr_handlers::handle_tim6();
    let interrupted = take_events();
    assert!(interrupted.contains(&BridgeEvent::Divider(0)));
    assert!(interrupted.contains(&BridgeEvent::Period(1777)));
    assert!(
        !interrupted.contains(&BridgeEvent::AllOff),
        "aborting a tone must not clear motor phase outputs"
    );
    assert!(!shared.tone_active());

    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    command(2);
    isr_handlers::handle_tim6();
    assert!(shared.tone_active());
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Disarmed);
    take_events();
    isr_handlers::handle_tim6();
    let stopped = take_events();
    assert!(stopped.contains(&BridgeEvent::AllOff));
    assert!(!stopped.contains(&BridgeEvent::ComStep(2)));
    assert!(!shared.tone_active());
}
