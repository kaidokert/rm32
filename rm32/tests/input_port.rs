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
    AllPwm(PwmMode),
    ComStep(u8),
    ComStepMode(u8, PwmMode),
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
#[path = "../../rm32_stm32/src/timer.rs"]
mod timer;

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
// Model the NVIC enable bits the input tier masks: an IRQ number is
// enabled unless its bit is set here.
pub mod peripheral {
    use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
    static MASKED: AtomicU32 = AtomicU32::new(0);
    static MASK_CALLS: AtomicUsize = AtomicUsize::new(0);
    pub fn mask_calls() -> usize {
        MASK_CALLS.load(Ordering::SeqCst)
    }
    pub struct NVIC;
    impl NVIC {
        pub fn is_enabled(irq: u16) -> bool {
            MASKED.load(Ordering::SeqCst) & (1 << irq) == 0
        }
        pub fn mask(irq: u16) {
            MASK_CALLS.fetch_add(1, Ordering::SeqCst);
            MASKED.fetch_or(1 << irq, Ordering::SeqCst);
        }
        /// # Safety
        /// Host model; no hardware effect.
        pub unsafe fn unmask(irq: u16) {
            MASKED.fetch_and(!(1 << irq), Ordering::SeqCst);
        }
    }
}
pub mod asm {
    pub fn dsb() {}
    pub fn isb() {}
}
const NON_INPUT_IRQ: u16 = 0;

/// Return true when every IRQ in `mcu::INPUT_IRQS` is masked in the host NVIC.
fn input_irqs_masked() -> bool {
    mcu::INPUT_IRQS
        .iter()
        .all(|&irq| !peripheral::NVIC::is_enabled(irq))
}

/// Return true when the representative priority-0 non-input IRQ remains enabled.
fn non_input_irq_enabled() -> bool {
    peripheral::NVIC::is_enabled(NON_INPUT_IRQ)
}
mod platform {
    pub type MainState = rm32::main_state::MainState<StmPlatform>;
    pub struct StmPlatform;
    impl rm32::platform::Platform for StmPlatform {
        const TICK_HZ: u32 = 20_000;
        const TIMER_HZ: u32 = 2_000_000;
        const CAPTURE_HZ: u32 = 48_000_000;
        const GCR_SHIFT: u8 = 6;
        const VOLTAGE_DIVIDER_X100: u16 = 1100;
        const MILLIVOLT_PER_AMP: u16 = 20;
        const CURRENT_OFFSET: i16 = 0;
        const USE_NTC: bool = false;
        const ADC_FULL_SCALE: u16 = 4095;
        const ADC_VREF_MV: u16 = 3300;
        const PWM_BASE_PERIOD: u16 = 1999;
    }
}
mod mcu {
    use std::sync::Mutex as HostMutex;

    /// Host stand-ins for the DShot DMA and EXTI frame IRQ numbers.
    pub const INPUT_IRQS: [u16; 2] = [5, 15];

    pub type TargetIsrHal = super::isr::IsrHal<
        super::Pwm,
        super::Input,
        super::Comp,
        super::Interval,
        super::Com,
        super::Phase,
    >;

    pub trait ChipConfig {
        const TIMER_PSC: u16;
    }
    pub struct Chip;
    impl ChipConfig for Chip {
        const TIMER_PSC: u16 = 31;
    }

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct MockTimerRegs {
        pub cr1: u32,
        pub cnt: u32,
        pub psc: u32,
        pub arr: u32,
        pub egr: u32,
        pub sr: u32,
        pub dier: u32,
        pub cr1_writes: u32,
        pub egr_writes: u32,
    }

    pub static TIM2_REGS: HostMutex<MockTimerRegs> = HostMutex::new(MockTimerRegs {
        cr1: 0,
        cnt: 0,
        psc: 0,
        arr: 0,
        egr: 0,
        sr: 0,
        dier: 0,
        cr1_writes: 0,
        egr_writes: 0,
    });
    pub static COM_REGS: HostMutex<MockTimerRegs> = HostMutex::new(MockTimerRegs {
        cr1: 0,
        cnt: 0,
        psc: 0,
        arr: 0,
        egr: 0,
        sr: 0,
        dier: 0,
        cr1_writes: 0,
        egr_writes: 0,
    });

    pub fn enable_tim2_clock() {}
    pub fn enable_com_timer_clock() {}

    pub struct Tim2Raw;
    impl super::timer::RawTimer for Tim2Raw {
        fn read_cr1(&self) -> u32 {
            TIM2_REGS.lock().unwrap().cr1
        }
        fn write_cr1(&self, v: u32) {
            let mut r = TIM2_REGS.lock().unwrap();
            r.cr1 = v;
            r.cr1_writes += 1;
        }
        fn read_cnt(&self) -> u32 {
            TIM2_REGS.lock().unwrap().cnt
        }
        fn write_cnt(&self, v: u32) {
            TIM2_REGS.lock().unwrap().cnt = v;
        }
        fn write_psc(&self, v: u32) {
            TIM2_REGS.lock().unwrap().psc = v;
        }
        fn write_arr(&self, v: u32) {
            TIM2_REGS.lock().unwrap().arr = v;
        }
        fn write_egr(&self, v: u32) {
            let mut r = TIM2_REGS.lock().unwrap();
            r.egr = v;
            r.egr_writes += 1;
        }
        fn read_sr(&self) -> u32 {
            TIM2_REGS.lock().unwrap().sr
        }
        fn write_sr(&self, v: u32) {
            TIM2_REGS.lock().unwrap().sr = v;
        }
        fn read_dier(&self) -> u32 {
            TIM2_REGS.lock().unwrap().dier
        }
        fn write_dier(&self, v: u32) {
            TIM2_REGS.lock().unwrap().dier = v;
        }
    }

    pub struct ComTimerRaw;
    impl super::timer::RawTimer for ComTimerRaw {
        fn read_cr1(&self) -> u32 {
            COM_REGS.lock().unwrap().cr1
        }
        fn write_cr1(&self, v: u32) {
            let mut r = COM_REGS.lock().unwrap();
            r.cr1 = v;
            r.cr1_writes += 1;
        }
        fn read_cnt(&self) -> u32 {
            COM_REGS.lock().unwrap().cnt
        }
        fn write_cnt(&self, v: u32) {
            COM_REGS.lock().unwrap().cnt = v;
        }
        fn write_psc(&self, v: u32) {
            COM_REGS.lock().unwrap().psc = v;
        }
        fn write_arr(&self, v: u32) {
            COM_REGS.lock().unwrap().arr = v;
        }
        fn write_egr(&self, v: u32) {
            let mut r = COM_REGS.lock().unwrap();
            r.egr = v;
            r.egr_writes += 1;
        }
        fn read_sr(&self) -> u32 {
            COM_REGS.lock().unwrap().sr
        }
        fn write_sr(&self, v: u32) {
            COM_REGS.lock().unwrap().sr = v;
        }
        fn read_dier(&self) -> u32 {
            COM_REGS.lock().unwrap().dier
        }
        fn write_dier(&self, v: u32) {
            COM_REGS.lock().unwrap().dier = v;
        }
    }
}

static TEST_SERIAL_LOCK: HostMutex<()> = HostMutex::new(());
static RECEIVE_DMA_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static LAST_RECEIVE_DMA_SELF_ADDR: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
static LAST_DMA_BUFFER_SELF_ADDR: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(0);
static LAST_EDGES_PRIMASK: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);
static LAST_EDGES_INPUT_IRQS_MASKED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);
static LAST_POST_DECODE_INPUT_IRQS_MASKED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);
static LAST_GCR_BUF_INPUT_IRQS_MASKED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);
static LAST_GCR_PRIMASK: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);
static LAST_GCR_INPUT_IRQS_MASKED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);
static LAST_PWM_DUTY_MASKED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(true);
static LAST_PWM_PERIOD_MASKED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(true);

struct Dma(UnsafeCell<[u32; 64]>);
// Tests are serialized (TEST_SERIAL_LOCK); the producer writes the buffer only
// between complete input-handler calls, and no buffer reference escapes one.
unsafe impl Sync for Dma {}
static DMA: Dma = Dma(UnsafeCell::new([0; 64]));
struct Input {
    gcr: [u32; 37],
}
impl InputCapture for Input {
    fn start_capture(&mut self) {
        RECEIVE_DMA_CALLS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        LAST_RECEIVE_DMA_SELF_ADDR.store(
            self as *const Self as usize,
            std::sync::atomic::Ordering::SeqCst,
        );
    }
    fn start_gcr_output(&mut self) {
        let primask = interrupt::masked();
        let input_masked = input_irqs_masked();
        assert!(!primask && input_masked && non_input_irq_enabled());
        LAST_GCR_PRIMASK.store(primask, std::sync::atomic::Ordering::SeqCst);
        LAST_GCR_INPUT_IRQS_MASKED.store(input_masked, std::sync::atomic::Ordering::SeqCst);
    }
    fn input_pin_state(&self) -> bool {
        assert!(!interrupt::masked() && input_irqs_masked() && non_input_irq_enabled());
        false
    }
    fn set_pull_up(&mut self) {}
    fn set_pull_down(&mut self) {}
    fn set_pull_none(&mut self) {}
    fn edges(&self) -> &[u32; 64] {
        // The input tier must be excluded: PRIMASK (thread-mode callers) or
        // the input IRQs masked in the NVIC (interrupt handlers).
        let primask = interrupt::masked();
        let input_masked = input_irqs_masked();
        assert!(!primask && input_masked && non_input_irq_enabled());
        // Simulate a UART RX interrupt preempting `handle_exti_frame` while
        // `INPUT_LOCAL` is mutably borrowed under `with_input_irqs_masked`.
        isr_handlers::handle_crsf_byte(0xC8);
        assert!(!interrupt::masked() && input_irqs_masked() && non_input_irq_enabled());
        LAST_EDGES_PRIMASK.store(primask, std::sync::atomic::Ordering::SeqCst);
        LAST_EDGES_INPUT_IRQS_MASKED.store(input_masked, std::sync::atomic::Ordering::SeqCst);
        LAST_DMA_BUFFER_SELF_ADDR.store(
            self as *const Self as usize,
            std::sync::atomic::Ordering::SeqCst,
        );
        // See DMA's serialized access contract above.
        unsafe { &*DMA.0.get() }
    }
    fn gcr_buffer(&mut self) -> &mut [u32; 37] {
        let primask = interrupt::masked();
        let input_masked = input_irqs_masked();
        assert!(!primask && input_masked && non_input_irq_enabled());
        LAST_GCR_BUF_INPUT_IRQS_MASKED.store(input_masked, std::sync::atomic::Ordering::SeqCst);
        &mut self.gcr
    }
    fn is_output(&self) -> bool {
        false
    }
    fn set_output_prescaler(&mut self, _: DshotOutputPrescaler) {
        assert!(!interrupt::masked() && input_irqs_masked() && non_input_irq_enabled());
    }
    fn set_capture_prescaler(&mut self, _: u16) {
        let primask = interrupt::masked();
        let input_masked = input_irqs_masked();
        assert!(!primask && input_masked && non_input_irq_enabled());
        LAST_POST_DECODE_INPUT_IRQS_MASKED.store(input_masked, std::sync::atomic::Ordering::SeqCst);
    }
}
struct Pwm;
impl bridge::TimerPwm for Pwm {
    fn set_duty_all(&mut self, value: u16) {
        LAST_PWM_DUTY_MASKED.store(interrupt::masked(), std::sync::atomic::Ordering::SeqCst);
        event(BridgeEvent::Duty(value));
    }
    fn set_auto_reload(&mut self, value: u16) {
        LAST_PWM_PERIOD_MASKED.store(interrupt::masked(), std::sync::atomic::Ordering::SeqCst);
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
    fn com_step(&mut self, step: u8, mode: PwmMode) {
        event(BridgeEvent::ComStep(step));
        event(BridgeEvent::ComStepMode(step, mode));
    }
    fn all_off(&mut self) {
        event(BridgeEvent::AllOff);
    }
    fn full_brake(&mut self) {}
    fn all_pwm(&mut self, mode: PwmMode) {
        event(BridgeEvent::AllPwm(mode));
    }
    fn proportional_brake(&mut self) {}
}
#[derive(Default)]
struct CompState {
    level: bool,
    pending: bool,
    masked: bool,
    repended: bool,
    last_armed_delay: Option<u16>,
    interval_count: u32,
}
static COMP_STATE: HostMutex<CompState> = HostMutex::new(CompState {
    level: false,
    pending: false,
    masked: false,
    repended: false,
    last_armed_delay: None,
    interval_count: 0,
});

struct Comp;
impl BemfSensor for Comp {
    fn select_phase(&mut self, _: rm32::hal::Phase, _: bool) {}
    fn enable_irq(&mut self) {
        COMP_STATE.lock().unwrap().masked = false;
    }
    fn mask_irq(&mut self) {
        let mut s = COMP_STATE.lock().unwrap();
        s.masked = true;
        s.pending = false;
    }
    fn level(&self) -> bool {
        assert!(!interrupt::masked() || COMP_STATE.lock().is_ok());
        COMP_STATE.lock().unwrap().level
    }
    fn pending(&self) -> bool {
        COMP_STATE.lock().unwrap().pending
    }
    fn clear_pending(&mut self) {
        COMP_STATE.lock().unwrap().pending = false;
    }
    fn repend(&mut self) {
        let mut s = COMP_STATE.lock().unwrap();
        s.pending = true;
        s.repended = true;
    }
}
struct Interval;
impl IntervalTimer for Interval {
    fn count(&self) -> u32 {
        COMP_STATE.lock().unwrap().interval_count
    }
    fn set_count(&mut self, v: u32) {
        COMP_STATE.lock().unwrap().interval_count = v;
    }
}
struct Com;
impl ComTimer for Com {
    fn set_and_enable(&mut self, delay: u16) {
        COMP_STATE.lock().unwrap().last_armed_delay = Some(delay);
    }
    fn disable_interrupt(&mut self) {}
    fn enable_interrupt(&mut self) {}
}

/// Encode `value` into the host DMA buffer and invoke `handle_exti_frame` outside `interrupt::free`.
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
    });
    // The handler runs as the EXTI ISR would: outside any producer critical
    // section, so the test sees the exclusion the handler itself takes.
    let next = isr_handlers::handle_exti_frame();
    assert_eq!(next.edges_per_burst, 32);
}
/// Repeat `raw_frame(value)` six times so DShot command confirmation succeeds.
fn command(value: u16) {
    for _ in 0..6 {
        raw_frame(value);
    }
}

#[test]
fn firmware_saves_and_recovers_configuration_from_live_state_after_first_irq() {
    let _serial = TEST_SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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
        writer: unsafe { rm32::shared_comm::SingleWriter::new() },
    });
    let mut staged_input_addr = 0usize;
    isr::with_isr_state(|s| {
        staged_input_addr = &s.hal.input as *const Input as usize;
    });
    assert_ne!(staged_input_addr, 0);
    let mut main = Main::new(
        &rm32::board::BoardConfig::DEFAULT,
        rm32::main_state::ChipParams {
            timer1_max_arr: 1999,
            cpu_mhz: 48,
        },
    );
    let shared = isr::shared();
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    shared.set_input_set(false);
    shared.set_dshot(false);
    RECEIVE_DMA_CALLS.store(0, std::sync::atomic::Ordering::SeqCst);
    LAST_RECEIVE_DMA_SELF_ADDR.store(0, std::sync::atomic::Ordering::SeqCst);
    LAST_DMA_BUFFER_SELF_ADDR.store(0, std::sync::atomic::Ordering::SeqCst);
    LAST_POST_DECODE_INPUT_IRQS_MASKED.store(false, std::sync::atomic::Ordering::SeqCst);
    for _ in 0..2 {
        interrupt::free(|_| {
            let buffer = unsafe { &mut *DMA.0.get() };
            for (i, slot) in buffer[..32].iter_mut().enumerate() {
                *slot = i as u32 * 12;
            }
        });
        let _ = isr_handlers::handle_exti_frame();
    }
    assert!(shared.input_set() && shared.dshot());
    raw_frame(0);
    // The frame decode must exclude only the input-tier IRQs in the NVIC and
    // leave PRIMASK and non-input IRQs clear/enabled from `edges()` through
    // `transfer.process`, `transfer::dispatch`, and `set_capture_prescaler()`.
    assert!(
        !LAST_EDGES_PRIMASK.load(std::sync::atomic::Ordering::SeqCst),
        "DShot frame decode must leave PRIMASK clear so COMP/COM preempt it"
    );
    assert!(
        LAST_EDGES_INPUT_IRQS_MASKED.load(std::sync::atomic::Ordering::SeqCst),
        "DShot frame decode must mask the input-tier IRQs in the NVIC"
    );
    assert!(
        LAST_POST_DECODE_INPUT_IRQS_MASKED.load(std::sync::atomic::Ordering::SeqCst),
        "input-tier IRQs must stay masked through transfer.process, dispatch, and set_capture_prescaler"
    );
    assert!(
        !input_irqs_masked() && non_input_irq_enabled(),
        "the input-tier IRQs must be re-enabled and non-input IRQs left enabled after the frame decode"
    );
    LAST_GCR_BUF_INPUT_IRQS_MASKED.store(false, std::sync::atomic::Ordering::SeqCst);
    LAST_GCR_PRIMASK.store(true, std::sync::atomic::Ordering::SeqCst);
    LAST_GCR_INPUT_IRQS_MASKED.store(false, std::sync::atomic::Ordering::SeqCst);
    shared.set_dshot_telemetry(true);
    isr_handlers::handle_dma_tc();
    shared.set_dshot_telemetry(false);
    assert!(
        !LAST_GCR_PRIMASK.load(std::sync::atomic::Ordering::SeqCst),
        "handle_dma_tc must leave PRIMASK clear so COMP/COM preempt it"
    );
    assert!(
        LAST_GCR_BUF_INPUT_IRQS_MASKED.load(std::sync::atomic::Ordering::SeqCst)
            && LAST_GCR_INPUT_IRQS_MASKED.load(std::sync::atomic::Ordering::SeqCst),
        "handle_dma_tc must keep input-tier IRQs masked across gcr_buffer, encode_response, and start_gcr_output"
    );
    assert!(
        !input_irqs_masked() && non_input_irq_enabled(),
        "the input-tier IRQs must be re-enabled and non-input IRQs left enabled after handle_dma_tc"
    );
    assert_eq!(
        RECEIVE_DMA_CALLS.load(std::sync::atomic::Ordering::SeqCst),
        1,
        "first ISR transfer must re-arm DMA at the live INPUT_LOCAL buffer address"
    );
    let rearmed_addr = LAST_RECEIVE_DMA_SELF_ADDR.load(std::sync::atomic::Ordering::SeqCst);
    let live_addr = LAST_DMA_BUFFER_SELF_ADDR.load(std::sync::atomic::Ordering::SeqCst);
    assert_ne!(
        rearmed_addr, staged_input_addr,
        "start_capture must not point at the consumed ISR_STATE staging cell"
    );
    assert_eq!(
        rearmed_addr, live_addr,
        "start_capture must be called on the permanent INPUT_LOCAL instance"
    );
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
    let entries = interrupt::entries();
    isr_handlers::handle_tim6();
    assert_eq!(
        interrupt::entries(),
        entries + 2,
        "handle_tim6 must open a preemption window between ten_khz_tick and tone::step"
    );
    assert!(!interrupt::masked());
    let start = take_events();
    assert!(
        start.windows(5).any(|w| w
            == [
                BridgeEvent::Period(1999),
                BridgeEvent::Divider(50),
                BridgeEvent::Duty(15),
                BridgeEvent::ComStep(2),
                BridgeEvent::ComStepMode(2, PwmMode::Diode),
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

    // Steady-state Running interrupt-mode tick executes PWM compare/period
    // updates with interrupts unmasked (PRIMASK = 0) and enters interrupt::free
    // only once for the brief priority-0 COM_LOCAL sync.
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Running);
    shared.set_adjusted_input(500);
    shared.set_e_com_time(3_000);
    shared.set_commutation_interval(500);
    shared.set_tim1_arr(1999);
    take_events();
    let entries = interrupt::entries();
    LAST_PWM_DUTY_MASKED.store(true, std::sync::atomic::Ordering::SeqCst);
    LAST_PWM_PERIOD_MASKED.store(true, std::sync::atomic::Ordering::SeqCst);
    isr_handlers::handle_tim6();
    assert_eq!(
        interrupt::entries(),
        entries + 1,
        "steady-state Running tick must enter interrupt::free only once for COM_LOCAL sync"
    );
    assert!(
        !LAST_PWM_DUTY_MASKED.load(std::sync::atomic::Ordering::SeqCst),
        "steady-state Running tick must update TIM1 duty compare with interrupts unmasked"
    );
    assert!(
        !LAST_PWM_PERIOD_MASKED.load(std::sync::atomic::Ordering::SeqCst),
        "steady-state Running tick must update TIM1 auto-reload period with interrupts unmasked"
    );
    assert_eq!(shared.gate_avg(), 1_000);

    // Priority-0 handle_comp_irq and handle_tim14 run without entering
    // interrupt::free (zero PRIMASK overhead on priority 0).
    {
        let mut cs = COMP_STATE.lock().unwrap();
        cs.pending = true;
        cs.level = true; // rising=true at step 1 -> pre-ZC noise
        cs.interval_count = 200; // <= gate_avg / 2 (500) -> gate closed
        cs.last_armed_delay = None;
    }
    let entries = interrupt::entries();
    isr_handlers::handle_comp_irq();
    assert_eq!(interrupt::entries(), entries);
    assert!(
        !COMP_STATE.lock().unwrap().pending,
        "pre-ZC noise blip must be cleared"
    );
    assert_eq!(COMP_STATE.lock().unwrap().last_armed_delay, None);

    {
        let mut cs = COMP_STATE.lock().unwrap();
        cs.pending = true;
        cs.level = false; // post-ZC edge
        cs.interval_count = 200; // gate still closed -> Camp
    }
    isr_handlers::handle_comp_irq();
    assert_eq!(interrupt::entries(), entries);
    assert!(
        COMP_STATE.lock().unwrap().pending,
        "early post-ZC edge must camp with EXTI pending until gate opens"
    );

    {
        let mut cs = COMP_STATE.lock().unwrap();
        cs.interval_count = 600; // > gate_avg / 2 (500) -> gate open
    }
    isr_handlers::handle_comp_irq();
    assert_eq!(interrupt::entries(), entries);
    assert!(!COMP_STATE.lock().unwrap().pending);
    assert!(
        COMP_STATE.lock().unwrap().last_armed_delay.is_some(),
        "open gate + passing persistence filter must arm commutation timer"
    );

    // Priority-0 handle_tim14 checks take_commutation_interrupt() and advances
    // commutation without entering interrupt::free, passing the configured
    // comp_pwm setting (initially 0 -> PwmMode::Diode).
    {
        let mut r = mcu::COM_REGS.lock().unwrap();
        r.sr = 1;
        r.dier = 1;
    }
    take_events();
    let entries = interrupt::entries();
    isr_handlers::handle_tim14();
    assert_eq!(interrupt::entries(), entries);
    assert_eq!(mcu::COM_REGS.lock().unwrap().sr, !1);
    let tim14_events = take_events();
    assert!(
        tim14_events
            .iter()
            .any(|e| matches!(e, BridgeEvent::ComStepMode(_, PwmMode::Diode))),
        "handle_tim14 must pass initial PwmMode::Diode to PhaseOutput::com_step"
    );

    // Mutate comp_pwm (EEPROM byte 20) via DShot programming while Armed, and
    // verify CFG_COMP_PWM and tick.config update for both tone::step and
    // priority-0 handle_tim14.
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    shared.set_adjusted_input(0);
    command(36);
    raw_frame(20);
    raw_frame(1);
    raw_frame(37);
    // Even if a DShot config write lands before the main loop's explicit
    // sync_config_writes call, isr_handlers::service_sine synchronizes
    // main.config and tick.config inside its masked critical section so sine
    // output and priority-0 commutation observe the same drive mode.
    let mut system = rm32::system::SystemTick::<platform::StmPlatform>::new();
    shared.set_motor_mode(rm32::motor_mode::MotorMode::StepperSine);
    shared.set_adjusted_input(100);
    take_events();
    let _ = isr_handlers::service_sine(&mut system, &mut main, 50);
    assert_eq!(main.config.comp_pwm, 1);
    let sine_events = take_events();
    assert!(
        sine_events.contains(&BridgeEvent::AllPwm(PwmMode::Complementary)),
        "service_sine must synchronize pending DShot comp_pwm before all_pwm"
    );
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    shared.set_adjusted_input(0);

    command(1);
    take_events();
    isr_handlers::handle_tim6();
    let tone_events = take_events();
    assert!(
        tone_events.contains(&BridgeEvent::ComStepMode(2, PwmMode::Complementary)),
        "tone::step must pass PwmMode::Complementary when comp_pwm=1"
    );
    for _ in 0..6000 {
        isr_handlers::handle_tim6();
    }

    shared.set_motor_mode(rm32::motor_mode::MotorMode::Running);
    {
        let mut r = mcu::COM_REGS.lock().unwrap();
        r.sr = 1;
        r.dier = 1;
    }
    take_events();
    let entries = interrupt::entries();
    isr_handlers::handle_tim14();
    assert_eq!(interrupt::entries(), entries);
    let tim14_comp_events = take_events();
    assert!(
        tim14_comp_events
            .iter()
            .any(|e| matches!(e, BridgeEvent::ComStepMode(_, PwmMode::Complementary))),
        "handle_tim14 must pass updated PwmMode::Complementary after config mutation"
    );

    // Mutating config while Armed sets CONFIG_DIRTY; if the motor enters
    // steady-state Running before the next tick, handle_tim6 copies inp.config
    // under with_input_irqs_masked (leaving PRIMASK clear until COM_LOCAL sync).
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Armed);
    shared.set_adjusted_input(0);
    command(36);
    raw_frame(20);
    raw_frame(0);
    raw_frame(37);
    shared.set_motor_mode(rm32::motor_mode::MotorMode::Running);
    shared.set_adjusted_input(500);
    let entries = interrupt::entries();
    let mask_calls = peripheral::mask_calls();
    isr_handlers::handle_tim6();
    assert_eq!(
        interrupt::entries(),
        entries + 1,
        "steady-state Running tick with CONFIG_DIRTY must use with_input_irqs_masked, not an extra interrupt::free"
    );
    assert_eq!(
        peripheral::mask_calls(),
        mask_calls + mcu::INPUT_IRQS.len(),
        "steady-state Running tick with CONFIG_DIRTY must mask mcu::INPUT_IRQS in the NVIC"
    );
    assert!(!input_irqs_masked() && non_input_irq_enabled());
    {
        let mut r = mcu::COM_REGS.lock().unwrap();
        r.sr = 1;
        r.dier = 1;
    }
    take_events();
    isr_handlers::handle_tim14();
    let tim14_diode_events = take_events();
    assert!(
        tim14_diode_events
            .iter()
            .any(|e| matches!(e, BridgeEvent::ComStepMode(_, PwmMode::Diode))),
        "handle_tim14 must observe PwmMode::Diode after handle_tim6's CONFIG_DIRTY copy"
    );
}

#[test]
fn timer_init_and_set_and_enable_match_free_running_am32_contract() {
    let _serial = TEST_SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    *mcu::TIM2_REGS.lock().unwrap() = mcu::MockTimerRegs::default();
    *mcu::COM_REGS.lock().unwrap() = mcu::MockTimerRegs::default();

    let _interval = timer::Tim2Interval::new();
    let tim2 = *mcu::TIM2_REGS.lock().unwrap();
    assert_eq!(tim2.psc, 31);
    assert_eq!(tim2.arr, 0xFFFF, "Tim2Interval must wrap at 16 bits");
    assert_eq!(tim2.cr1 & 1, 1);

    let mut com = timer::Tim14Com::new();
    let init_com = *mcu::COM_REGS.lock().unwrap();
    assert_eq!(init_com.psc, 31);
    assert_eq!(init_com.arr, 0xFFFF);
    assert_eq!(init_com.egr, 1);
    assert_eq!(
        init_com.cr1, 1,
        "Tim14Com::new must start free-running with CEN=1 and ARPE=0"
    );

    com.set_and_enable(345);
    let armed_com = *mcu::COM_REGS.lock().unwrap();
    assert_eq!(armed_com.cnt, 0);
    assert_eq!(armed_com.arr, 345);
    assert_eq!(armed_com.sr, 0);
    assert_eq!(armed_com.dier & 1, 1);
    assert_eq!(
        armed_com.cr1_writes, init_com.cr1_writes,
        "set_and_enable must not toggle CR1.CEN"
    );
    assert_eq!(
        armed_com.egr_writes, init_com.egr_writes,
        "set_and_enable must not fire EGR.UG"
    );
}
