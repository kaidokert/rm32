//! ISR-exclusive state and interrupt handlers.
//!
//! `IsrHal` and `IsrState` are generic over HAL types — no cfg blocks.
//! The concrete target type is resolved via `TargetIsrState` alias.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rm32::commutation::Commutation;
use rm32::config::EepromConfig;
use rm32::control::state::{BemfState, DutyState};
use rm32::crsf::CrsfParser;
use rm32::dshot_commands::CommandProcessor;
use rm32::edt::EdtScheduler;
use rm32::hal;
use rm32::transfer::TransferState;

use rm32::shared_state::SharedState;

/// ISR-exclusive hardware — generic over all HAL peripherals.
/// Zero cfg blocks. Concrete types resolved by `TargetIsrHal` alias.
pub struct IsrHal<P, I, C, IT, CT, PH>
where
    P: hal::PwmOutput,
    I: hal::InputCapture,
    C: hal::Comparator,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    pub pwm: P,
    pub input: I,
    pub comp: C,
    pub interval: IT,
    pub com_timer: CT,
    pub phase: PH,
}

impl<P, I, C, IT, CT, PH> hal::MotorHal for IsrHal<P, I, C, IT, CT, PH>
where
    P: hal::PwmOutput,
    I: hal::InputCapture,
    C: hal::Comparator,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    type Pwm = P;
    type Comp = C;
    type Phase = PH;
    type Interval = IT;
    type Com = CT;

    fn pwm(&mut self) -> &mut P {
        &mut self.pwm
    }
    fn comp(&mut self) -> &mut C {
        &mut self.comp
    }
    fn phase(&mut self) -> &mut PH {
        &mut self.phase
    }
    fn interval(&mut self) -> &mut IT {
        &mut self.interval
    }
    fn com_timer(&mut self) -> &mut CT {
        &mut self.com_timer
    }
}

/// ISR-exclusive state — generic over hardware.
pub struct IsrState<H> {
    pub commutation: Commutation,
    pub bemf: BemfState,
    pub duty: DutyState,
    pub hal: H,
    pub cmd: CommandProcessor,
    pub edt: EdtScheduler,
    pub crsf: CrsfParser,
    pub transfer: TransferState,
    pub config: EepromConfig,
    pub forward: bool,
    pub edt_armed: bool,
    pub counters: rm32::control::isr_logic::TickCounters,
    pub frametime_low: u16,
    pub frametime_high: u16,
    pub voltage_based_ramp: bool,
}

// Target-specific type alias — defined in mcu_xxx/chip.rs, re-exported via mcu::*.
pub use crate::mcu::TargetIsrHal;

/// Concrete ISR state for the selected target.
pub type TargetIsrState = IsrState<TargetIsrHal>;

// ============================================================
// Global state + accessors
// ============================================================

static ISR_STATE: Mutex<RefCell<Option<TargetIsrState>>> = Mutex::new(RefCell::new(None));
static SHARED: SharedState = SharedState::new();

pub fn shared() -> &'static SharedState {
    &SHARED
}

pub fn init_isr_state(state: TargetIsrState) {
    cortex_m::interrupt::free(|cs| {
        ISR_STATE.borrow(cs).replace(Some(state));
    });
}

pub fn take_isr_state() -> Option<TargetIsrState> {
    cortex_m::interrupt::free(|cs| ISR_STATE.borrow(cs).borrow_mut().take())
}

/// Access ISR state in a critical section (before interrupts take it).
pub fn with_isr_state(f: impl FnOnce(&mut TargetIsrState)) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut state) = *ISR_STATE.borrow(cs).borrow_mut() {
            f(state);
        }
    });
}
