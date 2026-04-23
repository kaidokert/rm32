//! ISR-exclusive state and interrupt handlers.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rm32::commutation::Commutation;
use rm32::config::EepromConfig;
use rm32::control::state::{BemfState, DutyState};
use rm32::crsf::CrsfParser;
use rm32::dshot_commands::CommandProcessor;
use rm32::edt::EdtScheduler;
use rm32::transfer::TransferState;
use rm32::hal::{PwmOutput, Comparator, PhaseOutput};

use crate::comparator::BemfComparator;
use crate::timer::{Tim2Interval, Tim14Com};
use crate::phase::PhaseDriver;
use crate::shared::SharedState;

#[cfg(feature = "stm32g071")]
use crate::input_capture::DshotCapture;
#[cfg(feature = "stm32g071")]
use crate::pwm::Tim1Pwm;

#[cfg(feature = "stm32f051")]
use crate::init::F051Pwm;
#[cfg(feature = "stm32l431")]
use crate::init::L431Pwm;

/// ISR-exclusive hardware — MCU-generic via cfg.
pub struct IsrHal {
    #[cfg(feature = "stm32g071")]
    pub pwm: Tim1Pwm,
    #[cfg(feature = "stm32f051")]
    pub pwm: F051Pwm,
    #[cfg(feature = "stm32l431")]
    pub pwm: L431Pwm,

    pub comp: BemfComparator,
    pub interval: Tim2Interval,
    pub com_timer: Tim14Com,
    pub phase: PhaseDriver,

    #[cfg(feature = "stm32g071")]
    pub input: DshotCapture,

    #[cfg(feature = "stm32f051")]
    pub input: crate::input_capture_f051::F051DshotCapture,

    #[cfg(feature = "stm32l431")]
    pub input: crate::input_capture_l431::L431DshotCapture,
}

/// ISR-exclusive state.
pub struct IsrState {
    pub commutation: Commutation,
    pub bemf: BemfState,
    pub duty: DutyState,
    pub hal: IsrHal,
    pub cmd: CommandProcessor,
    pub edt: EdtScheduler,
    pub crsf: CrsfParser,
    pub transfer: TransferState,
    pub config: EepromConfig,
    pub forward: bool,
    pub frametime_low: u16,
    pub frametime_high: u16,
    pub ten_khz_counter: u32,
    pub one_khz_loop_counter: u16,
    pub armed_timeout_count: u32,
}

static ISR_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));
static SHARED: SharedState = SharedState::new();

pub fn shared() -> &'static SharedState { &SHARED }

pub fn init_isr_state(state: IsrState) {
    cortex_m::interrupt::free(|cs| {
        ISR_STATE.borrow(cs).replace(Some(state));
    });
}

pub fn take_isr_state() -> Option<IsrState> {
    cortex_m::interrupt::free(|cs| ISR_STATE.borrow(cs).borrow_mut().take())
}

/// Access ISR state in a critical section (before interrupts take it).
pub fn with_isr_state(f: impl FnOnce(&mut IsrState)) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut state) = *ISR_STATE.borrow(cs).borrow_mut() {
            f(state);
        }
    });
}
