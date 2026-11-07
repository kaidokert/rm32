//! ISR-exclusive state and interrupt handlers.
//!
//! `IsrHal` and `IsrState` are generic over HAL types — no cfg blocks.
//! The concrete target type is resolved via `TargetIsrState` alias.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use portable_atomic::{AtomicBool, Ordering};
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
    P: crate::bridge::TimerPwm,
    I: hal::InputCapture,
    C: hal::BemfSensor,
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

impl<P, I, C, IT, CT, PH> rm32::platform::Platform for IsrHal<P, I, C, IT, CT, PH>
where
    P: crate::bridge::TimerPwm,
    I: hal::InputCapture,
    C: hal::BemfSensor,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    const TICK_HZ: u32 = <crate::platform::StmPlatform as rm32::platform::Platform>::TICK_HZ;
    const TIMER_HZ: u32 = <crate::platform::StmPlatform as rm32::platform::Platform>::TIMER_HZ;
    const CAPTURE_HZ: u32 = <crate::platform::StmPlatform as rm32::platform::Platform>::CAPTURE_HZ;
    const GCR_SHIFT: u8 = <crate::platform::StmPlatform as rm32::platform::Platform>::GCR_SHIFT;
    const VOLTAGE_DIVIDER_X100: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::VOLTAGE_DIVIDER_X100;
    const MILLIVOLT_PER_AMP: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::MILLIVOLT_PER_AMP;
    const CURRENT_OFFSET: i16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::CURRENT_OFFSET;
    const USE_NTC: bool = <crate::platform::StmPlatform as rm32::platform::Platform>::USE_NTC;
    const ADC_FULL_SCALE: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::ADC_FULL_SCALE;
    const ADC_VREF_MV: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::ADC_VREF_MV;
    const PWM_BASE_PERIOD: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::PWM_BASE_PERIOD;
}

impl<P, I, C, IT, CT, PH> hal::MotorHal for IsrHal<P, I, C, IT, CT, PH>
where
    P: crate::bridge::TimerPwm,
    I: hal::InputCapture,
    C: hal::BemfSensor,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    type Bridge<'a>
        = crate::bridge::StmBridge<'a, P, PH>
    where
        Self: 'a;
    type Bemf = C;
    type Interval = IT;
    type Com = CT;

    fn bridge(&mut self) -> Self::Bridge<'_> {
        crate::bridge::StmBridge {
            pwm: &mut self.pwm,
            phase: &mut self.phase,
        }
    }
    fn bemf(&mut self) -> &mut C {
        &mut self.comp
    }
    fn interval(&mut self) -> &mut IT {
        &mut self.interval
    }
    fn com_timer(&mut self) -> &mut CT {
        &mut self.com_timer
    }
}

/// Decomposes `IsrHal` into its peripheral components so `isr_handlers` can
/// partition live state into priority-disjoint cells without per-MCU aliases.
pub trait IsrHalParts {
    type Pwm: crate::bridge::TimerPwm;
    type Input: hal::InputCapture;
    type Comp: hal::BemfSensor;
    type Interval: hal::IntervalTimer;
    type Com: hal::ComTimer;
    type Phase: hal::PhaseOutput;

    #[allow(clippy::type_complexity)]
    fn into_parts(
        self,
    ) -> (
        Self::Pwm,
        Self::Input,
        Self::Comp,
        Self::Interval,
        Self::Com,
        Self::Phase,
    );
}

impl<P, I, C, IT, CT, PH> IsrHalParts for IsrHal<P, I, C, IT, CT, PH>
where
    P: crate::bridge::TimerPwm,
    I: hal::InputCapture,
    C: hal::BemfSensor,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    type Pwm = P;
    type Input = I;
    type Comp = C;
    type Interval = IT;
    type Com = CT;
    type Phase = PH;

    #[inline]
    fn into_parts(self) -> (P, I, C, IT, CT, PH) {
        (
            self.pwm,
            self.input,
            self.comp,
            self.interval,
            self.com_timer,
            self.phase,
        )
    }
}

/// Reborrowed motor HAL combining `TickIsrState`'s PWM timer with
/// `ComIsrState`'s comparator, timers, and phase driver inside a masked
/// critical section.
pub struct SplitMotorHal<'a, P, C, IT, CT, PH> {
    pub pwm: &'a mut P,
    pub comp: &'a mut C,
    pub interval: &'a mut IT,
    pub com_timer: &'a mut CT,
    pub phase: &'a mut PH,
}

impl<P, C, IT, CT, PH> rm32::platform::Platform for SplitMotorHal<'_, P, C, IT, CT, PH>
where
    P: crate::bridge::TimerPwm,
    C: hal::BemfSensor,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    const TICK_HZ: u32 = <crate::platform::StmPlatform as rm32::platform::Platform>::TICK_HZ;
    const TIMER_HZ: u32 = <crate::platform::StmPlatform as rm32::platform::Platform>::TIMER_HZ;
    const CAPTURE_HZ: u32 = <crate::platform::StmPlatform as rm32::platform::Platform>::CAPTURE_HZ;
    const GCR_SHIFT: u8 = <crate::platform::StmPlatform as rm32::platform::Platform>::GCR_SHIFT;
    const VOLTAGE_DIVIDER_X100: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::VOLTAGE_DIVIDER_X100;
    const MILLIVOLT_PER_AMP: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::MILLIVOLT_PER_AMP;
    const CURRENT_OFFSET: i16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::CURRENT_OFFSET;
    const USE_NTC: bool = <crate::platform::StmPlatform as rm32::platform::Platform>::USE_NTC;
    const ADC_FULL_SCALE: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::ADC_FULL_SCALE;
    const ADC_VREF_MV: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::ADC_VREF_MV;
    const PWM_BASE_PERIOD: u16 =
        <crate::platform::StmPlatform as rm32::platform::Platform>::PWM_BASE_PERIOD;
}

impl<P, C, IT, CT, PH> hal::MotorHal for SplitMotorHal<'_, P, C, IT, CT, PH>
where
    P: crate::bridge::TimerPwm,
    C: hal::BemfSensor,
    IT: hal::IntervalTimer,
    CT: hal::ComTimer,
    PH: hal::PhaseOutput,
{
    type Bridge<'b>
        = crate::bridge::StmBridge<'b, P, PH>
    where
        Self: 'b;
    type Bemf = C;
    type Interval = IT;
    type Com = CT;

    #[inline(always)]
    fn bridge(&mut self) -> Self::Bridge<'_> {
        crate::bridge::StmBridge {
            pwm: self.pwm,
            phase: self.phase,
        }
    }
    #[inline(always)]
    fn bemf(&mut self) -> &mut C {
        self.comp
    }
    #[inline(always)]
    fn interval(&mut self) -> &mut IT {
        self.interval
    }
    #[inline(always)]
    fn com_timer(&mut self) -> &mut CT {
        self.com_timer
    }
}

/// ISR-exclusive state — generic over hardware.
pub struct IsrState<H> {
    pub commutation: Commutation,
    pub bemf: BemfState,
    pub duty: DutyState,
    pub hal: H,
    pub cmd: CommandProcessor,
    pub tone: rm32::tone::ToneScheduler,
    pub edt: EdtScheduler,
    pub crsf: CrsfParser,
    pub transfer: TransferState,
    pub config: EepromConfig,
    pub forward: bool,
    pub edt_armed: bool,
    pub edt_arm_enable: bool,
    pub armed_timeout_count: u32,
    pub frametime_low: u16,
    pub frametime_high: u16,
    pub voltage_based_ramp: bool,
    pub writer: rm32::shared_comm::SingleWriter,
}

// Target-specific type alias — defined in mcu_xxx/chip.rs, re-exported via mcu::*.
pub use crate::mcu::TargetIsrHal;

/// Concrete ISR state for the selected target.
pub type TargetIsrState = IsrState<TargetIsrHal>;

// ============================================================
// Global state + accessors
// ============================================================

static ISR_STATE: Mutex<RefCell<core::mem::MaybeUninit<TargetIsrState>>> =
    Mutex::new(RefCell::new(core::mem::MaybeUninit::uninit()));
static ISR_STATE_STAGED: AtomicBool = AtomicBool::new(false);
static SHARED: SharedState = SharedState::new::<crate::platform::StmPlatform>();

pub fn shared() -> &'static SharedState {
    &SHARED
}

pub fn init_isr_state(state: TargetIsrState) {
    cortex_m::interrupt::free(|cs| {
        let mut slot = ISR_STATE.borrow(cs).borrow_mut();
        if ISR_STATE_STAGED.load(Ordering::Relaxed) {
            // SAFETY: `ISR_STATE_STAGED` is only set after `slot.write(state)`
            // and cleared before `assume_init_read()`.
            unsafe { slot.assume_init_drop() };
        }
        slot.write(state);
        ISR_STATE_STAGED.store(true, Ordering::Release);
    });
}

#[inline(always)]
pub fn is_isr_state_staged() -> bool {
    ISR_STATE_STAGED.load(Ordering::Acquire)
}

pub fn take_isr_state() -> Option<TargetIsrState> {
    cortex_m::interrupt::free(|cs| {
        if ISR_STATE_STAGED.swap(false, Ordering::AcqRel) {
            let slot = ISR_STATE.borrow(cs).borrow_mut();
            // SAFETY: `ISR_STATE_STAGED` was true and has just been atomically
            // cleared inside `interrupt::free`, transferring ownership out.
            Some(unsafe { slot.assume_init_read() })
        } else {
            None
        }
    })
}

/// Access ISR state in a critical section (before interrupts take it).
pub fn with_isr_state(f: impl FnOnce(&mut TargetIsrState)) {
    cortex_m::interrupt::free(|cs| {
        if ISR_STATE_STAGED.load(Ordering::Acquire) {
            let mut slot = ISR_STATE.borrow(cs).borrow_mut();
            // SAFETY: `ISR_STATE_STAGED` is true and interrupts are masked.
            f(unsafe { slot.assume_init_mut() });
        }
    });
}
