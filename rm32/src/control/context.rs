//! Motor control context — bundles state and HAL for ISR functions.
//!
//! Replaces 10+ individual arguments with a single `MotorContext`.
//! Generic over HAL traits for static dispatch (zero vtable overhead).

use crate::commutation::Commutation;
use crate::config::EepromConfig;
use crate::control::isr_logic::TickCounters;
use crate::control::state::{BemfState, DutyState};
use crate::hal;
use crate::shared_comm::SharedComm;

/// Bundles motor state and HAL hardware for ISR entry points.
///
/// All HAL types are generic — the compiler monomorphizes each ISR function
/// to the concrete MCU types, eliminating vtable overhead in the 20kHz loop.
pub struct MotorContext<'a, S, P, C, Ph, I, T>
where
    S: SharedComm,
    P: hal::PwmOutput,
    C: hal::Comparator,
    Ph: hal::PhaseOutput,
    I: hal::IntervalTimer,
    T: hal::ComTimer,
{
    // Motor state
    pub commutation: &'a mut Commutation,
    pub bemf: &'a mut BemfState,
    pub duty: &'a mut DutyState,
    pub config: &'a EepromConfig,
    pub counters: &'a mut TickCounters,

    // Shared ISR↔main state
    pub shared: &'a S,

    // HAL hardware
    pub pwm: &'a mut P,
    pub comp: &'a mut C,
    pub phase: &'a mut Ph,
    pub interval: &'a mut I,
    pub com_timer: &'a mut T,
}
