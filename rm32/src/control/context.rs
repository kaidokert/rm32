//! Motor control context — bundles state and HAL for ISR functions.
//!
//! Replaces 10+ individual arguments with a single `MotorContext`.
//! Generic over `S: SharedComm` and `H: MotorHal` for static dispatch.

use crate::commutation::Commutation;
use crate::config::EepromConfig;
use crate::control::isr_logic::TickCounters;
use crate::control::state::{BemfState, DutyState};
use crate::hal::MotorHal;
use crate::shared_comm::SharedComm;

/// Bundles motor state and HAL hardware for ISR entry points.
///
/// `H: MotorHal` bundles the 5 motor peripherals (PWM, comparator, phase,
/// interval timer, commutation timer) into a single type parameter —
/// the compiler monomorphizes to the concrete MCU types at compile time.
pub struct MotorContext<'a, S: SharedComm, H: MotorHal> {
    // Motor state
    pub commutation: &'a mut Commutation,
    pub bemf: &'a mut BemfState,
    pub duty: &'a mut DutyState,
    pub config: &'a EepromConfig,
    pub counters: &'a mut TickCounters,

    // Shared ISR↔main state
    pub shared: &'a S,

    // HAL hardware bundle
    pub hal: &'a mut H,
}
