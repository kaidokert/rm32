//! Main control loop state and logic.
//!
//! This module contains the motor state machine:
//! - Arming/disarming
//! - tenKhzRoutine equivalent (ramp, duty cycle, PID)
//! - main_loop equivalent (desync, eRPM, LVC, telemetry triggers)

pub mod context;
pub mod input;
pub mod isr_logic;
#[cfg(test)]
pub mod shared_impl;
pub mod state;
#[cfg(test)]
mod tests;
