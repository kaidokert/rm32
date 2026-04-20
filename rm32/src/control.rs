//! Main control loop state and logic.
//!
//! This module contains the motor state machine:
//! - Arming/disarming
//! - tenKhzRoutine equivalent (ramp, duty cycle, PID)
//! - main_loop equivalent (desync, eRPM, LVC, telemetry triggers)

pub mod state;
pub mod tick;
#[cfg(test)]
mod tests;
