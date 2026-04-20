//! RM32 — Rust reimplementation of AM32 brushless ESC firmware.
//!
//! This is the core no_std, no-alloc library containing all motor control logic.
//! Hardware interaction is abstracted via the [`hal`] trait module.

#![no_std]

pub mod hal;
pub mod config;
pub mod commutation;
pub mod control;
pub mod dshot;
pub mod signal;
pub mod telemetry;
pub mod pid;
pub mod functions;
pub mod bemf;
pub mod current;
pub mod sine;
pub mod dshot_commands;
pub mod eeprom;
