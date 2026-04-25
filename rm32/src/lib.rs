//! RM32 — Rust reimplementation of AM32 brushless ESC firmware.
//!
//! This is the core no_std, no-alloc library containing all motor control logic.
//! Hardware interaction is abstracted via the [`hal`] trait module.

#![no_std]

pub mod hal;
pub mod config;
pub mod constants;
pub mod units;
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
pub mod sounds;
pub mod polling;
pub mod board;
pub mod servo;
pub mod transfer;
pub mod filter;
pub mod edt;
pub mod crsf;
pub mod shared_comm;
pub mod motor_mode;
pub mod ws2812;
pub mod fixed_mode;
pub mod brushed;
