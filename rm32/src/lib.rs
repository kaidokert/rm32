//! RM32 — Rust reimplementation of AM32 brushless ESC firmware.
//!
//! This is the core no_std, no-alloc library containing all motor control logic.
//! Hardware interaction is abstracted via the [`hal`] trait module.

#![no_std]

pub mod bemf;
pub mod board;
pub mod brushed;
pub mod commutation;
pub mod config;
pub mod constants;
pub mod control;
pub mod crsf;
pub mod current;
pub mod dshot;
pub mod dshot_commands;
pub mod edt;
pub mod eeprom;
pub mod filter;
pub mod fixed_mode;
pub mod functions;
pub mod hal;
pub mod motor_mode;
pub mod ntc;
pub mod pid;
pub mod polling;
pub mod servo;
pub mod shared_comm;
pub mod signal;
pub mod sine;
pub mod sounds;
pub mod telemetry;
pub mod transfer;
pub mod units;
pub mod ws2812;
