//! MCU initialization — returns configured peripherals.
//!
//! The MCU-specific `init()` functions live in `mcu_xxx/init.rs`.
//! This module provides the shared `InitResult` type and re-exports
//! the active MCU's `init()` so callers can use `rm32_stm32::init::init()`.

use rm32::hal::System;

use crate::mcu::BemfComp;
use crate::timer::{Tim2Interval, Tim14Com};
use crate::phase::G0APhaseDriver;

/// Result of MCU initialization — everything main.rs needs, zero cfg.
pub struct InitResult<SYS, ADC, TELEM>
where
    SYS: System,
    ADC: rm32::hal::Adc,
    TELEM: rm32::hal::TelemetryUart,
{
    pub hal: crate::isr::TargetIsrHal,
    pub sys: SYS,
    pub adc: ADC,
    pub telem: TELEM,
}

/// Re-export the active MCU's init function.
pub use crate::mcu::init_mcu as init;
