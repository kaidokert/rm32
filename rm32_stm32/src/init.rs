//! MCU initialization — returns configured peripherals.
//!
//! The MCU-specific `init()` functions live in `mcu_xxx/init.rs`.
//! This module provides the shared `InitResult` type and re-exports
//! the active MCU's `init()` so callers can use `rm32_stm32::init::init()`.

use rm32::hal::System;

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
    /// The commutation ISR's plain-store token, minted after the NVIC
    /// priorities are set ([`single_writer`]).
    pub writer: rm32::shared_comm::SingleWriter,
}

/// Mint the commutation ISR's [`rm32::shared_comm::SingleWriter`].
///
/// Why the plain stores are sound on these parts (the only place this
/// argument holds -- other ports use `AtomicWriter`):
///
/// - Single core, one NVIC. The commutation-timer ISR sits at the highest
///   preemption level among the contexts that write `motor_mode` or
///   `zero_crosses` (the COMP ISR shares its level and cannot preempt it;
///   the control tick, the frame-processing EXTI and the main loop are
///   below it). So nothing that writes those words runs between its load
///   and its store.
/// - The lower-priority writers use read-modify-write updates that a
///   preempting store invalidates. M4 (L431, G431): `LDREX`/`STREX`, and
///   exception entry and return clear the local exclusive monitor, so a
///   preempted `STREX` fails and retries against the stored value. M0
///   (G071, F051): no exclusives; the atomics are critical sections with
///   interrupts masked, so the ISR cannot land inside one.
/// - The DShot DMA ISR may outrank the commutation timer (L431
///   `adjust_irq_priorities` swaps them for bidirectional telemetry) and
///   it writes neither word.
///
/// # Safety
///
/// Call once, after the chip's `init` has set the NVIC priorities, and
/// keep the layout above for the token's lifetime.
pub(crate) unsafe fn single_writer() -> rm32::shared_comm::SingleWriter {
    // SAFETY: the conditions above, which the caller upholds.
    unsafe { rm32::shared_comm::SingleWriter::new() }
}

/// Re-export the active MCU's init function.
pub use crate::mcu::init_mcu as init;
