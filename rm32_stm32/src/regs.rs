//! Initialization utilities — error types and timeout helpers.

/// Hardware initialization error with subsystem identification.
#[derive(Debug, Clone, Copy)]
pub enum InitError {
    /// A hardware flag didn't reach expected state within timeout.
    Timeout(&'static str),
    /// ADC calibration or enable failed.
    AdcInit(&'static str),
    /// Clock PLL lock or switch failed.
    ClockInit(&'static str),
    /// UART handshake failed.
    UartInit(&'static str),
    /// Flash unlock or erase failed.
    FlashError(&'static str),
}

/// Spin-wait for a condition with a cycle-counted timeout.
/// Returns `Ok(())` if condition becomes true, `Err(InitError::Timeout)` if not.
#[inline]
pub fn wait_for(
    mut condition: impl FnMut() -> bool,
    timeout_cycles: u32,
    name: &'static str,
) -> Result<(), InitError> {
    let mut count = 0u32;
    while !condition() {
        count += 1;
        if count >= timeout_cycles {
            return Err(InitError::Timeout(name));
        }
        cortex_m::asm::nop();
    }
    Ok(())
}
