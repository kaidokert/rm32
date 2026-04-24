//! Raw register access utilities.
//!
//! Consolidates the read/write/modify volatile register operations
//! used across all MCU peripheral modules. Keeps unsafe in one place.

#[inline(always)]
pub unsafe fn write(addr: u32, val: u32) {
    (addr as *mut u32).write_volatile(val);
}

#[inline(always)]
pub unsafe fn read(addr: u32) -> u32 {
    (addr as *const u32).read_volatile()
}

#[inline(always)]
pub unsafe fn modify(addr: u32, f: impl FnOnce(u32) -> u32) {
    let ptr = addr as *mut u32;
    ptr.write_volatile(f(ptr.read_volatile()));
}

/// Write to a register at base + offset.
#[inline(always)]
pub unsafe fn write_off(base: u32, offset: u32, val: u32) {
    ((base + offset) as *mut u32).write_volatile(val);
}

/// Read from a register at base + offset.
#[inline(always)]
pub unsafe fn read_off(base: u32, offset: u32) -> u32 {
    ((base + offset) as *const u32).read_volatile()
}

/// Modify a register at base + offset.
#[inline(always)]
pub unsafe fn modify_off(base: u32, offset: u32, f: impl FnOnce(u32) -> u32) {
    let ptr = (base + offset) as *mut u32;
    ptr.write_volatile(f(ptr.read_volatile()));
}

/// Hardware initialization error.
#[derive(Debug, Clone, Copy)]
pub enum InitError {
    /// A hardware flag didn't reach expected state within timeout.
    Timeout(&'static str),
}

/// Spin-wait for a condition with a cycle-counted timeout.
/// Returns `Ok(())` if condition becomes true, `Err(InitError::Timeout)` if not.
#[inline]
pub fn wait_for(mut condition: impl FnMut() -> bool, timeout_cycles: u32, name: &'static str) -> Result<(), InitError> {
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
