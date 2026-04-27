//! Compile-time GPIO pin identity for phase commutation.
//!
//! Each pin is a zero-sized type with associated constants for port base
//! address and pin number. Runtime register access uses these constants
//! directly — no branches, no indirection.
//!
//! MODER is at offset 0x00 and BSRR at offset 0x18 on all STM32 families.
//! No #[cfg] needed.

use crate::periph_addr;

/// A GPIO pin known at compile time.
pub trait GpioPin {
    /// Port base address (e.g. GPIOA = 0x4800_0000).
    const PORT: u32;
    /// Pin number (0..15).
    const PIN: u8;

    /// Precomputed MODER bit offset (pin * 2).
    const MODER_OFFSET: u32 = Self::PIN as u32 * 2;
    /// Precomputed MODER two-bit mask.
    const MODER_MASK: u32 = 0b11 << Self::MODER_OFFSET;
    /// Precomputed BSRR set bit.
    const BSRR_SET: u32 = 1 << Self::PIN as u32;
    /// Precomputed BSRR reset bit.
    const BSRR_RESET: u32 = 1 << (Self::PIN as u32 + 16);

    /// Set GPIO mode for this pin (output, alternate, etc).
    #[inline(always)]
    fn set_mode(mode: u32) {
        // SAFETY: PORT is a valid GPIO base address (from periph_addr).
        // MODER is at offset 0x00 on all STM32 families.
        // Read-modify-write is safe: called from ISR at fixed priority (no preemption).
        let moder = Self::PORT as *mut u32;
        unsafe {
            let val = moder.read_volatile();
            moder.write_volatile((val & !Self::MODER_MASK) | (mode << Self::MODER_OFFSET));
        }
    }

    /// Set pin high via BSRR (atomic, write-only).
    #[inline(always)]
    fn set_high() {
        // SAFETY: BSRR at offset 0x18 is write-only, bit-atomic — no read-modify-write needed.
        unsafe { ((Self::PORT + 0x18) as *mut u32).write_volatile(Self::BSRR_SET); }
    }

    /// Set pin low via BSRR reset bits (atomic, write-only).
    #[inline(always)]
    fn set_low() {
        // SAFETY: BSRR at offset 0x18 is write-only, bit-atomic.
        unsafe { ((Self::PORT + 0x18) as *mut u32).write_volatile(Self::BSRR_RESET); }
    }
}

// --- Pin definitions for motor phase outputs ---

pub struct PA7;
impl GpioPin for PA7 { const PORT: u32 = periph_addr::GPIOA; const PIN: u8 = 7; }

pub struct PA8;
impl GpioPin for PA8 { const PORT: u32 = periph_addr::GPIOA; const PIN: u8 = 8; }

pub struct PA9;
impl GpioPin for PA9 { const PORT: u32 = periph_addr::GPIOA; const PIN: u8 = 9; }

pub struct PA10;
impl GpioPin for PA10 { const PORT: u32 = periph_addr::GPIOA; const PIN: u8 = 10; }

pub struct PB0;
impl GpioPin for PB0 { const PORT: u32 = periph_addr::GPIOB; const PIN: u8 = 0; }

pub struct PB1;
impl GpioPin for PB1 { const PORT: u32 = periph_addr::GPIOB; const PIN: u8 = 1; }

pub struct PB10;
impl GpioPin for PB10 { const PORT: u32 = periph_addr::GPIOB; const PIN: u8 = 10; }
