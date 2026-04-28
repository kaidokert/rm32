//! Compile-time GPIO pin identity for phase commutation.
//!
//! Each pin is a zero-sized type associating a `GpioPort` and pin number.
//! All register access via `GpioPort` trait — zero `unsafe` at call sites,
//! zero runtime overhead (static dispatch, monomorphized to constants).

use crate::gpio_regs::GpioPort;
use crate::mcu::{PortA, PortB};

/// A GPIO pin known at compile time.
pub trait GpioPin {
    /// The port this pin belongs to.
    type Port: GpioPort;
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
    /// Safe: GpioPort handles the unsafe register access internally.
    #[inline]
    fn set_mode(mode: u32) {
        Self::Port::modify_moder(|v| (v & !Self::MODER_MASK) | (mode << Self::MODER_OFFSET));
    }

    /// Set pin high via BSRR (atomic, write-only).
    #[inline]
    fn set_high() {
        Self::Port::write_bsrr(Self::BSRR_SET);
    }

    /// Set pin low via BSRR reset bits (atomic, write-only).
    #[inline]
    fn set_low() {
        Self::Port::write_bsrr(Self::BSRR_RESET);
    }
}

// --- Pin definitions ---

macro_rules! gpio_pin {
    ($name:ident, $port:ty, $pin:literal) => {
        pub struct $name;
        impl GpioPin for $name {
            type Port = $port;
            const PIN: u8 = $pin;
        }
    };
}

gpio_pin!(PA7, PortA, 7);
gpio_pin!(PA8, PortA, 8);
gpio_pin!(PA9, PortA, 9);
gpio_pin!(PA10, PortA, 10);
gpio_pin!(PB0, PortB, 0);
gpio_pin!(PB1, PortB, 1);
gpio_pin!(PB10, PortB, 10);
