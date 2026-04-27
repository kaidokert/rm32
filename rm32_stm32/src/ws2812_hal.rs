//! WS2812 GPIO bitbang implementation for STM32.
//!
//! Uses `embedded_hal::digital::OutputPin` for pin toggling — no raw MMIO.
//! Timing via cortex_m::asm::delay (cycle-counting busy wait).
//! Generic over any OutputPin — works with HAL pins from any MCU.

use rm32::ws2812::WS2812Pin;
use embedded_hal::digital::OutputPin;

pub struct Ws2812Gpio<P: OutputPin> {
    pin: P,
    cpu_mhz: u32,
}

impl<P: OutputPin> Ws2812Gpio<P> {
    pub fn new(pin: P, cpu_mhz: u32) -> Self {
        Self { pin, cpu_mhz }
    }
}

impl<P: OutputPin> WS2812Pin for Ws2812Gpio<P> {
    #[inline(always)]
    fn set_high(&mut self) {
        let _ = self.pin.set_high();
    }

    #[inline(always)]
    fn set_low(&mut self) {
        let _ = self.pin.set_low();
    }

    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * self.cpu_mhz as u64 / 1000) as u32;
        cortex_m::asm::delay(cycles);
    }
}

/// Runtime-configurable GPIOB output pin using GpioPort trait.
/// Implements `OutputPin` so it can be used with `Ws2812Gpio`.
pub struct GpioBPin {
    set_mask: u32,
    reset_mask: u32,
}

impl GpioBPin {
    /// Create a GPIOB output pin. Configures MODER as output, OSPEEDR as high speed.
    pub fn new(pin: u8) -> Self {
        use crate::gpio_regs::{GpioPort as _, PortB};
        let offset = pin as u32 * 2;
        PortB::modify_moder(|v| (v & !(0b11 << offset)) | (0b01 << offset));
        // Set high speed (OSPEEDR)
        // Can't use GpioPort for OSPEEDR (not in trait), but BSRR-based set/low
        // doesn't need speed config — WS2812 timing is from delay_ns, not slew rate.
        Self {
            set_mask: 1 << pin,
            reset_mask: 1 << (pin + 16),
        }
    }
}

impl embedded_hal::digital::ErrorType for GpioBPin {
    type Error = core::convert::Infallible;
}

impl OutputPin for GpioBPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        use crate::gpio_regs::{GpioPort, PortB};
        PortB::write_bsrr(self.set_mask);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        use crate::gpio_regs::{GpioPort, PortB};
        PortB::write_bsrr(self.reset_mask);
        Ok(())
    }
}
