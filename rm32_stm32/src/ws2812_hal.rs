//! WS2812 GPIO bitbang implementation for STM32.
//!
//! Uses raw GPIO BSRR register for fast pin toggling.
//! Timing via cortex_m::asm::delay (cycle-counting busy wait).
//!
//! Default pin: PB8 (most common across AM32 boards).
//! Pin is configurable per board.

use rm32::ws2812::WS2812Pin;

const GPIOB_BASE: u32 = 0x4800_0400;
const BSRR: u32 = 0x18;
const MODER: u32 = 0x00;
const OSPEEDR: u32 = 0x08;

pub struct Ws2812Gpio {
    pin: u8,
    /// CPU cycles per nanosecond (e.g. 64 for 64MHz = 64 cycles/µs ≈ 0.064 cycles/ns)
    /// We store MHz directly and compute: delay_cycles = ns * mhz / 1000
    cpu_mhz: u32,
}

impl Ws2812Gpio {
    /// Create WS2812 driver on GPIOB pin `pin` at `cpu_mhz` MHz.
    pub fn new(pin: u8, cpu_mhz: u32) -> Self {
        // Configure pin as push-pull output, high speed
        unsafe {
            let moder = GPIOB_BASE as *mut u32;
            let offset = pin as u32 * 2;
            let v = moder.read_volatile();
            moder.write_volatile((v & !(0b11 << offset)) | (0b01 << offset)); // output

            let ospeedr = (GPIOB_BASE + OSPEEDR) as *mut u32;
            let v = ospeedr.read_volatile();
            ospeedr.write_volatile(v | (0b11 << offset)); // very high speed
        }
        Self { pin, cpu_mhz }
    }
}

impl WS2812Pin for Ws2812Gpio {
    #[inline(always)]
    fn set_high(&mut self) {
        unsafe {
            ((GPIOB_BASE + BSRR) as *mut u32).write_volatile(1 << self.pin);
        }
    }

    #[inline(always)]
    fn set_low(&mut self) {
        unsafe {
            ((GPIOB_BASE + BSRR) as *mut u32).write_volatile(1 << (self.pin + 16));
        }
    }

    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        // cycles = ns * mhz / 1000
        let cycles = (ns as u64 * self.cpu_mhz as u64 / 1000) as u32;
        cortex_m::asm::delay(cycles);
    }
}
