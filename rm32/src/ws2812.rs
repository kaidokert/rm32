//! WS2812 single-LED driver — platform-independent bitbang logic.
//!
//! Drives one WS2812 LED with 24-bit GRB color data.
//! The actual GPIO toggling and timing is provided by a HAL trait.
//! Interrupts should be disabled during `send_rgb()` (~24µs).

/// HAL interface for WS2812 bitbang output.
pub trait WS2812Pin {
    fn set_high(&mut self);
    fn set_low(&mut self);
    /// Busy-wait for approximately `ns` nanoseconds.
    fn delay_ns(&mut self, ns: u32);
}

/// LED status colors used by the firmware.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LedStatus {
    /// Dim red — power on, waiting for input
    Boot,
    /// Bright green — armed and ready
    Armed,
    /// Red — stuck rotor / error
    Error,
    /// Off
    Off,
}

impl LedStatus {
    /// Get (R, G, B) tuple for this status.
    pub fn rgb(self) -> (u8, u8, u8) {
        match self {
            LedStatus::Boot => (125, 0, 0),
            LedStatus::Armed => (0, 255, 0),
            LedStatus::Error => (255, 0, 0),
            LedStatus::Off => (0, 0, 0),
        }
    }
}

/// Send one WS2812 bit via bitbang.
/// WS2812 timing (±150ns tolerance):
///   bit 0: HIGH ~350ns, LOW ~800ns
///   bit 1: HIGH ~700ns, LOW ~600ns
#[inline(always)]
fn send_bit(pin: &mut dyn WS2812Pin, bit: bool) {
    if bit {
        pin.set_high();
        pin.delay_ns(700);
        pin.set_low();
        pin.delay_ns(600);
    } else {
        pin.set_high();
        pin.delay_ns(350);
        pin.set_low();
        pin.delay_ns(800);
    }
}

/// Send a single byte (MSB first) to the WS2812.
#[inline(always)]
fn send_byte(pin: &mut dyn WS2812Pin, byte: u8) {
    for i in (0..8).rev() {
        send_bit(pin, byte & (1 << i) != 0);
    }
}

/// Send RGB color to a single WS2812 LED.
/// WS2812 expects GRB order.
///
/// **Interrupts should be disabled by the caller** for the duration
/// of this call (~24µs for 24 bits).
pub fn send_rgb(pin: &mut dyn WS2812Pin, r: u8, g: u8, b: u8) {
    send_byte(pin, g); // WS2812 is GRB order
    send_byte(pin, r);
    send_byte(pin, b);
    // Reset: hold LOW for >50µs (caller's responsibility, or just wait)
    pin.set_low();
    pin.delay_ns(50_000);
}

/// Send a status color to the LED.
pub fn send_status(pin: &mut dyn WS2812Pin, status: LedStatus) {
    let (r, g, b) = status.rgb();
    send_rgb(pin, r, g, b);
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FakePin {
        call_count: u32,
    }

    impl FakePin {
        fn new() -> Self {
            Self { call_count: 0 }
        }
    }

    impl WS2812Pin for FakePin {
        fn set_high(&mut self) {
            self.call_count += 1;
        }
        fn set_low(&mut self) {
            self.call_count += 1;
        }
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn send_rgb_produces_correct_call_count() {
        let mut pin = FakePin::new();
        send_rgb(&mut pin, 0xFF, 0x00, 0x55);
        // 24 bits × 2 (high+low) + 1 final low for reset = 49
        assert_eq!(pin.call_count, 49);
    }

    #[test]
    fn status_colors() {
        assert_eq!(LedStatus::Boot.rgb(), (125, 0, 0));
        assert_eq!(LedStatus::Armed.rgb(), (0, 255, 0));
        assert_eq!(LedStatus::Error.rgb(), (255, 0, 0));
        assert_eq!(LedStatus::Off.rgb(), (0, 0, 0));
    }
}
