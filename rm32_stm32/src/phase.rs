//! Motor phase output control (6-step commutation via GPIO mode switching).
//!
//! Each phase has a high-side and low-side FET controlled by GPIO pins.
//! Three states per phase:
//! - PWM: high-side in alternate function (TIM1 drives it), low-side complementary or off
//! - LOW: low-side on (output high), high-side off (output low)
//! - FLOAT: both off (both outputs low) — phase disconnected for BEMF sensing
//!
//! Pin assignments for HARDWARE_GROUP_G0_A:
//!   Phase A: high=PA10, low=PB1
//!   Phase B: high=PA9,  low=PB0
//!   Phase C: high=PA8,  low=PA7

use rm32::hal::PhaseOutput;
use crate::pac::{GPIOA, GPIOB};

/// GPIO MODER values: 2 bits per pin
const MODE_OUTPUT: u32 = 0b01;
const MODE_ALTERNATE: u32 = 0b10;

/// Phase pin descriptor — port reference + pin number.
struct PhasePin {
    /// True = GPIOA, false = GPIOB
    port_is_a: bool,
    pin: u8,
}

impl PhasePin {
    #[inline(always)]
    fn moder_ptr(&self) -> *mut u32 {
        // MODER is at offset 0x00 from GPIO base — same as the PAC's RegisterBlock start
        if self.port_is_a { GPIOA::PTR as *mut u32 } else { GPIOB::PTR as *mut u32 }
    }

    #[inline(always)]
    fn bsrr_ptr(&self) -> *mut u32 {
        // BSRR is at offset 0x18
        if self.port_is_a {
            (GPIOA::PTR as u32 + 0x18) as *mut u32
        } else {
            (GPIOB::PTR as u32 + 0x18) as *mut u32
        }
    }

    /// Set MODER bits for this pin.
    #[inline(always)]
    fn set_mode(&self, mode: u32) {
        let offset = self.pin as u32 * 2;
        unsafe {
            let ptr = self.moder_ptr();
            ptr.write_volatile((ptr.read_volatile() & !(0b11 << offset)) | (mode << offset));
        }
    }

    /// Set pin high via BSRR (write-only, no read needed).
    #[inline(always)]
    fn set_high(&self) {
        unsafe { self.bsrr_ptr().write_volatile(1 << self.pin); }
    }

    /// Set pin low via BSRR reset bits.
    #[inline(always)]
    fn set_low(&self) {
        unsafe { self.bsrr_ptr().write_volatile(1 << (self.pin + 16)); }
    }
}

/// 3-phase driver for HARDWARE_GROUP_G0_A.
pub struct PhaseDriver {
    a_high: PhasePin, // PA10
    a_low: PhasePin,  // PB1
    b_high: PhasePin, // PA9
    b_low: PhasePin,  // PB0
    c_high: PhasePin, // PA8
    c_low: PhasePin,  // PA7
    comp_pwm: bool,
}

impl PhaseDriver {
    pub fn new_g0_a(comp_pwm: bool) -> Self {
        Self {
            a_high: PhasePin { port_is_a: true, pin: 10 },
            a_low: PhasePin { port_is_a: false, pin: 1 },
            b_high: PhasePin { port_is_a: true, pin: 9 },
            b_low: PhasePin { port_is_a: false, pin: 0 },
            c_high: PhasePin { port_is_a: true, pin: 8 },
            c_low: PhasePin { port_is_a: true, pin: 7 },
            comp_pwm,
        }
    }

    #[inline(always)]
    fn phase_pwm(&self, high: &PhasePin, low: &PhasePin) {
        if !self.comp_pwm {
            low.set_mode(MODE_OUTPUT);
            low.set_low();
        } else {
            low.set_mode(MODE_ALTERNATE);
        }
        high.set_mode(MODE_ALTERNATE);
    }

    #[inline(always)]
    fn phase_low(&self, high: &PhasePin, low: &PhasePin) {
        low.set_mode(MODE_OUTPUT);
        low.set_high();
        high.set_mode(MODE_OUTPUT);
        high.set_low();
    }

    #[inline(always)]
    fn phase_float(&self, high: &PhasePin, low: &PhasePin) {
        low.set_mode(MODE_OUTPUT);
        low.set_low();
        high.set_mode(MODE_OUTPUT);
        high.set_low();
    }
}

impl PhaseOutput for PhaseDriver {
    fn com_step(&mut self, step: u8) {
        match step {
            1 => { self.phase_float(&self.c_high, &self.c_low); self.phase_low(&self.b_high, &self.b_low); self.phase_pwm(&self.a_high, &self.a_low); }
            2 => { self.phase_float(&self.a_high, &self.a_low); self.phase_low(&self.b_high, &self.b_low); self.phase_pwm(&self.c_high, &self.c_low); }
            3 => { self.phase_float(&self.b_high, &self.b_low); self.phase_low(&self.a_high, &self.a_low); self.phase_pwm(&self.c_high, &self.c_low); }
            4 => { self.phase_float(&self.c_high, &self.c_low); self.phase_low(&self.a_high, &self.a_low); self.phase_pwm(&self.b_high, &self.b_low); }
            5 => { self.phase_float(&self.a_high, &self.a_low); self.phase_low(&self.c_high, &self.c_low); self.phase_pwm(&self.b_high, &self.b_low); }
            6 => { self.phase_float(&self.b_high, &self.b_low); self.phase_low(&self.c_high, &self.c_low); self.phase_pwm(&self.a_high, &self.a_low); }
            _ => {}
        }
    }

    fn all_off(&mut self) {
        self.phase_float(&self.a_high, &self.a_low);
        self.phase_float(&self.b_high, &self.b_low);
        self.phase_float(&self.c_high, &self.c_low);
    }

    fn full_brake(&mut self) {
        self.phase_low(&self.a_high, &self.a_low);
        self.phase_low(&self.b_high, &self.b_low);
        self.phase_low(&self.c_high, &self.c_low);
    }

    fn all_pwm(&mut self) {
        self.phase_pwm(&self.a_high, &self.a_low);
        self.phase_pwm(&self.b_high, &self.b_low);
        self.phase_pwm(&self.c_high, &self.c_low);
    }

    fn proportional_brake(&mut self) {
        self.a_high.set_mode(MODE_OUTPUT); self.a_high.set_low();
        self.b_high.set_mode(MODE_OUTPUT); self.b_high.set_low();
        self.c_high.set_mode(MODE_OUTPUT); self.c_high.set_low();
        self.a_low.set_mode(MODE_ALTERNATE);
        self.b_low.set_mode(MODE_ALTERNATE);
        self.c_low.set_mode(MODE_ALTERNATE);
    }
}
