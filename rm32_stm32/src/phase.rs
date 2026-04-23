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

// GPIO register offsets (standard for all STM32)
const MODER: u32 = 0x00;
const BSRR: u32 = 0x18;

const GPIOA_BASE: u32 = 0x4800_0000;
#[cfg(feature = "stm32g071")]
const GPIOB_BASE: u32 = 0x4800_0400;
#[cfg(feature = "stm32f051")]
const GPIOB_BASE: u32 = 0x4800_0400;
#[cfg(feature = "stm32l431")]
const GPIOB_BASE: u32 = 0x4800_0400;

/// GPIO mode values (STM32G0 MODER register: 2 bits per pin)
const MODE_INPUT: u32 = 0b00;
const MODE_OUTPUT: u32 = 0b01;
const MODE_ALTERNATE: u32 = 0b10;

/// Phase pin descriptor
struct PhasePin {
    port_is_a: bool, // true = GPIOA, false = GPIOB
    pin: u8,         // pin number 0-15
}

/// 3-phase driver for HARDWARE_GROUP_G0_A.
pub struct PhaseDriver {
    // Pin assignments
    a_high: PhasePin, // PA10
    a_low: PhasePin,  // PB1
    b_high: PhasePin, // PA9
    b_low: PhasePin,  // PB0
    c_high: PhasePin, // PA8
    c_low: PhasePin,  // PA7
    comp_pwm: bool,
}

impl PhaseDriver {
    /// Create phase driver for HARDWARE_GROUP_G0_A pin assignment.
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
    fn port_base(&self, pin: &PhasePin) -> u32 {
        if pin.port_is_a { GPIOA_BASE } else { GPIOB_BASE }
    }

    #[inline(always)]
    fn set_mode(&self, pin: &PhasePin, mode: u32) {
        let base = self.port_base(pin);
        let offset = pin.pin as u32 * 2;
        unsafe {
            let ptr = (base + MODER) as *mut u32;
            let val = ptr.read_volatile();
            ptr.write_volatile((val & !(0b11 << offset)) | (mode << offset));
        }
    }

    #[inline(always)]
    fn set_high(&self, pin: &PhasePin) {
        let base = self.port_base(pin);
        unsafe { ((base + BSRR) as *mut u32).write_volatile(1 << pin.pin); }
    }

    #[inline(always)]
    fn set_low(&self, pin: &PhasePin) {
        let base = self.port_base(pin);
        unsafe { ((base + BSRR) as *mut u32).write_volatile(1 << (pin.pin + 16)); }
    }

    /// Phase PWM: high-side alternate, low-side alternate (comp_pwm) or off.
    #[inline(always)]
    fn phase_pwm(&self, high: &PhasePin, low: &PhasePin) {
        if !self.comp_pwm {
            self.set_mode(low, MODE_OUTPUT);
            self.set_low(low);
        } else {
            self.set_mode(low, MODE_ALTERNATE);
        }
        self.set_mode(high, MODE_ALTERNATE);
    }

    /// Phase LOW: low-side on, high-side off.
    #[inline(always)]
    fn phase_low(&self, high: &PhasePin, low: &PhasePin) {
        self.set_mode(low, MODE_OUTPUT);
        self.set_high(low); // turn on low-side FET
        self.set_mode(high, MODE_OUTPUT);
        self.set_low(high); // turn off high-side FET
    }

    /// Phase FLOAT: both FETs off.
    #[inline(always)]
    fn phase_float(&self, high: &PhasePin, low: &PhasePin) {
        self.set_mode(low, MODE_OUTPUT);
        self.set_low(low); // low-side off
        self.set_mode(high, MODE_OUTPUT);
        self.set_low(high); // high-side off
    }
}

impl PhaseOutput for PhaseDriver {
    fn com_step(&mut self, step: u8) {
        match step {
            1 => { // A-B: A=PWM, B=LOW, C=FLOAT
                self.phase_float(&self.c_high, &self.c_low);
                self.phase_low(&self.b_high, &self.b_low);
                self.phase_pwm(&self.a_high, &self.a_low);
            }
            2 => { // C-B: C=PWM, B=LOW, A=FLOAT
                self.phase_float(&self.a_high, &self.a_low);
                self.phase_low(&self.b_high, &self.b_low);
                self.phase_pwm(&self.c_high, &self.c_low);
            }
            3 => { // C-A: C=PWM, A=LOW, B=FLOAT
                self.phase_float(&self.b_high, &self.b_low);
                self.phase_low(&self.a_high, &self.a_low);
                self.phase_pwm(&self.c_high, &self.c_low);
            }
            4 => { // B-A: B=PWM, A=LOW, C=FLOAT
                self.phase_float(&self.c_high, &self.c_low);
                self.phase_low(&self.a_high, &self.a_low);
                self.phase_pwm(&self.b_high, &self.b_low);
            }
            5 => { // B-C: B=PWM, C=LOW, A=FLOAT
                self.phase_float(&self.a_high, &self.a_low);
                self.phase_low(&self.c_high, &self.c_low);
                self.phase_pwm(&self.b_high, &self.b_low);
            }
            6 => { // A-C: A=PWM, C=LOW, B=FLOAT
                self.phase_float(&self.b_high, &self.b_low);
                self.phase_low(&self.c_high, &self.c_low);
                self.phase_pwm(&self.a_high, &self.a_low);
            }
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
        // All high-side off, all low-side PWM (duty cycle controls braking force)
        self.set_mode(&self.a_high, MODE_OUTPUT);
        self.set_low(&self.a_high);
        self.set_mode(&self.b_high, MODE_OUTPUT);
        self.set_low(&self.b_high);
        self.set_mode(&self.c_high, MODE_OUTPUT);
        self.set_low(&self.c_high);

        self.set_mode(&self.a_low, MODE_ALTERNATE);
        self.set_mode(&self.b_low, MODE_ALTERNATE);
        self.set_mode(&self.c_low, MODE_ALTERNATE);
    }
}
