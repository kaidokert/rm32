//! Motor phase output control (6-step commutation via GPIO mode switching).
//!
//! PhaseDriver is generic over 6 pin types — all pin identity is resolved
//! at compile time. Runtime methods contain no branches on port or pin number.
//!
//! Pin assignments for HARDWARE_GROUP_G0_A:
//!   Phase A: high=PA10, low=PB1
//!   Phase B: high=PA9,  low=PB0
//!   Phase C: high=PA8,  low=PA7

use crate::gpio_pin::GpioPin;
use crate::gpio_regs::GpioPort;
use core::marker::PhantomData;
use rm32::hal::PhaseOutput;

/// GPIO MODER values.
const MODE_OUTPUT: u32 = 0b01;
const MODE_ALTERNATE: u32 = 0b10;

/// Pulse output toggle function — stored as fn pointer to avoid storing raw addresses.
/// Monomorphized per pin type at `enable_pulse_output` call site.
type PulseToggleFn = fn(u32);

/// 3-phase driver, parameterized by 6 compile-time pin types.
///
/// AH/AL = Phase A high/low, BH/BL = Phase B, CH/CL = Phase C.
/// After monomorphization, all port/pin constants are inlined — zero overhead.
pub struct PhaseDriver<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin>
{
    comp_pwm: bool,
    /// PWM/enable bridge mode: low-side pins are enable (output high/low)
    /// instead of complementary PWM (alternate mode).
    bridge_enable: bool,
    /// RPM pulse output toggle function + pin mask.
    pulse: Option<(PulseToggleFn, u32)>,
    _pins: PhantomData<(AH, AL, BH, BL, CH, CL)>,
}

impl<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin>
    PhaseDriver<AH, AL, BH, BL, CH, CL>
{
    pub fn new(comp_pwm: bool) -> Self {
        Self {
            comp_pwm,
            bridge_enable: false,
            pulse: None,
            _pins: PhantomData,
        }
    }

    pub fn new_bridge(comp_pwm: bool) -> Self {
        Self {
            comp_pwm,
            bridge_enable: true,
            pulse: None,
            _pins: PhantomData,
        }
    }

    /// Enable RPM pulse output on the given pin.
    /// Creates a monomorphized toggle function for the pin's port.
    pub fn enable_pulse_output<P: GpioPin>(&mut self) {
        P::set_mode(MODE_OUTPUT);
        P::set_low();
        // Capture the port's ODR toggle as a monomorphized fn pointer.
        fn toggle<Port: GpioPort>(mask: u32) {
            Port::write_odr(Port::read_odr() ^ mask);
        }
        self.pulse = Some((toggle::<P::Port>, P::BSRR_SET));
    }

    /// Phase PWM: high-side alternate (TIM1), low-side depends on mode.
    ///
    /// Normal: low-side alternate (comp_pwm) or output LOW.
    /// Bridge: enable pin output HIGH (comp_pwm) or no-op.
    #[inline]
    fn phase_pwm<H: GpioPin, L: GpioPin>(&self) {
        if self.bridge_enable {
            if self.comp_pwm {
                L::set_mode(MODE_OUTPUT);
                L::set_high(); // enable on
            }
        } else if !self.comp_pwm {
            L::set_mode(MODE_OUTPUT);
            L::set_low();
        } else {
            L::set_mode(MODE_ALTERNATE);
        }
        H::set_mode(MODE_ALTERNATE);
    }

    /// Phase LOW: low-side/enable on, high-side/PWM off.
    #[inline]
    fn phase_low<H: GpioPin, L: GpioPin>() {
        L::set_mode(MODE_OUTPUT);
        L::set_high();
        H::set_mode(MODE_OUTPUT);
        H::set_low();
    }

    /// Phase FLOAT: both FETs off / enable off.
    #[inline]
    fn phase_float<H: GpioPin, L: GpioPin>() {
        L::set_mode(MODE_OUTPUT);
        L::set_low();
        H::set_mode(MODE_OUTPUT);
        H::set_low();
    }
}

impl<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin> PhaseOutput
    for PhaseDriver<AH, AL, BH, BL, CH, CL>
{
    fn com_step(&mut self, step: u8) {
        match step {
            1 => {
                Self::phase_float::<CH, CL>();
                Self::phase_low::<BH, BL>();
                self.phase_pwm::<AH, AL>();
            }
            2 => {
                Self::phase_float::<AH, AL>();
                Self::phase_low::<BH, BL>();
                self.phase_pwm::<CH, CL>();
            }
            3 => {
                Self::phase_float::<BH, BL>();
                Self::phase_low::<AH, AL>();
                self.phase_pwm::<CH, CL>();
            }
            4 => {
                Self::phase_float::<CH, CL>();
                Self::phase_low::<AH, AL>();
                self.phase_pwm::<BH, BL>();
            }
            5 => {
                Self::phase_float::<AH, AL>();
                Self::phase_low::<CH, CL>();
                self.phase_pwm::<BH, BL>();
            }
            6 => {
                Self::phase_float::<BH, BL>();
                Self::phase_low::<CH, CL>();
                self.phase_pwm::<AH, AL>();
            }
            _ => {}
        }
    }

    fn all_off(&mut self) {
        Self::phase_float::<AH, AL>();
        Self::phase_float::<BH, BL>();
        Self::phase_float::<CH, CL>();
    }

    fn full_brake(&mut self) {
        Self::phase_low::<AH, AL>();
        Self::phase_low::<BH, BL>();
        Self::phase_low::<CH, CL>();
    }

    fn all_pwm(&mut self) {
        self.phase_pwm::<AH, AL>();
        self.phase_pwm::<BH, BL>();
        self.phase_pwm::<CH, CL>();
    }

    fn proportional_brake(&mut self) {
        if self.bridge_enable {
            return; // not supported on PWM/enable bridge boards
        }
        AH::set_mode(MODE_OUTPUT);
        AH::set_low();
        BH::set_mode(MODE_OUTPUT);
        BH::set_low();
        CH::set_mode(MODE_OUTPUT);
        CH::set_low();
        AL::set_mode(MODE_ALTERNATE);
        BL::set_mode(MODE_ALTERNATE);
        CL::set_mode(MODE_ALTERNATE);
    }

    fn pulse_toggle(&mut self, step: u8) {
        if let Some((toggle_fn, mask)) = self.pulse {
            if step == 1 || step == 4 {
                toggle_fn(mask);
            }
        }
    }
}

// --- Board-specific type aliases ---

use crate::gpio_pin::{PA7, PA8, PA9, PA10, PB0, PB1};

/// G0_A / F0_A / L4_N pin assignment (all three MCUs share the same pins).
pub type G0APhaseDriver = PhaseDriver<PA10, PB1, PA9, PB0, PA8, PA7>;
