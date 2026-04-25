//! Motor phase output control (6-step commutation via GPIO mode switching).
//!
//! PhaseDriver is generic over 6 pin types — all pin identity is resolved
//! at compile time. Runtime methods contain no branches on port or pin number.
//!
//! Pin assignments for HARDWARE_GROUP_G0_A:
//!   Phase A: high=PA10, low=PB1
//!   Phase B: high=PA9,  low=PB0
//!   Phase C: high=PA8,  low=PA7

use core::marker::PhantomData;
use rm32::hal::PhaseOutput;
use crate::gpio_pin::GpioPin;

/// GPIO MODER values.
const MODE_OUTPUT: u32 = 0b01;
const MODE_ALTERNATE: u32 = 0b10;

/// 3-phase driver, parameterized by 6 compile-time pin types.
///
/// AH/AL = Phase A high/low, BH/BL = Phase B, CH/CL = Phase C.
/// After monomorphization, all port/pin constants are inlined — zero overhead.
pub struct PhaseDriver<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin> {
    comp_pwm: bool,
    _pins: PhantomData<(AH, AL, BH, BL, CH, CL)>,
}

impl<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin>
    PhaseDriver<AH, AL, BH, BL, CH, CL>
{
    pub fn new(comp_pwm: bool) -> Self {
        Self { comp_pwm, _pins: PhantomData }
    }

    /// Phase PWM: high-side alternate, low-side alternate (comp_pwm) or off.
    #[inline(always)]
    fn phase_pwm<H: GpioPin, L: GpioPin>(&self) {
        if !self.comp_pwm {
            L::set_mode(MODE_OUTPUT);
            L::set_low();
        } else {
            L::set_mode(MODE_ALTERNATE);
        }
        H::set_mode(MODE_ALTERNATE);
    }

    /// Phase LOW: low-side on, high-side off.
    #[inline(always)]
    fn phase_low<H: GpioPin, L: GpioPin>() {
        L::set_mode(MODE_OUTPUT);
        L::set_high();
        H::set_mode(MODE_OUTPUT);
        H::set_low();
    }

    /// Phase FLOAT: both FETs off.
    #[inline(always)]
    fn phase_float<H: GpioPin, L: GpioPin>() {
        L::set_mode(MODE_OUTPUT);
        L::set_low();
        H::set_mode(MODE_OUTPUT);
        H::set_low();
    }
}

impl<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin>
    PhaseOutput for PhaseDriver<AH, AL, BH, BL, CH, CL>
{
    fn com_step(&mut self, step: u8) {
        match step {
            1 => { Self::phase_float::<CH, CL>(); Self::phase_low::<BH, BL>(); self.phase_pwm::<AH, AL>(); }
            2 => { Self::phase_float::<AH, AL>(); Self::phase_low::<BH, BL>(); self.phase_pwm::<CH, CL>(); }
            3 => { Self::phase_float::<BH, BL>(); Self::phase_low::<AH, AL>(); self.phase_pwm::<CH, CL>(); }
            4 => { Self::phase_float::<CH, CL>(); Self::phase_low::<AH, AL>(); self.phase_pwm::<BH, BL>(); }
            5 => { Self::phase_float::<AH, AL>(); Self::phase_low::<CH, CL>(); self.phase_pwm::<BH, BL>(); }
            6 => { Self::phase_float::<BH, BL>(); Self::phase_low::<CH, CL>(); self.phase_pwm::<AH, AL>(); }
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
        AH::set_mode(MODE_OUTPUT); AH::set_low();
        BH::set_mode(MODE_OUTPUT); BH::set_low();
        CH::set_mode(MODE_OUTPUT); CH::set_low();
        AL::set_mode(MODE_ALTERNATE);
        BL::set_mode(MODE_ALTERNATE);
        CL::set_mode(MODE_ALTERNATE);
    }
}

// --- Board-specific type aliases ---

use crate::gpio_pin::{PA7, PA8, PA9, PA10, PB0, PB1};

/// G0_A / F0_A / L4_N pin assignment (all three MCUs share the same pins).
pub type G0APhaseDriver = PhaseDriver<PA10, PB1, PA9, PB0, PA8, PA7>;
