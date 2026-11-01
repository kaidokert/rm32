//! Motor phase output control (6-step commutation via GPIO mode switching).
//!
//! PhaseDriver is generic over 6 pin types — all pin identity is resolved
//! at compile time. On standard non-`bridge_enable` bridges, `com_step` uses
//! a compile-time `[bsrr_a, bsrr_b, moder_a_mask, moder_a_val, moder_b_mask,
//! moder_b_val]` lookup table derived from `rm32::bridge::step_legs`, issuing
//! one `BSRR` write per port followed by one `MODER` read-modify-write per port
//! so final gate levels land without intermediate mixed bridge states.
//!
//! Pin assignments for HARDWARE_GROUP_G0_A:
//!   Phase A: high=PA10, low=PB1
//!   Phase B: high=PA9,  low=PB0
//!   Phase C: high=PA8,  low=PA7

use crate::gpio_pin::GpioPin;
use crate::gpio_regs::GpioPort;
use core::marker::PhantomData;
use rm32::bridge::{LegRole, step_legs};
use rm32::hal::PhaseOutput;

/// GPIO MODER values.
const MODE_OUTPUT: u32 = 0b01;
const MODE_ALTERNATE: u32 = 0b10;

// Derive the operation order from the semantic table at compile time for the
// sequential `bridge_enable` fallback: floating first, low second, driven last.
const fn ordered_phases(step: u8) -> [u8; 3] {
    let legs = step_legs(step, false);
    let mut order = [0; 3];
    let mut phase = 0;
    while phase < 3 {
        let slot = match legs[phase] {
            LegRole::Float => 0,
            LegRole::LowSolid => 1,
            _ => 2,
        };
        order[slot] = phase as u8;
        phase += 1;
    }
    order
}
const PHASE_ORDER: [[u8; 3]; 6] = [
    ordered_phases(1),
    ordered_phases(2),
    ordered_phases(3),
    ordered_phases(4),
    ordered_phases(5),
    ordered_phases(6),
];

/// Pulse output toggle function — stored as fn pointer to avoid storing raw addresses.
/// Monomorphized per pin type at `enable_pulse_output` call site.
type PulseToggleFn = fn(u32);

/// 3-phase driver, parameterized by 6 compile-time pin types.
///
/// AH/AL = Phase A high/low, BH/BL = Phase B, CH/CL = Phase C.
/// Port and pin constants are resolved by monomorphization.
pub struct PhaseDriver<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin>
{
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
    /// Phase index 0=A, 1=B, 2=C -> `(hi_pin, hi_on_port_b, lo_pin, lo_on_port_b)`.
    const PINS: [(u8, bool, u8, bool); 3] = [
        (AH::PIN, AH::PORT_B, AL::PIN, AL::PORT_B),
        (BH::PIN, BH::PORT_B, BL::PIN, BL::PORT_B),
        (CH::PIN, CH::PORT_B, CL::PIN, CL::PORT_B),
    ];

    /// `[bsrr_a, bsrr_b, moder_a_mask, moder_a_val, moder_b_mask, moder_b_val]`
    /// for step `step_ix + 1` of the core's six-step table.
    const fn plan_step(step_ix: usize, comp: bool) -> [u32; 6] {
        let legs = step_legs(step_ix as u8 + 1, comp);
        let mut r = [0u32; 6];
        let mut idx = 0;
        while idx < 3 {
            let (hi, hi_b, lo, lo_b) = Self::PINS[idx];
            // (hi_mode, lo_mode, hi_level: 0 none/1 low, lo_level: 0 none/1 low/2 high)
            let (hi_mode, lo_mode, hi_lvl, lo_lvl) = match legs[idx] {
                // driven leg: hi AF; lo AF (complementary) or OUTPUT-low (diode)
                LegRole::PwmComplementary => (MODE_ALTERNATE, MODE_ALTERNATE, 0u32, 0u32),
                LegRole::PwmDiode => (MODE_ALTERNATE, MODE_OUTPUT, 0, 1),
                LegRole::LowSolid => (MODE_OUTPUT, MODE_OUTPUT, 1, 2), // low leg: hi off, low FET on
                LegRole::Float => (MODE_OUTPUT, MODE_OUTPUT, 1, 1),    // floating leg: both off
                LegRole::LowPwm => (MODE_OUTPUT, MODE_ALTERNATE, 1, 0), // hi off, low PWM
            };
            let hi_shift = hi as u32 * 2;
            let bsrr_hi = if hi_lvl == 1 {
                1u32 << (hi as u32 + 16)
            } else {
                0
            };
            if hi_b {
                r[1] |= bsrr_hi;
                r[4] |= 0b11 << hi_shift;
                r[5] |= hi_mode << hi_shift;
            } else {
                r[0] |= bsrr_hi;
                r[2] |= 0b11 << hi_shift;
                r[3] |= hi_mode << hi_shift;
            }

            let lo_shift = lo as u32 * 2;
            let bsrr_lo = match lo_lvl {
                1 => 1u32 << (lo as u32 + 16),
                2 => 1u32 << (lo as u32),
                _ => 0,
            };
            if lo_b {
                r[1] |= bsrr_lo;
                r[4] |= 0b11 << lo_shift;
                r[5] |= lo_mode << lo_shift;
            } else {
                r[0] |= bsrr_lo;
                r[2] |= 0b11 << lo_shift;
                r[3] |= lo_mode << lo_shift;
            }
            idx += 1;
        }
        r
    }

    const fn plan_table(comp: bool) -> [[u32; 6]; 6] {
        let mut t = [[0u32; 6]; 6];
        let mut s = 0;
        while s < 6 {
            t[s] = Self::plan_step(s, comp);
            s += 1;
        }
        t
    }

    const PLAN: [[[u32; 6]; 6]; 2] = [Self::plan_table(false), Self::plan_table(true)];

    pub fn new() -> Self {
        Self {
            bridge_enable: false,
            pulse: None,
            _pins: PhantomData,
        }
    }

    pub fn new_bridge() -> Self {
        Self {
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
    /// Normal: low-side alternate (complementary) or output LOW.
    /// Bridge: enable pin output HIGH (complementary) or no-op.
    #[inline]
    fn phase_pwm<H: GpioPin, L: GpioPin>(&self, comp: bool) {
        if self.bridge_enable {
            if comp {
                L::set_mode(MODE_OUTPUT);
                L::set_high(); // enable on
            }
        } else if !comp {
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

impl<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin> Default
    for PhaseDriver<AH, AL, BH, BL, CH, CL>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<AH: GpioPin, AL: GpioPin, BH: GpioPin, BL: GpioPin, CH: GpioPin, CL: GpioPin> PhaseOutput
    for PhaseDriver<AH, AL, BH, BL, CH, CL>
{
    #[inline(never)]
    #[allow(unexpected_cfgs)]
    fn com_step(&mut self, step: u8, complementary: bool) {
        #[cfg(all(target_arch = "arm", not(rm32_bridge_enable)))]
        {
            let Some(p) = Self::PLAN[complementary as usize].get((step as usize).wrapping_sub(1))
            else {
                return;
            };
            use crate::mcu::{PortA, PortB};
            PortA::write_bsrr(p[0]);
            PortB::write_bsrr(p[1]);
            PortA::modify_moder(|v| (v & !p[2]) | p[3]);
            PortB::modify_moder(|v| (v & !p[4]) | p[5]);
            return;
        }
        #[allow(unreachable_code)]
        if !self.bridge_enable {
            let Some(p) = Self::PLAN[complementary as usize].get((step as usize).wrapping_sub(1))
            else {
                return;
            };
            use crate::mcu::{PortA, PortB};
            PortA::write_bsrr(p[0]);
            PortB::write_bsrr(p[1]);
            PortA::modify_moder(|v| (v & !p[2]) | p[3]);
            PortB::modify_moder(|v| (v & !p[4]) | p[5]);
            return;
        }
        if !(1..=6).contains(&step) {
            return;
        }
        let [float, low, pwm] = PHASE_ORDER[(step - 1) as usize];
        match float {
            0 => Self::phase_float::<AH, AL>(),
            1 => Self::phase_float::<BH, BL>(),
            _ => Self::phase_float::<CH, CL>(),
        }
        match low {
            0 => Self::phase_low::<AH, AL>(),
            1 => Self::phase_low::<BH, BL>(),
            _ => Self::phase_low::<CH, CL>(),
        }
        match pwm {
            0 => self.phase_pwm::<AH, AL>(complementary),
            1 => self.phase_pwm::<BH, BL>(complementary),
            _ => self.phase_pwm::<CH, CL>(complementary),
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

    #[inline(never)]
    fn all_pwm(&mut self, complementary: bool) {
        self.phase_pwm::<AH, AL>(complementary);
        self.phase_pwm::<BH, BL>(complementary);
        self.phase_pwm::<CH, CL>(complementary);
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
        if let (Some((toggle_fn, mask)), 1 | 4) = (self.pulse, step) {
            toggle_fn(mask);
        }
    }
}

// --- Board-specific type aliases ---

use crate::gpio_pin::{PA7, PA8, PA9, PA10, PB0, PB1};

/// G0_A / F0_A / L4_N pin assignment (all three MCUs share the same pins).
pub type G0APhaseDriver = PhaseDriver<PA10, PB1, PA9, PB0, PA8, PA7>;
