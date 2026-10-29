//! The three-phase bridge, described semantically: what each leg does in
//! each commutation step, and the [`Bridge`] the core drives. Ports build
//! their register tables from [`step_legs`] at compile time, so the six-step
//! table exists once.

use crate::hal::PhaseOutput;

/// What one half-bridge leg (phase) is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegRole {
    /// Both FETs off (the floating phase: BEMF sensing).
    Float,
    /// Low side on, high side off (the current sink).
    LowSolid,
    /// High side PWM, low side off (freewheel through the body diode).
    PwmDiode,
    /// High and low side complementary PWM with dead time.
    PwmComplementary,
    /// High side off, low side PWM (proportional brake).
    LowPwm,
}

/// AM32's six-step table (`comStep`): the role of phases A, B, C in `step`
/// (1..=6); any other step leaves every leg floating. `complementary`
/// selects the driven leg's PWM style (the drive-mode policy).
pub const fn step_legs(step: u8, complementary: bool) -> [LegRole; 3] {
    use LegRole::{Float, LowSolid};
    let pwm = if complementary {
        LegRole::PwmComplementary
    } else {
        LegRole::PwmDiode
    };
    match step {
        1 => [pwm, LowSolid, Float],
        2 => [Float, LowSolid, pwm],
        3 => [LowSolid, Float, pwm],
        4 => [LowSolid, pwm, Float],
        5 => [Float, pwm, LowSolid],
        6 => [pwm, Float, LowSolid],
        _ => [Float, Float, Float],
    }
}

/// The power stage as the core drives it: the gate legs plus the PWM timer
/// behind them (AM32's TIM1 period, compare, dead-time and prescaler writes).
pub trait Bridge: PhaseOutput {
    /// Same compare value on all three phases.
    fn set_duty_all(&mut self, compare: u16);
    /// Per-phase compare values (sine start).
    fn set_phase_duties(&mut self, a: u16, b: u16, c: u16);
    /// PWM period in timer counts (AM32 `TIM1->ARR`: variable PWM, tones).
    fn set_period(&mut self, period: u16);
    /// Apply period and compares now (AM32 update event).
    fn latch(&mut self);
    /// Dead-time override in AM32 config units.
    fn set_dead_time_override(&mut self, dto: u16);
    /// Tone prescaler (AM32 `TIM1->PSC` in sounds); 0 restores motor PWM.
    fn set_tone_divider(&mut self, div: u16);
}

/// A `&mut` to a bridge is a bridge (lets a HAL hand out `&mut` its own).
impl<B: PhaseOutput + ?Sized> PhaseOutput for &mut B {
    #[inline(always)]
    fn com_step(&mut self, step: u8) {
        (**self).com_step(step);
    }
    #[inline(always)]
    fn all_off(&mut self) {
        (**self).all_off();
    }
    #[inline(always)]
    fn full_brake(&mut self) {
        (**self).full_brake();
    }
    #[inline(always)]
    fn all_pwm(&mut self) {
        (**self).all_pwm();
    }
    #[inline(always)]
    fn proportional_brake(&mut self) {
        (**self).proportional_brake();
    }
    #[inline(always)]
    fn pulse_toggle(&mut self, step: u8) {
        (**self).pulse_toggle(step);
    }
}

impl<B: Bridge + ?Sized> Bridge for &mut B {
    #[inline(always)]
    fn set_duty_all(&mut self, compare: u16) {
        (**self).set_duty_all(compare);
    }
    #[inline(always)]
    fn set_phase_duties(&mut self, a: u16, b: u16, c: u16) {
        (**self).set_phase_duties(a, b, c);
    }
    #[inline(always)]
    fn set_period(&mut self, period: u16) {
        (**self).set_period(period);
    }
    #[inline(always)]
    fn latch(&mut self) {
        (**self).latch();
    }
    #[inline(always)]
    fn set_dead_time_override(&mut self, dto: u16) {
        (**self).set_dead_time_override(dto);
    }
    #[inline(always)]
    fn set_tone_divider(&mut self, div: u16) {
        (**self).set_tone_divider(div);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// AM32 `comStep` (Src/main.c): per step, (driven phase, low phase);
    /// the remaining phase floats. Phases 0 = A, 1 = B, 2 = C.
    const AM32: [(usize, usize); 6] = [(0, 1), (2, 1), (2, 0), (1, 0), (1, 2), (0, 2)];

    #[test]
    fn step_legs_is_am32_comstep() {
        for (ix, &(pwm, low)) in AM32.iter().enumerate() {
            for comp in [false, true] {
                let legs = step_legs(ix as u8 + 1, comp);
                for (phase, &role) in legs.iter().enumerate() {
                    let want = if phase == pwm {
                        if comp {
                            LegRole::PwmComplementary
                        } else {
                            LegRole::PwmDiode
                        }
                    } else if phase == low {
                        LegRole::LowSolid
                    } else {
                        LegRole::Float
                    };
                    assert_eq!(role, want, "step {} phase {phase} comp {comp}", ix + 1);
                }
            }
        }
    }

    #[test]
    fn invalid_steps_float_everything() {
        for step in [0, 7, 255] {
            assert_eq!(step_legs(step, true), [LegRole::Float; 3]);
        }
    }
}
