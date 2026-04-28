//! Sinusoidal startup / gimbal mode drive.
//!
//! Uses a 360-entry lookup table to generate three-phase sinusoidal
//! PWM duty cycles for smooth motor startup or gimbal control.

/// 360-entry sine table (0-360 mapped to 0-360 duty units).
/// Matches C firmware's `pwmSin[]` array exactly.
pub static PWM_SIN: [i16; 360] = [
    180, 183, 186, 189, 193, 196, 199, 202, 205, 208, 211, 214, 217, 220, 224, 227, 230, 233, 236,
    239, 242, 245, 247, 250, 253, 256, 259, 262, 265, 267, 270, 273, 275, 278, 281, 283, 286, 288,
    291, 293, 296, 298, 300, 303, 305, 307, 309, 312, 314, 316, 318, 320, 322, 324, 326, 327, 329,
    331, 333, 334, 336, 337, 339, 340, 342, 343, 344, 346, 347, 348, 349, 350, 351, 352, 353, 354,
    355, 355, 356, 357, 357, 358, 358, 359, 359, 359, 360, 360, 360, 360, 360, 360, 360, 360, 360,
    359, 359, 359, 358, 358, 357, 357, 356, 355, 355, 354, 353, 352, 351, 350, 349, 348, 347, 346,
    344, 343, 342, 340, 339, 337, 336, 334, 333, 331, 329, 327, 326, 324, 322, 320, 318, 316, 314,
    312, 309, 307, 305, 303, 300, 298, 296, 293, 291, 288, 286, 283, 281, 278, 275, 273, 270, 267,
    265, 262, 259, 256, 253, 250, 247, 245, 242, 239, 236, 233, 230, 227, 224, 220, 217, 214, 211,
    208, 205, 202, 199, 196, 193, 189, 186, 183, 180, 177, 174, 171, 167, 164, 161, 158, 155, 152,
    149, 146, 143, 140, 136, 133, 130, 127, 124, 121, 118, 115, 113, 110, 107, 104, 101, 98, 95,
    93, 90, 87, 85, 82, 79, 77, 74, 72, 69, 67, 64, 62, 60, 57, 55, 53, 51, 48, 46, 44, 42, 40, 38,
    36, 34, 33, 31, 29, 27, 26, 24, 23, 21, 20, 18, 17, 16, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 5,
    4, 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 16, 17, 18, 20, 21, 23, 24, 26, 27, 29, 31, 33, 34, 36, 38, 40, 42, 44, 46,
    48, 51, 53, 55, 57, 60, 62, 64, 67, 69, 72, 74, 77, 79, 82, 85, 87, 90, 93, 95, 98, 101, 104,
    107, 110, 113, 115, 118, 121, 124, 127, 130, 133, 136, 140, 143, 146, 149, 152, 155, 158, 161,
    164, 167, 171, 174, 177,
];

/// Compute sinusoidal PWM compare values for three phases.
///
/// Returns (ch1, ch2, ch3) duty values for TIM1 compare registers.
/// `gate_drive_offset`: minimum duty to overcome FET gate capacitance
/// `timer1_max_arr`: TIM1 auto-reload value
/// `sine_mode_power`: power scaling 1-10 (10 = full, for non-gimbal sine startup)
/// `gimbal_mode`: if true, uses full power (no sine_mode_power scaling)
/// Sine table amplitude scaling factor.
const SINE_AMPLITUDE_SCALE: i32 = 2;
/// Duty cycle normalization (maps sine range to timer ARR).
const DUTY_NORMALIZATION: i32 = 2000;
/// Power scaling base (power is 1-10, divided by this for 10%-100%).
const POWER_SCALE_BASE: i32 = 10;
/// Sine divider for non-gimbal mode (halves amplitude).
const SINE_DIVIDER_NORMAL: i32 = 2;
/// Sine divider for gimbal mode (full amplitude).
const SINE_DIVIDER_GIMBAL: i32 = 1;

pub fn sine_drive(
    positions: &PhasePositions,
    gate_drive_offset: i16,
    timer1_max_arr: u16,
    sine_mode_power: u8,
    gimbal_mode: bool,
) -> (u16, u16, u16) {
    let arr = timer1_max_arr as i32;
    let power = if gimbal_mode {
        POWER_SCALE_BASE
    } else {
        sine_mode_power.max(1) as i32
    };
    let divider = if gimbal_mode {
        SINE_DIVIDER_GIMBAL
    } else {
        SINE_DIVIDER_NORMAL
    };

    let compute = |pos: i16| -> u16 {
        let sin_val = PWM_SIN[pos as usize] as i32;
        let duty = ((SINE_AMPLITUDE_SCALE * sin_val / divider + gate_drive_offset as i32) * arr
            / DUTY_NORMALIZATION)
            * power
            / POWER_SCALE_BASE;
        duty.max(0) as u16
    };

    (
        compute(positions.a),
        compute(positions.b),
        compute(positions.c),
    )
}

/// Result of a sine mode step — tells the caller what to do next.
pub enum SineStepResult {
    /// Continue sine stepping with this delay in microseconds
    Continue(u16),
    /// Transition to BLDC mode — motor is at changeover point
    Changeover { commutation_interval: u32, step: u8 },
    /// Throttle too low or not armed — do nothing
    Idle,
}

/// Execute one sine mode step. Called from the main loop when `stepper_sine` is active.
///
/// `input`: current throttle (0-2047)
/// `armed`: motor armed
/// `forward`: motor direction
/// `motor_poles`: from EEPROM config
/// `changeover_step`: commutation step to use when transitioning to BLDC
#[allow(clippy::too_many_arguments)]
pub fn sine_step(
    positions: &mut PhasePositions,
    input: u16,
    armed: bool,
    forward: bool,
    motor_poles: u8,
    changeover_step: u8,
    gate_drive_offset: i16,
    timer1_max_arr: u16,
    sine_mode_power: u8,
) -> (SineStepResult, (u16, u16, u16)) {
    let poles = if motor_poles == 0 {
        14
    } else {
        motor_poles as u16
    };

    if input > crate::constants::THROTTLE_MIN_SIGNAL && armed {
        if input < crate::constants::SINE_SLOW_STEP_THROTTLE {
            // Sine wave stepper mode — slow rotation
            positions.advance(forward);
            let pwm = sine_drive(
                positions,
                gate_drive_offset,
                timer1_max_arr,
                sine_mode_power,
                false,
            );
            let step_delay = crate::functions::map(
                input as i32,
                48,
                120,
                7000i32 / poles as i32,
                810i32 / poles as i32,
            ) as u16;
            (SineStepResult::Continue(step_delay), pwm)
        } else {
            // Higher throttle — accelerate to changeover
            positions.advance(forward);
            let pwm = sine_drive(
                positions,
                gate_drive_offset,
                timer1_max_arr,
                sine_mode_power,
                false,
            );

            if input > crate::constants::SINE_CHANGEOVER_THROTTLE && positions.a == 0 {
                // Phase wrapped to 0 at sufficient throttle — transition to BLDC
                (
                    SineStepResult::Changeover {
                        commutation_interval: 9000,
                        step: changeover_step,
                    },
                    pwm,
                )
            } else {
                let step_delay = if input > crate::constants::SINE_CHANGEOVER_THROTTLE {
                    crate::constants::SINE_FAST_STEP_DELAY
                } else {
                    crate::constants::SINE_MEDIUM_STEP_DELAY
                };
                (SineStepResult::Continue(step_delay), pwm)
            }
        }
    } else {
        let pwm = sine_drive(
            positions,
            gate_drive_offset,
            timer1_max_arr,
            sine_mode_power,
            false,
        );
        (SineStepResult::Idle, pwm)
    }
}

/// Gimbal mode step: maps input directly to angle, steps toward it.
/// Returns (delay_us, pwm_values).
pub fn gimbal_step(
    positions: &mut PhasePositions,
    current_angle: &mut i16,
    input: u16,
    gate_drive_offset: i16,
    timer1_max_arr: u16,
) -> (u16, (u16, u16, u16)) {
    let desired_angle = if input > 1000 {
        crate::functions::map(input as i32, 1000, 2000, 180, 360) as i16
    } else {
        crate::functions::map(input as i32, 0, 1000, 0, 180) as i16
    };

    if *current_angle > desired_angle {
        positions.advance(true); // forward
        *current_angle -= 1;
    } else if *current_angle < desired_angle {
        positions.advance(false); // reverse
        *current_angle += 1;
    }

    let pwm = sine_drive(positions, gate_drive_offset, timer1_max_arr, 10, true);
    (300, pwm) // 300us step delay for gimbal
}

/// Phase positions for sinusoidal drive (0-359 degrees).
#[derive(Clone, Default)]
pub struct PhasePositions {
    pub a: i16,
    pub b: i16,
    pub c: i16,
}

impl PhasePositions {
    /// Advance phase positions by one step.
    /// Forward=true decrements (motor convention), forward=false increments.
    pub fn advance(&mut self, forward: bool) {
        if !forward {
            self.a += 1;
            if self.a > 359 {
                self.a = 0;
            }
            self.b += 1;
            if self.b > 359 {
                self.b = 0;
            }
            self.c += 1;
            if self.c > 359 {
                self.c = 0;
            }
        } else {
            self.a -= 1;
            if self.a < 0 {
                self.a = 359;
            }
            self.b -= 1;
            if self.b < 0 {
                self.b = 359;
            }
            self.c -= 1;
            if self.c < 0 {
                self.c = 359;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_decrements() {
        let mut p = PhasePositions {
            a: 100,
            b: 219,
            c: 339,
        };
        p.advance(true);
        assert_eq!(p.a, 99);
        assert_eq!(p.b, 218);
        assert_eq!(p.c, 338);
    }

    #[test]
    fn reverse_increments() {
        let mut p = PhasePositions {
            a: 100,
            b: 219,
            c: 339,
        };
        p.advance(false);
        assert_eq!(p.a, 101);
        assert_eq!(p.b, 220);
        assert_eq!(p.c, 340);
    }

    #[test]
    fn forward_wraps_0_to_359() {
        let mut p = PhasePositions { a: 0, b: 0, c: 0 };
        p.advance(true);
        assert_eq!(p.a, 359);
        assert_eq!(p.b, 359);
        assert_eq!(p.c, 359);
    }

    #[test]
    fn reverse_wraps_359_to_0() {
        let mut p = PhasePositions {
            a: 359,
            b: 359,
            c: 359,
        };
        p.advance(false);
        assert_eq!(p.a, 0);
        assert_eq!(p.b, 0);
        assert_eq!(p.c, 0);
    }

    #[test]
    fn sine_table_symmetry() {
        // Table should be symmetric around index 180
        assert_eq!(PWM_SIN[0], PWM_SIN[359 - 359 + 0]); // first = 180
        assert_eq!(PWM_SIN[90], 360); // peak
        assert_eq!(PWM_SIN[270], 0); // trough
        assert_eq!(PWM_SIN[0], 180); // midpoint
    }

    #[test]
    fn sine_drive_produces_valid_pwm() {
        let p = PhasePositions {
            a: 0,
            b: 120,
            c: 240,
        };
        let (ch1, ch2, ch3) = sine_drive(&p, 60, 1999, 5, false);
        // All channels should be > 0 (gate_drive_offset ensures minimum)
        assert!(ch1 > 0);
        assert!(ch2 > 0);
        assert!(ch3 > 0);
        // All channels should be < timer1_max_arr
        assert!(ch1 < 1999);
        assert!(ch2 < 1999);
        assert!(ch3 < 1999);
    }

    #[test]
    fn sine_step_idle_when_not_armed() {
        let mut p = PhasePositions {
            a: 0,
            b: 120,
            c: 240,
        };
        let (result, _) = sine_step(&mut p, 100, false, true, 14, 1, 60, 1999, 5);
        assert!(matches!(result, SineStepResult::Idle));
    }

    #[test]
    fn sine_step_continues_at_low_throttle() {
        let mut p = PhasePositions {
            a: 0,
            b: 120,
            c: 240,
        };
        let (result, _) = sine_step(&mut p, 80, true, true, 14, 1, 60, 1999, 5);
        match result {
            SineStepResult::Continue(delay) => assert!(delay > 0),
            _ => panic!("expected Continue"),
        }
    }

    #[test]
    fn sine_step_changeover_at_high_throttle() {
        // Set phase A to 1 so one advance(true) wraps to 0
        let mut p = PhasePositions {
            a: 1,
            b: 121,
            c: 241,
        };
        let (result, _) = sine_step(&mut p, 500, true, true, 14, 3, 60, 1999, 5);
        match result {
            SineStepResult::Changeover {
                commutation_interval,
                step,
            } => {
                assert_eq!(commutation_interval, 9000);
                assert_eq!(step, 3);
            }
            _ => panic!("expected Changeover"),
        }
    }
}
