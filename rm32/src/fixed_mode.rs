//! Fixed duty / fixed speed development modes.
//!
//! These bypass input signal detection and arming, running the motor
//! at a constant duty cycle or target RPM. Used for bench testing.
//!
//! Enable via compile-time constants — not runtime EEPROM settings.

/// Fixed duty mode: bypasses signal input, runs at constant duty.
/// `power`: 0-100 (percent), maps to input = power * 20 + 47.
pub fn fixed_duty_input(power: u8) -> u16 {
    power as u16 * 20 + 47
}

/// Fixed speed mode: computes target e_com_time from RPM and motor poles.
/// `rpm`: target mechanical RPM
/// `motor_poles`: total pole count from EEPROM (e.g. 14)
pub fn fixed_speed_target(rpm: u32, motor_poles: u8) -> u32 {
    let poles = if motor_poles < 2 { 14 } else { motor_poles as u32 };
    60_000_000 / rpm / (poles / 2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_duty_100_percent() {
        assert_eq!(fixed_duty_input(100), 2047);
    }

    #[test]
    fn fixed_duty_0_percent() {
        assert_eq!(fixed_duty_input(0), 47);
    }

    #[test]
    fn fixed_speed_1000rpm_14pole() {
        let target = fixed_speed_target(1000, 14);
        // 60_000_000 / 1000 / 7 = 8571
        assert_eq!(target, 8571);
    }
}
