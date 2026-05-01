//! Servo PWM input processing with calibration, rate limiting, and bidir.

use crate::functions::{get_abs_dif, map};

/// Servo input processing state.
#[derive(Clone)]
pub struct ServoState {
    pub low_threshold: u16,
    pub high_threshold: u16,
    pub neutral: u16,
    pub dead_band: u8,
    max_deviation: u16,
    raw_input: i32,
    // Calibration
    pub(crate) calibration_required: bool,
    pub(crate) high_calibration_set: bool,
    high_calibration_counts: u8,
    low_calibration_counts: u8,
    last_high_threshold: u16,
}

impl Default for ServoState {
    fn default() -> Self {
        Self {
            low_threshold: 1100,
            high_threshold: 1900,
            neutral: 1500,
            dead_band: 100,
            max_deviation: 250,
            raw_input: 0,
            calibration_required: false,
            high_calibration_set: false,
            high_calibration_counts: 0,
            low_calibration_counts: 0,
            last_high_threshold: 0,
        }
    }
}

/// Result of processing a servo pulse.
pub enum ServoResult {
    /// Valid throttle value
    Throttle(u16),
    /// Pulse out of range
    OutOfRange,
    /// Calibration in progress (no output)
    Calibrating,
    /// Calibration high-end complete — play default tone
    CalibrationHighDone,
    /// Calibration complete — save settings, play changed tone
    CalibrationDone {
        low_threshold_eeprom: u8,
        high_threshold_eeprom: u8,
    },
}

impl ServoState {
    /// Process a servo pulse width (dma_buffer[1] - dma_buffer[0]).
    /// Returns the new newinput value with rate limiting applied.
    pub fn compute(
        &mut self,
        pulse_width: u16,
        current_newinput: u16,
        bidirectional: bool,
    ) -> ServoResult {
        // Validate pulse range (800-2200µs)
        if pulse_width <= 800 || pulse_width >= 2200 {
            return ServoResult::OutOfRange;
        }

        if self.calibration_required {
            return self.process_calibration(pulse_width);
        }

        // Normal mapping
        if bidirectional {
            if pulse_width <= self.neutral {
                self.raw_input = map(
                    pulse_width as i32,
                    self.low_threshold as i32,
                    self.neutral as i32,
                    0,
                    1000,
                );
            } else {
                self.raw_input = map(
                    pulse_width as i32,
                    self.neutral as i32 + 1,
                    self.high_threshold as i32,
                    1001,
                    2000,
                );
            }
        } else {
            self.raw_input = map(
                pulse_width as i32,
                self.low_threshold as i32,
                self.high_threshold as i32,
                47,
                2047,
            );
            if self.raw_input <= 48 {
                self.raw_input = 0;
            }
        }

        // Rate limiting
        let newinput = self.rate_limit(current_newinput);
        ServoResult::Throttle(newinput)
    }

    /// Apply rate limiting to smooth servo input transitions.
    fn rate_limit(&self, current: u16) -> u16 {
        let target = self.raw_input;
        let max_dev = self.max_deviation as i32;
        let cur = current as i32;

        if target - cur > max_dev {
            (cur + max_dev) as u16
        } else if cur - target > max_dev {
            (cur - max_dev) as u16
        } else {
            target as u16
        }
    }

    /// Process calibration sequence.
    fn process_calibration(&mut self, pulse_width: u16) -> ServoResult {
        if !self.high_calibration_set {
            // High-end calibration: averaging high pulses
            if self.high_calibration_counts == 0 {
                self.last_high_threshold = pulse_width;
            }
            self.high_calibration_counts += 1;

            if get_abs_dif(self.last_high_threshold as i32, self.high_threshold as i32) > 50 {
                self.calibration_required = false;
                return ServoResult::Calibrating;
            }

            self.high_threshold =
                ((7 * self.high_threshold as u32 + pulse_width as u32) >> 3) as u16;

            if self.high_calibration_counts > 50 {
                self.high_threshold -= 25;
                self.high_calibration_set = true;
                self.last_high_threshold = self.high_threshold;
                return ServoResult::CalibrationHighDone;
            }
            self.last_high_threshold = self.high_threshold;
            ServoResult::Calibrating
        } else {
            // Low-end calibration
            if pulse_width < 1250 {
                self.low_calibration_counts += 1;
                self.low_threshold =
                    ((7 * self.low_threshold as u32 + pulse_width as u32) >> 3) as u16;
            }
            if self.low_calibration_counts > 75 {
                self.low_threshold += 25;
                let high_eeprom = ((self.high_threshold - 1750) / 2) as u8;
                let low_eeprom = ((self.low_threshold - 750) / 2) as u8;
                self.calibration_required = false;
                self.low_calibration_counts = 0;
                return ServoResult::CalibrationDone {
                    low_threshold_eeprom: low_eeprom,
                    high_threshold_eeprom: high_eeprom,
                };
            }
            ServoResult::Calibrating
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn servo_unidirectional_mid() {
        let mut s = ServoState::default();
        s.max_deviation = 20000; // disable rate limiting for this test
        match s.compute(1500, 0, false) {
            ServoResult::Throttle(v) => assert!(v > 900 && v < 1200),
            _ => panic!("expected throttle"),
        }
    }

    #[test]
    fn servo_out_of_range() {
        let mut s = ServoState::default();
        assert!(matches!(s.compute(500, 0, false), ServoResult::OutOfRange));
        assert!(matches!(s.compute(2500, 0, false), ServoResult::OutOfRange));
    }

    #[test]
    fn servo_rate_limiting() {
        let mut s = ServoState::default();
        s.max_deviation = 100;
        match s.compute(1900, 500, false) {
            ServoResult::Throttle(v) => assert_eq!(v, 600), // 500 + 100
            _ => panic!("expected throttle"),
        }
    }

    #[test]
    fn servo_bidirectional_below_neutral() {
        let mut s = ServoState::default();
        s.max_deviation = 20000;
        match s.compute(1300, 0, true) {
            ServoResult::Throttle(v) => assert!(v > 0 && v < 1000),
            _ => panic!("expected throttle"),
        }
    }

    #[test]
    fn servo_bidirectional_above_neutral() {
        let mut s = ServoState::default();
        s.max_deviation = 20000;
        match s.compute(1700, 0, true) {
            ServoResult::Throttle(v) => assert!(v > 1000),
            _ => panic!("expected throttle"),
        }
    }

    #[test]
    fn calibration_high_end() {
        let mut s = ServoState::default();
        s.calibration_required = true;
        s.high_threshold = 1900;
        // Feed consistent pulses at ~1900
        for _ in 0..51 {
            let r = s.compute(1900, 0, false);
            if matches!(r, ServoResult::CalibrationHighDone) {
                assert!(s.high_calibration_set);
                return;
            }
        }
        panic!("calibration high should have completed");
    }
}
