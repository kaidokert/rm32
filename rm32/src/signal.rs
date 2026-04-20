//! Input signal detection and processing (DShot/Servo/auto-detect).

/// Signal detection result
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SignalType {
    None,
    Dshot600,
    Dshot300,
    ServoPwm,
}

/// Detect input type from DMA buffer pulse pattern.
pub fn detect_input(dma_buffer: &[u32], cpu_mhz: u8) -> SignalType {
    let mut smallest = 20000u16;
    let mut average_pulse = 0u32;
    let mut last = dma_buffer[0];

    for j in 1..31 {
        let diff = dma_buffer[j].wrapping_sub(last);
        if diff > 0 {
            if (diff as u16) < smallest {
                smallest = diff as u16;
            }
            average_pulse += diff;
        }
        last = dma_buffer[j];
    }
    average_pulse /= 32;

    // Check DShot600: smallest 1-4, average < 60
    if smallest >= 1 && smallest < 4 && average_pulse < 60 {
        return SignalType::Dshot600;
    }
    // Check DShot300: smallest 4-8, average < 100
    if smallest >= 4 && smallest <= 8 && average_pulse < 100 {
        return SignalType::Dshot300;
    }
    // Check Servo: smallest > 200
    if smallest > 200 && smallest < 20000 {
        return SignalType::ServoPwm;
    }

    SignalType::None
}

/// Compute servo input from pulse width.
/// Returns mapped throttle value (0-2047 for unidirectional).
pub fn compute_servo_unidirectional(
    pulse_width: u16,
    low_threshold: u16,
    high_threshold: u16,
) -> u16 {
    use crate::functions::map;
    let raw = map(
        pulse_width as i32,
        low_threshold as i32,
        high_threshold as i32,
        47,
        2047,
    );
    if raw <= 48 { 0 } else { raw as u16 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_dshot600() {
        let mut buf = [0u32; 32];
        for i in 0..32 { buf[i] = 100 + i as u32 * 3; }
        assert_eq!(detect_input(&buf, 48), SignalType::Dshot600);
    }

    #[test]
    fn detect_servo() {
        let mut buf = [0u32; 32];
        for i in 0..32 { buf[i] = 100 + i as u32 * 1000; }
        assert_eq!(detect_input(&buf, 48), SignalType::ServoPwm);
    }

    #[test]
    fn detect_out_of_range() {
        let buf = [0u32; 32]; // all zeros, no valid pulses
        assert_eq!(detect_input(&buf, 48), SignalType::None);
    }

    #[test]
    fn servo_unidirectional_mid() {
        let val = compute_servo_unidirectional(1500, 1100, 1900);
        assert!(val > 900 && val < 1200); // roughly mid-range
    }

    #[test]
    fn servo_unidirectional_below_threshold() {
        let val = compute_servo_unidirectional(1050, 1100, 1900);
        assert_eq!(val, 0);
    }

    #[test]
    fn servo_unidirectional_max() {
        let val = compute_servo_unidirectional(1900, 1100, 1900);
        assert_eq!(val, 2047);
    }

    #[test]
    fn detect_dshot300() {
        let mut buf = [0u32; 32];
        for i in 0..32 { buf[i] = 100 + i as u32 * 5; }
        assert_eq!(detect_input(&buf, 48), SignalType::Dshot300);
    }

    #[test]
    fn detect_rejects_ambiguous() {
        let mut buf = [0u32; 32];
        for i in 0..32 { buf[i] = 100 + i as u32 * 12; } // smallest=12, >8
        // average = 12*31/32 ~ 11, not matching servo (>200) either
        assert_eq!(detect_input(&buf, 48), SignalType::None);
    }

    #[test]
    fn servo_mid_range_bidir_below_neutral() {
        // Below neutral: maps 0-1000
        let val = compute_servo_unidirectional(1300, 1100, 1900);
        assert!(val > 0 && val < 2047);
    }

    #[test]
    fn servo_at_threshold_is_zero() {
        // Exactly at low threshold: map returns out_min=47, then 47 <= 48 -> 0
        let val = compute_servo_unidirectional(1100, 1100, 1900);
        assert_eq!(val, 0);
    }
}
