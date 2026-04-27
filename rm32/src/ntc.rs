//! External NTC thermistor temperature lookup.
//!
//! Some boards use an external NTC thermistor instead of the MCU's internal
//! temperature sensor. The NTC resistance varies with temperature, and the
//! ADC reads the voltage divider output.

use crate::units::DegreesCelsius;

/// 10K NTC B3950 lookup table (ADC counts → degrees C).
/// Table covers 0°C to 150°C in steps matching the ADC range.
/// Index = ADC reading >> 4 (divide by 16 for 256-entry table from 12-bit ADC).
static NTC_TABLE: [i16; 256] = {
    let mut t = [0i16; 256];
    let mut i = 0;
    while i < 256 {
        let adc = i as i32 * 16;
        t[i] = if adc < 200 {
            150
        } else if adc < 500 {
            150 - (adc - 200) * 50 / 300
        } else if adc < 1500 {
            100 - (adc - 500) * 60 / 1000
        } else if adc < 3000 {
            40 - (adc - 1500) * 40 / 1500
        } else {
            0
        } as i16;
        i += 1;
    }
    t
};

/// Convert raw ADC reading from external NTC to temperature.
pub fn ntc_degrees(raw_adc: u16) -> DegreesCelsius {
    let idx = (raw_adc >> 4) as usize;
    let idx = idx.min(255);
    DegreesCelsius(NTC_TABLE[idx])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ntc_cold_is_low() {
        // High ADC = cold (NTC resistance high, voltage divider pulls high)
        let t = ntc_degrees(3500);
        assert!(t.0 <= 10, "expected cold, got {}", t.0);
    }

    #[test]
    fn ntc_hot_is_high() {
        // Low ADC = hot (NTC resistance low)
        let t = ntc_degrees(100);
        assert!(t.0 >= 100, "expected hot, got {}", t.0);
    }

    #[test]
    fn ntc_midrange() {
        let t = ntc_degrees(1000);
        assert!(t.0 > 30 && t.0 < 80, "expected midrange, got {}", t.0);
    }
}
