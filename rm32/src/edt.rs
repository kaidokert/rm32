//! Extended DShot Telemetry (EDT) scheduler and frame encoding.
//!
//! EDT multiplexes current, voltage, and temperature data into
//! the bidir DShot response frames, alternating with standard eRPM.
//!
//! Data type codes (upper 4 bits of 12-bit frame):
//!   0b0010 = Temperature
//!   0b0100 = Voltage
//!   0b0110 = Current
//!   0b1110 = EDT init/deinit special frames

/// EDT data type nibbles
pub const EDT_TEMPERATURE: u16 = 0x2;
pub const EDT_VOLTAGE: u16 = 0x4;
pub const EDT_CURRENT: u16 = 0x6;
pub const EDT_SPECIAL: u16 = 0xE;

/// EDT init frame: 0xE00
pub const EDT_INIT_FRAME: u16 = 0xE00;
/// EDT deinit frame: 0xEFF
pub const EDT_DEINIT_FRAME: u16 = 0xEFF;

/// EDT scheduler state.
/// Manages the interleaving of eRPM and extended data frames.
#[derive(Clone, Default)]
pub struct EdtScheduler {
    /// Frame counter (increments each telemetry response)
    pub counter: u16,
    /// Whether the last frame sent was an extended frame
    pub last_sent_extended: bool,
    /// Whether EDT is active
    pub active: bool,
    /// Pending init frame to send
    pub send_init: bool,
    /// Pending deinit frame to send
    pub send_deinit: bool,
}

/// What the scheduler decided to send this frame.
pub enum EdtFrame {
    /// Send standard eRPM telemetry
    Erpm,
    /// Send an EDT data frame (12-bit value ready for GCR encoding)
    Extended(u16),
}

impl EdtScheduler {
    /// Decide what to send for this telemetry response.
    ///
    /// `current_ma`: actual current in milliamps
    /// `voltage_mv`: battery voltage in millivolts
    /// `temperature`: degrees celsius
    pub fn next_frame(&mut self, current_ma: i16, voltage_mv: u16, temperature: i16) -> EdtFrame {
        // Handle pending init/deinit special frames
        if self.send_init {
            self.send_init = false;
            self.active = true;
            return EdtFrame::Extended(EDT_INIT_FRAME);
        }
        if self.send_deinit {
            self.send_deinit = false;
            self.active = false;
            return EdtFrame::Extended(EDT_DEINIT_FRAME);
        }

        if !self.active {
            return EdtFrame::Erpm;
        }

        self.counter = self.counter.wrapping_add(1);

        // Alternate: extended frame, then eRPM
        if self.last_sent_extended {
            self.last_sent_extended = false;
            return EdtFrame::Erpm;
        }

        // Decide which extended data to send based on counter
        // Current: every 40 frames (~20Hz at 800Hz input)
        // Voltage: every 200 frames (~4Hz)
        // Temperature: every 200 frames, offset from voltage
        let frame = if self.counter.is_multiple_of(40) {
            // Current: 50mA per LSB
            let payload = ((current_ma as i32).max(0) / 50) as u8;
            (EDT_CURRENT << 8) | payload as u16
        } else if self.counter % 200 == 100 {
            // Voltage: 25mV per LSB
            let payload = (voltage_mv / 25).min(255) as u8;
            (EDT_VOLTAGE << 8) | payload as u16
        } else if self.counter % 200 == 150 {
            // Temperature: direct degrees C
            let payload = temperature as u8;
            (EDT_TEMPERATURE << 8) | payload as u16
        } else {
            return EdtFrame::Erpm;
        };

        self.last_sent_extended = true;
        EdtFrame::Extended(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_frame_sent_once() {
        let mut s = EdtScheduler::default();
        s.send_init = true;
        match s.next_frame(0, 0, 0) {
            EdtFrame::Extended(v) => assert_eq!(v, EDT_INIT_FRAME),
            _ => panic!("expected init frame"),
        }
        assert!(s.active);
        assert!(!s.send_init);
    }

    #[test]
    fn deinit_frame_deactivates() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.send_deinit = true;
        match s.next_frame(0, 0, 0) {
            EdtFrame::Extended(v) => assert_eq!(v, EDT_DEINIT_FRAME),
            _ => panic!("expected deinit frame"),
        }
        assert!(!s.active);
    }

    #[test]
    fn inactive_always_erpm() {
        let mut s = EdtScheduler::default();
        for _ in 0..300 {
            assert!(matches!(s.next_frame(500, 16800, 25), EdtFrame::Erpm));
        }
    }

    #[test]
    fn current_sent_every_40() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = u16::MAX; // next increment wraps to 0

        let frame = s.next_frame(1500, 16800, 30);
        match frame {
            EdtFrame::Extended(v) => {
                assert_eq!(v >> 8, EDT_CURRENT);
                assert_eq!(v & 0xFF, 30); // 1500mA / 50 = 30
            }
            _ => panic!("expected current frame at counter=0"),
        }
    }

    #[test]
    fn alternates_extended_erpm() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = u16::MAX;

        // First: extended (current at counter=0)
        assert!(matches!(
            s.next_frame(1000, 16800, 25),
            EdtFrame::Extended(_)
        ));
        // Next: forced eRPM
        assert!(matches!(s.next_frame(1000, 16800, 25), EdtFrame::Erpm));
    }

    #[test]
    fn voltage_encoding() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = 99; // next will be 100

        match s.next_frame(0, 4200, 25) {
            EdtFrame::Extended(v) => {
                assert_eq!(v >> 8, EDT_VOLTAGE);
                assert_eq!(v & 0xFF, 168); // 4200mV / 25 = 168
            }
            _ => panic!("expected voltage frame"),
        }
    }

    #[test]
    fn temperature_encoding() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = 149; // next will be 150

        match s.next_frame(0, 0, 45) {
            EdtFrame::Extended(v) => {
                assert_eq!(v >> 8, EDT_TEMPERATURE);
                assert_eq!(v & 0xFF, 45);
            }
            _ => panic!("expected temp frame"),
        }
    }
}
