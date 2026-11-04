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

use crate::units::{CentiAmps, CentiVolts};

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
    counter: u16,
    /// Whether the last frame sent was an extended frame
    last_sent_extended: bool,
    /// Whether EDT is active
    active: bool,
    /// Pending init frame to send
    send_init: bool,
    /// Pending deinit frame to send
    send_deinit: bool,
}

impl EdtScheduler {
    /// Request an EDT init frame on next telemetry response.
    pub fn request_init(&mut self) {
        self.send_init = true;
    }

    /// Request an EDT deinit frame on next telemetry response.
    pub fn request_deinit(&mut self) {
        self.send_deinit = true;
    }
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
    /// Current and voltage use the shared centiamp/centivolt measurement units.
    /// `temperature`: degrees celsius
    pub fn next_frame(
        &mut self,
        current: CentiAmps,
        voltage: CentiVolts,
        temperature: i16,
    ) -> EdtFrame {
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

        // Re-send EDT init periodically so a lost init frame does not leave
        // the peer treating typed EDT frames as eRPM indefinitely.
        if self.counter % 512 == 256 {
            return EdtFrame::Extended(EDT_INIT_FRAME);
        }

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
            // Current: 1A per LSB
            let payload = (current.to_centiamps() / 100).min(255) as u8;
            (EDT_CURRENT << 8) | payload as u16
        } else if self.counter % 200 == 100 {
            // Voltage: 0.25V per LSB
            let payload = (voltage.0 / 25).min(255) as u8;
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

/// The bidirectional DShot response for one telemetry slot: EDT chooses an
/// extended frame or eRPM (the commutation period, read only for an eRPM
/// slot), and the 12-bit value is GCR-encoded into `gcr` at the port's
/// padding (`7`) and `gcr_shift`. `on_extended` sees each extended value.
/// Returns without modifying `edt` or `gcr` if `gcr.len() < 29` (`padding + 22`).
#[inline(always)]
pub fn encode_response<F: FnOnce(u16)>(
    edt: &mut EdtScheduler,
    shared: &crate::shared_state::SharedState,
    gcr: &mut [u32],
    gcr_shift: u8,
    on_extended: F,
) {
    if gcr.len() < 29 {
        return;
    }
    let value_12bit = match edt.next_frame(
        shared.actual_current(),
        shared.battery_voltage(),
        shared.degrees_celsius(),
    ) {
        EdtFrame::Extended(v) => {
            on_extended(v);
            v
        }
        EdtFrame::Erpm => crate::dshot::erpm_to_12bit(shared.e_com_time() as u16, shared.running()),
    };
    crate::dshot::encode_gcr_frame(value_12bit, gcr, 7, gcr_shift);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voltage_wire_boundaries() {
        for (cv, payload) in [
            (0, 0),
            (24, 0),
            (25, 1),
            (49, 1),
            (50, 2),
            (6374, 254),
            (6375, 255),
            (6376, 255),
            (u16::MAX, 255),
        ] {
            let mut s = EdtScheduler {
                active: true,
                counter: 99,
                ..Default::default()
            };
            assert!(matches!(s.next_frame(CentiAmps::ZERO, CentiVolts(cv), 25),
                EdtFrame::Extended(frame) if frame == 0x400 | payload));
        }
    }

    #[test]
    fn current_wire_boundaries() {
        for (ca, payload) in [
            (i16::MIN, 0),
            (-1, 0),
            (0, 0),
            (99, 0),
            (100, 1),
            (199, 1),
            (25500, 255),
            (25600, 255),
            (i16::MAX, 255),
        ] {
            let mut s = EdtScheduler {
                active: true,
                counter: 39,
                ..Default::default()
            };
            assert!(matches!(s.next_frame(CentiAmps(ca), CentiVolts::ZERO, 25),
                EdtFrame::Extended(frame) if frame == 0x600 | payload));
        }
    }

    #[test]
    fn init_frame_sent_once() {
        let mut s = EdtScheduler::default();
        s.request_init();
        match s.next_frame(CentiAmps(0), CentiVolts(0), 0) {
            EdtFrame::Extended(v) => assert_eq!(v, EDT_INIT_FRAME),
            _ => panic!("expected init frame"),
        }
        assert!(s.active);
        // Verify init was consumed — next call should NOT produce another init
        match s.next_frame(CentiAmps(0), CentiVolts(0), 0) {
            EdtFrame::Extended(v) if v == EDT_INIT_FRAME => panic!("init should be one-shot"),
            _ => {} // any non-init frame is correct
        }
    }

    #[test]
    fn init_frame_is_periodically_reannounced() {
        let mut s = EdtScheduler::default();
        s.request_init();
        assert!(matches!(
            s.next_frame(CentiAmps(0), CentiVolts(0), 0),
            EdtFrame::Extended(EDT_INIT_FRAME)
        ));

        for _ in 0..255 {
            let _ = s.next_frame(CentiAmps(100), CentiVolts(1200), 25);
        }

        match s.next_frame(CentiAmps(100), CentiVolts(1200), 25) {
            EdtFrame::Extended(v) => assert_eq!(v, EDT_INIT_FRAME),
            _ => panic!("expected periodic init frame"),
        }
    }

    #[test]
    fn deinit_frame_deactivates() {
        let mut s = EdtScheduler::default();
        s.request_init(); // activate first
        s.next_frame(CentiAmps(0), CentiVolts(0), 0); // consume init
        s.request_deinit();
        match s.next_frame(CentiAmps(0), CentiVolts(0), 0) {
            EdtFrame::Extended(v) => assert_eq!(v, EDT_DEINIT_FRAME),
            _ => panic!("expected deinit frame"),
        }
        assert!(!s.active);
    }

    #[test]
    fn inactive_always_erpm() {
        let mut s = EdtScheduler::default();
        for _ in 0..300 {
            assert!(matches!(
                s.next_frame(CentiAmps(50), CentiVolts(1680), 25),
                EdtFrame::Erpm
            ));
        }
    }

    #[test]
    fn current_sent_every_40() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = u16::MAX; // next increment wraps to 0

        let frame = s.next_frame(CentiAmps(350), CentiVolts(1680), 30);
        match frame {
            EdtFrame::Extended(v) => {
                assert_eq!(v >> 8, EDT_CURRENT);
                assert_eq!(v & 0xFF, 3);
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
            s.next_frame(CentiAmps(100), CentiVolts(1680), 25),
            EdtFrame::Extended(_)
        ));
        // Next: forced eRPM
        assert!(matches!(
            s.next_frame(CentiAmps(100), CentiVolts(1680), 25),
            EdtFrame::Erpm
        ));
    }

    #[test]
    fn voltage_encoding() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = 99; // next will be 100

        match s.next_frame(CentiAmps(0), CentiVolts(1230), 25) {
            EdtFrame::Extended(v) => {
                assert_eq!(v >> 8, EDT_VOLTAGE);
                assert_eq!(v & 0xFF, 49);
            }
            _ => panic!("expected voltage frame"),
        }
    }

    #[test]
    fn temperature_encoding() {
        let mut s = EdtScheduler::default();
        s.active = true;
        s.counter = 149; // next will be 150

        match s.next_frame(CentiAmps(0), CentiVolts(0), 45) {
            EdtFrame::Extended(v) => {
                assert_eq!(v >> 8, EDT_TEMPERATURE);
                assert_eq!(v & 0xFF, 45);
            }
            _ => panic!("expected temp frame"),
        }
    }

    /// `encode_response` sends the scheduler's pending extended frame (seen by
    /// the callback) or eRPM from the shared commutation period, encoded
    /// identically to `encode_gcr_frame`.
    #[test]
    fn encode_response_selects_and_encodes() {
        let shared = crate::shared_state::SharedState::new::<crate::platform::TestPlatform>();
        shared.set_running(true);
        shared.set_e_com_time(1234);
        let mut edt = EdtScheduler::default();
        let mut seen = None;
        let mut got = [0u32; 37];
        let mut want = [0u32; 37];
        edt.request_init();
        let mut short = [0u32; 28];
        encode_response(&mut edt, &shared, &mut short, 7, |v| seen = Some(v));
        assert_eq!(
            (seen, short),
            (None, [0u32; 28]),
            "short buffer is ignored without advancing EDT"
        );
        encode_response(&mut edt, &shared, &mut got, 7, |v| seen = Some(v));
        crate::dshot::encode_gcr_frame(EDT_INIT_FRAME, &mut want, 7, 7);
        assert_eq!((seen, got), (Some(EDT_INIT_FRAME), want));
        seen = None;
        encode_response(&mut EdtScheduler::default(), &shared, &mut got, 7, |v| {
            seen = Some(v)
        });
        let erpm = crate::dshot::erpm_to_12bit(1234, true);
        assert_ne!(erpm, crate::dshot::erpm_to_12bit(1234, false));
        crate::dshot::encode_gcr_frame(erpm, &mut want, 7, 7);
        assert_eq!((seen, got), (None, want));
    }
}
