//! Transfer complete dispatcher.
//!
//! Equivalent to C `transfercomplete()` — the central dispatcher that handles
//! DMA completion for both DShot and servo input, auto-detection, bidir
//! telemetry, and unarmed frame averaging.

use crate::dshot;
use crate::functions::get_abs_dif;
use crate::servo::{ServoResult, ServoState};
use crate::signal;

/// Transfer complete processing state.
#[derive(Default)]
pub struct TransferState {
    pub servo: ServoState,
    // Unarmed DShot frame averaging
    average_count: u8,
    average_packet_length: u32,
    // Calibration entry
    enter_calibration_count: u8,
    last_input: u16,
    high_pin_count: u8,
    bidir_confirms: u8,
    pending_protocol: Option<DetectedProtocol>,
}

/// Detected input protocol during auto-detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectedProtocol {
    Dshot,
    Servo,
}

/// Primary action from transfer complete processing.
#[derive(Debug)]
pub enum TransferAction {
    /// No valid frame decoded (bad CRC, timing, or rising edge)
    None,
    /// Input protocol detected (first frame after boot)
    InputDetected(DetectedProtocol),
    /// Valid DShot throttle frame
    DshotThrottle { value: u16, telemetry: bool },
    /// Valid DShot command frame
    DshotCommand { cmd: u16, telemetry: bool },
    /// Valid servo throttle
    ServoThrottle(u16),
    /// Servo calibration in progress (signal alive, no throttle value)
    ServoCalibrating,
    /// Servo calibration complete; persist thresholds to EEPROM.
    ServoCalibrationDone {
        low_threshold: u8,
        high_threshold: u8,
    },
}

/// DMA capture setup requested for the next input frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureConfig {
    /// DMA transfer count.
    pub ndtr: u32,
    /// Timer prescaler update, if protocol detection changed the capture rate.
    pub prescaler: Option<u16>,
}

impl CaptureConfig {
    pub const DSHOT: Self = Self {
        ndtr: 32,
        prescaler: None,
    };
    pub fn dshot_detection(cpu_mhz: u8) -> Self {
        Self {
            ndtr: 33,
            prescaler: Some((cpu_mhz / 6) as u16),
        }
    }
    pub const SERVO: Self = Self {
        ndtr: 2,
        prescaler: None,
    };
    pub const SERVO_REALIGN: Self = Self {
        ndtr: 3,
        prescaler: None,
    };

    pub const DSHOT600_DETECTED: Self = Self {
        ndtr: 32,
        prescaler: Some(0),
    };
    pub const DSHOT300_DETECTED: Self = Self {
        ndtr: 32,
        prescaler: Some(1),
    };
    pub const DSHOT150_DETECTED: Self = Self {
        ndtr: 32,
        prescaler: Some(3),
    };

    pub fn servo_detected(cpu_mhz: u8) -> Self {
        Self {
            ndtr: 2,
            prescaler: Some(cpu_mhz.saturating_sub(1) as u16),
        }
    }
}

/// Actions the caller (ISR) should take after transfer complete.
pub struct TransferActions {
    /// Primary action
    pub action: TransferAction,
    /// Capture setup for the next DMA cycle.
    pub next_capture: CaptureConfig,
    /// DShot frame timing update (from unarmed averaging)
    pub frametime: Option<(u16, u16)>,
    /// Bidirectional DShot auto-detected.
    pub bidir_detected: bool,
}

impl TransferState {
    /// Process a DMA transfer complete event.
    ///
    /// `dma_buffer`: the captured DMA data (32 entries for DShot, 2-3 for servo)
    /// `input_set`: whether input type has been detected
    /// `dshot_mode`: whether DShot is the active input
    /// `servo_mode`: whether servo PWM is the active input
    /// `armed`: motor armed state
    /// `dshot_telemetry`: bidirectional DShot mode
    /// `input_pin_high`: current state of input pin (for servo edge detection)
    /// `adjusted_input`: current throttle for calibration entry check
    /// `current_newinput`: current newinput for rate limiting
    /// `bidirectional`: config bi_direction flag
    /// `disable_stick_cal`: config disable_stick_calibration flag
    /// `zero_input_count`: current zero input counter
    /// `frametime_low/high`: current DShot frame timing bounds
    /// `cpu_mhz`: MCU core/timer clock in MHz for capture prescaler selection
    #[allow(clippy::too_many_arguments)]
    pub fn process(
        &mut self,
        dma_buffer: &[u32],
        input_set: bool,
        dshot_mode: bool,
        servo_mode: bool,
        dshot_telemetry: bool,
        armed: bool,
        input_pin_high: bool,
        adjusted_input: u16,
        current_newinput: u16,
        bidirectional: bool,
        disable_stick_cal: bool,
        zero_input_count: &mut u16,
        frametime_low: u16,
        frametime_high: u16,
        cpu_mhz: u8,
    ) -> TransferActions {
        let mut action = TransferAction::None;
        let mut frametime = None;

        // --- Input detection ---
        if !input_set {
            let sig = signal::detect_input(dma_buffer, cpu_mhz);
            let protocol = match sig {
                signal::SignalType::Dshot600
                | signal::SignalType::Dshot300
                | signal::SignalType::Dshot150 => Some(DetectedProtocol::Dshot),
                signal::SignalType::ServoPwm => Some(DetectedProtocol::Servo),
                signal::SignalType::None => None,
            };
            let confirmed = match (protocol, self.pending_protocol) {
                (Some(protocol), Some(pending)) if protocol == pending => {
                    self.pending_protocol = None;
                    Some(protocol)
                }
                (Some(protocol), _) => {
                    self.pending_protocol = Some(protocol);
                    None
                }
                (None, _) => {
                    self.pending_protocol = None;
                    None
                }
            };
            let (action, next_capture) = if let Some(protocol) = confirmed {
                let capture = match sig {
                    signal::SignalType::Dshot600 => CaptureConfig::DSHOT600_DETECTED,
                    signal::SignalType::Dshot300 => CaptureConfig::DSHOT300_DETECTED,
                    signal::SignalType::Dshot150 => CaptureConfig::DSHOT150_DETECTED,
                    signal::SignalType::ServoPwm => CaptureConfig::servo_detected(cpu_mhz),
                    signal::SignalType::None => {
                        unreachable!("confirmed input detection cannot have SignalType::None")
                    }
                };
                (TransferAction::InputDetected(protocol), capture)
            } else {
                (
                    TransferAction::None,
                    CaptureConfig::dshot_detection(cpu_mhz),
                )
            };
            return TransferActions {
                action,
                next_capture,
                frametime,
                bidir_detected: false,
            };
        }

        let mut bidir_detected = false;

        // --- DShot processing ---
        if dshot_mode && dma_buffer.len() >= 32 {
            let buf: [u32; 32] = {
                let mut b = [0u32; 32];
                b.copy_from_slice(&dma_buffer[..32]);
                b
            };
            let frame = dshot::decode_frame(&buf, frametime_low, frametime_high, dshot_telemetry);
            if !armed && !dshot_telemetry && self.high_pin_count > 100 {
                let inverted = dshot::decode_frame(&buf, frametime_low, frametime_high, true);
                let inverted_ok = matches!(
                    inverted,
                    dshot::DshotFrame::Throttle { .. } | dshot::DshotFrame::Command { .. }
                );
                if inverted_ok {
                    self.bidir_confirms = self.bidir_confirms.saturating_add(1);
                    if self.bidir_confirms >= crate::constants::BIDIR_CONFIRM_FRAMES {
                        bidir_detected = true;
                    }
                } else {
                    self.bidir_confirms = 0;
                    if matches!(
                        frame,
                        dshot::DshotFrame::Throttle { .. } | dshot::DshotFrame::Command { .. }
                    ) {
                        self.high_pin_count = 0;
                    }
                }
            }
            action = match frame {
                dshot::DshotFrame::Throttle { value, telemetry } => {
                    TransferAction::DshotThrottle { value, telemetry }
                }
                dshot::DshotFrame::Command { cmd, telemetry } => {
                    TransferAction::DshotCommand { cmd, telemetry }
                }
                _ => TransferAction::None,
            };
        }
        // --- Servo processing (mutually exclusive with DShot) ---
        else if servo_mode {
            if input_pin_high {
                // Rising edge — wait for falling to get pulse width
            } else if dma_buffer.len() >= 2 {
                let pulse = dma_buffer[1].wrapping_sub(dma_buffer[0]) as u16;
                action = match self.servo.compute(pulse, current_newinput, bidirectional) {
                    ServoResult::Throttle(v) => TransferAction::ServoThrottle(v),
                    ServoResult::OutOfRange => {
                        *zero_input_count = 0;
                        TransferAction::None
                    }
                    ServoResult::Calibrating | ServoResult::CalibrationHighDone => {
                        TransferAction::ServoCalibrating
                    }
                    ServoResult::CalibrationDone {
                        low_threshold_eeprom,
                        high_threshold_eeprom,
                    } => TransferAction::ServoCalibrationDone {
                        low_threshold: low_threshold_eeprom,
                        high_threshold: high_threshold_eeprom,
                    },
                };
            }
        }

        // --- Unarmed housekeeping ---
        if !armed {
            if dshot_mode && !dshot_telemetry && input_pin_high {
                self.high_pin_count = self.high_pin_count.saturating_add(1);
            }

            // DShot frame averaging (for dshot_frametime calibration)
            if dshot_mode && self.average_count < 8 && *zero_input_count > 5 {
                self.average_count += 1;
                if dma_buffer.len() >= 32 {
                    self.average_packet_length +=
                        (dma_buffer[31].wrapping_sub(dma_buffer[0])) as u16 as u32;
                }
                if self.average_count == 8 {
                    let avg = self.average_packet_length >> 3;
                    let high = (avg + (self.average_packet_length >> 7)) as u16;
                    let low = (avg - (self.average_packet_length >> 7)) as u16;
                    frametime = Some((low, high));
                }
            }

            // Calibration entry detection
            if adjusted_input == 0 && !self.servo.calibration_required() {
                *zero_input_count += 1;
            } else if !disable_stick_cal {
                *zero_input_count = 0;
                if adjusted_input > crate::constants::CALIBRATION_MIN_THROTTLE {
                    if get_abs_dif(adjusted_input as i32, self.last_input as i32)
                        > crate::constants::CALIBRATION_MAX_JITTER
                    {
                        self.enter_calibration_count = 0;
                    } else {
                        self.enter_calibration_count += 1;
                    }
                    if self.enter_calibration_count > crate::constants::CALIBRATION_ENTRY_COUNT
                        && !self.servo.high_calibration_set()
                    {
                        self.servo.set_calibration_required(true);
                        self.enter_calibration_count = 0;
                    }
                    self.last_input = adjusted_input;
                }
            }
        }

        let next_capture = if servo_mode && input_pin_high {
            CaptureConfig::SERVO_REALIGN
        } else if servo_mode {
            CaptureConfig::SERVO
        } else {
            CaptureConfig::DSHOT
        };

        TransferActions {
            action,
            next_capture,
            frametime,
            bidir_detected,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dshot_dma_buffer(value: u16, telem: bool, inverted_crc: bool) -> [u32; 32] {
        let mut bits = [0u8; 16];
        for (i, bit) in bits[..11].iter_mut().enumerate() {
            *bit = ((value >> (10 - i)) & 1) as u8;
        }
        bits[11] = u8::from(telem);

        let mut crc = (bits[0] ^ bits[4] ^ bits[8]) << 3
            | (bits[1] ^ bits[5] ^ bits[9]) << 2
            | (bits[2] ^ bits[6] ^ bits[10]) << 1
            | (bits[3] ^ bits[7] ^ bits[11]);
        if inverted_crc {
            crc = (!crc) & 0xF;
        }
        bits[12] = (crc >> 3) & 1;
        bits[13] = (crc >> 2) & 1;
        bits[14] = (crc >> 1) & 1;
        bits[15] = crc & 1;

        let mut buf = [0u32; 32];
        let mut base = 1000u32;
        for (i, bit) in bits.iter().enumerate() {
            buf[i * 2] = base;
            buf[i * 2 + 1] = base + if *bit != 0 { 22 } else { 10 };
            base += 32;
        }
        buf
    }

    #[test]
    fn servo_pin_high_requests_realign_capture() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let actions = state.process(
            &[0, 0, 0],
            true,
            false,
            true,
            false,
            false,
            true,
            0,
            0,
            false,
            false,
            &mut zic,
            400,
            600,
            64,
        );

        assert_eq!(actions.next_capture, CaptureConfig::SERVO_REALIGN);
    }

    #[test]
    fn servo_pin_low_requests_normal_capture() {
        let mut state = TransferState::default();
        state.servo.set_calibration(1100, 1900, 1500, 100);
        let mut zic = 0;
        let actions = state.process(
            &[1000, 2500],
            true,
            false,
            true,
            false,
            false,
            false,
            0,
            0,
            false,
            false,
            &mut zic,
            400,
            600,
            64,
        );

        assert_eq!(actions.next_capture, CaptureConfig::SERVO);
        assert!(matches!(actions.action, TransferAction::ServoThrottle(_)));
    }

    #[test]
    fn dshot_mode_requests_dshot_capture() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let actions = state.process(
            &[0; 32], true, true, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600, 64,
        );

        assert_eq!(actions.next_capture, CaptureConfig::DSHOT);
    }

    #[test]
    fn normal_dshot_with_high_pin_does_not_commit_bidir() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, false);

        for _ in 0..200 {
            let actions = state.process(
                &frame, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600, 64,
            );

            assert!(!actions.bidir_detected);
            assert!(matches!(
                actions.action,
                TransferAction::DshotThrottle { value: 0, .. }
            ));
        }
    }

    #[test]
    fn inverted_crc_dshot_with_high_pin_commits_bidir_after_confirms() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, true);
        let mut detected_at = None;

        for tick in 0..150 {
            let actions = state.process(
                &frame, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600, 64,
            );

            if actions.bidir_detected {
                detected_at = Some(tick);
                break;
            }
        }

        assert!(matches!(detected_at, Some(104..=110)));
    }

    #[test]
    fn bidir_autodetect_is_unarmed_only() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, true);

        for _ in 0..150 {
            let actions = state.process(
                &frame, true, true, false, false, true, true, 0, 0, false, false, &mut zic, 400,
                600, 64,
            );

            assert!(!actions.bidir_detected);
        }
    }

    #[test]
    fn bidir_autodetect_requires_high_pin() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, true);

        for _ in 0..150 {
            let actions = state.process(
                &frame, true, true, false, false, false, false, 0, 0, false, false, &mut zic, 400,
                600, 64,
            );

            assert!(!actions.bidir_detected);
        }
    }

    #[test]
    fn first_dshot_detection_stays_tentative() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let mut buf = [0u32; 33];
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = 100 + i as u32 * 5;
        }

        let actions = state.process(
            &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400, 600,
            60,
        );

        assert!(matches!(actions.action, TransferAction::None));
        assert_eq!(actions.next_capture, CaptureConfig::dshot_detection(60));
    }

    #[test]
    fn invalid_detection_clears_pending_protocol() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let mut dshot = [0u32; 33];
        for (i, slot) in dshot.iter_mut().enumerate() {
            *slot = 100 + i as u32 * 5;
        }
        let invalid = [0u32; 33];

        let _ = state.process(
            &dshot, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600, 60,
        );
        let _ = state.process(
            &invalid, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600, 60,
        );
        let actions = state.process(
            &dshot, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600, 60,
        );

        assert!(matches!(actions.action, TransferAction::None));
        assert_eq!(actions.next_capture, CaptureConfig::dshot_detection(60));
    }

    #[test]
    fn matching_dshot_detection_locks_protocol_with_rate_prescaler() {
        for (step, expected_capture) in [
            (3, CaptureConfig::DSHOT600_DETECTED),
            (5, CaptureConfig::DSHOT300_DETECTED),
            (10, CaptureConfig::DSHOT150_DETECTED),
        ] {
            let mut state = TransferState::default();
            let mut zic = 0;
            let mut buf = [0u32; 33];
            for (i, slot) in buf.iter_mut().enumerate() {
                *slot = 100 + i as u32 * step;
            }

            let _ = state.process(
                &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
                600, 60,
            );
            let actions = state.process(
                &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
                600, 60,
            );

            assert!(matches!(
                actions.action,
                TransferAction::InputDetected(DetectedProtocol::Dshot)
            ));
            assert_eq!(actions.next_capture, expected_capture);
        }
    }

    #[test]
    fn matching_servo_detection_locks_protocol() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let mut buf = [0u32; 33];
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = 100 + i as u32 * 1000;
        }

        let _ = state.process(
            &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400, 600,
            60,
        );
        let actions = state.process(
            &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400, 600,
            60,
        );

        assert!(matches!(
            actions.action,
            TransferAction::InputDetected(DetectedProtocol::Servo)
        ));
        assert_eq!(actions.next_capture, CaptureConfig::servo_detected(60));
    }

    #[test]
    fn servo_calibration_done_returns_thresholds() {
        let mut state = TransferState::default();
        state.servo.set_calibration_required(true);
        let mut zic = 0;

        for _ in 0..51 {
            let high = state.process(
                &[1000, 2900],
                true,
                false,
                true,
                false,
                false,
                false,
                0,
                0,
                false,
                false,
                &mut zic,
                400,
                600,
                64,
            );
            assert!(matches!(
                high.action,
                TransferAction::ServoCalibrating | TransferAction::ServoCalibrationDone { .. }
            ));
        }

        let mut done = TransferAction::None;
        for _ in 0..76 {
            done = state
                .process(
                    &[1000, 2100],
                    true,
                    false,
                    true,
                    false,
                    false,
                    false,
                    0,
                    0,
                    false,
                    false,
                    &mut zic,
                    400,
                    600,
                    64,
                )
                .action;
        }

        assert!(matches!(
            done,
            TransferAction::ServoCalibrationDone {
                low_threshold: _,
                high_threshold: _
            }
        ));
    }
}
