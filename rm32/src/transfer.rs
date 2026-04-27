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
    pub average_count: u8,
    pub average_packet_length: u32,
    // Calibration entry
    pub enter_calibration_count: u8,
    pub last_input: u16,
}

/// Actions the caller (ISR) should take after transfer complete.
#[derive(Default)]
pub struct TransferActions {
    pub newinput: Option<u16>,
    pub send_telemetry: bool,
    pub signal_timeout_reset: bool,
    pub input_detected: bool,
    pub input_is_dshot: bool,
    pub input_is_servo: bool,
    pub save_settings: bool,
    pub play_tone: u8,      // 0=none, 1=default, 2=changed, 3=beacon
    pub dshot_command: u16, // 0=none, 1-47=DShot command to dispatch
    pub frametime_high: Option<u16>,
    pub frametime_low: Option<u16>,
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
    ) -> TransferActions {
        let mut actions = TransferActions::default();

        // --- Input detection ---
        if !input_set {
            let sig = signal::detect_input(dma_buffer, 48);
            match sig {
                signal::SignalType::Dshot600 | signal::SignalType::Dshot300 => {
                    actions.input_detected = true;
                    actions.input_is_dshot = true;
                }
                signal::SignalType::ServoPwm => {
                    actions.input_detected = true;
                    actions.input_is_servo = true;
                }
                _ => {}
            }
            return actions;
        }

        // --- DShot processing ---
        if dshot_mode && dma_buffer.len() >= 32 {
            let buf: [u32; 32] = {
                let mut b = [0u32; 32];
                b.copy_from_slice(&dma_buffer[..32]);
                b
            };
            let frame = dshot::decode_frame(&buf, frametime_low, frametime_high, dshot_telemetry);
            match frame {
                dshot::DshotFrame::Throttle { value, telemetry } => {
                    actions.newinput = Some(value);
                    actions.send_telemetry = telemetry;
                    actions.signal_timeout_reset = true;
                }
                dshot::DshotFrame::Command { cmd, telemetry } => {
                    actions.newinput = Some(0);
                    actions.send_telemetry = telemetry;
                    actions.signal_timeout_reset = true;
                    actions.dshot_command = cmd;
                }
                _ => {} // bad CRC or timing
            }
        }

        // --- Servo processing ---
        if servo_mode {
            if input_pin_high {
                // Rising edge — wait for falling to get pulse width
                // buffersize = 3 (capture next edge)
            } else {
                // Falling edge — pulse complete
                if dma_buffer.len() >= 2 {
                    let pulse = dma_buffer[1].wrapping_sub(dma_buffer[0]) as u16;
                    match self.servo.compute(pulse, current_newinput, bidirectional) {
                        ServoResult::Throttle(v) => {
                            actions.newinput = Some(v);
                            actions.signal_timeout_reset = true;
                        }
                        ServoResult::OutOfRange => {
                            *zero_input_count = 0;
                        }
                        ServoResult::Calibrating => {
                            actions.signal_timeout_reset = true;
                        }
                        ServoResult::CalibrationHighDone => {
                            actions.play_tone = 1; // default tone
                            actions.signal_timeout_reset = true;
                        }
                        ServoResult::CalibrationDone { .. } => {
                            actions.save_settings = true;
                            actions.play_tone = 2; // changed tone
                            actions.signal_timeout_reset = true;
                        }
                    }
                }
            }
        }

        // --- Unarmed housekeeping ---
        if !armed {
            // DShot frame averaging (for dshot_frametime calibration)
            if dshot_mode && self.average_count < 8 && *zero_input_count > 5 {
                self.average_count += 1;
                if dma_buffer.len() >= 32 {
                    self.average_packet_length +=
                        (dma_buffer[31].wrapping_sub(dma_buffer[0])) as u16 as u32;
                }
                if self.average_count == 8 {
                    let avg = self.average_packet_length >> 3;
                    actions.frametime_high = Some((avg + (self.average_packet_length >> 7)) as u16);
                    actions.frametime_low = Some((avg - (self.average_packet_length >> 7)) as u16);
                }
            }

            // Calibration entry detection
            if adjusted_input == 0 && !self.servo.calibration_required {
                *zero_input_count += 1;
            } else if !disable_stick_cal {
                *zero_input_count = 0;
                if adjusted_input > 1500 {
                    if get_abs_dif(adjusted_input as i32, self.last_input as i32) > 50 {
                        self.enter_calibration_count = 0;
                    } else {
                        self.enter_calibration_count += 1;
                    }
                    if self.enter_calibration_count > 50 && !self.servo.high_calibration_set {
                        actions.play_tone = 3; // beacon
                        self.servo.calibration_required = true;
                        self.enter_calibration_count = 0;
                    }
                    self.last_input = adjusted_input;
                }
            }
        }

        actions
    }
}
