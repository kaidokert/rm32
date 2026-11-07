//! Transfer complete dispatcher.
//!
//! Equivalent to C `transfercomplete()` — the central dispatcher that handles
//! DMA completion for both DShot and servo input, auto-detection, bidir
//! telemetry, and unarmed frame averaging.

use crate::dshot;
use crate::functions::get_abs_dif;
use crate::platform::Platform;
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

/// Capture setup requested for the next input frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureConfig {
    /// Edges (timestamps) to capture before the next frame callback (a
    /// DMA port's transfer count; AM32's `buffersize`).
    pub edges_per_burst: u32,
    /// Timer prescaler update, if protocol detection changed the capture rate.
    pub prescaler: Option<u16>,
}

impl CaptureConfig {
    pub const DSHOT: Self = Self {
        edges_per_burst: 32,
        prescaler: None,
    };
    /// DShot detection uses one full frame. Using 33 edges consumes the next
    /// frame's first edge and can leave steady-state capture misaligned.
    pub const fn dshot_detection<P: Platform>() -> Self {
        Self {
            edges_per_burst: 32,
            prescaler: Some((P::CAPTURE_HZ / 6_000_000) as u16),
        }
    }
    pub const SERVO: Self = Self {
        edges_per_burst: 2,
        prescaler: None,
    };
    pub const SERVO_REALIGN: Self = Self {
        edges_per_burst: 3,
        prescaler: None,
    };

    pub const DSHOT600_DETECTED: Self = Self {
        edges_per_burst: 32,
        prescaler: Some(0),
    };
    pub const DSHOT300_DETECTED: Self = Self {
        edges_per_burst: 32,
        prescaler: Some(1),
    };
    pub const DSHOT150_DETECTED: Self = Self {
        edges_per_burst: 32,
        prescaler: Some(3),
    };

    pub const fn servo_detected<P: Platform>() -> Self {
        Self {
            edges_per_burst: 2,
            prescaler: Some((P::CAPTURE_HZ / 1_000_000).saturating_sub(1) as u16),
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
    /// `edges`: the captured DMA data (32 entries for DShot, 2-3 for servo)
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
    /// `P`: the port's `Platform` (`CAPTURE_HZ` sets the detection prescalers)
    #[allow(clippy::too_many_arguments)]
    pub fn process<P: Platform>(
        &mut self,
        edges: &[u32],
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
        let mut action = TransferAction::None;
        let mut frametime = None;

        // --- Input detection ---
        if !input_set {
            let sig = signal::detect_input(edges);
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
                    signal::SignalType::ServoPwm => CaptureConfig::servo_detected::<P>(),
                    signal::SignalType::None => {
                        unreachable!("confirmed input detection cannot have SignalType::None")
                    }
                };
                (TransferAction::InputDetected(protocol), capture)
            } else {
                (TransferAction::None, CaptureConfig::dshot_detection::<P>())
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
        if dshot_mode && edges.len() >= 32 {
            let buf: [u32; 32] = {
                let mut b = [0u32; 32];
                b.copy_from_slice(&edges[..32]);
                b
            };
            let frame = dshot::decode_frame(&buf, frametime_low, frametime_high, dshot_telemetry);
            if !armed
                && !dshot_telemetry
                && input_pin_high
                && self.high_pin_count >= crate::constants::BIDIR_IDLE_HIGH_FRAMES
            {
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
            } else if edges.len() >= 2 {
                let pulse = edges[1].wrapping_sub(edges[0]) as u16;
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
            } else {
                self.high_pin_count = 0;
                self.bidir_confirms = 0;
            }
        } else {
            self.high_pin_count = 0;
            self.bidir_confirms = 0;
        }

        if !armed {
            // DShot frame averaging (for dshot_frametime calibration)
            if dshot_mode && self.average_count < 8 && *zero_input_count > 5 {
                self.average_count += 1;
                if edges.len() >= 32 {
                    self.average_packet_length += (edges[31].wrapping_sub(edges[0])) as u16 as u32;
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

    /// Process one DShot packet from a port that slices bits in hardware
    /// (PIO, RMT, SPI; no edge capture): [`process`](Self::process)'s DShot
    /// path at layer 1 ([`dshot::unpack`]). The port is DShot-only, so
    /// detection is two consecutive packets with a valid CRC (normal or
    /// bidirectional), like `process`'s two matching frames. Because a
    /// pre-sliced 16-bit word carries no edge timestamps, wire-rate
    /// configuration stays with the port (`next_capture` is always
    /// [`CaptureConfig::DSHOT`] with `prescaler: None`, and there is no
    /// frame time to average); everything else matches `process`.
    #[allow(clippy::too_many_arguments)]
    pub fn process_packet(
        &mut self,
        packet: u16,
        input_set: bool,
        dshot_telemetry: bool,
        armed: bool,
        input_pin_high: bool,
        adjusted_input: u16,
        disable_stick_cal: bool,
        zero_input_count: &mut u16,
    ) -> TransferActions {
        if !input_set {
            let valid = |f| {
                matches!(
                    f,
                    dshot::DshotFrame::Throttle { .. } | dshot::DshotFrame::Command { .. }
                )
            };
            let protocol = (valid(dshot::unpack(packet, false))
                || valid(dshot::unpack(packet, true)))
            .then_some(DetectedProtocol::Dshot);
            let action = match self.confirm_protocol(protocol) {
                Some(p) => TransferAction::InputDetected(p),
                None => TransferAction::None,
            };
            return TransferActions {
                action,
                next_capture: CaptureConfig::DSHOT,
                frametime: None,
                bidir_detected: false,
            };
        }
        let frame = dshot::unpack(packet, dshot_telemetry);
        let bidir_detected =
            self.bidir_probe(frame, armed, dshot_telemetry, input_pin_high, || {
                dshot::unpack(packet, true)
            });
        let action = frame_action(frame);
        self.idle_high_count(armed, true, dshot_telemetry, input_pin_high);
        if !armed {
            self.calibration_entry(adjusted_input, disable_stick_cal, zero_input_count);
        }
        TransferActions {
            action,
            next_capture: CaptureConfig::DSHOT,
            frametime: None,
            bidir_detected,
        }
    }

    /// Protocol re-confirmation: commit only on the second consecutive
    /// matching detection.
    #[inline(always)]
    fn confirm_protocol(&mut self, protocol: Option<DetectedProtocol>) -> Option<DetectedProtocol> {
        match (protocol, self.pending_protocol) {
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
        }
    }

    /// Bidirectional DShot auto-detect: while unarmed with the line idling
    /// high, count consecutive frames that decode under the inverted CRC
    /// (`inverted` decodes this frame that way); true once confirmed.
    #[inline(always)]
    fn bidir_probe(
        &mut self,
        frame: dshot::DshotFrame,
        armed: bool,
        dshot_telemetry: bool,
        input_pin_high: bool,
        inverted: impl FnOnce() -> dshot::DshotFrame,
    ) -> bool {
        let mut bidir_detected = false;
        if !armed
            && !dshot_telemetry
            && input_pin_high
            && self.high_pin_count >= crate::constants::BIDIR_IDLE_HIGH_FRAMES
        {
            let inverted_ok = matches!(
                inverted(),
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
        bidir_detected
    }

    /// Idle-high frame count for the bidir auto-detect (unarmed only).
    #[inline(always)]
    fn idle_high_count(
        &mut self,
        armed: bool,
        dshot_mode: bool,
        dshot_telemetry: bool,
        input_pin_high: bool,
    ) {
        if !armed {
            if dshot_mode && !dshot_telemetry && input_pin_high {
                self.high_pin_count = self.high_pin_count.saturating_add(1);
            } else {
                self.high_pin_count = 0;
                self.bidir_confirms = 0;
            }
        } else {
            self.high_pin_count = 0;
            self.bidir_confirms = 0;
        }
    }

    /// Stick-calibration entry detection (unarmed only).
    #[inline(always)]
    fn calibration_entry(
        &mut self,
        adjusted_input: u16,
        disable_stick_cal: bool,
        zero_input_count: &mut u16,
    ) {
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
}

/// A decoded frame's dispatch action.
#[inline(always)]
fn frame_action(frame: dshot::DshotFrame) -> TransferAction {
    match frame {
        dshot::DshotFrame::Throttle { value, telemetry } => {
            TransferAction::DshotThrottle { value, telemetry }
        }
        dshot::DshotFrame::Command { cmd, telemetry } => {
            TransferAction::DshotCommand { cmd, telemetry }
        }
        _ => TransferAction::None,
    }
}

/// ISR-side input state the transfer dispatch reads and updates (AM32
/// `transfercomplete`'s globals). Firmware and harness both build it from
/// their own fields.
pub struct InputContext<'a> {
    pub config: &'a mut crate::config::EepromConfig,
    pub cmd: &'a mut crate::dshot_commands::CommandProcessor,
    pub edt: &'a mut crate::edt::EdtScheduler,
    pub forward: &'a mut bool,
    pub edt_armed: &'a mut bool,
    pub edt_arm_enable: bool,
    pub frametime_low: &'a mut u16,
    pub frametime_high: &'a mut u16,
}

/// Apply one frame's `TransferActions` to the shared and ISR-side state:
/// the shared dispatch used by both firmware and the host harness.
/// `set_prescaler` receives the bidirectional DShot output prescaler when
/// detection locks DShot (the port's register work).
///
/// `on_command` runs first on a decoded command. Both callbacks run inside
/// their respective match arms so LLVM does not tail-duplicate shared-state
/// stores into a second path.
#[inline(always)]
pub fn dispatch<C: FnOnce(u16), P: FnOnce(crate::hal::DshotOutputPrescaler)>(
    actions: &TransferActions,
    shared: &crate::shared_state::SharedState,
    mut ctx: InputContext<'_>,
    on_command: C,
    set_prescaler: P,
) {
    // A sequence started while stopped must not survive loss of its gate.
    // Drop a decoded DShot frame in this call as well: its programming bytes
    // are not throttle. An invalid frame still cancels an ineligible sequence,
    // but otherwise retains the normal invalid-frame/no-side-effect behavior.
    let cancelled = ctx.cmd.in_programming_mode() && (!shared.armed() || shared.running());
    if cancelled {
        ctx.cmd.cancel_programming();
    }
    match actions.action {
        TransferAction::InputDetected(proto) => {
            shared.set_input_set(true);
            match proto {
                DetectedProtocol::Dshot => {
                    shared.set_dshot(true);
                    if let Some(prescaler) = actions.next_capture.prescaler {
                        set_prescaler(
                            crate::hal::DshotOutputPrescaler::new(prescaler)
                                .expect("invalid DShot output prescaler"),
                        );
                    }
                }
                DetectedProtocol::Servo => shared.set_servo_pwm(true),
            }
        }
        TransferAction::DshotThrottle { value, telemetry } => {
            if cancelled {
                shared.set_newinput(0);
            } else if ctx.cmd.in_programming_mode() {
                shared.set_newinput(0);
                apply_command(value, shared, &mut ctx);
            } else {
                // EDT arm gate: throttle passes once armed (or EDT not
                // required); zero always passes and disarms. The gate reads
                // `edt_armed` before the disarm; the store comes last so LLVM
                // keeps one `newinput` store (store-then-disarm got
                // tail-duplicated).
                let pass = !ctx.edt_arm_enable || *ctx.edt_armed || value == 0;
                if value == 0 && ctx.edt_arm_enable {
                    *ctx.edt_armed = false;
                }
                if pass {
                    shared.set_newinput(value);
                }
            }
            if telemetry {
                shared.set_send_telemetry(true);
            }
            shared.set_signal_timeout(0);
        }
        TransferAction::DshotCommand { cmd, telemetry } => {
            on_command(cmd);
            shared.set_newinput(0);
            if telemetry {
                shared.set_send_telemetry(true);
            }
            shared.set_signal_timeout(0);
            if !cancelled {
                apply_command(cmd, shared, &mut ctx);
            }
        }
        TransferAction::ServoThrottle(value) => {
            shared.set_newinput(value);
            shared.set_signal_timeout(0);
        }
        TransferAction::ServoCalibrating => {
            shared.set_signal_timeout(0);
        }
        TransferAction::ServoCalibrationDone {
            low_threshold,
            high_threshold,
        } => {
            // Persist to the config; publish the bytes so main's copy is
            // current when it acts on the save flag.
            ctx.config.servo_low_threshold = low_threshold;
            ctx.config.servo_high_threshold = high_threshold;
            shared.publish_servo_calibration(low_threshold, high_threshold);
            shared.push_config_write(
                core::mem::offset_of!(crate::config::EepromConfig, servo_low_threshold) as u8,
                low_threshold,
            );
            shared.push_config_write(
                core::mem::offset_of!(crate::config::EepromConfig, servo_high_threshold) as u8,
                high_threshold,
            );
            shared.set_save_settings_flag(true);
            shared.set_signal_timeout(0);
        }
        TransferAction::None => {}
    }
    if let Some((low, high)) = actions.frametime {
        *ctx.frametime_low = low;
        *ctx.frametime_high = high;
    }
    if actions.bidir_detected {
        shared.set_dshot_telemetry(true);
    }
}

/// One command or programming byte. Programming updates the config directly
/// too: publish each changed byte once, never truncating an unvalidated
/// programming position to a queue key.
#[inline(always)]
fn apply_command(
    value: u16,
    shared: &crate::shared_state::SharedState,
    ctx: &mut InputContext<'_>,
) {
    use crate::dshot_commands::CommandResult;
    let old_config = *ctx.config;
    let result = if ctx.cmd.in_programming_mode() {
        ctx.cmd
            .process_programming(value, ctx.config)
            .unwrap_or(CommandResult::None)
    } else {
        ctx.cmd.process(
            value,
            shared.armed(),
            shared.running(),
            ctx.config,
            ctx.forward,
            ctx.edt_armed,
            ctx.edt_arm_enable,
        )
    };
    for (offset, (old, new)) in old_config
        .as_bytes()
        .iter()
        .zip(ctx.config.as_bytes().iter())
        .enumerate()
    {
        if old != new {
            shared.push_config_write(offset as u8, *new);
        }
    }
    match result {
        CommandResult::SaveSettings => shared.set_save_settings_flag(true),
        CommandResult::PlayTone(tone) => shared.set_tone_request(tone),
        CommandResult::SendEscInfo => shared.set_send_esc_info_flag(true),
        CommandResult::ProgrammingCommit { .. } | CommandResult::None => {}
    }
    shared.set_forward(*ctx.forward);
    if ctx.cmd.take_edt_init() {
        ctx.edt.request_init();
    }
    if ctx.cmd.take_edt_deinit() {
        ctx.edt.request_deinit();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test platforms at the capture clocks the tests were written for.
    macro_rules! mhz {
        ($($name:ident = $mhz:expr),*) => {$(
            struct $name;
            impl Platform for $name {
                const TICK_HZ: u32 = 20_000;
                const TIMER_HZ: u32 = 2_000_000;
                const CAPTURE_HZ: u32 = $mhz * 1_000_000;
                const GCR_SHIFT: u8 = 7;
                const VOLTAGE_DIVIDER_X100: u16 = 1100;
                const MILLIVOLT_PER_AMP: u16 = 20;
                const CURRENT_OFFSET: i16 = 0;
                const USE_NTC: bool = false;
                const ADC_FULL_SCALE: u16 = 4095;
                const ADC_VREF_MV: u16 = 3300;
                const PWM_BASE_PERIOD: u16 = 1999;
            }
        )*};
    }
    mhz!(Mhz60 = 60, Mhz64 = 64, Mhz80 = 80);

    const DSHOT300_START_TICKS: u32 = 5000;
    const DSHOT300_ZERO_HIGHS: [u32; 16] = [
        47, 47, 47, 46, 47, 47, 47, 46, 47, 46, 47, 47, 46, 47, 47, 47,
    ];
    const DSHOT300_ZERO_LOWS: [u32; 16] = [
        87, 86, 87, 87, 86, 87, 87, 87, 86, 87, 87, 86, 87, 87, 87, 0,
    ];
    const INTER_FRAME_GAP_TICKS: u32 = 78_400;

    #[test]
    fn capture_config_edges_per_burst() {
        assert_eq!(CaptureConfig::DSHOT.edges_per_burst, 32);
        assert_eq!(CaptureConfig::SERVO.edges_per_burst, 2);
        assert_eq!(CaptureConfig::SERVO_REALIGN.edges_per_burst, 3);
    }

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

    fn dshot300_disarm_capture() -> [u32; 32] {
        let mut buf = [0u32; 32];
        let mut t = DSHOT300_START_TICKS;
        for i in 0..16 {
            buf[i * 2] = t;
            buf[i * 2 + 1] = t + DSHOT300_ZERO_HIGHS[i];
            t += DSHOT300_ZERO_HIGHS[i] + DSHOT300_ZERO_LOWS[i];
        }
        buf
    }

    fn gap_spanning_capture() -> [u32; 32] {
        let mut buf = [0u32; 32];
        let mut t = DSHOT300_START_TICKS;
        for slot in buf.iter_mut().take(9) {
            *slot = t;
            t += 67;
        }
        t += INTER_FRAME_GAP_TICKS;
        for slot in buf.iter_mut().skip(9) {
            *slot = t;
            t += 67;
        }
        buf
    }

    #[test]
    fn servo_pin_high_requests_realign_capture() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let actions = state.process::<Mhz64>(
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
        );

        assert_eq!(actions.next_capture, CaptureConfig::SERVO_REALIGN);
    }

    #[test]
    fn servo_pin_low_requests_normal_capture() {
        let mut state = TransferState::default();
        state.servo.set_calibration(1100, 1900, 1500, 100);
        let mut zic = 0;
        let actions = state.process::<Mhz64>(
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
        );

        assert_eq!(actions.next_capture, CaptureConfig::SERVO);
        assert!(matches!(actions.action, TransferAction::ServoThrottle(_)));
    }

    #[test]
    fn dshot_mode_requests_dshot_capture() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let actions = state.process::<Mhz64>(
            &[0; 32], true, true, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600,
        );

        assert_eq!(actions.next_capture, CaptureConfig::DSHOT);
    }

    #[test]
    fn dshot_detection_uses_one_frame_capture_window() {
        assert_eq!(
            CaptureConfig::dshot_detection::<Mhz60>().edges_per_burst,
            32
        );
        assert_eq!(
            CaptureConfig::dshot_detection::<Mhz60>().prescaler,
            Some(10)
        );
    }

    /// The 16-bit packet `dshot_dma_buffer` captures.
    fn dshot_packet(value: u16, telem: bool, inverted_crc: bool) -> u16 {
        let head = value << 1 | u16::from(telem);
        let mut crc = (head ^ head >> 4 ^ head >> 8) & 0xF;
        if inverted_crc {
            crc = !crc & 0xF;
        }
        head << 4 | crc
    }

    /// One frame through both paths, from the same state: identical
    /// actions and state, or panic. Returns `process_packet`'s result.
    struct Twin {
        edges: TransferState,
        packet: TransferState,
        zic_e: u16,
        zic_p: u16,
        frames: u32,
    }

    #[derive(Clone, Copy)]
    struct Frame {
        value: u16,
        telem: bool,
        inverted: bool,
        corrupt_bit: Option<usize>,
        armed: bool,
        dshot_telemetry: bool,
        pin_high: bool,
        adjusted: u16,
        no_cal: bool,
    }

    impl Twin {
        /// Create a fresh pair of edge-capture and packet-slicing transfer states.
        fn new() -> Self {
            Self {
                edges: TransferState::default(),
                packet: TransferState::default(),
                zic_e: 0,
                zic_p: 0,
                frames: 0,
            }
        }

        /// Advance both `process` and `process_packet` by one frame and assert identical state.
        fn step(&mut self, f: Frame) -> TransferActions {
            extern crate std;
            use std::format;
            let mut buf = dshot_dma_buffer(f.value, f.telem, f.inverted);
            let mut packet = dshot_packet(f.value, f.telem, f.inverted);
            if let Some(bit) = f.corrupt_bit {
                let one = packet >> (15 - bit) & 1 != 0;
                buf[bit * 2 + 1] = buf[bit * 2] + if one { 10 } else { 22 };
                packet ^= 1 << (15 - bit);
            }
            assert_eq!(
                dshot::decode_frame(&buf, 400, 600, false),
                dshot::unpack(packet, false)
            );
            let e = self.edges.process::<Mhz64>(
                &buf,
                true,
                true,
                false,
                f.dshot_telemetry,
                f.armed,
                f.pin_high,
                f.adjusted,
                0,
                false,
                f.no_cal,
                &mut self.zic_e,
                400,
                600,
            );
            let p = self.packet.process_packet(
                packet,
                true,
                f.dshot_telemetry,
                f.armed,
                f.pin_high,
                f.adjusted,
                f.no_cal,
                &mut self.zic_p,
            );
            let ctx = format!(
                "frame {}: value {} packet {packet:#06x}",
                self.frames, f.value
            );
            self.frames += 1;
            assert_eq!(
                format!("{:?}", e.action),
                format!("{:?}", p.action),
                "{ctx}"
            );
            assert_eq!(e.bidir_detected, p.bidir_detected, "{ctx}");
            assert_eq!(e.next_capture, p.next_capture, "{ctx}");
            assert_eq!(self.zic_e, self.zic_p, "{ctx}");
            let (a, b) = (&self.edges, &self.packet);
            assert_eq!(a.high_pin_count, b.high_pin_count, "{ctx}");
            assert_eq!(a.bidir_confirms, b.bidir_confirms, "{ctx}");
            assert_eq!(
                a.enter_calibration_count, b.enter_calibration_count,
                "{ctx}"
            );
            assert_eq!(a.last_input, b.last_input, "{ctx}");
            assert_eq!(
                a.servo.calibration_required(),
                b.servo.calibration_required(),
                "{ctx}"
            );
            p
        }
    }

    /// `process_packet` is `process`'s DShot path at layer 1. The bidir
    /// probe, idle-high count and calibration entry exist twice (`process`
    /// keeps its own copy so the firmware's DShot ISR stays as compiled),
    /// so drive both with the same frames and states and require identical
    /// actions and state: 20k pseudo-random frames, then the two stateful
    /// paths random frames rarely complete (calibration entry, bidir
    /// commit), each asserted to have fired. Frame-time averaging is
    /// `process`-only (a packet port has no frame time) and not compared.
    #[test]
    fn packet_path_matches_capture_path() {
        let mut t = Twin::new();
        let mut seed = 0x1234_5678u32;
        let mut next = |n: u32| {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) % n
        };
        // The bidir probe needs 100 consecutive idle-high frames first,
        // which random states never give: the last phase covers it.
        let (mut throttles, mut commands, mut bad) = (0, 0, 0);
        for _ in 0..20_000 {
            let corrupt = next(8) == 0;
            let f = Frame {
                value: next(2048) as u16,
                telem: next(2) == 1,
                inverted: next(3) == 0,
                corrupt_bit: corrupt.then(|| next(16) as usize),
                armed: next(4) == 0,
                dshot_telemetry: next(4) == 0,
                pin_high: next(5) != 0,
                adjusted: [0, 0, 100, 1800, 1830, next(2048) as u16][next(6) as usize],
                no_cal: next(10) == 0,
            };
            match t.step(f).action {
                TransferAction::DshotThrottle { .. } => throttles += 1,
                TransferAction::DshotCommand { .. } => commands += 1,
                _ => bad += 1,
            }
        }
        let counts = (throttles, commands, bad);
        assert!(
            throttles > 1000 && commands > 10 && bad > 1000,
            "(throttles, commands, bad) = {counts:?}"
        );

        // Stick-calibration entry: unarmed, steady high stick (> 1500).
        let steady = Frame {
            value: 1000,
            telem: false,
            inverted: false,
            corrupt_bit: None,
            armed: false,
            dshot_telemetry: false,
            pin_high: false,
            adjusted: 1800,
            no_cal: false,
        };
        let mut t = Twin::new();
        for _ in 0..200 {
            t.step(steady);
        }
        assert!(
            t.packet.servo.calibration_required(),
            "calibration entry never fired"
        );

        // Bidir commit: unarmed, line idling high, inverted-CRC frames.
        // First exercise an interrupted confirm sequence (valid normal frame
        // after idle-high threshold resets `bidir_confirms` and `high_pin_count`),
        // then run a full uninterrupted confirm sequence to completion.
        let mut t = Twin::new();
        for _ in 0..105 {
            t.step(Frame {
                inverted: true,
                pin_high: true,
                adjusted: 0,
                ..steady
            });
        }
        assert!(t.packet.bidir_confirms > 0);
        t.step(Frame {
            inverted: false,
            pin_high: true,
            adjusted: 0,
            ..steady
        });
        assert_eq!(t.packet.bidir_confirms, 0);
        assert_eq!(t.packet.high_pin_count, 1);

        let mut committed = false;
        for _ in 0..300 {
            committed |= t
                .step(Frame {
                    inverted: true,
                    pin_high: true,
                    adjusted: 0,
                    ..steady
                })
                .bidir_detected;
        }
        assert!(committed, "bidir auto-detect never committed");
    }

    #[test]
    fn packet_detection_needs_two_valid_packets() {
        let mut s = TransferState::default();
        let mut zic = 0;
        let good = dshot_packet(1000, false, false);
        let r = s.process_packet(good, false, false, false, false, 0, false, &mut zic);
        assert!(matches!(r.action, TransferAction::None));
        let r = s.process_packet(good ^ 1, false, false, false, false, 0, false, &mut zic);
        assert!(
            matches!(r.action, TransferAction::None),
            "bad CRC clears the pending detection"
        );
        let r = s.process_packet(good, false, false, false, false, 0, false, &mut zic);
        assert!(matches!(r.action, TransferAction::None));
        let bidir = dshot_packet(1000, false, true);
        let r = s.process_packet(bidir, false, false, false, false, 0, false, &mut zic);
        assert!(matches!(
            r.action,
            TransferAction::InputDetected(DetectedProtocol::Dshot)
        ));
    }

    #[test]
    fn normal_dshot_with_high_pin_does_not_commit_bidir() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, false);

        for _ in 0..200 {
            let actions = state.process::<Mhz64>(
                &frame, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600,
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
        let expected_detect_at = crate::constants::BIDIR_IDLE_HIGH_FRAMES as usize
            + crate::constants::BIDIR_CONFIRM_FRAMES as usize
            - 1;

        for tick in 0..150 {
            let actions = state.process::<Mhz64>(
                &frame, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600,
            );

            if actions.bidir_detected {
                detected_at = Some(tick);
                break;
            }
        }

        assert_eq!(detected_at, Some(expected_detect_at));
    }

    #[test]
    fn bidir_autodetect_is_unarmed_only() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, true);

        for _ in 0..150 {
            let actions = state.process::<Mhz64>(
                &frame, true, true, false, false, true, true, 0, 0, false, false, &mut zic, 400,
                600,
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
            let actions = state.process::<Mhz64>(
                &frame, true, true, false, false, false, false, 0, 0, false, false, &mut zic, 400,
                600,
            );

            assert!(!actions.bidir_detected);
        }
    }

    #[test]
    fn bidir_autodetect_resets_when_pin_goes_low() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let normal = dshot_dma_buffer(0, false, false);
        let inverted = dshot_dma_buffer(0, false, true);

        for _ in 0..150 {
            let actions = state.process::<Mhz64>(
                &normal, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600,
            );
            assert!(!actions.bidir_detected);
        }

        for _ in 0..10 {
            let actions = state.process::<Mhz64>(
                &inverted, true, true, false, false, false, false, 0, 0, false, false, &mut zic,
                400, 600,
            );
            assert!(!actions.bidir_detected);
        }
    }

    #[test]
    fn bidir_autodetect_resets_when_armed() {
        let mut state = TransferState::default();
        let mut zic = 0;
        let frame = dshot_dma_buffer(0, false, true);
        let partial_confirm_frames = crate::constants::BIDIR_IDLE_HIGH_FRAMES as usize
            + crate::constants::BIDIR_CONFIRM_FRAMES as usize
            - 1;

        for _ in 0..partial_confirm_frames {
            let actions = state.process::<Mhz64>(
                &frame, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600,
            );
            assert!(!actions.bidir_detected);
        }

        let armed = state.process::<Mhz64>(
            &frame, true, true, false, false, true, true, 0, 0, false, false, &mut zic, 400, 600,
        );
        assert!(!armed.bidir_detected);

        for _ in 0..crate::constants::BIDIR_CONFIRM_FRAMES {
            let actions = state.process::<Mhz64>(
                &frame, true, true, false, false, false, true, 0, 0, false, false, &mut zic, 400,
                600,
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

        let actions = state.process::<Mhz60>(
            &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400, 600,
        );

        assert!(matches!(actions.action, TransferAction::None));
        assert_eq!(
            actions.next_capture,
            CaptureConfig::dshot_detection::<Mhz60>()
        );
        assert_eq!(
            actions.next_capture.edges_per_burst,
            CaptureConfig::DSHOT.edges_per_burst
        );
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

        let _ = state.process::<Mhz60>(
            &dshot, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600,
        );
        let _ = state.process::<Mhz60>(
            &invalid, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600,
        );
        let actions = state.process::<Mhz60>(
            &dshot, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
            600,
        );

        assert!(matches!(actions.action, TransferAction::None));
        assert_eq!(
            actions.next_capture,
            CaptureConfig::dshot_detection::<Mhz60>()
        );
    }

    #[test]
    fn bf_dshot300_disarm_frame_decodes() {
        let buf = dshot300_disarm_capture();
        let mut state = TransferState::default();
        let mut zic = 0u16;
        let actions = state.process::<Mhz80>(
            &buf, true, true, false, false, false, false, 0, 0, false, false, &mut zic, 100, 60000,
        );

        assert!(
            matches!(
                actions.action,
                TransferAction::DshotThrottle { value: 0, .. }
            ),
            "expected DShot throttle 0, got {:?}",
            actions.action
        );
    }

    #[test]
    fn gap_spanning_capture_rejected() {
        let buf = gap_spanning_capture();
        let frame = dshot::decode_frame(&buf, 400, 600, false);

        assert!(
            matches!(frame, dshot::DshotFrame::InvalidTiming),
            "gap-spanning capture decoded as {:?}",
            frame
        );
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

            let _ = state.process::<Mhz60>(
                &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
                600,
            );
            let actions = state.process::<Mhz60>(
                &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400,
                600,
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

        let _ = state.process::<Mhz60>(
            &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400, 600,
        );
        let actions = state.process::<Mhz60>(
            &buf, false, false, false, false, false, false, 0, 0, false, false, &mut zic, 400, 600,
        );

        assert!(matches!(
            actions.action,
            TransferAction::InputDetected(DetectedProtocol::Servo)
        ));
        assert_eq!(
            actions.next_capture,
            CaptureConfig::servo_detected::<Mhz60>()
        );
    }

    #[test]
    fn servo_calibration_done_returns_thresholds() {
        let mut state = TransferState::default();
        state.servo.set_calibration_required(true);
        let mut zic = 0;

        for _ in 0..51 {
            let high = state.process::<Mhz64>(
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
            );
            assert!(matches!(
                high.action,
                TransferAction::ServoCalibrating | TransferAction::ServoCalibrationDone { .. }
            ));
        }

        let mut done = TransferAction::None;
        for _ in 0..76 {
            done = state
                .process::<Mhz64>(
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

    // ---- Shared dispatch tests -----------------------------------------

    /// Test fixture holding the ISR-owned input context fields for `dispatch`.
    struct Isr {
        config: crate::config::EepromConfig,
        cmd: crate::dshot_commands::CommandProcessor,
        edt: crate::edt::EdtScheduler,
        forward: bool,
        edt_armed: bool,
        edt_arm_enable: bool,
        ft: (u16, u16),
    }

    impl Isr {
        /// Create default ISR input state with the given `edt_arm_enable` setting.
        fn new(edt_arm_enable: bool) -> Self {
            Self {
                config: crate::config::EepromConfig::default(),
                cmd: Default::default(),
                edt: Default::default(),
                forward: true,
                edt_armed: false,
                edt_arm_enable,
                ft: (400, 600),
            }
        }
        /// Dispatch one action; returns (commands hooked, prescaler set).
        fn run(
            &mut self,
            shared: &crate::shared_state::SharedState,
            action: TransferAction,
        ) -> (Option<u16>, Option<u16>) {
            let actions = TransferActions {
                action,
                next_capture: CaptureConfig::DSHOT300_DETECTED,
                frametime: None,
                bidir_detected: false,
            };
            let (mut hooked, mut psc) = (None, None);
            dispatch(
                &actions,
                shared,
                InputContext {
                    config: &mut self.config,
                    cmd: &mut self.cmd,
                    edt: &mut self.edt,
                    forward: &mut self.forward,
                    edt_armed: &mut self.edt_armed,
                    edt_arm_enable: self.edt_arm_enable,
                    frametime_low: &mut self.ft.0,
                    frametime_high: &mut self.ft.1,
                },
                |cmd| assert!(hooked.replace(cmd).is_none()),
                |p| psc = Some(p.raw()),
            );
            (hooked, psc)
        }
    }

    /// Create a `SharedState` instance pre-Transitioned to the armed state.
    fn armed_shared() -> crate::shared_state::SharedState {
        let shared = crate::shared_state::SharedState::new::<Mhz64>();
        shared.set_armed(true);
        assert!(shared.armed());
        shared
    }

    /// Build a non-telemetry `TransferAction::DshotCommand` for tests.
    fn command(cmd: u16) -> TransferAction {
        TransferAction::DshotCommand {
            cmd,
            telemetry: false,
        }
    }

    /// A config-changing command publishes exactly the changed byte to main
    /// and updates the direction.
    #[test]
    fn dispatch_command_publishes_config_diff() {
        let shared = armed_shared();
        let mut isr = Isr::new(false);
        for _ in 0..6 {
            assert_eq!(
                isr.run(&shared, command(crate::dshot::commands::DIRECTION_REVERSED))
                    .0,
                Some(crate::dshot::commands::DIRECTION_REVERSED)
            );
        }
        assert_eq!(isr.config.dir_reversed, 1);
        assert!(!isr.forward);
        assert!(!shared.forward());
        let off = core::mem::offset_of!(crate::config::EepromConfig, dir_reversed) as u8;
        assert_eq!(shared.pop_config_write(), Some((off, 1)));
        assert_eq!(shared.pop_config_write(), None);
    }

    /// Beacon commands become the tick tone request.
    #[test]
    fn dispatch_beacon_requests_tone() {
        let shared = armed_shared();
        let mut isr = Isr::new(false);
        isr.run(&shared, command(3));
        assert_eq!(shared.take_tone_request(), 3);
    }

    /// EDT arm gate: throttle is held until EDT_ENABLE arms it, zero always
    /// passes and disarms, and EDT_ENABLE queues the EDT init frame.
    #[test]
    fn dispatch_edt_arm_gate() {
        let shared = armed_shared();
        let mut isr = Isr::new(true);
        let throttle = |value| TransferAction::DshotThrottle {
            value,
            telemetry: false,
        };
        shared.set_newinput(7);
        isr.run(&shared, throttle(500));
        assert_eq!(shared.newinput(), 7, "held before EDT arms");
        for _ in 0..6 {
            isr.run(&shared, command(crate::dshot::commands::EDT_ENABLE));
        }
        assert!(isr.edt_armed);
        assert!(matches!(
            isr.edt.next_frame(
                crate::units::CentiAmps::ZERO,
                crate::units::CentiVolts::ZERO,
                0
            ),
            crate::edt::EdtFrame::Extended(crate::edt::EDT_INIT_FRAME)
        ));
        isr.run(&shared, throttle(500));
        assert_eq!(shared.newinput(), 500, "passes once armed");
        isr.run(&shared, throttle(0));
        assert_eq!(shared.newinput(), 0);
        assert!(!isr.edt_armed, "zero disarms");
        isr.run(&shared, throttle(500));
        assert_eq!(shared.newinput(), 0, "held again");
    }

    /// Servo calibration persists both thresholds, publishes both bytes and
    /// asks main to save.
    #[test]
    fn dispatch_servo_calibration_publishes_thresholds() {
        let shared = armed_shared();
        let mut isr = Isr::new(false);
        isr.run(
            &shared,
            TransferAction::ServoCalibrationDone {
                low_threshold: 51,
                high_threshold: 99,
            },
        );
        use crate::config::EepromConfig;
        assert_eq!(
            (
                isr.config.servo_low_threshold,
                isr.config.servo_high_threshold
            ),
            (51, 99)
        );
        let lo = core::mem::offset_of!(EepromConfig, servo_low_threshold) as u8;
        let hi = core::mem::offset_of!(EepromConfig, servo_high_threshold) as u8;
        assert_eq!(shared.pop_config_write(), Some((lo, 51)));
        assert_eq!(shared.pop_config_write(), Some((hi, 99)));
        assert!(shared.save_settings_flag());
    }

    /// DShot detection hands the port its output prescaler; no command hook.
    #[test]
    fn dispatch_detection_sets_output_prescaler() {
        let shared = crate::shared_state::SharedState::new::<Mhz64>();
        let mut isr = Isr::new(false);
        let (hooked, psc) = isr.run(
            &shared,
            TransferAction::InputDetected(DetectedProtocol::Dshot),
        );
        assert_eq!((hooked, psc), (None, Some(1)));
        assert!(shared.input_set() && shared.dshot());
    }
}
