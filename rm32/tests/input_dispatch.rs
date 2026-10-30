//! Raw input through the shared dispatcher, including the ISR-to-main handoff.
use rm32::{
    board::BoardConfig,
    config::EepromConfig,
    crsf::CrsfParser,
    dshot_commands::CommandProcessor,
    edt::{EdtFrame, EdtScheduler},
    main_state::{ChipParams, MainState},
    motor_mode::MotorMode,
    platform::Platform,
    shared_state::SharedState,
    system::SystemTick,
    transfer::{self, CaptureConfig, InputContext, TransferState},
    units::{CentiAmps, CentiVolts},
};

struct Port;
impl Platform for Port {
    const TICK_HZ: u32 = 20_000;
    const TIMER_HZ: u32 = 2_000_000;
    const CAPTURE_HZ: u32 = 48_000_000;
    const GCR_SHIFT: u8 = 6;
    const PWM_BASE_PERIOD: u16 = 1999;
}
struct Rig {
    shared: SharedState,
    config: EepromConfig,
    main: MainState<Port>,
    transfer: TransferState,
    cmd: CommandProcessor,
    edt: EdtScheduler,
    edt_armed: bool,
    gate: bool,
    forward: bool,
    low: u16,
    high: u16,
    prescalers: Vec<u16>,
}
impl Rig {
    fn new() -> Self {
        let shared = SharedState::new::<Port>();
        shared.set_motor_mode(MotorMode::Armed);
        shared.set_input_set(true);
        shared.set_dshot(true);
        Self {
            shared,
            config: EepromConfig::default(),
            main: MainState::new(
                &BoardConfig::DEFAULT,
                ChipParams {
                    timer1_max_arr: 1999,
                    cpu_mhz: 48,
                },
            ),
            transfer: TransferState::default(),
            cmd: CommandProcessor::default(),
            edt: EdtScheduler::default(),
            edt_armed: false,
            gate: false,
            forward: true,
            low: 1400,
            high: 1650,
            prescalers: vec![],
        }
    }
    fn edges(&mut self, edges: &[u32], pin_high: bool) -> (CaptureConfig, Option<u8>) {
        let mut zero = self.shared.zero_input_count();
        let actions = self.transfer.process::<Port>(
            edges,
            self.shared.input_set(),
            self.shared.dshot(),
            self.shared.servo_pwm(),
            self.shared.dshot_telemetry(),
            self.shared.armed(),
            pin_high,
            self.shared.adjusted_input(),
            self.shared.newinput(),
            self.config.is_bidirectional(),
            self.config.disable_stick_calibration != 0,
            &mut zero,
            self.low,
            self.high,
        );
        self.shared.set_zero_input_count(zero);
        let result = transfer::dispatch(
            &actions,
            &self.shared,
            InputContext {
                config: &mut self.config,
                cmd: &mut self.cmd,
                edt: &mut self.edt,
                forward: &mut self.forward,
                edt_armed: &mut self.edt_armed,
                edt_arm_enable: self.gate,
                frametime_low: &mut self.low,
                frametime_high: &mut self.high,
            },
            |p| self.prescalers.push(p.raw()),
        );
        (actions.next_capture, result.tone)
    }
    fn frame(&mut self, value: u16) -> Option<u8> {
        self.edges(&capture(value, false, false), false).1
    }
    fn repeat(&mut self, command: u16) {
        for _ in 0..6 {
            self.frame(command);
        }
    }
    fn sync(&mut self) {
        SystemTick::sync_config_writes(&self.shared, &mut self.main);
    }
    fn save(&mut self) -> bool {
        SystemTick::prepare_config_save(&self.shared, &mut self.main, &self.config)
    }
    fn program(&mut self, offset: u16, byte: u16) {
        self.repeat(36);
        self.frame(offset);
        self.frame(byte);
        self.frame(37);
    }
}

// Independent packet CRC and pulse construction; never use the production codec
// to generate its own expected input. Counter wrap is intentional.
fn capture(value: u16, telemetry: bool, corrupt: bool) -> [u32; 32] {
    let payload = value * 2 + u16::from(telemetry);
    let crc =
        ((payload & 15) ^ ((payload / 16) & 15) ^ ((payload / 256) & 15)) ^ u16::from(corrupt);
    let packet = payload * 16 + crc;
    std::array::from_fn(|index| {
        let bit = index / 2;
        let high = if packet & (1 << (15 - bit)) == 0 {
            25
        } else {
            75
        };
        (u32::MAX - 200).wrapping_add(bit as u32 * 100 + if index % 2 == 0 { 0 } else { high })
    })
}

#[test]
fn detection_configures_capture_and_output_only_after_confirmation() {
    for (spacing, output, count, input_psc) in [
        (3, Some(0), 32, 0),
        (6, Some(1), 32, 1),
        (12, Some(3), 32, 3),
        (1000, None, 2, 47),
    ] {
        let mut r = Rig::new();
        r.shared.set_input_set(false);
        r.shared.set_dshot(false);
        let edges: [u32; 32] = std::array::from_fn(|i| 65520u32.wrapping_add(i as u32 * spacing));
        let first = r.edges(&edges, false).0;
        assert_eq!(first.ndtr, 32);
        assert_eq!(first.prescaler, Some(8));
        assert!(!r.shared.input_set());
        assert!(r.prescalers.is_empty());
        let second = r.edges(&edges, false).0;
        assert_eq!(second.ndtr, count);
        assert_eq!(second.prescaler, Some(input_psc));
        assert!(r.shared.input_set());
        assert_eq!(r.shared.servo_pwm(), output.is_none());
        assert_eq!(r.prescalers, output.into_iter().collect::<Vec<_>>());
    }
}

#[test]
fn invalid_frames_do_not_apply_input_refresh_timeout_or_advance_commands() {
    let mut r = Rig::new();
    r.shared.set_newinput(777);
    r.shared.set_signal_timeout(123);
    r.edges(&capture(8, true, true), false);
    r.edges(&[0; 32], false);
    r.edges(&[0; 2], false);
    assert_eq!(r.shared.newinput(), 777);
    assert_eq!(r.shared.signal_timeout(), 123);
    assert!(!r.shared.send_telemetry());
    for _ in 0..5 {
        r.frame(8);
    }
    assert_eq!(r.config.dir_reversed, 0);
    r.frame(8);
    assert_eq!(r.config.dir_reversed, 1);
}

#[test]
fn commands_publish_direction_and_config_and_preserve_repetition_and_motor_gates() {
    for mode in [MotorMode::Disarmed, MotorMode::Running] {
        let mut r = Rig::new();
        r.shared.set_motor_mode(mode);
        r.repeat(8);
        assert_eq!(r.config.dir_reversed, 0);
        assert!(r.shared.pop_config_write().is_none());
    }
    let mut r = Rig::new();
    for _ in 0..5 {
        r.frame(8);
    }
    r.frame(7); // Another command resets repetition.
    for _ in 0..5 {
        r.frame(8);
    }
    assert_eq!(r.config.dir_reversed, 0);
    r.frame(8);
    assert!(!r.forward);
    assert!(!r.shared.forward());
    assert_eq!(r.main.config.dir_reversed, 0); // No direct config mirror.
    r.sync();
    assert_eq!(r.main.config.dir_reversed, 1);
    r.repeat(10);
    r.sync();
    assert_eq!(r.main.config.bi_direction, 1);
    r.repeat(12);
    assert!(r.save());
    assert_eq!(r.main.config.as_bytes(), r.config.as_bytes());
    assert!(!r.save());
}

#[test]
fn edt_gate_telemetry_init_deinit_and_zero_throttle() {
    let mut r = Rig::new();
    r.gate = true;
    r.edges(&capture(400, true, false), false);
    assert_eq!(r.shared.newinput(), 0);
    assert!(r.shared.send_telemetry());
    assert_eq!(r.shared.signal_timeout(), 0);
    r.repeat(13);
    assert!(r.edt_armed);
    assert!(matches!(
        r.edt.next_frame(CentiAmps(0), CentiVolts(0), 0),
        EdtFrame::Extended(0xe00)
    ));
    r.frame(400);
    assert_eq!(r.shared.newinput(), 400);
    r.frame(0);
    assert!(!r.edt_armed);
    assert_eq!(r.shared.newinput(), 0);
    r.repeat(14);
    assert!(matches!(
        r.edt.next_frame(CentiAmps(0), CentiVolts(0), 0),
        EdtFrame::Extended(0xeff)
    ));
}

#[test]
fn tone_and_info_requests_remain_caller_effects() {
    let mut r = Rig::new();
    assert_eq!(r.frame(2), Some(2));
    r.repeat(6);
    assert!(r.shared.send_esc_info_flag());
    assert!(!r.shared.save_settings_flag());
}

#[test]
fn programming_uses_raw_values_suppresses_throttle_and_requires_commit() {
    let mut r = Rig::new();
    r.repeat(36);
    assert!(r.cmd.in_programming_mode());
    r.frame(48); // A decoded throttle value, used as byte offset.
    r.frame(201);
    assert_eq!(r.shared.newinput(), 0);
    let before = r.config;
    r.frame(38);
    assert_eq!(r.config.as_bytes(), before.as_bytes());
    r.edges(&capture(37, false, true), false);
    assert!(r.cmd.in_programming_mode());
    r.frame(37);
    assert!(!r.cmd.in_programming_mode());
    assert_eq!(r.config.as_bytes()[48], 201);
    assert!(!r.shared.save_settings_flag()); // Commit changes RAM; save is explicit.
    r.sync();
    assert_eq!(r.main.config.as_bytes()[48], 201);
    r.repeat(12);
    assert!(r.save());
    assert_eq!(r.main.config.as_bytes()[48], 201);
}

#[test]
fn programming_rejects_invalid_offsets_without_aliasing_and_allows_last_byte() {
    for offset in [192, 256, 304, 2047] {
        let mut r = Rig::new();
        let before = r.config;
        r.program(offset, 199);
        assert_eq!(r.config.as_bytes(), before.as_bytes());
        assert!(r.shared.pop_config_write().is_none());
        assert!(!r.cmd.in_programming_mode());
    }
    let mut r = Rig::new();
    r.program(191, 203);
    r.sync();
    assert_eq!(r.main.config.as_bytes()[191], 203);
}

#[test]
fn losing_programming_gate_drops_data_and_cancels_sequence() {
    for mode in [MotorMode::Disarmed, MotorMode::Running] {
        let mut r = Rig::new();
        r.repeat(36);
        r.frame(48);
        r.shared.set_motor_mode(mode);
        r.frame(201);
        assert!(!r.cmd.in_programming_mode());
        assert_eq!(r.shared.newinput(), 0);
        assert!(r.shared.pop_config_write().is_none());
    }
    let mut r = Rig::new();
    r.repeat(36);
    r.shared.set_motor_mode(MotorMode::Disarmed);
    r.shared.set_signal_timeout(123);
    r.edges(&capture(201, true, true), false);
    assert!(!r.cmd.in_programming_mode());
    assert_eq!(r.shared.signal_timeout(), 123);
    assert!(!r.shared.send_telemetry());
    assert!(r.shared.pop_config_write().is_none());
}

#[test]
fn queue_overflow_recovers_latest_config_and_cannot_overwrite_snapshot() {
    let mut r = Rig::new();
    for i in 0..12 {
        r.program(48 + i, 101 + i);
    }
    assert!(r.shared.config_write_overflowed());
    SystemTick::sync_config_snapshot(&r.shared, &mut r.main, &r.config);
    assert!(!r.shared.config_write_overflowed());
    r.sync();
    for i in 0..12 {
        assert_eq!(r.main.config.as_bytes()[48 + i], 101 + i as u8);
    }
    r.program(48, 209);
    r.repeat(12);
    assert!(r.save());
    assert_eq!(r.main.config.as_bytes()[48], 209);
    r.program(48, 210);
    r.repeat(12); // A request after the first snapshot must remain pending.
    assert_eq!(r.main.config.as_bytes()[48], 209);
    assert!(r.shared.save_settings_flag());
    assert!(r.save());
    assert_eq!(r.main.config.as_bytes()[48], 210);
}

#[test]
fn servo_edges_realign_validate_and_publish_calibration_even_with_full_queue() {
    let mut r = Rig::new();
    r.shared.set_motor_mode(MotorMode::Disarmed);
    r.shared.set_dshot(false);
    r.shared.set_servo_pwm(true);
    r.shared.set_adjusted_input(1900);
    r.config.disable_stick_calibration = 0;
    assert_eq!(r.edges(&[0, 1900], true).0.ndtr, 3);
    r.shared.set_signal_timeout(99);
    r.edges(&[0, 800], false);
    assert_eq!(r.shared.signal_timeout(), 99);
    for _ in 0..120 {
        r.edges(&[0, 1900], false);
    }
    for i in 0..8 {
        assert!(r.shared.push_config_write(i, i));
    }
    for _ in 0..76 {
        r.edges(&[65500, 66600], false);
    }
    assert!(r.shared.save_settings_flag());
    assert!(r.shared.config_write_overflowed());
    assert_eq!(r.config.servo_low_threshold, 187); // (1100 + 25 - 750) / 2
    assert_eq!(r.config.servo_high_threshold, 62); // (1900 - 25 - 1750) / 2
    assert!(r.save());
    assert_eq!(r.main.config.as_bytes(), r.config.as_bytes());
    assert!(r.shared.take_servo_calibration().is_none());
    r.sync();
    assert_eq!(r.main.config.servo_low_threshold, 187);
}

fn serial_frame(channels: &[u16; 16]) -> Vec<u8> {
    let mut bytes = vec![0xc8, 24, 0x16];
    bytes.resize(25, 0);
    for (i, channel) in channels.iter().enumerate() {
        for bit in 0..11 {
            let index = i * 11 + bit;
            bytes[3 + index / 8] |= (((channel >> bit) & 1) as u8) << (index % 8);
        }
    }
    let mut crc = 0u8;
    for byte in &bytes[2..] {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xd5
            } else {
                crc << 1
            };
        }
    }
    bytes.push(crc);
    bytes
}

#[test]
fn crsf_requires_complete_crc_valid_channels_and_recovers_after_short_frame() {
    let shared = SharedState::new::<Port>();
    let mut parser = CrsfParser::new();
    shared.set_newinput(777);
    shared.set_signal_timeout(99);
    let frame = serial_frame(&[172; 16]);
    for byte in &frame[..25] {
        parser.dispatch_byte(*byte, &shared);
    }
    assert_eq!(shared.newinput(), 777);
    assert_eq!(shared.signal_timeout(), 99);
    parser.dispatch_byte(frame[25] ^ 1, &shared);
    assert!(!shared.input_set());
    // CRC of the type byte 0x16 is 0xd3; valid CRC cannot justify a missing payload.
    for byte in [0xc8, 2, 0x16, 0xd3] {
        parser.dispatch_byte(byte, &shared);
    }
    assert_eq!(shared.newinput(), 777);
    for (value, expected) in [(172, 0), (1811, 2047), (992, 1024)] {
        let mut channels = [1811; 16];
        channels[2] = value;
        for byte in serial_frame(&channels) {
            parser.dispatch_byte(byte, &shared);
        }
        assert!(shared.input_set());
        assert_eq!(shared.newinput(), expected);
        assert_eq!(shared.signal_timeout(), 0);
    }
}
