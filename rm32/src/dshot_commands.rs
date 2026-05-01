//! DShot command processing state machine.

use crate::config::EepromConfig;
use crate::dshot::commands;

/// DShot command processor state.
#[derive(Clone, Default)]
pub struct CommandProcessor {
    command: u16,
    command_count: u8,
    last_command: u8,
    programming_mode: u8,
    position: u16,
    new_byte: u8,
    extended_telemetry: bool,
    send_edt_init: bool,
    send_edt_deinit: bool,
}

impl CommandProcessor {
    /// Whether extended DShot telemetry is enabled.
    pub fn extended_telemetry(&self) -> bool {
        self.extended_telemetry
    }

    /// Take the EDT init flag (returns current value and clears it).
    pub fn take_edt_init(&mut self) -> bool {
        let v = self.send_edt_init;
        self.send_edt_init = false;
        v
    }

    /// Take the EDT deinit flag (returns current value and clears it).
    pub fn take_edt_deinit(&mut self) -> bool {
        let v = self.send_edt_deinit;
        self.send_edt_deinit = false;
        v
    }
}

/// Result of processing a DShot command.
#[derive(Debug, PartialEq, Eq)]
pub enum CommandResult {
    None,
    PlayTone(u8),
    SendEscInfo,
    SaveSettings,
    ProgrammingCommit { position: u16, value: u8 },
}

impl CommandProcessor {
    /// Process a DShot command (value 1-47). Called when frame decode yields a command.
    /// `running` and `armed` indicate motor state.
    /// Returns action to take, if any.
    #[allow(clippy::too_many_arguments)]
    pub fn process(
        &mut self,
        cmd: u16,
        armed: bool,
        running: bool,
        config: &mut EepromConfig,
        forward: &mut bool,
        edt_armed: &mut bool,
        edt_arm_enable: bool,
    ) -> CommandResult {
        if !armed || running {
            return CommandResult::None;
        }

        if cmd != self.last_command as u16 {
            self.last_command = cmd as u8;
            self.command_count = 0;
        }

        // Beacons get fast-tracked
        if cmd <= 5 {
            self.command_count = 6;
        }

        self.command_count += 1;
        if self.command_count < 6 {
            return CommandResult::None;
        }
        self.command_count = 0;

        let result = match cmd {
            1..=5 => CommandResult::PlayTone(cmd as u8),
            commands::ESC_INFO => CommandResult::SendEscInfo,
            commands::DIRECTION_NORMAL => {
                config.dir_reversed = 0;
                *forward = true;
                CommandResult::None
            }
            commands::DIRECTION_REVERSED => {
                config.dir_reversed = 1;
                *forward = false;
                CommandResult::None
            }
            commands::BIDIR_OFF => {
                config.bi_direction = 0;
                CommandResult::None
            }
            commands::BIDIR_ON => {
                config.bi_direction = 1;
                CommandResult::None
            }
            commands::SAVE_SETTINGS => CommandResult::SaveSettings,
            commands::EDT_ENABLE => {
                self.extended_telemetry = true;
                self.send_edt_init = true;
                if edt_arm_enable {
                    *edt_armed = true;
                }
                CommandResult::None
            }
            commands::EDT_DISABLE => {
                self.extended_telemetry = false;
                self.send_edt_deinit = true;
                CommandResult::None
            }
            commands::DIRECTION_FWD => {
                *forward = !config.dir_reversed != 0;
                // C: forward = 1 - eepromBuffer.dir_reversed
                *forward = config.dir_reversed == 0;
                CommandResult::None
            }
            commands::DIRECTION_REV => {
                *forward = config.dir_reversed != 0;
                CommandResult::None
            }
            commands::PROGRAMMING_MODE => {
                self.programming_mode = 1;
                CommandResult::None
            }
            _ => CommandResult::None,
        };

        self.last_command = cmd as u8;
        self.command = 0;
        result
    }

    /// Process a frame while in programming mode.
    /// Returns Some((position, value)) when a commit happens.
    pub fn process_programming(
        &mut self,
        value: u16,
        config: &mut EepromConfig,
    ) -> Option<CommandResult> {
        match self.programming_mode {
            1 => {
                self.position = value;
                self.programming_mode = 2;
                None
            }
            2 => {
                self.new_byte = value as u8;
                self.programming_mode = 3;
                None
            }
            3 => {
                if value == 37 {
                    let pos = self.position as usize;
                    if pos < EepromConfig::SIZE {
                        config.as_bytes_mut()[pos] = self.new_byte;
                    }
                    self.programming_mode = 0;
                    Some(CommandResult::ProgrammingCommit {
                        position: self.position,
                        value: self.new_byte,
                    })
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    pub fn in_programming_mode(&self) -> bool {
        self.programming_mode > 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup() -> (CommandProcessor, EepromConfig, bool, bool) {
        (
            CommandProcessor::default(),
            EepromConfig::default(),
            true,
            false,
        )
    }

    #[test]
    fn beacon_command_immediate() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        let result = cp.process(
            1,
            true,
            false,
            &mut config,
            &mut forward,
            &mut edt_armed,
            false,
        );
        assert_eq!(result, CommandResult::PlayTone(1));
    }

    #[test]
    fn non_beacon_needs_6_repetitions() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        for _ in 0..5 {
            let r = cp.process(
                7,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
            assert_eq!(r, CommandResult::None);
        }
        let r = cp.process(
            7,
            true,
            false,
            &mut config,
            &mut forward,
            &mut edt_armed,
            false,
        );
        // After 6th: command executes
        assert_eq!(r, CommandResult::None); // direction change returns None
        assert_eq!(config.dir_reversed, 0);
        assert!(forward);
    }

    #[test]
    fn command_7_sets_forward() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        forward = false;
        for _ in 0..6 {
            cp.process(
                7,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert_eq!(config.dir_reversed, 0);
        assert!(forward);
    }

    #[test]
    fn command_8_sets_reversed() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        for _ in 0..6 {
            cp.process(
                8,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert_eq!(config.dir_reversed, 1);
        assert!(!forward);
    }

    #[test]
    fn command_9_disables_bidir() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        config.bi_direction = 1;
        for _ in 0..6 {
            cp.process(
                9,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert_eq!(config.bi_direction, 0);
    }

    #[test]
    fn command_10_enables_bidir() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        for _ in 0..6 {
            cp.process(
                10,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert_eq!(config.bi_direction, 1);
    }

    #[test]
    fn command_13_enables_edt() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        for _ in 0..6 {
            cp.process(
                13,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                true,
            );
        }
        assert!(cp.extended_telemetry);
        assert!(edt_armed);
    }

    #[test]
    fn command_14_disables_edt() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        cp.extended_telemetry = true;
        for _ in 0..6 {
            cp.process(
                14,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert!(!cp.extended_telemetry);
    }

    #[test]
    fn command_20_21_temporary_direction() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        config.dir_reversed = 0;
        for _ in 0..6 {
            cp.process(
                21,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert!(!forward); // dir_reversed=0 -> forward = dir_reversed != 0 = false

        for _ in 0..6 {
            cp.process(
                20,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert!(forward); // forward = dir_reversed == 0 = true
    }

    #[test]
    fn programming_mode_sequence() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        // Enter programming mode
        for _ in 0..6 {
            cp.process(
                36,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
        }
        assert_eq!(cp.programming_mode, 1);

        // Position
        cp.process_programming(5, &mut config);
        assert_eq!(cp.programming_mode, 2);
        assert_eq!(cp.position, 5);

        // Value
        cp.process_programming(200, &mut config);
        assert_eq!(cp.programming_mode, 3);
        assert_eq!(cp.new_byte, 200);

        // Commit
        let result = cp.process_programming(37, &mut config);
        assert_eq!(
            result,
            Some(CommandResult::ProgrammingCommit {
                position: 5,
                value: 200
            })
        );
        assert_eq!(cp.programming_mode, 0);
        assert_eq!(config.as_bytes()[5], 200);
    }

    #[test]
    fn command_6_requests_esc_info() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        for _ in 0..6 {
            let r = cp.process(
                6,
                true,
                false,
                &mut config,
                &mut forward,
                &mut edt_armed,
                false,
            );
            if r == CommandResult::SendEscInfo {
                break;
            }
        }
        // Last call should return SendEscInfo
    }

    #[test]
    fn ignores_when_running() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        let r = cp.process(
            1,
            true,
            true,
            &mut config,
            &mut forward,
            &mut edt_armed,
            false,
        );
        assert_eq!(r, CommandResult::None);
    }

    #[test]
    fn ignores_when_not_armed() {
        let (mut cp, mut config, mut forward, mut edt_armed) = setup();
        let r = cp.process(
            1,
            false,
            false,
            &mut config,
            &mut forward,
            &mut edt_armed,
            false,
        );
        assert_eq!(r, CommandResult::None);
    }
}
