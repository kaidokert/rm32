//! Tick-driven motor tones.
//!
//! Runtime tones share the bridge with commutation and braking, so the control
//! interrupt owns their PWM writes. Each tick either advances the note timer,
//! changes the note, or silences the tone. Starting motor output, braking, or
//! sine stepping always returns bridge ownership to that path.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Note {
    pub prescaler: u16,
    pub step: u8,
    pub milliseconds: u16,
}

pub const TONE_ARMED: u8 = 6;
pub const DEFAULT_VOLUME: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToneAction {
    Idle,
    StartNote(Note),
    Silence,
}

#[derive(Default)]
pub struct ToneScheduler {
    sequence: u8,
    note_index: u8,
    ticks_left: u32,
    repeats_left: u8,
    gap: bool,
    active: bool,
}

const DEFAULT_TUNE: [Note; 2] = [
    Note {
        prescaler: 50,
        step: 2,
        milliseconds: 150,
    },
    Note {
        prescaler: 30,
        step: 2,
        milliseconds: 150,
    },
];
const CHANGED_TUNE: [Note; 2] = [
    Note {
        prescaler: 40,
        step: 2,
        milliseconds: 150,
    },
    Note {
        prescaler: 80,
        step: 2,
        milliseconds: 150,
    },
];
const INPUT_TUNE_2: [Note; 3] = [
    Note {
        prescaler: 60,
        step: 1,
        milliseconds: 75,
    },
    Note {
        prescaler: 80,
        step: 1,
        milliseconds: 75,
    },
    Note {
        prescaler: 90,
        step: 1,
        milliseconds: 75,
    },
];
const ARMED_TUNE: [Note; 3] = [
    Note {
        prescaler: 80,
        step: 3,
        milliseconds: 100,
    },
    Note {
        prescaler: 70,
        step: 3,
        milliseconds: 100,
    },
    Note {
        prescaler: 40,
        step: 3,
        milliseconds: 100,
    },
];

fn beacon_note(index: u8) -> Option<Note> {
    if index >= 60 {
        return None;
    }
    let i = 119 - 2 * index as u16;
    Some(Note {
        prescaler: 10 + i / 2,
        // Step zero is the existing all-off tail of the public beacon tune.
        step: (i / 20) as u8,
        milliseconds: 10,
    })
}

impl ToneScheduler {
    fn note(&self) -> Option<Note> {
        if self.gap {
            return None;
        }
        match self.sequence {
            1 | 5 => DEFAULT_TUNE.get(self.note_index as usize).copied(),
            2 => CHANGED_TUNE.get(self.note_index as usize).copied(),
            3 => beacon_note(self.note_index),
            4 => INPUT_TUNE_2.get(self.note_index as usize).copied(),
            TONE_ARMED => ARMED_TUNE.get(self.note_index as usize).copied(),
            _ => None,
        }
    }

    fn begin_note<P: crate::constants::Timing>(&mut self) -> ToneAction {
        match self.note() {
            Some(note) => {
                self.ticks_left = note.milliseconds as u32 * P::CONTROL_TICKS_PER_MS as u32;
                ToneAction::StartNote(note)
            }
            None => {
                self.active = false;
                ToneAction::Silence
            }
        }
    }

    /// Advance one control tick. Requests 1..=5 are DShot beacon commands;
    /// the high byte carries the repeat count for the arming tune request.
    /// `bridge_busy` gives precedence to motor drive, braking and sine output.
    pub fn tick<P: crate::constants::Timing>(
        &mut self,
        request: u16,
        bridge_busy: bool,
    ) -> ToneAction {
        if bridge_busy {
            if self.active {
                self.active = false;
                self.gap = false;
                return ToneAction::Silence;
            }
            return ToneAction::Idle;
        }

        if !self.active {
            let sequence = request as u8;
            if sequence == 0 {
                return ToneAction::Idle;
            }
            self.sequence = sequence;
            self.note_index = 0;
            self.gap = false;
            self.repeats_left = if sequence == TONE_ARMED {
                ((request >> 8) as u8).max(1)
            } else {
                1
            };
            self.active = true;
            return self.begin_note::<P>();
        }

        if self.ticks_left > 1 {
            self.ticks_left -= 1;
            return ToneAction::Idle;
        }

        if self.gap {
            self.gap = false;
            self.note_index = 0;
            return self.begin_note::<P>();
        }

        self.note_index += 1;
        if self.sequence == TONE_ARMED
            && self.note_index as usize == ARMED_TUNE.len()
            && self.repeats_left > 1
        {
            self.repeats_left -= 1;
            self.gap = true;
            self.ticks_left = 100 * P::CONTROL_TICKS_PER_MS as u32;
            return ToneAction::Silence;
        }
        self.begin_note::<P>()
    }

    pub fn active(&self) -> bool {
        self.active
    }

    fn duty(&self, configured_volume: u8) -> u16 {
        if self.sequence == TONE_ARMED {
            duty(DEFAULT_VOLUME)
        } else {
            duty(configured_volume)
        }
    }
}

pub fn duty(beep_volume: u8) -> u16 {
    ((if beep_volume == 0 {
        DEFAULT_VOLUME
    } else {
        beep_volume
    })
    .min(11) as u16)
        * 3
}

/// Apply one scheduler decision. During ordinary notes no writes are needed:
/// `ten_khz_tick` yields bridge output to the active scheduler. If motor or
/// brake output takes ownership, restore timer settings without touching its
/// phase or duty writes.
pub fn step<P, B, M>(
    tone: &mut ToneScheduler,
    request: u16,
    bridge_busy: bool,
    beep_volume: u8,
    base_period: u16,
    bridge: M,
) where
    P: crate::constants::Timing,
    B: crate::bridge::Bridge,
    M: FnOnce() -> B,
{
    match tone.tick::<P>(request, bridge_busy) {
        ToneAction::StartNote(note) => {
            let mut bridge = bridge();
            bridge.set_period(base_period);
            bridge.set_tone_divider(note.prescaler);
            bridge.set_duty_all(tone.duty(beep_volume));
            bridge.com_step(note.step);
        }
        ToneAction::Silence => {
            let mut bridge = bridge();
            if bridge_busy {
                // The motor or brake path already applied this tick's phase
                // and compare. Reset pitch/period only; preserve those writes.
                bridge.set_tone_divider(0);
                bridge.set_period(base_period);
            } else {
                bridge.all_off();
                bridge.set_tone_divider(0);
                bridge.set_period(base_period);
            }
        }
        ToneAction::Idle => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::Platform;

    struct TestTime;
    impl Platform for TestTime {
        const TICK_HZ: u32 = 20_000;
        const TIMER_HZ: u32 = 2_000_000;
        const CAPTURE_HZ: u32 = 64_000_000;
        const GCR_SHIFT: u8 = 7;
        const PWM_BASE_PERIOD: u16 = 1999;
    }

    #[test]
    fn dshot_sequences_have_public_note_values_and_durations() {
        let mut scheduler = ToneScheduler::default();
        assert_eq!(
            scheduler.tick::<TestTime>(1, false),
            ToneAction::StartNote(DEFAULT_TUNE[0])
        );
        for _ in 1..3000 {
            assert_eq!(scheduler.tick::<TestTime>(0, false), ToneAction::Idle);
        }
        assert_eq!(
            scheduler.tick::<TestTime>(0, false),
            ToneAction::StartNote(DEFAULT_TUNE[1])
        );
        for _ in 1..3000 {
            assert_eq!(scheduler.tick::<TestTime>(0, false), ToneAction::Idle);
        }
        assert_eq!(scheduler.tick::<TestTime>(0, false), ToneAction::Silence);
        assert!(!scheduler.active());

        let mut beacon_repeat = ToneScheduler::default();
        assert_eq!(
            beacon_repeat.tick::<TestTime>(5, false),
            ToneAction::StartNote(DEFAULT_TUNE[0])
        );

        assert_eq!(beacon_note(0).unwrap().prescaler, 69);
        assert_eq!(beacon_note(59).unwrap().step, 0);

        let mut changed = ToneScheduler::default();
        assert_eq!(
            changed.tick::<TestTime>(2, false),
            ToneAction::StartNote(CHANGED_TUNE[0])
        );
        for _ in 1..3000 {
            changed.tick::<TestTime>(0, false);
        }
        assert_eq!(
            changed.tick::<TestTime>(0, false),
            ToneAction::StartNote(CHANGED_TUNE[1])
        );

        let mut input2 = ToneScheduler::default();
        assert_eq!(
            input2.tick::<TestTime>(4, false),
            ToneAction::StartNote(INPUT_TUNE_2[0])
        );
        for _ in 1..1500 {
            input2.tick::<TestTime>(0, false);
        }
        assert_eq!(
            input2.tick::<TestTime>(0, false),
            ToneAction::StartNote(INPUT_TUNE_2[1])
        );

        let mut beacon = ToneScheduler::default();
        let mut action = beacon.tick::<TestTime>(3, false);
        let mut notes = 0;
        for _ in 0..(60 * 10 * 20 + 1) {
            match action {
                ToneAction::StartNote(note) => {
                    assert_eq!(note, beacon_note(notes).unwrap());
                    notes += 1;
                }
                ToneAction::Silence => break,
                ToneAction::Idle => {}
            }
            action = beacon.tick::<TestTime>(0, false);
        }
        assert_eq!(notes, 60);
        assert!(!beacon.active());
    }

    #[test]
    fn arming_tune_repeats_with_a_silent_gap_per_cell() {
        let mut scheduler = ToneScheduler::default();
        assert_eq!(
            scheduler.tick::<TestTime>((3 << 8) | TONE_ARMED as u16, false),
            ToneAction::StartNote(ARMED_TUNE[0])
        );
        for repeat in 0..3 {
            for note in 0..3 {
                for _ in 1..2000 {
                    scheduler.tick::<TestTime>(0, false);
                }
                let action = scheduler.tick::<TestTime>(0, false);
                if note < 2 {
                    assert_eq!(action, ToneAction::StartNote(ARMED_TUNE[note + 1]));
                } else {
                    assert_eq!(action, ToneAction::Silence);
                }
            }
            if repeat < 2 {
                for _ in 1..2000 {
                    assert_eq!(scheduler.tick::<TestTime>(0, false), ToneAction::Idle);
                }
                assert_eq!(
                    scheduler.tick::<TestTime>(0, false),
                    ToneAction::StartNote(ARMED_TUNE[0])
                );
            }
        }
        assert!(!scheduler.active());
    }

    #[test]
    fn busy_bridge_aborts_without_claiming_or_rewriting_motor_output() {
        let mut scheduler = ToneScheduler::default();
        assert!(matches!(
            scheduler.tick::<TestTime>(1, false),
            ToneAction::StartNote(_)
        ));
        assert_eq!(scheduler.tick::<TestTime>(0, true), ToneAction::Silence);
        assert!(!scheduler.active());
        assert_eq!(scheduler.tick::<TestTime>(2, true), ToneAction::Idle);
        assert_eq!(
            scheduler.tick::<TestTime>(2, false),
            ToneAction::StartNote(CHANGED_TUNE[0])
        );
    }

    #[test]
    fn volume_keeps_legacy_default_and_scales_configured_volume() {
        assert_eq!(duty(0), 15);
        assert_eq!(duty(5), 15);
        assert_eq!(duty(11), 33);
        assert_eq!(duty(20), 33);
        let mut arming = ToneScheduler::default();
        arming.tick::<TestTime>((2 << 8) | TONE_ARMED as u16, false);
        assert_eq!(arming.duty(9), 15);
        let mut dshot = ToneScheduler::default();
        dshot.tick::<TestTime>(1, false);
        assert_eq!(dshot.duty(9), 27);
    }
}
