//! Motor-driven beep/tune generation.
//!
//! Produces audible tones by driving motor phases at audio frequencies.
//! Each note: activate a phase pair via com_step, set PWM prescaler for
//! frequency, set low duty cycle for volume, delay for duration.

use crate::hal::{PwmOutput, PhaseOutput, System};

/// Beep volume (0-11 maps to duty 0-33).
pub struct Sounds {
    volume: u8,
    tim1_autoreload: u16,
}

/// A single note: prescaler value (frequency) and duration in ms.
struct Note {
    prescaler: u16,
    step: u8,
    duration_ms: u16,
}

impl Sounds {
    pub fn new(tim1_autoreload: u16) -> Self {
        Self {
            volume: 15, // default ~volume 5
            tim1_autoreload,
        }
    }

    pub fn set_volume(&mut self, volume: u8) {
        let v = if volume > 11 { 11 } else { volume };
        self.volume = v * 3;
    }

    fn play_note(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
        prescaler: u16,
        step: u8,
        duration_ms: u32,
    ) {
        pwm.set_auto_reload(self.tim1_autoreload);
        pwm.set_prescaler(prescaler);
        pwm.set_duty_all(self.volume as u16);
        phase.com_step(step);
        sys.delay_millis(duration_ms);
        sys.reload_watchdog();
    }

    fn silence(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
    ) {
        phase.all_off();
        pwm.set_prescaler(0);
        pwm.set_auto_reload(self.tim1_autoreload);
    }

    /// Startup tune: three ascending tones.
    pub fn play_startup(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        sys.disable_irq();
        self.play_note(pwm, phase, sys, 55, 3, 200);
        self.play_note(pwm, phase, sys, 40, 5, 200);
        self.play_note(pwm, phase, sys, 25, 1, 200);
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// Input detected tune: three descending tones.
    pub fn play_input(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        sys.disable_irq();
        self.play_note(pwm, phase, sys, 80, 3, 100);
        self.play_note(pwm, phase, sys, 70, 3, 100);
        self.play_note(pwm, phase, sys, 40, 3, 100);
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// Second input tune (higher pitched).
    pub fn play_input2(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        sys.disable_irq();
        self.play_note(pwm, phase, sys, 60, 1, 75);
        self.play_note(pwm, phase, sys, 80, 1, 75);
        self.play_note(pwm, phase, sys, 90, 1, 75);
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// Default settings tone.
    pub fn play_default(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        self.play_note(pwm, phase, sys, 50, 2, 150);
        self.play_note(pwm, phase, sys, 30, 2, 150);
        self.silence(pwm, phase);
    }

    /// Settings changed tone (inverse of default).
    pub fn play_changed(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        self.play_note(pwm, phase, sys, 40, 2, 150);
        self.play_note(pwm, phase, sys, 80, 2, 150);
        self.silence(pwm, phase);
    }

    /// Beacon tune: sweeping frequency.
    pub fn play_beacon(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        sys.disable_irq();
        let mut i = 119i16;
        while i > 0 {
            sys.reload_watchdog();
            let step = (i / 20) as u8;
            let psc = 10 + (i / 2) as u16;
            self.play_note(pwm, phase, sys, psc, step, 10);
            i -= 2;
        }
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// Brushed motor startup tune: four ascending tones.
    pub fn play_brushed_startup(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        sys.disable_irq();
        self.play_note(pwm, phase, sys, 40, 1, 300);
        self.play_note(pwm, phase, sys, 30, 2, 300);
        self.play_note(pwm, phase, sys, 25, 3, 300);
        self.play_note(pwm, phase, sys, 20, 4, 300);
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// "Dusking" tune: descending-ascending melody.
    pub fn play_dusking(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        self.play_note(pwm, phase, sys, 60, 2, 200);
        self.play_note(pwm, phase, sys, 55, 2, 150);
        self.play_note(pwm, phase, sys, 50, 2, 150);
        self.play_note(pwm, phase, sys, 45, 2, 100);
        self.play_note(pwm, phase, sys, 50, 2, 100);
        self.play_note(pwm, phase, sys, 55, 2, 100);
        self.play_note(pwm, phase, sys, 25, 2, 200);
        self.play_note(pwm, phase, sys, 55, 2, 150);
        self.silence(pwm, phase);
    }

    /// Play a Blue Jay note at specific frequency and duration.
    pub fn play_note_freq(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
        freq_hz: u16,
        duration_ms: u16,
        cpu_mhz: u32,
    ) {
        let prescaler = 9u16;
        let reload = (cpu_mhz * 100000 / freq_hz as u32) as u16;
        pwm.set_prescaler(prescaler);
        pwm.set_auto_reload(reload);
        let scaled_vol = self.volume as u32 * reload as u32 / self.tim1_autoreload as u32;
        pwm.set_duty_all(scaled_vol as u16);
        phase.com_step(3);
        sys.delay_millis(duration_ms as u32);
    }
}
