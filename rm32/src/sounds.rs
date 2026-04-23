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

    /// Startup tune: plays BlueJay tune if present in EEPROM, else default 3-note tune.
    pub fn play_startup(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
    ) {
        sys.disable_irq();
        self.play_note(pwm, phase, sys, 55, 3, 200);
        self.play_note(pwm, phase, sys, 40, 5, 200);
        self.play_note(pwm, phase, sys, 25, 6, 200);
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// Startup with BlueJay tune check: plays custom tune if EEPROM tune data exists.
    pub fn play_startup_with_tune(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
        tune: &[u8; 128],
        cpu_mhz: u32,
    ) {
        sys.disable_irq();
        if tune[0] != 0xFF {
            // BlueJay tune present
            self.play_bluejay_tune(pwm, phase, sys, tune, cpu_mhz);
        } else {
            // Default startup tune
            self.play_note(pwm, phase, sys, 55, 3, 200);
            self.play_note(pwm, phase, sys, 40, 5, 200);
            self.play_note(pwm, phase, sys, 25, 6, 200);
        }
        self.silence(pwm, phase);
        sys.enable_irq();
    }

    /// Play a BlueJay-encoded tune from the 128-byte EEPROM tune array.
    fn play_bluejay_tune(
        &self,
        pwm: &mut impl PwmOutput,
        phase: &mut impl PhaseOutput,
        sys: &mut impl System,
        tune: &[u8; 128],
        cpu_mhz: u32,
    ) {
        let mut full_time_count = 0u32;
        phase.com_step(3);

        let mut i = 4;
        while i < 126 {
            sys.reload_watchdog();
            let t4 = tune[i];
            let t3 = tune[i + 1];

            if t4 == 0 && t3 == 0 {
                break;
            }

            if t4 == 255 && t3 != 0 {
                // Extend duration
                full_time_count += 1;
            } else if t3 == 0 {
                // Silence: duration = full_time_count * 255 + t4
                let duration = full_time_count * 255 + t4 as u32;
                pwm.set_duty_all(0);
                sys.delay_millis(duration);
                full_time_count = 0;
            } else {
                // Note: compute frequency and duration
                let total_pulses = full_time_count * 255 + t4 as u32;
                let t3_period = t3 as u32 * 247 + 4000;
                let duration = (total_pulses * t3_period) / 11000;
                let freq = 10_000_000u32 / t3_period;

                // Play note using prescaler=9 and computed ARR
                let timer_reload = cpu_mhz * 100_000 / freq;
                pwm.set_prescaler(9);
                pwm.set_auto_reload(timer_reload as u16);
                let duty = self.volume as u32 * timer_reload / self.tim1_autoreload as u32;
                pwm.set_duty_all(duty as u16);
                sys.delay_millis(duration);
                full_time_count = 0;
            }

            // Inter-note gap (if tune[3] > 239)
            if tune[3] > 239 {
                pwm.set_duty_all(0);
                sys.delay_millis(10 * (255 - tune[3] as u32));
            }

            i += 2;
        }
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
