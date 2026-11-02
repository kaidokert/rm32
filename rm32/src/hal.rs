//! Hardware Abstraction Layer traits.
//!
//! Uses `embedded-hal` 1.0 traits where applicable. ESC-specific traits
//! are defined here for things embedded-hal doesn't cover (comparator,
//! phase output, commutation timer, etc).
//!
//! Implementations are provided by platform crates (rm32_stm32, rm32_std).

/// Re-export embedded-hal traits we use
pub use embedded_hal::delay::DelayNs;
pub use embedded_hal::digital::InputPin;
pub use embedded_hal::pwm::SetDutyCycle;

/// The combined power-stage interface.
pub use crate::bridge::Bridge;

/// A motor phase, independent of the port's comparator mux encoding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Phase {
    A,
    B,
    C,
}

/// Low-side gate drive mode for PWM-driven bridge legs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PwmMode {
    /// Non-complementary drive: high-side FET switches PWM while the low-side
    /// FET stays off (freewheeling through the body diode).
    Diode = 0,
    /// Complementary PWM: high-side and low-side FETs switch in hardware
    /// complementary PWM with dead-time insertion (synchronous rectification).
    Complementary = 1,
}

impl PwmMode {
    /// True when this mode is [`PwmMode::Complementary`].
    #[inline(always)]
    pub const fn is_complementary(self) -> bool {
        matches!(self, Self::Complementary)
    }
}

const FLOATING_PHASE_TABLE: [Phase; 8] = {
    let mut table = [Phase::C; 8];
    let mut step = 1u8;
    while step <= 6 {
        let legs = crate::bridge::step_legs(step, PwmMode::Diode);
        table[step as usize] = if matches!(legs[0], crate::bridge::LegRole::Float) {
            Phase::A
        } else if matches!(legs[1], crate::bridge::LegRole::Float) {
            Phase::B
        } else {
            Phase::C
        };
        step += 1;
    }
    table
};

/// Floating phase for six-step commutation. Invalid steps retain the
/// established comparator fallback to C; they do not wrap into valid steps.
///
/// Uses a compile-time lookup table so Cortex-M0 targets emit a direct indexed
/// load instead of a multi-branch bit-test chain on every commutation.
#[inline(always)]
pub const fn floating_phase(step: u8) -> Phase {
    if step < 8 {
        FLOATING_PHASE_TABLE[step as usize]
    } else {
        Phase::C
    }
}

/// Comparator routing and edge-interrupt control for BEMF sensing.
pub trait BemfSensor {
    /// Route the floating phase and edge without unmasking the interrupt.
    /// The port handles mux encoding and comparator/EXTI polarity.
    fn select_phase(&mut self, phase: Phase, rising: bool);
    /// Unmask the selected comparator source without clearing pending edges.
    fn enable_irq(&mut self);
    /// Mask the interrupt and clear its pending edge state.
    fn mask_irq(&mut self);
    /// Current comparator output, with the port's established polarity.
    fn level(&self) -> bool;
    /// Persistence-filter read; identical to an ordinary read unless a port
    /// explicitly supplies a separately validated implementation.
    fn filter_read(&self) -> bool {
        self.level()
    }
    /// Whether any edge is pending on the comparator's owned EXTI line(s).
    fn pending(&self) -> bool;
    /// Discard pending edges on owned lines without changing the IRQ mask.
    fn clear_pending(&mut self);
    /// Software-pend the selected source without changing the IRQ mask.
    fn repend(&mut self);
}

/// Debug pulse output — toggles a GPIO on commutation for RPM measurement.
pub trait PulseOutput {
    fn toggle(&mut self);
}

/// Motor phase output control. Callers supply the active [`PwmMode`]
/// (`PwmMode::Complementary` vs `PwmMode::Diode`) explicitly; invalid
/// commutation steps leave outputs unchanged.
pub trait PhaseOutput {
    fn com_step(&mut self, step: u8, mode: PwmMode);
    fn all_off(&mut self);
    fn full_brake(&mut self);
    fn all_pwm(&mut self, mode: PwmMode);
    fn proportional_brake(&mut self);
    /// Toggle pulse output on commutation step 1/4 (debug RPM measurement).
    /// Default no-op — override for boards with pulse output pin.
    fn pulse_toggle(&mut self, _step: u8) {}
}

/// Interval timer (commutation timing measurement)
pub trait IntervalTimer {
    fn count(&self) -> u32;
    fn set_count(&mut self, val: u32);
}

/// Commutation timer (one-shot for next commutation event)
pub trait ComTimer {
    fn set_and_enable(&mut self, timeout: u16);
    fn disable_interrupt(&mut self);
    fn enable_interrupt(&mut self);
}

/// Bundle of ISR-level motor peripherals for static dispatch.
///
/// Reduces generic parameter count from 5 to 1 in ISR function signatures.
/// Implementors provide the concrete MCU-specific types.
pub trait MotorHal: crate::platform::Platform {
    type Bridge<'a>: Bridge
    where
        Self: 'a;
    type Bemf: BemfSensor;
    type Interval: IntervalTimer;
    type Com: ComTimer;

    fn bridge(&mut self) -> Self::Bridge<'_>;
    fn bemf(&mut self) -> &mut Self::Bemf;
    fn interval(&mut self) -> &mut Self::Interval;
    fn com_timer(&mut self) -> &mut Self::Com;
}

/// Serial telemetry output (KISS protocol)
pub trait TelemetryUart {
    fn send(&mut self, data: &[u8]);
}

/// Bidir DShot output timer prescaler.
///
/// AM32 uses these PSC values for DShot response timing: DShot600 on normal
/// clocks = 0, DShot300 or high-clock DShot600 = 1, high-clock DShot300 = 3.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DshotOutputPrescaler(u16);

impl DshotOutputPrescaler {
    pub const DSHOT600: Self = Self(0);

    pub fn new(value: u16) -> Option<Self> {
        match value {
            0 | 1 | 3 => Some(Self(value)),
            _ => None,
        }
    }

    pub fn raw(self) -> u16 {
        self.0
    }
}

/// Input signal capture (DShot/Servo via DMA)
pub trait InputCapture {
    fn start_capture(&mut self);
    fn start_gcr_output(&mut self);
    fn input_pin_state(&self) -> bool;
    fn set_pull_up(&mut self);
    fn set_pull_down(&mut self);
    fn set_pull_none(&mut self);
    /// Set inverted input polarity (for boards with signal inversion).
    fn set_inverted(&mut self, _inverted: bool) {}
    /// Access the captured edge timestamps (DShot/servo frames).
    fn edges(&self) -> &[u32; 64];
    /// Access the GCR encode buffer (bidirectional DShot telemetry).
    fn gcr_buffer(&mut self) -> &mut [u32; 37];
    /// Whether the input is currently in output mode (bidir DShot TX).
    fn is_output(&self) -> bool;
    /// Set the timer prescaler for bidir DShot output.
    /// Called during protocol detection. DShot600=0, DShot300=1, DShot150=3.
    fn set_output_prescaler(&mut self, prescaler: DshotOutputPrescaler);
    /// Record the active capture timer prescaler selected by protocol detection
    /// so `start_capture()` restores it after bidirectional DShot TX.
    fn set_capture_prescaler(&mut self, _prescaler: u16) {}
}

/// ADC readings (voltage, current, temperature)
/// Voltage/current samples are 12-bit counts (0..=4095) at a nominal 3.3 V reference.
pub trait Adc {
    fn start_conversion(&mut self);
    fn raw_voltage(&self) -> u16;
    fn raw_current(&self) -> u16;
    fn raw_temperature(&self) -> u16;
    fn calc_temperature(&self, raw: u16) -> crate::units::DegreesCelsius;
    /// Trigger second ADC conversion (for boards with dual ADC). Default no-op.
    fn start_conversion_2(&mut self) {}
}

/// Flash storage for persistent settings
pub trait Flash {
    fn read(&self, address: u32, buf: &mut [u8]);
    fn write(&mut self, address: u32, data: &[u8]);
}

/// Serial input RX (CRSF protocol)
pub trait SerialInput {
    /// Read available bytes into buffer. Returns number of bytes read.
    fn read_available(&mut self, buf: &mut [u8]) -> usize;
}

/// Emergency hardware safety — all FETs off, no state needed.
/// Used when ISR state is missing or unrecoverable error occurs.
pub trait EmergencyOff {
    /// Force all motor FETs off via direct GPIO writes. Does not require HAL state.
    fn emergency_off();
}

/// System control (IRQ, watchdog, reset)
pub trait System {
    fn reset(&mut self) -> !;
    fn enable_irq(&mut self);
    fn disable_irq(&mut self);
    fn irqs_enabled(&self) -> bool;
    /// Start the independent watchdog with given prescaler and reload values.
    fn start_watchdog(&mut self, prescaler: u8, reload: u16);
    fn reload_watchdog(&mut self);
    fn delay_micros(&mut self, us: u32);
    fn delay_millis(&mut self, ms: u32);
}

#[cfg(test)]
mod tests {
    use super::DshotOutputPrescaler;

    #[test]
    fn dshot_output_prescaler_accepts_supported_values() {
        for value in [0, 1, 3] {
            assert_eq!(DshotOutputPrescaler::new(value).unwrap().raw(), value);
        }
    }

    #[test]
    fn dshot_output_prescaler_rejects_unsupported_values() {
        assert_eq!(DshotOutputPrescaler::new(2), None);
        assert_eq!(DshotOutputPrescaler::new(4), None);
        assert_eq!(DshotOutputPrescaler::new(u16::MAX), None);
    }
}
