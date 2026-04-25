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

/// PWM output interface for 3-phase motor control.
/// Goes beyond embedded-hal's single-channel SetDutyCycle.
pub trait PwmOutput {
    fn set_duty_all(&mut self, duty: u16);
    fn set_auto_reload(&mut self, arr: u16);
    fn set_prescaler(&mut self, psc: u16);
    fn set_compare1(&mut self, val: u16);
    fn set_compare2(&mut self, val: u16);
    fn set_compare3(&mut self, val: u16);
    fn generate_update_event(&mut self);
    /// Override dead-time in TIM1 BDTR register (OR'd with existing DTG value).
    fn set_dead_time_override(&mut self, dtg: u16);
}

/// Comparator (BEMF sensing) interface
pub trait Comparator {
    fn output_level(&self) -> bool;
    /// Set the commutation step and rising/falling edge for BEMF sensing.
    fn set_step(&mut self, step: u8, rising: bool);
    fn change_input(&mut self);
    fn enable_interrupts(&mut self);
    fn mask_interrupts(&mut self);
}

/// Motor phase output control (6-step commutation)
pub trait PhaseOutput {
    fn com_step(&mut self, step: u8);
    fn all_off(&mut self);
    fn full_brake(&mut self);
    fn all_pwm(&mut self);
    fn proportional_brake(&mut self);
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

/// Serial telemetry output (KISS protocol)
pub trait TelemetryUart {
    fn send_dma(&mut self, data: &[u8]);
}

/// Input signal capture (DShot/Servo via DMA)
pub trait InputCapture {
    fn receive_dshot_dma(&mut self);
    fn send_dshot_dma(&mut self);
    fn input_pin_state(&self) -> bool;
    fn set_pull_up(&mut self);
    fn set_pull_down(&mut self);
    fn set_pull_none(&mut self);
}

/// ADC readings (voltage, current, temperature)
pub trait Adc {
    fn start_conversion(&mut self);
    fn raw_voltage(&self) -> u16;
    fn raw_current(&self) -> u16;
    fn raw_temperature(&self) -> u16;
    fn calc_temperature(&self, raw: u16) -> crate::units::DegreesCelsius;
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

/// System control (IRQ, watchdog, reset)
pub trait System {
    fn reset(&mut self) -> !;
    fn enable_irq(&mut self);
    fn disable_irq(&mut self);
    /// Start the independent watchdog with given prescaler and reload values.
    fn start_watchdog(&mut self, prescaler: u8, reload: u16);
    fn reload_watchdog(&mut self);
    fn delay_micros(&mut self, us: u32);
    fn delay_millis(&mut self, ms: u32);
}
