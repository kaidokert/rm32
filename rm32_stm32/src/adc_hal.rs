//! Abstraction over MCU-specific ADC register access.

use crate::regs::InitError;

/// ADC peripheral operations (MCU-specific).
pub trait AdcOps {
    /// Full ADC initialization: clocks, GPIO, DMA, calibration, enable.
    fn init(&self) -> Result<(), InitError>;
    /// Trigger a new conversion sequence.
    fn start_conversion(&self);
}

/// Temperature sensor calibration info (MCU-specific).
pub struct TempCalibration {
    pub cal1_addr: u32,
    pub cal2_addr: u32,
    pub cal1_temp: i32,
    pub cal2_temp: i32,
}
