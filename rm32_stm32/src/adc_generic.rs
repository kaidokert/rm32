//! Generic ADC driver — shared init sequence and Adc trait impl.
//!
//! MCU-specific register operations via `AdcPeripheral` trait.
//! The init sequence is identical across all MCUs — only register writes differ.

use rm32::hal::Adc;
use rm32::units;
use crate::adc_hal::{AdcPeripheral, TempCalibration};
use crate::dma_buf::DmaBuf;
use crate::regs::InitError;

/// Generic ADC reader parameterized over buffer size.
/// N=3 for single-ADC (temp, voltage, current), N=2 for dual-ADC per-unit.
pub struct GenericAdc<A: AdcPeripheral, const N: usize = 3> {
    ops: A,
    buf: &'static DmaBuf<u16, N>,
    temp_cal: TempCalibration,
}

impl<A: AdcPeripheral, const N: usize> GenericAdc<A, N> {
    pub fn new(ops: A, buf: &'static DmaBuf<u16, N>, temp_cal: TempCalibration) -> Self {
        Self { ops, buf, temp_cal }
    }

    /// Shared init sequence — same logical steps for all MCUs.
    /// Each step delegates to the MCU-specific `AdcPeripheral` impl.
    pub fn init(&self) -> Result<(), InitError> {
        self.ops.enable_clocks();
        self.ops.configure_pins();
        self.ops.configure_clock_source();
        self.ops.enable_temp_sensor();
        self.ops.configure_dma(self.buf.as_ptr(), N as u16);
        self.ops.power_up();
        self.ops.configure_sampling();
        self.ops.configure_sequence();
        self.ops.enable_dma_mode();
        self.ops.calibrate()?;
        self.ops.enable()?;
        Ok(())
    }

    /// Create a handle without re-initializing hardware.
    pub fn post_init(ops: A, buf: &'static DmaBuf<u16, N>, temp_cal: TempCalibration) -> Self {
        Self { ops, buf, temp_cal }
    }
}

impl<A: AdcPeripheral> Adc for GenericAdc<A, 3> {
    fn start_conversion(&mut self) {
        self.ops.start_conversion();
    }

    fn raw_current(&self) -> u16 { self.buf.read()[0] }
    fn raw_voltage(&self) -> u16 { self.buf.read()[1] }
    fn raw_temperature(&self) -> u16 { self.buf.read()[2] }

    fn calc_temperature(&self, raw: u16) -> units::DegreesCelsius {
        units::calc_temperature_from_cal(
            raw,
            self.temp_cal.cal1_addr,
            self.temp_cal.cal2_addr,
            self.temp_cal.cal1_temp,
            self.temp_cal.cal2_temp,
        )
    }
}
