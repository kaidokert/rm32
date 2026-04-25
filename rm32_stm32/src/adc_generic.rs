//! Generic ADC driver — shared Adc trait impl across all MCUs.
//!
//! MCU-specific init and conversion trigger via AdcOps trait.
//! Buffer access via shared DmaBuf. Temperature calculation via shared utility.

use rm32::hal::Adc;
use rm32::units;
use crate::adc_hal::{AdcOps, TempCalibration};
use crate::dma_buf::DmaBuf;
use crate::regs::InitError;

/// Generic ADC reader. The DMA buffer is a static DmaBuf (circular DMA requirement).
pub struct GenericAdc<A: AdcOps> {
    ops: A,
    buf: &'static DmaBuf<u16, 3>,
    temp_cal: TempCalibration,
}

impl<A: AdcOps> GenericAdc<A> {
    pub fn new(ops: A, buf: &'static DmaBuf<u16, 3>, temp_cal: TempCalibration) -> Self {
        Self { ops, buf, temp_cal }
    }

    pub fn init(&self) -> Result<(), InitError> {
        self.ops.init()
    }

    /// Create a handle without re-initializing hardware.
    pub fn post_init(ops: A, buf: &'static DmaBuf<u16, 3>, temp_cal: TempCalibration) -> Self {
        Self { ops, buf, temp_cal }
    }
}

impl<A: AdcOps> Adc for GenericAdc<A> {
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
