//! Stub implementations for peripherals not yet ported to a given MCU.

use rm32::hal::{Adc, TelemetryUart};

pub struct StubAdc;

impl Adc for StubAdc {
    fn start_conversion(&mut self) {}
    fn raw_voltage(&self) -> u16 { 0 }
    fn raw_current(&self) -> u16 { 0 }
    fn raw_temperature(&self) -> u16 { 0 }
    fn calc_temperature(&self, _raw: u16) -> rm32::units::DegreesCelsius { rm32::units::DegreesCelsius(25) }
}

pub struct StubTelem;

impl TelemetryUart for StubTelem {
    fn send_dma(&mut self, _data: &[u8]) {}
}
