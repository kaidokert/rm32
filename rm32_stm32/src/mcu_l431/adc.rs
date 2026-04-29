//! L431 ADC: CH8 current, CH11 voltage, CH17 temp. DMA1_CH1 circular.

use crate::adc_hal::AdcPeripheral;
use crate::pac::{ADC_COMMON, ADC1, DMA1, GPIOA, RCC};
use crate::regs::{InitError, wait_for};

crate::define_adc_boilerplate!(
    ops: L431AdcOps,
    type_name: L431Adc,
    cal1: 0x1FFF_75A8, cal2: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 130,
);

pub struct L431AdcOps;

impl AdcPeripheral for L431AdcOps {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.ahb2enr
            .modify(|_, w| w.adcen().set_bit().gpioaen().set_bit());
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
    }

    fn configure_pins(&self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa
            .moder
            .modify(|_, w| w.moder3().bits(0b11).moder6().bits(0b11));
    }

    fn configure_clock_source(&self) {
        let adc_common = unsafe { &*ADC_COMMON::ptr() };
        adc_common
            .ccr
            .modify(|_, w| unsafe { w.ckmode().bits(0b01) });
    }

    fn enable_temp_sensor(&self) {
        let adc_common = unsafe { &*ADC_COMMON::ptr() };
        adc_common.ccr.modify(|_, w| w.ch17sel().set_bit());
    }

    fn configure_dma(&self, buf_ptr: *const u16, buf_len: u16) {
        let adc = unsafe { &*ADC1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        dma.cselr
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0xF << 0)) });
        dma.ccr1.write(|w| unsafe { w.bits(0) });
        dma.cpar1
            .write(|w| unsafe { w.bits(adc.dr.as_ptr() as u32) });
        dma.cmar1.write(|w| unsafe { w.bits(buf_ptr as u32) });
        dma.cndtr1.write(|w| unsafe { w.bits(buf_len as u32) });
        dma.ccr1
            .write(|w| unsafe { w.bits((1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10)) });
        dma.ccr1.modify(|r, w| unsafe { w.bits(r.bits() | 1) });
    }

    fn configure_sampling(&self) {
        let adc = unsafe { &*ADC1::ptr() };
        adc.smpr1.modify(|_, w| unsafe { w.smp8().bits(0b100) });
        adc.smpr2.modify(|_, w| unsafe { w.smp11().bits(0b100) });
        adc.smpr2.modify(|_, w| unsafe { w.smp17().bits(0b100) });
    }

    fn configure_sequence(&self) {
        let adc = unsafe { &*ADC1::ptr() };
        adc.sqr1
            .write(|w| unsafe { w.l().bits(2).sq1().bits(8).sq2().bits(11).sq3().bits(17) });
    }

    fn enable_dma_mode(&self) {
        let adc = unsafe { &*ADC1::ptr() };
        adc.cfgr.write(|w| unsafe { w.bits((1 << 0) | (1 << 1)) });
    }

    fn power_up(&self) {
        let adc = unsafe { &*ADC1::ptr() };
        adc.cr.modify(|_, w| w.deeppwd().clear_bit());
        adc.cr.modify(|_, w| w.advregen().set_bit());
        cortex_m::asm::delay(80 * 20);
    }

    fn calibrate(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC1::ptr() };
        adc.cr.modify(|_, w| w.adcaldif().clear_bit());
        adc.cr.modify(|_, w| w.adcal().set_bit());
        wait_for(|| !adc.cr.read().adcal().bit_is_set(), 100_000, "ADC cal")?;
        cortex_m::asm::delay(80 * 20);
        Ok(())
    }

    fn enable(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC1::ptr() };
        adc.isr.write(|w| unsafe { w.bits(1 << 0) });
        adc.cr.modify(|_, w| w.aden().set_bit());
        wait_for(|| adc.isr.read().adrdy().bit_is_set(), 100_000, "ADC ready")?;
        Ok(())
    }

    fn start_conversion(&self) {
        let adc = unsafe { &*ADC1::ptr() };
        adc.cr.modify(|_, w| w.adstart().set_bit());
    }
}
