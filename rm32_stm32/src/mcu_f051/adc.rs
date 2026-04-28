//! F051 ADC: CH2 current, CH6 voltage, CH16 temp. DMA1_CH1 circular.

use crate::adc_hal::AdcPeripheral;
use crate::pac::{ADC, DMA1, GPIOA, RCC};
use crate::regs::{InitError, wait_for};

crate::define_adc_boilerplate!(
    ops: F051AdcOps,
    type_name: F051Adc,
    cal1: 0x1FFF_F7B8, cal2: 0x1FFF_F7C2,
    cal1_temp: 30, cal2_temp: 110,
);

pub struct F051AdcOps;

impl AdcPeripheral for F051AdcOps {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.adcen().set_bit());
        rcc.ahbenr.modify(|_, w| w.dmaen().set_bit());
    }

    fn configure_pins(&self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa
            .moder
            .modify(|_, w| w.moder2().analog().moder6().analog());
    }

    fn configure_clock_source(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr2.modify(|_, w| unsafe { w.bits(0b10 << 30) });
    }

    fn enable_temp_sensor(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.ccr.modify(|_, w| w.tsen().set_bit());
    }

    fn configure_dma(&self, buf_ptr: *const u16, buf_len: u16) {
        let dma = unsafe { &*DMA1::ptr() };
        let adc = unsafe { &*ADC::ptr() };

        dma.ch1.cr.write(|w| w.en().disabled());
        dma.ch1
            .par
            .write(|w| unsafe { w.bits(&adc.dr as *const _ as u32) });
        dma.ch1.mar.write(|w| unsafe { w.bits(buf_ptr as u32) });
        dma.ch1.ndtr.write(|w| unsafe { w.bits(buf_len as u32) });
        dma.ch1.cr.write(|w| {
            w.circ()
                .enabled()
                .minc()
                .enabled()
                .psize()
                .bits16()
                .msize()
                .bits16()
        });
        dma.ch1.cr.modify(|_, w| w.en().enabled());
    }

    fn configure_sampling(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.smpr.write(|w| unsafe { w.bits(0b110) });
    }

    fn configure_sequence(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.chselr
            .write(|w| unsafe { w.bits((1 << 2) | (1 << 6) | (1 << 16)) });
    }

    fn enable_dma_mode(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr1
            .modify(|_, w| w.dmaen().set_bit().dmacfg().set_bit());
        adc.cfgr1
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 3)) });
    }

    // power_up(): default no-op — F051 doesn't have deep power-down

    fn calibrate(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr.write(|w| w.adcal().start_calibration());
        wait_for(
            || !adc.cr.read().adcal().is_calibrating(),
            100_000,
            "ADC cal",
        )?;
        cortex_m::asm::delay(48 * 20);
        Ok(())
    }

    fn enable(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC::ptr() };
        adc.isr.write(|w| unsafe { w.bits(1 << 0) });
        adc.cr.write(|w| w.aden().set_bit());
        wait_for(|| adc.isr.read().adrdy().bit_is_set(), 100_000, "ADC ready")?;
        Ok(())
    }

    fn start_conversion(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr.modify(|_, w| w.adstart().start_conversion());
    }
}
