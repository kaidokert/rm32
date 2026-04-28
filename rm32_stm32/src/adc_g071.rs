//! G071 ADC: CH4 current, CH6 voltage, CH12 temp. DMA1_CH2 circular + DMAMUX.

use crate::pac::{ADC, DMA1, GPIOA, RCC};
use crate::adc_hal::AdcPeripheral;
use crate::regs::{InitError, wait_for};

crate::define_adc_boilerplate!(
    ops: G071AdcOps,
    type_name: AdcReader,
    cal1: 0x1FFF_75A8, cal2: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 130,
);

pub struct G071AdcOps;

impl AdcPeripheral for G071AdcOps {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 20)) }); // ADCEN
        rcc.ahbenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });   // DMA1EN
    }

    fn configure_pins(&self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.moder().modify(|r, w| unsafe {
            w.bits(r.bits() | (0b11 << 8) | (0b11 << 12)) // PA4, PA6 analog
        });
    }

    fn configure_clock_source(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr2().write(|w| unsafe { w.bits(0b10 << 30) }); // CKMODE
    }

    fn enable_temp_sensor(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.ccr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 23)) }); // TSEN
    }

    fn configure_dma(&self, buf_ptr: *const u16, buf_len: u16) {
        let adc = unsafe { &*ADC::ptr() };
        let dma = unsafe { &*DMA1::ptr() };
        let dmamux = unsafe { &*crate::pac::DMAMUX::ptr() };

        dmamux.ccr(1).modify(|r, w| unsafe { w.bits((r.bits() & !0x3F) | 5) });
        let ch = dma.ch2();
        ch.cr().write(|w| w.en().clear_bit());
        ch.par().write(|w| unsafe { w.bits(adc.dr().as_ptr() as u32) });
        ch.mar().write(|w| unsafe { w.bits(buf_ptr as u32) });
        ch.ndtr().write(|w| unsafe { w.bits(buf_len as u32) });
        ch.cr().write(|w| unsafe {
            w.bits((1<<1)|(1<<5)|(1<<7)|(0b10<<8)|(0b01<<10)|(0b10<<12))
        });
        ch.cr().modify(|r, w| unsafe { w.bits(r.bits() | 1) });
    }

    fn configure_sampling(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.smpr().write(|w| unsafe { w.bits(0b011 | (0b111 << 4)) });
    }

    fn configure_sequence(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 21)) }); // CHSELRMOD
        // Sequencer: CH4(current), CH6(voltage), CH12(temp), 0xF(end)
        adc.chselr1().write(|w| unsafe { w.bits(4 | (6 << 4) | (12 << 8) | (0xF << 12)) });
    }

    fn enable_dma_mode(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr1().modify(|r, w| unsafe {
            w.bits((r.bits() & !0b11) | (0b01 << 0)) // DMAEN
        });
        adc.cfgr1().modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 3)) });
    }

    fn calibrate(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr().write(|w| unsafe { w.bits(1 << 31) }); // ADCAL
        wait_for(|| unsafe { (&*ADC::ptr()).cr().read().bits() & (1 << 31) == 0 }, 100_000, "ADC cal")?;
        cortex_m::asm::delay(64 * 20);
        Ok(())
    }

    fn enable(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC::ptr() };
        adc.isr().write(|w| unsafe { w.bits(1 << 0) }); // clear ADRDY
        adc.cr().write(|w| unsafe { w.bits(1 << 0) });  // ADEN
        wait_for(|| unsafe { (&*ADC::ptr()).isr().read().bits() & (1 << 0) != 0 }, 100_000, "ADC ready")
    }

    fn start_conversion(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 2)) }); // ADSTART
    }
}
