//! F051 ADC: CH2 current, CH6 voltage, CH16 temp. DMA1_CH1 circular.

use crate::pac::{ADC, DMA1, GPIOA};
use crate::adc_hal::{AdcOps, TempCalibration};
use crate::adc_generic::GenericAdc;
use crate::dma_buf::DmaBuf;
use crate::regs::{InitError, wait_for};
use crate::periph_addr as addr;

static ADC_DMA_BUF: DmaBuf<u16, 3> = DmaBuf::new();

const TEMP_CAL: TempCalibration = TempCalibration {
    cal1_addr: 0x1FFF_F7B8, cal2_addr: 0x1FFF_F7C2,
    cal1_temp: 30, cal2_temp: 110,
};

pub struct F051AdcOps;

impl AdcOps for F051AdcOps {
    fn init(&self) -> Result<(), InitError> {
        let rcc_base = addr::rcc();
        unsafe {
            let apb2enr = (rcc_base + 0x18) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 9));
            let ahbenr = (rcc_base + 0x14) as *mut u32;
            ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0));
        }

        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.moder.modify(|_, w| w.moder2().analog().moder6().analog());

        let dma = unsafe { &*DMA1::ptr() };
        let adc = unsafe { &*ADC::ptr() };

        dma.ch1.cr.write(|w| w.en().disabled());
        dma.ch1.par.write(|w| unsafe { w.bits(&adc.dr as *const _ as u32) });
        dma.ch1.mar.write(|w| unsafe { w.bits(ADC_DMA_BUF.as_ptr() as u32) });
        dma.ch1.ndtr.write(|w| unsafe { w.bits(3) });
        dma.ch1.cr.write(|w| w.circ().enabled().minc().enabled().psize().bits16().msize().bits16());
        dma.ch1.cr.modify(|_, w| w.en().enabled());

        adc.cfgr2.modify(|_, w| unsafe { w.bits(0b10 << 30) });
        adc.ccr.modify(|_, w| w.tsen().set_bit());
        adc.smpr.write(|w| unsafe { w.bits(0b110) });
        adc.chselr.write(|w| unsafe { w.bits((1 << 2) | (1 << 6) | (1 << 16)) });
        adc.cfgr1.modify(|_, w| w.dmaen().set_bit().dmacfg().set_bit());
        adc.cfgr1.modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 3)) });

        adc.cr.write(|w| w.adcal().start_calibration());
        wait_for(|| !adc.cr.read().adcal().is_calibrating(), 100_000, "ADC cal")?;
        cortex_m::asm::delay(48 * 20);

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

pub type F051Adc = GenericAdc<F051AdcOps>;

pub fn new_adc() -> F051Adc {
    GenericAdc::new(F051AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}

pub fn post_init() -> F051Adc {
    GenericAdc::post_init(F051AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}
