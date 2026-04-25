//! L431 ADC: CH8 current, CH11 voltage, CH17 temp. DMA1_CH1 circular.

use crate::pac::{ADC1, ADC_COMMON, DMA1, GPIOA};
use crate::adc_hal::{AdcOps, TempCalibration};
use crate::adc_generic::GenericAdc;
use crate::dma_buf::DmaBuf;
use crate::regs::{modify as modify_reg, InitError, wait_for};
use crate::periph_addr as addr;

static ADC_DMA_BUF: DmaBuf<u16, 3> = DmaBuf::new();

const TEMP_CAL: TempCalibration = TempCalibration {
    cal1_addr: 0x1FFF_75A8, cal2_addr: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 130,
};

pub struct L431AdcOps;

impl AdcOps for L431AdcOps {
    fn init(&self) -> Result<(), InitError> {
        let rcc_base = addr::RCC;
        unsafe {
            modify_reg(rcc_base + 0x4C, |v| v | (1 << 13) | (1 << 0));
            modify_reg(rcc_base + 0x48, |v| v | (1 << 0));
        }

        let adc = unsafe { &*ADC1::ptr() };
        let adc_common = unsafe { &*ADC_COMMON::ptr() };
        let dma = unsafe { &*DMA1::ptr() };
        let gpioa = unsafe { &*GPIOA::ptr() };

        adc_common.ccr.modify(|_, w| w.ckmode().bits(0b01));
        gpioa.moder.modify(|_, w| unsafe { w.moder3().bits(0b11).moder6().bits(0b11) });
        adc_common.ccr.modify(|_, w| w.ch17sel().set_bit());

        dma.cselr.modify(|r, w| unsafe { w.bits(r.bits() & !(0xF << 0)) });
        dma.ccr1.write(|w| unsafe { w.bits(0) });
        dma.cpar1.write(|w| unsafe { w.bits(adc.dr.as_ptr() as u32) });
        dma.cmar1.write(|w| unsafe { w.bits(ADC_DMA_BUF.as_ptr() as u32) });
        dma.cndtr1.write(|w| unsafe { w.bits(3) });
        dma.ccr1.write(|w| unsafe { w.bits((1<<5)|(1<<7)|(0b01<<8)|(0b01<<10)) });
        dma.ccr1.modify(|r, w| unsafe { w.bits(r.bits() | 1) });

        adc.cr.modify(|_, w| w.deeppwd().clear_bit());
        adc.cr.modify(|_, w| w.advregen().set_bit());
        cortex_m::asm::delay(80 * 20);

        adc.smpr1.modify(|_, w| unsafe { w.smp8().bits(0b100) });
        adc.smpr2.modify(|_, w| unsafe { w.smp11().bits(0b100) });
        adc.smpr2.modify(|_, w| unsafe { w.smp17().bits(0b100) });
        adc.sqr1.write(|w| unsafe { w.l().bits(2).sq1().bits(8).sq2().bits(11).sq3().bits(17) });
        adc.cfgr.write(|w| unsafe { w.bits((1<<0)|(1<<1)) });

        adc.cr.modify(|_, w| w.adcaldif().clear_bit());
        adc.cr.modify(|_, w| w.adcal().set_bit());
        wait_for(|| !adc.cr.read().adcal().bit_is_set(), 100_000, "ADC cal")?;
        cortex_m::asm::delay(80 * 20);

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

pub type L431Adc = GenericAdc<L431AdcOps>;

pub fn new_adc() -> L431Adc {
    GenericAdc::new(L431AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}

pub fn post_init() -> L431Adc {
    GenericAdc::post_init(L431AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}
