//! G071 ADC: CH4 current, CH6 voltage, CH12 temp. DMA1_CH2 circular + DMAMUX.

use crate::pac::{ADC, DMA1, GPIOA, RCC};
use crate::adc_hal::{AdcOps, TempCalibration};
use crate::adc_generic::GenericAdc;
use crate::dma_buf::DmaBuf;
use crate::regs::{InitError, wait_for};

static ADC_DMA_BUF: DmaBuf<u16, 3> = DmaBuf::new();

const TEMP_CAL: TempCalibration = TempCalibration {
    cal1_addr: 0x1FFF_75A8, cal2_addr: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 130,
};

pub struct G071AdcOps;

impl AdcOps for G071AdcOps {
    fn init(&self) -> Result<(), InitError> {
        let rcc = unsafe { &*RCC::ptr() };
        let gpioa = unsafe { &*GPIOA::ptr() };
        let adc = unsafe { &*ADC::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 20)) });
        rcc.ahbenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });

        gpioa.moder().modify(|r, w| unsafe {
            w.bits(r.bits() | (0b11 << 8) | (0b11 << 12))
        });

        // DMAMUX: Channel 2 → ADC1 (request 5)
        let dmamux = unsafe { &*crate::pac::DMAMUX::ptr() };
        dmamux.ccr(1).modify(|r, w| unsafe { w.bits((r.bits() & !0x3F) | 5) });

        let ch = dma.ch2();
        ch.cr().write(|w| w.en().clear_bit());
        ch.par().write(|w| unsafe { w.bits(adc.dr().as_ptr() as u32) });
        ch.mar().write(|w| unsafe { w.bits(ADC_DMA_BUF.as_ptr() as u32) });
        ch.ndtr().write(|w| unsafe { w.bits(3) });
        ch.cr().write(|w| unsafe {
            w.bits((1<<1)|(1<<5)|(1<<7)|(0b10<<8)|(0b01<<10)|(0b10<<12))
        });
        ch.cr().modify(|r, w| unsafe { w.bits(r.bits() | 1) });

        adc.cfgr2().write(|w| unsafe { w.bits(0b10 << 30) });
        adc.ccr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 23)) }); // TSEN

        adc.smpr().write(|w| unsafe { w.bits(0b011 | (0b111 << 4)) });
        adc.cfgr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 21)) });
        // Sequencer: CH4(current), CH6(voltage), CH12(temp), 0xF(end)
        adc.chselr1().write(|w| unsafe { w.bits(4 | (6 << 4) | (12 << 8) | (0xF << 12)) });
        adc.cfgr1().modify(|r, w| unsafe {
            w.bits((r.bits() & !0b11) | (0b01 << 0))
        });
        adc.cfgr1().modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 3)) });

        adc.cr().write(|w| unsafe { w.bits(1 << 31) });
        wait_for(|| adc.cr().read().bits() & (1 << 31) == 0, 100_000, "ADC cal")?;
        cortex_m::asm::delay(64 * 20);

        adc.isr().write(|w| unsafe { w.bits(1 << 0) });
        adc.cr().write(|w| unsafe { w.bits(1 << 0) });
        wait_for(|| adc.isr().read().bits() & (1 << 0) != 0, 100_000, "ADC ready")?;
        Ok(())
    }

    fn start_conversion(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 2)) });
    }
}

pub type AdcReader = GenericAdc<G071AdcOps>;

pub fn new_adc() -> AdcReader {
    GenericAdc::new(G071AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}

pub fn post_init() -> AdcReader {
    GenericAdc::post_init(G071AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}
