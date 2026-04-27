//! G431 ADC driver.
//!
//! Single mode (PROTONDRIVE): ADC1 → temp, voltage, current via DMA1_CH2.
//! Dual mode (SEQURE): ADC1 → temp, NTC via DMA1_CH2; ADC2 → voltage, current via DMA1_CH4.

use crate::adc_hal::{AdcOps, TempCalibration};
use crate::adc_generic::GenericAdc;
use crate::dma_buf::DmaBuf;
use crate::regs::{InitError, wait_for};
use stm32g4::stm32g431 as pac;

static ADC_DMA_BUF: DmaBuf<u16, 3> = DmaBuf::new();
static ADC1_DMA_BUF: DmaBuf<u16, 2> = DmaBuf::new();
static ADC2_DMA_BUF: DmaBuf<u16, 2> = DmaBuf::new();

const TEMP_CAL: TempCalibration = TempCalibration {
    cal1_addr: 0x1FFF_75A8, cal2_addr: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 110,
};

pub struct G431AdcOps;

impl AdcOps for G431AdcOps {
    fn init(&self) -> Result<(), InitError> {
        let rcc = unsafe { &*pac::RCC::PTR };
        let gpioa = unsafe { &*pac::GPIOA::PTR };
        let adc1 = unsafe { &*pac::ADC1::PTR };
        let adc_common = unsafe { &*pac::ADC12_COMMON::PTR };
        let dma = unsafe { &*pac::DMA1::PTR };
        let dmamux = unsafe { &*pac::DMAMUX::PTR };

        unsafe {
            // Enable clocks
            rcc.ahb2enr().modify(|_, w| w.adc12en().set_bit().gpioaen().set_bit());
            rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());

            // PA4, PA5 as analog
            gpioa.moder().modify(|_, w| w.moder4().bits(0b11).moder5().bits(0b11));

            // ADC common: CKMODE = PCLK/4, enable temp sensor
            adc_common.ccr().write(|w| w.ckmode().bits(0b11).vsensesel().set_bit());

            // DMA1 Channel 2 → ADC1 (DMAMUX request 5)
            dmamux.ccr(1).write(|w| w.dmareq_id().bits(5));
            let ch2 = dma.ch2();
            ch2.cr().write(|w| w.bits(0)); // disable
            ch2.par().write(|w| w.bits(adc1.dr().as_ptr() as u32));
            ch2.mar().write(|w| w.bits(ADC_DMA_BUF.as_ptr() as u32));
            ch2.ndtr().write(|w| w.bits(3));
            // Circular, MINC, 16-bit psize, 16-bit msize
            ch2.cr().write(|w| w.bits((1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10)));
            ch2.cr().modify(|r, w| w.bits(r.bits() | 1)); // enable

            // ADC1: exit deep power-down, enable regulator
            adc1.cr().modify(|_, w| w.deeppwd().clear_bit());
            adc1.cr().modify(|_, w| w.advregen().set_bit());
            cortex_m::asm::delay(170 * 20);

            // Sampling times: 47.5 cycles
            adc1.smpr1().write(|w| w.bits(0b100 << 15 | 0b100 << 12)); // CH5, CH4
            adc1.smpr2().write(|w| w.bits(0b100 << 9)); // CH13

            // Sequence: 3 conversions — TEMPSENSOR(16), voltage(13), current(5)
            adc1.sqr1().write(|w| w.bits((2 << 0) | (16 << 6) | (13 << 12) | (5 << 18)));

            // CFGR: DMAEN + DMACFG (circular) + CONT
            adc1.cfgr().write(|w| w.dmaen().set_bit().dmacfg().set_bit().cont().set_bit());

            // Calibration
            adc1.cr().modify(|_, w| w.adcaldif().clear_bit());
            adc1.cr().modify(|_, w| w.adcal().set_bit());
            wait_for(|| !adc1.cr().read().adcal().bit(), 100_000, "ADC cal")?;
            cortex_m::asm::delay(170 * 20);

            // Enable ADC
            adc1.isr().write(|w| w.adrdy().clear_bit_by_one());
            adc1.cr().modify(|_, w| w.aden().set_bit());
            wait_for(|| adc1.isr().read().adrdy().bit(), 100_000, "ADC ready")?;
        }
        Ok(())
    }

    fn start_conversion(&self) {
        let adc1 = unsafe { &*pac::ADC1::PTR };
        unsafe { adc1.cr().modify(|_, w| w.adstart().set_bit()); }
    }
}

pub type G431Adc = GenericAdc<G431AdcOps>;

pub fn new_adc() -> G431Adc {
    GenericAdc::new(G431AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}

pub fn post_init() -> G431Adc {
    GenericAdc::post_init(G431AdcOps, &ADC_DMA_BUF, TEMP_CAL)
}

// ============================================================
// Dual ADC mode (SEQURE_G431)
// ============================================================

/// Dual ADC: implements Adc trait directly, owns two DMA buffers.
pub struct G431DualAdc;

impl G431DualAdc {
    pub fn init() -> Result<Self, InitError> {
        let rcc = unsafe { &*pac::RCC::PTR };
        let gpioa = unsafe { &*pac::GPIOA::PTR };
        let gpiob = unsafe { &*pac::GPIOB::PTR };
        let adc1 = unsafe { &*pac::ADC1::PTR };
        let adc2 = unsafe { &*pac::ADC2::PTR };
        let adc_common = unsafe { &*pac::ADC12_COMMON::PTR };
        let dma = unsafe { &*pac::DMA1::PTR };
        let dmamux = unsafe { &*pac::DMAMUX::PTR };

        unsafe {
            // Enable clocks
            rcc.ahb2enr().modify(|_, w| w.adc12en().set_bit().gpioaen().set_bit().gpioben().set_bit());
            rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());

            // PA6, PA7 as analog (ADC2), PB1 as analog (NTC)
            gpioa.moder().modify(|_, w| w.moder6().bits(0b11).moder7().bits(0b11));
            gpiob.moder().modify(|_, w| w.moder1().bits(0b11));

            // ADC common: CKMODE = PCLK/4, enable temp sensor
            adc_common.ccr().write(|w| w.ckmode().bits(0b11).vsensesel().set_bit());

            // DMA1 CH2 → ADC1
            dmamux.ccr(1).write(|w| w.dmareq_id().bits(5));
            let ch2 = dma.ch2();
            ch2.cr().write(|w| w.bits(0));
            ch2.par().write(|w| w.bits(adc1.dr().as_ptr() as u32));
            ch2.mar().write(|w| w.bits(ADC1_DMA_BUF.as_ptr() as u32));
            ch2.ndtr().write(|w| w.bits(2));
            ch2.cr().write(|w| w.bits((1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10)));
            ch2.cr().modify(|r, w| w.bits(r.bits() | 1));

            // DMA1 CH4 → ADC2
            dmamux.ccr(3).write(|w| w.dmareq_id().bits(36));
            let ch4 = dma.ch4();
            ch4.cr().write(|w| w.bits(0));
            ch4.par().write(|w| w.bits(adc2.dr().as_ptr() as u32));
            ch4.mar().write(|w| w.bits(ADC2_DMA_BUF.as_ptr() as u32));
            ch4.ndtr().write(|w| w.bits(2));
            ch4.cr().write(|w| w.bits((1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10)));
            ch4.cr().modify(|r, w| w.bits(r.bits() | 1));

            // ADC1: TEMPSENSOR(16) + NTC(12)
            adc1.cr().modify(|_, w| w.deeppwd().clear_bit());
            adc1.cr().modify(|_, w| w.advregen().set_bit());
            cortex_m::asm::delay(170 * 20);
            adc1.smpr2().write(|w| w.bits(0b100 << 6)); // SMP12 = 47.5 cycles
            adc1.sqr1().write(|w| w.bits((1 << 0) | (16 << 6) | (12 << 12)));
            adc1.cfgr().write(|w| w.dmaen().set_bit().dmacfg().set_bit());
            adc1.cr().modify(|_, w| w.adcaldif().clear_bit());
            adc1.cr().modify(|_, w| w.adcal().set_bit());
            wait_for(|| !adc1.cr().read().adcal().bit(), 100_000, "ADC1 cal")?;
            cortex_m::asm::delay(170 * 20);
            adc1.isr().write(|w| w.adrdy().clear_bit_by_one());
            adc1.cr().modify(|_, w| w.aden().set_bit());
            wait_for(|| adc1.isr().read().adrdy().bit(), 100_000, "ADC1 ready")?;

            // ADC2: Voltage(CH3) + Current(CH4)
            adc2.cr().modify(|_, w| w.deeppwd().clear_bit());
            adc2.cr().modify(|_, w| w.advregen().set_bit());
            cortex_m::asm::delay(170 * 20);
            adc2.smpr1().write(|w| w.bits((0b010 << 9) | (0b100 << 12))); // CH3=2.5, CH4=47.5
            adc2.sqr1().write(|w| w.bits((1 << 0) | (3 << 6) | (4 << 12)));
            adc2.cfgr().write(|w| w.dmaen().set_bit().dmacfg().set_bit());
            adc2.cr().modify(|_, w| w.adcaldif().clear_bit());
            adc2.cr().modify(|_, w| w.adcal().set_bit());
            wait_for(|| !adc2.cr().read().adcal().bit(), 100_000, "ADC2 cal")?;
            cortex_m::asm::delay(170 * 20);
            adc2.isr().write(|w| w.adrdy().clear_bit_by_one());
            adc2.cr().modify(|_, w| w.aden().set_bit());
            wait_for(|| adc2.isr().read().adrdy().bit(), 100_000, "ADC2 ready")?;
        }
        Ok(Self)
    }

    pub fn post_init() -> Self { Self }
}

impl rm32::hal::Adc for G431DualAdc {
    fn start_conversion(&mut self) {
        let adc1 = unsafe { &*pac::ADC1::PTR };
        unsafe { adc1.cr().modify(|_, w| w.adstart().set_bit()); }
    }

    fn start_conversion_2(&mut self) {
        let adc2 = unsafe { &*pac::ADC2::PTR };
        unsafe { adc2.cr().modify(|_, w| w.adstart().set_bit()); }
    }

    fn raw_temperature(&self) -> u16 { ADC1_DMA_BUF.read()[0] }
    fn raw_voltage(&self) -> u16 { ADC2_DMA_BUF.read()[0] }
    fn raw_current(&self) -> u16 { ADC2_DMA_BUF.read()[1] }

    fn calc_temperature(&self, raw: u16) -> rm32::units::DegreesCelsius {
        rm32::units::calc_temperature_from_cal(
            raw, TEMP_CAL.cal1_addr, TEMP_CAL.cal2_addr,
            TEMP_CAL.cal1_temp, TEMP_CAL.cal2_temp,
        )
    }
}
