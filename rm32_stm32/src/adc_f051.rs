//! ADC + DMA for voltage, current, and temperature measurement on STM32F051.
//!
//! ADC1 with 3-channel scan: current (PA2/CH2), voltage (PA6/CH6),
//! temperature (internal sensor CH16). DMA1 Channel 1 in circular mode.
//!
//! F051 ADC differences from G071:
//!   - No DMAMUX (fixed DMA channel assignment: ADC → DMA1_CH1)
//!   - CHSELR is a bitmask, not configurable sequence
//!   - Single common sampling time for all channels
//!   - Calibration values at different ROM addresses

use rm32::hal::Adc;

/// Static DMA buffer for ADC readings (3 x 16-bit).
static mut ADC_DMA_BUF: [u16; 3] = [0; 3];

use crate::periph_addr as addr;
use crate::pac::{ADC, DMA1, GPIOA};
use crate::regs::modify as modify_reg;

const RCC_BASE: u32 = addr::RCC;

pub struct F051Adc {
    _private: (),
}

impl F051Adc {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        unsafe {
            // Enable clocks: ADC (APB2ENR bit 9), DMA1 (AHBENR bit 0)
            let apb2enr = (RCC_BASE + 0x18) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 9)); // ADCEN
            let ahbenr = (RCC_BASE + 0x14) as *mut u32;
            ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0)); // DMA1EN

            // PA2, PA6 as analog via PAC
            let gpioa = &*GPIOA::ptr();
            gpioa.moder.modify(|_, w| {
                w.moder2().analog()
                 .moder6().analog()
            });

            // DMA1 Channel 1: periph→memory, 16-bit, memory increment, circular
            let dma = &*DMA1::ptr();
            let adc_ref = &*ADC::ptr();
            dma.ch1.cr.write(|w| w.en().disabled()); // disable
            dma.ch1.par.write(|w| w.bits(&adc_ref.dr as *const _ as u32));
            dma.ch1.mar.write(|w| w.bits(ADC_DMA_BUF.as_ptr() as u32));
            dma.ch1.ndtr.write(|w| w.bits(3));
            dma.ch1.cr.write(|w| {
                w.circ().enabled()
                 .minc().enabled()
                 .psize().bits16()
                 .msize().bits16()
            });
            dma.ch1.cr.modify(|_, w| w.en().enabled());

            let adc = &*ADC::ptr();

            // ADC clock: PCLK/4 — CFGR2 is at offset 0x10 from ADC base
            // cfgr2 is not directly in the PAC layout we can modify easily by name,
            // keep raw access for this CCR2/CFGR2 register
            adc.cfgr2.modify(|_, w| w.bits(0b10 << 30)); // CKMODE = PCLK/4

            // Enable temperature sensor via CCR register
            adc.ccr.modify(|_, w| w.tsen().set_bit());

            // Sampling time: 71.5 ADC clock cycles
            adc.smpr.write(|w| w.bits(0b110)); // SMP = 71.5 cycles

            // Channel selection: CH2 (current) | CH6 (voltage) | CH16 (temp)
            // F0 CHSELR is a bitmask — channels are scanned in ascending order
            adc.chselr.write(|w| w.bits((1 << 2) | (1 << 6) | (1 << 16)));

            // Enable DMA on ADC: DMAEN=1, DMACFG=1 (circular)
            adc.cfgr1.modify(|_, w| w.dmaen().set_bit().dmacfg().set_bit());

            // Resolution 12-bit (RES=00), right-aligned, scan direction forward
            adc.cfgr1.modify(|r, w| w.bits(r.bits() & !(0b11 << 3))); // RES=00 (12-bit)

            // Calibrate
            adc.cr.write(|w| w.adcal().start_calibration());
            while adc.cr.read().adcal().is_calibrating() {}

            // Stabilization delay
            cortex_m::asm::delay(48 * 20);

            // Enable ADC
            adc.isr.write(|w| w.bits(1 << 0)); // clear ADRDY
            adc.cr.write(|w| w.aden().set_bit());
            while adc.isr.read().adrdy().bit_is_clear() {} // wait ADRDY
        }
        Self { _private: () }
    }
}

impl Adc for F051Adc {
    fn start_conversion(&mut self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr.modify(|_, w| w.adstart().start_conversion());
    }

    fn raw_current(&self) -> u16 {
        // CH2 is lowest channel → first in DMA buffer (ascending scan)
        unsafe { ADC_DMA_BUF[0] }
    }

    fn raw_voltage(&self) -> u16 {
        // CH6 is second → index 1
        unsafe { ADC_DMA_BUF[1] }
    }

    fn raw_temperature(&self) -> u16 {
        // CH16 is third → index 2
        unsafe { ADC_DMA_BUF[2] }
    }

    fn calc_temperature(&self, raw: u16) -> i16 {
        // STM32F0 temp sensor calibration values
        // TS_CAL1 at 0x1FFFF7B8 (30C), TS_CAL2 at 0x1FFFF7C2 (110C)
        let ts_cal1 = unsafe { *(0x1FFF_F7B8 as *const u16) } as i32;
        let ts_cal2 = unsafe { *(0x1FFF_F7C2 as *const u16) } as i32;
        if ts_cal2 == ts_cal1 { return 25; }
        let temp = (110 - 30) * (raw as i32 - ts_cal1) / (ts_cal2 - ts_cal1) + 30;
        temp as i16
    }
}
