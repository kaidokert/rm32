//! ADC + DMA for voltage, current, and temperature on STM32L431.
//!
//! ADC1 with 3-rank scan: current (PA3/CH8), voltage (PA6/CH11),
//! temperature (internal CH17). DMA1 Channel 1 (request 0) circular.

use rm32::hal::Adc;

static mut ADC_DMA_BUF: [u16; 3] = [0; 3];

use crate::periph_addr as addr;
use crate::pac::{ADC1, ADC_COMMON, DMA1, GPIOA};
use crate::regs::modify as modify_reg;

const RCC_BASE: u32 = addr::RCC;

pub struct L431Adc { _private: () }

impl L431Adc {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        let adc = unsafe { &*ADC1::ptr() };
        let adc_common = unsafe { &*ADC_COMMON::ptr() };
        let dma = unsafe { &*DMA1::ptr() };
        let gpioa = unsafe { &*GPIOA::ptr() };

        unsafe {
            // Enable clocks: ADC (AHB2ENR bit 13), DMA1 (AHB1ENR bit 0), GPIOA (AHB2ENR bit 0)
            modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 13) | (1 << 0)); // AHB2ENR
            modify_reg(RCC_BASE + 0x48, |v| v | (1 << 0)); // AHB1ENR: DMA1EN

            // ADC clock: use HCLK/1 synchronous clock via ADC_CCR CKMODE=0b01
            adc_common.ccr.modify(|_, w| unsafe { w.ckmode().bits(0b01) });

            // PA3, PA6 as analog (MODER bits [7:6]=0b11 for PA3, [13:12]=0b11 for PA6)
            gpioa.moder.modify(|_, w| w.moder3().bits(0b11).moder6().bits(0b11));

            // Enable temperature sensor (TSEN bit 23 in CCR)
            adc_common.ccr.modify(|_, w| w.ch17sel().set_bit());

            // DMA CSELR: Channel 1 request = 0 (ADC1), bits [3:0]
            dma.cselr.modify(|r, w| w.bits(r.bits() & !(0xF << 0)));

            // DMA1 CH1: periph→memory, 16-bit, memory increment, circular
            dma.ccr1.write(|w| w.bits(0));
            dma.cpar1.write(|w| w.bits(adc.dr.as_ptr() as u32));
            dma.cmar1.write(|w| w.bits(ADC_DMA_BUF.as_ptr() as u32));
            dma.cndtr1.write(|w| w.bits(3));
            dma.ccr1.write(|w| w.bits(
                (1 << 5)       // CIRC
                | (1 << 7)     // MINC
                | (0b01 << 8)  // PSIZE = 16-bit
                | (0b01 << 10) // MSIZE = 16-bit
            ));
            dma.ccr1.modify(|r, w| w.bits(r.bits() | 1)); // EN

            // Disable deep power down, enable internal voltage regulator
            adc.cr.modify(|_, w| w.deeppwd().clear_bit()); // DEEPPWD = 0
            adc.cr.modify(|_, w| w.advregen().set_bit());   // ADVREGEN = 1
            // Wait for regulator startup (~20us at 80MHz)
            cortex_m::asm::delay(80 * 20);

            // Sampling time: 47.5 cycles for CH8, CH11, CH17
            // CH8: SMPR1 bits [26:24] = 0b100 (47.5 cycles)
            adc.smpr1.modify(|_, w| unsafe { w.smp8().bits(0b100) });
            // CH11: SMPR2 bits [5:3] = 0b100
            adc.smpr2.modify(|_, w| unsafe { w.smp11().bits(0b100) });
            // CH17: SMPR2 bits [23:21] = 0b100
            adc.smpr2.modify(|_, w| unsafe { w.smp17().bits(0b100) });

            // Sequence: 3 conversions
            // SQR1: L[3:0] = 2 (3 conversions), SQ1=CH8, SQ2=CH11, SQ3=CH17
            adc.sqr1.write(|w| unsafe { w.bits((2 << 0) | (8 << 6) | (11 << 12) | (17 << 18)) });

            // CFGR: DMA circular mode, resolution 12-bit
            adc.cfgr.write(|w| unsafe { w.bits(
                (1 << 0)   // DMAEN
                | (1 << 1) // DMACFG = circular
            )});

            // Calibrate (single-ended)
            adc.cr.modify(|_, w| w.adcaldif().clear_bit()); // ADCALDIF = 0 (single-ended)
            adc.cr.modify(|_, w| w.adcal().set_bit());       // ADCAL
            while adc.cr.read().adcal().bit_is_set() {}

            cortex_m::asm::delay(80 * 20);

            // Enable ADC
            adc.isr.write(|w| unsafe { w.bits(1 << 0) }); // clear ADRDY
            adc.cr.modify(|_, w| w.aden().set_bit()); // ADEN
            while adc.isr.read().adrdy().bit_is_clear() {}
        }
        Self { _private: () }
    }
}

impl Adc for L431Adc {
    fn start_conversion(&mut self) {
        let adc = unsafe { &*ADC1::ptr() };
        adc.cr.modify(|_, w| w.adstart().set_bit());
    }

    fn raw_current(&self) -> u16 { unsafe { ADC_DMA_BUF[0] } }
    fn raw_voltage(&self) -> u16 { unsafe { ADC_DMA_BUF[1] } }
    fn raw_temperature(&self) -> u16 { unsafe { ADC_DMA_BUF[2] } }

    fn calc_temperature(&self, raw: u16) -> i16 {
        // L431 temp calibration: TS_CAL1 at 0x1FFF75A8 (30C), TS_CAL2 at 0x1FFF75CA (130C)
        let ts_cal1 = unsafe { *(0x1FFF_75A8 as *const u16) } as i32;
        let ts_cal2 = unsafe { *(0x1FFF_75CA as *const u16) } as i32;
        if ts_cal2 == ts_cal1 { return 25; }
        let temp = (130 - 30) * (raw as i32 - ts_cal1) / (ts_cal2 - ts_cal1) + 30;
        temp as i16
    }
}
