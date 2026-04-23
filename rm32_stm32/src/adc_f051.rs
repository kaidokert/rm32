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

const RCC_BASE: u32 = 0x4002_1000;
const ADC_BASE: u32 = 0x4001_2400;
const DMA1_BASE: u32 = 0x4002_0000;
const GPIOA_BASE: u32 = 0x4800_0000;

// DMA1 Channel 1 registers
const DMA_CH1_CCR: u32 = DMA1_BASE + 0x08;
const DMA_CH1_CNDTR: u32 = DMA1_BASE + 0x0C;
const DMA_CH1_CPAR: u32 = DMA1_BASE + 0x10;
const DMA_CH1_CMAR: u32 = DMA1_BASE + 0x14;

// ADC register offsets
const ADC_ISR: u32 = ADC_BASE + 0x00;
const ADC_CR: u32 = ADC_BASE + 0x08;
const ADC_CFGR1: u32 = ADC_BASE + 0x0C;
const ADC_SMPR: u32 = ADC_BASE + 0x14;
const ADC_CHSELR: u32 = ADC_BASE + 0x28;
const ADC_DR: u32 = ADC_BASE + 0x40;
const ADC_CCR: u32 = ADC_BASE + 0x308; // Common config

use crate::regs::{write as write_reg, read as read_reg, modify as modify_reg};

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

            // PA2, PA6 as analog
            modify_reg(GPIOA_BASE, |v| v | (0b11 << 4) | (0b11 << 12)); // PA2, PA6

            // DMA1 Channel 1: periph→memory, 16-bit, memory increment, circular
            write_reg(DMA_CH1_CCR, 0); // disable
            write_reg(DMA_CH1_CPAR, ADC_DR);
            write_reg(DMA_CH1_CMAR, ADC_DMA_BUF.as_ptr() as u32);
            write_reg(DMA_CH1_CNDTR, 3);
            write_reg(DMA_CH1_CCR,
                (1 << 5)       // CIRC (circular mode)
                | (1 << 7)     // MINC (memory increment)
                | (0b01 << 8)  // PSIZE = 16-bit (F0 ADC DR is 16-bit accessible)
                | (0b01 << 10) // MSIZE = 16-bit
            );
            modify_reg(DMA_CH1_CCR, |v| v | 1); // EN

            // ADC clock: PCLK/4
            modify_reg(ADC_CFGR1 + 4, |_| 0b10 << 30); // CFGR2: CKMODE = PCLK/4

            // Enable temperature sensor
            modify_reg(ADC_CCR, |v| v | (1 << 23)); // TSEN

            // Sampling time: 71.5 ADC clock cycles
            write_reg(ADC_SMPR, 0b110); // SMP = 71.5 cycles

            // Channel selection: CH2 (current) | CH6 (voltage) | CH16 (temp)
            // F0 CHSELR is a bitmask — channels are scanned in ascending order
            write_reg(ADC_CHSELR, (1 << 2) | (1 << 6) | (1 << 16));

            // Enable DMA on ADC
            modify_reg(ADC_CFGR1, |v| (v & !(0b11)) | (1 << 0) | (1 << 1)); // DMAEN=1, DMACFG=1 (circular)

            // Resolution 12-bit, right-aligned, scan direction forward
            modify_reg(ADC_CFGR1, |v| v & !(0b11 << 3)); // RES=00 (12-bit)

            // Calibrate
            write_reg(ADC_CR, 1 << 31); // ADCAL
            while read_reg(ADC_CR) & (1 << 31) != 0 {}

            // Stabilization delay
            cortex_m::asm::delay(48 * 20);

            // Enable ADC
            write_reg(ADC_ISR, 1 << 0); // clear ADRDY
            write_reg(ADC_CR, 1 << 0); // ADEN
            while read_reg(ADC_ISR) & (1 << 0) == 0 {} // wait ADRDY
        }
        Self { _private: () }
    }
}

impl Adc for F051Adc {
    fn start_conversion(&mut self) {
        unsafe { modify_reg(ADC_CR, |v| v | (1 << 2)); } // ADSTART
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
