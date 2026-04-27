//! G431 ADC driver.
//!
//! Single mode (PROTONDRIVE): ADC1 → temp, voltage, current via DMA1_CH2.
//! Dual mode (SEQURE): ADC1 → temp, NTC via DMA1_CH2; ADC2 → voltage, current via DMA1_CH4.

use crate::adc_hal::{AdcOps, TempCalibration};
use crate::adc_generic::GenericAdc;
use crate::dma_buf::DmaBuf;
use crate::regs::{modify as modify_reg, write, InitError, wait_for};
use crate::periph_addr as addr;

static ADC_DMA_BUF: DmaBuf<u16, 3> = DmaBuf::new();
// Dual mode: separate buffers for ADC1 and ADC2
static ADC1_DMA_BUF: DmaBuf<u16, 2> = DmaBuf::new();
static ADC2_DMA_BUF: DmaBuf<u16, 2> = DmaBuf::new();

// G4 temp cal addresses (same as L4 family)
const TEMP_CAL: TempCalibration = TempCalibration {
    cal1_addr: 0x1FFF_75A8, cal2_addr: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 110,
};

// G431 peripheral base addresses
const RCC: u32 = 0x4002_1000;
const ADC1_BASE: u32 = 0x5000_0000;
const ADC_COMMON: u32 = 0x5000_0300;
const DMA1_BASE: u32 = 0x4002_0000;
const DMAMUX_BASE: u32 = 0x4002_0800;
const GPIOA: u32 = 0x4800_0000;

pub struct G431AdcOps;

impl AdcOps for G431AdcOps {
    fn init(&self) -> Result<(), InitError> {
        unsafe {
            // Enable clocks: ADC12 (AHB2ENR bit 13), DMA1 (AHB1ENR bit 0), GPIOA (AHB2ENR bit 0)
            modify_reg(RCC + 0x4C, |v| v | (1 << 13) | (1 << 0)); // AHB2ENR
            modify_reg(RCC + 0x48, |v| v | (1 << 0));              // AHB1ENR

            // PA4, PA5 as analog
            let moder = GPIOA as *mut u32;
            modify_reg(GPIOA, |v| (v & !(0b11 << 8 | 0b11 << 10)) | (0b11 << 8 | 0b11 << 10));

            // ADC common: CKMODE = PCLK/4
            write(ADC_COMMON + 0x08, 0b11 << 16); // CCR: CKMODE=0b11 (HCLK/4)
            // Enable temperature sensor channel
            modify_reg(ADC_COMMON + 0x08, |v| v | (1 << 23)); // VSENSESEL

            // DMA1 Channel 2 setup
            // DMAMUX: CH1 (DMA CH2 = DMAMUX CH1) → ADC1 request (5)
            write(DMAMUX_BASE + 0x04, 5); // DMAMUX_C1CR = ADC1

            let dma_ch2_base = DMA1_BASE + 0x1C; // Channel 2 offset
            write(dma_ch2_base + 0x00, 0); // CCR: disable
            write(dma_ch2_base + 0x08, ADC1_BASE + 0x40); // CPAR: ADC1_DR
            write(dma_ch2_base + 0x0C, ADC_DMA_BUF.as_ptr() as u32); // CMAR
            write(dma_ch2_base + 0x04, 3); // CNDTR: 3 transfers
            // CCR: circ, minc, 16-bit psize, 16-bit msize
            write(dma_ch2_base + 0x00, (1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10));
            modify_reg(dma_ch2_base + 0x00, |v| v | 1); // Enable

            // ADC1 configuration
            // Exit deep power-down
            modify_reg(ADC1_BASE + 0x08, |v| v & !(1 << 29)); // DEEPPWD clear
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 28));  // ADVREGEN enable
            cortex_m::asm::delay(170 * 20); // ~20us startup

            // Sampling time: 47.5 cycles for all channels
            write(ADC1_BASE + 0x14, 0b100 << 15 | 0b100 << 12); // SMPR1: CH5=47.5, CH4=47.5
            write(ADC1_BASE + 0x18, 0b100 << 9);                 // SMPR2: CH13=47.5
            // Enable temperature sensor sampling time
            modify_reg(ADC1_BASE + 0x14, |v| v | (0b100 << 0)); // SMP0 for TEMPSENSOR (CH16)

            // Sequence: 3 conversions — TEMPSENSOR(16), voltage(13), current(5)
            write(ADC1_BASE + 0x2C, (2 << 0) | (16 << 6) | (13 << 12) | (5 << 18)); // SQR1

            // CFGR: DMAEN + DMACFG (circular) + CONT
            write(ADC1_BASE + 0x0C, (1 << 0) | (1 << 1) | (1 << 13));

            // Calibration
            modify_reg(ADC1_BASE + 0x08, |v| v & !(1 << 30)); // ADCALDIF clear (single-ended)
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 31));  // ADCAL start
            wait_for(|| {
                let cr = (ADC1_BASE + 0x08) as *const u32;
                cr.read_volatile() & (1 << 31) == 0
            }, 100_000, "ADC cal")?;
            cortex_m::asm::delay(170 * 20);

            // Enable ADC
            write(ADC1_BASE + 0x00, 1 << 0); // ISR: clear ADRDY
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 0)); // ADEN
            wait_for(|| {
                let isr = (ADC1_BASE as *const u32).read_volatile();
                isr & (1 << 0) != 0
            }, 100_000, "ADC ready")?;
        }
        Ok(())
    }

    fn start_conversion(&self) {
        unsafe {
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 2)); // ADSTART
        }
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
// ADC1: TEMPSENSOR + NTC (PB1/CH12) → DMA1_CH2
// ADC2: Voltage (PA6/CH3) + Current (PA7/CH4) → DMA1_CH4
// ============================================================

const ADC2_BASE: u32 = 0x5000_0100;
const GPIOB: u32 = 0x4800_0400;

/// Dual ADC: implements Adc trait directly, owns two DMA buffers.
pub struct G431DualAdc;

impl G431DualAdc {
    pub fn init() -> Result<Self, InitError> {
        unsafe {
            // Enable clocks: ADC12, DMA1, GPIOA, GPIOB
            modify_reg(RCC + 0x4C, |v| v | (1 << 13) | (1 << 0) | (1 << 1)); // AHB2ENR
            modify_reg(RCC + 0x48, |v| v | (1 << 0)); // AHB1ENR

            // PA6, PA7 as analog (ADC2 inputs)
            modify_reg(GPIOA, |v| v | (0b11 << 12) | (0b11 << 14));
            // PB1 as analog (NTC input)
            modify_reg(GPIOB, |v| v | (0b11 << 2));

            // ADC common: CKMODE = PCLK/4, VSENSESEL, independent mode
            write(ADC_COMMON + 0x08, (0b11 << 16) | (1 << 23));

            // --- DMA1 Channel 2 → ADC1 ---
            write(DMAMUX_BASE + 0x04, 5); // DMAMUX_C1CR = ADC1
            let dma_ch2 = DMA1_BASE + 0x1C;
            write(dma_ch2 + 0x00, 0);
            write(dma_ch2 + 0x08, ADC1_BASE + 0x40); // CPAR
            write(dma_ch2 + 0x0C, ADC1_DMA_BUF.as_ptr() as u32);
            write(dma_ch2 + 0x04, 2); // 2 transfers
            write(dma_ch2 + 0x00, (1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10));
            modify_reg(dma_ch2 + 0x00, |v| v | 1);

            // --- DMA1 Channel 4 → ADC2 ---
            write(DMAMUX_BASE + 0x0C, 36); // DMAMUX_C3CR = ADC2 (request 36)
            let dma_ch4 = DMA1_BASE + 0x44;
            write(dma_ch4 + 0x00, 0);
            write(dma_ch4 + 0x08, ADC2_BASE + 0x40); // CPAR
            write(dma_ch4 + 0x0C, ADC2_DMA_BUF.as_ptr() as u32);
            write(dma_ch4 + 0x04, 2); // 2 transfers
            write(dma_ch4 + 0x00, (1 << 5) | (1 << 7) | (0b01 << 8) | (0b01 << 10));
            modify_reg(dma_ch4 + 0x00, |v| v | 1);

            // --- ADC1: TEMPSENSOR(16) + NTC(12) ---
            modify_reg(ADC1_BASE + 0x08, |v| v & !(1 << 29));
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 28));
            cortex_m::asm::delay(170 * 20);

            write(ADC1_BASE + 0x14, 0b100 << 0); // SMPR1: SMP0 for temp sensor
            write(ADC1_BASE + 0x18, 0b100 << 6); // SMPR2: SMP12 = 47.5 cycles
            write(ADC1_BASE + 0x2C, (1 << 0) | (16 << 6) | (12 << 12)); // SQR1: 2 ranks, temp+NTC
            write(ADC1_BASE + 0x0C, (1 << 0) | (1 << 1)); // CFGR: DMAEN + DMACFG

            modify_reg(ADC1_BASE + 0x08, |v| v & !(1 << 30));
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 31));
            wait_for(|| (ADC1_BASE + 0x08) as *const u32 == core::ptr::null() || {
                ((ADC1_BASE + 0x08) as *const u32).read_volatile() & (1 << 31) == 0
            }, 100_000, "ADC1 cal")?;
            cortex_m::asm::delay(170 * 20);

            write(ADC1_BASE + 0x00, 1);
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 0));
            wait_for(|| {
                (ADC1_BASE as *const u32).read_volatile() & 1 != 0
            }, 100_000, "ADC1 ready")?;

            // --- ADC2: Voltage(CH3) + Current(CH4) ---
            modify_reg(ADC2_BASE + 0x08, |v| v & !(1 << 29));
            modify_reg(ADC2_BASE + 0x08, |v| v | (1 << 28));
            cortex_m::asm::delay(170 * 20);

            write(ADC2_BASE + 0x14, (0b010 << 9) | (0b100 << 12)); // SMPR1: CH3=2.5cyc, CH4=47.5cyc
            write(ADC2_BASE + 0x2C, (1 << 0) | (3 << 6) | (4 << 12)); // SQR1: 2 ranks
            write(ADC2_BASE + 0x0C, (1 << 0) | (1 << 1)); // CFGR: DMAEN + DMACFG

            modify_reg(ADC2_BASE + 0x08, |v| v & !(1 << 30));
            modify_reg(ADC2_BASE + 0x08, |v| v | (1 << 31));
            wait_for(|| {
                ((ADC2_BASE + 0x08) as *const u32).read_volatile() & (1 << 31) == 0
            }, 100_000, "ADC2 cal")?;
            cortex_m::asm::delay(170 * 20);

            write(ADC2_BASE + 0x00, 1);
            modify_reg(ADC2_BASE + 0x08, |v| v | (1 << 0));
            wait_for(|| {
                (ADC2_BASE as *const u32).read_volatile() & 1 != 0
            }, 100_000, "ADC2 ready")?;
        }
        Ok(Self)
    }

    pub fn post_init() -> Self { Self }
}

impl rm32::hal::Adc for G431DualAdc {
    fn start_conversion(&mut self) {
        unsafe {
            modify_reg(ADC1_BASE + 0x08, |v| v | (1 << 2));
        }
    }

    fn start_conversion_2(&mut self) {
        unsafe {
            modify_reg(ADC2_BASE + 0x08, |v| v | (1 << 2));
        }
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
