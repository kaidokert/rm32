//! ADC + DMA for voltage, current, and temperature on STM32L431.
//!
//! ADC1 with 3-rank scan: current (PA3/CH8), voltage (PA6/CH11),
//! temperature (internal CH17). DMA1 Channel 1 (request 0) circular.

use rm32::hal::Adc;

static mut ADC_DMA_BUF: [u16; 3] = [0; 3];

const RCC_BASE: u32 = 0x4002_1000;
const ADC1_BASE: u32 = 0x5004_0000; // L4 ADC1 base
const DMA1_BASE: u32 = 0x4002_0000;
const GPIOA_BASE: u32 = 0x4800_0000;

// ADC register offsets (L4)
const ADC_ISR: u32 = ADC1_BASE + 0x00;
const ADC_CR: u32 = ADC1_BASE + 0x08;
const ADC_CFGR: u32 = ADC1_BASE + 0x0C;
const ADC_SMPR1: u32 = ADC1_BASE + 0x14;
const ADC_SMPR2: u32 = ADC1_BASE + 0x18;
const ADC_SQR1: u32 = ADC1_BASE + 0x30;
const ADC_SQR2: u32 = ADC1_BASE + 0x34;
const ADC_DR: u32 = ADC1_BASE + 0x40;
const ADC_CCR: u32 = ADC1_BASE + 0x300 + 0x08; // Common config register

// DMA1 Channel 1
const DMA_CH1_CCR: u32 = DMA1_BASE + 0x08;
const DMA_CH1_CNDTR: u32 = DMA1_BASE + 0x0C;
const DMA_CH1_CPAR: u32 = DMA1_BASE + 0x10;
const DMA_CH1_CMAR: u32 = DMA1_BASE + 0x14;
const DMA_CSELR: u32 = DMA1_BASE + 0xA8;

#[inline(always)]
unsafe fn write_reg(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }
#[inline(always)]
unsafe fn read_reg(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
#[inline(always)]
unsafe fn modify_reg(addr: u32, f: impl FnOnce(u32) -> u32) {
    let ptr = addr as *mut u32;
    ptr.write_volatile(f(ptr.read_volatile()));
}

pub struct L431Adc { _private: () }

impl L431Adc {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        unsafe {
            // Enable clocks: ADC (AHB2ENR bit 13), DMA1 (AHB1ENR bit 0), GPIOA (AHB2ENR bit 0)
            modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 13) | (1 << 0)); // AHB2ENR
            modify_reg(RCC_BASE + 0x48, |v| v | (1 << 0)); // AHB1ENR: DMA1EN

            // ADC clock: select system clock via ADCSEL in RCC_CCIPR (bits [29:28])
            // 0b11 = no clock (reset), 0b01 = PLLSAI1, 0b10 = PLLSAI2, 0b00 = no
            // Use HCLK/1 synchronous clock via ADC_CCR CKMODE
            // Actually, simplest: use ADC_CCR CKMODE = 0b01 (HCLK/1)
            modify_reg(ADC_CCR, |v| (v & !(0b11 << 16)) | (0b01 << 16)); // CKMODE=HCLK/1

            // PA3, PA6 as analog
            modify_reg(GPIOA_BASE, |v| v | (0b11 << 6) | (0b11 << 12));

            // Enable temperature sensor (TSEN bit 23 in CCR)
            modify_reg(ADC_CCR, |v| v | (1 << 23));

            // DMA CSELR: Channel 1 request = 0 (ADC1)
            modify_reg(DMA_CSELR, |v| v & !(0xF << 0));

            // DMA1 CH1: periph→memory, 16-bit, memory increment, circular
            write_reg(DMA_CH1_CCR, 0);
            write_reg(DMA_CH1_CPAR, ADC_DR);
            write_reg(DMA_CH1_CMAR, ADC_DMA_BUF.as_ptr() as u32);
            write_reg(DMA_CH1_CNDTR, 3);
            write_reg(DMA_CH1_CCR,
                (1 << 5)       // CIRC
                | (1 << 7)     // MINC
                | (0b01 << 8)  // PSIZE = 16-bit
                | (0b01 << 10) // MSIZE = 16-bit
            );
            modify_reg(DMA_CH1_CCR, |v| v | 1); // EN

            // Disable deep power down, enable internal voltage regulator
            modify_reg(ADC_CR, |v| v & !(1 << 29)); // DEEPPWD = 0
            modify_reg(ADC_CR, |v| v | (1 << 28));  // ADVREGEN = 1
            // Wait for regulator startup (~20us at 80MHz)
            cortex_m::asm::delay(80 * 20);

            // Sampling time: 47.5 cycles for CH8, CH11, CH17
            // SMPR1 handles CH0-9, SMPR2 handles CH10-18
            // CH8: SMPR1 bits [26:24] = 0b100 (47.5 cycles)
            modify_reg(ADC_SMPR1, |v| (v & !(0b111 << 24)) | (0b100 << 24));
            // CH11: SMPR2 bits [5:3] = 0b100
            modify_reg(ADC_SMPR2, |v| (v & !(0b111 << 3)) | (0b100 << 3));
            // CH17: SMPR2 bits [23:21] = 0b100
            modify_reg(ADC_SMPR2, |v| (v & !(0b111 << 21)) | (0b100 << 21));

            // Sequence: 3 conversions
            // SQR1: L[3:0] = 2 (3 conversions), SQ1[10:6] = 8 (CH8)
            write_reg(ADC_SQR1, (2 << 0) | (8 << 6) | (11 << 12) | (17 << 18));

            // CFGR: DMA circular mode, resolution 12-bit
            write_reg(ADC_CFGR,
                (1 << 0)   // DMAEN
                | (1 << 1) // DMACFG = circular
            );

            // Calibrate (single-ended)
            modify_reg(ADC_CR, |v| v & !(1 << 30)); // ADCALDIF = 0 (single-ended)
            modify_reg(ADC_CR, |v| v | (1 << 31));  // ADCAL
            while read_reg(ADC_CR) & (1 << 31) != 0 {}

            cortex_m::asm::delay(80 * 20);

            // Enable ADC
            write_reg(ADC_ISR, 1 << 0); // clear ADRDY
            modify_reg(ADC_CR, |v| v | (1 << 0)); // ADEN
            while read_reg(ADC_ISR) & (1 << 0) == 0 {}
        }
        Self { _private: () }
    }
}

impl Adc for L431Adc {
    fn start_conversion(&mut self) {
        unsafe { modify_reg(ADC_CR, |v| v | (1 << 2)); } // ADSTART
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
