//! DMA-based input capture for DShot/Servo signal reception on STM32L431.
//!
//! Uses TIM15 CH1 (PA2, AF14) + DMA1 Channel 5 (request 7).
//! L431 has a DMA request mux (CSELR register) unlike F051's fixed mapping.

use rm32::hal::InputCapture;

static mut DMA_BUFFER: [u32; 64] = [0; 64];
static mut GCR_BUFFER: [u32; 37] = [0; 37];

pub unsafe fn dma_buffer() -> &'static [u32; 64] { &DMA_BUFFER }
pub unsafe fn gcr_buffer() -> &'static mut [u32; 37] { &mut GCR_BUFFER }

const RCC_BASE: u32 = 0x4002_1000;
const DMA1_BASE: u32 = 0x4002_0000;
const TIM15_BASE: u32 = 0x4001_4000;
const GPIOA_BASE: u32 = 0x4800_0000;

// DMA1 Channel 5 registers (offset = 0x08 + (n-1)*0x14, n=5 → 0x08 + 4*0x14 = 0x58)
const DMA_CH5_CCR: u32 = DMA1_BASE + 0x58;
const DMA_CH5_CNDTR: u32 = DMA1_BASE + 0x5C;
const DMA_CH5_CPAR: u32 = DMA1_BASE + 0x60;
const DMA_CH5_CMAR: u32 = DMA1_BASE + 0x64;

// DMA CSELR (channel selection register) at DMA1_BASE + 0xA8
const DMA_CSELR: u32 = DMA1_BASE + 0xA8;

const CR1: u32 = 0x00;
const DIER: u32 = 0x0C;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;

use crate::regs::{write as write_reg, read as read_reg, modify as modify_reg};

pub struct L431DshotCapture {
    buffer_size: u16,
    ic_prescaler: u8,
    out_put: bool,
}

impl L431DshotCapture {
    pub fn new() -> Self {
        Self {
            buffer_size: 32,
            ic_prescaler: 80 / 6, // CPU_FREQUENCY_MHZ / 6 ≈ 13
            out_put: false,
        }
    }

    pub fn is_output(&self) -> bool { self.out_put }

    pub fn init(&self) {
        unsafe {
            // Enable clocks: TIM15 (APB2ENR bit 16), DMA1 (AHB1ENR bit 0), GPIOA (AHB2ENR bit 0)
            modify_reg(RCC_BASE + 0x60, |v| v | (1 << 16)); // APB2ENR: TIM15EN
            modify_reg(RCC_BASE + 0x48, |v| v | (1 << 0));  // AHB1ENR: DMA1EN
            modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 0));  // AHB2ENR: GPIOAEN

            // PA2 as alternate function AF14 (TIM15_CH1)
            modify_reg(GPIOA_BASE, |v| (v & !(0b11 << 4)) | (0b10 << 4));
            // AFRL: PA2 = AF14 (bits [11:8])
            modify_reg(GPIOA_BASE + 0x20, |v| (v & !(0xF << 8)) | (14 << 8));

            // DMA CSELR: Channel 5 request = 7 (TIM15_CH1)
            // CH5 uses bits [19:16]
            modify_reg(DMA_CSELR, |v| (v & !(0xF << 16)) | (7 << 16));
        }
    }
}

impl InputCapture for L431DshotCapture {
    fn receive_dshot_dma(&mut self) {
        unsafe {
            write_reg(DMA_CH5_CCR, 0); // disable DMA

            // Reset TIM15 (APB2RSTR bit 16)
            modify_reg(RCC_BASE + 0x20, |v| v | (1 << 16));
            modify_reg(RCC_BASE + 0x20, |v| v & !(1 << 16));

            // TIM15 input capture mode
            write_reg(TIM15_BASE + CCMR1, 0x41);
            write_reg(TIM15_BASE + CCER, 0x0A);
            write_reg(TIM15_BASE + PSC, self.ic_prescaler as u32);
            write_reg(TIM15_BASE + ARR, 0xFFFF);
            write_reg(TIM15_BASE + EGR, 1);
            write_reg(TIM15_BASE + CNT, 0);
            self.out_put = false;

            // DMA CH5: periph→memory, 32-bit, TCIE
            write_reg(DMA_CH5_CMAR, DMA_BUFFER.as_ptr() as u32);
            write_reg(DMA_CH5_CPAR, (TIM15_BASE + CCR1) as u32);
            write_reg(DMA_CH5_CNDTR, self.buffer_size as u32);
            write_reg(DMA_CH5_CCR, 0x98B);

            modify_reg(TIM15_BASE + DIER, |v| v | (1 << 9)); // CC1DE
            modify_reg(TIM15_BASE + CCER, |v| v | 1);
            modify_reg(TIM15_BASE + CR1, |v| v | 1);
        }
    }

    fn send_dshot_dma(&mut self) {
        unsafe {
            write_reg(DMA_CH5_CCR, 0);

            modify_reg(RCC_BASE + 0x20, |v| v | (1 << 16));
            modify_reg(RCC_BASE + 0x20, |v| v & !(1 << 16));

            write_reg(TIM15_BASE + CCMR1, 0x60);
            write_reg(TIM15_BASE + CCER, 0x03);
            write_reg(TIM15_BASE + PSC, 0);
            write_reg(TIM15_BASE + ARR, 61);
            write_reg(TIM15_BASE + EGR, 1);
            modify_reg(TIM15_BASE + 0x44, |v| v | (1 << 15)); // BDTR MOE
            self.out_put = true;

            write_reg(DMA_CH5_CMAR, GCR_BUFFER.as_ptr() as u32);
            write_reg(DMA_CH5_CPAR, (TIM15_BASE + CCR1) as u32);
            write_reg(DMA_CH5_CNDTR, 23 + self.buffer_size as u32 / 4);
            write_reg(DMA_CH5_CCR, 0x99B);

            modify_reg(TIM15_BASE + DIER, |v| v | (1 << 9));
            modify_reg(TIM15_BASE + CCER, |v| v | 1);
            modify_reg(TIM15_BASE + CR1, |v| v | 1);
        }
    }

    fn input_pin_state(&self) -> bool {
        (unsafe { read_reg(GPIOA_BASE + 0x10) }) & (1 << 2) != 0
    }

    fn set_pull_up(&mut self) {
        unsafe { modify_reg(GPIOA_BASE + 0x0C, |v| (v & !(0b11 << 4)) | (0b01 << 4)); }
    }

    fn set_pull_down(&mut self) {
        unsafe { modify_reg(GPIOA_BASE + 0x0C, |v| (v & !(0b11 << 4)) | (0b10 << 4)); }
    }

    fn set_pull_none(&mut self) {
        unsafe { modify_reg(GPIOA_BASE + 0x0C, |v| v & !(0b11 << 4)); }
    }
}
