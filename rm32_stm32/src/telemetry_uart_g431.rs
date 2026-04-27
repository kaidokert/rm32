//! USART2 telemetry TX via DMA for STM32G431 (KISS protocol).
//!
//! USART2 on PB3 (AF7), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 3 (DMAMUX request 27 = USART2_TX).

use rm32::hal::TelemetryUart;
use crate::regs::{modify as modify_reg, write};

const RCC: u32 = 0x4002_1000;
const USART2_BASE: u32 = 0x4000_4400;
const DMA1_BASE: u32 = 0x4002_0000;
const DMAMUX_BASE: u32 = 0x4002_0800;
const GPIOB: u32 = 0x4800_0400;

// DMA1 Channel 3 registers
const DMA_CH3_CCR: u32 = DMA1_BASE + 0x30;
const DMA_CH3_CNDTR: u32 = DMA1_BASE + 0x34;
const DMA_CH3_CPAR: u32 = DMA1_BASE + 0x38;
const DMA_CH3_CMAR: u32 = DMA1_BASE + 0x3C;

pub struct G431TelemUart { tx_buf: [u8; 49] }

impl G431TelemUart {
    pub fn post_init() -> Self { Self { tx_buf: [0; 49] } }

    pub fn init() -> Result<Self, crate::regs::InitError> {
        unsafe {
            // Enable clocks: USART2 (APB1ENR1 bit 17), GPIOB (AHB2ENR bit 1), DMA1 (AHB1ENR bit 0)
            modify_reg(RCC + 0x58, |v| v | (1 << 17)); // APB1ENR1
            modify_reg(RCC + 0x4C, |v| v | (1 << 1));  // AHB2ENR GPIOBEN
            modify_reg(RCC + 0x48, |v| v | (1 << 0));  // AHB1ENR DMA1EN

            // PB3: AF7 (USART2_TX), open-drain, pull-up
            modify_reg(GPIOB, |v| (v & !(0b11 << 6)) | (0b10 << 6));       // MODER3 = AF
            modify_reg(GPIOB + 0x04, |v| v | (1 << 3));                      // OTYPER OT3 = open-drain
            modify_reg(GPIOB + 0x0C, |v| (v & !(0b11 << 6)) | (0b01 << 6)); // PUPDR3 = pull-up
            modify_reg(GPIOB + 0x20, |v| (v & !(0xF << 12)) | (7 << 12));   // AFRL3 = AF7

            // USART2: disable first
            write(USART2_BASE + 0x00, 0); // CR1
            // BRR = 170_000_000 / 115200 ≈ 1476
            write(USART2_BASE + 0x0C, 1476); // BRR
            // Half-duplex
            modify_reg(USART2_BASE + 0x08, |v| v | (1 << 3)); // CR3 HDSEL
            // Enable TX + UE
            write(USART2_BASE + 0x00, (1 << 3) | (1 << 0)); // CR1: TE + UE

            // Wait TEACK
            crate::regs::wait_for(|| {
                let isr = (USART2_BASE + 0x1C) as *const u32;
                isr.read_volatile() & (1 << 21) != 0
            }, 100_000, "USART TEACK")?;

            // DMAMUX: CH2 (DMA CH3) → USART2_TX request (27)
            write(DMAMUX_BASE + 0x08, 27); // DMAMUX_C2CR

            // DMA CH3: memory→periph, 8-bit, MINC, TCIE
            write(DMA_CH3_CPAR, USART2_BASE + 0x28); // TDR
            write(DMA_CH3_CMAR, 0);
            write(DMA_CH3_CCR,
                (1 << 1)   // TCIE
                | (1 << 3) // TEIE
                | (1 << 4) // DIR = memory-to-periph
                | (1 << 7) // MINC
            );
        }
        Ok(Self { tx_buf: [0; 49] })
    }
}

impl TelemetryUart for G431TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        self.tx_buf[..len].copy_from_slice(&data[..len]);
        unsafe {
            // Disable DMA
            modify_reg(DMA_CH3_CCR, |v| v & !1);
            write(DMA_CH3_CMAR, self.tx_buf.as_ptr() as u32);
            write(DMA_CH3_CNDTR, len as u32);
            // Enable USART DMAT
            modify_reg(USART2_BASE + 0x08, |v| v | (1 << 7)); // CR3 DMAT
            // Enable DMA
            modify_reg(DMA_CH3_CCR, |v| v | 1);
        }
    }
}
