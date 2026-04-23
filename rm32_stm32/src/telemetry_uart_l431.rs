//! USART1 telemetry TX via DMA for STM32L431 (KISS protocol).
//!
//! USART1 on PB6 (AF7), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 4 (request 2).

use rm32::hal::TelemetryUart;

static mut TX_BUF: [u8; 49] = [0; 49];

const RCC_BASE: u32 = 0x4002_1000;
const DMA1_BASE: u32 = 0x4002_0000;
const GPIOB_BASE: u32 = 0x4800_0400;
const USART1_BASE: u32 = 0x4001_3800;

// DMA1 Channel 4 registers
const DMA_CH4_CCR: u32 = DMA1_BASE + 0x44;
const DMA_CH4_CNDTR: u32 = DMA1_BASE + 0x48;
const DMA_CH4_CPAR: u32 = DMA1_BASE + 0x4C;
const DMA_CH4_CMAR: u32 = DMA1_BASE + 0x50;
const DMA_CSELR: u32 = DMA1_BASE + 0xA8;

const USART_CR1: u32 = USART1_BASE + 0x00;
const USART_CR3: u32 = USART1_BASE + 0x08;
const USART_BRR: u32 = USART1_BASE + 0x0C;
const USART_ISR: u32 = USART1_BASE + 0x1C;
const USART_TDR: u32 = USART1_BASE + 0x28;

use crate::regs::{write as write_reg, read as read_reg, modify as modify_reg};

pub struct L431TelemUart { _private: () }

impl L431TelemUart {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        unsafe {
            // Enable clocks: USART1 (APB2ENR bit 14), GPIOB (AHB2ENR bit 1), DMA1 (AHB1ENR bit 0)
            modify_reg(RCC_BASE + 0x60, |v| v | (1 << 14)); // APB2ENR
            modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 1));  // AHB2ENR GPIOBEN
            modify_reg(RCC_BASE + 0x48, |v| v | (1 << 0));  // AHB1ENR DMA1EN

            // PB6: AF7 (USART1_TX), open-drain, pull-up
            modify_reg(GPIOB_BASE, |v| (v & !(0b11 << 12)) | (0b10 << 12)); // AF mode
            modify_reg(GPIOB_BASE + 0x04, |v| v | (1 << 6)); // open-drain
            modify_reg(GPIOB_BASE + 0x0C, |v| (v & !(0b11 << 12)) | (0b01 << 12)); // pull-up
            // AFRL: PB6 = AF7 (bits [27:24])
            modify_reg(GPIOB_BASE + 0x20, |v| (v & !(0xF << 24)) | (7 << 24));

            // USART1: disable first
            write_reg(USART_CR1, 0);
            // BRR = 80_000_000 / 115200 ≈ 694
            write_reg(USART_BRR, 694);
            // Half-duplex
            modify_reg(USART_CR3, |v| v | (1 << 3)); // HDSEL
            // Enable TX + RX + UE
            write_reg(USART_CR1, (1 << 3) | (1 << 2) | (1 << 0));

            // Wait TEACK + REACK
            while read_reg(USART_ISR) & (1 << 21) == 0 {}
            while read_reg(USART_ISR) & (1 << 22) == 0 {}

            // DMA CSELR: CH4 request = 2 (USART1_TX)
            // CH4 uses bits [15:12]
            modify_reg(DMA_CSELR, |v| (v & !(0xF << 12)) | (2 << 12));

            // DMA CH4: memory→periph, 8-bit, MINC, TCIE
            write_reg(DMA_CH4_CPAR, USART_TDR);
            write_reg(DMA_CH4_CMAR, TX_BUF.as_ptr() as u32);
            write_reg(DMA_CH4_CCR,
                (1 << 1)   // TCIE
                | (1 << 3) // TEIE
                | (1 << 4) // DIR = memory-to-periph
                | (1 << 7) // MINC
            );
        }
        Self { _private: () }
    }
}

impl TelemetryUart for L431TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        unsafe {
            TX_BUF[..len].copy_from_slice(&data[..len]);
            modify_reg(DMA_CH4_CCR, |v| v & !1); // disable
            write_reg(DMA_CH4_CMAR, TX_BUF.as_ptr() as u32);
            write_reg(DMA_CH4_CNDTR, len as u32);
            modify_reg(USART_CR3, |v| v | (1 << 7)); // DMAT
            modify_reg(DMA_CH4_CCR, |v| v | 1); // enable
        }
    }
}
