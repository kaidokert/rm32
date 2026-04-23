//! USART1 telemetry TX via DMA for STM32F051 (KISS protocol).
//!
//! USART1 on PB6 (AF0), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 2 for TX transfers (fixed assignment, no DMAMUX on F0).

use rm32::hal::TelemetryUart;

/// Static TX buffer — DMA reads from here.
static mut TX_BUF: [u8; 49] = [0; 49];

const RCC_BASE: u32 = 0x4002_1000;
const DMA1_BASE: u32 = 0x4002_0000;
const GPIOB_BASE: u32 = 0x4800_0400;
const USART1_BASE: u32 = 0x4001_3800;

// DMA1 Channel 2 registers
const DMA_CH2_CCR: u32 = DMA1_BASE + 0x1C;
const DMA_CH2_CNDTR: u32 = DMA1_BASE + 0x20;
const DMA_CH2_CPAR: u32 = DMA1_BASE + 0x24;
const DMA_CH2_CMAR: u32 = DMA1_BASE + 0x28;

// USART register offsets
const USART_CR1: u32 = USART1_BASE + 0x00;
const USART_CR3: u32 = USART1_BASE + 0x08;
const USART_BRR: u32 = USART1_BASE + 0x0C;
const USART_ISR: u32 = USART1_BASE + 0x1C;
const USART_TDR: u32 = USART1_BASE + 0x28;

#[inline(always)]
unsafe fn write_reg(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }
#[inline(always)]
unsafe fn read_reg(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
#[inline(always)]
unsafe fn modify_reg(addr: u32, f: impl FnOnce(u32) -> u32) {
    let ptr = addr as *mut u32;
    ptr.write_volatile(f(ptr.read_volatile()));
}

pub struct F051TelemUart {
    _private: (),
}

impl F051TelemUart {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        unsafe {
            // Enable clocks: USART1 (APB2ENR bit 14), GPIOB (AHBENR bit 18), DMA1 (AHBENR bit 0)
            let apb2enr = (RCC_BASE + 0x18) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 14)); // USART1EN
            let ahbenr = (RCC_BASE + 0x14) as *mut u32;
            ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0) | (1 << 18)); // DMA1EN, GPIOBEN

            // PB6: alternate function (AF0 = USART1_TX), open-drain, pull-up
            // MODER: AF mode (0b10)
            modify_reg(GPIOB_BASE, |v| (v & !(0b11 << 12)) | (0b10 << 12)); // PB6 = AF
            // OTYPER: open-drain
            modify_reg(GPIOB_BASE + 0x04, |v| v | (1 << 6));
            // PUPDR: pull-up
            modify_reg(GPIOB_BASE + 0x0C, |v| (v & !(0b11 << 12)) | (0b01 << 12));
            // AFRL: PB6 = AF0 (bits [27:24] = 0)
            modify_reg(GPIOB_BASE + 0x20, |v| v & !(0xF << 24));

            // USART1 config: disable first
            write_reg(USART_CR1, 0);
            // BRR = 48_000_000 / 115200 ≈ 417
            write_reg(USART_BRR, 417);
            // Half-duplex mode
            modify_reg(USART_CR3, |v| v | (1 << 3)); // HDSEL
            // Enable TX + RX + UE
            write_reg(USART_CR1, (1 << 3) | (1 << 2) | (1 << 0)); // TE | RE | UE

            // Wait for TEACK (bit 21) + REACK (bit 22)
            while read_reg(USART_ISR) & (1 << 21) == 0 {}
            while read_reg(USART_ISR) & (1 << 22) == 0 {}

            // DMA1 Channel 2: USART1_TX (fixed assignment on F0)
            // Configure: memory→periph, 8-bit, memory increment, TC interrupt
            write_reg(DMA_CH2_CPAR, USART_TDR);
            write_reg(DMA_CH2_CMAR, TX_BUF.as_ptr() as u32);
            write_reg(DMA_CH2_CCR,
                (1 << 1)   // TCIE
                | (1 << 3) // TEIE
                | (1 << 4) // DIR = memory-to-periph
                | (1 << 7) // MINC
            );
        }
        Self { _private: () }
    }
}

impl TelemetryUart for F051TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        unsafe {
            TX_BUF[..len].copy_from_slice(&data[..len]);

            // Disable DMA channel first
            modify_reg(DMA_CH2_CCR, |v| v & !1);

            // Set memory address and count
            write_reg(DMA_CH2_CMAR, TX_BUF.as_ptr() as u32);
            write_reg(DMA_CH2_CNDTR, len as u32);

            // Enable USART DMA TX request
            modify_reg(USART_CR3, |v| v | (1 << 7)); // DMAT

            // Enable DMA channel
            modify_reg(DMA_CH2_CCR, |v| v | 1);
        }
    }
}
