//! USART1 telemetry TX via DMA for STM32L431 (KISS protocol).
//!
//! USART1 on PB6 (AF7), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 4 (request 2).

use rm32::hal::TelemetryUart;

static mut TX_BUF: [u8; 49] = [0; 49];

use crate::periph_addr as addr;
use crate::pac::{DMA1, GPIOB, USART1};
use crate::regs::modify as modify_reg;

const RCC_BASE: u32 = addr::RCC;

pub struct L431TelemUart { _private: () }

impl L431TelemUart {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        let gpiob = unsafe { &*GPIOB::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        unsafe {
            // Enable clocks: USART1 (APB2ENR bit 14), GPIOB (AHB2ENR bit 1), DMA1 (AHB1ENR bit 0)
            modify_reg(RCC_BASE + 0x60, |v| v | (1 << 14)); // APB2ENR
            modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 1));  // AHB2ENR GPIOBEN
            modify_reg(RCC_BASE + 0x48, |v| v | (1 << 0));  // AHB1ENR DMA1EN

            // PB6: AF7 (USART1_TX), open-drain, pull-up
            gpiob.moder.modify(|_, w| w.moder6().bits(0b10)); // AF mode
            gpiob.otyper.modify(|_, w| w.ot6().set_bit());    // open-drain
            gpiob.pupdr.modify(|_, w| w.pupdr6().bits(0b01)); // pull-up
            // AFRL: PB6 = AF7 (bits [27:24])
            gpiob.afrl.modify(|_, w| w.afrl6().bits(7));

            // USART1: disable first
            usart.cr1.write(|w| w.bits(0));
            // BRR = 80_000_000 / 115200 ≈ 694
            usart.brr.write(|w| w.bits(694));
            // Half-duplex
            usart.cr3.modify(|_, w| w.hdsel().set_bit());
            // Enable TX + RX + UE
            usart.cr1.write(|w| w.te().set_bit().re().set_bit().ue().set_bit());

            // Wait TEACK + REACK
            while !usart.isr.read().teack().bit_is_set() {}
            while !usart.isr.read().reack().bit_is_set() {}

            // DMA CSELR: CH4 request = 2 (USART1_TX), bits [15:12]
            dma.cselr.modify(|r, w| w.bits((r.bits() & !(0xF << 12)) | (2 << 12)));

            // DMA CH4: memory→periph, 8-bit, MINC, TCIE
            dma.cpar4.write(|w| w.bits(usart.tdr.as_ptr() as u32));
            dma.cmar4.write(|w| w.bits(TX_BUF.as_ptr() as u32));
            dma.ccr4.write(|w| w.bits(
                (1 << 1)   // TCIE
                | (1 << 3) // TEIE
                | (1 << 4) // DIR = memory-to-periph
                | (1 << 7) // MINC
            ));
        }
        Self { _private: () }
    }
}

impl TelemetryUart for L431TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        unsafe {
            TX_BUF[..len].copy_from_slice(&data[..len]);
            dma.ccr4.modify(|r, w| w.bits(r.bits() & !1)); // disable
            dma.cmar4.write(|w| w.bits(TX_BUF.as_ptr() as u32));
            dma.cndtr4.write(|w| w.bits(len as u32));
            usart.cr3.modify(|_, w| w.dmat().set_bit()); // DMAT
            dma.ccr4.modify(|r, w| w.bits(r.bits() | 1)); // enable
        }
    }
}
