//! USART1 telemetry TX via DMA for STM32F051 (KISS protocol).
//!
//! USART1 on PB6 (AF0), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 2 for TX transfers (fixed assignment, no DMAMUX on F0).

use rm32::hal::TelemetryUart;
use crate::pac::{DMA1, GPIOB, RCC, USART1};
use crate::regs::modify as modify_reg;
use crate::periph_addr as addr;

/// Static TX buffer — DMA reads from here.
static mut TX_BUF: [u8; 49] = [0; 49];

pub struct F051TelemUart {
    _private: (),
}

impl F051TelemUart {
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Self {
        let rcc = unsafe { &*RCC::ptr() };
        let gpiob = unsafe { &*GPIOB::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Enable clocks
        unsafe {
            let rcc_base = addr::RCC;
            let apb2enr = (rcc_base + 0x18) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 14)); // USART1EN
            let ahbenr = (rcc_base + 0x14) as *mut u32;
            ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0) | (1 << 18)); // DMA1EN, GPIOBEN
        }

        // PB6: alternate function (AF0 = USART1_TX), open-drain, pull-up
        let gpiob_base = addr::GPIOB;
        unsafe {
            modify_reg(gpiob_base, |v| (v & !(0b11 << 12)) | (0b10 << 12));
            modify_reg(gpiob_base + 0x04, |v| v | (1 << 6)); // open-drain
            modify_reg(gpiob_base + 0x0C, |v| (v & !(0b11 << 12)) | (0b01 << 12)); // pull-up
            modify_reg(gpiob_base + 0x20, |v| v & !(0xF << 24)); // AF0
        }

        // USART1 config via PAC accessors
        usart.cr1.write(|w| unsafe { w.bits(0) }); // disable
        usart.brr.write(|w| unsafe { w.bits(417) }); // 48MHz / 115200
        usart.cr3.write(|w| w.hdsel().set_bit()); // half-duplex
        usart.cr1.write(|w| w.te().set_bit().re().set_bit().ue().set_bit());

        // Wait for TEACK + REACK
        while !usart.isr.read().teack().bit_is_set() {}
        while !usart.isr.read().reack().bit_is_set() {}

        // DMA1 Channel 2: USART1_TX (fixed on F0)
        dma.ch2.par.write(|w| unsafe { w.bits(usart.tdr.as_ptr() as u32) });
        dma.ch2.mar.write(|w| unsafe { w.bits(TX_BUF.as_ptr() as u32) });
        dma.ch2.cr.write(|w| {
            w.tcie().enabled()
             .teie().enabled()
             .dir().from_memory()
             .minc().enabled()
        });

        Self { _private: () }
    }
}

impl TelemetryUart for F051TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        unsafe {
            TX_BUF[..len].copy_from_slice(&data[..len]);
        }

        // Disable DMA channel
        dma.ch2.cr.modify(|_, w| w.en().clear_bit());
        // Set address and count
        dma.ch2.mar.write(|w| unsafe { w.bits(TX_BUF.as_ptr() as u32) });
        dma.ch2.ndtr.write(|w| unsafe { w.bits(len as u32) });
        // Enable USART DMA TX request
        usart.cr3.modify(|_, w| w.dmat().set_bit());
        // Enable DMA channel
        dma.ch2.cr.modify(|_, w| w.en().set_bit());
    }
}
