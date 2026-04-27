//! USART1 telemetry TX via DMA for STM32F051 (KISS protocol).
//!
//! USART1 on PB6 (AF0), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 2 for TX transfers (fixed assignment, no DMAMUX on F0).

use rm32::hal::TelemetryUart;
use crate::pac::{DMA1, GPIOB, RCC, USART1};

pub struct F051TelemUart {
    tx_buf: [u8; 49],
}

impl F051TelemUart {
    pub fn post_init() -> Self { Self { tx_buf: [0; 49] } }

    pub fn init() -> Result<Self, crate::regs::InitError> {
        let rcc = unsafe { &*RCC::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Enable clocks
        unsafe {
            rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
            rcc.ahbenr.modify(|_, w| w.dmaen().set_bit().iopben().set_bit());
        }

        // PB6: alternate function (AF0 = USART1_TX), open-drain, pull-up
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.moder.modify(|_, w| w.moder6().alternate());
        gpiob.otyper.modify(|_, w| w.ot6().open_drain());
        gpiob.pupdr.modify(|_, w| w.pupdr6().pull_up());
        gpiob.afrl.modify(|_, w| w.afrl6().af0());

        // USART1 config via PAC accessors
        usart.cr1.write(|w| unsafe { w.bits(0) }); // disable
        usart.brr.write(|w| unsafe { w.bits(417) }); // 48MHz / 115200
        usart.cr3.write(|w| w.hdsel().set_bit()); // half-duplex
        usart.cr1.write(|w| w.te().set_bit().re().set_bit().ue().set_bit());

        // Wait for TEACK + REACK
        crate::regs::wait_for(|| usart.isr.read().teack().bit_is_set(), 100_000, "USART TEACK")?;
        crate::regs::wait_for(|| usart.isr.read().reack().bit_is_set(), 100_000, "USART REACK")?;

        // DMA1 Channel 2: USART1_TX (fixed on F0)
        dma.ch2.par.write(|w| unsafe { w.bits(usart.tdr.as_ptr() as u32) });
        dma.ch2.mar.write(|w| unsafe { w.bits(0) }); // set by send_dma()
        dma.ch2.cr.write(|w| {
            w.tcie().enabled()
             .teie().enabled()
             .dir().from_memory()
             .minc().enabled()
        });

        Ok(Self { tx_buf: [0; 49] })
    }
}

impl TelemetryUart for F051TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        self.tx_buf[..len].copy_from_slice(&data[..len]);

        // Disable DMA channel
        dma.ch2.cr.modify(|_, w| w.en().clear_bit());
        // Set address and count
        dma.ch2.mar.write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });
        dma.ch2.ndtr.write(|w| unsafe { w.bits(len as u32) });
        // Enable USART DMA TX request
        usart.cr3.modify(|_, w| w.dmat().set_bit());
        // Enable DMA channel
        dma.ch2.cr.modify(|_, w| w.en().set_bit());
    }
}
