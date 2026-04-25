//! USART1 telemetry TX via DMA (KISS protocol).
//!
//! USART1 on PB6 (AF0), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 3 for TX transfers.
//!
//! The C firmware uses this for KISS ESC telemetry (10-byte packets)
//! and ESC info responses (49-byte packets).

use crate::pac::{DMA1, GPIOB, RCC, USART1};
use rm32::hal::TelemetryUart;

pub struct TelemUart {
    tx_buf: [u8; 49],
}

impl TelemUart {
    /// Create a handle to already-initialized USART hardware.
    pub fn post_init() -> Self { Self { tx_buf: [0; 49] } }

    /// Initialize USART1 + DMA3 for half-duplex telemetry TX.
    pub fn init() -> Result<Self, crate::regs::InitError> {
        let rcc = unsafe { &*RCC::ptr() };
        let gpiob = unsafe { &*GPIOB::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Enable clocks: USART1 (APB2), GPIOB (IOP), DMA1 (AHB)
        rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 14)) }); // USART1EN
        rcc.iopenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 1)) });   // GPIOBEN
        rcc.ahbenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });   // DMA1EN

        // PB6: alternate function 0 (USART1_TX), open-drain, pull-up
        let pin = 6u32;
        let moder_off = pin * 2;
        gpiob.moder().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 << moder_off)) | (0b10 << moder_off))
        });
        // Open-drain
        gpiob.otyper().modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin)) });
        // Pull-up
        let pupdr_off = pin * 2;
        gpiob.pupdr().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 << pupdr_off)) | (0b01 << pupdr_off))
        });
        // AF0 for PB6 (AFRL register, pin 6 = bits [27:24])
        let afr_off = pin * 4;
        gpiob.afrl().modify(|r, w| unsafe {
            w.bits(r.bits() & !(0xF << afr_off)) // AF0 = 0
        });

        // USART1 config: 115200 baud @ 64MHz, 8N1, half-duplex
        usart.cr1().write(|w| unsafe { w.bits(0) }); // disable first
        // BRR = 64_000_000 / 115200 ≈ 556
        usart.brr().write(|w| unsafe { w.bits(556) });
        // Half-duplex mode
        usart.cr3().write(|w| w.hdsel().set_bit());
        // Enable TX+RX, then enable USART
        usart.cr1().write(|w| w.te().set_bit().re().set_bit().ue().set_bit());

        // Wait for TEACK + REACK
        crate::regs::wait_for(|| usart.isr().read().teack().bit_is_set(), 100_000, "USART TEACK")?;
        crate::regs::wait_for(|| usart.isr().read().reack().bit_is_set(), 100_000, "USART REACK")?;

        // DMA1 Channel 3: USART1_TX
        // DMAMUX channel 2 (0-indexed) → USART1_TX request
        // DMAMUX: Channel 2 → USART1_TX (request 51)
        let dmamux = unsafe { &*crate::pac::DMAMUX::ptr() };
        dmamux.ccr(2).modify(|r, w| unsafe { w.bits((r.bits() & !0x3F) | 51) });

        // Configure DMA channel 3 (index 2)
        // MAR is set to 0 here; send_dma() sets the actual buffer address before each transfer.
        dma.ch(2).par().write(|w| unsafe { w.bits(usart.tdr().as_ptr() as u32) });
        dma.ch(2).mar().write(|w| unsafe { w.bits(0) });

        // Enable TC + TE interrupts on DMA channel 3
        // (DMA2_3 IRQ handler clears send_telemetry flag)
        dma.ch(2).cr().write(|w| unsafe {
            w.bits(
                (1 << 1)  // TCIE
                | (1 << 3)  // TEIE
                | (1 << 4)  // DIR = memory-to-periph
                | (1 << 7)  // MINC
            )
        });

        Ok(Self { tx_buf: [0; 49] })
    }
}

impl TelemetryUart for TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let usart = unsafe { &*USART1::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Copy data into instance buffer
        let len = data.len().min(49);
        self.tx_buf[..len].copy_from_slice(&data[..len]);

        // Set TX direction
        usart.cr1().modify(|_, w| w.te().set_bit());

        // Configure and start DMA transfer
        dma.ch(2).cr().modify(|_, w| w.en().clear_bit()); // disable first
        dma.ch(2).mar().write(|w| unsafe { w.bits(self.tx_buf.as_ptr() as u32) });
        dma.ch(2).ndtr().write(|w| unsafe { w.bits(len as u32) });

        // Enable USART DMA TX request
        usart.cr3().modify(|_, w| w.dmat().set_bit());

        // Enable DMA channel
        dma.ch(2).cr().modify(|_, w| w.en().set_bit());
    }
}
