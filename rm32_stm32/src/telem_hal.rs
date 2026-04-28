//! Telemetry UART driver abstraction — shared init/send sequence across STM32 families.
//!
//! `UartPeripheral` trait splits init into discrete operations.
//! `GenericTelemUart` provides the shared `TelemetryUart` impl with tx buffer management.

use crate::regs::InitError;
use rm32::hal::TelemetryUart;

/// MCU-specific UART + DMA register operations.
pub trait UartPeripheral {
    /// Enable USART, GPIO, DMA clocks via RCC.
    fn enable_clocks(&self);
    /// Configure TX pin as alternate function, open-drain, pull-up.
    fn configure_pin(&self);
    /// Configure USART: baud rate, half-duplex, enable TX+UE.
    fn configure_usart(&self);
    /// Wait for USART transmit enable acknowledge.
    fn wait_ready(&self) -> Result<(), InitError>;
    /// Configure DMA routing (CSELR on L4/F0, DMAMUX on G0/G4).
    fn configure_dma_routing(&self);
    /// Configure DMA channel: PAR→USART_TDR, memory-to-periph, MINC, TCIE.
    fn configure_dma_channel(&self);
    /// DMA send: disable channel, set MAR+NDTR, enable DMAT, enable channel.
    fn send_dma_raw(&self, buf_ptr: *const u8, len: u16);
}

/// Generic telemetry UART with shared tx buffer and send logic.
pub struct GenericTelemUart<U: UartPeripheral> {
    ops: U,
    tx_buf: [u8; 49],
}

impl<U: UartPeripheral> GenericTelemUart<U> {
    /// Shared init sequence — called from MCU-specific convenience constructors.
    pub fn new_init(ops: U) -> Result<Self, InitError> {
        ops.enable_clocks();
        ops.configure_pin();
        ops.configure_usart();
        ops.wait_ready()?;
        ops.configure_dma_routing();
        ops.configure_dma_channel();
        Ok(Self {
            ops,
            tx_buf: [0; 49],
        })
    }

    pub fn new_post_init(ops: U) -> Self {
        Self {
            ops,
            tx_buf: [0; 49],
        }
    }
}

impl<U: UartPeripheral> TelemetryUart for GenericTelemUart<U> {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        self.tx_buf[..len].copy_from_slice(&data[..len]);
        self.ops.send_dma_raw(self.tx_buf.as_ptr(), len as u16);
    }
}
