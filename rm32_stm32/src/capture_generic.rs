//! Generic DShot/Servo input capture driver — one implementation for all MCUs.
//!
//! MCU-specific register details abstracted via DmaOps + TimerOps + InputPinOps.

use rm32::hal::InputCapture;
use crate::capture_hal::{DmaOps, TimerOps, InputPinOps};

pub struct GenericCapture<D: DmaOps, T: TimerOps, P: InputPinOps> {
    pub buffer_size: u16,
    out_put: bool,
    dma_buf: [u32; 64],
    gcr_buf: [u32; 37],
    dma: D,
    timer: T,
    pin: P,
}

impl<D: DmaOps, T: TimerOps, P: InputPinOps> GenericCapture<D, T, P> {
    pub fn new(dma: D, timer: T, pin: P) -> Self {
        Self {
            buffer_size: 32,
            out_put: false,
            dma_buf: [0; 64],
            gcr_buf: [0; 37],
            dma, timer, pin,
        }
    }

}

impl<D: DmaOps, T: TimerOps, P: InputPinOps> InputCapture for GenericCapture<D, T, P> {
    fn receive_dshot_dma(&mut self) {
        self.dma.disable();
        self.timer.reset();
        self.timer.configure_capture(0); // prescaler set per-MCU in TimerOps impl
        self.out_put = false;

        self.dma.set_mar(self.dma_buf.as_ptr() as u32);
        self.dma.set_par(self.timer.ccr_addr());
        self.dma.set_ndtr(self.buffer_size as u32);
        self.dma.start_rx();
        self.timer.start();
    }

    fn send_dshot_dma(&mut self) {
        self.dma.disable();
        self.timer.reset();
        self.timer.configure_output(92); // DShot bit period
        self.out_put = true;

        self.dma.set_mar(self.gcr_buf.as_ptr() as u32);
        self.dma.set_par(self.timer.ccr_addr());
        self.dma.set_ndtr(23 + self.buffer_size as u32 / 4);
        self.dma.start_tx();
        self.timer.start();
    }

    fn input_pin_state(&self) -> bool { self.pin.read() }
    fn set_pull_up(&mut self) { self.pin.set_pull_up(); }
    fn set_pull_down(&mut self) { self.pin.set_pull_down(); }
    fn set_pull_none(&mut self) { self.pin.set_pull_none(); }
    fn dma_buffer(&self) -> &[u32; 64] { &self.dma_buf }
    fn gcr_buffer(&mut self) -> &mut [u32; 37] { &mut self.gcr_buf }
    fn is_output(&self) -> bool { self.out_put }
}
