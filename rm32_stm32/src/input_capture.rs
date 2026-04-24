//! DMA-based input capture for DShot/Servo signal reception.
//!
//! Uses TIM3 CH1 (PB4, HARDWARE_GROUP_G0_A) + DMA1 Channel 1 to capture
//! pulse widths into a buffer. DMA transfer complete interrupt triggers
//! frame processing.
//!
//! For HARDWARE_GROUP_G0_A:
//!   Input pin: PB4 (TIM3_CH1)
//!   DMA: DMA1_Channel1
//!   Timer: TIM3

use crate::pac::{DMA1, TIM3, RCC, GPIOB};
use rm32::hal::InputCapture;

/// Input capture state — owns DMA and GCR buffers.
pub struct DshotCapture {
    buffer_size: u16,
    ic_prescaler: u8,
    out_put: bool,
    dma_buf: [u32; 64],
    gcr_buf: [u32; 37],
}

impl DshotCapture {
    pub fn new() -> Self {
        Self {
            buffer_size: 32,
            ic_prescaler: 64 / 6, // CPU_FREQUENCY_MHZ / 6
            out_put: false,
            dma_buf: [0; 64],
            gcr_buf: [0; 37],
        }
    }

    /// Returns true if currently in output (TX) mode.
    pub fn is_output(&self) -> bool { self.out_put }
    pub fn dma_buffer(&self) -> &[u32; 64] { &self.dma_buf }
    pub fn gcr_buffer(&mut self) -> &mut [u32; 37] { &mut self.gcr_buf }

    /// Initialize TIM3 + DMA1_CH1 for input capture.
    pub fn init(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        let gpiob = unsafe { &*GPIOB::ptr() };
        let tim3 = unsafe { &*TIM3::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Enable clocks (TIM3, DMA1, GPIOB) via raw bits
        rcc.apbenr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 1)) });  // TIM3EN
        rcc.ahbenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });   // DMA1EN
        rcc.iopenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 1)) });   // GPIOBEN

        // PB4 as alternate function (AF1 = TIM3_CH1)
        let pin = 4u32;
        let moder_offset = pin * 2;
        gpiob.moder().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 << moder_offset)) | (0b10 << moder_offset))
        });
        // AFR[0] for pins 0-7, AF1 for TIM3
        let afr_offset = pin * 4;
        gpiob.afrl().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0xF << afr_offset)) | (1 << afr_offset))
        });

        // DMA1 Channel 1: memory-to-memory disabled, peripheral->memory,
        // 32-bit transfers, memory increment, circular disabled
        // DMAMUX: TIM3_CH1 request
        let dmamux = unsafe { &*(0x4002_0800 as *const DmaMuxRegs) };
        dmamux.c0cr.modify(|v| (v & !0x3F) | 32); // TIM3_CH1 = request 32

        // Initial DMA config done in receive_dshot_dma()
        let _ = (tim3, dma); // used in receive/send
    }

    /// Configure for DShot/servo reception (input capture mode).
    pub fn receive(&mut self) {
        let tim3 = unsafe { &*TIM3::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Reset TIM3
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apbrstr1().modify(|_, w| w.tim3rst().set_bit());
        rcc.apbrstr1().modify(|_, w| w.tim3rst().clear_bit());

        // IC mode: CC1 as input, filter=4
        tim3.ccmr1_output().write(|w| unsafe { w.bits(0x41) });
        // Capture on both edges
        tim3.ccer().write(|w| unsafe { w.bits(0x0A) });
        // Prescaler and ARR
        tim3.psc().write(|w| unsafe { w.bits(self.ic_prescaler as u32) });
        tim3.arr().write(|w| unsafe { w.bits(0xFFFF) });
        tim3.egr().write(|w| w.ug().set_bit());
        tim3.cnt().write(|w| unsafe { w.bits(0) });

        self.out_put = false;

        // DMA1 Channel 1 config
        dma.ch(0).cr().write(|w| unsafe { w.bits(0) }); // disable first
        dma.ch(0).mar().write(|w| unsafe { w.bits(self.dma_buf.as_ptr() as u32) });
        dma.ch(0).par().write(|w| unsafe { w.bits(tim3.ccr1().as_ptr() as u32) });
        dma.ch(0).ndtr().write(|w| unsafe { w.bits(self.buffer_size as u32) });
        // Enable: mem increment, 32-bit periph/mem, transfer complete IRQ
        dma.ch(0).cr().write(|w| unsafe { w.bits(0x98B) });

        // Enable DMA request from TIM3 CC1
        tim3.dier().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 9)) }); // CC1DE
        tim3.ccer().modify(|r, w| unsafe { w.bits(r.bits() | 1) }); // CC1E
        tim3.cr1().modify(|_, w| w.cen().set_bit());
    }
}

impl InputCapture for DshotCapture {
    fn receive_dshot_dma(&mut self) {
        self.receive();
    }

    fn send_dshot_dma(&mut self) {
        let tim3 = unsafe { &*TIM3::ptr() };
        let dma = unsafe { &*DMA1::ptr() };
        let rcc = unsafe { &*RCC::ptr() };

        // Reset TIM3
        rcc.apbrstr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 1)) });
        rcc.apbrstr1().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 1)) });

        // Switch to PWM output mode: CCMR1=0x60 (PWM mode 1), CCER=0x03 (output enable)
        tim3.ccmr1_output().write(|w| unsafe { w.bits(0x60) });
        tim3.ccer().write(|w| unsafe { w.bits(0x03) });

        // Prescaler for output (0 for DShot600, 1 for DShot300)
        tim3.psc().write(|w| unsafe { w.bits(0) }); // TODO: use output_timer_prescaler
        tim3.arr().write(|w| unsafe { w.bits(92) }); // bit period
        self.out_put = true;
        tim3.egr().write(|w| w.ug().set_bit());

        // DMA: memory→peripheral, read GCR buffer, write to CCR1
        dma.ch(0).cr().write(|w| unsafe { w.bits(0) }); // disable
        dma.ch(0).mar().write(|w| unsafe { w.bits(self.gcr_buf.as_ptr() as u32) });
        dma.ch(0).par().write(|w| unsafe { w.bits(tim3.ccr1().as_ptr() as u32) });
        dma.ch(0).ndtr().write(|w| unsafe { w.bits(23 + self.buffer_size as u32 / 4) }); // 23 + padding
        // 0x99B = DIR=1 (mem→periph) | MINC | PSIZE=16bit | MSIZE=32bit | TCIE | EN
        dma.ch(0).cr().write(|w| unsafe { w.bits(0x99B) });

        // Enable DMA request, output, start
        tim3.dier().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 9)) }); // CC1DE
        tim3.ccer().modify(|r, w| unsafe { w.bits(r.bits() | 1) }); // CC1E
        tim3.cr1().modify(|_, w| w.cen().set_bit());
    }

    fn input_pin_state(&self) -> bool {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.idr().read().bits() & (1 << 4) != 0
    }

    fn set_pull_up(&mut self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        let pin = 4u32;
        let offset = pin * 2;
        gpiob.pupdr().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
        });
    }

    fn set_pull_down(&mut self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        let pin = 4u32;
        let offset = pin * 2;
        gpiob.pupdr().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
        });
    }

    fn set_pull_none(&mut self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        let pin = 4u32;
        let offset = pin * 2;
        gpiob.pupdr().modify(|r, w| unsafe {
            w.bits(r.bits() & !(0b11 << offset))
        });
    }
}

/// DMAMUX register block (not in PAC for G071 properly)
#[repr(C)]
struct DmaMuxRegs {
    c0cr: VolatileCell<u32>,
}

#[repr(transparent)]
pub struct VolatileCell<T>(core::cell::UnsafeCell<T>);

impl VolatileCell<u32> {
    pub fn modify(&self, f: impl FnOnce(u32) -> u32) {
        unsafe {
            let ptr = self.0.get();
            ptr.write_volatile(f(ptr.read_volatile()));
        }
    }
}
