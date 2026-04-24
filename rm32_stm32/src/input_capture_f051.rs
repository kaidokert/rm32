//! DMA-based input capture for DShot/Servo signal reception on STM32F051.
//!
//! Uses TIM15 CH1 (PA2) + DMA1 Channel 5 to capture pulse widths.
//! F051 has no DMAMUX — DMA channel assignments are fixed.
//!
//! For SISKIN_F051:
//!   Input pin: PA2 (TIM15_CH1, AF0)
//!   DMA: DMA1_Channel5
//!   Timer: TIM15

use rm32::hal::InputCapture;
use crate::periph_addr as addr;
use crate::pac::{DMA1, TIM15, GPIOA};
use crate::regs::modify as modify_reg;

const RCC_BASE: u32 = addr::RCC;

pub struct F051DshotCapture {
    buffer_size: u16,
    ic_prescaler: u8,
    out_put: bool,
    /// DMA capture buffer — owned by struct, address passed to DMA hardware.
    dma_buf: [u32; 64],
    /// GCR telemetry response buffer — owned by struct.
    gcr_buf: [u32; 37],
}

impl F051DshotCapture {
    pub fn new() -> Self {
        Self {
            buffer_size: 32,
            ic_prescaler: 48 / 6,
            out_put: false,
            dma_buf: [0; 64],
            gcr_buf: [0; 37],
        }
    }

    pub fn is_output(&self) -> bool { self.out_put }

    /// Borrow the DMA buffer for frame decoding.
    /// Safe: only called from ISR context after DMA transfer completes.
    pub fn dma_buffer(&self) -> &[u32; 64] { &self.dma_buf }

    /// Mutably borrow the GCR buffer for encoding telemetry.
    /// Safe: only called from ISR context when DMA is not active on this buffer.
    pub fn gcr_buffer(&mut self) -> &mut [u32; 37] { &mut self.gcr_buf }

    pub fn init(&self) {
        unsafe {
            let apb2enr = (RCC_BASE + 0x18) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 16));
            let ahbenr = (RCC_BASE + 0x14) as *mut u32;
            ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0) | (1 << 17));

            let gpioa = &*GPIOA::ptr();
            gpioa.moder.modify(|_, w| w.moder2().alternate());
            gpioa.afrl.modify(|_, w| w.afrl2().bits(0));
        }
    }
}

impl InputCapture for F051DshotCapture {
    fn receive_dshot_dma(&mut self) {
        unsafe {
            let dma = &*DMA1::ptr();
            dma.ch5.cr.write(|w| w.en().disabled());

            let apb2rstr = (RCC_BASE + 0x0C) as *mut u32;
            modify_reg(apb2rstr as u32, |v| v | (1 << 16));
            modify_reg(apb2rstr as u32, |v| v & !(1 << 16));

            let tim = &*TIM15::ptr();
            tim.ccmr1_input().write(|w| w.bits(0x41));
            tim.ccer.write(|w| w.bits(0x0A));
            tim.psc.write(|w| w.bits(self.ic_prescaler as u32));
            tim.arr.write(|w| w.bits(0xFFFF));
            tim.egr.write(|w| w.ug().set_bit());
            tim.cnt.write(|w| w.bits(0));
            self.out_put = false;

            // DMA address points to struct-owned buffer
            dma.ch5.mar.write(|w| w.bits(self.dma_buf.as_ptr() as u32));
            dma.ch5.par.write(|w| w.bits(&tim.ccr1 as *const _ as u32));
            dma.ch5.ndtr.write(|w| w.bits(self.buffer_size as u32));
            dma.ch5.cr.write(|w| {
                w.tcie().enabled()
                 .minc().enabled()
                 .psize().bits32()
                 .msize().bits32()
                 .en().enabled()
            });

            tim.dier.modify(|_, w| w.cc1de().set_bit());
            tim.ccer.modify(|_, w| w.cc1e().set_bit());
            tim.cr1.modify(|_, w| w.cen().set_bit());
        }
    }

    fn send_dshot_dma(&mut self) {
        unsafe {
            let dma = &*DMA1::ptr();
            dma.ch5.cr.write(|w| w.en().disabled());

            let apb2rstr = (RCC_BASE + 0x0C) as *mut u32;
            modify_reg(apb2rstr as u32, |v| v | (1 << 16));
            modify_reg(apb2rstr as u32, |v| v & !(1 << 16));

            let tim = &*TIM15::ptr();
            tim.ccmr1_output().write(|w| w.bits(0x60));
            tim.ccer.write(|w| w.bits(0x03));
            tim.psc.write(|w| w.bits(0));
            tim.arr.write(|w| w.bits(61));
            tim.egr.write(|w| w.ug().set_bit());
            tim.bdtr.modify(|_, w| w.moe().set_bit());
            self.out_put = true;

            // DMA address points to struct-owned GCR buffer
            dma.ch5.mar.write(|w| w.bits(self.gcr_buf.as_ptr() as u32));
            dma.ch5.par.write(|w| w.bits(&tim.ccr1 as *const _ as u32));
            dma.ch5.ndtr.write(|w| w.bits(23 + self.buffer_size as u32 / 4));
            dma.ch5.cr.write(|w| {
                w.dir().from_memory()
                 .tcie().enabled()
                 .minc().enabled()
                 .psize().bits32()
                 .msize().bits32()
                 .en().enabled()
            });

            tim.dier.modify(|_, w| w.cc1de().set_bit());
            tim.ccer.modify(|_, w| w.cc1e().set_bit());
            tim.cr1.modify(|_, w| w.cen().set_bit());
        }
    }

    fn input_pin_state(&self) -> bool {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.idr.read().idr2().bit()
    }

    fn set_pull_up(&mut self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(0b01) });
    }

    fn set_pull_down(&mut self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(0b10) });
    }

    fn set_pull_none(&mut self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(0b00) });
    }
}
