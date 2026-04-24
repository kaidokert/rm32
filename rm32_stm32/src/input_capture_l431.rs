//! DMA-based input capture for DShot/Servo signal reception on STM32L431.
//!
//! Uses TIM15 CH1 (PA2, AF14) + DMA1 Channel 5 (request 7).
//! L431 has a DMA request mux (CSELR register) unlike F051's fixed mapping.

use rm32::hal::InputCapture;
use crate::periph_addr as addr;
use crate::pac::{DMA1, GPIOA, TIM15};
use crate::regs::{read as read_reg, modify as modify_reg};

const RCC_BASE: u32 = addr::RCC;

pub struct L431DshotCapture {
    buffer_size: u16,
    ic_prescaler: u8,
    out_put: bool,
    dma_buf: [u32; 64],
    gcr_buf: [u32; 37],
}

impl L431DshotCapture {
    pub fn new() -> Self {
        Self {
            buffer_size: 32,
            ic_prescaler: 80 / 6,
            out_put: false,
            dma_buf: [0; 64],
            gcr_buf: [0; 37],
        }
    }

    pub fn is_output(&self) -> bool { self.out_put }
    pub fn dma_buffer(&self) -> &[u32; 64] { &self.dma_buf }
    pub fn gcr_buffer(&mut self) -> &mut [u32; 37] { &mut self.gcr_buf }

    pub fn init(&self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        unsafe {
            // Enable clocks: TIM15 (APB2ENR bit 16), DMA1 (AHB1ENR bit 0), GPIOA (AHB2ENR bit 0)
            modify_reg(RCC_BASE + 0x60, |v| v | (1 << 16)); // APB2ENR: TIM15EN
            modify_reg(RCC_BASE + 0x48, |v| v | (1 << 0));  // AHB1ENR: DMA1EN
            modify_reg(RCC_BASE + 0x4C, |v| v | (1 << 0));  // AHB2ENR: GPIOAEN

            // PA2 as alternate function AF14 (TIM15_CH1)
            gpioa.moder.modify(|_, w| w.moder2().bits(0b10));
            // AFRL: PA2 = AF14 (bits [11:8])
            gpioa.afrl.modify(|_, w| w.afrl2().bits(14));

            // DMA CSELR: Channel 5 request = 7 (TIM15_CH1), bits [19:16]
            dma.cselr.modify(|r, w| w.bits((r.bits() & !(0xF << 16)) | (7 << 16)));
        }
    }
}

impl InputCapture for L431DshotCapture {
    fn receive_dshot_dma(&mut self) {
        let tim = unsafe { &*TIM15::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        unsafe {
            dma.ccr5.write(|w| w.bits(0)); // disable DMA

            // Reset TIM15 (APB2RSTR bit 16)
            modify_reg(RCC_BASE + 0x20, |v| v | (1 << 16));
            modify_reg(RCC_BASE + 0x20, |v| v & !(1 << 16));

            // TIM15 input capture mode
            tim.ccmr1_output().write(|w| unsafe { w.bits(0x41) });
            tim.ccer.write(|w| w.bits(0x0A));
            tim.psc.write(|w| w.bits(self.ic_prescaler as u32));
            tim.arr.write(|w| w.bits(0xFFFF));
            tim.egr.write(|w| w.bits(1));
            tim.cnt.write(|w| w.bits(0));
            self.out_put = false;

            // DMA CH5: periph→memory, 32-bit, TCIE
            dma.cmar5.write(|w| w.bits(self.dma_buf.as_ptr() as u32));
            dma.cpar5.write(|w| w.bits(tim.ccr1.as_ptr() as u32));
            dma.cndtr5.write(|w| w.bits(self.buffer_size as u32));
            dma.ccr5.write(|w| w.bits(0x98B));

            tim.dier.modify(|r, w| w.bits(r.bits() | (1 << 9))); // CC1DE
            tim.ccer.modify(|r, w| w.bits(r.bits() | 1));
            tim.cr1.modify(|r, w| w.bits(r.bits() | 1));
        }
    }

    fn send_dshot_dma(&mut self) {
        let tim = unsafe { &*TIM15::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        unsafe {
            dma.ccr5.write(|w| w.bits(0));

            modify_reg(RCC_BASE + 0x20, |v| v | (1 << 16));
            modify_reg(RCC_BASE + 0x20, |v| v & !(1 << 16));

            tim.ccmr1_output().write(|w| unsafe { w.bits(0x60) });
            tim.ccer.write(|w| w.bits(0x03));
            tim.psc.write(|w| w.bits(0));
            tim.arr.write(|w| w.bits(61));
            tim.egr.write(|w| w.bits(1));
            modify_reg(TIM15::ptr() as u32 + 0x44, |v| v | (1 << 15)); // BDTR MOE
            self.out_put = true;

            dma.cmar5.write(|w| w.bits(self.gcr_buf.as_ptr() as u32));
            dma.cpar5.write(|w| w.bits(tim.ccr1.as_ptr() as u32));
            dma.cndtr5.write(|w| w.bits(23 + self.buffer_size as u32 / 4));
            dma.ccr5.write(|w| w.bits(0x99B));

            tim.dier.modify(|r, w| w.bits(r.bits() | (1 << 9)));
            tim.ccer.modify(|r, w| w.bits(r.bits() | 1));
            tim.cr1.modify(|r, w| w.bits(r.bits() | 1));
        }
    }

    fn input_pin_state(&self) -> bool {
        let gpioa_idr = addr::GPIOA + 0x10;
        (unsafe { read_reg(gpioa_idr) }) & (1 << 2) != 0
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
