//! F051 input capture: TIM15 CH1 (PA2/AF0) + DMA1 Channel 5.

use crate::pac::{DMA1, GPIOA, TIM15};
use crate::capture_hal::{DmaOps, TimerOps, InputPinOps};
use crate::capture_generic::GenericCapture;
use crate::regs::modify as modify_reg;
use crate::periph_addr as addr;

fn rcc_base() -> u32 { addr::rcc() }

// --- DMA1 Channel 5 (F051, fixed assignment) ---
pub struct F051Dma;

impl DmaOps for F051Dma {
    fn disable(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch5.cr.write(|w| w.en().disabled());
    }
    fn set_mar(&self, a: u32) { unsafe { &*DMA1::ptr() }.ch5.mar.write(|w| unsafe { w.bits(a) }); }
    fn set_par(&self, a: u32) { unsafe { &*DMA1::ptr() }.ch5.par.write(|w| unsafe { w.bits(a) }); }
    fn set_ndtr(&self, n: u32) { unsafe { &*DMA1::ptr() }.ch5.ndtr.write(|w| unsafe { w.bits(n) }); }
    fn start_rx(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch5.cr.write(|w| {
            w.tcie().enabled().minc().enabled()
             .psize().bits32().msize().bits32().en().enabled()
        });
    }
    fn start_tx(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch5.cr.write(|w| {
            w.dir().from_memory().tcie().enabled().minc().enabled()
             .psize().bits32().msize().bits32().en().enabled()
        });
    }
}

// --- TIM15 (F051) ---
pub struct F051Timer { pub prescaler: u8 }

impl TimerOps for F051Timer {
    fn reset(&self) {
        unsafe {
            modify_reg(rcc_base() + 0x0C, |v| v | (1 << 16));
            modify_reg(rcc_base() + 0x0C, |v| v & !(1 << 16));
        }
    }
    fn configure_capture(&self, _: u8) {
        let tim = unsafe { &*TIM15::ptr() };
        tim.ccmr1_input().write(|w| unsafe { w.bits(0x41) });
        tim.ccer.write(|w| unsafe { w.bits(0x0A) });
        tim.psc.write(|w| unsafe { w.bits(self.prescaler as u32) });
        tim.arr.write(|w| unsafe { w.bits(0xFFFF) });
        tim.egr.write(|w| w.ug().set_bit());
        tim.cnt.write(|w| unsafe { w.bits(0) });
    }
    fn configure_output(&self, period: u16) {
        let tim = unsafe { &*TIM15::ptr() };
        tim.ccmr1_output().write(|w| unsafe { w.bits(0x60) });
        tim.ccer.write(|w| unsafe { w.bits(0x03) });
        tim.psc.write(|w| unsafe { w.bits(0) });
        tim.arr.write(|w| unsafe { w.bits(period as u32) });
        tim.egr.write(|w| w.ug().set_bit());
        tim.bdtr.modify(|_, w| w.moe().set_bit());
    }
    fn start(&self) {
        let tim = unsafe { &*TIM15::ptr() };
        tim.dier.modify(|_, w| w.cc1de().set_bit());
        tim.ccer.modify(|_, w| w.cc1e().set_bit());
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }
    fn ccr_addr(&self) -> u32 {
        let tim = unsafe { &*TIM15::ptr() };
        &tim.ccr1 as *const _ as u32
    }
}

// --- PA2 input pin (F051) ---
pub struct F051Pin;

impl InputPinOps for F051Pin {
    fn read(&self) -> bool {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.idr.read().idr2().bit()
    }
    fn set_pull_up(&self) {
        unsafe { &*GPIOA::ptr() }.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(0b01) });
    }
    fn set_pull_down(&self) {
        unsafe { &*GPIOA::ptr() }.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(0b10) });
    }
    fn set_pull_none(&self) {
        unsafe { &*GPIOA::ptr() }.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(0b00) });
    }
}

pub type F051DshotCapture = GenericCapture<F051Dma, F051Timer, F051Pin>;

pub fn init_f051() {
    unsafe {
        let apb2enr = (rcc_base() + 0x18) as *mut u32;
        apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 16));
        let ahbenr = (rcc_base() + 0x14) as *mut u32;
        ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0) | (1 << 17));

        let gpioa = &*GPIOA::ptr();
        gpioa.moder.modify(|_, w| w.moder2().alternate());
        gpioa.afrl.modify(|_, w| w.afrl2().bits(0));
    }
}

pub fn new_capture() -> F051DshotCapture {
    GenericCapture::new(F051Dma, F051Timer { prescaler: 48 / 6 }, F051Pin)
}
