//! L431 input capture: TIM15 CH1 (PA2/AF14) + DMA1 Channel 5 (request 7).

use crate::pac::{DMA1, GPIOA, RCC, TIM15};
use crate::capture_hal::{DmaOps, TimerOps, InputPinOps};
use crate::capture_generic::GenericCapture;

// --- DMA1 Channel 5 (L431, flat registers) ---
pub struct L431Dma;

impl DmaOps for L431Dma {
    fn disable(&self) { unsafe { &*DMA1::ptr() }.ccr5.write(|w| unsafe { w.bits(0) }); }
    fn set_mar(&self, a: u32) { unsafe { &*DMA1::ptr() }.cmar5.write(|w| unsafe { w.bits(a) }); }
    fn set_par(&self, a: u32) { unsafe { &*DMA1::ptr() }.cpar5.write(|w| unsafe { w.bits(a) }); }
    fn set_ndtr(&self, n: u32) { unsafe { &*DMA1::ptr() }.cndtr5.write(|w| unsafe { w.bits(n) }); }
    fn start_rx(&self) {
        unsafe { &*DMA1::ptr() }.ccr5.write(|w| unsafe { w.bits(0x98B) });
    }
    fn start_tx(&self) {
        unsafe { &*DMA1::ptr() }.ccr5.write(|w| unsafe { w.bits(0x99B) });
    }
}

// --- TIM15 (L431) ---
pub struct L431Timer { pub prescaler: u8 }

impl TimerOps for L431Timer {
    fn reset(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apb2rstr.modify(|_, w| w.tim15rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim15rst().clear_bit());
    }
    fn configure_capture(&self, _: u8) {
        let tim = unsafe { &*TIM15::ptr() };
        tim.ccmr1_output().write(|w| unsafe { w.bits(0x41) });
        tim.ccer.write(|w| unsafe { w.bits(0x0A) });
        tim.psc.write(|w| unsafe { w.bits(self.prescaler as u32) });
        tim.arr.write(|w| unsafe { w.bits(0xFFFF) });
        tim.egr.write(|w| unsafe { w.bits(1) });
        tim.cnt.write(|w| unsafe { w.bits(0) });
    }
    fn configure_output(&self, period: u16) {
        let tim = unsafe { &*TIM15::ptr() };
        tim.ccmr1_output().write(|w| unsafe { w.bits(0x60) });
        tim.ccer.write(|w| unsafe { w.bits(0x03) });
        tim.psc.write(|w| unsafe { w.bits(0) });
        tim.arr.write(|w| unsafe { w.bits(period as u32) });
        tim.egr.write(|w| unsafe { w.bits(1) });
        tim.bdtr.modify(|_, w| w.moe().set_bit());
    }
    fn start(&self) {
        let tim = unsafe { &*TIM15::ptr() };
        tim.dier.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 9)) });
        tim.ccer.modify(|r, w| unsafe { w.bits(r.bits() | 1) });
        tim.cr1.modify(|r, w| unsafe { w.bits(r.bits() | 1) });
    }
    fn ccr_addr(&self) -> u32 {
        let tim = unsafe { &*TIM15::ptr() };
        tim.ccr1.as_ptr() as u32
    }
}

// --- PA2 input pin (L431, AF14) ---
pub struct L431Pin;

impl InputPinOps for L431Pin {
    fn read(&self) -> bool {
        unsafe { &*GPIOA::ptr() }.idr.read().idr2().bit()
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

pub type L431DshotCapture = GenericCapture<L431Dma, L431Timer, L431Pin>;

pub fn init_l431() {
    let rcc = unsafe { &*RCC::ptr() };
    let dma = unsafe { &*DMA1::ptr() };
    let gpioa = unsafe { &*GPIOA::ptr() };
    unsafe {
        rcc.apb2enr.modify(|_, w| w.tim15en().set_bit());
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
        rcc.ahb2enr.modify(|_, w| w.gpioaen().set_bit());
    }
    gpioa.moder.modify(|_, w| w.moder2().bits(0b10));
    gpioa.afrl.modify(|_, w| w.afrl2().bits(14));
    dma.cselr.modify(|r, w| unsafe { w.bits((r.bits() & !(0xF << 16)) | (7 << 16)) });
}

pub fn new_capture() -> L431DshotCapture {
    GenericCapture::new(L431Dma, L431Timer { prescaler: 80 / 6 }, L431Pin)
}
