//! G071 input capture: TIM3 CH1 (PB4) + DMA1 Channel 1 + DMAMUX.

use crate::pac::{DMA1, DMAMUX, GPIOB, RCC, TIM3};
use crate::capture_hal::{DmaOps, TimerOps, InputPinOps};
use crate::capture_generic::GenericCapture;

// --- DMA1 Channel 1 (G071) ---
pub struct G071Dma;

impl DmaOps for G071Dma {
    fn disable(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch1().cr().write(|w| w.en().clear_bit());
    }
    fn set_mar(&self, addr: u32) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch1().mar().write(|w| unsafe { w.bits(addr) });
    }
    fn set_par(&self, addr: u32) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch1().par().write(|w| unsafe { w.bits(addr) });
    }
    fn set_ndtr(&self, count: u32) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch1().ndtr().write(|w| unsafe { w.bits(count) });
    }
    fn start_rx(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch1().cr().write(|w| unsafe {
            w.tcie().set_bit().minc().set_bit()
             .psize().bits(0b10).msize().bits(0b10).en().set_bit()
        });
    }
    fn start_tx(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        dma.ch1().cr().write(|w| unsafe {
            w.dir().set_bit().tcie().set_bit().minc().set_bit()
             .psize().bits(0b10).msize().bits(0b10).en().set_bit()
        });
    }
}

// --- TIM3 (G071) ---
pub struct G071Timer { pub prescaler: u8 }

impl TimerOps for G071Timer {
    fn reset(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apbrstr1().modify(|_, w| w.tim3rst().set_bit());
        rcc.apbrstr1().modify(|_, w| w.tim3rst().clear_bit());
    }
    fn configure_capture(&self, _: u8) {
        let tim = unsafe { &*TIM3::ptr() };
        tim.ccmr1_output().write(|w| unsafe { w.bits(0x41) });
        tim.ccer().write(|w| unsafe { w.bits(0x0A) });
        tim.psc().write(|w| unsafe { w.bits(self.prescaler as u32) });
        tim.arr().write(|w| unsafe { w.bits(0xFFFF) });
        tim.egr().write(|w| w.ug().set_bit());
        tim.cnt().write(|w| unsafe { w.bits(0) });
    }
    fn configure_output(&self, period: u16) {
        let tim = unsafe { &*TIM3::ptr() };
        tim.ccmr1_output().write(|w| unsafe { w.bits(0x60) });
        tim.ccer().write(|w| unsafe { w.bits(0x03) });
        tim.psc().write(|w| unsafe { w.bits(0) });
        tim.arr().write(|w| unsafe { w.bits(period as u32) });
        tim.egr().write(|w| w.ug().set_bit());
    }
    fn start(&self) {
        let tim = unsafe { &*TIM3::ptr() };
        tim.dier().modify(|_, w| w.cc1de().set_bit());
        tim.ccer().modify(|_, w| w.cc1e().set_bit());
        tim.cr1().modify(|_, w| w.cen().set_bit());
    }
    fn ccr_addr(&self) -> u32 {
        let tim = unsafe { &*TIM3::ptr() };
        tim.ccr1().as_ptr() as u32
    }
}

// --- PB4 input pin (G071) ---
pub struct G071Pin;

impl InputPinOps for G071Pin {
    fn read(&self) -> bool {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.idr().read().idr4().bit()
    }
    fn set_pull_up(&self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.pupdr().modify(|_, w| w.pupdr4().pull_up());
    }
    fn set_pull_down(&self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.pupdr().modify(|_, w| w.pupdr4().pull_down());
    }
    fn set_pull_none(&self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.pupdr().modify(|_, w| w.pupdr4().floating());
    }
}

/// G071 DShot capture type alias.
pub type DshotCapture = GenericCapture<G071Dma, G071Timer, G071Pin>;

/// Initialize G071 input capture hardware (clocks, GPIO, DMAMUX).
pub fn init_g071() {
    let rcc = unsafe { &*RCC::ptr() };
    let gpiob = unsafe { &*GPIOB::ptr() };
    let dmamux = unsafe { &*DMAMUX::ptr() };

    rcc.apbenr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 1)) });
    rcc.ahbenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });
    rcc.iopenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 1)) });

    gpiob.moder().modify(|_, w| w.moder4().alternate());
    gpiob.afrl().modify(|_, w| w.afr(4).af1());

    dmamux.ccr(0).modify(|r, w| unsafe { w.bits((r.bits() & !0x3F) | 32) });
}

/// Create a new G071 DshotCapture instance.
pub fn new_capture() -> DshotCapture {
    GenericCapture::new(
        G071Dma,
        G071Timer { prescaler: 64 / 6 },
        G071Pin,
    )
}
