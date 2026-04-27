//! G431 input capture: TIM15 CH1 (PA2/AF9) + DMA1 Channel 1 (DMAMUX req 78).

use crate::capture_hal::{DmaOps, TimerOps, InputPinOps};
use crate::capture_generic::GenericCapture;
use stm32g4::stm32g431 as pac;

// --- DMA1 Channel 1 (G431) ---
pub struct G431Dma;

impl DmaOps for G431Dma {
    fn disable(&self) {
        let ch = unsafe { &*pac::DMA1::PTR }.ch1();
        unsafe { ch.cr().write(|w| w.bits(0)); }
    }
    fn set_mar(&self, a: u32) {
        let ch = unsafe { &*pac::DMA1::PTR }.ch1();
        unsafe { ch.mar().write(|w| w.bits(a)); }
    }
    fn set_par(&self, a: u32) {
        let ch = unsafe { &*pac::DMA1::PTR }.ch1();
        unsafe { ch.par().write(|w| w.bits(a)); }
    }
    fn set_ndtr(&self, n: u32) {
        let ch = unsafe { &*pac::DMA1::PTR }.ch1();
        unsafe { ch.ndtr().write(|w| w.bits(n)); }
    }
    fn start_rx(&self) {
        let ch = unsafe { &*pac::DMA1::PTR }.ch1();
        unsafe { ch.cr().write(|w| w.bits(0x98B)); }
    }
    fn start_tx(&self) {
        let ch = unsafe { &*pac::DMA1::PTR }.ch1();
        unsafe { ch.cr().write(|w| w.bits(0x99B)); }
    }
}

// --- TIM15 (G431) ---
pub struct G431Timer { pub prescaler: u8 }

impl TimerOps for G431Timer {
    fn reset(&self) {
        let rcc = unsafe { &*pac::RCC::PTR };
        unsafe {
            rcc.apb2rstr().modify(|_, w| w.tim15rst().set_bit());
            rcc.apb2rstr().modify(|_, w| w.tim15rst().clear_bit());
        }
    }
    fn configure_capture(&self, _: u8) {
        let tim = unsafe { &*pac::TIM15::PTR };
        unsafe {
            tim.ccmr1_input().write(|w| w.bits(0x41));
            tim.ccer().write(|w| w.bits(0x0A));
            tim.psc().write(|w| w.psc().bits(self.prescaler as u16));
            tim.arr().write(|w| w.arr().bits(0xFFFF));
            tim.egr().write(|w| w.ug().set_bit());
            tim.cnt().write(|w| w.cnt().bits(0));
        }
    }
    fn configure_output(&self, period: u16) {
        let tim = unsafe { &*pac::TIM15::PTR };
        unsafe {
            tim.ccmr1_output().write(|w| w.bits(0x60));
            tim.ccer().write(|w| w.bits(0x03));
            tim.psc().write(|w| w.psc().bits(0));
            tim.arr().write(|w| w.arr().bits(period as u32));
            tim.egr().write(|w| w.ug().set_bit());
            tim.bdtr().modify(|_, w| w.moe().set_bit());
        }
    }
    fn start(&self) {
        let tim = unsafe { &*pac::TIM15::PTR };
        unsafe {
            tim.dier().modify(|r, w| w.bits(r.bits() | (1 << 9))); // CC1DE
            tim.ccer().modify(|r, w| w.bits(r.bits() | 1));         // CC1E
            tim.cr1().modify(|_, w| w.cen().set_bit());
        }
    }
    fn ccr_addr(&self) -> u32 {
        let tim = unsafe { &*pac::TIM15::PTR };
        tim.ccr1().as_ptr() as u32
    }
}

// --- PA2 input pin (G431, AF9 for TIM15_CH1) ---
pub struct G431Pin;

impl InputPinOps for G431Pin {
    fn read(&self) -> bool {
        let gpioa = unsafe { &*pac::GPIOA::PTR };
        gpioa.idr().read().idr2().bit()
    }
    fn set_pull_up(&self) {
        let gpioa = unsafe { &*pac::GPIOA::PTR };
        unsafe { gpioa.pupdr().modify(|_, w| w.pupdr2().bits(0b01)); }
    }
    fn set_pull_down(&self) {
        let gpioa = unsafe { &*pac::GPIOA::PTR };
        unsafe { gpioa.pupdr().modify(|_, w| w.pupdr2().bits(0b10)); }
    }
    fn set_pull_none(&self) {
        let gpioa = unsafe { &*pac::GPIOA::PTR };
        unsafe { gpioa.pupdr().modify(|_, w| w.pupdr2().bits(0b00)); }
    }
}

pub type G431DshotCapture = GenericCapture<G431Dma, G431Timer, G431Pin>;

pub fn init_g431() {
    let rcc = unsafe { &*pac::RCC::PTR };
    let gpioa = unsafe { &*pac::GPIOA::PTR };
    let dmamux = unsafe { &*pac::DMAMUX::PTR };

    unsafe {
        // Enable clocks
        rcc.apb2enr().modify(|_, w| w.tim15en().set_bit());
        rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());
        rcc.ahb2enr().modify(|_, w| w.gpioaen().set_bit());

        // PA2: AF9 (TIM15_CH1)
        gpioa.moder().modify(|_, w| w.moder2().bits(0b10));
        gpioa.afrl().modify(|_, w| w.afrl2().bits(9));

        // DMAMUX: CH0 (DMA CH1) → TIM15_CH1 request (78)
        dmamux.ccr(0).write(|w| w.dmareq_id().bits(78));
    }
}

pub fn new_capture() -> G431DshotCapture {
    GenericCapture::new(G431Dma, G431Timer { prescaler: 170 / 6 }, G431Pin)
}
