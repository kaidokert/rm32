//! G431 input capture: TIM15 CH1 (PA2/AF9) + DMA1 Channel 1 (DMAMUX req 78).

use crate::capture_hal::{DmaOps, TimerOps, InputPinOps};
use crate::capture_generic::GenericCapture;
use crate::regs::modify as modify_reg;

const RCC: u32 = 0x4002_1000;
const DMA1_BASE: u32 = 0x4002_0000;
const DMAMUX_BASE: u32 = 0x4002_0800;
const TIM15_BASE: u32 = 0x4001_4000;
const GPIOA: u32 = 0x4800_0000;

// --- DMA1 Channel 1 (G431) ---
pub struct G431Dma;

impl DmaOps for G431Dma {
    fn disable(&self) {
        unsafe { ((DMA1_BASE + 0x08) as *mut u32).write_volatile(0); } // CCR1
    }
    fn set_mar(&self, a: u32) {
        unsafe { ((DMA1_BASE + 0x14) as *mut u32).write_volatile(a); } // CMAR1
    }
    fn set_par(&self, a: u32) {
        unsafe { ((DMA1_BASE + 0x10) as *mut u32).write_volatile(a); } // CPAR1
    }
    fn set_ndtr(&self, n: u32) {
        unsafe { ((DMA1_BASE + 0x0C) as *mut u32).write_volatile(n); } // CNDTR1
    }
    fn start_rx(&self) {
        // MINC | 32-bit PSIZE | 32-bit MSIZE | TCIE | EN
        unsafe { ((DMA1_BASE + 0x08) as *mut u32).write_volatile(0x98B); }
    }
    fn start_tx(&self) {
        // DIR | MINC | 32-bit PSIZE | 32-bit MSIZE | TCIE | EN
        unsafe { ((DMA1_BASE + 0x08) as *mut u32).write_volatile(0x99B); }
    }
}

// --- TIM15 (G431) ---
pub struct G431Timer { pub prescaler: u8 }

impl TimerOps for G431Timer {
    fn reset(&self) {
        unsafe {
            modify_reg(RCC + 0x20, |v| v | (1 << 16));   // APB2RSTR TIM15RST
            modify_reg(RCC + 0x20, |v| v & !(1 << 16));
        }
    }
    fn configure_capture(&self, _: u8) {
        unsafe {
            ((TIM15_BASE + 0x18) as *mut u32).write_volatile(0x41); // CCMR1: IC1 both edges
            ((TIM15_BASE + 0x20) as *mut u32).write_volatile(0x0A); // CCER
            ((TIM15_BASE + 0x28) as *mut u32).write_volatile(self.prescaler as u32); // PSC
            ((TIM15_BASE + 0x2C) as *mut u32).write_volatile(0xFFFF); // ARR
            ((TIM15_BASE + 0x14) as *mut u32).write_volatile(1); // EGR: UG
            ((TIM15_BASE + 0x24) as *mut u32).write_volatile(0); // CNT
        }
    }
    fn configure_output(&self, period: u16) {
        unsafe {
            ((TIM15_BASE + 0x18) as *mut u32).write_volatile(0x60); // CCMR1: PWM
            ((TIM15_BASE + 0x20) as *mut u32).write_volatile(0x03); // CCER
            ((TIM15_BASE + 0x28) as *mut u32).write_volatile(0);    // PSC
            ((TIM15_BASE + 0x2C) as *mut u32).write_volatile(period as u32); // ARR
            ((TIM15_BASE + 0x14) as *mut u32).write_volatile(1);    // EGR: UG
            modify_reg(TIM15_BASE + 0x44, |v| v | (1 << 15));      // BDTR MOE
        }
    }
    fn start(&self) {
        unsafe {
            modify_reg(TIM15_BASE + 0x0C, |v| v | (1 << 9)); // DIER CC1DE
            modify_reg(TIM15_BASE + 0x20, |v| v | 1);         // CCER CC1E
            modify_reg(TIM15_BASE + 0x00, |v| v | 1);         // CR1 CEN
        }
    }
    fn ccr_addr(&self) -> u32 {
        TIM15_BASE + 0x34 // CCR1
    }
}

// --- PA2 input pin (G431, AF9 for TIM15_CH1) ---
pub struct G431Pin;

impl InputPinOps for G431Pin {
    fn read(&self) -> bool {
        unsafe { ((GPIOA + 0x10) as *const u32).read_volatile() & (1 << 2) != 0 }
    }
    fn set_pull_up(&self) {
        unsafe { modify_reg(GPIOA + 0x0C, |v| (v & !(0b11 << 4)) | (0b01 << 4)); }
    }
    fn set_pull_down(&self) {
        unsafe { modify_reg(GPIOA + 0x0C, |v| (v & !(0b11 << 4)) | (0b10 << 4)); }
    }
    fn set_pull_none(&self) {
        unsafe { modify_reg(GPIOA + 0x0C, |v| v & !(0b11 << 4)); }
    }
}

pub type G431DshotCapture = GenericCapture<G431Dma, G431Timer, G431Pin>;

pub fn init_g431() {
    unsafe {
        // Enable clocks: TIM15 (APB2ENR bit 16), DMA1 (AHB1ENR bit 0), GPIOA (AHB2ENR bit 0)
        modify_reg(RCC + 0x60, |v| v | (1 << 16)); // APB2ENR
        modify_reg(RCC + 0x48, |v| v | (1 << 0));  // AHB1ENR
        modify_reg(RCC + 0x4C, |v| v | (1 << 0));  // AHB2ENR

        // PA2: AF9 (TIM15_CH1)
        modify_reg(GPIOA, |v| (v & !(0b11 << 4)) | (0b10 << 4)); // MODER2 = AF
        modify_reg(GPIOA + 0x20, |v| (v & !(0xF << 8)) | (9 << 8)); // AFRL2 = AF9

        // DMAMUX: CH0 (DMA CH1) → TIM15_CH1 request (78)
        ((DMAMUX_BASE) as *mut u32).write_volatile(78); // DMAMUX_C0CR
    }
}

pub fn new_capture() -> G431DshotCapture {
    GenericCapture::new(G431Dma, G431Timer { prescaler: 170 / 6 }, G431Pin)
}
