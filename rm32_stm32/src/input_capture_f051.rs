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

/// Static DMA buffer — written by DMA, read by ISR.
static mut DMA_BUFFER: [u32; 64] = [0; 64];

/// Static GCR telemetry response buffer for bidir DShot TX.
static mut GCR_BUFFER: [u32; 37] = [0; 37];

/// Get a reference to the DMA buffer for frame decoding in ISR.
///
/// # Safety
/// Only call from ISR context (DMA is disabled when reading).
pub unsafe fn dma_buffer() -> &'static [u32; 64] {
    &DMA_BUFFER
}

/// Get a mutable reference to the GCR buffer for encoding telemetry response.
///
/// # Safety
/// Only call from ISR context when DMA is not active on this buffer.
pub unsafe fn gcr_buffer() -> &'static mut [u32; 37] {
    &mut GCR_BUFFER
}

// F051 register base addresses
const RCC_BASE: u32 = 0x4002_1000;
const DMA1_BASE: u32 = 0x4002_0000;
const TIM15_BASE: u32 = 0x4001_4000;
const GPIOA_BASE: u32 = 0x4800_0000;

// DMA1 Channel 5 register offsets (CH5 = base + 0x44 + n*0x14 where n=4)
const DMA_CH5_CCR: u32 = DMA1_BASE + 0x58;
const DMA_CH5_CNDTR: u32 = DMA1_BASE + 0x5C;
const DMA_CH5_CPAR: u32 = DMA1_BASE + 0x60;
const DMA_CH5_CMAR: u32 = DMA1_BASE + 0x64;

// Timer register offsets
const CR1: u32 = 0x00;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;

#[inline(always)]
unsafe fn write_reg(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }
#[inline(always)]
unsafe fn read_reg(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
#[inline(always)]
unsafe fn modify_reg(addr: u32, f: impl FnOnce(u32) -> u32) {
    let ptr = addr as *mut u32;
    ptr.write_volatile(f(ptr.read_volatile()));
}

pub struct F051DshotCapture {
    buffer_size: u16,
    ic_prescaler: u8,
    out_put: bool,
}

impl F051DshotCapture {
    pub fn new() -> Self {
        Self {
            buffer_size: 32,
            ic_prescaler: 48 / 6, // CPU_FREQUENCY_MHZ / 6 = 8
            out_put: false,
        }
    }

    pub fn is_output(&self) -> bool {
        self.out_put
    }

    /// Initialize TIM15 + DMA1_CH5 for input capture.
    pub fn init(&self) {
        unsafe {
            // Enable clocks: TIM15 (APB2ENR bit 16), DMA1 (AHBENR bit 0), GPIOA (AHBENR bit 17)
            let apb2enr = (RCC_BASE + 0x18) as *mut u32;
            apb2enr.write_volatile(apb2enr.read_volatile() | (1 << 16)); // TIM15EN

            let ahbenr = (RCC_BASE + 0x14) as *mut u32;
            ahbenr.write_volatile(ahbenr.read_volatile() | (1 << 0) | (1 << 17)); // DMA1EN, GPIOAEN

            // PA2 as alternate function (AF0 = TIM15_CH1)
            let moder = GPIOA_BASE as *mut u32;
            modify_reg(moder as u32, |v| (v & !(0b11 << 4)) | (0b10 << 4)); // PA2 = AF

            // AFRL: PA2 = AF0 (bits [11:8])
            let afrl = (GPIOA_BASE + 0x20) as *mut u32;
            modify_reg(afrl as u32, |v| v & !(0xF << 8)); // AF0 = 0
        }
    }

    fn receive_impl(&mut self) {
        unsafe {
            // Disable DMA channel first
            write_reg(DMA_CH5_CCR, 0);

            // Reset TIM15 via APB2RSTR bit 16
            let apb2rstr = (RCC_BASE + 0x0C) as *mut u32;
            modify_reg(apb2rstr as u32, |v| v | (1 << 16));
            modify_reg(apb2rstr as u32, |v| v & !(1 << 16));

            // TIM15 input capture mode
            write_reg(TIM15_BASE + CCMR1, 0x41); // IC1 mapped to TI1, filter=4
            write_reg(TIM15_BASE + CCER, 0x0A);  // Capture on both edges
            write_reg(TIM15_BASE + PSC, self.ic_prescaler as u32);
            write_reg(TIM15_BASE + ARR, 0xFFFF);
            write_reg(TIM15_BASE + EGR, 1); // UG
            write_reg(TIM15_BASE + CNT, 0);

            self.out_put = false;

            // DMA1 CH5: periph→memory, 32-bit, memory increment, TC interrupt
            write_reg(DMA_CH5_CMAR, DMA_BUFFER.as_ptr() as u32);
            write_reg(DMA_CH5_CPAR, (TIM15_BASE + CCR1) as u32);
            write_reg(DMA_CH5_CNDTR, self.buffer_size as u32);
            // 0x98B: TCIE | MINC | PSIZE=32 | MSIZE=32 | DIR=0 (periph→mem) | EN
            write_reg(DMA_CH5_CCR, 0x98B);

            // Enable DMA request from TIM15 CC1
            modify_reg(TIM15_BASE + DIER, |v| v | (1 << 9)); // CC1DE
            modify_reg(TIM15_BASE + CCER, |v| v | 1); // CC1E
            modify_reg(TIM15_BASE + CR1, |v| v | 1); // CEN
        }
    }
}

impl InputCapture for F051DshotCapture {
    fn receive_dshot_dma(&mut self) {
        self.receive_impl();
    }

    fn send_dshot_dma(&mut self) {
        unsafe {
            // Disable DMA
            write_reg(DMA_CH5_CCR, 0);

            // Reset TIM15
            let apb2rstr = (RCC_BASE + 0x0C) as *mut u32;
            modify_reg(apb2rstr as u32, |v| v | (1 << 16));
            modify_reg(apb2rstr as u32, |v| v & !(1 << 16));

            // PWM output mode
            write_reg(TIM15_BASE + CCMR1, 0x60); // PWM mode 1
            write_reg(TIM15_BASE + CCER, 0x03);  // Output enable
            write_reg(TIM15_BASE + PSC, 0);
            write_reg(TIM15_BASE + ARR, 61); // Bit period
            write_reg(TIM15_BASE + EGR, 1);
            // Enable MOE (main output enable) for TIM15 — BDTR at offset 0x44
            modify_reg(TIM15_BASE + 0x44, |v| v | (1 << 15));
            self.out_put = true;

            // DMA: memory→periph, GCR buffer → CCR1
            write_reg(DMA_CH5_CMAR, GCR_BUFFER.as_ptr() as u32);
            write_reg(DMA_CH5_CPAR, (TIM15_BASE + CCR1) as u32);
            write_reg(DMA_CH5_CNDTR, 23 + self.buffer_size as u32 / 4);
            // 0x99B: DIR=1 (mem→periph) | MINC | PSIZE=32 | MSIZE=32 | TCIE | EN
            write_reg(DMA_CH5_CCR, 0x99B);

            // Enable DMA request, output, start
            modify_reg(TIM15_BASE + DIER, |v| v | (1 << 9)); // CC1DE
            modify_reg(TIM15_BASE + CCER, |v| v | 1); // CC1E
            modify_reg(TIM15_BASE + CR1, |v| v | 1); // CEN
        }
    }

    fn input_pin_state(&self) -> bool {
        // PA2 IDR
        (unsafe { read_reg(GPIOA_BASE + 0x10) }) & (1 << 2) != 0
    }

    fn set_pull_up(&mut self) {
        unsafe { modify_reg(GPIOA_BASE + 0x0C, |v| (v & !(0b11 << 4)) | (0b01 << 4)); }
    }

    fn set_pull_down(&mut self) {
        unsafe { modify_reg(GPIOA_BASE + 0x0C, |v| (v & !(0b11 << 4)) | (0b10 << 4)); }
    }

    fn set_pull_none(&mut self) {
        unsafe { modify_reg(GPIOA_BASE + 0x0C, |v| v & !(0b11 << 4)); }
    }
}
