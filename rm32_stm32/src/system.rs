//! System control (IRQ, watchdog, reset).

use stm32g0xx_hal::watchdog::{IndependedWatchdog, IWDGExt};
use stm32g0xx_hal::stm32::IWDG;
use rm32::hal::System;

pub struct SystemControl {
    wdg: IndependedWatchdog,
}

impl SystemControl {
    pub fn new(iwdg: IWDG) -> Self {
        Self {
            wdg: iwdg.constrain(),
        }
    }
}

impl System for SystemControl {
    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }

    fn enable_irq(&mut self) {
        unsafe { cortex_m::interrupt::enable() };
    }

    fn disable_irq(&mut self) {
        cortex_m::interrupt::disable();
    }

    fn start_watchdog(&mut self, prescaler: u8, reload: u16) {
        // IWDG registers are the same address on all STM32
        const IWDG: u32 = 0x4000_3000;
        unsafe {
            (IWDG as *mut u32).write_volatile(0x5555); // unlock
            ((IWDG + 4) as *mut u32).write_volatile(prescaler as u32);
            ((IWDG + 8) as *mut u32).write_volatile(reload as u32);
            while ((IWDG + 0x0C) as *const u32).read_volatile() & 0x03 != 0 {}
            (IWDG as *mut u32).write_volatile(0xCCCC); // start
            (IWDG as *mut u32).write_volatile(0xAAAA); // reload
        }
    }

    fn reload_watchdog(&mut self) {
        self.wdg.feed();
    }

    fn delay_micros(&mut self, us: u32) {
        cortex_m::asm::delay(us * 64); // ~64 cycles per µs at 64MHz
    }

    fn delay_millis(&mut self, ms: u32) {
        for _ in 0..ms {
            self.delay_micros(1000);
        }
    }
}
