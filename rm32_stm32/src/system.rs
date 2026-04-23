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
