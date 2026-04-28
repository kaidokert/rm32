//! System control (IRQ, watchdog, reset) for STM32G431.

pub struct System { _private: () }

impl System {
    pub fn new() -> Self { Self { _private: () } }
}

impl rm32::hal::System for System {
    fn reset(&mut self) -> ! { cortex_m::peripheral::SCB::sys_reset() }
    fn enable_irq(&mut self) { unsafe { cortex_m::interrupt::enable() }; }
    fn disable_irq(&mut self) { cortex_m::interrupt::disable(); }
    fn start_watchdog(&mut self, prescaler: u8, reload: u16) {
        let iwdg = unsafe { &*crate::pac::IWDG::PTR };
        unsafe {
            iwdg.kr().write(|w| w.bits(0x5555));
            iwdg.pr().write(|w| w.pr().bits(prescaler));
            iwdg.rlr().write(|w| w.rl().bits(reload as u16));
            while iwdg.sr().read().bits() & 0x03 != 0 {}
            iwdg.kr().write(|w| w.bits(0xCCCC));
            iwdg.kr().write(|w| w.bits(0xAAAA));
        }
    }
    fn reload_watchdog(&mut self) {
        let iwdg = unsafe { &*crate::pac::IWDG::PTR };
        unsafe { iwdg.kr().write(|w| w.bits(0xAAAA)); }
    }
    fn delay_micros(&mut self, us: u32) { cortex_m::asm::delay(us * 170); }
    fn delay_millis(&mut self, ms: u32) { for _ in 0..ms { self.delay_micros(1000); } }
}
