//! System control (IRQ, watchdog, reset) for STM32L431.

/// Snapshot reset-cause flags and clear them. Sticky in RCC_CSR (RM0394
/// §6.4.27) across resets; AM32 bootloader doesn't clear them. Calling this
/// once early in main reflects the *current* boot's cause; after the call,
/// flags are cleared so the next reset reflects only its own cause.
pub fn read_and_clear_reset_cause() -> rm32::reset_cause::ResetCause {
    use rm32::reset_cause::ResetCause;
    let rcc = unsafe { &*crate::pac::RCC::PTR };
    let csr = rcc.csr.read();
    let mut r = ResetCause::empty();
    if csr.lpwrstf().bit_is_set() {
        r |= ResetCause::LOW_POWER;
    }
    if csr.wwdgrstf().bit_is_set() {
        r |= ResetCause::WINDOW_WATCHDOG;
    }
    if csr.iwdgrstf().bit_is_set() {
        r |= ResetCause::INDEP_WATCHDOG;
    }
    if csr.sftrstf().bit_is_set() {
        r |= ResetCause::SOFTWARE;
    }
    if csr.borrstf().bit_is_set() {
        r |= ResetCause::BROWNOUT;
    }
    if csr.pinrstf().bit_is_set() {
        r |= ResetCause::PIN;
    }
    if csr.oblrstf().bit_is_set() {
        r |= ResetCause::OPTION_BYTE;
    }
    if csr.firewallrstf().bit_is_set() {
        r |= ResetCause::FIREWALL;
    }
    // RMVF=1 commands the hardware to clear bits 31..24. modify() preserves
    // LSE/LSI control bits in the low half via read-modify-write.
    rcc.csr.modify(|_, w| w.rmvf().set_bit());
    r
}

pub struct System {
    _private: (),
}

impl System {
    pub fn new() -> Self {
        Self { _private: () }
    }
}

impl rm32::hal::System for System {
    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
    fn enable_irq(&mut self) {
        unsafe { cortex_m::interrupt::enable() };
    }
    fn disable_irq(&mut self) {
        cortex_m::interrupt::disable();
    }
    fn irqs_enabled(&self) -> bool {
        cortex_m::register::primask::read().is_active()
    }
    fn start_watchdog(&mut self, prescaler: u8, reload: u16) {
        // L431 default option byte is IWDG_SW=1 (software start). LSI is not
        // running after reset, and the SR busy-wait below would hang forever
        // unless LSI is up. Enable LSI explicitly first.
        let rcc = unsafe { &*crate::pac::RCC::PTR };
        rcc.csr.modify(|_, w| w.lsion().set_bit());
        while rcc.csr.read().lsirdy().bit_is_clear() {}

        let iwdg = unsafe { &*crate::pac::IWDG::PTR };
        unsafe {
            // Activate IWDG first — STM HAL pattern. Without IWDG running,
            // SR.PVU/RVU would never clear after PR/RLR writes.
            iwdg.kr.write(|w| w.bits(0xCCCC));
            iwdg.kr.write(|w| w.bits(0x5555));
            iwdg.pr.write(|w| w.bits(prescaler as u32));
            iwdg.rlr.write(|w| w.bits(reload as u32));
            while iwdg.sr.read().bits() & 0x03 != 0 {}
            iwdg.kr.write(|w| w.bits(0xAAAA));
        }
    }
    fn reload_watchdog(&mut self) {
        let iwdg = unsafe { &*crate::pac::IWDG::PTR };
        unsafe {
            iwdg.kr.write(|w| w.bits(0xAAAA));
        }
    }
    fn delay_micros(&mut self, us: u32) {
        crate::delay::delay_us(us);
    }
    fn delay_millis(&mut self, ms: u32) {
        crate::delay::delay_ms(ms);
    }
}
