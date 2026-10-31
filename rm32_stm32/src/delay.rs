//! Busy-wait delays on the free-running SysTick (core clock, 24-bit down
//! counter, interrupt off).
//!
//! AM32 `delayMicros` (`Src/functions.c`) waits on a 1 us utility timer.
//! `cortex_m::asm::delay(n)` spins `n` loop passes of about 3 (M4) to 4 (M0)
//! cycles, so the former `asm::delay(us * MHz)` waited 3-4x too long, which
//! stretched sine-start steps and tones on F051/G071/G431.

const SYST_MASK: u32 = 0x00FF_FFFF;
const SYST_CSR_FREE_RUN: u32 = 0b101; // CLKSOURCE = core, TICKINT = 0, ENABLE

/// Start SysTick free-running at the core clock with its interrupt off,
/// reconfiguring it if a bootloader left it stopped or with a different reload
/// or clock/interrupt configuration.
pub fn systick_free_run() {
    // SAFETY: SysTick has no handler in this firmware; this configures the
    // 24-bit core-clock free-running counter whenever it is not already in
    // that state.
    unsafe {
        let st = &*cortex_m::peripheral::SYST::PTR;
        if (st.csr.read() & 0b111) != SYST_CSR_FREE_RUN || st.rvr.read() != SYST_MASK {
            st.csr.write(0);
            st.rvr.write(SYST_MASK);
            st.cvr.write(0);
            st.csr.write(SYST_CSR_FREE_RUN);
        }
    }
}

/// Wait at least `us` microseconds at the active chip's core clock
/// ([`rm32::functions::delay_micros_on`], tested against AM32 on host).
/// The loop must not be preempted for a whole SysTick period (2^24 cycles,
/// >= 98 ms).
pub fn delay_us(us: u32) {
    use crate::mcu::{Chip, ChipConfig};
    systick_free_run();
    // SAFETY: read-only access to the free-running SysTick counter.
    let st = unsafe { &*cortex_m::peripheral::SYST::PTR };
    rm32::functions::delay_micros_on(us, Chip::CPU_FREQUENCY_MHZ, SYST_MASK, || st.cvr.read());
}

/// AM32 `delayMillis` on [`delay_us`].
pub fn delay_ms(ms: u32) {
    rm32::functions::delay_millis_on(ms, delay_us);
}
