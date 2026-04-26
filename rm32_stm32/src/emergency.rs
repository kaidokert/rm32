//! Emergency FET-off implementation — forces all motor outputs low.
//!
//! Uses raw GPIO BSRR writes that work without any HAL state.
//! Safe to call from any context, including ISR with missing state.

use crate::periph_addr;

/// G0_A / F0_A / L4_N pin layout emergency shutdown.
pub struct G0AEmergencyOff;

impl rm32::hal::EmergencyOff for G0AEmergencyOff {
    fn emergency_off() {
        const BSRR: u32 = 0x18;
        unsafe {
            // Reset PA7/8/9/10 (high-side FETs off)
            ((periph_addr::GPIOA + BSRR) as *mut u32).write_volatile(
                (1 << (7 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << (10 + 16))
            );
            // Reset PB0/1 (low-side FETs off)
            ((periph_addr::GPIOB + BSRR) as *mut u32).write_volatile(
                (1 << 16) | (1 << (1 + 16))
            );
        }
    }
}
