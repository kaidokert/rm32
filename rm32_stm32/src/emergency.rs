//! Emergency FET-off implementation — forces all motor outputs low.
//!
//! Uses GpioPort trait for safe BSRR writes — no raw pointers at call site.
//! Safe to call from any context, including ISR with missing state.

use crate::gpio_regs::GpioPort;
use crate::mcu::{PortA, PortB};

/// G0_A / F0_A / L4_N pin layout emergency shutdown.
pub struct G0AEmergencyOff;

impl rm32::hal::EmergencyOff for G0AEmergencyOff {
    fn emergency_off() {
        // BSRR reset bits force GPIO outputs low regardless of peripheral state.
        // GpioPort::write_bsrr is safe (write-only, bit-atomic register).
        PortA::write_bsrr(
            (1 << (7 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << (10 + 16))
        );
        PortB::write_bsrr(
            (1 << 16) | (1 << (1 + 16))
        );
    }
}
