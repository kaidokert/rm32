//! Safe panic handler for motor controller firmware.
//!
//! Any panic — math overflow, array OOB, unwrap failure, ISR state missing —
//! results in all FETs forced off, interrupts disabled, CPU halted.
//! This prevents a stuck-high FET from burning the motor/ESC.
//!
//! Replaces `panic_halt` which halts without safing hardware.

use core::panic::PanicInfo;

use rm32::hal::EmergencyOff;

#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // 1. Force all FETs off via direct GPIO writes (no state needed)
    crate::emergency::G0AEmergencyOff::emergency_off();

    // 2. Disable all interrupts to prevent further ISR triggers
    cortex_m::interrupt::disable();

    // 3. Halt — CPU stops here, motor is safe
    loop {
        cortex_m::asm::nop();
    }
}
