//! Safe panic handler for motor controller firmware.
//!
//! Any panic — math overflow, array OOB, unwrap failure, ISR state missing —
//! results in all FETs forced off, interrupts disabled, CPU halted.
//! This prevents a stuck-high FET from burning the motor/ESC.
//!
//! Replaces `panic_halt` which halts without safing hardware.
//!
//! Also includes a HardFault exception handler that RTT-logs the stacked
//! exception frame + SCB fault status registers before halting. Lets us
//! identify the faulting instruction without having to single-step.

use core::panic::PanicInfo;

use cortex_m_rt::{ExceptionFrame, exception};

use rm32::hal::EmergencyOff;

#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // 1. Force all FETs off via direct GPIO writes (no state needed)
    crate::emergency::G0AEmergencyOff::emergency_off();

    // 2. Disable all interrupts to prevent further ISR triggers
    cortex_m::interrupt::disable();

    // 3. Log via dprintln. Avoid full PanicInfo Display formatting (which pulls
    // in heavy formatting code and risks stack overflow when called from a deep
    // ISR stack); emit only file:line to identify the panic site.
    if let Some(loc) = info.location() {
        crate::dprintln!("PANIC at {}:{}", loc.file(), loc.line());
    } else {
        crate::dprintln!("PANIC (no location)");
    }

    // 4. Halt — CPU stops here, motor is safe
    loop {
        cortex_m::asm::nop();
    }
}

#[cfg(not(test))]
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // FETs off first.
    crate::emergency::G0AEmergencyOff::emergency_off();
    cortex_m::interrupt::disable();

    crate::dprintln!("=== HardFault ===");
    crate::dprintln!(
        "PC={:#010x} LR={:#010x} PSR={:#010x}",
        ef.pc(),
        ef.lr(),
        ef.xpsr()
    );
    // CFSR/HFSR/MMFAR/BFAR only exist on Cortex-M3+ (not M0/M0+)
    #[cfg(any(feature = "stm32l431", feature = "stm32g431"))]
    {
        let scb = unsafe { &*cortex_m::peripheral::SCB::PTR };
        let cfsr = scb.cfsr.read();
        let hfsr = scb.hfsr.read();
        let mmfar = scb.mmfar.read();
        let bfar = scb.bfar.read();
        crate::dprintln!("CFSR={:#010x} HFSR={:#010x}", cfsr, hfsr);
        crate::dprintln!("MMFAR={:#010x} BFAR={:#010x}", mmfar, bfar);
    }

    loop {
        cortex_m::asm::nop();
    }
}
