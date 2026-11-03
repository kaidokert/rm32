//! Emergency FET-off implementation — forces all motor outputs low.
//!
//! Clears GPIO output latches via `BSRR`, switches all six bridge pins out of
//! `TIM1` alternate-function mode into GPIO output mode (`MODER = 0b01`),
//! disables `TIM1` main output enable (`BDTR.MOE = 0`), and deasserts the
//! external gate-driver `ENABLE` pin when present.
//! Safe to call from any context, including `panic` and `HardFault`.
#![allow(unexpected_cfgs)]

use crate::gpio_regs::GpioPort;
use crate::mcu::{PortA, PortB};

#[cfg(all(target_arch = "arm", feature = "stm32g071"))]
#[allow(dead_code)]
mod board_cfg {
    use crate as rm32_stm32;
    include!(concat!(env!("OUT_DIR"), "/board_config.rs"));
}

/// G0_A / F0_A / L4_N pin layout emergency shutdown.
pub struct G0AEmergencyOff;

impl rm32::hal::EmergencyOff for G0AEmergencyOff {
    fn emergency_off() {
        // 1. Pre-stage GPIO output latches low via atomic BSRR reset bits.
        PortA::write_bsrr((1 << (7 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << (10 + 16)));
        PortB::write_bsrr((1 << 16) | (1 << (1 + 16)));

        // 2. Switch PA7..PA10 and PB0..PB1 from TIM1 alternate function (0b10)
        // to GPIO output (0b01) so pins actively drive their cleared ODR latches.
        const PORT_A_MASK: u32 = (0b11 << 14) | (0b11 << 16) | (0b11 << 18) | (0b11 << 20);
        const PORT_A_OUT: u32 = (0b01 << 14) | (0b01 << 16) | (0b01 << 18) | (0b01 << 20);
        PortA::modify_moder(|v| (v & !PORT_A_MASK) | PORT_A_OUT);
        PortB::modify_moder(|v| (v & !0b1111) | 0b0101);

        // 3. Disable TIM1 main output enable (BDTR.MOE = 0) and gate-driver ENABLE.
        #[cfg(all(target_arch = "arm", feature = "stm32g071"))]
        {
            crate::mcu_g071::pwm::set_moe(false);
            board_cfg::BOARD_EXT.set_gate_enable(false);
        }
        #[cfg(all(target_arch = "arm", any(feature = "stm32f051", feature = "stm32l431")))]
        unsafe {
            (&*crate::pac::TIM1::ptr())
                .bdtr
                .modify(|_, w| w.moe().clear_bit());
        }
        #[cfg(all(target_arch = "arm", feature = "stm32g431"))]
        unsafe {
            (&*crate::pac::TIM1::PTR)
                .bdtr()
                .modify(|_, w| w.moe().clear_bit());
        }
    }
}
