//! Peripheral base addresses — derived from PAC for correctness.
//!
//! Wraps `pac::PERIPHERAL::PTR` to produce u32 addresses for raw MMIO.
//! Using PAC addresses prevents transcription errors (e.g., GPIO is at
//! 0x5000_0000 on G0 but 0x4800_0000 on F0/L4/G4).

use crate::pac;

// GPIO — IOPORT bus on G0 (0x5000_xxxx), AHB on F0/L4/G4 (0x4800_xxxx).
pub fn gpioa() -> u32 { pac::GPIOA::PTR as u32 }
pub fn gpiob() -> u32 { pac::GPIOB::PTR as u32 }

// Timers
pub fn tim1() -> u32 { pac::TIM1::PTR as u32 }
pub fn tim2() -> u32 { pac::TIM2::PTR as u32 }

#[cfg(any(feature = "stm32g071", feature = "stm32f051"))]
pub fn tim14() -> u32 { pac::TIM14::PTR as u32 }

#[cfg(any(feature = "stm32l431", feature = "stm32g431"))]
pub fn tim16() -> u32 { pac::TIM16::PTR as u32 }

// Clocks
pub fn rcc() -> u32 { pac::RCC::PTR as u32 }

// DMA
pub fn dma1() -> u32 { pac::DMA1::PTR as u32 }
