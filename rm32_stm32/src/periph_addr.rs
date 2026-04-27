//! Peripheral base addresses — single source of truth.
//!
//! Values match PAC definitions (e.g. `pac::GPIOA::PTR`).
//! Defined as u32 constants since PAC pointers can't be cast to
//! integers in const context.

// GPIO (same across all 3 MCUs)
pub const GPIOA: u32 = 0x4800_0000;
pub const GPIOB: u32 = 0x4800_0400;

// Clocks
pub const RCC: u32 = 0x4002_1000;

// DMA
pub const DMA1: u32 = 0x4002_0000;

// Timers
pub const TIM1: u32 = 0x4001_2C00;
pub const TIM2: u32 = 0x4000_0000;

#[cfg(any(feature = "stm32g071", feature = "stm32f051"))]
pub const TIM14: u32 = 0x4000_2000;
#[cfg(any(feature = "stm32l431", feature = "stm32g431"))]
pub const TIM16: u32 = 0x4001_4400;

#[cfg(any(feature = "stm32f051", feature = "stm32l431"))]
pub const TIM15: u32 = 0x4001_4000;

// USART
pub const USART1: u32 = 0x4001_3800;

// ADC
#[cfg(feature = "stm32f051")]
pub const ADC: u32 = 0x4001_2400;
#[cfg(feature = "stm32g071")]
pub const ADC: u32 = 0x4001_2400; // same on G0
#[cfg(feature = "stm32l431")]
pub const ADC1: u32 = 0x5004_0000;

// EXTI
pub const EXTI: u32 = 0x4001_0400;

// DMA channel register helpers (base + channel offset)
// Channel n registers start at DMA1 + 0x08 + (n-1) * 0x14
pub const fn dma_ch_ccr(ch: u32) -> u32 { DMA1 + 0x08 + (ch - 1) * 0x14 }
pub const fn dma_ch_cndtr(ch: u32) -> u32 { DMA1 + 0x0C + (ch - 1) * 0x14 }
pub const fn dma_ch_cpar(ch: u32) -> u32 { DMA1 + 0x10 + (ch - 1) * 0x14 }
pub const fn dma_ch_cmar(ch: u32) -> u32 { DMA1 + 0x14 + (ch - 1) * 0x14 }
