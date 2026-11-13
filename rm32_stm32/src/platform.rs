//! The STM32 port's `rm32::platform::Platform`: clocks derived from the
//! active chip's constants, board constants from the board yaml.

use crate::mcu::{Chip, ChipConfig};

mod board {
    include!(concat!(env!("OUT_DIR"), "/board_platform.rs"));
}

/// Zero-sized platform marker for the active STM32 chip.
pub struct StmPlatform;

impl rm32::platform::Platform for StmPlatform {
    /// TIM6 update rate.
    const TICK_HZ: u32 = 20_000;
    /// Interval/commutation timer: CPU clock / (TIMER_PSC + 1).
    const TIMER_HZ: u32 = Chip::CPU_FREQUENCY_MHZ * 1_000_000 / (Chip::TIMER_PSC as u32 + 1);
    /// The input-capture timer runs from the CPU clock.
    const CAPTURE_HZ: u32 = Chip::CPU_FREQUENCY_MHZ * 1_000_000;
    const GCR_SHIFT: u8 = Chip::GCR_SHIFT;
    const VOLTAGE_DIVIDER_X100: u16 = board::VOLTAGE_DIVIDER_X100;
    const MILLIVOLT_PER_AMP: u16 = board::MILLIVOLT_PER_AMP;
    const CURRENT_OFFSET: i16 = board::CURRENT_OFFSET;
    const USE_NTC: bool = board::USE_NTC;
    /// 12-bit ADC on VDDA = 3.3 V (AM32's `* 3300 / 4095`).
    const ADC_FULL_SCALE: u16 = 4095;
    const ADC_VREF_MV: u16 = 3300;
    /// TIM1 auto-reload for 24 kHz at the chip clock (AM32 `TIMER1_MAX_ARR`).
    const PWM_BASE_PERIOD: u16 = Chip::TIM1_AUTORELOAD;
}

/// The active compile-time tuning policy for this port.
pub type Tuning = rm32::tuning::Am32Tuning;

/// The core's main-loop state on this port, under the active tuning policy.
pub type MainState = rm32::main_state::MainState<StmPlatform, rm32::main_state::NoLed, Tuning>;
