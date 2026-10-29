//! Clock facts for the selected STM32 port.
use crate::mcu::{Chip, ChipConfig};
pub struct StmPlatform;
impl rm32::platform::Platform for StmPlatform {
    const TICK_HZ: u32 = 20_000;
    const TIMER_HZ: u32 = Chip::CPU_FREQUENCY_MHZ * 1_000_000 / (Chip::TIMER_PSC as u32 + 1);
    const CAPTURE_HZ: u32 = Chip::CPU_FREQUENCY_MHZ * 1_000_000;
    const GCR_SHIFT: u8 = Chip::GCR_SHIFT;
    const PWM_BASE_PERIOD: u16 = Chip::TIM1_AUTORELOAD;
}
