//! Abstraction over MCU-specific DMA and Timer register access for input capture.
//!
//! Allows a single generic DshotCapture implementation across G071/F051/L431.

/// DMA channel operations needed for input capture.
pub trait DmaOps {
    /// Disable the DMA channel.
    fn disable(&self);
    /// Set memory address register.
    fn set_mar(&self, addr: u32);
    /// Set peripheral address register.
    fn set_par(&self, addr: u32);
    /// Set transfer count.
    fn set_ndtr(&self, count: u32);
    /// Configure and enable for periph→memory, 32-bit, TCIE, MINC.
    fn start_rx(&self);
    /// Configure and enable for memory→periph, 32-bit, TCIE, MINC.
    fn start_tx(&self);
}

/// Timer operations needed for input capture.
pub trait TimerOps {
    /// Reset the timer via RCC.
    fn reset(&self);
    /// Configure for input capture: CCMR1=IC, both edges, prescaler, ARR=0xFFFF.
    fn configure_capture(&self, prescaler: u8);
    /// Configure for PWM output: CCMR1=PWM1, output enable, ARR=period.
    fn configure_output(&self, period: u16);
    /// Enable timer + DMA request + capture/output channel.
    fn start(&self);
    /// Get raw pointer to CCR1 register (for DMA PAR).
    fn ccr_addr(&self) -> u32;
}

/// GPIO input pin for signal detection.
pub trait InputPinOps {
    fn read(&self) -> bool;
    fn set_pull_up(&self);
    fn set_pull_down(&self);
    fn set_pull_none(&self);
}
