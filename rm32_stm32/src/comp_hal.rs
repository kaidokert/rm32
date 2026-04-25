//! Abstraction over MCU-specific comparator and EXTI register access.

/// Comparator register operations.
pub trait CompOps {
    /// Read comparator output level (true = high).
    fn output(&self) -> bool;
    /// Write the inverting input mux (phase selection).
    fn set_inmsel(&self, phase: u32);
}

/// EXTI edge trigger operations for the comparator line.
pub trait ExtiOps {
    fn set_rising_edge(&self);
    fn set_falling_edge(&self);
    fn enable_interrupt(&self);
    fn mask_and_clear(&self);
}

/// Phase input mux values (MCU-specific).
pub struct InmselMap {
    pub phase_a: u32,
    pub phase_b: u32,
    pub phase_c: u32,
}
