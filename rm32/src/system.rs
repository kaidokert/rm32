//! Unified system tick — single entry point for main-loop pipeline.
//!
//! Both the firmware (`main.rs`) and the test harness (`harness.rs`)
//! call these functions. This ensures the control pipeline is identical
//! in both contexts, eliminating the "harness vs firmware" divergence
//! that caused bugs across multiple review rounds.

use crate::control::input::{self, InputState};
use crate::hal::{Adc, TelemetryUart};
use crate::main_state::MainState;
use crate::shared_state::SharedState;
use embedded_hal::digital::OutputPin;

/// Main-loop system tick state.
///
/// Owns the `InputState` that was previously duplicated between
/// harness and firmware. Both call the same `tick_input()` and
/// `tick_main()` methods.
pub struct SystemTick {
    pub input_state: InputState,
}

impl SystemTick {
    pub fn new() -> Self {
        Self {
            input_state: InputState::new(),
        }
    }

    /// Run input processing pipeline.
    ///
    /// Call this BEFORE the ISR tick (harness) or independently (firmware,
    /// where the ISR tick runs in the actual interrupt).
    pub fn tick_input<LED: OutputPin>(&mut self, shared: &SharedState, main: &mut MainState<LED>) {
        // Recompute input mode from config + detected protocol each tick.
        // Cheap (a few comparisons) and ensures mode stays in sync with config.
        self.input_state.mode =
            crate::input_mapping::InputMode::from_config(&main.config, shared.dshot());
        input::process_input(
            shared,
            &main.config,
            &mut main.protection,
            &mut self.input_state,
        );
    }

    /// Run main-loop pipeline.
    ///
    /// Call this AFTER the ISR tick.
    pub fn tick_main<LED: OutputPin>(
        &self,
        shared: &SharedState,
        main: &mut MainState<LED>,
        adc: &mut dyn Adc,
        telem: &mut dyn TelemetryUart,
    ) {
        main.tick(shared, adc, telem);
    }
}

impl Default for SystemTick {
    fn default() -> Self {
        Self::new()
    }
}
