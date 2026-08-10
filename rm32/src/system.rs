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
use crate::sine::PhasePositions;
use embedded_hal::digital::OutputPin;

/// Main-loop system tick state.
///
/// Owns main-loop state shared by firmware and the host harness.
pub struct SystemTick {
    pub input_state: InputState,
    sine_positions: PhasePositions,
}

impl SystemTick {
    pub fn new() -> Self {
        Self {
            input_state: InputState::new(),
            sine_positions: PhasePositions::new(),
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

    /// Run one complete main-loop pipeline with an injected ISR step.
    ///
    /// The host harness runs the ISR inline through `isr_tick`; firmware can
    /// pass a no-op because the ISR runs asynchronously on hardware. The
    /// callback receives `main` so host-side ISR synchronization can happen
    /// before `tick_main()`.
    pub fn run_tick<LED: OutputPin>(
        &mut self,
        shared: &SharedState,
        main: &mut MainState<LED>,
        adc: &mut dyn Adc,
        telem: &mut dyn TelemetryUart,
        isr_tick: impl FnOnce(&mut MainState<LED>),
    ) {
        self.tick_input(shared, main);
        isr_tick(main);
        self.tick_main(shared, main, adc, telem);
    }

    /// Process sine mode stepping.
    pub fn tick_sine(
        &mut self,
        shared: &SharedState,
        config: &crate::config::EepromConfig,
        dead_time: i16,
        tim1_autoreload: u16,
    ) -> Option<(crate::sine::SineStepResult, (u16, u16, u16))> {
        if !shared.stepper_sine() {
            return None;
        }
        Some(crate::sine::sine_step(
            &mut self.sine_positions,
            shared.newinput(),
            shared.armed(),
            shared.forward(),
            config.motor_poles,
            5,
            dead_time,
            tim1_autoreload,
            config.sine_mode_power,
        ))
    }

    /// Apply sine changeover state transitions after `tick_sine` returns Changeover.
    pub fn apply_sine_changeover<LED: OutputPin>(
        &mut self,
        shared: &SharedState,
        main: &mut MainState<LED>,
        commutation_interval: u32,
    ) {
        shared.transition(crate::motor_mode::MotorEvent::ExitSine);
        shared.set_commutation_interval(commutation_interval);
        shared.set_zero_crosses(20);
        shared.set_prop_brake_active(false);
        main.timing_mut().set_average_interval(commutation_interval);
        main.timing_mut()
            .set_last_average_interval(commutation_interval);
    }

    /// Handle sine-mode idle brake-on-stop policy.
    pub fn handle_sine_idle(config: &crate::config::EepromConfig, tim1_arr: u16) -> bool {
        if config.brake_on_stop == 1 {
            let prop_brake_duty = config.drag_brake_strength as u32 * 200;
            let adjusted = tim1_arr as u32 - ((prop_brake_duty * tim1_arr as u32) / 2000);
            adjusted >= 100
        } else {
            false
        }
    }
}

impl Default for SystemTick {
    fn default() -> Self {
        Self::new()
    }
}
