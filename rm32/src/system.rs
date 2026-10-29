//! Unified system tick — single entry point for main-loop pipeline.
//!
//! Both the firmware (`main.rs`) and the test harness (`harness.rs`)
//! call these functions. This ensures the control pipeline is identical
//! in both contexts, eliminating the "harness vs firmware" divergence
//! that caused bugs across multiple review rounds.

use crate::control::input::{self, InputState};
use crate::hal::{Adc, BemfSensor, Bridge, ComTimer, MotorHal, PhaseOutput, TelemetryUart};
use crate::main_state::MainState;
use crate::shared_state::SharedState;
use crate::sine::PhasePositions;
use embedded_hal::digital::OutputPin;

/// Main-loop system tick state.
///
/// Owns main-loop state shared by firmware and the host harness.
pub struct SystemTick<P: crate::constants::Timing> {
    _platform: core::marker::PhantomData<P>,
    pub input_state: InputState,
    sine_positions: PhasePositions,
}

impl<P: crate::constants::Timing> SystemTick<P> {
    pub fn new() -> Self {
        Self {
            _platform: core::marker::PhantomData,
            input_state: InputState::new::<P>(),
            sine_positions: PhasePositions::new(),
        }
    }

    /// Run input processing pipeline.
    ///
    /// Call this BEFORE the ISR tick (harness) or independently (firmware,
    /// where the ISR tick runs in the actual interrupt).
    pub fn tick_input<LED: OutputPin>(
        &mut self,
        shared: &SharedState,
        main: &mut MainState<P, LED>,
    ) {
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
        main: &mut MainState<P, LED>,
        adc: &mut dyn Adc,
        telem: &mut dyn TelemetryUart,
    ) {
        main.tick(shared, adc, telem);
    }

    /// Run one complete main-loop pipeline with an injected ISR step.
    ///
    /// The host harness runs the ISR inline through `isr_tick`; firmware can
    /// pass a no-op because the ISR runs asynchronously on hardware. The
    /// ISR-to-main handoff is synchronized before `tick_main()`.
    pub fn run_tick<LED: OutputPin>(
        &mut self,
        shared: &SharedState,
        main: &mut MainState<P, LED>,
        adc: &mut dyn Adc,
        telem: &mut dyn TelemetryUart,
        isr_tick: impl FnOnce(),
    ) {
        Self::sync_config_writes(shared, main);
        self.tick_input(shared, main);
        isr_tick();
        Self::sync_config_writes(shared, main);
        self.sync_isr_to_main(shared, main);
        self.tick_main(shared, main, adc, telem);
    }

    /// Consume ISR-published flags that affect main-loop state.
    pub fn sync_isr_to_main<LED: OutputPin>(
        &self,
        shared: &SharedState,
        main: &mut MainState<P, LED>,
    ) {
        if shared.take_desync_check_pending() {
            main.set_desync_check(true);
        }
    }

    pub fn sync_config_writes<LED: OutputPin>(shared: &SharedState, main: &mut MainState<P, LED>) {
        while let Some((offset, value)) = shared.pop_config_write() {
            if let Some(byte) = main.config.as_bytes_mut().get_mut(offset as usize) {
                *byte = value;
            }
        }
    }

    /// Recover from a bounded-queue overflow or prepare a persistent snapshot.
    /// The caller must exclude the input producer for this entire operation.
    /// Discard old queued bytes before copying, so they cannot undo the snapshot.
    pub fn sync_config_snapshot<LED: OutputPin>(
        shared: &SharedState,
        main: &mut MainState<P, LED>,
        authoritative: &crate::config::EepromConfig,
    ) {
        while shared.pop_config_write().is_some() {}
        main.config = *authoritative;
        shared.clear_config_write_overflow();
    }

    /// Claim one save and prepare the exact config to persist. Call with the
    /// input producer excluded; perform flash I/O after releasing that exclusion.
    /// A subsequent producer request must remain pending for the next save.
    pub fn prepare_config_save<LED: OutputPin>(
        shared: &SharedState,
        main: &mut MainState<P, LED>,
        authoritative: &crate::config::EepromConfig,
    ) -> bool {
        if !shared.save_settings_flag() {
            return false;
        }
        Self::sync_config_snapshot(shared, main, authoritative);
        shared.take_servo_calibration();
        shared.set_save_settings_flag(false);
        true
    }

    /// Calculate one step using mapped/protected input. Hardware writes are
    /// performed only by `service_sine` while interrupt access is excluded.
    fn tick_sine(
        &mut self,
        shared: &SharedState,
        config: &crate::config::EepromConfig,
        dead_time: i16,
        tim1_autoreload: u16,
    ) -> Option<(crate::sine::SineStepResult, (u16, u16, u16))> {
        if !shared.stepper_sine() {
            return None;
        }
        Some(crate::sine::sine_step::<P>(
            &mut self.sine_positions,
            shared.adjusted_input(),
            shared.armed(),
            shared.forward(),
            config.motor_poles,
            crate::constants::SINE_CHANGEOVER_STEP,
            dead_time,
            tim1_autoreload,
            config.sine_mode_power,
        ))
    }

    /// Service sine output with exclusive access to the motor peripherals.
    /// Firmware must exclude motor interrupts for this entire call. The returned
    /// delay is performed AFTER leaving that critical section.
    #[allow(clippy::too_many_arguments)]
    pub fn service_sine<LED: OutputPin, H: MotorHal>(
        &mut self,
        shared: &SharedState,
        main: &mut MainState<P, LED>,
        hal: &mut H,
        duty: &mut crate::control::state::DutyState,
        commutation_step: u8,
        dead_time: i16,
    ) -> Option<u16> {
        // The main-loop stepper and ISR must interpret timer counts alike.
        assert!(P::TIMER_HZ == H::TIMER_HZ && P::TICK_HZ == H::TICK_HZ);
        if !shared.stepper_sine() || shared.isr_action() == crate::shared_comm::IsrAction::AllOff {
            duty.sine_outputs_on = false;
            return None;
        }
        if shared.changeover_step() != 0 {
            // Wait for the ISR to accept or cancel the published handoff.
            return None;
        }
        if shared.adjusted_input() <= crate::constants::THROTTLE_MIN_SIGNAL {
            duty.sine_outputs_on = false;
            let brake = Self::handle_sine_idle(&main.config, shared.tim1_arr());
            self.input_state.set_prop_brake_active(brake);
            shared.set_prop_brake_active(brake);
            return None;
        }
        if !duty.sine_outputs_on {
            self.sine_positions = PhasePositions::from_step(commutation_step);
            hal.com_timer().disable_interrupt();
            hal.bemf().mask_irq();
            hal.bridge().set_duty_all(0);
            hal.bridge().latch();
            hal.bridge().all_pwm();
            duty.active_brake_on = false;
            duty.sine_outputs_on = true;
        }
        self.input_state.set_prop_brake_active(false);
        shared.set_prop_brake_active(false);
        let (result, (a, b, c)) =
            self.tick_sine(shared, &main.config, dead_time, shared.tim1_arr())?;
        hal.bridge().set_phase_duties(a, b, c);
        match result {
            crate::sine::SineStepResult::Continue(delay) => Some(delay),
            crate::sine::SineStepResult::Changeover {
                commutation_interval,
                step,
            } => {
                self.apply_sine_changeover(shared, main, commutation_interval, step);
                duty.sine_outputs_on = false;
                None
            }
            crate::sine::SineStepResult::Idle => unreachable!(),
        }
    }

    /// Apply sine changeover state transitions after `tick_sine` returns Changeover.
    fn apply_sine_changeover<LED: OutputPin>(
        &mut self,
        shared: &SharedState,
        main: &mut MainState<P, LED>,
        commutation_interval: u32,
        step: u8,
    ) {
        shared.set_commutation_interval(commutation_interval);
        shared.set_zero_crosses(20);
        shared.set_prop_brake_active(false);
        main.timing_mut().set_average_interval(commutation_interval);
        main.timing_mut()
            .set_last_average_interval(commutation_interval);
        shared.set_changeover_step(step);
        // ISR consumes the request, establishes hardware state, then exits sine.
    }

    /// Handle sine-mode idle brake-on-stop policy.
    pub fn handle_sine_idle(config: &crate::config::EepromConfig, tim1_arr: u16) -> bool {
        if config.brake_on_stop == 1 {
            let prop_brake_duty = config.drag_brake_strength as u32 * 200;
            let tim1_arr = tim1_arr as u32;
            let scaled = ((prop_brake_duty * tim1_arr) / 2000).min(tim1_arr);
            let adjusted = tim1_arr - scaled;
            adjusted >= 100
        } else {
            false
        }
    }
}

impl<P: crate::constants::Timing> Default for SystemTick<P> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use crate::board::BoardConfig;
    use crate::config::EepromConfig;
    use crate::main_state::ChipParams;
    use crate::motor_mode::MotorEvent;
    use crate::sine::SineStepResult;

    use super::*;
    type SystemTick = super::SystemTick<crate::platform::TestPlatform>;
    type MainState<LED = crate::main_state::NoLed> =
        crate::main_state::MainState<crate::platform::TestPlatform, LED>;

    struct MockAdc;

    impl crate::hal::Adc for MockAdc {
        fn start_conversion(&mut self) {}
        fn raw_voltage(&self) -> u16 {
            0
        }
        fn raw_current(&self) -> u16 {
            0
        }
        fn raw_temperature(&self) -> u16 {
            0
        }
        fn calc_temperature(&self, _: u16) -> crate::units::DegreesCelsius {
            crate::units::DegreesCelsius(25)
        }
    }

    struct MockTelem;

    impl crate::hal::TelemetryUart for MockTelem {
        fn send_dma(&mut self, _: &[u8]) {}
    }

    fn main_state() -> MainState {
        MainState::new(
            &BoardConfig::DEFAULT,
            ChipParams {
                timer1_max_arr: 1999,
                cpu_mhz: 48,
            },
        )
    }

    #[test]
    fn sine_uses_protected_mapped_input() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        shared.transition(MotorEvent::Arm);
        shared.transition(MotorEvent::EnterSine);
        shared.set_newinput(1000);
        shared.set_adjusted_input(0);
        let mut system = SystemTick::new();
        let (result, _) = system
            .tick_sine(&shared, &EepromConfig::default(), 60, 1999)
            .unwrap();
        assert!(matches!(result, SineStepResult::Idle));
    }

    #[test]
    fn config_write_through_reaches_main_copy() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = main_state();
        let offset = core::mem::offset_of!(EepromConfig, dir_reversed) as u8;

        shared.push_config_write(offset, 1);
        shared.push_config_write(3, 77);
        SystemTick::sync_config_writes(&shared, &mut main);

        assert_eq!(main.config.dir_reversed, 1);
        assert_eq!(main.config.as_bytes()[3], 77);
        assert!(shared.pop_config_write().is_none());

        shared.push_config_write(offset, 2);
        SystemTick::sync_config_writes(&shared, &mut main);
        assert_eq!(main.config.dir_reversed, 2);
    }

    #[test]
    fn run_tick_drains_isr_config_writes() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = main_state();
        let mut system = SystemTick::new();
        let offset = core::mem::offset_of!(EepromConfig, dir_reversed) as u8;

        for _ in 0..<crate::platform::TestPlatform as crate::constants::Timing>::PID_LOOP_DIVIDER {
            shared.one_khz_counter_inc();
        }
        shared.set_send_telemetry(true);
        system.run_tick(&shared, &mut main, &mut MockAdc, &mut MockTelem, || {
            shared.push_config_write(offset, 1);
            shared.push_config_write(u8::MAX, 99);
        });

        assert_eq!(main.config.dir_reversed, 1);
        assert!(shared.pop_config_write().is_none());
    }

    #[test]
    fn sync_isr_to_main_consumes_pending_desync_check() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = main_state();
        let system = SystemTick::new();

        shared.set_desync_check_pending(true);
        system.sync_isr_to_main(&shared, &mut main);

        assert!(main.desync_check());
        assert!(!shared.desync_check_pending());
    }

    #[test]
    fn tick_sine_runs_only_in_sine_mode() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut system = SystemTick::new();
        let config = EepromConfig::default();

        assert!(system.tick_sine(&shared, &config, 60, 1999).is_none());

        shared.transition(MotorEvent::Arm);
        shared.transition(MotorEvent::EnterSine);
        shared.set_newinput(crate::constants::SINE_CHANGEOVER_THROTTLE + 1);
        shared.set_adjusted_input(crate::constants::SINE_CHANGEOVER_THROTTLE + 1);

        let Some((
            SineStepResult::Changeover {
                commutation_interval,
                step,
            },
            pwm,
        )) = system.tick_sine(&shared, &config, 60, 1999)
        else {
            panic!("expected sine changeover");
        };

        assert_eq!(commutation_interval, 9000);
        assert_eq!(step, crate::constants::SINE_CHANGEOVER_STEP);
        assert!(pwm.0 > 0 || pwm.1 > 0 || pwm.2 > 0);
    }

    #[test]
    fn apply_sine_changeover_publishes_handoff_before_exit() {
        let shared = SharedState::new::<crate::platform::TestPlatform>();
        let mut main = main_state();
        let mut system = SystemTick::new();

        shared.transition(MotorEvent::Arm);
        shared.transition(MotorEvent::EnterSine);
        shared.set_prop_brake_active(true);

        system.apply_sine_changeover(&shared, &mut main, 9000, 5);

        assert!(shared.stepper_sine());
        assert_eq!(shared.commutation_interval(), 9000);
        assert_eq!(shared.zero_crosses(), 20);
        assert_eq!(shared.changeover_step(), 5);
        assert_eq!(main.timing().average_interval(), 9000);
        assert!(!shared.prop_brake_active());
    }

    #[test]
    fn handle_sine_idle_applies_brake_policy_without_underflow() {
        let mut config = EepromConfig::default();

        assert!(!SystemTick::handle_sine_idle(&config, 1999));

        config.brake_on_stop = 1;
        config.drag_brake_strength = 5;
        assert!(SystemTick::handle_sine_idle(&config, 1999));

        config.drag_brake_strength = u8::MAX;
        assert!(!SystemTick::handle_sine_idle(&config, 1999));
    }
}
