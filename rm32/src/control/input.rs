//! Input signal processing pipeline — bidirectional mapping, stuck rotor
//! protection, brake logic, sine start mapping.
//!
//! This is the "glue" between raw DShot/servo input and the ISR control loop.
//! Called before `isr_logic::ten_khz_tick()` in both firmware and test harness.

use crate::commutation::Commutation;
use crate::config::EepromConfig;
use crate::constants::BEMF_FAULT_LATCHED;
use crate::control::state::{DutyState, ProtectionState};
use crate::input_mapping;
use crate::shared_comm::SharedComm;

/// Persistent state for bidirectional input processing.
#[derive(Clone, Default)]
pub struct InputState {
    pub prop_brake_active: bool,
    pub return_to_center: bool,
    pub reverse_speed_threshold: u16,
    /// Mapped input value after all processing (0-2047)
    pub input: u16,
}

impl InputState {
    pub fn new() -> Self {
        Self {
            reverse_speed_threshold: 1500,
            ..Default::default()
        }
    }
}

/// Process raw input through the signal pipeline.
///
/// This is the production equivalent of tick.rs `set_input()`. Must be called
/// before `isr_logic::ten_khz_tick()` every tick.
///
/// Handles:
/// - Bidirectional DShot/RC-car throttle mapping
/// - Stuck rotor (BEMF timeout) protection latch
/// - Sine start throttle mapping
/// - Brake-on-stop activation
pub fn process_input<S: SharedComm>(
    shared: &S,
    commutation: &mut Commutation,
    config: &EepromConfig,
    duty: &DutyState,
    protection: &mut ProtectionState,
    input_state: &mut InputState,
    is_dshot: bool,
) {
    let newinput = shared.newinput();

    // --- Bidirectional throttle mapping ---
    if config.bi_direction != 0 {
        if is_dshot {
            if config.rc_car_reverse != 0 {
                let r = input_mapping::dshot_rc_car(
                    newinput,
                    commutation.forward,
                    config.dir_reversed != 0,
                    input_state.prop_brake_active,
                    input_state.return_to_center,
                );
                shared.set_adjusted_input(r.adjusted);
                if r.reverse {
                    commutation.forward = !commutation.forward;
                    input_state.return_to_center = false;
                }
                if r.prop_brake {
                    input_state.prop_brake_active = true;
                }
                // Zero input with active brake → clear brake, enable return_to_center
                if newinput <= 47 && input_state.prop_brake_active {
                    input_state.prop_brake_active = false;
                    input_state.return_to_center = true;
                }
            } else {
                let r = input_mapping::dshot_bidir(
                    newinput,
                    commutation.forward,
                    config.dir_reversed != 0,
                    shared.commutation_interval(),
                    duty.cycle,
                    shared.stepper_sine(),
                    input_state.reverse_speed_threshold,
                );
                shared.set_adjusted_input(r.adjusted);
                if r.reverse {
                    commutation.forward = !commutation.forward;
                    shared.set_zero_crosses(0);
                    shared.set_old_routine(true);
                }
            }
        }
        // Servo bidirectional: pass through (mapping done in transfer.rs)
    } else {
        // Unidirectional: adjusted = newinput (no mapping needed)
        shared.set_adjusted_input(newinput);
    }

    // --- Stuck rotor protection latch ---
    if protection.bemf_timeout_happened > protection.bemf_timeout
        && config.stuck_rotor_protection != 0
    {
        input_state.input = 0;
        protection.bemf_timeout_happened = BEMF_FAULT_LATCHED;
        return;
    }

    // --- Sine start throttle mapping ---
    if config.use_sine_start != 0 {
        input_state.input = input_mapping::sine_start_map(
            shared.adjusted_input(),
            config.sine_mode_changeover_throttle_level,
        );
    } else {
        input_state.input = shared.adjusted_input();
    }

    // --- Brake-on-stop ---
    if shared.armed()
        && !shared.stepper_sine()
        && input_state.input < 47
        && config.brake_on_stop == 1
        && config.comp_pwm != 0
    {
        input_state.prop_brake_active = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::shared_impl::TestShared;
    use crate::motor_mode::MotorMode;

    fn setup() -> (
        TestShared,
        Commutation,
        EepromConfig,
        DutyState,
        ProtectionState,
        InputState,
    ) {
        let shared = TestShared::new();
        shared.mode.set(MotorMode::Armed);
        (
            shared,
            Commutation::new(),
            EepromConfig::default(),
            DutyState::default(),
            ProtectionState::default(),
            InputState::new(),
        )
    }

    #[test]
    fn unidirectional_passthrough() {
        let (shared, mut comm, config, duty, mut prot, mut input) = setup();
        shared.newinput.set(1000);
        shared.adjusted_input.set(1000);
        process_input(
            &shared, &mut comm, &config, &duty, &mut prot, &mut input, true,
        );
        assert_eq!(input.input, 1000);
    }

    #[test]
    fn bemf_timeout_latches_input_zero() {
        let (shared, mut comm, mut config, duty, mut prot, mut input) = setup();
        config.stuck_rotor_protection = 1;
        prot.bemf_timeout_happened = 20;
        prot.bemf_timeout = 10;
        shared.newinput.set(500);
        process_input(
            &shared, &mut comm, &config, &duty, &mut prot, &mut input, true,
        );
        assert_eq!(input.input, 0);
        assert_eq!(prot.bemf_timeout_happened, BEMF_FAULT_LATCHED);
    }

    #[test]
    fn bidir_dshot_forward_maps() {
        let (shared, mut comm, mut config, duty, mut prot, mut input) = setup();
        config.bi_direction = 1;
        shared.newinput.set(1200);
        process_input(
            &shared, &mut comm, &config, &duty, &mut prot, &mut input, true,
        );
        // adjusted_input should be mapped to bidir value: ((1200-1048)*2+47)-1 = 350
        assert_eq!(shared.adjusted_input.get(), 350);
    }

    #[test]
    fn brake_on_stop_activates() {
        let (shared, mut comm, mut config, duty, mut prot, mut input) = setup();
        config.brake_on_stop = 1;
        config.comp_pwm = 1;
        shared.newinput.set(0);
        shared.adjusted_input.set(0);
        process_input(
            &shared, &mut comm, &config, &duty, &mut prot, &mut input, true,
        );
        assert!(input.prop_brake_active);
    }
}
