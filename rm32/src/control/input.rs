//! Input signal processing pipeline — bidirectional mapping, stuck rotor
//! protection, brake logic, sine start mapping.
//!
//! This is the "glue" between raw DShot/servo input and the ISR control loop.
//! Called before `isr_logic::ten_khz_tick()` in both firmware and test harness.
//!
//! CRITICAL: Every exit path MUST publish the final mapped value to
//! `shared.set_adjusted_input()`. The ISR reads ONLY `adjusted_input` for
//! throttle→setpoint mapping. Any value not published there is invisible
//! to the motor.

use crate::config::EepromConfig;
use crate::constants::{BEMF_FAULT_LATCHED, THROTTLE_MIN_SIGNAL};
use crate::control::state::ProtectionState;
use crate::input_mapping;
use crate::shared_comm::SharedComm;

/// Persistent state for bidirectional input processing.
#[derive(Clone, Default)]
pub struct InputState {
    pub prop_brake_active: bool,
    pub return_to_center: bool,
    pub reverse_speed_threshold: u16,
    /// Mapped input value after all processing (0-2047).
    /// This is a local copy — the authoritative value is `shared.adjusted_input()`.
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
/// - Bidirectional DShot/servo/RC-car throttle mapping
/// - Stuck rotor (BEMF timeout) protection latch
/// - Sine start throttle mapping
/// - Brake-on-stop activation
///
/// All results are published to `shared.set_adjusted_input()` so the ISR
/// can read them immediately.
pub fn process_input<S: SharedComm>(
    shared: &S,
    config: &EepromConfig,
    protection: &mut ProtectionState,
    input_state: &mut InputState,
    is_dshot: bool,
) {
    let newinput = shared.newinput();

    // --- Stuck rotor protection latch ---
    // Check FIRST: if latched, force everything to zero regardless of input.
    // Uses raw newinput (not adjusted) so the latch persists even though
    // we ourselves zeroed adjusted_input on the previous tick.
    if protection.bemf_timeout_happened > protection.bemf_timeout
        && config.stuck_rotor_protection != 0
    {
        input_state.input = 0;
        shared.set_adjusted_input(0);
        protection.bemf_timeout_happened = BEMF_FAULT_LATCHED;
        return;
    }

    // --- Bidirectional throttle mapping ---
    if config.bi_direction != 0 {
        if is_dshot {
            if config.rc_car_reverse != 0 {
                let r = input_mapping::dshot_rc_car(
                    newinput,
                    shared.forward(),
                    config.dir_reversed != 0,
                    input_state.prop_brake_active,
                    input_state.return_to_center,
                );
                shared.set_adjusted_input(r.adjusted);
                if r.reverse {
                    shared.set_forward(!shared.forward());
                    input_state.return_to_center = false;
                }
                if r.prop_brake {
                    input_state.prop_brake_active = true;
                }
                if newinput <= THROTTLE_MIN_SIGNAL && input_state.prop_brake_active {
                    input_state.prop_brake_active = false;
                    input_state.return_to_center = true;
                }
            } else {
                let r = input_mapping::dshot_bidir(
                    newinput,
                    shared.forward(),
                    config.dir_reversed != 0,
                    shared.commutation_interval(),
                    shared.duty_cycle(),
                    shared.stepper_sine(),
                    input_state.reverse_speed_threshold,
                );
                shared.set_adjusted_input(r.adjusted);
                if r.reverse {
                    shared.set_forward(!shared.forward());
                    shared.set_zero_crosses(0);
                    shared.set_old_routine(true);
                }
            }
        } else {
            // Servo bidirectional
            let dead_band = config.servo_dead_band as u16;
            if config.rc_car_reverse != 0 {
                let r = input_mapping::servo_rc_car(
                    newinput,
                    shared.forward(),
                    config.dir_reversed != 0,
                    input_state.prop_brake_active,
                    input_state.return_to_center,
                    dead_band,
                );
                shared.set_adjusted_input(r.adjusted);
                if r.reverse {
                    shared.set_forward(!shared.forward());
                    input_state.return_to_center = false;
                }
                if r.prop_brake {
                    input_state.prop_brake_active = true;
                }
                // Dead band with active brake → clear brake, enable return_to_center
                let center = crate::constants::SERVO_CENTER;
                let db2 = dead_band << 1;
                if newinput >= center.saturating_sub(db2)
                    && newinput <= center + db2
                    && input_state.prop_brake_active
                {
                    input_state.prop_brake_active = false;
                    input_state.return_to_center = true;
                }
            } else {
                let r = input_mapping::servo_bidir(
                    newinput,
                    shared.forward(),
                    config.dir_reversed != 0,
                    shared.commutation_interval(),
                    shared.duty_cycle(),
                    shared.stepper_sine(),
                    input_state.reverse_speed_threshold,
                    dead_band,
                );
                shared.set_adjusted_input(r.adjusted);
                if r.reverse {
                    shared.set_forward(!shared.forward());
                    shared.set_zero_crosses(0);
                    shared.set_old_routine(true);
                }
            }
        }
    } else {
        // Unidirectional: adjusted = newinput
        shared.set_adjusted_input(newinput);
    }

    // --- Sine start throttle mapping ---
    // Maps adjusted_input through sine curve, then publishes BACK to adjusted_input
    // so the ISR setpoint path sees the shaped value.
    let adjusted = shared.adjusted_input();
    if config.use_sine_start != 0 {
        let mapped =
            input_mapping::sine_start_map(adjusted, config.sine_mode_changeover_throttle_level);
        input_state.input = mapped;
        shared.set_adjusted_input(mapped); // ISR must see the sine-shaped value
    } else {
        input_state.input = adjusted;
    }

    // --- Brake-on-stop ---
    // Skip for RC-car reverse (it has its own brake handshake)
    if shared.armed()
        && !shared.stepper_sine()
        && config.rc_car_reverse == 0
        && input_state.input < THROTTLE_MIN_SIGNAL
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

    fn setup() -> (TestShared, EepromConfig, ProtectionState, InputState) {
        let shared = TestShared::new();
        shared.mode.set(MotorMode::Armed);
        (
            shared,
            EepromConfig::default(),
            ProtectionState::default(),
            InputState::new(),
        )
    }

    #[test]
    fn unidirectional_passthrough() {
        let (shared, config, mut prot, mut input) = setup();
        shared.newinput.set(1000);
        process_input(&shared, &config, &mut prot, &mut input, true);
        assert_eq!(input.input, 1000);
        assert_eq!(shared.adjusted_input.get(), 1000);
    }

    #[test]
    fn bemf_timeout_latches_input_zero() {
        let (shared, mut config, mut prot, mut input) = setup();
        config.stuck_rotor_protection = 1;
        prot.bemf_timeout_happened = 20;
        prot.bemf_timeout = 10;
        shared.newinput.set(500);
        process_input(&shared, &config, &mut prot, &mut input, true);
        assert_eq!(input.input, 0);
        assert_eq!(shared.adjusted_input.get(), 0);
        assert_eq!(prot.bemf_timeout_happened, BEMF_FAULT_LATCHED);
    }

    #[test]
    fn bidir_dshot_forward_maps() {
        let (shared, mut config, mut prot, mut input) = setup();
        config.bi_direction = 1;
        shared.newinput.set(1200);
        process_input(&shared, &config, &mut prot, &mut input, true);
        assert_eq!(shared.adjusted_input.get(), 350);
        assert_eq!(input.input, 350); // also synced to local
    }

    #[test]
    fn brake_on_stop_activates() {
        let (shared, mut config, mut prot, mut input) = setup();
        config.brake_on_stop = 1;
        config.comp_pwm = 1;
        shared.newinput.set(0);
        process_input(&shared, &config, &mut prot, &mut input, true);
        assert!(input.prop_brake_active);
    }

    #[test]
    fn sine_start_publishes_to_shared() {
        let (shared, mut config, mut prot, mut input) = setup();
        config.use_sine_start = 1;
        config.sine_mode_changeover_throttle_level = 10; // changeover = 200
        shared.newinput.set(100);
        process_input(&shared, &config, &mut prot, &mut input, true);
        // Sine mapping should produce a value and publish it
        let mapped = shared.adjusted_input.get();
        assert!(
            mapped > 0,
            "sine mapping should produce nonzero for input=100"
        );
        assert_eq!(input.input, mapped, "local and shared must agree");
    }

    #[test]
    fn brake_on_stop_skipped_for_rc_car() {
        let (shared, mut config, mut prot, mut input) = setup();
        config.brake_on_stop = 1;
        config.comp_pwm = 1;
        config.rc_car_reverse = 1;
        config.bi_direction = 1;
        shared.newinput.set(0);
        process_input(&shared, &config, &mut prot, &mut input, true);
        assert!(!input.prop_brake_active, "RC-car has its own brake logic");
    }
}
