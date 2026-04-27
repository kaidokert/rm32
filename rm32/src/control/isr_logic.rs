//! ISR-level control logic — platform-independent, fully testable.
//!
//! These free functions implement the bodies of the 20kHz tick, commutation
//! timer, and BEMF comparator ISRs. They operate on split state (individual
//! sub-structs + SharedComm) rather than a monolithic MotorState.

use crate::commutation::Commutation;
use crate::config::EepromConfig;
use crate::constants::*;
use crate::control::state::{BemfState, DutyState};
use crate::functions::map;
use crate::hal;
use crate::shared_comm::SharedComm;

/// Counters and config owned exclusively by the ISR tick.
pub struct TickCounters {
    pub ten_khz_counter: u32,
    pub one_khz_loop_counter: u16,
    pub armed_timeout_count: u32,
    pub tim1_arr: u16,
    pub voltage_based_ramp: bool,
    pub pulse_output: bool,
}

/// 20kHz control loop tick.
///
/// Handles: throttle→setpoint mapping, arming, BEMF polling (old_routine),
/// ramp rate limiting, PWM output.
#[allow(clippy::too_many_arguments)]
pub fn ten_khz_tick(
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    duty: &mut DutyState,
    config: &EepromConfig,
    counters: &mut TickCounters,
    shared: &dyn SharedComm,
    pwm: &mut dyn hal::PwmOutput,
    comp: &mut dyn hal::Comparator,
    phase: &mut dyn hal::PhaseOutput,
    interval: &mut dyn hal::IntervalTimer,
) {
    // Throttle → setpoint
    let newinput = shared.newinput();
    shared.set_adjusted_input(newinput);
    if shared.armed() && !shared.stepper_sine() {
        if newinput >= THROTTLE_MIN_SIGNAL {
            let min_duty = duty.minimum;
            let setpoint = map(
                newinput as i32,
                THROTTLE_MIN_SIGNAL as i32,
                DSHOT_MAX_THROTTLE as i32,
                min_duty as i32,
                DUTY_SCALE_MAX as i32,
            ) as u16;
            shared.set_duty_cycle_setpoint(setpoint);
            if !shared.running() {
                shared.set_running(true);
                duty.last = duty.min_startup;
                let step = commutation.advance();
                phase.com_step(step);
                comp.set_step(step, commutation.rising);
                comp.change_input();
                comp.enable_interrupts();
            }
        } else {
            shared.set_duty_cycle_setpoint(0);
            // Active brake mode 2: hold motor in comStep(2) at fixed power
            if config.brake_on_stop == 2 {
                phase.com_step(2);
                let brake_duty = (config.active_brake_power as u32 * counters.tim1_arr as u32
                    / DUTY_SCALE_MAX as u32)
                    * 10;
                pwm.set_duty_all(brake_duty as u16);
            }
        }
    }

    // Core tick
    let setpoint = shared.duty_cycle_setpoint();
    duty.cycle = setpoint;
    counters.ten_khz_counter += 1;
    shared.increment_signal_timeout();
    duty.ramp_count += 1;
    counters.one_khz_loop_counter += 1;

    // Arming
    if !shared.armed() {
        if shared.input_set() && shared.adjusted_input() == 0 {
            counters.armed_timeout_count += 1;
            if counters.armed_timeout_count > ARMING_TIMEOUT_TICKS {
                shared.set_armed(true);
                counters.armed_timeout_count = 0;
            }
        } else {
            counters.armed_timeout_count = 0;
        }
    }

    // Old routine BEMF polling
    if shared.old_routine() && shared.running() && !shared.stepper_sine() {
        bemf_polling(commutation, bemf, shared, comp, phase, interval);
    }

    // Ramp rate limiting
    ramp_limit(duty, shared, counters.voltage_based_ramp);

    // Apply stall protection boost (crawler/RC car low-RPM boost)
    let stall_boost = shared.stall_protection_adjust();
    if stall_boost > 0 && shared.running() {
        duty.cycle = duty.cycle.saturating_add(stall_boost);
    }

    // PWM output
    let tim1_arr = counters.tim1_arr;
    if shared.armed() && shared.running() {
        let adj = ((duty.cycle as u32 * tim1_arr as u32) / DUTY_SCALE_MAX as u32 + 1) as u16;
        pwm.set_duty_all(adj);
    } else {
        pwm.set_duty_all(0);
    }
    duty.last = duty.cycle;
    pwm.set_auto_reload(tim1_arr);
}

/// BEMF polling (old_routine path). Called from 20kHz tick when old_routine is active.
fn bemf_polling(
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    shared: &dyn SharedComm,
    comp: &mut dyn hal::Comparator,
    phase: &mut dyn hal::PhaseOutput,
    interval: &mut dyn hal::IntervalTimer,
) {
    comp.mask_interrupts();
    let comp_level = comp.output_level();
    let current_state = !comp_level;
    if commutation.rising {
        if current_state {
            bemf.counter += 1;
        } else {
            bemf.bad_count += 1;
            if bemf.bad_count > bemf.bad_count_threshold {
                bemf.counter = 0;
            }
        }
    } else if !current_state {
        bemf.counter += 1;
    } else {
        bemf.bad_count += 1;
        if bemf.bad_count > bemf.bad_count_threshold {
            bemf.counter = 0;
        }
    }
    let threshold = if commutation.rising {
        bemf.min_counts_up
    } else {
        bemf.min_counts_down
    };
    if !bemf.zc_found && bemf.counter > threshold {
        bemf.zc_found = true;
        bemf.this_zc_time = interval.count() as u16;
        interval.set_count(0);
        let ci = shared.commutation_interval();
        let new_ci = (bemf.this_zc_time as u32 + 3 * ci) / 4;
        shared.set_commutation_interval(new_ci);
        let advance = (bemf.temp_advance as u32 * new_ci) >> ADVANCE_SHIFT;
        bemf.wait_time = (new_ci as u16 / 2).wrapping_sub(advance as u16);
        let zc = shared.zero_crosses();
        if zc >= 5 {
            while (interval.count() as u16) < bemf.wait_time {}
        }
        let step = commutation.advance();
        phase.com_step(step);
        comp.set_step(step, commutation.rising);
        comp.change_input();
        bemf.counter = 0;
        bemf.bad_count = 0;
        shared.increment_zero_crosses();
        let zc = shared.zero_crosses();
        let ci = shared.commutation_interval();
        if zc >= OLD_ROUTINE_EXIT_ZC && ci <= OLD_ROUTINE_EXIT_INTERVAL {
            shared.set_old_routine(false);
            comp.enable_interrupts();
        }
    }
}

/// Ramp rate limiting.
fn ramp_limit(duty: &mut DutyState, shared: &dyn SharedComm, voltage_based: bool) {
    if duty.ramp_count > duty.ramp_divider as u16 {
        duty.ramp_count = 0;
        if voltage_based {
            // Scale ramp rate by battery voltage (lower voltage = faster ramp)
            let v_change = map(shared.battery_voltage() as i32, 800, 2200, 10, 1) as u8;
            let ci = shared.commutation_interval();
            duty.max_change = if ci > 200 {
                v_change
            } else {
                v_change.saturating_mul(3)
            };
        } else {
            let zc = shared.zero_crosses();
            if zc < 150 || duty.last < 150 {
                duty.max_change = duty.max_ramp_startup;
            } else if duty.last > 500 {
                duty.max_change = duty.max_ramp_low_rpm;
            } else {
                duty.max_change = duty.max_ramp_high_rpm;
            }
        }
        let change = duty.max_change as u16;
        if duty.cycle > duty.last + change {
            duty.cycle = duty.last + change;
        }
        if duty.last > duty.cycle + change {
            duty.cycle = duty.last - change;
        }
    } else {
        duty.cycle = duty.last;
    }
}

/// Commutation timer expired (TIM14/TIM16 ISR body).
pub fn commutation_timer_expired(
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    shared: &dyn SharedComm,
    com_timer: &mut dyn hal::ComTimer,
    comp: &mut dyn hal::Comparator,
    phase: &mut dyn hal::PhaseOutput,
) {
    com_timer.disable_interrupt();
    let step = commutation.advance();
    phase.com_step(step);
    comp.set_step(step, commutation.rising);
    comp.change_input();
    let zc_avg = (bemf.last_zc_time as u32 + bemf.this_zc_time as u32) >> 1;
    let ci = shared.commutation_interval();
    let new_ci = (ci + zc_avg) >> 1;
    shared.set_commutation_interval(new_ci);
    let advance = (new_ci * bemf.temp_advance as u32) >> ADVANCE_SHIFT;
    bemf.wait_time = (new_ci as u16 >> 1).wrapping_sub(advance as u16);
    comp.enable_interrupts();
    shared.increment_zero_crosses();
    bemf.counter = 0;
    bemf.zc_found = false;
}

/// BEMF zero-cross detected (COMP ISR body).
pub fn bemf_zero_cross(
    commutation: &Commutation,
    bemf: &mut BemfState,
    comp: &mut dyn hal::Comparator,
    interval: &mut dyn hal::IntervalTimer,
    com_timer: &mut dyn hal::ComTimer,
) {
    for _ in 0..bemf.filter_level {
        if comp.output_level() == commutation.rising {
            return;
        }
    }
    comp.mask_interrupts();
    bemf.last_zc_time = bemf.this_zc_time;
    bemf.this_zc_time = interval.count() as u16;
    interval.set_count(0);
    com_timer.set_and_enable(bemf.wait_time + 1);
}
