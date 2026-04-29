//! ISR-level control logic — platform-independent, fully testable.
//!
//! All functions take `MotorContext<S, H>` for static dispatch.
//! No `&dyn` trait objects — the compiler monomorphizes to concrete MCU types,
//! eliminating vtable overhead in the 20kHz ISR.

use crate::commutation::Commutation;
use crate::constants::*;
use crate::control::context::MotorContext;
use crate::control::state::{BemfState, DutyState};
use crate::functions::map;
use crate::hal::{self, ComTimer, Comparator, IntervalTimer, MotorHal, PhaseOutput, PwmOutput};
use crate::motor_mode::MotorEvent;
use crate::shared_comm::SharedComm;

/// Counters and config owned exclusively by the ISR tick.
pub struct TickCounters {
    pub ten_khz_counter: u32,
    pub one_khz_loop_counter: u16,
    pub armed_timeout_count: u32,
    pub tim1_arr: u16,
    pub voltage_based_ramp: bool,
}

/// 20kHz control loop tick.
///
/// Handles: throttle→setpoint mapping, arming, BEMF polling (old_routine),
/// ramp rate limiting, PWM output.
pub fn ten_khz_tick<S: SharedComm, H: MotorHal>(ctx: &mut MotorContext<S, H>) {
    // Sync direction from shared (main loop may flip for bidirectional)
    ctx.commutation.forward = ctx.shared.forward();

    // Throttle → setpoint
    // Read adjusted_input (set by process_input: bidir-mapped or raw passthrough)
    let input = ctx.shared.adjusted_input();
    if ctx.shared.armed() && !ctx.shared.stepper_sine() {
        if input >= THROTTLE_MIN_SIGNAL {
            let min_duty = ctx.duty.minimum;
            let setpoint = map(
                input as i32,
                THROTTLE_MIN_SIGNAL as i32,
                DSHOT_MAX_THROTTLE as i32,
                min_duty as i32,
                DUTY_SCALE_MAX as i32,
            ) as u16;
            ctx.shared.set_duty_cycle_setpoint(setpoint);
            if !ctx.shared.running() {
                ctx.shared.transition(MotorEvent::StartMotor);
                ctx.duty.last = ctx.duty.min_startup;
                let step = ctx.commutation.advance();
                ctx.hal.phase().com_step(step);
                ctx.hal.comp().set_step(step, ctx.commutation.rising);
                ctx.hal.comp().change_input();
                ctx.hal.comp().enable_interrupts();
            }
        } else {
            ctx.shared.set_duty_cycle_setpoint(0);
            if ctx.config.brake_on_stop == 2 {
                ctx.hal.phase().com_step(2);
                let brake_duty = (ctx.config.active_brake_power as u32
                    * ctx.counters.tim1_arr as u32
                    / DUTY_SCALE_MAX as u32)
                    * 10;
                ctx.hal.pwm().set_duty_all(brake_duty as u16);
            }
        }
    }

    // Core tick
    let setpoint = ctx.shared.duty_cycle_setpoint();
    ctx.duty.cycle = setpoint;
    ctx.counters.ten_khz_counter += 1;
    ctx.shared.increment_signal_timeout();
    ctx.duty.ramp_count += 1;
    ctx.counters.one_khz_loop_counter += 1;

    // Arming
    if !ctx.shared.armed() {
        if ctx.shared.input_set() && ctx.shared.adjusted_input() == 0 {
            ctx.counters.armed_timeout_count += 1;
            if ctx.counters.armed_timeout_count > ARMING_TIMEOUT_TICKS {
                ctx.shared.transition(MotorEvent::Arm);
                ctx.counters.armed_timeout_count = 0;
            }
        } else {
            ctx.counters.armed_timeout_count = 0;
        }
    }

    // Old routine BEMF polling
    if ctx.shared.old_routine() && ctx.shared.running() && !ctx.shared.stepper_sine() {
        bemf_polling(ctx);
    }

    // Ramp rate limiting
    ramp_limit(ctx.duty, ctx.shared, ctx.counters.voltage_based_ramp);

    // Apply stall protection boost
    let stall_boost = ctx.shared.stall_protection_adjust();
    if stall_boost > 0 && ctx.shared.running() {
        ctx.duty.cycle = ctx.duty.cycle.saturating_add(stall_boost);
    }

    // Sync main→ISR published state (main computes, ISR applies)
    ctx.counters.tim1_arr = ctx.shared.tim1_arr();
    ctx.duty.maximum = ctx.shared.duty_maximum();
    ctx.bemf.filter_level = ctx.shared.filter_level();
    ctx.bemf.temp_advance = ctx.shared.auto_advance();
    let min_counts = ctx.shared.min_bemf_counts();
    ctx.bemf.min_counts_up = min_counts;
    ctx.bemf.min_counts_down = min_counts;

    // Enforce duty ceiling (eRPM/temperature protection)
    if ctx.duty.cycle > ctx.duty.maximum {
        ctx.duty.cycle = ctx.duty.maximum;
    }

    // PWM output
    let tim1_arr = ctx.counters.tim1_arr;
    if ctx.shared.armed() && ctx.shared.running() {
        let adj = ((ctx.duty.cycle as u32 * tim1_arr as u32) / DUTY_SCALE_MAX as u32 + 1) as u16;
        ctx.hal.pwm().set_duty_all(adj);
    } else {
        ctx.hal.pwm().set_duty_all(0);
    }
    ctx.duty.last = ctx.duty.cycle;
    ctx.shared.set_duty_cycle(ctx.duty.cycle);
    ctx.hal.pwm().set_auto_reload(tim1_arr);

    // Sync ISR→shared direction (Commutation owns truth, shared publishes for main loop)
    ctx.shared.set_forward(ctx.commutation.forward);
}

/// BEMF polling (old_routine path).
fn bemf_polling<S: SharedComm, H: MotorHal>(ctx: &mut MotorContext<S, H>) {
    ctx.hal.comp().mask_interrupts();
    let comp_level = ctx.hal.comp().output_level();
    let current_state = !comp_level;
    if ctx.commutation.rising {
        if current_state {
            ctx.bemf.counter += 1;
        } else {
            ctx.bemf.bad_count += 1;
            if ctx.bemf.bad_count > ctx.bemf.bad_count_threshold {
                ctx.bemf.counter = 0;
            }
        }
    } else if !current_state {
        ctx.bemf.counter += 1;
    } else {
        ctx.bemf.bad_count += 1;
        if ctx.bemf.bad_count > ctx.bemf.bad_count_threshold {
            ctx.bemf.counter = 0;
        }
    }
    let threshold = if ctx.commutation.rising {
        ctx.bemf.min_counts_up
    } else {
        ctx.bemf.min_counts_down
    };
    if !ctx.bemf.zc_found && ctx.bemf.counter > threshold {
        ctx.bemf.zc_found = true;
        ctx.bemf.last_zc_time = ctx.bemf.this_zc_time;
        ctx.bemf.this_zc_time = ctx.hal.interval().count() as u16;
        ctx.hal.interval().set_count(0);
        let ci = ctx.shared.commutation_interval();
        let new_ci = (ctx.bemf.this_zc_time as u32 + 3 * ci) / 4;
        ctx.shared.set_commutation_interval(new_ci);
        let advance = (ctx.bemf.temp_advance as u32 * new_ci) >> ADVANCE_SHIFT;
        ctx.bemf.wait_time = (new_ci as u16 / 2).wrapping_sub(advance as u16);
        let zc = ctx.shared.zero_crosses();
        if zc < MIN_ZC_FOR_ADVANCE {
            let step = ctx.commutation.advance();
            ctx.hal.phase().com_step(step);
            ctx.hal.phase().pulse_toggle(step);
            ctx.hal.comp().set_step(step, ctx.commutation.rising);
            ctx.hal.comp().change_input();
            ctx.bemf.counter = 0;
            ctx.bemf.bad_count = 0;
            ctx.shared.increment_zero_crosses();
        } else {
            ctx.hal.com_timer().set_and_enable(ctx.bemf.wait_time + 1);
        }
    }
}

/// Ramp rate limiting.
fn ramp_limit<S: SharedComm>(duty: &mut DutyState, shared: &S, voltage_based: bool) {
    if duty.ramp_count > duty.ramp_divider as u16 {
        duty.ramp_count = 0;
        if voltage_based {
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
pub fn commutation_timer_expired<S, C, Ph, T>(
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    shared: &S,
    com_timer: &mut T,
    comp: &mut C,
    phase: &mut Ph,
) where
    S: SharedComm,
    C: hal::Comparator,
    Ph: hal::PhaseOutput,
    T: hal::ComTimer,
{
    com_timer.disable_interrupt();
    let step = commutation.advance();
    phase.com_step(step);
    phase.pulse_toggle(step);
    comp.set_step(step, commutation.rising);
    comp.change_input();

    if !shared.old_routine() {
        let zc_avg = (bemf.last_zc_time as u32 + bemf.this_zc_time as u32) >> 1;
        let ci = shared.commutation_interval();
        let new_ci = (ci + zc_avg) >> 1;
        shared.set_commutation_interval(new_ci);
        let advance = (new_ci * bemf.temp_advance as u32) >> ADVANCE_SHIFT;
        bemf.wait_time = (new_ci as u16 >> 1).wrapping_sub(advance as u16);
    }

    comp.enable_interrupts();
    bemf.counter = 0;
    bemf.bad_count = 0;
    bemf.zc_found = false;
    shared.increment_zero_crosses();

    let zc = shared.zero_crosses();
    let ci = shared.commutation_interval();
    if shared.old_routine() && zc >= OLD_ROUTINE_EXIT_ZC && ci <= OLD_ROUTINE_EXIT_INTERVAL {
        shared.transition(MotorEvent::BemfLocked);
    }
}

/// BEMF zero-cross detected (COMP ISR body).
pub fn bemf_zero_cross<C: hal::Comparator, I: hal::IntervalTimer, T: hal::ComTimer>(
    commutation: &Commutation,
    bemf: &mut BemfState,
    comp: &mut C,
    interval: &mut I,
    com_timer: &mut T,
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
