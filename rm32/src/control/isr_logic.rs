//! ISR-level control logic — platform-independent, fully testable.
//!
//! All functions take `MotorContext<S, H>` for static dispatch.
//! No `&dyn` trait objects — the compiler monomorphizes to concrete MCU types,
//! eliminating vtable overhead in the 20kHz ISR.

use crate::commutation::Commutation;
use crate::constants::*;
use crate::control::context::MotorContext;
use crate::control::state::{BemfState, DutyState};
use crate::hal::{self, BemfSensor, Bridge, ComTimer, IntervalTimer, MotorHal, PhaseOutput};
use crate::motor_mode::MotorEvent;
use crate::shared_comm::{IsrAction, SharedComm};

const COMMUTATE_KICK_DELAY_TICKS: u16 = 1;

/// Control loop tick at `H::TICK_HZ` (20 kHz on STM32).
///
/// Handles: throttle→setpoint mapping, arming, BEMF polling (old_routine),
/// ramp rate limiting, PWM output.
pub fn ten_khz_tick<S: SharedComm, H: MotorHal>(ctx: &mut MotorContext<S, H>) {
    ctx.shared.increment_control_ticks();
    ctx.shared.one_khz_counter_inc();

    if ctx.config.telemetry_on_interval != 0 {
        let limit = telemetry_interval_ticks::<H>(ctx.config.telemetry_on_interval);
        if ctx.shared.telem_counter_check_and_inc(limit) {
            ctx.shared.set_send_telemetry(true);
        }
    }

    if !ctx.shared.running() {
        // Keep COMP masked while phases are not commutating.
        ctx.hal.bemf().mask_irq();
    }

    let action = ctx.shared.isr_action();
    let action_interval_count = ctx.hal.interval().count();
    match action {
        IsrAction::AllOff => {
            ctx.duty.sine_outputs_on = false;
            ctx.duty.active_brake_on = false;
            ctx.shared.set_changeover_step(0);
            ctx.hal.com_timer().disable_interrupt();
            ctx.hal.bridge().all_off();
            ctx.hal.bridge().set_duty_all(0);
            ctx.hal.bridge().latch();
            ctx.hal.bemf().mask_irq();
            ctx.shared.clear_isr_action(IsrAction::AllOff);
            return;
        }
        IsrAction::ResetIntervalTimer => {
            ctx.hal.interval().set_count(0);
            ctx.shared.clear_isr_action(IsrAction::ResetIntervalTimer);
        }
        IsrAction::CommutateKick => {
            if ctx.shared.running() {
                let count = action_interval_count.min(u16::MAX as u32) as u16;
                let ci = ctx.shared.commutation_interval();
                let new_ci = ctx.bemf.record_zero_cross(count, ci);
                ctx.shared.set_commutation_interval(new_ci);
                ctx.hal
                    .com_timer()
                    .set_and_enable(COMMUTATE_KICK_DELAY_TICKS);
            }
            ctx.hal.interval().set_count(0);
            ctx.shared.clear_isr_action(IsrAction::CommutateKick);
        }
        IsrAction::None => {}
    }

    // Direction must be current before deriving the handoff comparator edge.
    ctx.commutation.forward = ctx.shared.forward();
    let input = ctx.shared.adjusted_input();
    let changeover = ctx.shared.changeover_step();
    let handoff = (1..=6).contains(&changeover)
        && ctx.shared.stepper_sine()
        && input > SINE_CHANGEOVER_THROTTLE;
    if handoff {
        ctx.hal.com_timer().disable_interrupt();
        ctx.hal.bemf().mask_irq();
        ctx.hal.bridge().set_duty_all(0);
        ctx.hal.bridge().latch();
        ctx.commutation.set_step(changeover);
        ctx.hal.bridge().com_step(changeover);
        let ci = ctx.shared.commutation_interval();
        ctx.hal.interval().set_count(ci);
        ctx.bemf.reset_for_step();
        ctx.hal.bemf().select_phase(
            crate::hal::floating_phase(changeover),
            ctx.commutation.rising,
        );
        ctx.shared.transition(MotorEvent::ExitSine);
        ctx.hal.com_timer().set_and_enable(ci as u16);
    }
    // A disarm or throttle release cancels a queued handoff.
    ctx.shared.set_changeover_step(0);

    // Sync direction from shared (main loop may flip for bidirectional)
    ctx.commutation.forward = ctx.shared.forward();
    let tim1_arr = ctx.shared.tim1_arr();

    // Throttle → setpoint
    // Read adjusted_input (set by process_input: bidir-mapped or raw passthrough)
    let sine_start = ctx.config.use_sine_start != 0;
    let start_threshold = if sine_start {
        SINE_ENTRY_THROTTLE
    } else {
        THROTTLE_MIN_SIGNAL
    };
    if ctx.shared.armed() && sine_start && input < start_threshold {
        if !ctx.shared.stepper_sine() {
            // Do not hold a driven six-step sector while main has yet to enter sine.
            ctx.hal.bridge().all_off();
            ctx.hal.bridge().set_duty_all(0);
            ctx.hal.bridge().latch();
            ctx.duty.sine_outputs_on = false;
        }
        ctx.shared.transition(MotorEvent::StopMotor);
        ctx.shared.transition(MotorEvent::EnterSine);
        ctx.shared.set_duty_cycle_setpoint(0);
        ctx.shared.set_zero_crosses(0);
        ctx.bemf.reset_for_step();
        ctx.hal.com_timer().disable_interrupt();
        ctx.hal.bemf().mask_irq();
    }
    if ctx.shared.armed() && !ctx.shared.stepper_sine() {
        if input >= start_threshold {
            let setpoint = ctx.duty.compute_setpoint(
                input,
                ctx.shared.zero_crosses(),
                ctx.config.stall_protection,
                sine_start,
            );
            ctx.shared.set_duty_cycle_setpoint(setpoint);
            if !ctx.shared.running() {
                ctx.hal.bridge().all_off();
                if ctx.duty.active_brake_on {
                    // Do not apply the old brake compare to the restart sector.
                    ctx.hal.bridge().set_duty_all(0);
                    ctx.hal.bridge().latch();
                    ctx.duty.active_brake_on = false;
                }
                ctx.shared.transition(MotorEvent::StartMotor);
                ctx.duty.start_motor();
                let step = ctx.commutation.advance();
                let e_com = ctx
                    .commutation
                    .record_interval::<H>(ctx.shared.commutation_interval() as u16);
                ctx.shared.set_e_com_time(e_com);
                ctx.hal.bridge().com_step(step);
                ctx.hal
                    .bemf()
                    .select_phase(crate::hal::floating_phase(step), ctx.commutation.rising);
                ctx.shared
                    .set_commutation_interval(H::STARTUP_COMMUTATION_INTERVAL);
                ctx.hal
                    .interval()
                    .set_count(H::STARTUP_COMMUTATION_INTERVAL / 2);
            }
        } else {
            ctx.shared.set_duty_cycle_setpoint(0);
            if !ctx.shared.running() {
                // Match AM32 idle housekeeping: stale run counters must not carry
                // into the next startup attempt.
                ctx.shared.set_zero_crosses(0);
                ctx.bemf.reset_for_step();
            }
        }
    }

    // Core tick
    let setpoint = ctx.shared.duty_cycle_setpoint();
    ctx.duty.set_cycle(setpoint);
    ctx.shared.increment_signal_timeout();
    ctx.duty.increment_ramp_count();

    // Arming
    if !ctx.shared.armed() {
        if ctx.shared.input_set() && ctx.shared.adjusted_input() == 0 {
            *ctx.armed_timeout_count += 1;
            if *ctx.armed_timeout_count > H::ARMING_TIMEOUT_TICKS {
                ctx.shared.transition(MotorEvent::Arm);
                *ctx.armed_timeout_count = 0;
            }
        } else {
            *ctx.armed_timeout_count = 0;
        }
    }

    // Old routine BEMF polling
    if !handoff && ctx.shared.old_routine() && ctx.shared.running() && !ctx.shared.stepper_sine() {
        bemf_polling(ctx);
    }

    // Ramp rate limiting
    let average_interval =
        crate::platform::e_com_us_to_avg_ticks::<H>(ctx.shared.e_com_time()) as u32;
    ctx.duty.ramp_limit::<H>(
        ctx.shared.battery_voltage(),
        ctx.shared.commutation_interval(),
        ctx.shared.zero_crosses(),
        average_interval,
        ctx.voltage_based_ramp,
    );

    // Sync main→ISR published state (main computes, ISR applies)
    ctx.bemf.sync_config(
        ctx.shared.filter_level(),
        ctx.shared.auto_advance(),
        ctx.shared.min_bemf_counts(),
    );

    // Apply stall boost + duty/current ceilings
    let stall_boost = if ctx.shared.running() {
        ctx.shared.stall_protection_adjust()
    } else {
        0
    };
    ctx.duty.clamp_ceilings(
        stall_boost,
        ctx.shared.duty_maximum(),
        ctx.shared.current_limit_adjust(),
    );

    // PWM output
    let sine_drive = ctx.shared.stepper_sine() && input > THROTTLE_MIN_SIGNAL;
    if !sine_drive {
        ctx.duty.sine_outputs_on = false;
    }
    let active_brake = ctx.shared.armed()
        && !ctx.shared.running()
        && input < start_threshold
        && !sine_drive
        && ctx.config.brake_on_stop == 2;
    if !active_brake {
        ctx.duty.active_brake_on = false;
    }
    let tone_owns_bridge = ctx.shared.tone_active()
        && !crate::tone::bridge_busy(ctx.shared, active_brake || ctx.duty.active_brake_on);
    if tone_owns_bridge {
        // Runtime tone output is owned by the control-tick scheduler. Skipping
        // the normal idle PWM writes keeps AllOff and braking from muting it.
    } else if sine_drive {
        // Main owns the individual phase compares until the handoff is consumed.
    } else if active_brake {
        if !ctx.duty.active_brake_on {
            ctx.hal.com_timer().disable_interrupt();
            ctx.hal.bemf().mask_irq();
            ctx.hal.bridge().set_duty_all(0);
            ctx.hal.bridge().latch();
            ctx.hal.bridge().com_step(2);
            ctx.duty.active_brake_on = true;
        }
        let brake_duty =
            (ctx.config.active_brake_power as u32 * tim1_arr as u32 / DUTY_SCALE_MAX as u32) * 10;
        ctx.hal
            .bridge()
            .set_duty_all(brake_duty.min(tim1_arr as u32) as u16);
    } else if ctx.shared.armed() && ctx.shared.running() {
        ctx.hal
            .bridge()
            .set_duty_all(ctx.duty.pwm_compare(tim1_arr));
    } else if ctx.shared.prop_brake_active() {
        ctx.hal.bridge().proportional_brake();
        ctx.hal.bridge().set_duty_all(DutyState::brake_compare(
            ctx.config.drag_brake_strength,
            tim1_arr,
        ));
    } else {
        ctx.hal.bridge().all_off();
        ctx.hal.bridge().set_duty_all(0);
    }
    let final_duty = ctx.duty.finalize();
    ctx.shared.set_duty_cycle(final_duty);
    if !tone_owns_bridge {
        ctx.hal.bridge().set_period(tim1_arr);
    }

    // Sync ISR→shared (Commutation owns truth, shared publishes for main loop)
    ctx.shared.set_forward(ctx.commutation.forward);
    ctx.shared
        .set_interval_timer_count(ctx.hal.interval().count());
}

/// BEMF polling (old_routine path).
fn bemf_polling<S: SharedComm, H: MotorHal>(ctx: &mut MotorContext<S, H>) {
    ctx.hal.bemf().mask_irq();
    let comp_level = ctx.hal.bemf().level();
    let rising = ctx.commutation.rising;
    ctx.bemf.update(comp_level, rising);

    if ctx.bemf.zero_cross_detected(rising) {
        let interval_count = ctx.hal.interval().count() as u16;
        ctx.hal.interval().set_count(0);
        let ci = ctx.shared.commutation_interval();
        let new_ci = ctx.bemf.record_zero_cross(interval_count, ci);
        ctx.shared.set_commutation_interval(new_ci);

        if ctx.shared.zero_crosses() < MIN_ZC_FOR_ADVANCE {
            let step = ctx.commutation.advance();
            let e_com = ctx
                .commutation
                .record_interval::<H>(ctx.shared.commutation_interval() as u16);
            ctx.shared.set_e_com_time(e_com);
            ctx.hal.bridge().com_step(step);
            ctx.hal.bridge().pulse_toggle(step);
            ctx.hal
                .bemf()
                .select_phase(crate::hal::floating_phase(step), ctx.commutation.rising);
            ctx.bemf.reset_for_step();
            ctx.shared.increment_zero_crosses();
        } else {
            ctx.hal
                .com_timer()
                .set_and_enable(ctx.bemf.com_timer_delay());
        }
    }
}

/// Commutation timer expired (TIM14/TIM16 ISR body).
#[allow(clippy::too_many_arguments)]
pub fn commutation_timer_expired<P: Timing, S, C, Ph, T>(
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    shared: &S,
    com_timer: &mut T,
    comp: &mut C,
    phase: &mut Ph,
    bidirectional: bool,
    strict_changeover: bool,
) where
    S: SharedComm,
    C: hal::BemfSensor,
    Ph: hal::PhaseOutput,
    T: hal::ComTimer,
{
    com_timer.disable_interrupt();
    if !shared.running() {
        comp.mask_irq();
        return;
    }

    let step = commutation.advance();
    if commutation.desync_check() {
        shared.set_desync_check_pending(true);
        commutation.set_desync_check(false);
    }
    let e_com = commutation.record_interval::<P>(shared.commutation_interval() as u16);
    shared.set_e_com_time(e_com);
    phase.com_step(step);
    phase.pulse_toggle(step);
    comp.select_phase(crate::hal::floating_phase(step), commutation.rising);

    let exit_interval = if bidirectional {
        P::OLD_ROUTINE_EXIT_INTERVAL / 2
    } else {
        P::OLD_ROUTINE_EXIT_INTERVAL
    };
    let was_interrupt_mode = !shared.old_routine();

    if was_interrupt_mode
        && crate::platform::e_com_us_to_avg_ticks::<P>(e_com) as u32
            > exit_interval + P::OLD_ROUTINE_REENTRY_MARGIN
    {
        shared.set_old_routine(true);
    }

    if was_interrupt_mode {
        let new_ci = bemf.update_timing_from_timer(shared.commutation_interval());
        shared.set_commutation_interval(new_ci);
    }

    if !shared.old_routine() {
        comp.enable_irq();
    }
    bemf.reset_after_commutation();
    shared.increment_zero_crosses();

    let zc = shared.zero_crosses();
    let ci = shared.commutation_interval();
    let exit_interval = if bidirectional {
        P::OLD_ROUTINE_EXIT_INTERVAL / 2
    } else {
        P::OLD_ROUTINE_EXIT_INTERVAL
    };
    let changeover_met = if strict_changeover {
        zc >= OLD_ROUTINE_EXIT_ZC && ci <= exit_interval
    } else {
        ci < exit_interval
    };
    if !was_interrupt_mode && shared.old_routine() && changeover_met {
        shared.transition(MotorEvent::BemfLocked);
        comp.enable_irq();
    }
}

/// BEMF zero-cross detected (COMP ISR body).
pub fn bemf_zero_cross<
    S: SharedComm,
    C: hal::BemfSensor,
    I: hal::IntervalTimer,
    T: hal::ComTimer,
>(
    shared: &S,
    commutation: &Commutation,
    bemf: &mut BemfState,
    comp: &mut C,
    interval: &mut I,
    com_timer: &mut T,
) -> bool {
    // An IRQ already pending when sine/polling masks the comparator must not
    // restart the commutation timer or consume a polling-mode zero crossing.
    if !shared.running() || shared.old_routine() {
        comp.mask_irq();
        return false;
    }
    for _ in 0..bemf.filter_level() {
        if comp.filter_read() == commutation.rising() {
            return false;
        }
    }
    comp.mask_irq();
    let count = interval.count() as u16;
    interval.set_count(0);
    bemf.record_zc_timing(count);
    com_timer.set_and_enable(bemf.com_timer_delay());
    true
}
