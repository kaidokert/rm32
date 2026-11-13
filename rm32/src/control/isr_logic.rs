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

/// True when the motor is in steady-state interrupt-driven six-step mode
/// (`MotorMode::Running`) with no pending one-shot ISR action, sine handoff,
/// or sub-threshold throttle stop.
///
/// On this fast path, the control tick's setpoint, ramp-limit, duty-clamp,
/// and TIM1 compare/period updates touch only `DutyState` and the PWM timer,
/// allowing the priority-2/3 `TIM6` handler to run them with interrupts
/// unmasked (`PRIMASK = 0`) so priority-0 `COMP` and commutation-timer ISRs
/// can preempt mid-tick without latency jitter.
#[inline]
pub fn is_steady_running_tick<S: SharedComm>(
    shared: &S,
    config: &crate::config::EepromConfig,
) -> bool {
    let sine_start = config.use_sine_start != 0;
    let start_threshold = if sine_start {
        SINE_ENTRY_THROTTLE
    } else {
        THROTTLE_MIN_SIGNAL
    };
    shared.running()
        && !shared.old_routine()
        && !shared.stepper_sine()
        && !shared.tone_active()
        && shared.isr_action() == IsrAction::None
        && shared.changeover_step() == 0
        && shared.adjusted_input() >= start_threshold
}

/// Unmasked portion of a steady-state `Running` control tick.
///
/// Requires `is_steady_running_tick(shared, config)` at entry. Must be paired
/// with `steady_running_tick_sync_com` inside a brief critical section to
/// synchronize direction, BEMF filter settings, and the published interval
/// timer count with the priority-0 commutation/comparator state.
pub fn steady_running_tick_unmasked<P, S, F>(
    shared: &S,
    duty: &mut DutyState,
    config: &crate::config::EepromConfig,
    voltage_based_ramp: bool,
    apply_pwm: F,
) where
    P: Timing,
    S: SharedComm,
    F: FnOnce(u16, u16),
{
    shared.increment_control_ticks();
    shared.one_khz_counter_inc();

    if config.telemetry_on_interval != 0 {
        let limit = telemetry_interval_ticks::<P>(config.telemetry_on_interval);
        if shared.telem_counter_check_and_inc(limit) {
            shared.set_send_telemetry(true);
        }
    }

    shared.set_changeover_step(0);
    let tim1_arr = shared.tim1_arr();
    let input = shared.adjusted_input();
    let sine_start = config.use_sine_start != 0;

    let setpoint = duty.compute_setpoint(
        input,
        shared.zero_crosses(),
        config.stall_protection,
        sine_start,
    );
    shared.set_duty_cycle_setpoint(setpoint);

    // Running is guaranteed here, as `ten_khz_tick`'s kick block requires.
    let kick_down = shared.take_duty_kick_down();
    let kick_half = shared.take_duty_kick_half();
    if kick_down {
        duty.kick_down();
    } else if kick_half {
        duty.kick_half();
    }

    duty.set_cycle(setpoint);
    shared.increment_signal_timeout();
    duty.increment_ramp_count();

    let average_interval = crate::platform::e_com_us_to_avg_ticks::<P>(shared.e_com_time());
    shared.set_gate_avg(average_interval.max(0) as u32);
    duty.ramp_limit::<P>(
        shared.battery_voltage(),
        shared.commutation_interval(),
        shared.zero_crosses(),
        average_interval as u32,
        voltage_based_ramp,
    );

    duty.clamp_ceilings(
        shared.stall_protection_adjust(),
        shared.duty_maximum(),
        shared.current_limit_adjust(),
    );

    duty.sine_outputs_on = false;
    duty.active_brake_on = false;
    let compare = duty.pwm_compare(tim1_arr);
    let final_duty = duty.finalize();
    shared.set_duty_cycle(final_duty);
    // Re-check `isr_action` and `tone_active` before touching PWM hardware in
    // case an input ISR preempted the unmasked calculation and published a stop
    // (`IsrAction::AllOff`) or tone state changed.
    if !shared.tone_active() && shared.isr_action() == IsrAction::None {
        apply_pwm(compare, tim1_arr);
    }
}

/// Priority-0 state synchronization for a steady-state `Running` control tick.
///
/// Called inside a brief interrupt-masked critical section by the `TIM6`
/// handler to sync direction, BEMF filter parameters, and the interval timer
/// snapshot without holding `PRIMASK = 1` across ramp/duty calculations.
#[inline]
pub fn steady_running_tick_sync_com<S, I>(
    shared: &S,
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    interval: &I,
) where
    S: SharedComm,
    I: IntervalTimer,
{
    commutation.forward = shared.forward();
    bemf.sync_config(
        shared.filter_level(),
        shared.auto_advance(),
        shared.min_bemf_counts(),
    );
    shared.set_forward(commutation.forward);
    shared.set_interval_timer_count(interval.count());
}

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
            ctx.shared.take_duty_kick_down();
            ctx.shared.take_duty_kick_half();
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
        ctx.hal.bridge().com_step(changeover, ctx.config.pwm_mode());
        let ci = ctx.shared.commutation_interval();
        ctx.hal.interval().set_count(ci);
        ctx.bemf.reset_for_step();
        route_bemf(ctx.hal.bemf(), ctx.commutation);
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
                ctx.hal.bridge().com_step(step, ctx.config.pwm_mode());
                route_bemf(ctx.hal.bemf(), ctx.commutation);
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

    // Apply this after startup handling: a desync can stop the motor to Armed,
    // and `start_motor` must not overwrite the recovery ramp origin.
    // Keep it separate from the prioritized timer action so both can run.
    let kick_down = ctx.shared.take_duty_kick_down();
    let kick_half = ctx.shared.take_duty_kick_half();
    if ctx.shared.running() {
        if kick_down {
            ctx.duty.kick_down();
        } else if kick_half {
            ctx.duty.kick_half();
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

    // Ramp rate limiting. The average is read here, after `bemf_polling`
    // may have updated e_com this tick, and is also the COMP gate's latch
    // (see `MotorStatus::gate_avg`): one division per tick, clamped at 0 only.
    let average_interval = crate::platform::e_com_us_to_avg_ticks::<H>(ctx.shared.e_com_time());
    ctx.shared.set_gate_avg(average_interval.max(0) as u32);
    ctx.duty.ramp_limit::<H>(
        ctx.shared.battery_voltage(),
        ctx.shared.commutation_interval(),
        ctx.shared.zero_crosses(),
        average_interval as u32,
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
            ctx.hal.bridge().com_step(2, ctx.config.pwm_mode());
            ctx.duty.active_brake_on = true;
        }
        // AM32 main.c:1936.
        let brake_duty =
            crate::fast_math::div2000_u32(ctx.config.active_brake_power as u32 * tim1_arr as u32)
                * 10;
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

/// Route the comparator to the phase floating in the current commutation
/// step, armed for its BEMF edge. Out of line, with the port's
/// `select_phase` inlined into this one copy so callers pass two pointers
/// rather than duplicating the phase table at each call site.
#[inline(never)]
fn route_bemf<B: BemfSensor>(sensor: &mut B, commutation: &Commutation) {
    sensor.select_phase(hal::floating_phase(commutation.step), commutation.rising);
}

/// What `on_bemf_irq` did with a comparator interrupt.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BemfIrq {
    /// No edge pending (shared vector, or already handled): nothing done.
    Spurious,
    /// Gate open: pending cleared; the caller runs `bemf_zero_cross`.
    Open,
    /// Gate closed, comparator at the pre-ZC level: a noise blip, cleared
    /// (re-raised if the level flipped during the clear).
    Noise,
    /// Gate closed, comparator at the post-ZC level: left pending, so the
    /// interrupt tail-chains at priority 0 until the free-running interval
    /// timer crosses `gate_avg / 2` (exact AM32 `COMP_IRQHandler` behavior;
    /// masking EXTI instead would defer re-enable to the 50 us `TIM6` tick
    /// and add up to 50 us of commutation phase lag).
    Camp,
}

/// The COMP interrupt's acceptance gate (AM32 `COMP_IRQHandler`,
/// `stm32l4xx_it.c` / `stm32g0xx_it.c`): the gate opens once `gate_avg > 0`
/// and the interval timer has run past half the control tick's latched average
/// (`MotorStatus::gate_avg`). Gate open on an edge -> `Open` (the caller then
/// runs `bemf_zero_cross`, which masks on accept and stays armed on a
/// persistence reject). Closed -> `Noise` or `Camp`.
///
/// The pre-ZC re-read after the clear (`repend`) closes the edge-swallow
/// race where a real crossing landing between the level read and the clear
/// would otherwise be cleared with the noise blip.
#[inline(always)]
pub fn on_bemf_irq<S, B, I, E>(
    shared: &S,
    sensor: &mut B,
    interval: &I,
    rising: bool,
    entered: E,
) -> BemfIrq
where
    S: crate::shared_comm::MotorStatus,
    B: BemfSensor,
    I: IntervalTimer,
    E: FnOnce(),
{
    if !sensor.pending() {
        return BemfIrq::Spurious;
    }
    entered();
    let gate_avg = shared.gate_avg();
    if gate_avg == 0 {
        sensor.clear_pending();
        return BemfIrq::Noise;
    }
    if interval.count() > gate_avg >> 1 {
        sensor.clear_pending();
        BemfIrq::Open
    } else if sensor.level() == rising {
        sensor.clear_pending();
        if sensor.level() != rising {
            sensor.repend();
        }
        BemfIrq::Noise
    } else {
        BemfIrq::Camp
    }
}

/// BEMF polling (old_routine path).
fn bemf_polling<S: SharedComm, H: MotorHal>(ctx: &mut MotorContext<S, H>) {
    ctx.hal.bemf().mask_irq();
    let comp_level = ctx.hal.bemf().level();
    let rising = ctx.commutation.rising;
    ctx.bemf.update(comp_level, rising);

    if ctx.bemf.zero_cross_detected(rising) {
        // Saturate, as the action path does above: a timer wider than 16 bits
        // must not wrap a long interval into a short one.
        let interval_count = ctx.hal.interval().count().min(u16::MAX as u32) as u16;
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
            ctx.hal.bridge().com_step(step, ctx.config.pwm_mode());
            ctx.hal.bridge().pulse_toggle(step);
            route_bemf(ctx.hal.bemf(), ctx.commutation);
            ctx.bemf.reset_for_step();
            ctx.shared.increment_zero_crosses();
        } else {
            ctx.hal
                .com_timer()
                .set_and_enable(ctx.bemf.com_timer_delay());
        }
    }
}

/// Commutation timer expired (the commutation-timer ISR body).
#[allow(clippy::too_many_arguments)]
pub fn commutation_timer_expired<P, S, C, B, T, W>(
    commutation: &mut Commutation,
    bemf: &mut BemfState,
    shared: &S,
    writer: &W,
    com_timer: &mut T,
    comp: &mut C,
    bridge: &mut B,
    pwm_mode: hal::PwmMode,
    bidirectional: bool,
    strict_changeover: bool,
) where
    P: Timing,
    S: SharedComm,
    C: hal::BemfSensor,
    B: hal::PhaseOutput,
    T: hal::ComTimer,
    W: crate::shared_comm::ModeWriter,
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
    let ci0 = shared.commutation_interval();
    let e_com = commutation.record_interval::<P>(ci0 as u16);
    shared.set_e_com_time(e_com);
    bridge.com_step(step, pwm_mode);
    bridge.pulse_toggle(step);
    route_bemf(comp, commutation);

    let exit_interval = if bidirectional {
        P::OLD_ROUTINE_EXIT_INTERVAL / 2
    } else {
        P::OLD_ROUTINE_EXIT_INTERVAL
    };
    if !shared.old_routine() {
        // Interrupt mode.
        let to_old = crate::platform::e_com_us_to_avg_ticks::<P>(e_com) as u32
            > exit_interval + P::OLD_ROUTINE_REENTRY_MARGIN;
        if to_old {
            writer.set_old_routine(shared, true);
        }
        let new_ci = bemf.update_timing_from_timer(ci0);
        shared.set_commutation_interval(new_ci);
        if !to_old {
            comp.enable_irq();
        }
        bemf.reset_after_commutation();
        writer.increment_zero_crosses(shared);
    } else {
        // Polling mode (old_routine): comparator interrupts stay off until changeover.
        bemf.reset_after_commutation();
        writer.increment_zero_crosses(shared);
        let zc = shared.zero_crosses();
        let changeover_met = if strict_changeover {
            zc >= OLD_ROUTINE_EXIT_ZC && ci0 <= exit_interval
        } else {
            ci0 < exit_interval
        };
        if changeover_met {
            if shared.gate_avg() == 0 {
                let avg = crate::platform::e_com_us_to_avg_ticks::<P>(e_com).max(1) as u32;
                shared.set_gate_avg(avg);
            }
            writer.transition(shared, MotorEvent::BemfLocked);
            comp.enable_irq();
        }
    }
}

/// BEMF zero-cross detected (COMP ISR body).
///
/// **IMPORTANT for porting**: the noise-filter loop below early-returns
/// BEFORE `comp.mask_irq()` (which is the only path that clears the EXTI
/// pending bit if called directly). Platform ISR wrappers must route through
/// `on_bemf_irq` (`isr_handlers::handle_comp_irq`), which owns the EXTI
/// pending bit and runs AM32's half-average-interval gate before calling this
/// function.
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
    let count = interval.count().min(u16::MAX as u32) as u16;
    interval.set_count(0);
    bemf.record_zc_timing(count);
    com_timer.set_and_enable(bemf.com_timer_delay());
    true
}
