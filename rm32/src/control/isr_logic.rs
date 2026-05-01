//! ISR-level control logic — platform-independent, fully testable.
//!
//! All functions take `MotorContext<S, H>` for static dispatch.
//! No `&dyn` trait objects — the compiler monomorphizes to concrete MCU types,
//! eliminating vtable overhead in the 20kHz ISR.

use crate::commutation::Commutation;
use crate::constants::*;
use crate::control::context::MotorContext;
use crate::control::state::{BemfState, DutyState};
use crate::hal::{self, ComTimer, Comparator, IntervalTimer, MotorHal, PhaseOutput, PwmOutput};
use crate::motor_mode::MotorEvent;
use crate::shared_comm::SharedComm;

/// Counters and config owned exclusively by the ISR tick.
pub struct TickCounters {
    pub(crate) ten_khz_counter: u32,
    pub(crate) one_khz_loop_counter: u16,
    pub(crate) armed_timeout_count: u32,
    pub(crate) tim1_arr: u16,
}

impl TickCounters {
    /// Set TIM1 auto-reload (from EEPROM-derived motor config).
    pub fn set_tim1_arr(&mut self, v: u16) {
        self.tim1_arr = v;
    }

    /// Read armed timeout count (for harness reporting).
    pub fn armed_timeout_count(&self) -> u32 {
        self.armed_timeout_count
    }

    /// Create with the given TIM1 auto-reload value. All counters start at zero.
    pub fn new(tim1_arr: u16) -> Self {
        Self {
            ten_khz_counter: 0,
            one_khz_loop_counter: 0,
            armed_timeout_count: 0,
            tim1_arr,
        }
    }
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
            let setpoint = ctx.duty.compute_setpoint(
                input,
                ctx.shared.zero_crosses(),
                ctx.config.stall_protection,
            );
            ctx.shared.set_duty_cycle_setpoint(setpoint);
            if !ctx.shared.running() {
                ctx.shared.transition(MotorEvent::StartMotor);
                ctx.duty.start_motor();
                let step = ctx.commutation.advance();
                let e_com = ctx
                    .commutation
                    .record_interval(ctx.shared.commutation_interval() as u16);
                ctx.shared.set_e_com_time(e_com);
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
    ctx.duty.set_cycle(setpoint);
    ctx.counters.ten_khz_counter += 1;
    ctx.shared.increment_signal_timeout();
    ctx.duty.increment_ramp_count();
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
    let average_interval = (ctx.shared.e_com_time() / 3) as u32;
    ctx.duty.ramp_limit(
        ctx.shared.battery_voltage(),
        ctx.shared.commutation_interval(),
        ctx.shared.zero_crosses(),
        average_interval,
        ctx.voltage_based_ramp,
    );

    // Sync main→ISR published state (main computes, ISR applies)
    ctx.counters.tim1_arr = ctx.shared.tim1_arr();
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
    let tim1_arr = ctx.counters.tim1_arr;
    if ctx.shared.armed() && ctx.shared.running() {
        ctx.hal.pwm().set_duty_all(ctx.duty.pwm_compare(tim1_arr));
    } else if ctx.shared.prop_brake_active() {
        ctx.hal.pwm().set_duty_all(DutyState::brake_compare(
            ctx.config.drag_brake_strength,
            tim1_arr,
        ));
    } else {
        ctx.hal.pwm().set_duty_all(0);
    }
    let final_duty = ctx.duty.finalize();
    ctx.shared.set_duty_cycle(final_duty);
    ctx.hal.pwm().set_auto_reload(tim1_arr);

    // Sync ISR→shared (Commutation owns truth, shared publishes for main loop)
    ctx.shared.set_forward(ctx.commutation.forward);
    ctx.shared
        .set_interval_timer_count(ctx.hal.interval().count());
}

/// BEMF polling (old_routine path).
fn bemf_polling<S: SharedComm, H: MotorHal>(ctx: &mut MotorContext<S, H>) {
    ctx.hal.comp().mask_interrupts();
    let comp_level = ctx.hal.comp().output_level();
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
                .record_interval(ctx.shared.commutation_interval() as u16);
            ctx.shared.set_e_com_time(e_com);
            ctx.hal.phase().com_step(step);
            ctx.hal.phase().pulse_toggle(step);
            ctx.hal.comp().set_step(step, ctx.commutation.rising);
            ctx.hal.comp().change_input();
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
    let e_com = commutation.record_interval(shared.commutation_interval() as u16);
    shared.set_e_com_time(e_com);
    phase.com_step(step);
    phase.pulse_toggle(step);
    comp.set_step(step, commutation.rising);
    comp.change_input();

    if !shared.old_routine() {
        let new_ci = bemf.update_timing_from_timer(shared.commutation_interval());
        shared.set_commutation_interval(new_ci);
    }

    comp.enable_interrupts();
    bemf.reset_after_commutation();
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
    for _ in 0..bemf.filter_level() {
        if comp.output_level() == commutation.rising() {
            return;
        }
    }
    comp.mask_interrupts();
    let count = interval.count() as u16;
    interval.set_count(0);
    bemf.record_zc_timing(count);
    com_timer.set_and_enable(bemf.com_timer_delay());
}
