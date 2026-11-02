//! SharedComm — the state the ISRs and the main loop share.
//!
//! On real hardware, this is implemented via atomics (SharedState).
//! For testing, TestShared implements it with Cell fields.
//!
//! Split into groups by what the state is about. No method has a default
//! body: every implementation states every value, so a missed one is a
//! compile error instead of a plausible constant (`tim1_arr = 1999`,
//! `filter_level = 5`). The only provided methods are pure derivations,
//! in the blanket-implemented [`MotorStatusExt`]:
//! - [`MotorStatus`]: motor mode, commutation timing, direction
//! - [`Throttle`]: input detection, throttle and duty setpoints
//! - [`Drive`]: main-loop-computed drive parameters the ISRs apply
//! - [`Protections`]: duty ceilings from the protection loops
//! - [`Telemetry`]: telemetry requests, counters and measurements
//! - [`Commands`]: ISR actions, settings requests, control-tick handoffs

use crate::motor_mode::{MotorEvent, MotorMode};

/// Action requested from main loop to ISR context.
///
/// Priority-ordered: AllOff supersedes ResetIntervalTimer (if both are
/// needed, the motor is being killed so the timer reset is moot).
/// Stored as AtomicU8 in SharedState. Main writes via `request_isr_action`;
/// ISR reads via `isr_action`, executes, and clears to None.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum IsrAction {
    /// No pending action.
    None = 0,
    /// Reset interval timer to 0 (stall handler, matches C's zcfoundroutine).
    ResetIntervalTimer = 1,
    /// BEMF-timeout recovery: re-arm the commutation chain now — the COM
    /// timer may be dead after a timeout; AM32's zcfoundroutine actively
    /// re-commutates.
    CommutateKick = 2,
    /// Kill all FETs + mask comparator interrupts (LVC, stuck rotor).
    /// MUST stay the highest value — `request_isr_action` uses fetch_max
    /// for priority, and a kill outranks every recovery action.
    AllOff = 3,
}

impl IsrAction {
    pub const fn from_u8(value: u8) -> Self {
        match value {
            x if x == Self::ResetIntervalTimer as u8 => Self::ResetIntervalTimer,
            x if x == Self::CommutateKick as u8 => Self::CommutateKick,
            x if x == Self::AllOff as u8 => Self::AllOff,
            _ => Self::None,
        }
    }
}

/// Motor mode state machine and commutation state (ISR <-> main).
pub trait MotorStatus {
    fn motor_mode(&self) -> MotorMode;
    fn set_motor_mode(&self, mode: MotorMode);

    /// Apply a state transition event atomically.
    fn transition(&self, event: MotorEvent);
    fn set_armed(&self, v: bool);
    fn set_running(&self, v: bool);
    fn set_old_routine(&self, v: bool);
    fn set_stepper_sine(&self, v: bool);

    fn zero_crosses(&self) -> u32;
    fn set_zero_crosses(&self, v: u32);
    fn increment_zero_crosses(&self);
    fn commutation_interval(&self) -> u32;
    fn set_commutation_interval(&self, v: u32);
    fn e_com_time(&self) -> i32;
    fn set_e_com_time(&self, v: i32);
    /// COMP acceptance-gate average (interval-timer ticks), latched by the
    /// control tick at 20 kHz and read by the COMP ISR (matching AM32's
    /// slow-loop `average_interval` read in `COMP_IRQHandler`).
    /// 0 = not latched yet (gate remains closed until a positive average is latched).
    fn gate_avg(&self) -> u32;
    fn set_gate_avg(&self, v: u32);
    fn interval_timer_count(&self) -> u32;
    fn set_interval_timer_count(&self, v: u32);

    /// Current duty cycle (ISR writes each tick, main reads for bidir speed gate).
    fn duty_cycle(&self) -> u16;
    fn set_duty_cycle(&self, v: u16);

    /// Motor direction (ISR syncs from commutation, input flips on bidir).
    fn forward(&self) -> bool;
    fn set_forward(&self, v: bool);

    /// ISR-published request for main-loop desync checks.
    fn desync_check_pending(&self) -> bool;
    fn set_desync_check_pending(&self, v: bool);

    /// Sine changeover step request (0 = none, 1-6 = execute changeover with step).
    /// Main sets during sine changeover; ISR applies com_step + enables interrupts.
    fn changeover_step(&self) -> u8;
    fn set_changeover_step(&self, step: u8);
}

/// Mode predicates derived from [`MotorStatus::motor_mode`]. Blanket
/// implemented, so no implementation can override or forget them.
pub trait MotorStatusExt: MotorStatus {
    fn armed(&self) -> bool {
        self.motor_mode().is_armed()
    }
    fn running(&self) -> bool {
        self.motor_mode().is_running()
    }
    fn old_routine(&self) -> bool {
        self.motor_mode().is_old_routine()
    }
    fn stepper_sine(&self) -> bool {
        self.motor_mode().is_stepper_sine()
    }
}

impl<T: MotorStatus + ?Sized> MotorStatusExt for T {}

/// Input detection, throttle and the duty setpoint.
pub trait Throttle {
    fn input_set(&self) -> bool;
    fn set_input_set(&self, v: bool);
    fn dshot_telemetry(&self) -> bool;

    /// Whether detected input is DShot (vs servo). ISR transfer handler sets this.
    fn is_dshot(&self) -> bool;
    fn set_is_dshot(&self, v: bool);

    fn newinput(&self) -> u16;
    fn set_newinput(&self, v: u16);
    fn adjusted_input(&self) -> u16;
    fn set_adjusted_input(&self, v: u16);
    fn duty_cycle_setpoint(&self) -> u16;
    fn set_duty_cycle_setpoint(&self, v: u16);

    fn signal_timeout(&self) -> u32;
    fn increment_signal_timeout(&self);

    /// Proportional brake active (main sets, ISR reads for brake-on-stop).
    fn prop_brake_active(&self) -> bool;
    fn set_prop_brake_active(&self, v: bool);
}

/// Drive parameters the main loop computes and the ISRs apply.
pub trait Drive {
    /// PWM timer auto-reload value (variable PWM). Main publishes, ISR applies.
    /// Starts at the port's PWM base period (`Platform::PWM_BASE_PERIOD`).
    fn tim1_arr(&self) -> u16;
    fn set_tim1_arr(&self, v: u16);

    /// BEMF filter level. Main computes based on motor speed, ISR uses for ZC detection.
    fn filter_level(&self) -> u8;
    fn set_filter_level(&self, v: u8);

    /// Min BEMF counts for zero-cross acceptance. Main adjusts during startup.
    fn min_bemf_counts(&self) -> u8;
    fn set_min_bemf_counts(&self, v: u8);

    /// Auto advance level. Main computes from duty cycle, ISR uses for timing.
    fn auto_advance(&self) -> u8;
    fn set_auto_advance(&self, v: u8);
}

/// Duty ceilings published by the protection loops.
pub trait Protections {
    /// Max duty cycle (eRPM/temperature limiting). Main publishes, ISR applies.
    fn duty_maximum(&self) -> u16;
    fn set_duty_maximum(&self, v: u16);
    fn stall_protection_adjust(&self) -> u16;
    fn set_stall_protection_adjust(&self, v: u16);
    /// Current limit duty ceiling (main PID publishes, ISR clamps duty).
    fn current_limit_adjust(&self) -> u16;
    fn set_current_limit_adjust(&self, v: u16);
}

/// Telemetry requests, the telemetry interval counter and the
/// measurements EDT reports.
pub trait Telemetry {
    fn send_telemetry(&self) -> bool;
    fn set_send_telemetry(&self, v: bool);
    fn send_esc_info_flag(&self) -> bool;
    fn set_send_esc_info_flag(&self, v: bool);

    /// ISR-side (20 kHz): increment the interval-telemetry counter; if it
    /// has exceeded `limit`, reset it and return true (fire telemetry).
    /// Matches AM32's `telem_ms_count` block.
    fn telem_counter_check_and_inc(&self, limit: u16) -> bool;

    /// Measurement publish for EDT telemetry (main writes, ISR reads).
    fn battery_voltage(&self) -> crate::units::CentiVolts;
    fn set_battery_voltage(&self, v: crate::units::CentiVolts);
    fn set_actual_current(&self, v: crate::units::CentiAmps);
    fn set_degrees_celsius(&self, v: i16);
}

/// ISR action requests, settings requests and control-tick handoffs.
pub trait Commands {
    /// ISR action request from main loop.
    ///
    /// Variant values are ordered by priority; implementations keep the
    /// highest pending action and clear only the action the ISR handled.
    fn isr_action(&self) -> IsrAction;
    fn request_isr_action(&self, action: IsrAction);
    fn clear_isr_action(&self, action: IsrAction);

    /// Request that the next control tick reduce duty after a detected desync.
    fn request_duty_kick_down(&self);
    /// Consume a pending desync duty reduction, independently of timer actions.
    fn take_duty_kick_down(&self) -> bool;

    fn save_settings_flag(&self) -> bool;
    fn set_save_settings_flag(&self, v: bool);

    /// Free-running control ISR count, wrapping at u32::MAX. Never reset by main.
    fn control_ticks(&self) -> u32;
    /// Advance the free-running control-tick count (TIM6 ISR side, once per
    /// tick). The main loop measures its 1 s and LED periods against it, as
    /// AM32's main loop polls `tenkhzcounter` / `ledcounter`, which
    /// `tenKhzRoutine` increments.
    fn increment_control_ticks(&self);
    /// Increment 1 kHz dispatch counter (TIM6 ISR side, 20 kHz). Matches
    /// AM32's `one_khz_loop_counter++`.
    fn one_khz_counter_inc(&self);
    /// Main-side: returns true and resets counter if it has exceeded
    /// `divider` (typically PID_LOOP_DIVIDER = 20).
    fn one_khz_counter_check_and_reset(&self, divider: u8) -> bool;

    /// Whether the ISR tone scheduler owns the bridge PWM until its next
    /// sequence transition. Motor output and braking take priority.
    fn tone_active(&self) -> bool;
}

/// All shared state. Blanket implemented for any type with every group.
pub trait SharedComm:
    MotorStatus + MotorStatusExt + Throttle + Drive + Protections + Telemetry + Commands
{
}

impl<T> SharedComm for T where
    T: MotorStatus + Throttle + Drive + Protections + Telemetry + Commands + ?Sized
{
}
