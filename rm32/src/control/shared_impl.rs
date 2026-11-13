//! SharedComm test implementation using Cell for interior mutability.

use crate::motor_mode::{MotorEvent, MotorMode};
use crate::platform::{Platform, TestPlatform};
use crate::shared_comm::{
    Commands, Drive, IsrAction, MotorStatus, MotorStatusExt, Protections, Telemetry, Throttle,
};
use crate::units::{CentiAmps, CentiVolts};
use core::cell::Cell;

/// Test-friendly SharedComm that uses Cell for interior mutability.
pub struct TestShared {
    pub mode: Cell<MotorMode>,
    pub input_set: Cell<bool>,
    pub dshot_telemetry: Cell<bool>,
    pub is_dshot: Cell<bool>,
    pub newinput: Cell<u16>,
    pub adjusted_input: Cell<u16>,
    pub duty_cycle_setpoint: Cell<u16>,
    pub duty_cycle: Cell<u16>,
    pub forward: Cell<bool>,
    pub zero_crosses: Cell<u32>,
    pub commutation_interval: Cell<u32>,
    pub e_com_time: Cell<i32>,
    pub gate_avg: Cell<u32>,
    pub signal_timeout: Cell<u32>,
    pub send_telemetry: Cell<bool>,
    // Main→ISR published state
    pub tim1_arr: Cell<u16>,
    pub duty_maximum: Cell<u16>,
    pub filter_level: Cell<u8>,
    pub min_bemf_counts: Cell<u8>,
    pub auto_advance: Cell<u8>,
    pub interval_timer_count: Cell<u32>,
    pub one_khz_counter: Cell<u8>,
    pub control_ticks: Cell<u32>,
    pub telem_counter: Cell<u16>,
    pub prop_brake_active: Cell<bool>,
    pub tone_active: Cell<bool>,
    pub isr_action: Cell<IsrAction>,
    pub duty_kick_down: Cell<bool>,
    pub duty_kick_half: Cell<bool>,
    pub changeover_step: Cell<u8>,
    pub desync_check_pending: Cell<bool>,
    pub stall_protection_adjust: Cell<u16>,
    pub current_limit_adjust: Cell<u16>,
    pub battery_voltage: Cell<CentiVolts>,
    pub actual_current: Cell<CentiAmps>,
    pub degrees_celsius: Cell<i16>,
    pub save_settings_flag: Cell<bool>,
    pub send_esc_info_flag: Cell<bool>,
}

impl Default for TestShared {
    fn default() -> Self {
        Self::new()
    }
}

impl TestShared {
    pub fn new() -> Self {
        Self {
            mode: Cell::new(MotorMode::Disarmed),
            input_set: Cell::new(false),
            dshot_telemetry: Cell::new(false),
            is_dshot: Cell::new(false),
            newinput: Cell::new(0),
            adjusted_input: Cell::new(0),
            duty_cycle_setpoint: Cell::new(0),
            duty_cycle: Cell::new(0),
            forward: Cell::new(true),
            zero_crosses: Cell::new(0),
            commutation_interval: Cell::new(12500),
            e_com_time: Cell::new(0),
            gate_avg: Cell::new(0),
            signal_timeout: Cell::new(0),
            send_telemetry: Cell::new(false),
            tim1_arr: Cell::new(TestPlatform::PWM_BASE_PERIOD),
            duty_maximum: Cell::new(2000),
            filter_level: Cell::new(5),
            min_bemf_counts: Cell::new(2),
            auto_advance: Cell::new(0),
            interval_timer_count: Cell::new(0),
            one_khz_counter: Cell::new(0),
            control_ticks: Cell::new(0),
            telem_counter: Cell::new(0),
            prop_brake_active: Cell::new(false),
            tone_active: Cell::new(false),
            isr_action: Cell::new(IsrAction::None),
            duty_kick_down: Cell::new(false),
            duty_kick_half: Cell::new(false),
            changeover_step: Cell::new(0),
            desync_check_pending: Cell::new(false),
            stall_protection_adjust: Cell::new(0),
            current_limit_adjust: Cell::new(2000),
            battery_voltage: Cell::new(CentiVolts::ZERO),
            actual_current: Cell::new(CentiAmps::ZERO),
            degrees_celsius: Cell::new(0),
            save_settings_flag: Cell::new(false),
            send_esc_info_flag: Cell::new(false),
        }
    }
}

impl MotorStatus for TestShared {
    fn motor_mode(&self) -> MotorMode {
        self.mode.get()
    }
    fn set_motor_mode(&self, mode: MotorMode) {
        self.mode.set(mode);
    }
    fn transition(&self, event: MotorEvent) {
        let new = self.mode.get().transition(event);
        self.mode.set(new);
    }
    fn set_armed(&self, v: bool) {
        if v && !self.armed() {
            self.mode.set(MotorMode::Armed);
        } else if !v {
            self.mode.set(MotorMode::Disarmed);
        }
    }
    fn set_running(&self, v: bool) {
        if v && !self.running() {
            self.mode.set(MotorMode::OldRoutine);
        } else if !v && self.running() {
            self.mode.set(MotorMode::Armed);
        }
    }
    fn set_old_routine(&self, v: bool) {
        if v && self.running() {
            self.mode.set(MotorMode::OldRoutine);
        } else if !v && self.old_routine() {
            self.mode.set(MotorMode::Running);
        }
    }
    fn set_stepper_sine(&self, v: bool) {
        if v {
            self.mode.set(MotorMode::StepperSine);
        } else if self.stepper_sine() {
            self.mode.set(MotorMode::Armed);
        }
    }
    /// Single-threaded `_isr0` transition delegate.
    unsafe fn transition_isr0(&self, event: MotorEvent) {
        self.transition(event);
    }
    /// Single-threaded `_isr0` `old_routine` delegate.
    unsafe fn set_old_routine_isr0(&self, v: bool) {
        self.set_old_routine(v);
    }
    fn zero_crosses(&self) -> u32 {
        self.zero_crosses.get()
    }
    fn set_zero_crosses(&self, v: u32) {
        self.zero_crosses.set(v);
    }
    fn increment_zero_crosses(&self) {
        let v = self.zero_crosses.get();
        if v < 10000 {
            self.zero_crosses.set(v + 1);
        }
    }
    /// Single-threaded `_isr0` `zero_crosses` increment delegate.
    unsafe fn increment_zero_crosses_isr0(&self) {
        self.increment_zero_crosses();
    }
    fn commutation_interval(&self) -> u32 {
        self.commutation_interval.get()
    }
    fn set_commutation_interval(&self, v: u32) {
        self.commutation_interval.set(v);
    }
    fn e_com_time(&self) -> i32 {
        self.e_com_time.get()
    }
    fn set_e_com_time(&self, v: i32) {
        self.e_com_time.set(v);
    }
    fn gate_avg(&self) -> u32 {
        self.gate_avg.get()
    }
    fn set_gate_avg(&self, v: u32) {
        self.gate_avg.set(v);
    }
    fn interval_timer_count(&self) -> u32 {
        self.interval_timer_count.get()
    }
    fn set_interval_timer_count(&self, v: u32) {
        self.interval_timer_count.set(v);
    }
    fn duty_cycle(&self) -> u16 {
        self.duty_cycle.get()
    }
    fn set_duty_cycle(&self, v: u16) {
        self.duty_cycle.set(v);
    }
    fn forward(&self) -> bool {
        self.forward.get()
    }
    fn set_forward(&self, v: bool) {
        self.forward.set(v);
    }
    fn desync_check_pending(&self) -> bool {
        self.desync_check_pending.get()
    }
    fn set_desync_check_pending(&self, v: bool) {
        self.desync_check_pending.set(v);
    }
    fn changeover_step(&self) -> u8 {
        self.changeover_step.get()
    }
    fn set_changeover_step(&self, step: u8) {
        self.changeover_step.set(step);
    }
}

impl Throttle for TestShared {
    fn input_set(&self) -> bool {
        self.input_set.get()
    }
    fn set_input_set(&self, v: bool) {
        self.input_set.set(v);
    }
    fn dshot_telemetry(&self) -> bool {
        self.dshot_telemetry.get()
    }
    fn is_dshot(&self) -> bool {
        self.is_dshot.get()
    }
    fn set_is_dshot(&self, v: bool) {
        self.is_dshot.set(v);
    }
    fn newinput(&self) -> u16 {
        self.newinput.get()
    }
    fn set_newinput(&self, v: u16) {
        self.newinput.set(v);
    }
    fn adjusted_input(&self) -> u16 {
        self.adjusted_input.get()
    }
    fn set_adjusted_input(&self, v: u16) {
        self.adjusted_input.set(v);
    }
    fn duty_cycle_setpoint(&self) -> u16 {
        self.duty_cycle_setpoint.get()
    }
    fn set_duty_cycle_setpoint(&self, v: u16) {
        self.duty_cycle_setpoint.set(v);
    }
    fn signal_timeout(&self) -> u32 {
        self.signal_timeout.get()
    }
    fn increment_signal_timeout(&self) {
        let v = self.signal_timeout.get();
        if v < u32::MAX {
            self.signal_timeout.set(v + 1);
        }
    }
    fn prop_brake_active(&self) -> bool {
        self.prop_brake_active.get()
    }
    fn set_prop_brake_active(&self, v: bool) {
        self.prop_brake_active.set(v);
    }
}

impl Drive for TestShared {
    fn tim1_arr(&self) -> u16 {
        self.tim1_arr.get()
    }
    fn set_tim1_arr(&self, v: u16) {
        self.tim1_arr.set(v);
    }
    fn filter_level(&self) -> u8 {
        self.filter_level.get()
    }
    fn set_filter_level(&self, v: u8) {
        self.filter_level.set(v);
    }
    fn min_bemf_counts(&self) -> u8 {
        self.min_bemf_counts.get()
    }
    fn set_min_bemf_counts(&self, v: u8) {
        self.min_bemf_counts.set(v);
    }
    fn auto_advance(&self) -> u8 {
        self.auto_advance.get()
    }
    fn set_auto_advance(&self, v: u8) {
        self.auto_advance.set(v);
    }
}

impl Protections for TestShared {
    fn duty_maximum(&self) -> u16 {
        self.duty_maximum.get()
    }
    fn set_duty_maximum(&self, v: u16) {
        self.duty_maximum.set(v);
    }
    fn stall_protection_adjust(&self) -> u16 {
        self.stall_protection_adjust.get()
    }
    fn set_stall_protection_adjust(&self, v: u16) {
        self.stall_protection_adjust.set(v);
    }
    fn current_limit_adjust(&self) -> u16 {
        self.current_limit_adjust.get()
    }
    fn set_current_limit_adjust(&self, v: u16) {
        self.current_limit_adjust.set(v);
    }
}

impl Telemetry for TestShared {
    fn send_telemetry(&self) -> bool {
        self.send_telemetry.get()
    }
    fn set_send_telemetry(&self, v: bool) {
        self.send_telemetry.set(v);
    }
    fn send_esc_info_flag(&self) -> bool {
        self.send_esc_info_flag.get()
    }
    fn set_send_esc_info_flag(&self, v: bool) {
        self.send_esc_info_flag.set(v);
    }
    fn telem_counter_check_and_inc(&self, limit: u16) -> bool {
        let next = self.telem_counter.get().saturating_add(1);
        if next >= limit {
            self.telem_counter.set(0);
            true
        } else {
            self.telem_counter.set(next);
            false
        }
    }
    fn battery_voltage(&self) -> CentiVolts {
        self.battery_voltage.get()
    }
    fn set_battery_voltage(&self, v: CentiVolts) {
        self.battery_voltage.set(v);
    }
    fn set_actual_current(&self, v: CentiAmps) {
        self.actual_current.set(v);
    }
    fn set_degrees_celsius(&self, v: i16) {
        self.degrees_celsius.set(v);
    }
}

impl Commands for TestShared {
    fn isr_action(&self) -> IsrAction {
        self.isr_action.get()
    }
    fn request_isr_action(&self, action: IsrAction) {
        if action as u8 > self.isr_action.get() as u8 {
            self.isr_action.set(action);
        }
    }
    fn clear_isr_action(&self, action: IsrAction) {
        if self.isr_action.get() == action {
            self.isr_action.set(IsrAction::None);
        }
    }
    fn request_duty_kick_down(&self) {
        self.duty_kick_down.set(true);
    }
    fn take_duty_kick_down(&self) -> bool {
        self.duty_kick_down.replace(false)
    }
    fn request_duty_kick_half(&self) {
        self.duty_kick_half.set(true);
    }
    fn take_duty_kick_half(&self) -> bool {
        self.duty_kick_half.replace(false)
    }
    fn save_settings_flag(&self) -> bool {
        self.save_settings_flag.get()
    }
    fn set_save_settings_flag(&self, v: bool) {
        self.save_settings_flag.set(v);
    }
    fn control_ticks(&self) -> u32 {
        self.control_ticks.get()
    }
    fn increment_control_ticks(&self) {
        self.control_ticks
            .set(self.control_ticks.get().wrapping_add(1));
    }
    fn one_khz_counter_inc(&self) {
        self.one_khz_counter
            .set(self.one_khz_counter.get().saturating_add(1));
    }
    fn one_khz_counter_check_and_reset(&self, divider: u8) -> bool {
        if self.one_khz_counter.get() >= divider {
            self.one_khz_counter.set(0);
            true
        } else {
            false
        }
    }
    fn tone_active(&self) -> bool {
        self.tone_active.get()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changeover_step_handoff_roundtrips() {
        let shared = TestShared::new();

        assert_eq!(shared.changeover_step(), 0);
        shared.set_changeover_step(5);
        assert_eq!(shared.changeover_step(), 5);
    }

    #[test]
    fn desync_check_pending_roundtrips() {
        let shared = TestShared::new();

        assert!(!shared.desync_check_pending());
        shared.set_desync_check_pending(true);
        assert!(shared.desync_check_pending());
        shared.set_desync_check_pending(false);
        assert!(!shared.desync_check_pending());
    }

    #[test]
    fn domain_traits_roundtrip_without_dummy_defaults() {
        let shared = TestShared::default();

        // MotorStatus & MotorStatusExt
        assert_eq!(shared.motor_mode(), MotorMode::Disarmed);
        shared.set_motor_mode(MotorMode::Armed);
        assert!(shared.armed());
        shared.set_armed(false);
        assert!(!shared.armed());
        shared.set_armed(true);
        assert!(shared.armed());
        shared.set_running(true);
        assert!(shared.running());
        assert!(shared.old_routine());
        shared.set_old_routine(false);
        assert!(shared.running());
        assert!(!shared.old_routine());
        shared.set_old_routine(true);
        assert!(shared.old_routine());
        shared.set_running(false);
        assert!(!shared.running());
        shared.set_stepper_sine(true);
        assert!(shared.stepper_sine());
        shared.set_stepper_sine(false);
        assert!(!shared.stepper_sine());
        shared.transition(MotorEvent::StartMotor);
        assert!(shared.running());
        shared.set_zero_crosses(10);
        assert_eq!(shared.zero_crosses(), 10);
        shared.increment_zero_crosses();
        assert_eq!(shared.zero_crosses(), 11);
        shared.set_commutation_interval(900);
        assert_eq!(shared.commutation_interval(), 900);
        shared.set_e_com_time(4321);
        assert_eq!(shared.e_com_time(), 4321);
        shared.set_gate_avg(321);
        assert_eq!(shared.gate_avg(), 321);
        shared.set_interval_timer_count(999);
        assert_eq!(shared.interval_timer_count(), 999);
        shared.set_duty_cycle(777);
        assert_eq!(shared.duty_cycle(), 777);
        shared.set_forward(false);
        assert!(!shared.forward());

        // Throttle
        assert!(!shared.dshot_telemetry());
        shared.set_input_set(true);
        assert!(shared.input_set());
        shared.set_is_dshot(true);
        assert!(shared.is_dshot());
        shared.set_newinput(1100);
        assert_eq!(shared.newinput(), 1100);
        shared.set_adjusted_input(900);
        assert_eq!(shared.adjusted_input(), 900);
        shared.set_duty_cycle_setpoint(800);
        assert_eq!(shared.duty_cycle_setpoint(), 800);
        shared.increment_signal_timeout();
        assert_eq!(shared.signal_timeout(), 1);
        shared.set_prop_brake_active(true);
        assert!(shared.prop_brake_active());

        // Drive
        shared.set_tim1_arr(1500);
        assert_eq!(shared.tim1_arr(), 1500);
        shared.set_filter_level(8);
        assert_eq!(shared.filter_level(), 8);
        shared.set_min_bemf_counts(4);
        assert_eq!(shared.min_bemf_counts(), 4);
        shared.set_auto_advance(3);
        assert_eq!(shared.auto_advance(), 3);

        // Protections
        shared.set_duty_maximum(1600);
        assert_eq!(shared.duty_maximum(), 1600);
        shared.set_stall_protection_adjust(42);
        assert_eq!(shared.stall_protection_adjust(), 42);
        shared.set_current_limit_adjust(1400);
        assert_eq!(shared.current_limit_adjust(), 1400);

        // Telemetry
        shared.set_send_telemetry(true);
        assert!(shared.send_telemetry());
        shared.set_send_esc_info_flag(true);
        assert!(shared.send_esc_info_flag());
        shared.set_battery_voltage(CentiVolts(1680));
        assert_eq!(shared.battery_voltage(), CentiVolts(1680));
        shared.set_actual_current(CentiAmps(1250));
        assert_eq!(shared.actual_current.get(), CentiAmps(1250));
        shared.set_degrees_celsius(55);
        assert_eq!(shared.degrees_celsius.get(), 55);
        assert!(!shared.telem_counter_check_and_inc(2));
        assert!(shared.telem_counter_check_and_inc(2));

        // Commands
        shared.request_isr_action(IsrAction::CommutateKick);
        assert_eq!(shared.isr_action(), IsrAction::CommutateKick);
        shared.clear_isr_action(IsrAction::CommutateKick);
        assert_eq!(shared.isr_action(), IsrAction::None);
        shared.request_duty_kick_down();
        assert!(shared.take_duty_kick_down());
        assert!(!shared.take_duty_kick_down());
        shared.set_save_settings_flag(true);
        assert!(shared.save_settings_flag());
        shared.increment_control_ticks();
        assert_eq!(shared.control_ticks(), 1);
        shared.one_khz_counter_inc();
        assert!(!shared.one_khz_counter_check_and_reset(2));
        shared.one_khz_counter_inc();
        assert!(shared.one_khz_counter_check_and_reset(2));
        shared.tone_active.set(true);
        assert!(shared.tone_active());
    }
}
