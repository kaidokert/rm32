//! ISR logic functions — shared between MCU targets.
//!
//! Each function contains the actual ISR body. The MCU-specific
//! `#[interrupt]` wrappers in `mcu_*/interrupts.rs` clear peripheral flags and
//! delegate here.
//!
//! # Priority-Partitioned ISR State
//!
//! To allow priority-0 comparator (`COMP`) and commutation-timer (`TIM14`/`TIM16`)
//! interrupts to preempt the 20 kHz control loop (`TIM6`, priority 2/3) without
//! either `PRIMASK` latency jitter on priority 0 or overlapping `&mut` borrows,
//! the live state is split on first entry into three disjoint static cells:
//!
//! 1. [`COM_LOCAL`] (`ComIsrState`): `commutation`, `bemf`, `comp`, `interval`,
//!    `com_timer`, `phase`, and `writer`.
//!    - Priority-0 handlers (`handle_comp_irq`, `handle_comp`, `handle_tim14`)
//!      share the same NVIC preemption priority on every MCU and never preempt
//!      one another, so they borrow `COM_LOCAL` without `cortex_m::interrupt::free`.
//!    - Lower-priority or thread-mode callers (`handle_tim6`, `service_sine`)
//!      only borrow `COM_LOCAL` inside `COM_LOCAL.with_masked` (`interrupt::free`),
//!      which masks priority-0 interrupts for the duration of that closure.
//!
//! 2. [`TICK_LOCAL`] (`TickIsrState`): `duty`, `tone`, `pwm`, `config` snapshot,
//!    `armed_timeout_count`, and `voltage_based_ramp`.
//!    - Accessed only by `handle_tim6` (which cannot preempt itself) and by
//!      `service_sine` in thread mode (`main`) with interrupts masked.
//!    - On steady-state `Running` interrupt-mode ticks, `handle_tim6` runs the
//!      setpoint, division, ramp-limit, duty-clamp, and `TIM1` compare/period
//!      writes on `TICK_LOCAL` with interrupts unmasked (`PRIMASK = 0`),
//!      confining `interrupt::free` to the brief 4-line `COM_LOCAL` sync.
//!
//! 3. [`INPUT_LOCAL`] (`InputIsrState`): `input`, `cmd`, `edt`,
//!    `transfer`, authoritative `config`, and protocol state.
//!    - Interrupt handlers (`handle_dma_tc`, `handle_exti_frame`, the tick's
//!      config copy) use `INPUT_LOCAL.with_input_irqs_masked`, which masks
//!      only `mcu::INPUT_IRQS` in the NVIC so priority-0 comparator and
//!      commutation interrupts are not delayed by DShot frame decoding.
//!      Thread-mode callers use `INPUT_LOCAL.with_masked` (`interrupt::free`). When the
//!      input peripheral first moves out of the boot staging cell into
//!      `INPUT_LOCAL`, `start_capture()` is re-armed so DMA `CMAR` points
//!      at the permanent `INPUT_LOCAL` buffer address.
//!
//! 4. [`CRSF_LOCAL`] (`CrsfParser`): UART CRSF state machine, accessed
//!    exclusively by `handle_crsf_byte` under `CRSF_LOCAL.with_masked` so a
//!    UART RX interrupt cannot alias `INPUT_LOCAL` if it preempts a DShot ISR.

use crate::bridge::TimerPwm as _;
use crate::isr::{self, IsrHalParts, SplitMotorHal, TargetIsrHal};
use core::cell::UnsafeCell;
use portable_atomic::{AtomicBool, AtomicU8, Ordering};
use rm32::commutation::Commutation;
use rm32::config::EepromConfig;
use rm32::control::state::{BemfState, DutyState};
use rm32::crsf::CrsfParser;
use rm32::dshot_commands::CommandProcessor;
use rm32::edt::EdtScheduler;
use rm32::hal::InputCapture;
use rm32::transfer::{CaptureConfig, TransferState};

struct ComIsrState<C, IT, CT, PH> {
    commutation: Commutation,
    bemf: BemfState,
    comp: C,
    interval: IT,
    com_timer: CT,
    phase: PH,
    writer: rm32::shared_comm::SingleWriter,
}

struct TickIsrState<P> {
    duty: DutyState,
    tone: rm32::tone::ToneScheduler,
    pwm: P,
    config: EepromConfig,
    armed_timeout_count: u32,
    voltage_based_ramp: bool,
}

struct InputIsrState<I> {
    input: I,
    cmd: CommandProcessor,
    edt: EdtScheduler,
    transfer: TransferState,
    config: EepromConfig,
    forward: bool,
    edt_armed: bool,
    edt_arm_enable: bool,
    frametime_low: u16,
    frametime_high: u16,
}

type TargetComIsrState = ComIsrState<
    <TargetIsrHal as IsrHalParts>::Comp,
    <TargetIsrHal as IsrHalParts>::Interval,
    <TargetIsrHal as IsrHalParts>::Com,
    <TargetIsrHal as IsrHalParts>::Phase,
>;
type TargetTickIsrState = TickIsrState<<TargetIsrHal as IsrHalParts>::Pwm>;
type TargetInputIsrState = InputIsrState<<TargetIsrHal as IsrHalParts>::Input>;

struct IsrTierCell<T> {
    init: AtomicBool,
    value: UnsafeCell<core::mem::MaybeUninit<T>>,
}

// SAFETY: Single-core MCU; each tier cell is accessed either inside
// `cortex_m::interrupt::free` or by non-preempting handlers of that tier's
// dedicated NVIC priority level (see module-level safety doc).
unsafe impl<T> Sync for IsrTierCell<T> {}

impl<T> IsrTierCell<T> {
    const fn new() -> Self {
        Self {
            init: AtomicBool::new(false),
            value: UnsafeCell::new(core::mem::MaybeUninit::uninit()),
        }
    }

    /// Replace the cell's contents.
    ///
    /// # Safety
    /// Caller must hold `cortex_m::interrupt::free` with no active reference to
    /// this cell on the call stack.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    unsafe fn set(&self, value: T) -> &mut T {
        let slot = unsafe { &mut *self.value.get() };
        if self.init.load(Ordering::Relaxed) {
            // SAFETY: `self.init` is only set after `slot.write(...)`.
            unsafe { slot.assume_init_drop() };
        }
        let r = slot.write(value);
        self.init.store(true, Ordering::Release);
        r
    }

    /// Borrow the cell's contents without changing `PRIMASK`.
    ///
    /// # Safety
    /// Caller must guarantee exclusive access to this tier cell for the
    /// lifetime of the returned reference (either by running at the tier's
    /// non-preemptible NVIC priority with no re-entrant calls, or by holding
    /// `cortex_m::interrupt::free`).
    #[inline(always)]
    #[allow(clippy::mut_from_ref)]
    unsafe fn get_unchecked(&self) -> &mut T {
        assert!(
            self.init.load(Ordering::Relaxed),
            "ISR state not initialized"
        );
        // SAFETY: Verified initialized above; exclusive access guaranteed by caller.
        unsafe { (*self.value.get()).assume_init_mut() }
    }

    /// Borrow the cell's contents with interrupts masked (`PRIMASK = 1`) for
    /// the duration of `f`.
    #[inline]
    fn with_masked<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        cortex_m::interrupt::free(|_| {
            // SAFETY: Interrupts are masked on this single-core MCU for the
            // duration of `f`, and closures do not re-enter `with_masked` on
            // the same cell.
            let state = unsafe { self.get_unchecked() };
            f(state)
        })
    }
}

impl IsrTierCell<TargetInputIsrState> {
    /// Borrow `INPUT_LOCAL` from an interrupt handler with only the input
    /// tier's IRQs (`mcu::INPUT_IRQS`) masked in the NVIC; `PRIMASK` stays 0.
    ///
    /// `with_masked` (`PRIMASK`) here held the priority-0 comparator and
    /// commutation timer off for the whole DShot frame decode (~19 us on
    /// STM32L431), delaying high-speed commutations. AM32's frame handler runs
    /// preemptible under COMP/COM. Thread-mode callers keep `with_masked`:
    /// the tick can preempt `main` and also borrows this cell.
    ///
    /// Must be called from handler mode (an ISR), never from `main`.
    #[inline]
    fn with_input_irqs_masked<R>(&self, f: impl FnOnce(&mut TargetInputIsrState) -> R) -> R {
        use cortex_m::peripheral::NVIC;
        let irqs = crate::mcu::INPUT_IRQS;
        let was = irqs.map(NVIC::is_enabled);
        for irq in irqs {
            NVIC::mask(irq);
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        // SAFETY: Every interrupt that borrows `INPUT_LOCAL` is masked in the
        // NVIC, the tick (the only other ISR user) cannot preempt an input
        // ISR and is itself excluded here when it is the caller, and
        // thread-mode users hold PRIMASK, so no ISR runs this while `main`
        // holds the cell. No closure re-enters this cell.
        let r = f(unsafe { self.get_unchecked() });
        for (irq, on) in irqs.into_iter().zip(was) {
            if on {
                // SAFETY: Restores an IRQ that was enabled on entry.
                unsafe { NVIC::unmask(irq) };
            }
        }
        r
    }
}

static COM_LOCAL: IsrTierCell<TargetComIsrState> = IsrTierCell::new();
static TICK_LOCAL: IsrTierCell<TargetTickIsrState> = IsrTierCell::new();
static INPUT_LOCAL: IsrTierCell<TargetInputIsrState> = IsrTierCell::new();
static CRSF_LOCAL: IsrTierCell<CrsfParser> = IsrTierCell::new();

const CFG_FLAG_COMP_PWM: u8 = 1 << 0;
const CFG_FLAG_BIDIR: u8 = 1 << 1;
const CFG_FLAG_STRICT_CHANGEOVER: u8 = 1 << 2;
static COM_CONFIG_FLAGS: AtomicU8 = AtomicU8::new(0);
static CONFIG_DIRTY: AtomicBool = AtomicBool::new(false);

#[inline]
fn latch_com_config_flags(config: &EepromConfig) {
    let flags = (u8::from(config.use_comp_pwm()) * CFG_FLAG_COMP_PWM)
        | (u8::from(config.is_bidirectional()) * CFG_FLAG_BIDIR)
        | (u8::from(config.use_strict_changeover()) * CFG_FLAG_STRICT_CHANGEOVER);
    COM_CONFIG_FLAGS.store(flags, Ordering::Relaxed);
}

#[inline(always)]
fn ensure_split_init() {
    if isr::is_isr_state_staged() {
        install_staged_isr_state();
    }
}

#[cold]
fn install_staged_isr_state() {
    cortex_m::interrupt::free(|_| {
        if let Some(state) = isr::take_isr_state() {
            let (pwm, input, comp, interval, com_timer, phase) = state.hal.into_parts();
            latch_com_config_flags(&state.config);
            CONFIG_DIRTY.store(false, Ordering::Release);
            // SAFETY: Interrupts are masked and no tier cell is borrowed while
            // staging is consumed.
            unsafe {
                COM_LOCAL.set(ComIsrState {
                    commutation: state.commutation,
                    bemf: state.bemf,
                    comp,
                    interval,
                    com_timer,
                    phase,
                    writer: state.writer,
                });
                TICK_LOCAL.set(TickIsrState {
                    duty: state.duty,
                    tone: state.tone,
                    pwm,
                    config: state.config,
                    armed_timeout_count: state.armed_timeout_count,
                    voltage_based_ramp: state.voltage_based_ramp,
                });
                let inp = INPUT_LOCAL.set(InputIsrState {
                    input,
                    cmd: state.cmd,
                    edt: state.edt,
                    transfer: state.transfer,
                    config: state.config,
                    forward: state.forward,
                    edt_armed: state.edt_armed,
                    edt_arm_enable: state.edt_arm_enable,
                    frametime_low: state.frametime_low,
                    frametime_high: state.frametime_high,
                });
                CRSF_LOCAL.set(state.crsf);
                // Re-arm input capture DMA so the peripheral's memory address
                // register points at `INPUT_LOCAL` rather than the consumed
                // `ISR_STATE` staging cell.
                inp.input.start_capture();
            }
        }
    });
}

/// Run the main-loop sine operation against the live motor state, including
/// after the first ISR has taken it from the initialization cell. Keep the
/// returned delay outside interrupt exclusion.
pub fn service_sine(
    system: &mut rm32::system::SystemTick<crate::platform::StmPlatform>,
    main: &mut crate::platform::MainState,
    dead_time: i16,
) -> Option<u16> {
    ensure_split_init();
    COM_LOCAL.with_masked(|com| {
        // SAFETY: Interrupts are masked (`with_masked`), and `service_sine` is
        // only called from `main` (thread mode), so `handle_tim6` cannot be
        // active on the call stack.
        let tick = unsafe { TICK_LOCAL.get_unchecked() };
        let shared = isr::shared();
        if CONFIG_DIRTY.swap(false, Ordering::AcqRel) || shared.config_write_overflowed() {
            // SAFETY: Interrupts are masked (`with_masked`), so no input ISR
            // or main-loop config accessor can overlap this copy.
            let inp = unsafe { INPUT_LOCAL.get_unchecked() };
            tick.config = inp.config;
            rm32::system::SystemTick::<crate::platform::StmPlatform>::sync_config_snapshot(
                shared,
                main,
                &inp.config,
            );
            latch_com_config_flags(&inp.config);
        } else {
            rm32::system::SystemTick::<crate::platform::StmPlatform>::sync_config_writes(
                shared, main,
            );
            latch_com_config_flags(&main.config);
        }
        let mut hal = SplitMotorHal {
            pwm: &mut tick.pwm,
            comp: &mut com.comp,
            interval: &mut com.interval,
            com_timer: &mut com.com_timer,
            phase: &mut com.phase,
        };
        system.service_sine(
            shared,
            main,
            &mut hal,
            &mut tick.duty,
            com.commutation.step(),
            dead_time,
        )
    })
}

/// Recover input configuration after the bounded publication queue fills.
/// Access the live ISR cell, not the initialization cell consumed by the first IRQ.
pub fn resync_input_config(main: &mut crate::platform::MainState) {
    let shared = isr::shared();
    if shared.config_write_overflowed() {
        ensure_split_init();
        INPUT_LOCAL.with_masked(|inp| {
            rm32::system::SystemTick::<crate::platform::StmPlatform>::sync_config_snapshot(
                shared,
                main,
                &inp.config,
            );
        });
    }
}

/// Claim a save and copy authoritative bytes with the input producer excluded.
/// Flash I/O must happen after this function returns, outside the exclusion.
pub fn prepare_input_save(main: &mut crate::platform::MainState) -> bool {
    if !isr::shared().save_settings_flag() {
        return false;
    }
    ensure_split_init();
    INPUT_LOCAL.with_masked(|inp| {
        rm32::system::SystemTick::<crate::platform::StmPlatform>::prepare_config_save(
            isr::shared(),
            main,
            &inp.config,
        )
    })
}

struct LockedComPhase;

impl rm32::hal::PhaseOutput for LockedComPhase {
    #[inline]
    fn com_step(&mut self, step: u8, mode: rm32::hal::PwmMode) {
        COM_LOCAL.with_masked(|com| com.phase.com_step(step, mode));
    }
    #[inline]
    fn all_off(&mut self) {
        COM_LOCAL.with_masked(|com| com.phase.all_off());
    }
    #[inline]
    fn full_brake(&mut self) {
        COM_LOCAL.with_masked(|com| com.phase.full_brake());
    }
    #[inline]
    fn all_pwm(&mut self, mode: rm32::hal::PwmMode) {
        COM_LOCAL.with_masked(|com| com.phase.all_pwm(mode));
    }
    #[inline]
    fn proportional_brake(&mut self) {
        COM_LOCAL.with_masked(|com| com.phase.proportional_brake());
    }
    #[inline]
    fn pulse_toggle(&mut self, step: u8) {
        COM_LOCAL.with_masked(|com| com.phase.pulse_toggle(step));
    }
}

/// 20kHz control loop tick (TIM6 ISR body).
pub fn handle_tim6() {
    use rm32::hal::{BemfSensor as _, ComTimer as _, PhaseOutput as _};
    ensure_split_init();
    let shared = isr::shared();
    // SAFETY: `TICK_LOCAL` is only accessed by `handle_tim6` (which cannot
    // preempt itself) and `service_sine` (which runs in thread mode with
    // interrupts masked, so `handle_tim6` and `service_sine` never overlap).
    let tick = unsafe { TICK_LOCAL.get_unchecked() };

    if shared.running() && CONFIG_DIRTY.load(Ordering::Acquire) {
        INPUT_LOCAL.with_input_irqs_masked(|inp| {
            if CONFIG_DIRTY.swap(false, Ordering::AcqRel) {
                tick.config = inp.config;
                latch_com_config_flags(&tick.config);
            }
        });
    }

    if rm32::control::isr_logic::is_steady_running_tick(shared, &tick.config) {
        let pwm = &mut tick.pwm;
        rm32::control::isr_logic::steady_running_tick_unmasked::<crate::platform::StmPlatform, _, _>(
            shared,
            &mut tick.duty,
            &tick.config,
            tick.voltage_based_ramp,
            |compare, period| {
                pwm.set_duty_all(compare);
                pwm.set_auto_reload(period);
            },
        );
        let pwm = &mut tick.pwm;
        COM_LOCAL.with_masked(|com| {
            if CONFIG_DIRTY.swap(false, Ordering::AcqRel) {
                // SAFETY: Interrupts are masked (`with_masked`), so no input
                // ISR or main-loop config accessor can overlap this copy.
                tick.config = unsafe { INPUT_LOCAL.get_unchecked().config };
                latch_com_config_flags(&tick.config);
            }
            if shared.isr_action() == rm32::shared_comm::IsrAction::AllOff {
                shared.take_duty_kick_down();
                shared.take_duty_kick_half();
                shared.set_changeover_step(0);
                com.com_timer.disable_interrupt();
                com.phase.all_off();
                pwm.set_duty_all(0);
                pwm.generate_update_event();
                com.comp.mask_irq();
                shared.clear_isr_action(rm32::shared_comm::IsrAction::AllOff);
                return;
            }
            rm32::control::isr_logic::steady_running_tick_sync_com(
                shared,
                &mut com.commutation,
                &mut com.bemf,
                &com.interval,
            );
        });
    } else {
        COM_LOCAL.with_masked(|com| {
            if CONFIG_DIRTY.swap(false, Ordering::AcqRel) {
                // SAFETY: Interrupts are masked (`with_masked`), so no input
                // ISR or main-loop config accessor can overlap this copy.
                tick.config = unsafe { INPUT_LOCAL.get_unchecked().config };
                latch_com_config_flags(&tick.config);
            }
            let mut hal = SplitMotorHal {
                pwm: &mut tick.pwm,
                comp: &mut com.comp,
                interval: &mut com.interval,
                com_timer: &mut com.com_timer,
                phase: &mut com.phase,
            };
            let mut ctx = rm32::control::context::MotorContext {
                commutation: &mut com.commutation,
                bemf: &mut com.bemf,
                duty: &mut tick.duty,
                config: &tick.config,
                armed_timeout_count: &mut tick.armed_timeout_count,
                voltage_based_ramp: tick.voltage_based_ramp,
                shared,
                hal: &mut hal,
            };
            rm32::control::isr_logic::ten_khz_tick(&mut ctx);
        });
    }

    let request = shared.take_tone_request();
    let bridge_busy = rm32::tone::bridge_busy(shared, tick.duty.active_brake_on());
    if !tick.tone.active() && (request as u8 == 0 || bridge_busy) {
        return;
    }
    // Run `tone::step` with interrupts unmasked (`PRIMASK = 0`), acquiring
    // `COM_LOCAL.with_masked` only inside `LockedComPhase` when a note transition
    // or silence actually writes the phase driver.
    let mut locked_phase = LockedComPhase;
    let config = &tick.config;
    let pwm = &mut tick.pwm;
    rm32::tone::step::<crate::platform::StmPlatform, _, _>(
        &mut tick.tone,
        request,
        bridge_busy,
        config,
        shared.tim1_arr(),
        || crate::bridge::StmBridge {
            pwm,
            phase: &mut locked_phase,
        },
    );
    shared.set_tone_active(tick.tone.active());
}

/// Commutation timer expired (TIM14/TIM16 ISR body, priority 0).
pub fn handle_tim14() {
    if !crate::timer::take_commutation_interrupt() {
        return;
    }
    ensure_split_init();
    let shared = isr::shared();
    // SAFETY: `TIM14`/`TIM16` and `COMP` share the same NVIC preemption
    // priority on every MCU and cannot preempt each other; lower-priority
    // callers (`handle_tim6`, `service_sine`) mask interrupts via `with_masked`.
    let com = unsafe { COM_LOCAL.get_unchecked() };
    let cfg_flags = COM_CONFIG_FLAGS.load(Ordering::Relaxed);
    let pwm_mode = match cfg_flags & CFG_FLAG_COMP_PWM != 0 {
        false => rm32::hal::PwmMode::Diode,
        true => rm32::hal::PwmMode::Complementary,
    };
    rm32::control::isr_logic::commutation_timer_expired::<
        crate::platform::StmPlatform,
        _,
        _,
        _,
        _,
        _,
    >(
        &mut com.commutation,
        &mut com.bemf,
        shared,
        &com.writer,
        &mut com.com_timer,
        &mut com.comp,
        &mut com.phase,
        pwm_mode,
        cfg_flags & CFG_FLAG_BIDIR != 0,
        cfg_flags & CFG_FLAG_STRICT_CHANGEOVER != 0,
    );
}

/// Comparator interrupt entry point across all MCUs (priority 0): runs the
/// core's half-average-interval acceptance gate (`on_bemf_irq`, matching AM32
/// `COMP_IRQHandler`), then delegates to `bemf_zero_cross` when the gate opens.
#[inline(always)]
pub fn handle_comp_irq() {
    use rm32::control::isr_logic::BemfIrq;
    use rm32::hal::BemfSensor as _;
    ensure_split_init();
    let shared = isr::shared();
    // SAFETY: Same priority-0 invariant as `handle_tim14`.
    let com = unsafe { COM_LOCAL.get_unchecked() };
    if !shared.running() || shared.old_routine() {
        com.comp.mask_irq();
        com.comp.clear_pending();
        return;
    }
    if rm32::control::isr_logic::on_bemf_irq(
        shared,
        &mut com.comp,
        &com.interval,
        com.commutation.rising(),
        || {},
    ) == BemfIrq::Open
    {
        let _accepted = rm32::control::isr_logic::bemf_zero_cross(
            shared,
            &com.commutation,
            &mut com.bemf,
            &mut com.comp,
            &mut com.interval,
            &mut com.com_timer,
        );
    }
}

/// DMA transfer complete (input capture ISR body).
pub fn handle_dma_tc() {
    ensure_split_init();
    let shared = isr::shared();
    INPUT_LOCAL.with_input_irqs_masked(|inp| {
        if shared.armed() && shared.dshot_telemetry() {
            if inp.input.is_output() {
                inp.input.start_capture();
            } else {
                rm32::edt::encode_response(
                    &mut inp.edt,
                    shared,
                    inp.input.gcr_buffer(),
                    <crate::platform::StmPlatform as rm32::platform::Platform>::GCR_SHIFT,
                    |_| {},
                );

                inp.input.start_gcr_output();
            }
        }
    });
}

/// Software-triggered frame processing (EXTI ISR body).
pub fn handle_exti_frame() -> CaptureConfig {
    ensure_split_init();
    let shared = isr::shared();
    INPUT_LOCAL.with_input_irqs_masked(|inp| {
        let buf = inp.input.edges();

        let pin_high = inp.input.input_pin_state();

        let mut zic = shared.zero_input_count();
        let actions = inp.transfer.process::<crate::platform::StmPlatform>(
            buf,
            shared.input_set(),
            shared.dshot(),
            shared.servo_pwm(),
            shared.dshot_telemetry(),
            shared.armed(),
            pin_high,
            shared.adjusted_input(),
            shared.newinput(),
            inp.config.bi_direction != 0,
            inp.config.disable_stick_calibration != 0,
            &mut zic,
            inp.frametime_low,
            inp.frametime_high,
        );
        shared.set_zero_input_count(zic);

        let may_mutate_config = inp.cmd.in_programming_mode()
            || matches!(
                actions.action,
                rm32::transfer::TransferAction::DshotCommand { .. }
                    | rm32::transfer::TransferAction::ServoCalibrationDone { .. }
            );
        let prev_config = may_mutate_config.then_some(inp.config);
        rm32::transfer::dispatch(
            &actions,
            shared,
            rm32::transfer::InputContext {
                config: &mut inp.config,
                cmd: &mut inp.cmd,
                edt: &mut inp.edt,
                forward: &mut inp.forward,
                edt_armed: &mut inp.edt_armed,
                edt_arm_enable: inp.edt_arm_enable,
                frametime_low: &mut inp.frametime_low,
                frametime_high: &mut inp.frametime_high,
            },
            |_| {},
            |prescaler| inp.input.set_output_prescaler(prescaler),
        );
        if prev_config.is_some_and(|prev| prev.as_bytes() != inp.config.as_bytes()) {
            latch_com_config_flags(&inp.config);
            CONFIG_DIRTY.store(true, Ordering::Release);
        }
        if let Some(psc) = actions.next_capture.prescaler {
            inp.input.set_capture_prescaler(psc);
        }
        actions.next_capture
    })
}

/// CRSF UART RX byte handler. Call from UART RX interrupt with each received byte.
pub fn handle_crsf_byte(byte: u8) {
    ensure_split_init();
    let shared = isr::shared();
    CRSF_LOCAL.with_masked(|crsf| {
        crsf.dispatch_byte(byte, shared);
    });
}
