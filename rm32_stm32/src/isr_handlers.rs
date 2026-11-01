//! ISR logic functions — shared between MCU targets.
//!
//! Each function contains the actual ISR body. The MCU-specific
//! `#[interrupt]` wrappers in `interrupts_g071.rs` / `interrupts_f051.rs`
//! just clear the flag and call these.

use crate::isr::{self, TargetIsrState};
use rm32::hal::{InputCapture, MotorHal};
use rm32::transfer::CaptureConfig;

/// Single-core ISR-local cell for mutable ISR state.
///
/// # Safety invariants for `Sync` impl
///
/// This type wraps `UnsafeCell<Option<TargetIsrState>>` and implements `Sync`
/// (required for `static` placement). This is sound because:
///
/// 1. **Interrupt-masked borrows**: `with()` enters `cortex_m::interrupt::free`
///    for the entire lifetime of the `&mut TargetIsrState` reference, so a
///    higher-priority ISR (e.g. `COMP`/`TIM14` at priority 0 preempting `TIM6`
///    or `EXTI`) cannot enter `with()` while another borrow is active.
///
/// 2. **Single core**: All STM32 targets (G071/F051/L431/G431) are
///    single-core. No other hart can access this cell.
///
/// 3. **Non-reentrant closures**: Callers do not re-enter `ISR_LOCAL.with()`
///    from inside the closure, expose the reference, or block inside it.
///
/// 4. **Init-once**: `with()` lazily initializes from `take_isr_state()`
///    exactly once (first ISR or sine service invocation). The `Option`
///    transitions `None` -> `Some` exactly once.
struct IsrCell(core::cell::UnsafeCell<Option<TargetIsrState>>);

// SAFETY: See struct-level doc. Single-core + `interrupt::free` closure scope = exclusive access.
unsafe impl Sync for IsrCell {}

impl IsrCell {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(None))
    }

    /// Borrow the ISR state with interrupts masked for the closure's duration.
    /// Panics if state was never initialized — the project-wide panic handler
    /// in `panic.rs` forces all FETs off before halting.
    #[inline]
    fn with<R>(&self, f: impl FnOnce(&mut TargetIsrState) -> R) -> R {
        cortex_m::interrupt::free(|_| {
            // SAFETY: Interrupts are masked on this single-core MCU for the
            // entire duration of `f`, and closures do not re-enter `with()`.
            let opt = unsafe { &mut *self.0.get() };
            let state = opt
                .get_or_insert_with(|| isr::take_isr_state().expect("ISR state not initialized"));
            f(state)
        })
    }
}

static ISR_LOCAL: IsrCell = IsrCell::new();

/// Run the main-loop sine operation against the live motor state, including
/// after the first ISR has taken it from the initialization cell. Keep the
/// returned delay outside interrupt exclusion.
pub fn service_sine(
    system: &mut rm32::system::SystemTick<crate::platform::StmPlatform>,
    main: &mut rm32::main_state::MainState<crate::platform::StmPlatform>,
    dead_time: i16,
) -> Option<u16> {
    ISR_LOCAL.with(|state| {
        system.service_sine(
            isr::shared(),
            main,
            &mut state.hal,
            &mut state.duty,
            state.commutation.step(),
            dead_time,
        )
    })
}

/// Recover input configuration after the bounded publication queue fills.
/// Access the live ISR cell, not the initialization cell consumed by the first IRQ.
pub fn resync_input_config(main: &mut rm32::main_state::MainState<crate::platform::StmPlatform>) {
    let shared = isr::shared();
    if shared.config_write_overflowed() {
        ISR_LOCAL.with(|state| {
            rm32::system::SystemTick::<crate::platform::StmPlatform>::sync_config_snapshot(
                shared,
                main,
                &state.config,
            );
        });
    }
}

/// Claim a save and copy authoritative bytes with the input producer excluded.
/// Flash I/O must happen after this function returns, outside the exclusion.
pub fn prepare_input_save(
    main: &mut rm32::main_state::MainState<crate::platform::StmPlatform>,
) -> bool {
    if !isr::shared().save_settings_flag() {
        return false;
    }
    ISR_LOCAL.with(|state| {
        rm32::system::SystemTick::<crate::platform::StmPlatform>::prepare_config_save(
            isr::shared(),
            main,
            &state.config,
        )
    })
}

/// 20kHz control loop tick (TIM6 ISR body).
pub fn handle_tim6() {
    let shared = isr::shared();
    ISR_LOCAL.with(|state| {
        let mut ctx = rm32::control::context::MotorContext {
            commutation: &mut state.commutation,
            bemf: &mut state.bemf,
            duty: &mut state.duty,
            config: &state.config,
            armed_timeout_count: &mut state.armed_timeout_count,
            voltage_based_ramp: state.voltage_based_ramp,
            shared,
            hal: &mut state.hal,
        };
        rm32::control::isr_logic::ten_khz_tick(&mut ctx);
    });
    // Separate critical section so any priority-0 COMP/TIM14 interrupt that
    // pended during `ten_khz_tick` can preempt before tone housekeeping runs.
    ISR_LOCAL.with(|state| {
        let request = shared.take_tone_request();
        let bridge_busy = rm32::tone::bridge_busy(shared, state.duty.active_brake_on());
        rm32::tone::step::<crate::platform::StmPlatform, _, _>(
            &mut state.tone,
            request,
            bridge_busy,
            state.config.beep_volume,
            shared.tim1_arr(),
            || state.hal.bridge(),
        );
        shared.set_tone_active(state.tone.active());
    });
}

/// Commutation timer expired (TIM14 ISR body).
pub fn handle_tim14() {
    if !crate::timer::take_commutation_interrupt() {
        return;
    }
    let shared = isr::shared();
    ISR_LOCAL.with(|state| {
        rm32::control::isr_logic::commutation_timer_expired::<
            crate::platform::StmPlatform,
            _,
            _,
            _,
            _,
        >(
            &mut state.commutation,
            &mut state.bemf,
            shared,
            &mut state.hal.com_timer,
            &mut state.hal.comp,
            &mut state.hal.phase,
            state.config.is_bidirectional(),
            state.config.use_strict_changeover(),
        );
    });
}

/// BEMF zero-cross detected (COMP ISR body).
pub fn handle_comp() {
    ISR_LOCAL.with(|state| {
        let _accepted = rm32::control::isr_logic::bemf_zero_cross(
            isr::shared(),
            &state.commutation,
            &mut state.bemf,
            &mut state.hal.comp,
            &mut state.hal.interval,
            &mut state.hal.com_timer,
        );
    });
}

/// DMA transfer complete (input capture ISR body).
pub fn handle_dma_tc() {
    let shared = isr::shared();
    ISR_LOCAL.with(|state| {
        if shared.armed() && shared.dshot_telemetry() {
            if state.hal.input.is_output() {
                state.hal.input.receive_dshot_dma();
            } else {
                let gcr = state.hal.input.gcr_buffer();

                // EDT: decide whether to send eRPM or extended data frame
                let value_12bit = match state.edt.next_frame(
                    shared.actual_current(),
                    shared.battery_voltage(),
                    shared.degrees_celsius(),
                ) {
                    rm32::edt::EdtFrame::Extended(v) => v,
                    rm32::edt::EdtFrame::Erpm => {
                        rm32::dshot::erpm_to_12bit(shared.e_com_time() as u16, shared.running())
                    }
                };
                rm32::dshot::encode_gcr_frame(
                    value_12bit,
                    gcr,
                    7,
                    <crate::platform::StmPlatform as rm32::platform::Platform>::GCR_SHIFT,
                );

                state.hal.input.send_dshot_dma();
            }
        }
    });
}

/// Software-triggered frame processing (EXTI ISR body).
pub fn handle_exti_frame() -> CaptureConfig {
    let shared = isr::shared();
    ISR_LOCAL.with(|state| {
        let buf = state.hal.input.dma_buffer();

        let pin_high = state.hal.input.input_pin_state();

        let mut zic = shared.zero_input_count();
        let actions = state.transfer.process::<crate::platform::StmPlatform>(
            buf,
            shared.input_set(),
            shared.dshot(),
            shared.servo_pwm(),
            shared.dshot_telemetry(),
            shared.armed(),
            pin_high,
            shared.adjusted_input(),
            shared.newinput(),
            state.config.bi_direction != 0,
            state.config.disable_stick_calibration != 0,
            &mut zic,
            state.frametime_low,
            state.frametime_high,
        );
        shared.set_zero_input_count(zic);

        let result = rm32::transfer::dispatch(
            &actions,
            shared,
            rm32::transfer::InputContext {
                config: &mut state.config,
                cmd: &mut state.cmd,
                edt: &mut state.edt,
                forward: &mut state.forward,
                edt_armed: &mut state.edt_armed,
                edt_arm_enable: state.edt_arm_enable,
                frametime_low: &mut state.frametime_low,
                frametime_high: &mut state.frametime_high,
            },
            |prescaler| state.hal.input.set_output_prescaler(prescaler),
        );
        if let Some(psc) = actions.next_capture.prescaler {
            state.hal.input.set_capture_prescaler(psc);
        }
        if let Some(tone) = result.tone {
            shared.set_tone_request(tone);
        }
        actions.next_capture
    })
}

/// CRSF UART RX byte handler. Call from UART RX interrupt with each received byte.
pub fn handle_crsf_byte(byte: u8) {
    let shared = isr::shared();
    ISR_LOCAL.with(|state| {
        state.crsf.dispatch_byte(byte, shared);
    });
}
