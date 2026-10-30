//! ISR logic functions — shared between MCU targets.
//!
//! Each function contains the actual ISR body. The MCU-specific
//! `#[interrupt]` wrappers in `interrupts_g071.rs` / `interrupts_f051.rs`
//! just clear the flag and call these.

use crate::isr::{self, TargetIsrState};
use rm32::hal::{InputCapture, MotorHal};
use rm32::transfer::CaptureConfig;

/// Single-core ISR-local cell for zero-overhead mutable ISR state.
///
/// # Safety invariants for `Sync` impl
///
/// This type wraps `UnsafeCell<Option<TargetIsrState>>` and implements `Sync`
/// (required for `static` placement). This is sound because:
///
/// 1. **Single writer**: Only called from ISR handlers that share the same
///    NVIC priority level. Cortex-M's priority-based preemption model
///    guarantees that equal-priority ISRs cannot preempt each other.
///
/// 2. **Single core**: All STM32 targets (G071/F051/L431/G431) are
///    single-core. No other hart can access this cell.
///
/// 3. **Serialized main-loop access**: The fixed sine/configuration entry points
///    mask interrupts for their entire borrow. It does not call ISR handlers,
///    expose the reference, or delay while holding it.
///
/// 4. **Init-once**: `get()` lazily initializes from `take_isr_state()`
///    exactly once (first ISR or sine service invocation). Subsequent calls return the
///    same `&mut`. The `Option` transitions None→Some exactly once.
///
/// If any of these invariants change (e.g., adding a higher-priority ISR
/// that accesses motor state), this must be replaced with
/// `cortex_m::interrupt::Mutex<RefCell<...>>`.
struct IsrCell(core::cell::UnsafeCell<Option<TargetIsrState>>);

// SAFETY: See struct-level doc. Single-core + same-priority ISR = exclusive access.
unsafe impl Sync for IsrCell {}

impl IsrCell {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(None))
    }

    /// Get or initialize the ISR state.
    /// Panics if state was never initialized — the project-wide panic handler
    /// in `panic.rs` forces all FETs off before halting.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    fn get(&self) -> &mut TargetIsrState {
        // SAFETY: Called from same-priority ISRs or fixed main-loop entry points with interrupts
        // masked. No concurrent/reentrant access (see struct-level safety doc).
        let opt = unsafe { &mut *self.0.get() };
        opt.get_or_insert_with(|| isr::take_isr_state().expect("ISR state not initialized"))
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
    cortex_m::interrupt::free(|_| {
        let state = ISR_LOCAL.get();
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
        cortex_m::interrupt::free(|_| {
            let state = ISR_LOCAL.get();
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
    cortex_m::interrupt::free(|_| {
        let state = ISR_LOCAL.get();
        rm32::system::SystemTick::<crate::platform::StmPlatform>::prepare_config_save(
            isr::shared(),
            main,
            &state.config,
        )
    })
}

/// 20kHz control loop tick (TIM6 ISR body).
pub fn handle_tim6() {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();
    {
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
    }
    let request = shared.take_tone_request();
    let bridge_busy = !shared.armed()
        || shared.running()
        || shared.stepper_sine()
        || shared.prop_brake_active()
        || state.duty.active_brake_on();
    rm32::tone::step::<crate::platform::StmPlatform, _, _>(
        &mut state.tone,
        request,
        bridge_busy,
        state.config.beep_volume,
        shared.tim1_arr(),
        || state.hal.bridge(),
    );
    shared.set_tone_active(state.tone.active());
}

/// Commutation timer expired (TIM14 ISR body).
pub fn handle_tim14() {
    if !crate::timer::take_commutation_interrupt() {
        return;
    }
    let state = ISR_LOCAL.get();
    let shared = isr::shared();
    rm32::control::isr_logic::commutation_timer_expired::<crate::platform::StmPlatform, _, _, _, _>(
        &mut state.commutation,
        &mut state.bemf,
        shared,
        &mut state.hal.com_timer,
        &mut state.hal.comp,
        &mut state.hal.phase,
        state.config.is_bidirectional(),
        state.config.use_strict_changeover(),
    );
}

/// BEMF zero-cross detected (COMP ISR body).
pub fn handle_comp() {
    let state = ISR_LOCAL.get();
    let _accepted = rm32::control::isr_logic::bemf_zero_cross(
        isr::shared(),
        &state.commutation,
        &mut state.bemf,
        &mut state.hal.comp,
        &mut state.hal.interval,
        &mut state.hal.com_timer,
    );
}

/// DMA transfer complete (input capture ISR body).
pub fn handle_dma_tc() {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();

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
}

/// Software-triggered frame processing (EXTI ISR body).
pub fn handle_exti_frame() -> CaptureConfig {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();

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
    if let Some(tone) = result.tone {
        shared.set_tone_request(tone);
    }
    actions.next_capture
}

/// CRSF UART RX byte handler. Call from UART RX interrupt with each received byte.
pub fn handle_crsf_byte(byte: u8) {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();

    state.crsf.dispatch_byte(byte, shared);
}
