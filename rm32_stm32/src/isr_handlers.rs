//! ISR logic functions — shared between MCU targets.
//!
//! Each function contains the actual ISR body. The MCU-specific
//! `#[interrupt]` wrappers in `interrupts_g071.rs` / `interrupts_f051.rs`
//! just clear the flag and call these.

use crate::isr::{self, IsrState};
use rm32::hal::InputCapture;

/// Single-core ISR-local cell for zero-overhead mutable ISR state.
///
/// # Safety invariants for `Sync` impl
///
/// This type wraps `UnsafeCell<Option<IsrState>>` and implements `Sync`
/// (required for `static` placement). This is sound because:
///
/// 1. **Single writer**: Only called from ISR handlers that share the same
///    NVIC priority level. Cortex-M's priority-based preemption model
///    guarantees that equal-priority ISRs cannot preempt each other.
///
/// 2. **Single core**: All STM32 targets (G071/F051/L431/G431) are
///    single-core. No other hart can access this cell.
///
/// 3. **No main-loop access**: The main loop communicates via `SharedState`
///    atomics, never touching `ISR_LOCAL`.
///
/// 4. **Init-once**: `get()` lazily initializes from `take_isr_state()`
///    exactly once (first ISR invocation). Subsequent calls return the
///    same `&mut`. The `Option` transitions None→Some exactly once.
///
/// If any of these invariants change (e.g., adding a higher-priority ISR
/// that accesses motor state), this must be replaced with
/// `cortex_m::interrupt::Mutex<RefCell<...>>`.
struct IsrCell(core::cell::UnsafeCell<Option<IsrState>>);

// SAFETY: See struct-level doc. Single-core + same-priority ISR = exclusive access.
unsafe impl Sync for IsrCell {}

impl IsrCell {
    const fn new() -> Self { Self(core::cell::UnsafeCell::new(None)) }

    /// Get or initialize the ISR state.
    /// If never initialized, enters emergency shutdown (all FETs off).
    #[inline(always)]
    #[allow(clippy::mut_from_ref)]
    fn get(&self) -> &mut IsrState {
        // SAFETY: Called only from ISR context at a single priority level.
        // No concurrent access possible (see struct-level safety doc).
        let opt = unsafe { &mut *self.0.get() };
        opt.get_or_insert_with(|| {
            match isr::take_isr_state() {
                Some(s) => s,
                None => {
                    use rm32::hal::EmergencyOff;
                    crate::emergency::G0AEmergencyOff::emergency_off();
                    loop { cortex_m::asm::nop(); }
                }
            }
        })
    }
}

static ISR_LOCAL: IsrCell = IsrCell::new();

/// 20kHz control loop tick (TIM6 ISR body).
pub fn handle_tim6() {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();
    let mut counters = rm32::control::isr_logic::TickCounters {
        ten_khz_counter: state.ten_khz_counter,
        one_khz_loop_counter: state.one_khz_loop_counter,
        armed_timeout_count: state.armed_timeout_count,
        tim1_arr: state.tim1_arr,
        voltage_based_ramp: state.voltage_based_ramp,
    };
    rm32::control::isr_logic::ten_khz_tick(
        &mut state.commutation,
        &mut state.bemf,
        &mut state.duty,
        &state.config,
        &mut counters,
        shared,
        &mut state.hal.pwm,
        &mut state.hal.comp,
        &mut state.hal.phase,
        &mut state.hal.interval,
    );
    state.ten_khz_counter = counters.ten_khz_counter;
    state.one_khz_loop_counter = counters.one_khz_loop_counter;
    state.armed_timeout_count = counters.armed_timeout_count;
    state.tim1_arr = counters.tim1_arr;
}

/// Commutation timer expired (TIM14 ISR body).
pub fn handle_tim14() {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();
    rm32::control::isr_logic::commutation_timer_expired(
        &mut state.commutation,
        &mut state.bemf,
        shared,
        &mut state.hal.com_timer,
        &mut state.hal.comp,
        &mut state.hal.phase,
    );
}

/// BEMF zero-cross detected (COMP ISR body).
pub fn handle_comp() {
    let state = ISR_LOCAL.get();
    rm32::control::isr_logic::bemf_zero_cross(
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
                value_12bit, gcr, 7, crate::config::GCR_SHIFT,
            );

            state.hal.input.send_dshot_dma();
        }
    }
}

/// Software-triggered frame processing (EXTI ISR body).
pub fn handle_exti_frame() {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();

    let buf = state.hal.input.dma_buffer();

    let pin_high = state.hal.input.input_pin_state();

    let mut zic = shared.zero_input_count();
    let actions = state.transfer.process(
        buf, shared.input_set(), shared.dshot(), shared.servo_pwm(),
        shared.dshot_telemetry(),
        shared.armed(), pin_high, shared.adjusted_input(), shared.newinput(),
        state.config.bi_direction != 0, state.config.disable_stick_calibration != 0,
        &mut zic, state.frametime_low, state.frametime_high,
    );
    shared.set_zero_input_count(zic);

    if let Some(v) = actions.newinput { shared.set_newinput(v); }
    if actions.send_telemetry { shared.set_send_telemetry(true); }
    if actions.signal_timeout_reset { shared.set_signal_timeout(0); }
    if actions.input_detected {
        shared.set_input_set(true);
        if actions.input_is_dshot { shared.set_dshot(true); }
        if actions.input_is_servo { shared.set_servo_pwm(true); }
    }
    if let Some(fth) = actions.frametime_high { state.frametime_high = fth; }
    if let Some(ftl) = actions.frametime_low { state.frametime_low = ftl; }

    // DShot command dispatch
    if actions.dshot_command > 0 {
        let result = state.cmd.process(
            actions.dshot_command,
            shared.armed(),
            shared.running(),
            &mut state.config,
            &mut state.forward,
            &mut state.edt_armed,
            state.cmd.extended_telemetry,
        );
        match result {
            rm32::dshot_commands::CommandResult::SaveSettings => {
                shared.set_save_settings_flag(true);
            }
            rm32::dshot_commands::CommandResult::PlayTone(_tone) => {
                // Beacons: handled in main loop
            }
            rm32::dshot_commands::CommandResult::SendEscInfo => {
                shared.set_send_esc_info_flag(true);
            }
            _ => {}
        }

        // Propagate EDT init/deinit flags from CommandProcessor to scheduler
        if state.cmd.send_edt_init {
            state.cmd.send_edt_init = false;
            state.edt.send_init = true;
        }
        if state.cmd.send_edt_deinit {
            state.cmd.send_edt_deinit = false;
            state.edt.send_deinit = true;
        }
    }
}

/// CRSF UART RX byte handler. Call from UART RX interrupt with each received byte.
pub fn handle_crsf_byte(byte: u8) {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();

    if let Some(rm32::crsf::CrsfResult::Channels(channels)) = state.crsf.feed(byte) {
        let ch = state.crsf.throttle_channel as usize;
        let throttle = rm32::crsf::CrsfParser::channel_to_throttle(channels[ch]);
        shared.set_newinput(throttle);
        shared.set_signal_timeout(0);
        if !shared.input_set() {
            shared.set_input_set(true);
        }
    }
}
