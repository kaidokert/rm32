//! ISR logic functions — shared between MCU targets.
//!
//! Each function contains the actual ISR body. The MCU-specific
//! `#[interrupt]` wrappers in `interrupts_g071.rs` / `interrupts_f051.rs`
//! just clear the flag and call these.

use crate::isr::{self, IsrState};
use rm32::hal::{PwmOutput, Comparator, IntervalTimer, ComTimer, PhaseOutput};

/// ISR-local state. Taken from global on first ISR entry.
static mut ISR_LOCAL: Option<IsrState> = None;

/// Get or initialize the ISR-local state.
/// # Safety
/// Only call from ISR context (single-core, same-priority).
#[inline(always)]
pub unsafe fn isr_state() -> &'static mut IsrState {
    ISR_LOCAL.get_or_insert_with(|| {
        isr::take_isr_state().expect("ISR state not initialized")
    })
}

/// 20kHz control loop tick (TIM6 ISR body).
pub fn handle_tim6() {
    let state = unsafe { isr_state() };
    let shared = isr::shared();
    let mut counters = rm32::control::isr_logic::TickCounters {
        ten_khz_counter: state.ten_khz_counter,
        one_khz_loop_counter: state.one_khz_loop_counter,
        armed_timeout_count: state.armed_timeout_count,
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
}

/// Commutation timer expired (TIM14 ISR body).
pub fn handle_tim14() {
    let state = unsafe { isr_state() };
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
    let state = unsafe { isr_state() };
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
    let state = unsafe { isr_state() };
    let shared = isr::shared();

    if shared.armed() && shared.dshot_telemetry() {
        if state.hal.input.is_output() {
            use rm32::hal::InputCapture;
            state.hal.input.receive_dshot_dma();
        } else {
            #[cfg(feature = "stm32g071")]
            let gcr = unsafe { crate::input_capture::gcr_buffer() };
            #[cfg(feature = "stm32f051")]
            let gcr = unsafe { crate::input_capture_f051::gcr_buffer() };
            #[cfg(feature = "stm32l431")]
            let gcr = unsafe { crate::input_capture_l431::gcr_buffer() };

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
            use rm32::hal::InputCapture;
            state.hal.input.send_dshot_dma();
        }
    }
}

/// Software-triggered frame processing (EXTI ISR body).
pub fn handle_exti_frame() {
    let state = unsafe { isr_state() };
    let shared = isr::shared();

    #[cfg(feature = "stm32g071")]
    let buf = unsafe { crate::input_capture::dma_buffer() };
    #[cfg(feature = "stm32f051")]
    let buf = unsafe { crate::input_capture_f051::dma_buffer() };
    #[cfg(feature = "stm32l431")]
    let buf = unsafe { crate::input_capture_l431::dma_buffer() };

    #[cfg(feature = "stm32g071")]
    let pin_high = unsafe { (0x4800_0410 as *const u32).read_volatile() } & (1 << 4) != 0; // GPIOB IDR, PB4
    #[cfg(feature = "stm32f051")]
    let pin_high = unsafe { (0x4800_0010 as *const u32).read_volatile() } & (1 << 2) != 0; // GPIOA IDR, PA2
    #[cfg(feature = "stm32l431")]
    let pin_high = unsafe { (0x4800_0010 as *const u32).read_volatile() } & (1 << 15) != 0; // GPIOA IDR, PA15

    let mut zic = shared.zero_input_count();
    let actions = state.transfer.process(
        buf, shared.input_set(), shared.dshot(), shared.servo_pwm(),
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
        let mut edt_armed = false;
        let result = state.cmd.process(
            actions.dshot_command,
            shared.armed(),
            shared.running(),
            &mut state.config,
            &mut state.forward,
            &mut edt_armed,
            state.cmd.extended_telemetry,
        );
        match result {
            rm32::dshot_commands::CommandResult::SaveSettings => {
                // Signal main loop to save (via shared flag or similar)
            }
            rm32::dshot_commands::CommandResult::PlayTone(_tone) => {
                // Beacons: would need PWM output, handled in main
            }
            rm32::dshot_commands::CommandResult::SendEscInfo => {
                // Signal main loop to send ESC info packet
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
    let state = unsafe { isr_state() };
    let shared = isr::shared();

    if let Some(result) = state.crsf.feed(byte) {
        match result {
            rm32::crsf::CrsfResult::Channels(channels) => {
                let ch = state.crsf.throttle_channel as usize;
                let throttle = rm32::crsf::CrsfParser::channel_to_throttle(channels[ch]);
                shared.set_newinput(throttle);
                shared.set_signal_timeout(0);
                if !shared.input_set() {
                    shared.set_input_set(true);
                }
            }
            _ => {} // BadCrc, OtherFrame, Incomplete — ignore
        }
    }
}
