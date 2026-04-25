//! ISR logic functions — shared between MCU targets.
//!
//! Each function contains the actual ISR body. The MCU-specific
//! `#[interrupt]` wrappers in `interrupts_g071.rs` / `interrupts_f051.rs`
//! just clear the flag and call these.

use crate::isr::{self, IsrState};

/// Single-core ISR-local cell. Safe because:
/// - Only accessed from ISR context (same priority level, no preemption)
/// - Single-core Cortex-M guarantees no concurrent access
struct IsrCell(core::cell::UnsafeCell<Option<IsrState>>);
unsafe impl Sync for IsrCell {}

impl IsrCell {
    const fn new() -> Self { Self(core::cell::UnsafeCell::new(None)) }

    /// Get or initialize the ISR state.
    /// If never initialized, enters emergency shutdown (all FETs off).
    #[inline(always)]
    #[allow(clippy::mut_from_ref)] // Intentional: UnsafeCell interior mutability for ISR-local state
    fn get(&self) -> &mut IsrState {
        let opt = unsafe { &mut *self.0.get() };
        opt.get_or_insert_with(|| {
            match isr::take_isr_state() {
                Some(s) => s,
                None => {
                    // Emergency: all FETs off via GPIO BSRR (no HAL — state is missing)
                    use crate::periph_addr;
                    const BSRR: u32 = 0x18; // GPIO Bit Set/Reset Register offset
                    unsafe {
                        // Reset PA7/8/9/10 (high-side FETs off)
                        ((periph_addr::GPIOA + BSRR) as *mut u32).write_volatile(
                            (1 << (7+16)) | (1 << (8+16)) | (1 << (9+16)) | (1 << (10+16))
                        );
                        // Reset PB0/1 (low-side FETs off)
                        ((periph_addr::GPIOB + BSRR) as *mut u32).write_volatile(
                            (1 << 16) | (1 << (1+16))
                        );
                    }
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
            use rm32::hal::InputCapture;
            state.hal.input.receive_dshot_dma();
        } else {
            #[cfg(feature = "stm32g071")]
            let gcr = state.hal.input.gcr_buffer();
            #[cfg(feature = "stm32f051")]
            let gcr = state.hal.input.gcr_buffer();
            #[cfg(feature = "stm32l431")]
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
            use rm32::hal::InputCapture;
            state.hal.input.send_dshot_dma();
        }
    }
}

/// Software-triggered frame processing (EXTI ISR body).
pub fn handle_exti_frame() {
    let state = ISR_LOCAL.get();
    let shared = isr::shared();

    #[cfg(feature = "stm32g071")]
    let buf = state.hal.input.dma_buffer();
    #[cfg(feature = "stm32f051")]
    let buf = state.hal.input.dma_buffer();
    #[cfg(feature = "stm32l431")]
    let buf = state.hal.input.dma_buffer();

    let pin_high = {
        use rm32::hal::InputCapture;
        state.hal.input.input_pin_state()
    };

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
