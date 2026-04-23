//! Interrupt handlers for the ESC.
//!
//! Each ISR accesses IsrState via a local static (taken from global on first call)
//! and SharedState via atomic reads/writes.

use stm32g0xx_hal::stm32::interrupt;

use crate::isr::{self, IsrState};
use crate::shared::SharedState;
use rm32::hal::{PwmOutput, Comparator, IntervalTimer, ComTimer, PhaseOutput};
use rm32::functions::map;

/// ISR-local state. Initialized on first interrupt entry.
/// Safe because: single-core, same-priority ISRs can't preempt each other,
/// and main never accesses this after interrupts are enabled.
static mut ISR_LOCAL: Option<IsrState> = None;

/// Get or initialize the ISR-local state.
///
/// # Safety
/// Only called from ISR context. On Cortex-M0+ all ISRs are same priority
/// (no nesting), so this is effectively single-threaded access.
#[inline(always)]
unsafe fn isr_state() -> &'static mut IsrState {
    ISR_LOCAL.get_or_insert_with(|| {
        isr::take_isr_state().expect("ISR state not initialized")
    })
}

/// TIM6 interrupt — 20kHz control loop tick.
/// Equivalent to C's TIM6_DAC_IRQHandler → tenKhzRoutine().
#[interrupt]
fn TIM6_DAC_LPTIM1() {
    let state = unsafe { isr_state() };
    let shared = isr::shared();

    // Clear interrupt flag
    let tim6 = unsafe { &*stm32g0xx_hal::stm32::TIM6::ptr() };
    tim6.sr().modify(|_, w| w.uif().clear_bit());

    // --- Throttle → setpoint mapping ---
    // Read newinput from shared (set by DShot decode ISR) and compute setpoint.
    // Unidirectional: adjusted_input = newinput, input = adjusted_input.
    // If armed and input >= 47: setpoint = map(input, 47, 2047, min_duty, 2000).
    let newinput = shared.newinput();
    shared.set_adjusted_input(newinput);
    if shared.armed() && !shared.stepper_sine() {
        if newinput >= 47 {
            // Map throttle to duty cycle setpoint
            let min_duty = state.duty.minimum;
            let setpoint = map(newinput as i32, 47, 2047, min_duty as i32, 2000) as u16;
            shared.set_duty_cycle_setpoint(setpoint);

            // Start motor if not running
            if !shared.running() {
                shared.set_running(true);
                state.duty.last = state.duty.min_startup;
                // Commutate once to start (equivalent to startMotor)
                let step = state.commutation.advance();
                state.hal.phase.com_step(step);
                state.hal.comp.set_step(step, state.commutation.rising);
                state.hal.comp.change_input();
                state.hal.comp.enable_interrupts();
            }
        } else {
            shared.set_duty_cycle_setpoint(0);
        }
    }

    // --- Core 20kHz tick logic ---
    let setpoint = shared.duty_cycle_setpoint();
    state.duty.cycle = setpoint;
    state.ten_khz_counter += 1;
    shared.increment_signal_timeout();
    state.duty.ramp_count += 1;
    state.one_khz_loop_counter += 1;

    // Arming
    if !shared.armed() {
        if shared.input_set() && shared.adjusted_input() == 0 {
            state.armed_timeout_count += 1;
            if state.armed_timeout_count > 20000 {
                shared.set_armed(true);
                state.armed_timeout_count = 0;
            }
        } else {
            state.armed_timeout_count = 0;
        }
    }

    // --- Old routine BEMF polling ---
    // During startup, motor commutation is driven by polling the comparator
    // rather than waiting for interrupt-driven zero-cross detection.
    if shared.old_routine() && shared.running() && !shared.stepper_sine() {
        state.hal.comp.mask_interrupts();

        // Sample comparator (getBemfState equivalent)
        let comp_level = state.hal.comp.output_level();
        let current_state = !comp_level; // polarity reversed
        if state.commutation.rising {
            if current_state {
                state.bemf.counter += 1;
            } else {
                state.bemf.bad_count += 1;
                if state.bemf.bad_count > state.bemf.bad_count_threshold {
                    state.bemf.counter = 0;
                }
            }
        } else {
            if !current_state {
                state.bemf.counter += 1;
            } else {
                state.bemf.bad_count += 1;
                if state.bemf.bad_count > state.bemf.bad_count_threshold {
                    state.bemf.counter = 0;
                }
            }
        }

        // Check if enough BEMF counts for zero-cross
        let threshold = if state.commutation.rising {
            state.bemf.min_counts_up
        } else {
            state.bemf.min_counts_down
        };

        if !state.bemf.zc_found && state.bemf.counter > threshold {
            state.bemf.zc_found = true;

            // zcfoundroutine: read interval, compute timing, commutate
            state.bemf.this_zc_time = state.hal.interval.count() as u16;
            state.hal.interval.set_count(0);

            let ci = shared.commutation_interval();
            let new_ci = (state.bemf.this_zc_time as u32 + 3 * ci) / 4;
            shared.set_commutation_interval(new_ci);

            let advance = (state.bemf.temp_advance as u32 * new_ci) >> 6;
            state.bemf.wait_time = (new_ci as u16 / 2).wrapping_sub(advance as u16);

            // Brief wait for commutation timing (non-blocking on early crosses)
            let zc = shared.zero_crosses();
            if zc >= 5 {
                // Busy-wait (matching C zcfoundroutine behavior)
                while (state.hal.interval.count() as u16) < state.bemf.wait_time {
                    // Tight loop — exits when timer exceeds wait_time
                }
            }

            // Commutate
            let step = state.commutation.advance();
            state.hal.phase.com_step(step);
            state.hal.comp.set_step(step, state.commutation.rising);
            state.hal.comp.change_input();
            state.bemf.counter = 0;
            state.bemf.bad_count = 0;

            shared.increment_zero_crosses();

            // Check for transition to interrupt-driven mode
            let zc = shared.zero_crosses();
            let ci = shared.commutation_interval();
            if zc >= 20 && ci <= 2000 {
                shared.set_old_routine(false);
                state.hal.comp.enable_interrupts();
            }
        }
    }

    // Ramp rate limiting
    if state.duty.ramp_count > state.duty.ramp_divider as u16 {
        state.duty.ramp_count = 0;

        let zc = shared.zero_crosses();
        if zc < 150 || state.duty.last < 150 {
            state.duty.max_change = state.duty.max_ramp_startup;
        } else if state.duty.last > 500 {
            // Using last_duty as proxy for average_interval comparison
            state.duty.max_change = state.duty.max_ramp_low_rpm;
        } else {
            state.duty.max_change = state.duty.max_ramp_high_rpm;
        }

        let change = state.duty.max_change as u16;
        if state.duty.cycle > state.duty.last + change {
            state.duty.cycle = state.duty.last + change;
        }
        if state.duty.last > state.duty.cycle + change {
            state.duty.cycle = state.duty.last - change;
        }
    } else {
        state.duty.cycle = state.duty.last;
    }

    // PWM output
    let tim1_arr = 1999u16; // TODO: get from config/shared
    if shared.armed() && shared.running() {
        let adj = ((state.duty.cycle as u32 * tim1_arr as u32) / 2000 + 1) as u16;
        state.hal.pwm.set_duty_all(adj);
    } else {
        state.hal.pwm.set_duty_all(0);
    }

    state.duty.last = state.duty.cycle;
    state.hal.pwm.set_auto_reload(tim1_arr);
}

/// TIM14 interrupt — commutation timer expired.
/// Equivalent to C's TIM14_IRQHandler → PeriodElapsedCallback().
#[interrupt]
fn TIM14() {
    let state = unsafe { isr_state() };
    let shared = isr::shared();

    // Clear interrupt flag
    state.hal.com_timer.disable_interrupt();

    // Commutate
    let step = state.commutation.advance();
    state.hal.phase.com_step(step);
    state.hal.comp.set_step(step, state.commutation.rising);
    state.hal.comp.change_input();

    // Update commutation interval
    let zc_avg = ((state.bemf.last_zc_time as u32 + state.bemf.this_zc_time as u32) >> 1) as u32;
    let ci = shared.commutation_interval();
    let new_ci = (ci + zc_avg) >> 1;
    shared.set_commutation_interval(new_ci);

    // Advance calculation
    let advance = (new_ci * state.bemf.temp_advance as u32) >> 6;
    state.bemf.wait_time = (new_ci as u16 >> 1).wrapping_sub(advance as u16);

    // Enable comparator interrupts (switch to interrupt-driven mode)
    state.hal.comp.enable_interrupts();

    // Increment zero crosses
    shared.increment_zero_crosses();

    state.bemf.counter = 0;
    state.bemf.zc_found = false;
}

/// ADC/COMP interrupt — BEMF zero-cross detected.
/// Equivalent to C's ADC1_COMP_IRQHandler → interruptRoutine().
#[interrupt]
fn ADC_COMP() {
    let state = unsafe { isr_state() };
    let _shared = isr::shared();

    // Filter: check comparator multiple times
    for _ in 0..state.bemf.filter_level {
        if state.hal.comp.output_level() == state.commutation.rising {
            return; // false alarm
        }
    }

    // Zero-cross confirmed
    state.hal.comp.mask_interrupts();
    state.bemf.last_zc_time = state.bemf.this_zc_time;
    state.bemf.this_zc_time = state.hal.interval.count() as u16;
    state.hal.interval.set_count(0);

    // Set commutation timer for next event
    state.hal.com_timer.set_and_enable(state.bemf.wait_time + 1);
}

/// DMA1 Channel 1 interrupt — input capture or telemetry TX complete.
/// Handles bidirectional DShot telemetry mode switching (RX↔TX).
#[interrupt]
fn DMA1_CHANNEL1() {
    let state = unsafe { isr_state() };
    let shared = isr::shared();
    let dma = unsafe { &*stm32g0xx_hal::stm32::DMA1::ptr() };

    if dma.isr().read().tcif1().bit_is_set() {
        dma.ifcr().write(|w| w.cgif1().set_bit());
        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());

        // Bidirectional DShot telemetry mode
        if shared.armed() && shared.dshot_telemetry() {
            if state.hal.input.is_output() {
                // Just finished sending GCR response → switch back to receive
                use rm32::hal::InputCapture;
                state.hal.input.receive_dshot_dma();
                // Frame will be decoded on next DMA complete
            } else {
                // Just finished receiving → encode GCR response and send
                let gcr = unsafe { crate::input_capture::gcr_buffer() };
                rm32::dshot::encode_telemetry_with_shift(
                    shared.e_com_time() as u16,
                    shared.running(),
                    gcr,
                    7, // buffer_padding for G071
                    rm32::dshot::GCR_SHIFT_G0,
                );
                use rm32::hal::InputCapture;
                state.hal.input.send_dshot_dma();
            }
            // Trigger EXTI for throttle processing
            let exti = unsafe { &*stm32g0xx_hal::stm32::EXTI::ptr() };
            exti.swier1().write(|w| unsafe { w.bits(1 << 15) });
            return;
        }

        // Normal (non-bidir) path: trigger EXTI for frame processing
        let exti = unsafe { &*stm32g0xx_hal::stm32::EXTI::ptr() };
        exti.swier1().write(|w| unsafe { w.bits(1 << 15) });
    }

    if dma.isr().read().htif1().bit_is_set() {
        dma.ifcr().write(|w| w.chtif1().set_bit());
    }
}

/// EXTI4_15 interrupt — software-triggered frame processing.
/// Full transfer complete dispatcher: DShot decode, servo input, auto-detect,
/// command processing, unarmed housekeeping.
#[interrupt]
fn EXTI4_15() {
    let state = unsafe { isr_state() };
    let shared = isr::shared();

    // Clear EXTI15 pending
    let exti = unsafe { &*stm32g0xx_hal::stm32::EXTI::ptr() };
    exti.rpr1().write(|w| unsafe { w.bits(1 << 15) });
    exti.fpr1().write(|w| unsafe { w.bits(1 << 15) });

    // Read DMA buffer
    let buf = unsafe { crate::input_capture::dma_buffer() };

    // Determine input pin state (for servo edge detection)
    let gpiob = unsafe { &*stm32g0xx_hal::stm32::GPIOB::ptr() };
    let pin_high = gpiob.idr().read().bits() & (1 << 4) != 0;

    // Run transfer complete dispatcher
    let mut zero_input_count = shared.zero_input_count();
    let actions = state.transfer.process(
        buf,
        shared.input_set(),
        shared.dshot(),
        shared.servo_pwm(),
        shared.armed(),
        pin_high,
        shared.adjusted_input(),
        shared.newinput(),
        state.config.bi_direction != 0,
        state.config.disable_stick_calibration != 0,
        &mut zero_input_count,
        state.frametime_low,
        state.frametime_high,
    );
    shared.set_zero_input_count(zero_input_count);

    // Apply actions
    if let Some(v) = actions.newinput {
        shared.set_newinput(v);
    }
    if actions.send_telemetry {
        shared.set_send_telemetry(true);
    }
    if actions.signal_timeout_reset {
        shared.set_signal_timeout(0);
    }
    if actions.input_detected {
        shared.set_input_set(true);
        if actions.input_is_dshot { shared.set_dshot(true); }
        if actions.input_is_servo { shared.set_servo_pwm(true); }
    }
    if let Some(fth) = actions.frametime_high {
        state.frametime_high = fth;
    }
    if let Some(ftl) = actions.frametime_low {
        state.frametime_low = ftl;
    }

    // DShot command dispatch (if a command was decoded)
    // The transfer dispatcher sets newinput=0 for commands;
    // we detect this and check if the frame was actually a command.
    // For proper command dispatch, we re-decode here since TransferState
    // doesn't expose the command number directly.
    // TODO: have TransferActions carry the command number.

    // Re-enable DMA capture for next frame
    let dma = unsafe { &*stm32g0xx_hal::stm32::DMA1::ptr() };
    let tim3 = unsafe { &*stm32g0xx_hal::stm32::TIM3::ptr() };
    let buf_size = if shared.servo_pwm() && pin_high { 3u32 } else if shared.servo_pwm() { 2 } else { 32 };
    dma.ch(0).ndtr().write(|w| unsafe { w.bits(buf_size) });
    dma.ch(0).cr().modify(|_, w| w.en().set_bit());
    tim3.cr1().modify(|_, w| w.cen().set_bit());
}
