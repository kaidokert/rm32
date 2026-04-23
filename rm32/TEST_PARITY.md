# RM32 Test Parity Tracker

## Current: 156/156 unit tests (100% parity with C)
## Blackbox: 29/29 vectors pass on both C and Rust harnesses

## Logic Implementation Status: ~90%

| Category | C Tests | Rust Tests | Logic Status |
|----------|---------|------------|--------------|
| commutate | 10 | 11 | DONE |
| functions/misc | 6 | 9 | DONE |
| pid | 5 | 8 | DONE |
| bemf | 4 | 6 | DONE |
| advance (sine) | 4 | 4 | DONE |
| telemetry | 3 | 4 | DONE |
| interrupt | 3 | 3 | DONE |
| callback | 2 | 4 | DONE |
| current | 2 | 5 | DONE |
| setInput | 24 | 22 | DONE |
| dshot decode/encode | 24 | 13 | DONE |
| dshot commands | — | 13 | DONE |
| mainLoop | 17 | 14 | DONE |
| tenKhz | 14 | 12 | DONE |
| signal | 14 | 10 | DONE |
| eeprom | 6 | 11 | DONE |

## Firmware Status (rm32_stm32, STM32G071)

7.4KB release binary, all ISRs wired, zero placeholder TODOs.

| Subsystem | Status |
|-----------|--------|
| Clock init (64MHz PLL) | DONE |
| TIM1 3-phase PWM + dead-time | DONE |
| TIM2 interval timer | DONE |
| TIM6 20kHz tick ISR | DONE |
| TIM14 commutation ISR | DONE |
| COMP2 BEMF sensing + input mux switching | DONE |
| Phase driver (6-step GPIO commutation) | DONE |
| DMA input capture (TIM3+DMA1, PB4) | DONE |
| DShot frame decode in ISR | DONE |
| EXTI software trigger for frame processing | DONE |
| Watchdog (HAL `.feed()`) | DONE |
| NVIC interrupt routing | DONE |
| SharedState atomics (ISR↔main) | DONE |
| Main loop (desync, eRPM, LVC, temperature) | DONE |

## Remaining Gaps

### P1 — Should have for real-world use

1. **Sounds/Beeps** (`sounds.rs`)
   - Motor-driven beep for startup tune, arming confirmation, beacons
   - Needs: phase output in fixed-frequency mode, delay timing
   - C: ~200 lines in sounds.c
   - Priority: HIGH for user feedback during bench testing

2. **Protocol handover / transfercomplete**
   - Full `transfercomplete()` dispatcher with:
     - DShot bidir telem path (out_put flag switching)
     - Servo PWM pulse measurement
     - Auto-detect (DShot vs servo)
     - Unarmed frame averaging for dshot_frametime calibration
   - Currently: EXTI ISR decodes DShot frames directly (simplified)
   - Priority: HIGH for servo input support

3. **zcfoundroutine (blocking polling mode)**
   - Used during first few commutations before interrupt mode is reliable
   - Blocking busy-wait on INTERVAL_TIMER_COUNT
   - Currently: not ported (old_routine flag exists but polling loop absent)
   - Priority: MEDIUM (motor starts in old_routine, switches to interrupt mode)

### P2 — Nice to have

4. **DroneCAN**
   - Entire UAVCAN/DroneCAN stack (libcanard + message codecs)
   - ~42KB of C code + generated DSDL
   - Priority: LOW for initial testing (DShot is primary input)

5. **WS2812 LED strip**
   - Addressable LED support for some boards
   - Priority: LOW

6. **CRSF serial input**
   - Alternative serial protocol input
   - Priority: LOW

## Architecture Summary

```
rm32 (no_std library, 156 tests, 84% code coverage)
├── pid.rs, commutation.rs, bemf.rs, sine.rs, current.rs
├── dshot.rs, dshot_commands.rs, signal.rs, telemetry.rs
├── functions.rs, config.rs, eeprom.rs
├── control/ (state.rs, tick.rs — set_input, ten_khz_tick, main_loop_tick)
└── hal.rs (traits: PwmOutput, Comparator, PhaseOutput, etc.)

rm32_stm32 (STM32G071 HAL, 7.4KB firmware)
├── pwm.rs (TIM1, 3 PwmPin objects)
├── comparator.rs (COMP2, EXTI18, input mux switching)
├── timer.rs (TIM2 interval, TIM14 one-shot)
├── phase.rs (6-step GPIO commutation)
├── input_capture.rs (TIM3+DMA1, static buffer)
├── comp_init.rs (COMP2 register setup)
├── shared.rs (AtomicBool/U16/U32 — lock-free ISR↔main)
├── isr.rs (IsrState + .take() pattern)
├── interrupts.rs (TIM6, TIM14, ADC_COMP, DMA1, EXTI4_15)
├── main_loop.rs (MainState — protection, telemetry, PID)
├── system.rs (watchdog via HAL, IRQ, reset)
└── bin/main.rs (entry point, peripheral init, NVIC)

rm32_std (host harness, 29 blackbox vectors)
└── bin/rm32_harness (stdin/stdout protocol, same as C am32_harness)
```
