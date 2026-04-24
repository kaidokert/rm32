# RM32 Structural Improvements — Prioritized

## DONE

| # | Item | What was done |
|---|------|--------------|
| S1 | `map()` recursion | Replaced with linear interpolation + i64 intermediate |
| S2 | Magic constants | `constants.rs` with named values, applied to isr_logic + main_loop |
| S3 | ISR `expect()` | Emergency FET-off + infinite loop instead of panic |
| S4 | Register utils | Single `regs.rs` module, 8 duplicate copies removed |
| S8 | Raw pointer access | PAC sweep: all F051/L431 peripherals converted. COMP, EXTI, DMA, ADC, TIM, USART, GPIO all use PAC accessors. `periph_addr.rs` centralizes remaining base addresses. Only RCC clock enables remain raw. |

## REMAINING

### S5. Motor state machine enum
- **Current:** `armed`, `running`, `old_routine`, `stepper_sine` are separate bools — 16 possible combinations, most invalid
- **Fix:** `enum MotorMode { Disarmed, Armed, OldRoutine, Running, StepperSine }` (or similar)
- **Risk:** Medium — touches core state machine + SharedComm trait + shared atomics
- **Effort:** 2-3 hours
- **Impact:** Makes invalid states unrepresentable, clearer control flow

### S6. Safe EEPROM serialization + size assert
- **Current:** `unsafe { &*(self as *const Self as *const [u8; 192]) }` — works but fragile
- **Fix:** Add `const_assert_eq!(size_of::<EepromConfig>(), 192)`. Consider `zerocopy` or manual `to_bytes()`/`from_bytes()`
- **Risk:** Low — isolated to config.rs
- **Effort:** 1 hour

### S7. DMA buffer ownership
- **Current:** `static mut DMA_BUFFER: [u32; 64]` with `unsafe` accessor functions
- **Fix:** Use `cortex_m::singleton!()` or move into DshotCapture struct
- **Risk:** Medium — DMA hardware holds raw pointer, lifetime must be `'static`
- **Effort:** 1 hour per MCU (3 MCUs)

### S9. Newtype units (selective)
- **Current:** All values are raw u16/u32/i32
- **Fix:** `struct Micros(u32)`, `struct MilliAmps(i16)` for key interfaces
- **Risk:** Low but high churn
- **Effort:** Large — touch every function signature
- **Note:** Best done selectively when touching those APIs

### S10. Tick function decomposition
- **Status:** Already refactored once (moved to `isr_logic.rs` in core). Testable via `TestShared` + mock HAL.
- **Current size:** ~100 lines — manageable
- **Fix:** Extract `ArmingSequence`, `RampLimiter`, `BemfPoller` as separate functions
- **When:** If the function grows further

## NOT DOING

| # | Item | Reason |
|---|------|--------|
| S8-alt | Full HAL abstraction | Core library already uses HAL traits. Raw access is only in HAL implementation layer. |
| S9-alt | `f32` units | No FPU on Cortex-M0/M0+. Would be catastrophically slow. |

## Implementation Order

1. ~~S1~~ DONE
2. ~~S3~~ DONE
3. ~~S4~~ DONE
4. ~~S2~~ DONE
5. ~~S8~~ DONE (PAC sweep)
6. **S6** — Safe EEPROM serialization + size assert (1 hr)
7. **S5** — Motor state enum (2-3 hr)
8. **S7** — DMA buffer ownership (3 hr)
9. **S9** — Newtypes (selective, opportunistic)
