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

### S7. DMA buffer ownership — DONE
- Moved 6 of 9 `static mut` buffers into owning structs (input capture × 3 MCUs, telemetry × 3 MCUs)
- ADC buffers remain `static mut` (circular DMA, documented reason)
- ISR accesses via safe struct methods instead of `unsafe fn` free functions

### S11. Fallible init with Result
- **Current:** `init()` functions return `Self` and hang on hardware failure (busy-wait loops)
- **Fix:** Return `Result<Self, InitError>` with timeout on hardware flag waits
- **Risk:** Low — isolated to init paths, doesn't affect ISR performance
- **Effort:** 2 hours
- **Impact:** Firmware can enter safe mode instead of hanging on bad hardware

### S12. Convert phase.rs BSRR to PAC
- **Current:** `phase.rs` uses raw `(GPIOA_BASE + BSRR) as *mut u32` for pin toggling
- **Fix:** Use PAC `gpioa.bsrr().write(|w| ...)` — should be zero-cost, same codegen
- **Risk:** Low — verify no performance regression in ISR
- **Effort:** 30 minutes

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
6. ~~S6~~ DONE (size assert + from_bytes)
7. ~~S5~~ DONE (MotorMode enum, AtomicU8)
8. ~~S7~~ DONE (DMA buffers in structs, ADC stays static)
9. **S12** — Convert phase.rs BSRR to PAC (30 min)
10. **S11** — Fallible init with Result (2 hr)
11. **S9** — Newtypes (selective, opportunistic)
12. **S10** — Tick decomposition (if it grows)

## NOT ADDING (from latest review)

| Suggestion | Why not |
|-----------|---------|
| Generic ADC/UART drivers | 3 concrete types with MCU-specific register layouts. Generics add complexity for no benefit. |
| Const generic pins | Pin assignments are board compile-time constants. Over-engineering. |
| Bitfield crates | PAC already provides named bitfields. |
| bytemuck for EEPROM | All fields `u8`/`[u8;N]`, no padding. Size assert catches layout issues. |
| HAL trait bypass fix | Reviewer misread — core uses traits, raw access is in HAL *implementation*. |
