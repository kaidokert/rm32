# RM32 Structural Improvements — Prioritized

From review: "Rust skin over a C mindset." Assessment and action plan below.

## Priority: DO NOW — DONE

### S1. Replace recursive `map()` with linear equation — DONE
- **Current:** Binary search recursion — O(log n), stack usage, hard to reason about
- **Fix:** Standard `(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min` with i64 intermediate
- **Risk:** Low — existing tests verify behavior, blackbox tests catch regressions
- **Effort:** 10 minutes
- **Impact:** Simpler, faster, no stack growth

### S2. Replace magic constants with named constants — DONE
- **Current:** `47`, `2047`, `1999`, `20000`, `10000`, `150`, `500` scattered everywhere
- **Fix:** Define `const THROTTLE_MIN: u16 = 47`, `const DSHOT_MAX: u16 = 2047`, `const TIM1_DEFAULT_ARR: u16 = 1999`, etc.
- **Risk:** None — pure rename
- **Effort:** 1 hour
- **Impact:** Much more readable, fewer bugs from typos

### S3. Replace `expect()` in ISR with safe fallback — DONE
- **Current:** `isr::take_isr_state().expect("ISR state not initialized")` — panics in ISR = hard fault
- **Fix:** Return early or enter safe mode (`all_off()`) if state is missing
- **Risk:** Low
- **Effort:** 15 minutes
- **Impact:** Prevents undiagnosable hard faults on misconfiguration

### S4. Consolidate register utility functions — DONE
- **Current:** `read_reg`, `write_reg`, `modify_reg` duplicated in 8+ files
- **Fix:** Single `reg.rs` module in rm32_stm32, imported everywhere
- **Risk:** None — pure dedup
- **Effort:** 30 minutes
- **Impact:** Less code, one place to audit for correctness

## Priority: DO SOON

### S5. Motor state machine enum
- **Current:** `armed`, `running`, `old_routine`, `stepper_sine` are separate bools — 16 possible combinations, most invalid
- **Fix:** `enum MotorMode { Disarmed, Armed, OldRoutine, Running, StepperSine }` (or similar)
- **Risk:** Medium — touches core state machine, needs careful testing
- **Effort:** 2-3 hours
- **Effort justification:** Many call sites check combinations of these flags
- **Impact:** Makes invalid states unrepresentable, clearer control flow

### S6. Safe EEPROM serialization
- **Current:** `unsafe { &*(self as *const Self as *const [u8; 192]) }` — works but fragile
- **Fix:** Manual `to_bytes()` / `from_bytes()` or use `zerocopy` crate (no_std compatible)
- **Risk:** Low — isolated to config.rs
- **Effort:** 1 hour
- **Impact:** No more `repr(C)` requirement, safer byte conversion

### S7. DMA buffer ownership
- **Current:** `static mut DMA_BUFFER: [u32; 64]` with `unsafe` accessor functions
- **Fix:** Move buffer into the DshotCapture struct, or use `cortex_m::singleton!()` for safe static init
- **Risk:** Medium — DMA peripheral holds raw pointer to buffer, lifetime must be `'static`
- **Effort:** 1 hour per MCU (3 MCUs)
- **Impact:** Fewer `static mut`, clearer ownership

## Priority: DO LATER

### S8. Reduce raw pointer peripheral access
- **Current:** 158 raw pointer casts (`as *mut u32`) for register access in rm32_stm32
- **Fix:** Use PAC register accessors where available. For F051/L431 where PAC names diverge, keep raw but centralize base addresses.
- **Risk:** Medium — PAC API differences between MCUs are why raw was used
- **Note:** G071 already uses PAC/HAL for some peripherals (PWM, comparator). F051/L431 use raw because their PAC/HAL crates have API gaps or UB issues (we had to fix `write_volatile` errors in the L4 HAL)
- **Effort:** Large (half day per MCU)
- **Impact:** Better, but diminishing returns — raw register access is correct and tested

### S9. Newtype units (Micros, Amps, Rpm)
- **Current:** All values are raw u16/u32/i32
- **Fix:** Newtype wrappers like `struct Micros(u32)`, `struct MilliAmps(i16)`
- **Risk:** Low but high churn — touches every function signature
- **Effort:** Large (full day)
- **Impact:** Self-documenting, prevents unit confusion. But adds verbosity.
- **Note:** Consider selectively — Micros for timing values, skip for simple counters

### S10. Refactor tick functions into smaller components
- **Current:** `ten_khz_tick` in isr_logic.rs is ~90 lines handling arming, throttle, BEMF, ramp, PWM
- **Status:** Already refactored once (moved from isr_handlers.rs to core library). Now testable.
- **Fix:** Extract `ArmingSequence`, `RampLimiter`, `BemfPoller` as separate structs/functions
- **Risk:** Medium — tight coupling between these concerns (they share state)
- **Effort:** 2-3 hours
- **Impact:** Cleaner but may add indirection. Current size is manageable.

## Priority: NOT DOING

### S8-alt. Full HAL abstraction for all peripherals
- **Why not:** We already have HAL traits in rm32 core (PwmOutput, Comparator, PhaseOutput, etc.). The raw register access is only in rm32_stm32 (the HAL implementation). The core library is already fully abstract. The review's claim that HAL traits are "bypassed" is incorrect — they are the interface between core and HAL.

### S9-alt. `f32`-based units
- **Why not:** No FPU on Cortex-M0/M0+. All math must be integer. The review suggests `Amps(f32)` which would be catastrophically slow.

---

## Implementation Order

1. ~~**S1** — Replace `map()` (10 min)~~ DONE
2. ~~**S3** — ISR panic → safe fallback (15 min)~~ DONE
3. ~~**S4** — Consolidate register utils (30 min)~~ DONE
4. ~~**S2** — Named constants (1 hr)~~ DONE
5. **S6** — Safe EEPROM serialization + size assert (1 hr)
6. **S5** — Motor state enum (2-3 hr, when ready for state machine cleanup)
7. **S7** — DMA buffer ownership (3 hr, when touching input capture)
8. **S9** — Newtypes for key units (selective, when touching those APIs)
