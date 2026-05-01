# RM32 Project Critique & Architectural Review

This document provides a comprehensive technical review of the RM32 ESC firmware, focusing on canonical Rust patterns, architectural integrity, and ecosystem integration.

## KEY FOCUS AREAS FOR NEXT REFACTOR

1.  **State Pivot (Enum vs Boolean):** Collapse the "Boolean Explosion" (64+ invalid flag combinations) into a strict `InputMode` enum to make invalid states unrepresentable.
2.  **Interface Integrity (Atomic vs Local):** Enforce `SharedComm` atomics as the only source of truth, eliminating the "Refactoring Gap" caused by leaky local state and shadow variables.
3.  **Total Unification (Library vs Bootstrap):** Strip all business logic from `main.rs` and `harness.rs` into a shared `System::tick` library function to ensure the harness tests the physical "territory," not just the "map."

## 0. High-Level Verdict: "Canonical Rust" Score

**Overall: Solid.** The project is highly "Rusty" for an embedded/no_std codebase. It demonstrates clear intent and strong domain modeling.

### Strengths
*   **Module Boundaries:** Clean separation between core logic (`rm32`) and hardware implementation (`rm32_stm32`).
*   **Domain Types:** Excellent use of Newtypes (e.g., `MilliVolts`, `TimerTicks`) instead of raw primitives.
*   **Constants Layer:** Centralized `constants.rs` with semantic naming and documentation.
*   **Hardware Abstraction:** Mature use of traits and static dispatch to ensure zero-cost abstraction over MCU-specific peripherals.

---

## 1. Canonical Rust & Crate Ecosystem Integration

While the foundation is solid, there are opportunities to improve ergonomics and safety by leaning further into the ecosystem.

*   **Time and Frequency (`fugit`):** The use of raw `u32` for ticks is prone to scale errors.
    *   **Recommendation:** Use [fugit](https://crates.io/crates/fugit) for `Duration` and `Rate` types to handle clock-math at compile time.
*   **Logging and Debugging (`defmt`):** Essential for debugging 20kHz loops without `core::fmt` overhead.
    *   **Recommendation:** Integrate [defmt](https://crates.io/crates/defmt) as an optional feature.
*   **Volatile Access:** Replace manual pointer manipulation with [vcell](https://crates.io/crates/vcell) or [volatile-register](https://crates.io/crates/volatile-register).
*   **Configuration Modeling (`num_enum`):**
    *   **Recommendation:** Use [num_enum](https://crates.io/crates/num_enum) for safe `TryFrom<u8>` conversions of EEPROM flags and `InputType`.
*   **Error Handling:** Use [thiserror](https://crates.io/crates/thiserror) on the host/harness side to improve ergonomics for structured command parsing.

---

## 2. Style & SOLID Violations

*   **Visibility & Encapsulation:** Many fields in `control/state.rs` (e.g., `BemfState`, `DutyState`) are `pub`.
    *   **Critique:** This leaks safety invariants. Internal counters (like `bad_count`) should be `pub(crate)` to prevent accidental external modification.
*   **Typing for PWM Duty:** The project uses raw `u16` for duty cycle values across the codebase.
    *   **Critique:** A `u16` value of `100` has different physical meanings at 24kHz vs 48kHz (as it is relative to `timer1_max_arr`). This is an "implicit contract" that is prone to scaling errors.
    *   **Better Pattern:** Use a `Fraction` or `Percent` type (or a Newtype `Duty(u16)`) that is explicitly bound to or calculated against the current `max_arr`. This prevents passing a frequency-incompatible duty value to the PWM driver.
*   **Interface Segregation (SOLID):** The `MotorHal` and `ControlHal` traits are textbook examples of good ISP.
*   **God Object (SRP Violation):** `MotorState` aggregates too many concerns (input, protection, telemetry, PID, timing).
    *   **Recommendation:** Consider decomposing into focused managers (e.g., `ProtectionManager`).

---

## 3. DRY, KISS, and Architectural Debt

### The "Dual Control-Path" Problem
The project currently exposes both `control::tick` and `control::isr_logic` modules simultaneously.
*   **Critique:** This is significant technical debt. `isr_logic.rs` is modern and constant-driven, but `tick.rs` contains magic numbers (`47`, `1047`, `2047`, etc.) and is used by the test harness.
*   **Risk:** Tests are verifying legacy code in `tick.rs` while the hardware runs `isr_logic.rs`.
*   **Action:** Deprecate `tick.rs` and update the harness to use `isr_logic.rs`.

### KISS in Signal Processing
The `set_input` family has deep branching that is hard to test exhaustively.
*   **Recommendation:** Consider a table-driven or strategy-based decomposition for different modes (unidirectional, bidirectional, RC-car).

---

## 4. Magic Numbers & Constants

*   **Regression in `tick.rs`:** As noted, this file bypasses the constants layer.
*   **MCU Identifiers:** Bootloader device codes and ADC calibration addresses in `main.rs` are magic numbers.
    *   **Recommendation:** Move these into the `ChipConfig` trait in `mcu.rs`.

---

## 5. Runtime & Toolchain Policy

*   **Edition Mismatch:** `rm32` is Edition 2024, `rm32_stm32` is 2021. This can surprise contributors.
*   **MSRV Friction:** `fixed = "1.31.0"` requires Rust 1.93, but some environments are on 1.92, causing test failures.
    *   **Action:** Add an explicit MSRV policy and a `rust-toolchain.toml`.

---

## Quick "Next 5" Practical Actions

1.  **Consolidate Control Path:** Refactor `harness.rs` to use `isr_logic.rs` and delete the legacy `tick.rs`.
2.  **Refactor `tick.rs` Literals:** Move remaining literals to `constants.rs` if `tick.rs` cannot be immediately retired.
3.  **Encapsulate State:** Change `pub` fields to `pub(crate)` in `control/state.rs`.
4.  **Add `rust-toolchain.toml`:** Align editions and define a clear MSRV.
5.  **Safe Enum Parsing:** Introduce `num_enum` for EEPROM-backed configuration enums.

---
From https://github.com/kaidokert/rm32/pull/4

- Several harness config keys (e.g. EDT_ARM_ENABLE, dshot_telemetry, process_adc) are now explicitly ignored or no-ops; if the C harness or older Rust harness relied on these affecting behavior, it would be good to either wire them through or document that they are intentionally unsupported to avoid silent divergence between implementations.

- The new harness constructs MainState manually with many fields and defaults that are effectively duplicated from the firmware path; consider factoring this into a shared constructor/helper (or Default + a small override) so that future changes to MainState initialization cannot drift between the real firmware and the host-side harness.

---
From https://github.com/kaidokert/rm32/pull/5
- Several of the input mapping functions (e.g. servo_bidir, servo_rc_car, dshot_bidir) take a long list of scalar parameters; consider introducing small config structs for these call sites to make it harder to pass arguments in the wrong order and to keep the mapping API easier to extend.
- The new load_eeprom path in the harness re-implements only part of what the firmware does (e.g. it sets some fields from derive_motor_config but leaves things like current limiting as TODOs); it may be worth factoring this into a shared helper so harness and firmware stay aligned when EEPROM behavior changes.
- In MainState::tick the consumed-current integration and stall detection rely on hardcoded thresholds (e.g. > 20000 ticks and the BEMF stall threshold) tied to an assumed tick rate; consider expressing these in terms of the actual tick frequency or shared constants so behavior remains correct if the ISR rate changes or is reused in other contexts.
----

from https://github.com/kaidokert/rm32/pull/6

Hey - I've left some high level feedback:

    - The new Harness struct in harness.rs reimplements a lot of wiring that already exists in the firmware (e.g. MainState construction, individual fields of SharedState), which makes it easy for behavior to diverge; consider factoring common initialization into a helper or builder in the core crate so the harness and firmware stay in lockstep when fields are added or semantics change.
    - The input_mapping and control::input::process_input modules now encode much of the old set_input state machine in fairly large functions; it would be easier to reason about and maintain if some of the RC-car/bidir branches were split into smaller helpers or enums representing mode (unidirectional / bidir / rc-car), and if the implicit invariants (e.g. when return_to_center is allowed to flip direction) were documented more explicitly.
    - With SharedComm gaining many new methods and default no-op implementations, it’s easy to accidentally rely on a value that is never set for a particular implementation; you might want to add a lightweight test or a compile-time helper that ensures SharedState and TestShared override all non-optional accessors that isr_logic or main_state depend on (e.g. tim1_arr, duty_maximum, current_limit_adjust).

---

From https://github.com/kaidokert/rm32/pull/7

    - MainState is now being manually constructed in multiple places (e.g. rm32_stm32 main and the host harness), which duplicates a lot of field initialization logic; consider adding a constructor/helper that takes the board parameters + EepromConfig and returns a fully-initialized MainState to reduce the risk of these diverging over time.
    - The new bidirectional input handling is split between input::process_input and input_mapping::* with a fair amount of duplicated mapping logic for DShot vs servo and RC-car vs non-RC-car; it may be worth factoring out some shared helpers or documenting a single definitive truth-table for the direction/brake state machine to make future changes safer.


---

## 6. Input Pipeline State Machine Refactor (Post-Merge Priority)

### Problem

The input processing pipeline (`control::input::process_input` + `input_mapping`) uses 6+ independent boolean/u8 flags to determine behavior:
- `is_dshot`, `config.bi_direction`, `config.rc_car_reverse`, `config.use_sine_start`
- `input_state.prop_brake_active`, `input_state.return_to_center`

This creates 64 possible flag combinations, most of which are invalid. The code uses nested `if/else` chains that implicitly encode which combinations are legal. Every bug fix in one mode introduces regressions in adjacent modes because the developer must mentally reconstruct the full state machine from scattered boolean checks.

**Evidence:** Across 6 review rounds of the harness refactor PR, at least 4 bugs were introduced by fixes that were correct for one mode but broke another:
- Bidir DShot `reverse` flag clobbered by `replace_all`
- Servo bidir fault clearing range included DShot active throttle values
- `prop_brake_active` never cleared in non-RC-car modes
- Sine start mapping applied to DShot command values

### Fix (DONE)

Replaced the boolean flags with `InputMode` + `ReverseMode` enums:

```rust
enum ReverseMode {
    SpeedGated,  // normal bidir: speed-gated direction flip
    RcCar,       // RC-car: brake-and-reverse with return-to-center handshake
}

enum InputMode {
    Unidirectional,
    BidirDshot(ReverseMode),
    BidirServo { mode: ReverseMode, dead_band: u16 },
}
```

`process_input` is now a `match` on `InputMode` — each arm handles exactly one
mode with no cross-contamination. `InputMode::from_config()` computes the mode
from EEPROM config + detected protocol. `SystemTick::tick_input()` recomputes
it each tick (cheap, keeps mode in sync with runtime config changes).

RC-car brake/reverse handshake extracted into `apply_rc_car_result()` shared
between DShot and servo RC-car arms.

### Test Matrix (DONE)

6 C-first golden vectors covering fault/recovery across all modes:
- `uni_brake_on_stop`, `bidir_dshot_fault`, `bidir_dshot_recovery`
- `bidir_dshot_rccar_fault`, `servo_bidir_fault`, `servo_rccar_fault`

38 total blackbox vectors, 43 pass, 1 xfail (desync_recovery).

---

## 7. The Refactoring Gap (Post-Mortem Findings)

Despite careful derivation and extensive blackbox testing, several critical bugs survived into PR iterations. This section analyzes the root causes of these "Refactoring Gaps" to inform future development.

### 1. The Proxy Problem (Harness vs. Firmware)
The project currently suffers from a "Testing the Map, not the Territory" failure. Automated tests run against `harness.rs`, while the hardware runs `main.rs`.
*   **The Gap:** Logic implemented or fixed in the harness (like "Derive & Apply" for current limits) was never mirrored in the firmware's `main.rs`.
*   **Recommendation:** Move all "Glue Logic" (initialization, state synchronization, hardware configuration derivation) into a shared library function (e.g., `rm32::MainState::init_system`). If the harness and firmware share 100% of their setup code, these gaps disappear.

### 2. Intent vs. Execution (The Flag Trap)
Tests often asserted that a specific flag (like `prop_brake_active`) was set correctly but failed to verify that the motor actually performed the action.
*   **The Gap:** The ISR was sometimes missing the logic to *read* the flag, or the mapping from flag to PWM duty was incorrect. The "Intent" was tested, but the "Execution" was not.
*   **Recommendation:** Blackbox tests must assert **Physical Effects** (e.g., `pwm_duty`, `step_advance`) in addition to internal status flags.

### 3. The "Inertia" of Old Code (Logic Leakage)
Small, critical snippets of logic (like Startup Duty Clamping) were buried inside massive functions in the legacy C code.
*   **The Gap:** During extraction into clean modules (`input.rs`, `isr_logic.rs`), these snippets fell through the cracks because they didn't clearly belong to a single "new" module. Neither module claimed ownership, so the logic vanished.
*   **Recommendation:** Audit new implementations against the legacy `tick.rs` specifically for "dangling" conditionals that weren't ported.

### 4. Visibility of State (Shadow State Mismatch)
The harness sometimes maintained local state (for its `print_state` output) that diverged from the atomic state used by the ISR.
*   **The Gap:** A test would pass because the *local* `temp_advance` looked correct, but the ISR was using an *atomic* value that was still zero.
*   **Recommendation:** Enforce `SharedComm` (the atomics) as the **Only Source of Truth**. Eliminate local shadow variables in the harness that duplicate state already present in `SharedComm` or `MotorContext`.

### 5. Summary Strategy for Resolution
To reach "Justifiable Completeness," the project must move from **Modular Refactoring** to **System Unification**:
- **Unify Entry Points:** Create a single `tick_system` call used by both harness and firmware.
- **Assert Impure Outputs:** Test the PWM and Phase behavior, not just the state machine transitions.
- **Eliminate Main-Loop Divergence:** Ensure `harness.rs` and `main.rs` have zero unique business logic.
