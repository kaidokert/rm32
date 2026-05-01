# PR #4 Review — Full Comments

## Critical & High Priority

### 1. [HIGH] sourcery-ai — `rm32_stm32/src/bin/main.rs:244`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

**issue (bug_risk):** Hard-coding `isr_input_is_dshot = true` can break non-DShot input paths and diverges from ISR-detected input type.

Since `process_input` already depends on `is_dshot` to choose between bidir DShot vs servo/unidirectional paths, this should use the real input type, not a constant. On STM32, you could mirror what the harness does by exposing the ISR/transfer-detected type (e.g., via a `SharedComm` flag or small enum) into `main` and passing that through to `process_input` to keep behavior consistent across input modes.

</details>

---

### 2. [HIGH] gemini-code-assist — `rm32/src/main_state.rs:168`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

![high](https://www.gstatic.com/codereviewagent/high-priority.svg)

The check `if self.protection.bemf_timeout_happened != BEMF_FAULT_LATCHED` combined with the use of `adj_input` for clearing logic creates a permanent lockout. When a stuck rotor is detected, `process_input` forces `adjusted_input` to 0, which would normally clear the fault in the next tick. However, this block skips clearing if the fault is already latched. This means the fault can never be cleared by the user centering the throttle. Clearing logic should use `shared.newinput()` and be accessible even when latched.

```rust
        // BEMF timeout clearing — use raw newinput to allow clearing latched faults
        let newinput = shared.newinput();
        if zc > 1000 || newinput == 0 {
            self.protection.bemf_timeout_happened = 0;
        }
        if zc > 100 && newinput < 200 {
            self.protection.bemf_timeout_happened = 0;
        }
        if self.config.use_sine_start != 0
            && newinput < crate::constants::SINE_BEMF_CLEAR_THROTTLE
        {
            self.protection.bemf_timeout_happened = 0;
        }
```

</details>

---

### 3. [HIGH] chatgpt-codex-connector — `rm32_stm32/src/bin/main.rs:244`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

**<sub><sub>![P1 Badge](https://img.shields.io/badge/P1-orange?style=flat)</sub></sub>  Detect actual input protocol before preprocessing**

`process_input` is always invoked with `isr_input_is_dshot = true`, so PWM/servo input is misclassified as DShot. In bidirectional setups this sends servo values through DShot split-point/reverse logic, which can block or mis-handle reverse/brake transitions and produce incorrect throttle behavior whenever `input_type` is servo.

Useful? React with 👍 / 👎.

</details>

---

### 4. [HIGH] chatgpt-codex-connector — `rm32/src/control/input.rs:116`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

**<sub><sub>![P1 Badge](https://img.shields.io/badge/P1-orange?style=flat)</sub></sub>  Publish sine-start mapped throttle to shared input**

When `use_sine_start` is enabled, the mapped throttle is written only to `input_state.input`, but the ISR setpoint path reads `shared.adjusted_input()`. Because the mapped value is never published back to shared state, sine-start shaping is effectively bypassed and motor start/ramp behavior no longer follows the configured sine-start curve.

Useful? React with 👍 / 👎.

</details>

---

### 5. [HIGH] chatgpt-codex-connector — `rm32/src/control/input.rs:94`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

**<sub><sub>![P1 Badge](https://img.shields.io/badge/P1-orange?style=flat)</sub></sub>  Restore servo bidirectional reverse/brake mapping**

The servo bidirectional branch now does a raw passthrough (`adjusted_input = newinput`) and drops the previous direction/deadband/speed-gated reversal handling from `set_input_servo_bidir`. `transfer.rs` only converts pulse widths to a scalar throttle and does not perform those mode transitions, so bidirectional servo mode can no longer reliably trigger direction flips and associated braking behavior.

Useful? React with 👍 / 👎.

</details>

---

## Major & Medium Priority

### 6. [MAJOR] coderabbitai — `rm32_stm32/src/bin/main.rs:245`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**Don't force the DShot path in firmware.**

Line 244 hardcodes `isr_input_is_dshot = true`, so `process_input()` will always take the DShot/bidirectional path even when the active input source is servo PWM. That makes non-DShot boards follow the wrong mapping/brake logic.



Also applies to: 284-290

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@rm32_stm32/src/bin/main.rs` around lines 242 - 245, The code currently forces
DShot handling by setting isr_input_is_dshot = true which causes process_input()
to always take the bidirectional/DShot path; change this so the input type is
determined dynamically (or via configuration) instead of hardcoded: remove the
boolean literal assignment and instead read the actual input type flag set by
the ISR transfer handler (the same place referenced by your TODO), expose or
query that flag from the loop before calling
rm32::control::input::process_input(), and ensure both places where
isr_input_is_dshot is used (around the initial assignment and the block at lines
~284-290) consult that runtime flag so servo PWM boards follow the correct
mapping/brake logic rather than DShot.
```

</details>

<!-- fingerprinting:phantom:medusa:grasshopper:0501a98b-6e9b-4280-9cc0-17056a632546 -->

<!-- 4e71b3a2 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 7. [MAJOR] coderabbitai — `rm32/src/bin/harness.rs:414`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**Harness still skips the firmware `stepper_sine` path.**

`Harness::do_tick()` goes straight into `process_input()`/`main.tick()`/`ten_khz_tick()`, but the firmware main loop runs a dedicated `sine_step()` block first when `shared.stepper_sine()` is active. Any blackbox vector that enters sine start/changeover is still exercising different logic in harness and firmware.

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@rm32/src/bin/harness.rs` around lines 363 - 414, Harness::do_tick currently
bypasses the firmware's sine-step path; detect when the sine path is active by
checking shared.stepper_sine() near the top of do_tick (before calling
input::process_input and main.tick) and invoke the same sine handling the
firmware uses (the sine_step / sine_stepper block) so the harness executes
identical logic on sine start/changeover; ensure you call the same function(s)
or inline the same steps the firmware does, preserve any flag clears or state
transitions the firmware performs (e.g., clearing shared.stepper_sine or
updating commutation/duty as the firmware does) and keep this behavior before
main.tick and isr_logic::ten_khz_tick so tests exercise the exact firmware path.
```

</details>

<!-- fingerprinting:phantom:medusa:grasshopper:0501a98b-6e9b-4280-9cc0-17056a632546 -->

<!-- 4e71b3a2 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 8. [MAJOR] coderabbitai — `rm32/src/bin/harness.rs:645`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**`eeprom.*` overrides are not applied the same way as firmware settings.**

These keys only mutate `self.config`, and `load_eeprom` is a no-op. In firmware, the loaded config is passed through `derive_motor_config()` and the derived duty thresholds, PWM ARR, PID gains, voltage cutoff, and servo thresholds are pushed into `MainState`/ISR state before control starts. Vectors that tweak EEPROM values here will miss those runtime effects.



Also applies to: 677-680

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@rm32/src/bin/harness.rs` around lines 612 - 645, The eeprom.* match arms
currently only mutate self.config but don't apply derived runtime values; after
updating any EEPROM-related key (the match block handling "eeprom.*" and the
similar block around lines 677-680), call derive_motor_config(&self.config) and
copy the resulting derived fields (duty thresholds, PWM ARR/period, PID gains,
voltage cutoff, servo thresholds, motor_kv-derived field, etc.) into self.main
and the ISR/control state so the harness mirrors firmware behavior; also replace
the no-op load_eeprom usage by invoking the derive+apply sequence whenever
EEPROM vectors or keys are changed so runtime behavior matches firmware.
```

</details>

<!-- fingerprinting:phantom:medusa:grasshopper:0501a98b-6e9b-4280-9cc0-17056a632546 -->

<!-- 4e71b3a2 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 9. [MAJOR] coderabbitai — `rm32/src/control/input.rs:74`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**`brake_on_stop` relatches the RC-car reverse handshake.**

Line 71-74 clears `prop_brake_active` and sets `return_to_center` so the next stick movement can flip direction. The unconditional brake-on-stop block then sets `prop_brake_active = true` again while input is still zero, so the next reverse command toggles direction with brake still latched and subsequent throttle stays pinned at 0. Please gate this block out for RC-car reverse mode, or explicitly clear the brake latch when `r.reverse` fires.

<details>
<summary>Possible fix</summary>

```diff
-    if shared.armed()
+    if shared.armed()
         && !shared.stepper_sine()
         && input_state.input < crate::constants::THROTTLE_MIN_SIGNAL
         && config.brake_on_stop == 1
         && config.comp_pwm != 0
+        && config.rc_car_reverse == 0
     {
         input_state.prop_brake_active = true;
     }
```
</details>


Also applies to: 121-129

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@rm32/src/control/input.rs` around lines 70 - 74, The unconditional
"brake-on-stop" path is relatching the RC-car reverse handshake by setting
input_state.prop_brake_active = true while input is still zero, undoing the
previous clear and return_to_center set in the zero-input branch; update the
logic in the brake_on_stop handler (the block that sets prop_brake_active = true
when newinput is zero) to skip or gate that behavior when RC-car reverse mode is
active (check r.reverse or the RC reverse mode flag), or alternatively
explicitly clear input_state.prop_brake_active when r.reverse fires; apply the
same change for the second occurrence of this pattern later in the file (the
similar block around the later throttle/brake handling).
```

</details>

<!-- fingerprinting:phantom:medusa:ocelot:72d8f055-d671-42f5-8824-a753d618fe76 -->

<!-- 4e71b3a2 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 10. [MAJOR] coderabbitai — `rm32/src/main_state.rs:27`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**Clamp `variable_pwm_mode1()` before the `u16` cast.**

This helper is fed from `shared.commutation_interval()` in Line 332-340. With the current startup/default interval values, the linear `map()` call extrapolates far past `timer1_max_arr`, and the final `as u16` can wrap to a garbage ARR. That can publish an invalid PWM period before the first stable commutations.

<details>
<summary>Possible fix</summary>

```diff
 pub fn variable_pwm_mode1(commutation_interval: u32, timer1_max_arr: u16) -> u16 {
-    crate::functions::map(
-        commutation_interval as i32,
+    crate::functions::map(
+        (commutation_interval as i32).clamp(96, 200),
         96,
         200,
         timer1_max_arr as i32 / 2,
         timer1_max_arr as i32,
     ) as u16
 }
```
</details>


Also applies to: 330-340

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@rm32/src/main_state.rs` around lines 20 - 27, The mapped PWM value from
crate::functions::map in variable_pwm_mode1 can overflow when cast to u16, so
clamp the mapped result to the allowed range before casting: compute the i32
result, clamp it between timer1_max_arr as i32 / 2 and timer1_max_arr as i32 (or
use min/max), then cast the clamped value to u16; apply the same pattern
wherever shared.commutation_interval() feed is converted to ARR (the same
mapping logic around variable_pwm_mode1 usage) to prevent wrapping garbage PWM
periods.
```

</details>

<!-- fingerprinting:phantom:medusa:ocelot:72d8f055-d671-42f5-8824-a753d618fe76 -->

<!-- 4e71b3a2 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 11. [MAJOR] coderabbitai — `rm32/src/main_state.rs:77`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**Clamp the eRPM/temperature maps in `duty_ceiling()`.**

Both protection branches assume `map()` clamps, but it extrapolates. Once `k_erpm` goes above `high_rpm` or temperature goes above `temperature_limit + 10`, the result can leave the intended range and the `as u16` cast can wrap negative values. That makes the duty ceiling relax again at the hottest / fastest end instead of tightening.

<details>
<summary>Possible fix</summary>

```diff
     let erpm_max = if k_erpm > 0 && high_rpm > low_rpm {
-        crate::functions::map(k_erpm, low_rpm, high_rpm, 600, 2000) as u16
+        crate::functions::map(k_erpm.clamp(low_rpm, high_rpm), low_rpm, high_rpm, 600, 2000)
+            as u16
     } else {
         2000
     };

-    let temp_max = if degrees_celsius > temperature_limit as i16 {
+    let temp_max = if temperature_limit != 0 && degrees_celsius > temperature_limit as i16 {
         crate::functions::map(
-            degrees_celsius as i32,
+            (degrees_celsius as i32)
+                .clamp(temperature_limit as i32 - 10, temperature_limit as i32 + 10),
             temperature_limit as i32 - 10,
             temperature_limit as i32 + 10,
             1000,
             1,
         ) as u16
```
</details>


Also applies to: 343-350

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@rm32/src/main_state.rs` around lines 59 - 77, The duty_ceiling() logic
assumes crate::functions::map clamps but it extrapolates and can produce values
outside the intended [1,2000] range (causing negative-to-u16 wrap); fix by
clamping the mapped results before casting and before taking
erpm_max.min(temp_max): for erpm_max (symbol k_erpm, low_rpm, high_rpm,
crate::functions::map) and for temp_max (symbols degrees_celsius,
temperature_limit, crate::functions::map) apply a clamp to the mapped i32 result
to the intended bounds (1..=2000 or 600..=2000 as appropriate) then cast to u16
so the values cannot underflow/overflow and erpm_max.min(temp_max) behaves
correctly.
```

</details>

<!-- fingerprinting:phantom:medusa:ocelot:72d8f055-d671-42f5-8824-a753d618fe76 -->

<!-- 4e71b3a2 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 12. [MAJOR] coderabbitai — `tests/blackbox/vectors/stuck_rotor.txt:10`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

_⚠️ Potential issue_ | _🟠 Major_

**This vector locks in a harness-only timing skew.**

The expected latch point is shifted to match a documented “2 ticks later than C” behavior, so the test now passes when the harness still diverges from firmware. That undercuts this PR’s goal; I’d either fix the harness/public state timing and assert the firmware-equivalent stop point, or mark this as an explicit parity gap instead of codifying the skew.

Based on learnings, known Rust/C semantic mismatches like `tests/blackbox/vectors/desync_recovery.txt` are intentionally tracked as explicit parity gaps instead of being normalized into passing vectors.


Also applies to: 24-47

<details>
<summary>🤖 Prompt for AI Agents</summary>

```
Verify each finding against the current code and only fix it if needed.

In `@tests/blackbox/vectors/stuck_rotor.txt` around lines 8 - 10, The test vector
was shifted to accommodate a harness-only 2-tick timing skew; either restore the
firmware-equivalent expected latch point and fix the harness/public-state timing
so that interval_timer is set each tick (and zcfoundroutine behavior matches
firmware ordering), or explicitly mark this vector as a parity gap (similar to
tests/blackbox/vectors/desync_recovery.txt) instead of normalizing the skew into
a passing test; update the vector to reflect one of these two actions and
document which choice.
```

</details>

<!-- fingerprinting:phantom:medusa:grasshopper:0501a98b-6e9b-4280-9cc0-17056a632546 -->

<!-- d98c2f50 -->

<!-- This is an auto-generated comment by CodeRabbit -->

</details>

---

### 13. [MEDIUM] gemini-code-assist — `rm32/src/control/input.rs:71`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

![medium](https://www.gstatic.com/codereviewagent/medium-priority.svg)

Avoid using magic numbers for throttle thresholds. Use the `THROTTLE_MIN_SIGNAL` constant instead of `47`.

```suggestion
                if newinput <= crate::constants::THROTTLE_MIN_SIGNAL && input_state.prop_brake_active {
```

</details>

---

### 14. [MEDIUM] gemini-code-assist — `rm32/src/control/input.rs:125`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

![medium](https://www.gstatic.com/codereviewagent/medium-priority.svg)

The magic number `1` for `brake_on_stop` should be replaced with a named constant or an enum variant to clarify the intent (e.g., `BRAKE_MODE_DRAG`). Note that `isr_logic.rs` checks for mode `2`.

</details>

---

### 15. [MEDIUM] gemini-code-assist — `rm32/src/main_state.rs:373`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

![medium](https://www.gstatic.com/codereviewagent/medium-priority.svg)

Magic numbers `100`, `13`, and `23` should be defined as constants. `2000` should be replaced with `DUTY_SCALE_MAX`.

```suggestion
            let level =
                crate::functions::map(shared.duty_cycle_setpoint() as i32, 100, crate::constants::DUTY_SCALE_MAX as i32, 13, 23) as u8;
```

</details>

---

### 16. [MEDIUM] gemini-code-assist — `rm32/src/main_state.rs:58`

**Resolution:** TODO

<details>
<summary>Full comment</summary>

![medium](https://www.gstatic.com/codereviewagent/medium-priority.svg)

The constants `3200` and `384` used for RPM range calculation are opaque. They should be defined as named constants with documentation explaining their derivation (e.g., based on the 32/poles factor).

</details>

---
