# PR #2 Review Feedback — Resolution Tracker

## Critical — Real bugs

| # | Source | File:Line | Issue | Status |
|---|--------|-----------|-------|--------|
| 1 | coderabbit, chatgpt-codex | main_state.rs:280 | **Division by zero**: `32 / poles` panics when `motor_poles > 32` | FIXED: rewritten as `kv * poles / 3200`. Extracted to `duty_ceiling()` with unit test. |
| 2 | coderabbit, sourcery | control/input.rs:66,100 | **Stale adjusted_input**: bidir writes `newinput`, sine reads `adjusted_input` | FIXED: bidir now writes `adjusted_input`. ISR reads `adjusted_input` not `newinput`. |
| 3 | chatgpt-codex, coderabbit | isr_logic.rs:108-109 | **duty_maximum never enforced**: synced but cycle never clamped | FIXED: added `duty.cycle = duty.cycle.min(duty.maximum)` before PWM output. |

## High — Incorrect behavior

| # | Source | File:Line | Issue | Status |
|---|--------|-----------|-------|--------|
| 4 | gemini, chatgpt-codex, coderabbit | main_state.rs:245 | **Consumed current counter wrong cadence** | NOTED: matches C firmware. Added TODO comment. |
| 5 | coderabbit | main_state.rs:299 | **Duty ceilings overwrite each other** | FIXED: `duty_ceiling()` returns `min(erpm_max, temp_max)`. |
| 6 | sourcery | shared_comm.rs:136 | **auto_advance written but never read by ISR** | FIXED: ISR sync reads `shared.auto_advance()` into `bemf.temp_advance`. |
| 7 | coderabbit | shared_comm.rs:133 | **Default published values are 0** | FIXED: defaults now 1999/2000/5/2/0 (sensible hardware values). |

## Medium — Harness/test quality

| # | Source | File:Line | Issue | Status |
|---|--------|-----------|-------|--------|
| 8 | sourcery, coderabbit | harness.rs:466 | **send_esc_info_flag hardcoded 0** | FIXED: reads `shared.send_esc_info_flag()`. |
| 9 | sourcery | harness.rs:554 | **zero_input_count config ignored** | FIXED: wired to `self.zero_input_count`. |
| 10 | coderabbit | harness.rs:568 | **voltage/current overrides not routed through mock ADC** | FIXED: config routes to `self.adc` fields. |
| 11 | sourcery | tests.rs:19 | **Variable PWM mode 2 no longer unit-tested** | FIXED: extracted `variable_pwm_mode2()`, `duty_ceiling()` as pure functions with 10 unit tests. |
| 12 | coderabbit | rm32_stm32 main.rs:131 | **Derived ARR not propagated to MainState** | FIXED: `main_state.timer1_max_arr = motor_cfg.timer1_max_arr`. |
