# Visibility Audit: `control/state.rs` and `MainState`

## Summary

Changing all `pub` fields in `control/state.rs` to `pub(crate)` produces
**16 cross-crate errors** and reveals **14 completely dead fields**. But the
deeper problem is that `MainState` is a 35-field god object that duplicates
fields from the decomposed structs in `state.rs`. The fix is not "delete
the small structs" — it's "adopt them properly and stop scattering their
fields across MainState."

## Dead Fields (14) — Delete

These exist in `state.rs` structs but are never read or written anywhere:

| Struct | Field | Type | Verdict |
|--------|-------|------|---------|
| `BemfState` | `auto_advance_level` | `u8` | Superseded by `SharedComm::auto_advance` |
| `DutyState` | `setpoint` | `u16` | Superseded by `SharedComm::duty_cycle_setpoint` |
| `TelemetryState` | `send_esc_info` | `bool` | Superseded by `SharedComm::send_esc_info_flag` |
| `TelemetryState` | `ms_count` | `u16` | Telemetry timer never used in Rust path |
| `ProtectionState` | `low_voltage_cutoff` | `bool` | Never set — LVC uses `low_voltage_count` instead |
| `ProtectionState` | `desync_happened` | `u32` | Never incremented — desync uses `commutation.desync_check` |
| `Measurements` | `consumed_current` | `i32` | mAh integration never ported — **TODO or remove** |
| `InputState` | `edt_arm_enable` | `bool` | Handled via ISR state directly |
| `TimingState` | `polling_mode_changeover` | `u32` | Hardcoded as constant in `isr_logic.rs` |
| `PidState` | `speed` | `Pid` | Duplicated as `MainState::speed_pid` |
| `PidState` | `stall` | `Pid` | Duplicated as `MainState::stall_pid` |
| `PidState` | `stall_adjust` | `i32` | Duplicated as `MainState::stall_protection_adjust` |
| `PidState` | `use_speed_control` | `bool` | Duplicated as `MainState::use_speed_control_loop` |
| `PidState` | `input_override` | `i32` | Duplicated as `MainState::speed_input_override` |

## The God Object Problem: `MainState`

`MainState` has 35 fields. Many are loose scalars that belong in the
decomposed structs already defined in `state.rs`:

### PID fields — belong in `PidState`

`MainState` has 7 PID-related fields that should be in `PidState`:

| MainState field | Should be in |
|----------------|-------------|
| `current_pid: Pid` | `PidState::current` (already exists) |
| `speed_pid: Pid` | `PidState::speed` (already exists, currently dead) |
| `stall_pid: Pid` | `PidState::stall` (already exists, currently dead) |
| `use_current_limit: bool` | `PidState::use_current_limit` (already exists) |
| `current_limit_adjust: i16` | `PidState::current_limit_adjust` (already exists) |
| `stall_protection_adjust: i32` | `PidState::stall_adjust` (already exists, currently dead) |
| `use_speed_control_loop: bool` | `PidState::use_speed_control` (already exists, currently dead) |
| `speed_input_override: i32` | `PidState::input_override` (already exists, currently dead) |
| `stall_protect_target_interval: u16` | `PidState` (new field) |
| `target_e_com_time: u32` | `PidState` (new field, speed PID target) |

**Fix:** Delete the 5 "dead" PidState fields and the 8+ loose MainState
fields. Consolidate into a single `PidState` with all PID concerns, then
`MainState` holds `pub pid: PidState`.

### Timing/RPM fields — belong in `TimingState`

| MainState field | Should be in |
|----------------|-------------|
| `e_rpm: u16` | `TimingState::e_rpm` (already exists) |
| `average_interval: u32` | `TimingState::average_interval` (already exists) |
| `last_average_interval: u32` | `TimingState::last_average_interval` (already exists) |
| `commutation_intervals: [u16; 6]` | `TimingState::commutation_intervals` (already exists) |

**Fix:** `MainState` should hold `pub timing: TimingState` instead of 4
loose fields. `TimingState` already has these fields — they're just also
in `MainState` as duplicates.

### Measurement/sensor fields — belong in `Measurements`

| MainState field | Should be in |
|----------------|-------------|
| `voltage_divider: u16` | Board-level config, not runtime state |
| `millivolt_per_amp: u16` | Board-level config, not runtime state |
| `current_offset: i16` | Board-level config, not runtime state |
| `current_filter: CurrentFilter` | `Measurements` (measurement processing) |
| `voltage_filter: EwmaPow2<3>` | `Measurements` (measurement processing) |
| `use_ntc: bool` | Board-level config |
| `cell_count: u8` | LVC concern — could be in `ProtectionState` or `Measurements` |
| `motor_kv: u16` | Motor config, not runtime state |
| `low_cell_volt_cutoff: u16` | LVC concern — `ProtectionState` |

**Fix:** Board constants (`voltage_divider`, `millivolt_per_amp`,
`current_offset`, `use_ntc`) belong in `MainStateParams` (already exists)
or a `BoardConfig` sub-struct. Filters belong with `Measurements`.
LVC fields belong in `ProtectionState`.

### What should remain in `MainState`

After decomposition, `MainState` should be a thin coordinator holding:
- `pub config: EepromConfig`
- `pub protection: ProtectionState`
- `pub measurements: Measurements`
- `pub telemetry: TelemetryState`
- `pub pid: PidState`
- `pub timing: TimingState`
- `pub board: BoardParams` (voltage_divider, millivolt_per_amp, etc.)
- `pub led: LED` + `led_counter`
- `pub desync_check: bool`
- `pub last_armed: bool` / `just_armed: bool`
- `pub timer1_max_arr: u16` / `cpu_mhz: u8`
- `pub ten_khz_counter: u32`

That's ~12 fields (mostly sub-structs) instead of 35 loose scalars.

## `SharedComm` — Same Pattern, Different Layer

`SharedComm` has 63 methods and a flat namespace. The same decomposition
applies: group related methods into sub-traits or sub-structs. But this
is a separate, larger refactor — the ISR↔main boundary has real atomicity
constraints that make sub-structuring harder.

Not in scope for this audit, but noted for future work.

## Cross-Crate Access (16 fields)

These fields are accessed from harness and/or firmware outside `rm32`:

| Struct | Fields | Accessed By |
|--------|--------|-------------|
| `BemfState` | `counter`, `zc_found`, `filter_level`, `temp_advance` | harness MotorContext, print_state |
| `DutyState` | `cycle`, `last`, `adjusted`, `minimum`, `min_startup`, `startup_max` | harness MotorContext, firmware ISR init |
| `ProtectionState` | `bemf_timeout_happened`, `bemf_timeout`, `low_voltage_count` | harness config injection, firmware LED |
| `Measurements` | `battery_voltage`, `actual_current`, `degrees_celsius` | harness print_state |

These are legitimately cross-crate. Options:
- **Keep `pub`** — acceptable for now, these are data structs not invariant holders
- **Add accessors later** — when/if field invariants emerge

## Recommended Sequence

1. **Delete 14 dead fields** from state.rs — pure removal, zero behavior change
2. **Adopt `PidState` into `MainState`** — move loose PID fields back into
   PidState, replace with `pub pid: PidState`
3. **Adopt `TimingState` into `MainState`** — replace 4 loose timing fields
4. **Move filters into `Measurements`** — `current_filter`, `voltage_filter`
5. **Extract `BoardParams`** from `MainState` — voltage_divider, millivolt_per_amp,
   current_offset, use_ntc (partially done via `MainStateParams`)
6. **Change non-cross-crate fields to `pub(crate)`** — prevents new leaks
7. **Dead code audit** — after visibility tightening, check for write-only fields
