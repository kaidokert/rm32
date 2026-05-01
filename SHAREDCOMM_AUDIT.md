# SharedComm Decomposition Plan

## Problem

`SharedComm` is a 63-method flat trait that serves as the only ISR↔main
communication channel. Every field that needs to cross the interrupt
boundary gets dumped here with no organization. This makes it:

1. Impossible to reason about which context owns which data
2. Easy to add fields to SharedComm instead of putting them where they belong
3. The TestShared/SharedState implementations must mirror every method or
   silently get default no-ops (the prop_brake_active bug)

## Current Method Count by Category

| Category | Methods | Direction | Notes |
|----------|---------|-----------|-------|
| Motor state machine | 11 | bidirectional | MotorMode + convenience getters/setters |
| ISR→Main timing | 9 | ISR writes, main reads | zero_crosses, commutation_interval, e_com_time, etc. |
| Main→ISR control | 13 | main writes, ISR reads | adjusted_input, PID outputs, PWM config, measurements |
| Input/DMA | 5 | DMA ISR writes, both read | newinput, input_set, is_dshot, dshot_telemetry |
| One-shot flags | 6 | bidirectional | send_telemetry, save_settings, send_esc_info |
| **Total** | **44 unique fields** | | (63 methods = getters + setters + helpers) |

## Proposed Sub-Traits

### 1. `MotorState` (11 methods)
Motor mode state machine. Already well-encapsulated via MotorMode enum.
```
motor_mode, set_motor_mode, transition
armed, running, old_routine, stepper_sine + setters
```

### 2. `IsrTiming` (9 methods) — ISR writes, main reads
```
zero_crosses, set_zero_crosses, increment_zero_crosses
commutation_interval, set_commutation_interval
e_com_time, set_e_com_time
interval_timer_count, set_interval_timer_count
```

### 3. `MainControl` (13 methods) — main writes, ISR reads
```
adjusted_input, set_adjusted_input
duty_cycle_setpoint, set_duty_cycle_setpoint
prop_brake_active, set_prop_brake_active
stall_protection_adjust, set_stall_protection_adjust
current_limit_adjust, set_current_limit_adjust
tim1_arr, set_tim1_arr
duty_maximum, set_duty_maximum
filter_level, set_filter_level
min_bemf_counts, set_min_bemf_counts
auto_advance, set_auto_advance
```

### 4. `InputSignal` (5 methods) — DMA ISR writes, both read
```
newinput, set_newinput
input_set, set_input_set
is_dshot, set_is_dshot
dshot_telemetry (read-only from main's perspective)
signal_timeout, increment_signal_timeout
```

### 5. `IsrFeedback` (7 methods) — ISR writes, main reads
```
duty_cycle, set_duty_cycle
forward, set_forward (bidirectional — input writes, ISR syncs)
```

### 6. `Telemetry` (9 methods) — main writes for EDT, one-shot flags
```
actual_current, set_actual_current
battery_voltage, set_battery_voltage, battery_voltage
degrees_celsius, set_degrees_celsius
send_telemetry, set_send_telemetry
save_settings_flag, set_save_settings_flag
send_esc_info_flag, set_send_esc_info_flag
```

## Implementation Strategy

**Option A: Sub-traits** — split SharedComm into 6 sub-traits, SharedComm
becomes a super-trait that requires all of them. Callers import only the
sub-trait they need. Gradual migration: start with one sub-trait, move
methods over, update callers.

**Option B: Sub-structs** — group atomic fields in SharedState into
sub-structs (e.g., `SharedState { motor: MotorState, timing: IsrTiming, ... }`).
SharedComm trait stays flat but implementations are organized. Simpler
refactor but doesn't enforce interface segregation.

**Option C: Sub-structs with accessor traits** — combine B with
trait-based access. Sub-structs own atomics, accessor traits provide
typed read/write. Most Rustic but largest change.

## Recommended: Option A (sub-traits)

Sub-traits are the smallest useful change:
- No struct layout changes needed
- Callers can be migrated one function at a time
- Compiler enforces that ISR-only code doesn't accidentally depend on
  main-only fields
- TestShared compile errors become scoped: "you forgot to implement
  IsrTiming" is more actionable than "63 methods, some have defaults"

## Naming Collisions (discovered during field privacy pass)

The encapsulation work surfaced a concrete example of the problem:
`set_battery_voltage` exists on three different types with three
different signatures:

- `Measurements::set_battery_voltage(&mut self, v: MilliVolts)` — domain struct, typed
- `SharedState::set_battery_voltage(&self, v: u16)` — atomic publish for ISR/EDT
- `SharedComm::set_battery_voltage(&self, _v: u16) {}` — trait default no-op

Same pattern repeats for `actual_current`, `degrees_celsius`. The
domain struct owns the data (correct), SharedComm publishes it across
the ISR boundary (necessary), but the naming collision creates
confusion about which is authoritative.

This will resolve naturally when SharedComm is decomposed — the
measurement publish methods belong in a `Telemetry` sub-trait with
a distinct namespace, not sharing names with the domain struct.

## Sequence

1. Extract `MotorState` sub-trait (11 methods, already logically grouped)
2. Extract `IsrTiming` sub-trait (9 methods, clear ISR→main flow)
3. Extract `MainControl` sub-trait (13 methods, clear main→ISR flow)
4. Extract `Telemetry` sub-trait (measurement publish + flags)
5. Remaining methods stay in SharedComm until natural groups emerge
6. At each step: update callers to import the sub-trait they need
