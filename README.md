# RM32 — Rust Reimplementation of AM32 ESC Firmware

A no_std, no-alloc Rust port of the [AM32](https://github.com/am32-firmware/AM32) brushless ESC firmware.

**Completely untested and experimental** - it's just a code port with no hardware validation.

## Architecture

```
rm32/          Core library — all motor control logic, protocol decoders, HAL traits
rm32_stm32/    STM32 HAL implementation — peripheral drivers, ISR wrappers, MCU init
```

## Supported Targets

| MCU | Architecture | Board Example | Status |
|-----|-------------|---------------|--------|
| STM32G071 | Cortex-M0+ | AM32 G071 64K ESC | Complete |
| STM32F051 | Cortex-M0 | Siskin F051 | Port compiles; image exceeds 27 KiB application flash |
| STM32L431 | Cortex-M4F | Neutron L431 | Complete |

Additional STM32 targets can be added following the same pattern (cfg-gated modules + shared ISR logic).

## Building

F051 firmware currently exceeds its 27 KiB application partition; reducing its
size is separate follow-up work. The cross-build checks validate compilation,
not deployability. Root-invoked Cargo commands may omit the linker configuration
under `rm32_stm32/.cargo`; verify the real flash/RAM limits with an explicit full
link before flashing any target.

```bash
# Core library tests (host)
cargo test -p rm32

# STM32 firmware (one target at a time)
cd rm32_stm32
cargo build --release --features stm32g071 --no-default-features --target thumbv6m-none-eabi
# F051: compilation alone does not establish flash fit (see above)
cargo build --release --features stm32f051 --no-default-features --target thumbv6m-none-eabi
cargo build --release --features stm32l431 --no-default-features --target thumbv7em-none-eabihf

# Blackbox tests (requires Python 3 + pytest)
cd ../tests/blackbox
HARNESS_EXE=../../downloads/target/release/rm32_harness python3 -m pytest test_vectors.py -v
```

## Library compatibility

Input actions are now applied by `transfer::dispatch` in both firmware and the
host harness. `InputContext` borrows the caller's command/configuration and EDT
state; `DispatchResult` returns tone intent without playing it. CRSF callers use
`CrsfParser::dispatch_byte`. Capture decoding, command repetition, and the
armed/stopped command gate are unchanged.

Programming command 36 now connects to the existing programming state machine:
after six accepted command repetitions, the next valid DShot values supply the
EEPROM byte offset and value; value 37 commits the byte to RAM. Programming data
does not become motor throttle. Out-of-range offsets are rejected, loss of the
armed/stopped gate cancels the sequence, and command 12 remains the explicit
save request. CRC-valid CRSF channel frames with missing payload are rejected.

Input configuration changes reach main through the bounded publication queue.
If it fills, main recovers an authoritative snapshot with input interrupts
excluded. Saves claim a request and copy the live ISR configuration under the
same exclusion, then perform flash I/O afterwards. Host port tests execute the
production ISR adapters after their first-interrupt ownership transfer; they
model exclusion and ordering, not hardware interrupt latency.

The harness accepts `crsf c8 ...` with space-separated hexadecimal bytes for raw
serial-input tests. Explicit `eeprom.*` configuration injection still updates the
test setup; command processing no longer relies on unconditional config copies
that could hide missing ISR-to-main publication.

Both crates are experimental pre-1.0 libraries. Source-breaking changes use a new
minor version. Version 0.4 combines the power-stage and BEMF interfaces:

- Implement `bridge::Bridge` over the phase driver and PWM timer. `MotorHal`
  returns a borrowed `Bridge<'a>` through `bridge()` and exposes `Bemf` through
  `bemf()`. Separate `pwm()`, `phase()`, and `comp()` accessors are removed.
  STM32 uses `StmBridge` over its existing timer and GPIO drivers; host ports
  can return `&mut Self` when they implement `Bridge` directly.
- Replace `Comparator` with `BemfSensor`. Route a `hal::Phase` and edge together
  with `select_phase`; use `floating_phase(step)` for six-step commutation.
  IRQ enable, mask-and-clear, pending, clear, and software-pend operations are
  explicit. `filter_read` defaults to an ordinary level read. These operations
  do not add interrupt entry gates or change the persistence filter.
- `step_legs` defines the six-step table shared by GPIO and BEMF selection.
  The GPIO driver retains its constructor-selected complementary/bridge-enable
  mode and existing write order. Firmware constructors still select
  non-complementary drive; this change does not apply a new EEPROM drive policy.
- Tone methods now take one `Bridge` instead of separate PWM and phase outputs.
  Tones remain blocking, with the same interrupt ownership. `latch` explicitly
  applies queued timer values; per-phase compares remain ordered A, B, C.

The host register tests execute production GPIO, bridge, and all four comparator
adapters with observable register substitutes. They verify software ordering and
routing, not analog behavior, gate timing, or interrupt latency. Those still need
hardware validation. F051 flash fit remains a separate limitation.

Version 0.3 makes the clock contract explicit:

- Implement `platform::Platform` for the port and its `MotorHal`, supplying
  control, interval/commutation, and capture clock rates, GCR bit shift, and
  base PWM period. STM32 uses `platform::StmPlatform`.
- Select that port in `MainState<P, LED>`, `SystemTick<P>`, `Measurements<P>`,
  and `SharedState::new::<P>()`. Shared state no longer has a clock-independent
  `Default`. Board ADC calibration still comes from `BoardConfig`.
- Use `constants::Timing` for clock-derived thresholds; raw clock-dependent
  globals are removed. Select `P` in `sine_step`, `commutation_timer_expired`,
  `TransferState::process`, and capture-setup constructors. Transfer processing
  no longer takes a separate CPU MHz argument. Signal timeouts are now `u32`.

STM32 control ticks remain 20 kHz and interval timers remain 2 MHz. Other clock
rates require execution-time and hardware validation; see `Platform` for the
counter constraints. PWM ownership, measurement scaling, and telemetry wire
formats are unchanged.

Version 0.2 introduced these motor service interfaces:

- `control::isr_logic::bemf_zero_cross` now requires shared motor state as its
  first argument so it can reject interrupts in stopped, sine, and polling modes.
- Replace separate `SystemTick::tick_sine` / `apply_sine_changeover` calls with
  `SystemTick::service_sine`, supplying the live HAL, duty state, and commutation
  sector. Exclude motor interrupts for the call; perform its returned delay
  afterwards. STM32 firmware uses `isr_handlers::service_sine` for that exclusion.
- `RawTimer` implementations must provide a real `read_sr` status-register read.
  A constant default would either suppress real commutations or accept stale IRQs.

The split sine methods are private implementation details in 0.2; the coordinated
service owns output setup and handoff publication. EEPROM and telemetry wire
formats are unchanged.
