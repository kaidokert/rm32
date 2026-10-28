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

Both crates are experimental pre-1.0 libraries. Source-breaking changes use a new
minor version. Version 0.2 changes the motor service interfaces:

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
