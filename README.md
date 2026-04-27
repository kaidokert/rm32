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
| STM32F051 | Cortex-M0 | Siskin F051 | Complete |
| STM32L431 | Cortex-M4F | Neutron L431 | Complete |

Additional STM32 targets can be added following the same pattern (cfg-gated modules + shared ISR logic).

## Building

```bash
# Core library tests (host)
cargo test -p rm32

# STM32 firmware (one target at a time)
cd rm32_stm32
cargo build --release --features stm32g071 --no-default-features --target thumbv6m-none-eabi
cargo build --release --features stm32f051 --no-default-features --target thumbv6m-none-eabi
cargo build --release --features stm32l431 --no-default-features --target thumbv7em-none-eabihf

# Blackbox tests (requires Python 3 + pytest)
cd ../tests/blackbox
HARNESS_EXE=../../downloads/target/release/rm32_harness python3 -m pytest test_vectors.py -v
```
