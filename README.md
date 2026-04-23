# RM32 — Rust Reimplementation of AM32 ESC Firmware

A no_std, no-alloc Rust port of the [AM32](https://github.com/am32-firmware/AM32) brushless ESC firmware.

## Architecture

```
rm32/          Core library — all motor control logic, protocol decoders, HAL traits
rm32_stm32/    STM32 HAL implementation — peripheral drivers, ISR wrappers, MCU init
rm32_std/      Host test harness — same stdin/stdout protocol as C am32_harness
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

## Test Coverage

- **199 unit tests** — motor control, DShot decode/encode, EDT, CRSF, servo, commutation, PID, EEPROM validation, EWMA filter, WS2812
- **33 blackbox test vectors** — arming, DShot commands, servo input, desync recovery, LVC, variable PWM, BEMF timeout, telemetry, and more
- Tests run against both C and Rust harnesses to verify behavioral parity

## Implemented Features

- 6-step BLDC commutation with BEMF zero-cross detection
- DShot 300/600 input with bidirectional telemetry (GCR encoding)
- Servo PWM input with auto-detection, calibration, and rate limiting
- CRSF serial input parser (420kbaud, CRC-8/DVB-S2, 16-channel decode)
- Extended DShot Telemetry (EDT) — current/voltage/temperature multiplexing
- KISS ESC telemetry (10-byte packets via UART DMA)
- DShot command processing (beacons, direction, bidir, EDT, save, programming mode)
- EEPROM config with version migration and blank-flash fallback
- Variable PWM modes 0 (fixed), 1 (manual range), 2 (automatic eRPM-based)
- PID current/speed/stall control loops
- Low voltage cutoff with fast startup timeout
- Dynamic BEMF timeout thresholds (throttle-dependent)
- Dynamic interrupt priority swap (Cortex-M4F only)
- IWDG watchdog (per-MCU timeout configuration)
- WS2812 LED status indicator (boot/armed)
- Battery voltage EWMA smoothing
- Current sensing with per-board offset calibration

## Not Implemented

The following features from the C firmware are intentionally not ported:

| Feature | Reason |
|---------|--------|
| **MultiShot input** | Obsolete protocol, fully replaced by DShot |
| **DroneCAN** | Deferred — requires no-alloc DroneCAN crate work |
| **Brushed motor mode** | Different motor type entirely; not needed for BLDC ESCs |
| **Additional MCU targets** (F031, G031, F415, AT32, GD32) | 3 STM32 targets prove the abstraction; more can be added on demand |
| **CRSF UART RX hardware wiring** | Parser complete; HAL wiring deferred to board with CRSF hardware |
