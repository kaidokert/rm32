# RM32 vs AM32 Audit Checklist

Status key: `[ ]` = open, `[x]` = done, `[~]` = partially done, `[?]` = needs investigation, `[N]` = won't do

---

## 1. Major Functional Gaps

### 1.1 Variable PWM Modes
- [x] Mode 0 (fixed): `tim1_arr` stays at `timer1_max_arr` — works by default
- [x] Mode 1 (manual range): maps `commutation_interval` 96-200 → 50%-100% of max ARR
- [x] Mode 2 (automatic): scales `average_interval` × `cpu_mhz/9`, clamped to 100-250 — implemented
- [x] Added `cpu_mhz` field to MotorState for mode 2 scaling
- [ ] `pwm_frequency` config field → `timer1_max_arr` initialization — needs verification
- [ ] Blackbox test coverage for all 3 modes
- [ ] Unit test for mode 2 clamping behavior

### 1.2 WS2812 LED Strip
- [x] `ws2812.rs` in rm32 core — platform-independent bitbang: `WS2812Pin` trait, `send_rgb()`, `send_status()`
- [x] `LedStatus` enum: Boot (dim red), Armed (green), Error (red), Off
- [x] `ws2812_hal.rs` in rm32_stm32 — GPIO BSRR bitbang on GPIOB, `delay_ns` via `cortex_m::asm::delay`
- [x] `BoardConfig` — `has_led: bool` + `led_pin: Option<u8>` (PB8 default)
- [x] `main.rs` — Boot LED at startup, Armed LED on state transition (IRQs disabled during send)
- [x] 2 unit tests (call count, status colors)
- [ ] Error LED on stuck rotor (needs BEMF timeout path in main loop)

### 1.3 MultiShot Input
- [N] Not implementing — DShot has replaced it
- [ ] Document as "not supported" in root README.md (alongside DroneCAN)
- **Decision:** Won't do. Dead protocol. List in README as explicitly unimplemented.

### 1.4 Extended DShot Telemetry (EDT)
- [x] `EDT_ENABLE`/`EDT_DISABLE` command handling in CommandProcessor
- [x] `EdtScheduler` in rm32 core (`edt.rs`) — counter-based frame type selection
- [x] Scheduler: current every 40 frames (50mA/LSB), voltage every 200 (25mV/LSB), temp every 200 (°C)
- [x] Init frame `0xE00` and deinit frame `0xEFF` special values
- [x] Refactored `dshot.rs`: `encode_gcr_frame()` takes raw 12-bit, `erpm_to_12bit()` extracted
- [x] `handle_dma_tc()` consults scheduler — sends EDT or eRPM per frame
- [x] Shared atomics for current/voltage/temp (main→ISR)
- [x] CommandProcessor EDT flags propagated to scheduler in EXTI handler
- [x] 7 unit tests (init, deinit, inactive, current encoding, alternation, voltage, temperature)
- [x] EDT covered by 7 unit tests in edt.rs + dshot_commands_extended blackbox vector (end-to-end requires bidir DShot HW)

### 1.5 CRSF / Serial Input
- [x] `CrsfParser` in rm32 core (`crsf.rs`) — byte-at-a-time parser with sync, CRC-8/DVB-S2, 11-bit channel unpacking
- [x] CRC-8/DVB-S2 lookup table (polynomial 0xD5)
- [x] 16 × 11-bit channel unpacking from 22-byte payload
- [x] `channel_to_throttle()` maps CRSF range (172-1811) to ESC range (0-2047)
- [x] `SerialInput` HAL trait added
- [x] `CrsfParser` added to IsrState
- [x] `handle_crsf_byte()` ISR handler — feeds parser, sets newinput on valid channel frame
- [x] 9 unit tests (CRC, unpacking, throttle mapping, frame parsing, bad CRC, resync)
- [ ] Per-MCU UART RX interrupt/DMA wiring (call `handle_crsf_byte` from UART RX ISR)
- [ ] UART RX init at 420kbaud per MCU (reuse telemetry USART in RX mode, or second USART)
- **Note:** Core parser is complete and tested. HAL wiring deferred until board with CRSF input is available for testing. Exceeds C parity (C has no parser at all).

### 1.6 Target Support
- [x] STM32G071 — complete
- [x] STM32F051 — complete
- [x] STM32L431 — complete
- [N] STM32F031, G031, F415, F421, GD32E230, AT32F421 — not planned
- **Decision:** 3 STM32 targets are enough. Priority is clean multi-chip abstractions, not breadth. Additional MCUs can be added later using the same pattern (cfg-gated modules + shared ISR logic).

---

## 2. Logic Discrepancies

### 2.1 Desync Recovery — Interval Reset
- [x] `average_interval = 5000` reset when `zc > 100` during desync — fixed in main_loop.rs
- [x] Also sets `old_routine = true` on desync (matches C)
- [x] Note: C has a bug (zeros `zero_crosses` before checking `> 100`); Rust checks first
- [ ] Blackbox test vector for desync recovery

### 2.2 Low Voltage Cutoff Timing — Stepper Sine
- [x] Fast LVC timeout (1000 = 0.1s) during `stepper_sine`, normal (10000 = 10s) otherwise — fixed
- [ ] Blackbox test vector for fast LVC during startup

### 2.3 Battery Voltage Smoothing
- [x] `EwmaPow2<K>` filter added to rm32 core (`filter.rs`) — matches C's `(7*y+x)>>3` behavior
- [x] `voltage_filter: EwmaPow2<3>` added to MainState, wired into ADC measurement path
- [x] 4 unit tests (passthrough, convergence, step smoothing, K=1 fast response)
- **Reference:** `/opt/m/robotics/drones/AM32/downloads/ref/priv-servo-rs/servo/src/ewma_pow2.rs`

### 2.4 Current Offset & Scaling
- [x] `current_offset: i16` already in `BoardConfig` — per-board values set (e.g. NEUTRON_L431=498)
- [x] Full C scaling formula applied: `(smoothed * 3300/41 - offset*100) / mv_per_amp`
- [x] `current_offset` wired from `BOARD` config into `MainState`
- [x] Fixed telemetry current encoding (was double-scaling, now mA→centiamps)
- [ ] Consider replacing CurrentFilter with 50-sample moving average to match C exactly
- [ ] Unit test: verify scaling with known offset values

### 2.5 BEMF Timeout Dynamics
- [x] Dynamic threshold: 100 when `adjusted_input < 150`, 10 otherwise — fixed
- [x] Suppression: `bemf_timeout_happened = 0` when `zc > 100 && adj_input < 200` — fixed
- [x] Suppression: `bemf_timeout_happened = 0` when `use_sine_start && adj_input < 160` — fixed
- [x] Crawler mode skipped (C comments it as "no longer used")
- [ ] Blackbox test for low-throttle BEMF timeout behavior

### 2.6 Dynamic Interrupt Priority
- [x] L431: NVIC priority swap in main loop based on `commutation_interval` vs threshold (60)
- [x] Low eRPM (interval > 60): DMA1_CH5 priority 0, TIM1_UP_TIM16+COMP priority 1
- [x] High eRPM: commutation+comp priority 0, DMA priority 1
- [x] Gated behind `#[cfg(feature = "stm32l431")]` — no-op on M0+ targets
- [x] Uses raw NVIC_IPR register writes (IRQs 15, 25, 55)

---

## 3. Architectural Observations

### 3.1 Outdated TODOs
- [x] TODO.md rewritten this session — old stale entries removed
- [ ] Scrub remaining stale entries against this audit checklist
- **Decision:** Quick fix now.

### 3.2 Logic Duplication — ten_khz_tick
- [x] `SharedComm` trait in rm32 core (`shared_comm.rs`) — abstracts ISR↔main shared state
- [x] `TestShared` in `control/shared_impl.rs` — Cell-based impl for unit testing
- [x] `SharedComm` impl for `SharedState` in rm32_stm32 (`shared.rs`) — delegates to atomics
- [x] `isr_logic.rs` in rm32 core — `ten_khz_tick()`, `commutation_timer_expired()`, `bemf_zero_cross()` as free functions taking split state + `&dyn SharedComm` + HAL traits
- [x] `isr_handlers.rs` reduced to thin wrappers (~15 lines each) calling core functions
- [x] `Comparator::set_step()` added to HAL trait (was inherent-only)
- [x] All 182 tests pass, all 3 MCU targets compile
- [ ] Add unit tests for `isr_logic` functions using `TestShared` + `MockHal`

### 3.3 DShot Command Pipeline
- [x] TransferState now returns `dshot_command` field to ISR — fixed
- [x] EXTI handler dispatches via CommandProcessor — wired up
- **Note:** This was fixed during the current session.

---

## 4. Missing Safety/Sanity Checks

### 4.1 EEPROM Validation
- [x] `EepromConfig::is_valid()` — rejects `eeprom_version > EEPROM_VERSION` (catches blank 0xFF flash)
- [x] `EepromConfig::apply_version_defaults()` — migrates old configs (matches C's loadEEpromSettings)
- [x] main.rs: falls back to `EepromConfig::default()` if invalid, then applies version defaults
- [ ] Unit test: blank flash (all 0xFF) → falls back to defaults

### 4.2 Watchdog Reload Frequency
- [x] **IWDG was never started** — Rust had feed code but never configured/enabled the watchdog
- [x] Now started in main.rs after startup tune (matching C sequencing)
- [x] G071: prescaler /4, reload 4095 → 410ms timeout
- [x] F051: prescaler /16, reload 4000 → 1600ms timeout
- [x] L431: prescaler /16, reload 4000 → 1600ms timeout
- [x] Existing feed locations sufficient: main loop (20kHz cadence) + sound routines
- [x] Flash erase (~50ms) well within all budgets — no extra feeds needed
- [x] No sprinkling — clean, targeted fix

---

## Interview Notes

_Fill in after discussing each point:_

### Priority Classification
After review, classify each item:
- **P0 — Safety critical:** Must fix before any hardware testing
- **P1 — Functional parity:** Needed for production use
- **P2 — Nice to have:** Can ship without, add later
- **P3 — Won't do:** Not relevant to our targets/use case

| Item | Priority | Notes |
|------|----------|-------|
| 1.1 Variable PWM Modes | P1 | Support all 3. Mode 2 code + blackbox tests needed |
| 1.2 WS2812 LED | P2 done | Bitbang driver + boot/armed status. Error LED TODO |
| 1.3 MultiShot | N | Won't do. Document in README |
| 1.4 EDT | P1 done | Scheduler + GCR encoding + shared atomics. 7 tests |
| 1.5 CRSF | P1 done | Parser complete (9 tests). HAL UART RX wiring deferred to board testing |
| 1.6 Additional MCUs | N | 3 STM32s enough. Clean abstractions are the goal |
| 2.1 Desync interval reset | P0 | Safety. Add interval reset + blackbox test |
| 2.2 LVC stepper_sine timing | P0 | Safety. Fast cutoff during startup |
| 2.3 Voltage smoothing | P0 | Safety. EWMA from priv-servo-rs, K=3 |
| 2.4 Current offset | P1 | Per-board offset + 50-sample avg + C scaling formula |
| 2.5 BEMF timeout dynamics | P0 | Safety. Dynamic threshold + low-throttle suppression |
| 2.6 Dynamic IRQ priority | P1 done | L431 NVIC swap in main loop, threshold=60 |
| 3.1 Stale TODOs | P2 done | TODO.md rewritten, README.md created |
| 3.2 Logic duplication | P1 done | SharedComm trait + isr_logic.rs in core. Thin wrappers |
| 4.1 EEPROM validation | P0 | Safety. Blank flash = garbage config without this |
| 4.2 Watchdog coverage | P1 done | IWDG was never started! Now configured + enabled per MCU |
