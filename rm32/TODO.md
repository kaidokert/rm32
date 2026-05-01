# RM32 Polish & Style TODO

Architecture is complete. These are refinement items — zero identity risk, zero behavioral change.

## Done

- [x] **Move MotorState builder math to rm32** — `EepromConfig::derive_motor_config()` in
  `rm32/src/config.rs` with 7 unit tests. main.rs calls it instead of inline math.
- [x] **DmaBuf alignment to 32 bytes** — `#[repr(align(32))]` for cache-line safety on M4.
- [x] **BEMF fault sentinel constant** — `BEMF_FAULT_LATCHED = 102` replaces magic number in tick.rs.
- [x] **Standardize `// SAFETY:` docs** — Added to key unsafe blocks: chip.rs clock enables,
  main.rs bootloader reads, dma_buf.rs, isr_handlers.rs IsrCell, adc_hal.rs TempCalibration,
  flash.rs memory-mapped read.
- [x] **InitError enum** — Already structured: `AdcInit`, `ClockInit`, `UartInit`, `FlashError`.

## Deferred (assessed, low ROI for this codebase)

- **Replace IsrCell with `static_cell`** — Doesn't fit the ISR pattern. ISR needs
  lazy-init-then-cache-for-repeated-access (`get()` called every 50µs). `static_cell`
  provides init-once-and-return but the ISR has no persistent local to store the
  returned `&'static mut`. Current IsrCell is correct and well-documented.

- **Move GenericAdc & GenericTelem to rm32** — Init sequence is 12 lines of trait calls.
  Would require also moving DmaBuf, AdcPeripheral, TempCalibration, InitError to core crate.
  The actual complexity is in each MCU's register-level impl, not the sequencing.

- **Result for non-critical IO** — DMA send is fire-and-forget on hardware. Flash write
  errors can't be handled usefully in an ESC (no UI, no retry path). Adding `Result`
  would add ceremony without actionable error handling.

- **Full polarity enum (`rising: bool` → `BemfPolarity`)** — Deeply wired through HAL
  traits, commutation, ISR logic (~20 files). `rising` is already clear as a field name.
  Named the magic sentinel constant instead.
