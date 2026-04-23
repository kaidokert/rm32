# RM32 STM32 Firmware — Status

199 unit tests, 33 blackbox tests. All 3 MCU targets compile.

## Target Status — All Complete

| Target | Arch | Peripherals | Notes |
|--------|------|-------------|-------|
| STM32G071 | Cortex-M0+ | All done | HAL-based PWM, DMA input capture, ADC, UART telem |
| STM32F051 | Cortex-M0 | All done | Raw register peripherals, TIM15 input capture |
| STM32L431 | Cortex-M4F | All done | DMA request mux, dynamic IRQ priority swap |

## Remaining Work

### Hardware Validation
- [ ] Verify zcfoundroutine busy-wait timing on real hardware
- [ ] Verify WS2812 LED bitbang timing on real hardware
- [ ] End-to-end motor spin test on each MCU

### Minor Items
- [ ] WS2812 error LED on stuck rotor (BEMF timeout path in main loop)
- [ ] `pwm_frequency` config field → `timer1_max_arr` init verification
- [ ] Unit tests for variable_pwm mode 2 clamping

### Not Implemented (by design)
See README.md for the full list of intentionally unimplemented features.
