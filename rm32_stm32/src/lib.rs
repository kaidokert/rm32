//! RM32 STM32 HAL implementation.
//!
//! Provides hardware drivers for STM32G0xx (initially STM32G071).
//! Implements the `rm32::hal` traits for real hardware.

#![no_std]

// TODO: Implement HAL traits for STM32G071 peripherals:
// - TIM1 → PwmOutput + PhaseOutput
// - TIM2 → IntervalTimer
// - TIM6 → 10kHz tick source
// - TIM14/TIM16 → ComTimer
// - COMP1 → Comparator
// - USART2 → TelemetryUart
// - ADC → Adc
// - Flash → Flash
// - DMA → InputCapture
