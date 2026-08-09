//! G071 interrupt vectors — thin wrappers calling shared handlers.

use crate::isr_handlers;
use stm32g0xx_hal::stm32::interrupt;

#[interrupt]
fn TIM6_DAC_LPTIM1() {
    let tim6 = unsafe { &*stm32g0xx_hal::stm32::TIM6::ptr() };
    tim6.sr().modify(|_, w| w.uif().clear_bit());
    isr_handlers::handle_tim6();
}

#[interrupt]
fn TIM14() {
    isr_handlers::handle_tim14();
}

#[interrupt]
fn ADC_COMP() {
    isr_handlers::handle_comp();
}

#[interrupt]
fn DMA1_CHANNEL1() {
    let dma = unsafe { &*stm32g0xx_hal::stm32::DMA1::ptr() };
    if dma.isr().read().tcif1().bit_is_set() {
        dma.ifcr().write(|w| w.cgif1().set_bit());
        dma.ch(0).cr().modify(|_, w| w.en().clear_bit());
        isr_handlers::handle_dma_tc();
        let exti = unsafe { &*stm32g0xx_hal::stm32::EXTI::ptr() };
        exti.swier1().write(|w| unsafe { w.bits(1 << 15) });
    }
    if dma.isr().read().htif1().bit_is_set() {
        dma.ifcr().write(|w| w.chtif1().set_bit());
    }
}

#[interrupt]
fn EXTI4_15() {
    let exti = unsafe { &*stm32g0xx_hal::stm32::EXTI::ptr() };
    exti.rpr1().write(|w| unsafe { w.bits(1 << 15) });
    exti.fpr1().write(|w| unsafe { w.bits(1 << 15) });
    let next_capture = isr_handlers::handle_exti_frame();

    let tim3 = unsafe { &*stm32g0xx_hal::stm32::TIM3::ptr() };
    if let Some(psc) = next_capture.prescaler {
        tim3.psc().write(|w| unsafe { w.bits(psc as u32) });
        tim3.egr().write(|w| w.ug().set_bit());
    }

    // Re-enable DMA
    let dma = unsafe { &*stm32g0xx_hal::stm32::DMA1::ptr() };
    dma.ch(0)
        .ndtr()
        .write(|w| unsafe { w.bits(next_capture.ndtr) });
    dma.ch(0).cr().modify(|_, w| w.en().set_bit());
    tim3.cr1().modify(|_, w| w.cen().set_bit());
}
