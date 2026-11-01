//! G071 ADC: CH4 current, CH6 voltage, CH12 temp. DMA1_CH2 circular + DMAMUX.

use crate::adc_hal::AdcPeripheral;
use crate::pac::{ADC, DMA1, GPIOA, RCC};
use crate::regs::{InitError, wait_for};

crate::define_adc_boilerplate!(
    ops: G071AdcOps,
    type_name: AdcReader,
    cal1: 0x1FFF_75A8, cal2: 0x1FFF_75CA,
    cal1_temp: 30, cal2_temp: 130,
    cal_vref_mv: 3000,
);

pub struct G071AdcOps;

/// Regular sequence, CHSELRMOD = 1 nibble format, 0xF terminated.
const SEQUENCE: u32 = 4 | (6 << 4) | (12 << 8) | (0xF << 12); // I, V, temp

/// Program the regular sequence with the RM0444 §15.3.8 handshake: set
/// CHSELRMOD (ADSTART = 0), wait for and clear CCRDY, write CHSELR1, wait
/// CCRDY again, and verify by readback. A CHSELR1 write issued before CCRDY
/// is ignored by hardware, which would leave CHSELR1 = 0 (eight conversions
/// of IN0) instead of scanning current, voltage, and temperature.
fn program_sequence() -> Result<(), InitError> {
    let adc = unsafe { &*ADC::ptr() };
    const CCRDY: u32 = 1 << 13;
    adc.cfgr1()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 21)) }); // CHSELRMOD
    wait_for(
        || unsafe { (&*ADC::ptr()).isr().read().bits() & CCRDY != 0 },
        100_000,
        "ADC CCRDY (mode)",
    )?;
    adc.isr().write(|w| unsafe { w.bits(CCRDY) });
    adc.chselr1().write(|w| unsafe { w.bits(SEQUENCE) });
    wait_for(
        || unsafe { (&*ADC::ptr()).isr().read().bits() & CCRDY != 0 },
        100_000,
        "ADC CCRDY (sequence)",
    )?;
    adc.isr().write(|w| unsafe { w.bits(CCRDY) });
    if adc.chselr1().read().bits() != SEQUENCE {
        return Err(InitError::Timeout("ADC CHSELR readback"));
    }
    Ok(())
}

/// ADC_CR.ADVREGEN — kept set on every CR write.
const ADVREGEN: u32 = 1 << 28;

impl AdcPeripheral for G071AdcOps {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apbenr2()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 20)) }); // ADCEN
        rcc.ahbenr()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) }); // DMA1EN
    }

    fn configure_pins(&self) {
        let gpioa = unsafe { &*GPIOA::ptr() };
        gpioa.moder().modify(|r, w| unsafe {
            w.bits(r.bits() | (0b11 << 8) | (0b11 << 12)) // PA4, PA6 analog
        });
    }

    fn configure_clock_source(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr2().write(|w| unsafe { w.bits(0b10 << 30) }); // CKMODE
    }

    fn enable_temp_sensor(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.ccr()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 23)) }); // TSEN
    }

    fn configure_dma(&self, buf_ptr: *const u16, buf_len: u16) {
        let adc = unsafe { &*ADC::ptr() };
        let dma = unsafe { &*DMA1::ptr() };
        let dmamux = unsafe { &*crate::pac::DMAMUX::ptr() };

        dmamux
            .ccr(1)
            .modify(|r, w| unsafe { w.bits((r.bits() & !0x3F) | 5) });
        let ch = dma.ch2();
        ch.cr().write(|w| w.en().clear_bit());
        ch.par()
            .write(|w| unsafe { w.bits(adc.dr().as_ptr() as u32) });
        ch.mar().write(|w| unsafe { w.bits(buf_ptr as u32) });
        ch.ndtr().write(|w| unsafe { w.bits(buf_len as u32) });
        ch.cr().write(|w| unsafe {
            w.bits((1 << 1) | (1 << 5) | (1 << 7) | (0b10 << 8) | (0b01 << 10) | (0b10 << 12))
        });
        ch.cr().modify(|r, w| unsafe { w.bits(r.bits() | 1) });
    }

    fn configure_sampling(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.smpr()
            .write(|w| unsafe { w.bits(0b011 | (0b111 << 4)) });
    }

    /// Sequence programming is deferred to `enable()` (after ADRDY), with
    /// the RM0444 CCRDY handshake — see `program_sequence`.
    fn configure_sequence(&self) {}

    fn enable_dma_mode(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cfgr1().modify(|r, w| unsafe {
            w.bits((r.bits() & !0b11) | (0b01 << 0)) // DMAEN
        });
        adc.cfgr1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 3)) });
    }

    /// ADC voltage regulator: must be on (and settled, tADCVREG_STUP
    /// <= 20 µs) before calibration (RM0444 §15.3.2; AM32
    /// `LL_ADC_EnableInternalRegulator`). Every subsequent CR write preserves
    /// `ADVREGEN` so calibration and conversions do not power down the ADC.
    fn power_up(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr().write(|w| unsafe { w.bits(ADVREGEN) });
        cortex_m::asm::delay(64 * 25);
    }

    fn calibrate(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr().write(|w| unsafe { w.bits(ADVREGEN | (1 << 31)) }); // ADCAL
        wait_for(
            || unsafe { (&*ADC::ptr()).cr().read().bits() & (1 << 31) == 0 },
            100_000,
            "ADC cal",
        )?;
        cortex_m::asm::delay(64 * 20);
        Ok(())
    }

    fn enable(&self) -> Result<(), InitError> {
        let adc = unsafe { &*ADC::ptr() };
        adc.isr().write(|w| unsafe { w.bits(1 << 0) }); // clear ADRDY
        adc.cr().write(|w| unsafe { w.bits(ADVREGEN | (1 << 0)) }); // ADEN
        wait_for(
            || unsafe { (&*ADC::ptr()).isr().read().bits() & (1 << 0) != 0 },
            100_000,
            "ADC ready",
        )?;
        if let Err(e) = program_sequence() {
            adc.cr().write(|w| unsafe { w.bits(ADVREGEN | (1 << 1)) }); // ADDIS
            return Err(e);
        }
        Ok(())
    }

    fn start_conversion(&self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 2)) }); // ADSTART
    }
}
