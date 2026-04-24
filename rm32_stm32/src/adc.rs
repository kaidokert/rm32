//! ADC + DMA for voltage, current, and temperature measurement.
//!
//! ADC1 with 3-channel scan: current (PA4/CH4), voltage (PA6/CH6),
//! temperature (internal sensor). DMA1 Channel 2 in circular mode.
//! Software-triggered conversions.
//!
//! For HARDWARE_GROUP_G0_A:
//!   Current: PA4 (ADC_IN4)
//!   Voltage: PA6 (ADC_IN6)
//!   Temperature: internal sensor (CH_TEMPSENSOR)

use crate::pac::{ADC, DMA1, GPIOA, RCC};
use rm32::hal::Adc;

// ADC DMA buffer is intentionally kept as a static mut rather than a field on AdcReader.
// The ADC DMA runs in circular mode: the MAR is configured once in init() and the
// hardware continuously refills the buffer without software involvement. If the buffer
// were a struct field, the struct (and therefore the buffer) could be moved in memory
// after init() returns, causing DMA to write to a stale address and silently corrupt
// memory. Keeping it static guarantees a fixed address for the lifetime of the program.
static mut ADC_DMA_BUF: [u16; 3] = [0; 3];

pub struct AdcReader {
    _private: (),
}

impl AdcReader {
    /// Create a handle to already-initialized ADC hardware.
    pub fn post_init() -> Self { Self { _private: () } }

    pub fn init() -> Result<Self, crate::regs::InitError> {
        let rcc = unsafe { &*RCC::ptr() };
        let gpioa = unsafe { &*GPIOA::ptr() };
        let adc = unsafe { &*ADC::ptr() };
        let dma = unsafe { &*DMA1::ptr() };

        // Enable clocks
        rcc.apbenr2().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 20)) }); // ADCEN
        rcc.ahbenr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });   // DMA1EN

        // PA4, PA6 as analog
        gpioa.moder().modify(|r, w| unsafe {
            w.bits(r.bits() | (0b11 << 8) | (0b11 << 12)) // PA4, PA6 = analog
        });

        // DMAMUX: Channel 2 (index 1) → ADC1 (request 5)
        let dmamux_base = 0x4002_0800u32;
        unsafe {
            let c1cr = (dmamux_base + 4) as *mut u32;
            let val = c1cr.read_volatile();
            c1cr.write_volatile((val & !0x3F) | 5);
        }

        // DMA1 Channel 2: periph→memory, 16-bit, memory increment, circular
        dma.ch(1).cr().write(|w| unsafe { w.bits(0) }); // disable
        dma.ch(1).par().write(|w| unsafe { w.bits(adc.dr().as_ptr() as u32) });
        dma.ch(1).mar().write(|w| unsafe { w.bits(ADC_DMA_BUF.as_ptr() as u32) });
        dma.ch(1).ndtr().write(|w| unsafe { w.bits(3) });
        dma.ch(1).cr().write(|w| unsafe {
            w.bits(
                (1 << 1)     // TCIE (transfer complete interrupt)
                | (1 << 5)   // CIRC (circular mode)
                | (1 << 7)   // MINC (memory increment)
                | (0b10 << 8)  // PSIZE = 32-bit (ADC DR is 32-bit wide)
                | (0b01 << 10) // MSIZE = 16-bit (buffer is u16)
                | (0b10 << 12) // PL = high priority
            )
        });
        dma.ch(1).cr().modify(|r, w| unsafe { w.bits(r.bits() | 1) }); // EN

        // Configure ADC
        // Clock: async /4
        adc.cfgr2().write(|w| unsafe { w.bits(0b10 << 30) }); // CKMODE=00, PRESC=DIV4 is in CCR

        // Enable internal temp sensor
        let adc_ccr = unsafe { &*((ADC::ptr() as u32 + 0x308) as *const crate::input_capture::VolatileCell<u32>) };
        adc_ccr.modify(|v| v | (1 << 23)); // TSEN

        // Sampling time: common 1 = 19.5 cycles, common 2 = 160.5 cycles
        adc.smpr().write(|w| unsafe {
            w.bits((0b011 << 0) | (0b111 << 4)) // SMP1=19.5, SMP2=160.5
        });

        // Channel sequence: 3 ranks
        // CHSELR in configurable mode: rank1=CH4(current), rank2=CH6(voltage), rank3=CH_TEMP(12)
        adc.cfgr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 21)) }); // CHSELRMOD=1
        // CHSELR register at offset 0x28 from ADC base
        let adc_base = ADC::ptr() as u32;
        unsafe {
            let chselr = (adc_base + 0x28) as *mut u32;
            chselr.write_volatile(
                (4 << 0)      // SQ1 = channel 4 (current)
                | (6 << 4)    // SQ2 = channel 6 (voltage)
                | (12 << 8)   // SQ3 = channel 12 (temp sensor)
                | (0xF << 12) // SQ4 = end
            );
        }

        // DMA transfer mode: limited (one DMA request per conversion)
        adc.cfgr1().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 << 0)) | (0b01 << 0)) // DMAEN=1, DMACFG=0 (one-shot per sequence)
        });

        // Resolution 12-bit, right-aligned
        adc.cfgr1().modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 3)) }); // RES=00 (12-bit)

        // Calibrate
        adc.cr().write(|w| unsafe { w.bits(1 << 31) }); // ADCAL
        crate::regs::wait_for(|| unsafe { adc.cr().read().bits() } & (1 << 31) == 0, 100_000, "ADC calibration")?;

        // Enable ADC
        cortex_m::asm::delay(64 * 20); // stabilization delay
        adc.isr().write(|w| unsafe { w.bits(1 << 0) }); // clear ADRDY
        adc.cr().write(|w| unsafe { w.bits(1 << 0) }); // ADEN
        crate::regs::wait_for(|| adc.isr().read().bits() & (1 << 0) != 0, 100_000, "ADC ready")?;

        Ok(Self { _private: () })
    }
}

impl Adc for AdcReader {
    fn start_conversion(&mut self) {
        let adc = unsafe { &*ADC::ptr() };
        adc.cr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 2)) }); // ADSTART
    }

    fn raw_voltage(&self) -> u16 {
        unsafe { ADC_DMA_BUF[1] }
    }

    fn raw_current(&self) -> u16 {
        unsafe { ADC_DMA_BUF[0] }
    }

    fn raw_temperature(&self) -> u16 {
        unsafe { ADC_DMA_BUF[2] }
    }

    fn calc_temperature(&self, raw: u16) -> i16 {
        // STM32G0 internal temp sensor formula:
        // temp = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1)) * (raw - TS_CAL1) + 30
        // Calibration values at 0x1FFF75A8 (30°C) and 0x1FFF75CA (130°C)
        let ts_cal1 = unsafe { *(0x1FFF_75A8 as *const u16) } as i32;
        let ts_cal2 = unsafe { *(0x1FFF_75CA as *const u16) } as i32;
        if ts_cal2 == ts_cal1 { return 25; } // avoid division by zero
        let temp = (130 - 30) * (raw as i32 - ts_cal1) / (ts_cal2 - ts_cal1) + 30;
        temp as i16
    }
}
