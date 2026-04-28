//! ADC driver abstraction — "our own HAL" for ADC init across STM32 families.
//!
//! The `AdcPeripheral` trait splits the monolithic init sequence into discrete
//! operations. Each MCU implements the register-level details; the shared
//! init sequence in `adc_generic.rs` calls them in the correct order.

use crate::regs::InitError;

/// MCU-specific ADC register operations.
/// Each method maps to one logical step of the ADC init sequence.
/// The generic driver calls these in order — MCU impls provide register writes.
pub trait AdcPeripheral {
    /// Enable ADC + DMA clocks via RCC.
    fn enable_clocks(&self);
    /// Configure GPIO pins as analog inputs.
    fn configure_pins(&self);
    /// Configure ADC clock source (CKMODE, CCR, etc).
    fn configure_clock_source(&self);
    /// Enable temperature sensor channel.
    fn enable_temp_sensor(&self);
    /// Setup DMA channel: PAR→ADC_DR, MAR→buf, NDTR=len, circular, 16-bit, enable.
    fn configure_dma(&self, buf_ptr: *const u16, buf_len: u16);
    /// Set sampling time for each channel.
    fn configure_sampling(&self);
    /// Configure channel sequence (which channels, in what order).
    fn configure_sequence(&self);
    /// Enable DMA mode in ADC config register.
    fn enable_dma_mode(&self);
    /// Exit deep power-down and enable voltage regulator.
    /// Default no-op — F051 doesn't have deep power-down.
    fn power_up(&self) {}
    /// Run ADC self-calibration.
    fn calibrate(&self) -> Result<(), InitError>;
    /// Enable ADC (ADEN + wait ADRDY).
    fn enable(&self) -> Result<(), InitError>;
    /// Trigger a new conversion.
    fn start_conversion(&self);
}

/// Temperature sensor calibration values (read from ROM at init time).
pub struct TempCalibration {
    pub cal1_val: u16,
    pub cal2_val: u16,
    pub cal1_temp: i32,
    pub cal2_temp: i32,
}

impl TempCalibration {
    /// Read calibration values from ROM addresses.
    ///
    /// # Safety
    /// `cal1_addr` and `cal2_addr` must point to valid, aligned, read-only
    /// factory calibration u16 values in ROM (per STM32 datasheet).
    pub unsafe fn from_rom(cal1_addr: u32, cal2_addr: u32, cal1_temp: i32, cal2_temp: i32) -> Self {
        // SAFETY: Caller guarantees cal1_addr and cal2_addr point to valid,
        // aligned, read-only factory calibration data in ROM (system memory).
        // These addresses are fixed per STM32 datasheet and always readable.
        Self {
            cal1_val: *(cal1_addr as *const u16),
            cal2_val: *(cal2_addr as *const u16),
            cal1_temp,
            cal2_temp,
        }
    }
}

/// Generate ADC boilerplate: DMA buffer static, temp cal const, type alias, constructors.
#[macro_export]
macro_rules! define_adc_boilerplate {
    (
        ops: $ops:ident,
        type_name: $type_name:ident,
        cal1: $cal1:expr, cal2: $cal2:expr,
        cal1_temp: $ct1:expr, cal2_temp: $ct2:expr $(,)?
    ) => {
        static ADC_DMA_BUF: $crate::dma_buf::DmaBuf<u16, 3> = $crate::dma_buf::DmaBuf::new();

        fn temp_cal() -> $crate::adc_hal::TempCalibration {
            let (a1, a2, t1, t2) = ($cal1, $cal2, $ct1, $ct2);
            // SAFETY: ROM calibration addresses are const per STM32 datasheet.
            unsafe { $crate::adc_hal::TempCalibration::from_rom(a1, a2, t1, t2) }
        }

        pub type $type_name = $crate::adc_generic::GenericAdc<$ops>;

        pub fn new_adc() -> $type_name {
            $crate::adc_generic::GenericAdc::new($ops, &ADC_DMA_BUF, temp_cal())
        }

        pub fn post_init() -> $type_name {
            $crate::adc_generic::GenericAdc::post_init($ops, &ADC_DMA_BUF, temp_cal())
        }
    };
}
