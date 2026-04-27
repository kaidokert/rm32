//! Abstraction over MCU-specific ADC register access.

use crate::regs::InitError;

/// ADC peripheral operations (MCU-specific).
pub trait AdcOps {
    /// Full ADC initialization: clocks, GPIO, DMA, calibration, enable.
    fn init(&self) -> Result<(), InitError>;
    /// Trigger a new conversion sequence.
    fn start_conversion(&self);
}

/// Temperature sensor calibration info (MCU-specific).
pub struct TempCalibration {
    pub cal1_addr: u32,
    pub cal2_addr: u32,
    pub cal1_temp: i32,
    pub cal2_temp: i32,
}

/// Generate ADC boilerplate: DMA buffer static, temp cal const, type alias, constructors.
/// The `init()` impl is MCU-specific and must be written manually.
///
/// Usage:
/// ```ignore
/// define_adc_boilerplate!(
///     ops: MyAdcOps,
///     type_name: MyAdc,
///     cal1: 0x1FFF_75A8, cal2: 0x1FFF_75CA,
///     cal1_temp: 30, cal2_temp: 110,
/// );
/// ```
#[macro_export]
macro_rules! define_adc_boilerplate {
    (
        ops: $ops:ident,
        type_name: $type_name:ident,
        cal1: $cal1:expr, cal2: $cal2:expr,
        cal1_temp: $ct1:expr, cal2_temp: $ct2:expr $(,)?
    ) => {
        static ADC_DMA_BUF: $crate::dma_buf::DmaBuf<u16, 3> = $crate::dma_buf::DmaBuf::new();

        const TEMP_CAL: $crate::adc_hal::TempCalibration = $crate::adc_hal::TempCalibration {
            cal1_addr: $cal1, cal2_addr: $cal2,
            cal1_temp: $ct1, cal2_temp: $ct2,
        };

        pub type $type_name = $crate::adc_generic::GenericAdc<$ops>;

        pub fn new_adc() -> $type_name {
            $crate::adc_generic::GenericAdc::new($ops, &ADC_DMA_BUF, TEMP_CAL)
        }

        pub fn post_init() -> $type_name {
            $crate::adc_generic::GenericAdc::post_init($ops, &ADC_DMA_BUF, TEMP_CAL)
        }
    };
}
