//! USART1 telemetry TX via DMA for STM32F051 (KISS protocol).
//! USART1 on PB6 (AF0), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 2 (fixed assignment on F0).

use crate::pac::{DMA1, GPIOB, RCC, USART1};
use crate::telem_hal::UartPeripheral;
use crate::regs::InitError;

pub struct F051Uart;

impl UartPeripheral for F051Uart {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        unsafe {
            rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
            rcc.ahbenr.modify(|_, w| w.dmaen().set_bit().iopben().set_bit());
        }
    }

    fn configure_pin(&self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        gpiob.moder.modify(|_, w| w.moder6().alternate());
        gpiob.otyper.modify(|_, w| w.ot6().open_drain());
        gpiob.pupdr.modify(|_, w| w.pupdr6().pull_up());
        gpiob.afrl.modify(|_, w| w.afrl6().af0());
    }

    fn configure_usart(&self) {
        let usart = unsafe { &*USART1::ptr() };
        usart.cr1.write(|w| unsafe { w.bits(0) });
        usart.brr.write(|w| unsafe { w.bits(417) }); // 48MHz / 115200
        usart.cr3.write(|w| w.hdsel().set_bit());
        usart.cr1.write(|w| w.te().set_bit().re().set_bit().ue().set_bit());
    }

    fn wait_ready(&self) -> Result<(), InitError> {
        let usart = unsafe { &*USART1::ptr() };
        crate::regs::wait_for(|| usart.isr.read().teack().bit_is_set(), 100_000, "USART TEACK")?;
        crate::regs::wait_for(|| usart.isr.read().reack().bit_is_set(), 100_000, "USART REACK")
    }

    fn configure_dma_routing(&self) {
        // F051: fixed DMA channel assignment, no routing needed
    }

    fn configure_dma_channel(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        dma.ch2.par.write(|w| unsafe { w.bits(usart.tdr.as_ptr() as u32) });
        dma.ch2.mar.write(|w| unsafe { w.bits(0) });
        dma.ch2.cr.write(|w| {
            w.tcie().enabled().teie().enabled().dir().from_memory().minc().enabled()
        });
    }

    fn send_dma_raw(&self, buf_ptr: *const u8, len: u16) {
        let dma = unsafe { &*DMA1::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        dma.ch2.cr.modify(|_, w| w.en().clear_bit());
        dma.ch2.mar.write(|w| unsafe { w.bits(buf_ptr as u32) });
        dma.ch2.ndtr.write(|w| unsafe { w.bits(len as u32) });
        usart.cr3.modify(|_, w| w.dmat().set_bit());
        dma.ch2.cr.modify(|_, w| w.en().set_bit());
    }
}

pub type F051TelemUart = crate::telem_hal::GenericTelemUart<F051Uart>;

impl F051TelemUart {
    pub fn init() -> Result<Self, InitError> { crate::telem_hal::GenericTelemUart::new_init(F051Uart) }
    pub fn post_init() -> Self { crate::telem_hal::GenericTelemUart::new_post_init(F051Uart) }
}
