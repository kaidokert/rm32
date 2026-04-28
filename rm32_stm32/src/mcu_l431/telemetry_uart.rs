//! USART1 telemetry TX via DMA for STM32L431 (KISS protocol).
//! USART1 on PB6 (AF7), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 4 (request 2).

use crate::pac::{DMA1, GPIOB, RCC, USART1};
use crate::regs::InitError;
use crate::telem_hal::UartPeripheral;

pub struct L431Uart;

impl UartPeripheral for L431Uart {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
        rcc.ahb2enr.modify(|_, w| w.gpioben().set_bit());
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
    }

    fn configure_pin(&self) {
        let gpiob = unsafe { &*GPIOB::ptr() };
        unsafe {
            gpiob.moder.modify(|_, w| w.moder6().bits(0b10));
            gpiob.otyper.modify(|_, w| w.ot6().set_bit());
            gpiob.pupdr.modify(|_, w| w.pupdr6().bits(0b01));
            gpiob.afrl.modify(|_, w| w.afrl6().bits(7));
        }
    }

    fn configure_usart(&self) {
        let usart = unsafe { &*USART1::ptr() };
        unsafe {
            usart.cr1.write(|w| w.bits(0));
            usart.brr.write(|w| w.bits(694)); // 80MHz / 115200
            usart.cr3.modify(|_, w| w.hdsel().set_bit());
            usart
                .cr1
                .write(|w| w.te().set_bit().re().set_bit().ue().set_bit());
        }
    }

    fn wait_ready(&self) -> Result<(), InitError> {
        let usart = unsafe { &*USART1::ptr() };
        crate::regs::wait_for(
            || usart.isr.read().teack().bit_is_set(),
            100_000,
            "USART TEACK",
        )?;
        crate::regs::wait_for(
            || usart.isr.read().reack().bit_is_set(),
            100_000,
            "USART REACK",
        )
    }

    fn configure_dma_routing(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        // CSELR: CH4 request = 2 (USART1_TX)
        dma.cselr
            .modify(|r, w| unsafe { w.bits((r.bits() & !(0xF << 12)) | (2 << 12)) });
    }

    fn configure_dma_channel(&self) {
        let dma = unsafe { &*DMA1::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        unsafe {
            dma.cpar4.write(|w| w.bits(usart.tdr.as_ptr() as u32));
            dma.cmar4.write(|w| w.bits(0));
            dma.ccr4.write(|w| {
                w.bits(
                    (1 << 1) | (1 << 3) | (1 << 4) | (1 << 7), // TCIE, TEIE, DIR, MINC
                )
            });
        }
    }

    fn send_dma_raw(&self, buf_ptr: *const u8, len: u16) {
        let dma = unsafe { &*DMA1::ptr() };
        let usart = unsafe { &*USART1::ptr() };
        unsafe {
            dma.ccr4.modify(|r, w| w.bits(r.bits() & !1));
            dma.cmar4.write(|w| w.bits(buf_ptr as u32));
            dma.cndtr4.write(|w| w.bits(len as u32));
            usart.cr3.modify(|_, w| w.dmat().set_bit());
            dma.ccr4.modify(|r, w| w.bits(r.bits() | 1));
        }
    }
}

pub type L431TelemUart = crate::telem_hal::GenericTelemUart<L431Uart>;

impl L431TelemUart {
    pub fn init() -> Result<Self, InitError> {
        crate::telem_hal::GenericTelemUart::new_init(L431Uart)
    }
    pub fn post_init() -> Self {
        crate::telem_hal::GenericTelemUart::new_post_init(L431Uart)
    }
}
