//! USART2 telemetry TX via DMA for STM32G431 (KISS protocol).
//! USART2 on PB3 (AF7), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 3 (DMAMUX request 27 = USART2_TX).

use crate::telem_hal::UartPeripheral;
use crate::regs::InitError;
use crate::pac;

pub struct G431Uart;

impl UartPeripheral for G431Uart {
    fn enable_clocks(&self) {
        let rcc = unsafe { &*pac::RCC::PTR };
        unsafe {
            rcc.apb1enr1().modify(|_, w| w.usart2en().set_bit());
            rcc.ahb2enr().modify(|_, w| w.gpioben().set_bit());
            rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());
        }
    }

    fn configure_pin(&self) {
        let gpiob = unsafe { &*pac::GPIOB::PTR };
        unsafe {
            gpiob.moder().modify(|_, w| w.moder3().bits(0b10));
            gpiob.otyper().modify(|_, w| w.ot3().set_bit());
            gpiob.pupdr().modify(|_, w| w.pupdr3().bits(0b01));
            gpiob.afrl().modify(|_, w| w.afrl3().bits(7));
        }
    }

    fn configure_usart(&self) {
        let usart = unsafe { &*pac::USART2::PTR };
        unsafe {
            usart.cr1().write(|w| w.bits(0));
            usart.brr().write(|w| w.bits(1476)); // 170MHz / 115200
            usart.cr3().modify(|_, w| w.hdsel().set_bit());
            usart.cr1().write(|w| w.te().set_bit().ue().set_bit());
        }
    }

    fn wait_ready(&self) -> Result<(), InitError> {
        let usart = unsafe { &*pac::USART2::PTR };
        crate::regs::wait_for(|| unsafe { usart.isr().read().teack().bit() }, 100_000, "USART TEACK")
    }

    fn configure_dma_routing(&self) {
        let dmamux = unsafe { &*pac::DMAMUX::PTR };
        unsafe { dmamux.ccr(2).write(|w| w.dmareq_id().bits(27)); }
    }

    fn configure_dma_channel(&self) {
        let dma = unsafe { &*pac::DMA1::PTR };
        let usart = unsafe { &*pac::USART2::PTR };
        let ch3 = dma.ch3();
        unsafe {
            ch3.par().write(|w| w.bits(usart.tdr().as_ptr() as u32));
            ch3.mar().write(|w| w.bits(0));
            ch3.cr().write(|w| w.tcie().set_bit().teie().set_bit().dir().set_bit().minc().set_bit());
        }
    }

    fn send_dma_raw(&self, buf_ptr: *const u8, len: u16) {
        let dma = unsafe { &*pac::DMA1::PTR };
        let usart = unsafe { &*pac::USART2::PTR };
        let ch3 = dma.ch3();
        unsafe {
            ch3.cr().modify(|_, w| w.en().clear_bit());
            ch3.mar().write(|w| w.bits(buf_ptr as u32));
            ch3.ndtr().write(|w| w.bits(len as u32));
            usart.cr3().modify(|_, w| w.dmat().set_bit());
            ch3.cr().modify(|_, w| w.en().set_bit());
        }
    }
}

pub type G431TelemUart = crate::telem_hal::GenericTelemUart<G431Uart>;

impl G431TelemUart {
    pub fn init() -> Result<Self, InitError> { crate::telem_hal::GenericTelemUart::new_init(G431Uart) }
    pub fn post_init() -> Self { crate::telem_hal::GenericTelemUart::new_post_init(G431Uart) }
}
