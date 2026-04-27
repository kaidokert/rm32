//! USART2 telemetry TX via DMA for STM32G431 (KISS protocol).
//!
//! USART2 on PB3 (AF7), half-duplex open-drain, 115200 baud.
//! DMA1 Channel 3 (DMAMUX request 27 = USART2_TX).

use rm32::hal::TelemetryUart;
use stm32g4::stm32g431 as pac;

pub struct G431TelemUart { tx_buf: [u8; 49] }

impl G431TelemUart {
    pub fn post_init() -> Self { Self { tx_buf: [0; 49] } }

    pub fn init() -> Result<Self, crate::regs::InitError> {
        let rcc = unsafe { &*pac::RCC::PTR };
        let gpiob = unsafe { &*pac::GPIOB::PTR };
        let usart = unsafe { &*pac::USART2::PTR };
        let dma = unsafe { &*pac::DMA1::PTR };
        let dmamux = unsafe { &*pac::DMAMUX::PTR };

        unsafe {
            // Enable clocks
            rcc.apb1enr1().modify(|_, w| w.usart2en().set_bit());
            rcc.ahb2enr().modify(|_, w| w.gpioben().set_bit());
            rcc.ahb1enr().modify(|_, w| w.dma1en().set_bit());

            // PB3: AF7 (USART2_TX), open-drain, pull-up
            gpiob.moder().modify(|_, w| w.moder3().bits(0b10));
            gpiob.otyper().modify(|_, w| w.ot3().set_bit());
            gpiob.pupdr().modify(|_, w| w.pupdr3().bits(0b01));
            gpiob.afrl().modify(|_, w| w.afrl3().bits(7));

            // USART2: disable, configure, enable
            usart.cr1().write(|w| w.bits(0));
            usart.brr().write(|w| w.bits(1476)); // 170MHz / 115200
            usart.cr3().modify(|_, w| w.hdsel().set_bit());
            usart.cr1().write(|w| w.te().set_bit().ue().set_bit());

            // Wait TEACK
            crate::regs::wait_for(|| {
                usart.isr().read().teack().bit()
            }, 100_000, "USART TEACK")?;

            // DMAMUX: CH2 (DMA CH3) → USART2_TX (27)
            dmamux.ccr(2).write(|w| w.dmareq_id().bits(27));

            // DMA CH3: memory→periph, 8-bit, MINC, TCIE
            let ch3 = dma.ch3();
            ch3.par().write(|w| w.bits(usart.tdr().as_ptr() as u32));
            ch3.mar().write(|w| w.bits(0));
            ch3.cr().write(|w| {
                w.tcie().set_bit()
                 .teie().set_bit()
                 .dir().set_bit() // memory-to-periph
                 .minc().set_bit()
            });
        }
        Ok(Self { tx_buf: [0; 49] })
    }
}

impl TelemetryUart for G431TelemUart {
    fn send_dma(&mut self, data: &[u8]) {
        let len = data.len().min(49);
        let dma = unsafe { &*pac::DMA1::PTR };
        let usart = unsafe { &*pac::USART2::PTR };
        let ch3 = dma.ch3();

        self.tx_buf[..len].copy_from_slice(&data[..len]);
        unsafe {
            ch3.cr().modify(|_, w| w.en().clear_bit());
            ch3.mar().write(|w| w.bits(self.tx_buf.as_ptr() as u32));
            ch3.ndtr().write(|w| w.bits(len as u32));
            usart.cr3().modify(|_, w| w.dmat().set_bit());
            ch3.cr().modify(|_, w| w.en().set_bit());
        }
    }
}
