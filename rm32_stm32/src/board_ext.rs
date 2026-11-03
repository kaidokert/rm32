//! STM32-only board extensions emitted alongside `BOARD` by `build.rs`.
//!
//! Keeps `rm32::board::BoardConfig` untouched for boards that carry STM32
//! comparator register encodings or external gate-driver control pins
//! (`gate_enable_pin`, `nfault_pin`).

/// GPIO pin encoded as `(port_index, pin_number)`: `PA0` = `(0, 0)`,
/// `PD1` = `(3, 1)`, etc.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PinRef {
    /// 0 = GPIOA, 1 = GPIOB, 2 = GPIOC, 3 = GPIOD, 4 = GPIOE, 5 = GPIOF.
    pub port: u8,
    /// 0..=15.
    pub pin: u8,
}

/// BEMF comparator pin mapping — STM32-specific packed register values.
///
/// Each field is a pre-computed value that the per-MCU `set_inmsel()` writes
/// to the comparator register(s). The encoding is MCU-specific:
/// - L431: `(inmesel << 8) | inmsel_3bit` — set_inmsel unpacks both fields
/// - G071: 4-bit INMSEL value (set_inmsel shifts left by 4)
/// - F051: full lower-16-bit CSR value including COMP1_EN
/// - G431: `(config << 16) | comp_selector` for dual-comp routing
///
/// Values are computed by `rm32_stm32/build.rs` from symbolic pin names in
/// the board YAML. Lives in `rm32_stm32` because `rm32` never reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BemfPins {
    pub phase_a: u32,
    pub phase_b: u32,
    pub phase_c: u32,
}

/// STM32-only board fields from the board YAML. All optional fields default
/// to `None` on standard AM32 boards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoardExt {
    /// Gate-driver ENABLE output: held low until the ESC is armed.
    pub gate_enable: Option<PinRef>,
    /// Gate-driver nFAULT (open-drain, active low); input with pull-up.
    pub nfault: Option<PinRef>,
    /// BEMF comparator pin mapping (from board YAML).
    pub bemf_pins: BemfPins,
}

#[cfg(feature = "stm32g071")]
mod gpio {
    use super::PinRef;

    const GPIO_BASE: u32 = 0x5000_0000;
    const GPIO_STRIDE: u32 = 0x400;
    const RCC_IOPENR: u32 = 0x4002_1000 + 0x34;

    #[inline]
    fn reg(p: PinRef, off: u32) -> *mut u32 {
        (GPIO_BASE + p.port as u32 * GPIO_STRIDE + off) as *mut u32
    }

    fn enable_port_clock(p: PinRef) {
        // SAFETY: RCC_IOPENR read-modify-write of this port's enable bit;
        // boot-time only.
        unsafe {
            let r = RCC_IOPENR as *mut u32;
            r.write_volatile(r.read_volatile() | (1 << p.port));
        }
    }

    fn set_mode(p: PinRef, mode: u32) {
        let sh = p.pin as u32 * 2;
        // SAFETY: MODER RMW of this pin's field only.
        unsafe {
            let m = reg(p, 0x00);
            m.write_volatile((m.read_volatile() & !(0b11 << sh)) | (mode << sh));
        }
    }

    pub fn init_output_low(p: PinRef) {
        enable_port_clock(p);
        write(p, false);
        set_mode(p, 0b01);
    }

    pub fn init_input_pullup(p: PinRef) {
        enable_port_clock(p);
        let sh = p.pin as u32 * 2;
        // SAFETY: PUPDR RMW of this pin's field only.
        unsafe {
            let r = reg(p, 0x0C);
            r.write_volatile((r.read_volatile() & !(0b11 << sh)) | (0b01 << sh));
        }
        set_mode(p, 0b00);
    }

    #[inline]
    pub fn write(p: PinRef, high: bool) {
        let bit = if high {
            1u32 << p.pin
        } else {
            1u32 << (p.pin + 16)
        };
        // SAFETY: BSRR is write-only, atomic per bit.
        unsafe { reg(p, 0x18).write_volatile(bit) };
    }

    #[inline]
    pub fn read(p: PinRef) -> bool {
        // SAFETY: IDR read.
        unsafe { reg(p, 0x10).read_volatile() & (1 << p.pin) != 0 }
    }

    #[inline]
    pub fn read_odr(p: PinRef) -> bool {
        // SAFETY: ODR read.
        unsafe { reg(p, 0x14).read_volatile() & (1 << p.pin) != 0 }
    }
}

#[cfg(feature = "stm32g071")]
impl BoardExt {
    /// Boot: gate driver disabled (ENABLE low), nFAULT input with pull-up so
    /// a floating line can never read "healthy".
    pub fn init_pins(&self) {
        if let Some(p) = self.gate_enable {
            gpio::init_output_low(p);
        }
        if let Some(p) = self.nfault {
            gpio::init_input_pullup(p);
        }
    }

    /// Gate-driver ENABLE level. No-op on boards without the pin.
    #[inline]
    pub fn set_gate_enable(&self, on: bool) {
        if let Some(p) = self.gate_enable {
            gpio::write(p, on);
        }
    }

    #[inline]
    pub fn gate_enabled(&self) -> bool {
        self.gate_enable.map(gpio::read_odr).unwrap_or(true)
    }

    /// True while the gate driver's open-drain `nFAULT` pin is low.
    ///
    /// Note: gate drivers such as DRV8304H hold `nFAULT` low while `ENABLE`
    /// is low (sleep mode) and for ~1–5 ms after `ENABLE` rises while the
    /// charge pump wakes up, and dropping `ENABLE` clears a latched fault.
    /// Callers that act on `fault()` must apply wake-up blanking and latch the
    /// trip rather than feeding `fault()` directly back into `drive_gate`.
    #[inline]
    pub fn fault(&self) -> bool {
        self.nfault.map(|p| !gpio::read(p)).unwrap_or(false)
    }

    /// Main-loop gate policy: TIM1 main output enable (MOE) and the
    /// gate-driver ENABLE follow the armed state, so a disarmed ESC cannot
    /// drive the bridge even if a stray GPIO/AF write lands. Call every
    /// main-loop pass; writes only on a change.
    #[inline]
    pub fn drive_gate(&self, armed: bool) {
        use crate::mcu_g071::pwm;
        if self.gate_enable.is_none() {
            return; // AM32 boards: MOE set at init, no driver enable pin
        }
        if armed == self.gate_enabled() && armed == pwm::moe() {
            return;
        }
        cortex_m::interrupt::free(|_| {
            let armed = armed && crate::isr::shared().armed();
            if armed {
                self.set_gate_enable(true);
                pwm::set_moe(true);
            } else {
                pwm::set_moe(false);
                self.set_gate_enable(false);
            }
        });
    }
}

#[cfg(not(feature = "stm32g071"))]
impl BoardExt {
    pub fn init_pins(&self) {
        assert!(
            self.gate_enable.is_none() && self.nfault.is_none(),
            "gate_enable_pin / nfault_pin are implemented for G071 only"
        );
    }
    #[inline]
    pub fn set_gate_enable(&self, _on: bool) {}
    #[inline]
    pub fn gate_enabled(&self) -> bool {
        true
    }
    #[inline]
    pub fn fault(&self) -> bool {
        false
    }
    /// Other MCUs keep their AM32-matched init-time MOE.
    #[inline]
    pub fn drive_gate(&self, _armed: bool) {}
}
