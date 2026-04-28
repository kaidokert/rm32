//! F051 flash: field-style PAC, half-word programming, address-register erase.

use crate::flash::FlashPeripheral;
use crate::pac;

macro_rules! flash_reg {
    () => {
        &*pac::FLASH::PTR
    };
}

pub struct F051Flash;

impl F051Flash {
    #[inline]
    fn write_ar(val: u32) {
        unsafe {
            flash_reg!().ar.write(|w| w.bits(val));
        }
    }
}

impl FlashPeripheral for F051Flash {
    const BSY_BIT: u32 = 1 << 0;
    const LOCK_BIT: u32 = 1 << 7;
    const PER_BIT: u32 = 1 << 1;
    const PG_BIT: u32 = 1 << 0;
    const STRT_BIT: u32 = 1 << 6;
    const PAGE_SIZE: u32 = 0x400;

    #[inline]
    fn read_sr() -> u32 {
        unsafe { flash_reg!() }.sr.read().bits()
    }
    #[inline]
    fn read_cr() -> u32 {
        unsafe { flash_reg!() }.cr.read().bits()
    }
    #[inline]
    fn write_keyr(val: u32) {
        unsafe {
            flash_reg!().keyr.write(|w| w.bits(val));
        }
    }
    #[inline]
    fn write_sr(val: u32) {
        unsafe {
            flash_reg!().sr.write(|w| w.bits(val));
        }
    }
    #[inline]
    fn write_cr(val: u32) {
        unsafe {
            flash_reg!().cr.write(|w| w.bits(val));
        }
    }
    #[inline]
    fn modify_cr(f: impl FnOnce(u32) -> u32) {
        let v = Self::read_cr();
        Self::write_cr(f(v));
    }

    fn erase_page_impl(address: u32, _page_size: u32) {
        Self::modify_cr(|v| v | Self::PER_BIT);
        Self::write_ar(address);
        Self::modify_cr(|v| v | Self::STRT_BIT);
    }

    fn program_impl(address: u32, data: &[u8]) {
        let mut offset = 0u32;
        let mut i = 0;
        while i < data.len() {
            let hw = if i + 1 < data.len() {
                data[i] as u16 | ((data[i + 1] as u16) << 8)
            } else {
                data[i] as u16 | 0xFF00
            };

            unsafe {
                Self::modify_cr(|v| v | Self::PG_BIT);
                core::ptr::write_volatile((address + offset) as *mut u16, hw);
            }
            Self::wait_bsy();
            Self::modify_cr(|v| v & !Self::PG_BIT);
            offset += 2;
            i += 2;
        }
    }
}

pub type FlashStorage = crate::flash::FlashStorage<F051Flash>;
