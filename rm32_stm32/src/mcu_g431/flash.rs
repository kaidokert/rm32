//! G431 flash: method-style PAC, double-word programming, page-number erase.

use crate::flash::FlashPeripheral;
use crate::pac;

macro_rules! flash_reg {
    () => {
        &*pac::FLASH::PTR
    };
}

pub struct G431Flash;

impl FlashPeripheral for G431Flash {
    const BSY_BIT: u32 = 1 << 16;
    const LOCK_BIT: u32 = 1 << 31;
    const PER_BIT: u32 = 1 << 1;
    const PG_BIT: u32 = 1 << 0;
    const STRT_BIT: u32 = 1 << 16;
    const PAGE_SIZE: u32 = 0x800;

    #[inline]
    fn read_sr() -> u32 {
        unsafe { flash_reg!() }.sr().read().bits()
    }
    #[inline]
    fn read_cr() -> u32 {
        unsafe { flash_reg!() }.cr().read().bits()
    }
    #[inline]
    fn write_keyr(val: u32) {
        unsafe {
            flash_reg!().keyr().write(|w| w.bits(val));
        }
    }
    #[inline]
    fn write_sr(val: u32) {
        unsafe {
            flash_reg!().sr().write(|w| w.bits(val));
        }
    }
    #[inline]
    fn write_cr(val: u32) {
        unsafe {
            flash_reg!().cr().write(|w| w.bits(val));
        }
    }
    #[inline]
    fn modify_cr(f: impl FnOnce(u32) -> u32) {
        let v = Self::read_cr();
        Self::write_cr(f(v));
    }

    fn erase_page_impl(address: u32, page_size: u32) {
        let page = address / page_size;
        Self::modify_cr(|v| (v & !(0x3F << 3)) | Self::PER_BIT | (page << 3));
        Self::modify_cr(|v| v | Self::STRT_BIT);
    }

    fn program_impl(address: u32, data: &[u8]) {
        let mut offset = 0u32;
        let mut i = 0;
        while i < data.len() {
            let mut word_lo = 0u32;
            let mut word_hi = 0u32;
            for b in 0..4 {
                if i + b < data.len() {
                    word_lo |= (data[i + b] as u32) << (b * 8);
                }
            }
            for b in 0..4 {
                if i + 4 + b < data.len() {
                    word_hi |= (data[i + 4 + b] as u32) << (b * 8);
                }
            }

            unsafe {
                Self::modify_cr(|v| v | Self::PG_BIT);
                core::ptr::write_volatile((address + offset) as *mut u32, word_lo);
                core::ptr::write_volatile((address + offset + 4) as *mut u32, word_hi);
            }
            Self::wait_bsy();
            Self::modify_cr(|v| v & !Self::PG_BIT);
            offset += 8;
            i += 8;
        }
    }
}

pub type FlashStorage = crate::flash::FlashStorage<G431Flash>;
