//! Flash EEPROM read/write for STM32G071.
//!
//! Settings are stored in the last flash page (2KB).
//! STM32G0 flash: 2KB pages, double-word (8-byte) programming.

use rm32::hal::Flash;
use crate::pac;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

// Bit constants differ between MCU families
#[cfg(feature = "stm32g071")]
mod regs {
    pub const BSY_BIT: u32 = 1 << 16; // BSY1
    pub const LOCK_BIT: u32 = 1 << 31;
    pub const PER_BIT: u32 = 1 << 1;
    pub const PG_BIT: u32 = 1 << 0;
    pub const STRT_BIT: u32 = 1 << 16;
    pub const PAGE_SIZE: u32 = 0x800;
}

#[cfg(feature = "stm32f051")]
mod regs {
    pub const BSY_BIT: u32 = 1 << 0; // BSY
    pub const LOCK_BIT: u32 = 1 << 7;
    pub const PER_BIT: u32 = 1 << 1;
    pub const PG_BIT: u32 = 1 << 0;
    pub const STRT_BIT: u32 = 1 << 6;
    pub const PAGE_SIZE: u32 = 0x400; // 1KB for F051
}

#[cfg(any(feature = "stm32l431", feature = "stm32g431"))]
mod regs {
    pub const BSY_BIT: u32 = 1 << 16; // BSY
    pub const LOCK_BIT: u32 = 1 << 31;
    pub const PER_BIT: u32 = 1 << 1;
    pub const PG_BIT: u32 = 1 << 0;
    pub const STRT_BIT: u32 = 1 << 16;
    pub const PAGE_SIZE: u32 = 0x800; // 2KB for L431/G431
}

/// PAC-based flash register access. Bridges method vs field accessor styles.
macro_rules! flash_reg {
    () => { unsafe { &*pac::FLASH::PTR } };
}

// G071/G431: method accessors — flash.keyr(), flash.sr(), flash.cr()
#[cfg(any(feature = "stm32g071", feature = "stm32g431"))]
mod pac_flash {
    use super::*;
    #[inline(always)]
    pub unsafe fn read_sr() -> u32 { flash_reg!().sr().read().bits() }
    #[inline(always)]
    pub unsafe fn read_cr() -> u32 { flash_reg!().cr().read().bits() }
    #[inline(always)]
    pub unsafe fn write_keyr(val: u32) { flash_reg!().keyr().write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn write_sr(val: u32) { flash_reg!().sr().write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn write_cr(val: u32) { flash_reg!().cr().write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn modify_cr(f: impl FnOnce(u32) -> u32) {
        let v = read_cr();
        write_cr(f(v));
    }
}

// F051: field accessors — flash.keyr, flash.sr, flash.cr, flash.ar
#[cfg(feature = "stm32f051")]
mod pac_flash {
    use super::*;
    #[inline(always)]
    pub unsafe fn read_sr() -> u32 { flash_reg!().sr.read().bits() }
    #[inline(always)]
    pub unsafe fn read_cr() -> u32 { flash_reg!().cr.read().bits() }
    #[inline(always)]
    pub unsafe fn write_keyr(val: u32) { flash_reg!().keyr.write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn write_sr(val: u32) { flash_reg!().sr.write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn write_cr(val: u32) { flash_reg!().cr.write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn modify_cr(f: impl FnOnce(u32) -> u32) {
        let v = read_cr();
        write_cr(f(v));
    }
    #[inline(always)]
    pub unsafe fn write_ar(val: u32) { flash_reg!().ar.write(|w| w.bits(val)); }
}

// L431: field accessors — flash.keyr, flash.sr, flash.cr
#[cfg(feature = "stm32l431")]
mod pac_flash {
    use super::*;
    #[inline(always)]
    pub unsafe fn read_sr() -> u32 { flash_reg!().sr.read().bits() }
    #[inline(always)]
    pub unsafe fn read_cr() -> u32 { flash_reg!().cr.read().bits() }
    #[inline(always)]
    pub unsafe fn write_keyr(val: u32) { flash_reg!().keyr.write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn write_sr(val: u32) { flash_reg!().sr.write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn write_cr(val: u32) { flash_reg!().cr.write(|w| w.bits(val)); }
    #[inline(always)]
    pub unsafe fn modify_cr(f: impl FnOnce(u32) -> u32) {
        let v = read_cr();
        write_cr(f(v));
    }
}

pub struct FlashStorage {
    _private: (),
}

impl Default for FlashStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashStorage {
    pub fn new() -> Self {
        Self { _private: () }
    }

    fn wait_bsy(&self) {
        // Timeout after ~50ms at 64MHz (flash erase is typically <40ms)
        let mut timeout = 500_000u32;
        while unsafe { pac_flash::read_sr() } & regs::BSY_BIT != 0 {
            timeout -= 1;
            if timeout == 0 { break; } // prevent infinite hang on flash failure
        }
    }

    fn unlock(&self) {
        self.wait_bsy();
        if unsafe { pac_flash::read_cr() } & regs::LOCK_BIT != 0 {
            unsafe {
                pac_flash::write_keyr(FLASH_KEY1);
                pac_flash::write_keyr(FLASH_KEY2);
            }
        }
    }

    fn lock(&self) {
        unsafe { pac_flash::modify_cr(|v| v | regs::LOCK_BIT); }
    }

    fn erase_page(&self, address: u32) {
        #[cfg(any(feature = "stm32g071", feature = "stm32l431"))]
        {
            let page = address / regs::PAGE_SIZE;
            unsafe {
                pac_flash::modify_cr(|v| (v & !(0x3F << 3)) | regs::PER_BIT | (page << 3));
                pac_flash::modify_cr(|v| v | regs::STRT_BIT);
            }
        }
        #[cfg(feature = "stm32f051")]
        {
            unsafe {
                pac_flash::modify_cr(|v| v | regs::PER_BIT);
                pac_flash::write_ar(address);
                pac_flash::modify_cr(|v| v | regs::STRT_BIT);
            }
        }
        self.wait_bsy();
        if unsafe { pac_flash::read_sr() } & (1 << 5) != 0 { // EOP
            unsafe { pac_flash::write_sr(1 << 5); }
        }
        unsafe { pac_flash::modify_cr(|v| v & !regs::PER_BIT); }
    }
}

impl Flash for FlashStorage {
    fn read(&self, address: u32, buf: &mut [u8]) {
        // Flash is memory-mapped — just read directly
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = unsafe { *((address + i as u32) as *const u8) };
        }
    }

    fn write(&mut self, address: u32, data: &[u8]) {
        self.unlock();

        if address.is_multiple_of(regs::PAGE_SIZE) {
            self.erase_page(address);
        }

        // G0: double-word (8 bytes) programming
        // F0: half-word (2 bytes) programming
        let mut offset = 0u32;
        let mut i = 0;

        #[cfg(any(feature = "stm32g071", feature = "stm32l431"))]
        while i < data.len() {
            let mut word_lo = 0u32;
            let mut word_hi = 0u32;
            for b in 0..4 { if i + b < data.len() { word_lo |= (data[i + b] as u32) << (b * 8); } }
            for b in 0..4 { if i + 4 + b < data.len() { word_hi |= (data[i + 4 + b] as u32) << (b * 8); } }

            unsafe {
                pac_flash::modify_cr(|v| v | regs::PG_BIT);
                core::ptr::write_volatile((address + offset) as *mut u32, word_lo);
                core::ptr::write_volatile((address + offset + 4) as *mut u32, word_hi);
            }
            self.wait_bsy();
            unsafe { pac_flash::modify_cr(|v| v & !regs::PG_BIT); }
            offset += 8;
            i += 8;
        }

        #[cfg(feature = "stm32f051")]
        while i < data.len() {
            let hw = if i + 1 < data.len() {
                data[i] as u16 | ((data[i + 1] as u16) << 8)
            } else {
                data[i] as u16 | 0xFF00
            };

            unsafe {
                pac_flash::modify_cr(|v| v | regs::PG_BIT);
                core::ptr::write_volatile((address + offset) as *mut u16, hw);
            }
            self.wait_bsy();
            unsafe { pac_flash::modify_cr(|v| v & !regs::PG_BIT); }
            offset += 2;
            i += 2;
        }

        self.lock();
    }
}
