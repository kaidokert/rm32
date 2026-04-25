//! Flash EEPROM read/write for STM32G071.
//!
//! Settings are stored in the last flash page (2KB).
//! STM32G0 flash: 2KB pages, double-word (8-byte) programming.

use rm32::hal::Flash;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

// Flash register base and offsets differ between F0 and G0
#[cfg(feature = "stm32g071")]
mod regs {
    pub const BASE: u32 = 0x4002_2000;
    pub const KEYR: u32 = BASE + 0x08;
    pub const SR: u32 = BASE + 0x10;
    pub const CR: u32 = BASE + 0x14;
    pub const BSY_BIT: u32 = 1 << 16; // BSY1
    pub const LOCK_BIT: u32 = 1 << 31;
    pub const PER_BIT: u32 = 1 << 1;
    pub const PG_BIT: u32 = 1 << 0;
    pub const STRT_BIT: u32 = 1 << 16;
    pub const PAGE_SIZE: u32 = 0x800;
}

#[cfg(feature = "stm32f051")]
mod regs {
    pub const BASE: u32 = 0x4002_2000;
    pub const KEYR: u32 = BASE + 0x04;
    pub const SR: u32 = BASE + 0x0C;
    pub const CR: u32 = BASE + 0x10;
    pub const BSY_BIT: u32 = 1 << 0; // BSY
    pub const LOCK_BIT: u32 = 1 << 7;
    pub const PER_BIT: u32 = 1 << 1;
    pub const PG_BIT: u32 = 1 << 0;
    pub const STRT_BIT: u32 = 1 << 6;
    pub const PAGE_SIZE: u32 = 0x400; // 1KB for F051
}

#[cfg(feature = "stm32l431")]
mod regs {
    pub const BASE: u32 = 0x4002_2000;
    pub const KEYR: u32 = BASE + 0x08;
    pub const SR: u32 = BASE + 0x10;
    pub const CR: u32 = BASE + 0x14;
    pub const BSY_BIT: u32 = 1 << 16; // BSY
    pub const LOCK_BIT: u32 = 1 << 31;
    pub const PER_BIT: u32 = 1 << 1;
    pub const PG_BIT: u32 = 1 << 0;
    pub const STRT_BIT: u32 = 1 << 16;
    pub const PAGE_SIZE: u32 = 0x800; // 2KB for L431
}

pub struct FlashStorage {
    _private: (),
}

impl FlashStorage {
    pub fn new() -> Self {
        Self { _private: () }
    }

    #[inline(always)]
    unsafe fn read_reg(addr: u32) -> u32 { (addr as *const u32).read_volatile() }
    #[inline(always)]
    unsafe fn write_reg(addr: u32, val: u32) { (addr as *mut u32).write_volatile(val); }
    #[inline(always)]
    unsafe fn modify_reg(addr: u32, f: impl FnOnce(u32) -> u32) {
        let ptr = addr as *mut u32;
        ptr.write_volatile(f(ptr.read_volatile()));
    }

    fn wait_bsy(&self) {
        // Timeout after ~50ms at 64MHz (flash erase is typically <40ms)
        let mut timeout = 500_000u32;
        while unsafe { Self::read_reg(regs::SR) } & regs::BSY_BIT != 0 {
            timeout -= 1;
            if timeout == 0 { break; } // prevent infinite hang on flash failure
        }
    }

    fn unlock(&self) {
        self.wait_bsy();
        if unsafe { Self::read_reg(regs::CR) } & regs::LOCK_BIT != 0 {
            unsafe {
                Self::write_reg(regs::KEYR, FLASH_KEY1);
                Self::write_reg(regs::KEYR, FLASH_KEY2);
            }
        }
    }

    fn lock(&self) {
        unsafe { Self::modify_reg(regs::CR, |v| v | regs::LOCK_BIT); }
    }

    fn erase_page(&self, address: u32) {
        #[cfg(any(feature = "stm32g071", feature = "stm32l431"))]
        {
            let page = address / regs::PAGE_SIZE;
            unsafe {
                Self::modify_reg(regs::CR, |v| (v & !(0x3F << 3)) | regs::PER_BIT | (page << 3));
                Self::modify_reg(regs::CR, |v| v | regs::STRT_BIT);
            }
        }
        #[cfg(feature = "stm32f051")]
        {
            unsafe {
                Self::modify_reg(regs::CR, |v| v | regs::PER_BIT);
                // F0: write the address to AR register (offset 0x14)
                Self::write_reg(regs::BASE + 0x14, address);
                Self::modify_reg(regs::CR, |v| v | regs::STRT_BIT);
            }
        }
        self.wait_bsy();
        if unsafe { Self::read_reg(regs::SR) } & (1 << 5) != 0 { // EOP
            unsafe { Self::write_reg(regs::SR, 1 << 5); }
        }
        unsafe { Self::modify_reg(regs::CR, |v| v & !regs::PER_BIT); }
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

        if address % regs::PAGE_SIZE == 0 {
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
                Self::modify_reg(regs::CR, |v| v | regs::PG_BIT);
                core::ptr::write_volatile((address + offset) as *mut u32, word_lo);
                core::ptr::write_volatile((address + offset + 4) as *mut u32, word_hi);
            }
            self.wait_bsy();
            unsafe { Self::modify_reg(regs::CR, |v| v & !regs::PG_BIT); }
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
                Self::modify_reg(regs::CR, |v| v | regs::PG_BIT);
                core::ptr::write_volatile((address + offset) as *mut u16, hw);
            }
            self.wait_bsy();
            unsafe { Self::modify_reg(regs::CR, |v| v & !regs::PG_BIT); }
            offset += 2;
            i += 2;
        }

        self.lock();
    }
}
