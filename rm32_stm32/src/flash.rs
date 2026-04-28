//! Flash EEPROM read/write — generic over MCU family.
//!
//! Settings are stored in the last flash page.
//! MCU-specific register access and programming strategies are provided
//! by the `FlashPeripheral` trait, implemented in each `mcu_xxx/flash.rs`.

use rm32::hal::Flash;

pub const FLASH_KEY1: u32 = 0x4567_0123;
pub const FLASH_KEY2: u32 = 0xCDEF_89AB;

/// MCU-specific flash register operations and programming strategy.
pub trait FlashPeripheral {
    const BSY_BIT: u32;
    const LOCK_BIT: u32;
    const PER_BIT: u32;
    const PG_BIT: u32;
    const STRT_BIT: u32;
    const PAGE_SIZE: u32;

    unsafe fn read_sr() -> u32;
    unsafe fn read_cr() -> u32;
    unsafe fn write_keyr(val: u32);
    unsafe fn write_sr(val: u32);
    unsafe fn write_cr(val: u32);
    unsafe fn modify_cr(f: impl FnOnce(u32) -> u32);

    /// Wait for BSY flag to clear with timeout.
    fn wait_bsy() {
        let mut timeout = 500_000u32;
        while unsafe { Self::read_sr() } & Self::BSY_BIT != 0 {
            timeout -= 1;
            if timeout == 0 { break; }
        }
    }

    /// Erase a page. Different strategy per MCU family.
    /// Must set PER + page select + STRT in CR (and write AR for F0).
    /// Common post-erase cleanup (wait, EOP, clear PER) is handled by the caller.
    fn erase_page_impl(address: u32, page_size: u32);

    /// Program data. Different word size per MCU family.
    /// Handles the full programming loop including PG_BIT and wait_bsy per word.
    fn program_impl(address: u32, data: &[u8]);
}

pub struct FlashStorage<F: FlashPeripheral> {
    _ops: core::marker::PhantomData<F>,
}

impl<F: FlashPeripheral> Default for FlashStorage<F> {
    fn default() -> Self {
        Self::new()
    }
}

impl<F: FlashPeripheral> FlashStorage<F> {
    pub fn new() -> Self {
        Self { _ops: core::marker::PhantomData }
    }

    fn unlock(&self) {
        F::wait_bsy();
        if unsafe { F::read_cr() } & F::LOCK_BIT != 0 {
            unsafe {
                F::write_keyr(FLASH_KEY1);
                F::write_keyr(FLASH_KEY2);
            }
        }
    }

    fn lock(&self) {
        unsafe { F::modify_cr(|v| v | F::LOCK_BIT); }
    }

    fn erase_page(&self, address: u32) {
        F::erase_page_impl(address, F::PAGE_SIZE);
        F::wait_bsy();
        if unsafe { F::read_sr() } & (1 << 5) != 0 { // EOP
            unsafe { F::write_sr(1 << 5); }
        }
        unsafe { F::modify_cr(|v| v & !F::PER_BIT); }
    }
}

impl<F: FlashPeripheral> Flash for FlashStorage<F> {
    fn read(&self, address: u32, buf: &mut [u8]) {
        // Flash is memory-mapped — just read directly
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = unsafe { *((address + i as u32) as *const u8) };
        }
    }

    fn write(&mut self, address: u32, data: &[u8]) {
        self.unlock();

        if address.is_multiple_of(F::PAGE_SIZE) {
            self.erase_page(address);
        }

        F::program_impl(address, data);

        self.lock();
    }
}

