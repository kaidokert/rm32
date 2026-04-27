//! Safe wrapper for DMA buffers that must be `'static`.
//!
//! DMA hardware writes to a fixed memory address. The buffer must:
//! 1. Have a stable `'static` address (no moving after DMA is configured)
//! 2. Not be accessed while DMA is active
//!
//! This wrapper provides safe `as_ptr()` for DMA configuration and
//! safe read access after DMA completes.

use core::cell::UnsafeCell;

/// A DMA-safe static buffer. Wraps a fixed-size array with interior mutability.
///
/// # Safety contract
/// - DMA hardware writes to this buffer via the raw pointer from `as_ptr()`
/// - Software reads via `as_slice()` only when DMA is not active (between transfers)
/// - This is safe on single-core Cortex-M where ISR and main don't overlap
#[repr(align(4))]
pub struct DmaBuf<T, const N: usize>(UnsafeCell<[T; N]>);

// SAFETY: Single-core Cortex-M. DMA writes and software reads are sequenced by the
// DMA Transfer Complete interrupt — software only reads after DMA has finished writing.
// No concurrent access occurs.
unsafe impl<T, const N: usize> Sync for DmaBuf<T, N> {}

impl<T: Copy + Default, const N: usize> Default for DmaBuf<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Default, const N: usize> DmaBuf<T, N> {
    pub const fn new() -> Self {
        // SAFETY: T is Copy+Default (u16/u32), so zeroed memory is a valid value.
        // core::mem::zeroed is used instead of Default::default() because Default isn't const.
        Self(UnsafeCell::new(unsafe { core::mem::zeroed() }))
    }

    /// Raw pointer for DMA peripheral MAR register.
    pub fn as_ptr(&self) -> *const T {
        self.0.get() as *const T
    }

    /// Read buffer contents. Only call when DMA is not actively writing.
    pub fn read(&self) -> &[T; N] {
        // SAFETY: Caller ensures DMA is not writing (called after TC interrupt).
        unsafe { &*self.0.get() }
    }
}
