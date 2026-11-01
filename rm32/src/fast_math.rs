//! Exact constant-divisor arithmetic for motor-control paths.
//!
//! AM32 writes these as plain `/` on every core; on Cortex-M0, which has no
//! divide instruction, the compiler turns them into library division calls.
//! These helpers return Rust's truncating division result for every value of
//! their argument type using only shifts, adds and small multiplies, so every
//! chip runs the same code with the same result.

/// Exact signed division by three, truncating toward zero, for every `i32`.
#[inline(always)]
pub(crate) const fn div3_i32(n: i32) -> i32 {
    if n < 0 {
        -(div3_u32(n.unsigned_abs()) as i32)
    } else {
        div3_u32(n as u32) as i32
    }
}

/// Exact unsigned division by three for every `u32`.
#[inline(never)]
const fn div3_u32(n: u32) -> u32 {
    let mut q = (n >> 2) + (n >> 4);
    q += q >> 4;
    q += q >> 8;
    q += q >> 16;
    let r = n - q * 3;
    q + ((11 * r) >> 5)
}

/// Exact unsigned division by five for every `u32` (Hacker's Delight
/// `divu5`): the shift series underestimates by at most a few, and the
/// remainder correction `7r >> 5` adds the missing quotient.
#[inline(never)]
const fn div5_u32(n: u32) -> u32 {
    let mut q = (n >> 1) + (n >> 2);
    q += q >> 4;
    q += q >> 8;
    q += q >> 16;
    q >>= 2;
    let r = n - q * 5;
    q + ((7 * r) >> 5)
}

/// Exact unsigned division by ten for every `u32`:
/// `floor(floor(n / 5) / 2) == floor(n / 10)`.
#[inline(always)]
pub(crate) const fn div10_u32(n: u32) -> u32 {
    div5_u32(n) >> 1
}

/// Exact unsigned division by [`crate::constants::DUTY_SCALE_MAX`] (2000) for
/// every `u32`: `n / 2000 == ((n >> 4) / 5 / 5 / 5)`, each step a floor of a
/// floor. Branch-free, so the cost is the same on every tick.
#[inline(always)]
pub(crate) const fn div2000_u32(n: u32) -> u32 {
    div5_u32(div5_u32(div5_u32(n >> 4)))
}

const _: () = assert!(crate::constants::DUTY_SCALE_MAX == 2000);

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    /// Run `f` on every `u32`, split across threads; a debug-build sweep of
    /// 2^32 values on one thread takes about a minute.
    fn every_u32(f: fn(u32)) {
        let threads = std::thread::available_parallelism().map_or(8, |n| n.get()) as u64;
        let span = (1u64 << 32).div_ceil(threads);
        std::thread::scope(|s| {
            for t in 0..threads {
                s.spawn(move || {
                    let lo = t * span;
                    let hi = ((t + 1) * span).min(1 << 32);
                    for n in lo..hi {
                        f(n as u32);
                    }
                });
            }
        });
    }

    #[test]
    fn div3_matches_division_for_every_u32_and_i32() {
        every_u32(|n| {
            assert_eq!(div3_u32(n), n / 3, "n={n}");
            let i = n as i32;
            assert_eq!(div3_i32(i), i / 3, "i={i}");
        });
    }

    #[test]
    fn div5_and_div10_match_division_for_every_u32() {
        every_u32(|n| {
            assert_eq!(div5_u32(n), n / 5, "n={n}");
            assert_eq!(div10_u32(n), n / 10, "n={n}");
        });
    }

    #[test]
    fn div2000_matches_division_for_every_u32() {
        every_u32(|n| assert_eq!(div2000_u32(n), n / 2000, "n={n}"));
    }
}
