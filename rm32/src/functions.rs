//! Utility functions matching C functions.c

/// AM32 `map()` (`Src/functions.c:22`): clamp, then bisect both ranges with
/// floored midpoints until the input interval is one step wide. This is not
/// the straight line `(x - in_min) * span / in_span + out_min`; the two differ
/// by a few counts on most inputs, so every caller must use this one.
///
/// AM32 recurses; the recursion is a tail call, so it is a loop here and
/// needs no division on any core. AM32's `in_min > in_max` swap branch is
/// unreachable (passing both clamps implies `in_min < x < in_max`), so a
/// reversed input range returns `out_max` for `x >= in_max`, else `out_min`.
/// Kept for AM32 parity; callers wanting a falling line reverse the *output*
/// range instead.
pub fn map(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i32 {
    let (mut in_min, mut in_max, mut out_min, mut out_max) = (in_min, in_max, out_min, out_max);
    loop {
        if x >= in_max {
            return out_max;
        }
        if x <= in_min {
            return out_min;
        }
        if out_min == out_max {
            return out_min;
        }
        let in_mid = (in_min + in_max) >> 1;
        let out_mid = (out_min + out_max) >> 1;
        if in_min == in_mid {
            return out_mid;
        }
        if x <= in_mid {
            in_max = in_mid;
            out_max = out_mid;
        } else {
            in_min = in_mid + 1;
            out_min = out_mid;
        }
    }
}

/// Absolute difference between two integers.
pub fn get_abs_dif(a: i32, b: i32) -> u32 {
    (a - b).unsigned_abs()
}

/// CRC-8 update (KISS telemetry polynomial).
pub fn update_crc8(crc: u8, seed: u8) -> u8 {
    let mut crc_u = crc ^ seed;
    for _ in 0..8 {
        crc_u = if crc_u & 0x80 != 0 {
            0x07 ^ (crc_u << 1)
        } else {
            crc_u << 1
        };
    }
    crc_u
}

/// CRC-8 over a buffer.
pub fn get_crc8(buf: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in buf {
        crc = update_crc8(b, crc);
    }
    crc
}

/// AM32 `delayMicros` (`Src/functions.c`) on any free-running down-counter:
/// return at the first read at which `us` microseconds have elapsed since
/// the first read. `read_down` returns the counter (`mask + 1` states,
/// `counts_per_us` counts per microsecond). Elapsed counts accumulate across
/// reads, so the counter may wrap any number of times as long as consecutive
/// reads are less than one counter period apart.
///
/// AM32 compares `(uint16_t)micros`, so a request of 65536 us or more in C
/// waits `micros % 65536`. This waits the full time; every AM32 and rm32
/// caller passes less than 65536.
pub fn delay_micros_on(us: u32, counts_per_us: u32, mask: u32, mut read_down: impl FnMut() -> u32) {
    let mut remaining = us.saturating_mul(counts_per_us);
    let mut prev = read_down();
    // As AM32's `while`, the exit test always follows at least one more read.
    loop {
        let now = read_down();
        remaining = remaining.saturating_sub(prev.wrapping_sub(now) & mask);
        prev = now;
        if remaining == 0 {
            return;
        }
    }
}

/// AM32 `delayMillis`: `millis` calls of a 1000 us delay.
pub fn delay_millis_on(ms: u32, mut delay_micros: impl FnMut(u32)) {
    for _ in 0..ms {
        delay_micros(1000);
    }
}

/// Literal transcriptions of AM32 `Src/functions.c`, kept as test oracles.
/// They are deliberately *not* tidied: recursion, `long` (i32 on ARM, widened
/// here so the oracle cannot overflow) and statement order follow the C.
#[cfg(test)]
pub(crate) mod am32_c {
    /// `long map(long x, long in_min, long in_max, long out_min, long out_max)`
    pub(crate) fn map(x: i64, in_min: i64, in_max: i64, out_min: i64, out_max: i64) -> i64 {
        if x >= in_max {
            return out_max;
        }
        if x <= in_min {
            return out_min;
        }
        if in_min > in_max {
            return map(x, in_max, in_min, out_max, out_min);
        }
        if out_min == out_max {
            return out_min;
        }
        let in_mid = (in_min + in_max) >> 1;
        let out_mid = (out_min + out_max) >> 1;
        if in_min == in_mid {
            return out_mid;
        }
        if x <= in_mid {
            map(x, in_min, in_mid, out_min, out_mid)
        } else {
            map(x, in_mid + 1, in_max, out_mid, out_max)
        }
    }

    /// `uint32_t getAbsDif(int number1, int number2)`
    pub(crate) fn get_abs_dif(number1: i32, number2: i32) -> u32 {
        let mut result = number1 as i64 - number2 as i64;
        if result < 0 {
            result = -result;
        }
        result as u32
    }

    /// `uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)`
    pub(crate) fn update_crc8(crc: u8, crc_seed: u8) -> u8 {
        let mut crc_u = crc;
        crc_u ^= crc_seed;
        for _ in 0..8 {
            crc_u = if crc_u & 0x80 != 0 {
                0x7 ^ (crc_u << 1)
            } else {
                crc_u << 1
            };
        }
        crc_u
    }

    /// `void delayMicros(uint32_t micros)`, with `get_timer_us16` supplied.
    pub(crate) fn delay_micros(micros: u32, mut get_timer_us16: impl FnMut() -> u16) {
        let cval_start = get_timer_us16();
        while (get_timer_us16().wrapping_sub(cval_start)) < (micros as u16) {}
    }

    /// `void delayMillis(uint32_t millis)`
    pub(crate) fn delay_millis(mut millis: u32, mut delay_micros: impl FnMut(u32)) {
        while millis > 0 {
            millis -= 1;
            delay_micros(1000);
        }
    }

    /// `uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen)`
    pub(crate) fn get_crc8(buf: &[u8], buf_len: u8) -> u8 {
        let mut crc = 0;
        for &b in &buf[..buf_len as usize] {
            crc = update_crc8(b, crc);
        }
        crc
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every `x` in `xs` for one parameter set, against the C transcription.
    fn check(site: &str, xs: core::ops::RangeInclusive<i32>, p: (i32, i32, i32, i32)) {
        let (a, b, c, d) = p;
        for x in xs {
            let want = am32_c::map(x as i64, a as i64, b as i64, c as i64, d as i64);
            assert_eq!(
                map(x, a, b, c, d) as i64,
                want,
                "{site}: map({x}, {a}, {b}, {c}, {d})"
            );
        }
    }

    // One test per AM32 `map()` call that rm32 makes; each sweeps every
    // input a few counts past both ends, over the configuration range the
    // parameters can take.

    #[test]
    fn map_matches_am32_reverse_speed_threshold() {
        // main.c:818 map(motor_kv, 300, 3000, 1000, 500); kv = byte*40+20
        check("main.c:818", 0..=10_300, (300, 3000, 1000, 500));
    }

    #[test]
    fn map_matches_am32_servo_dead_band_maps() {
        // main.c:1012/1050 and 1027/1064; servo_dead_band is a u8
        for db in 0..=255 {
            check(
                "main.c:1012",
                900..=2100,
                (1000 + (db << 1), 2000, 47, 2047),
            );
            check("main.c:1027", -10..=1100, (0, 1000 - (db << 1), 2047, 47));
        }
    }

    #[test]
    fn map_matches_am32_sine_start_input() {
        // main.c:1172/1176; changeover level is a u8 (x20)
        for lvl in 0..=255 {
            let co = lvl * 20;
            check("main.c:1172", 0..=co + 5, (30, co, 47, 160));
            check("main.c:1176", co - 5..=2060, (co, 2047, 160, 2047));
        }
    }

    #[test]
    fn map_matches_am32_duty_setpoint() {
        // main.c:1227/1229; minimum_duty_cycle (+dead time) over 0..=2000
        for min in 0..=2000 {
            check("main.c:1227", 130..=2060, (137, 2047, min + 40, 2000));
            check("main.c:1229", 40..=2060, (47, 2047, min, 2000));
        }
    }

    #[test]
    fn map_matches_am32_voltage_ramp() {
        // main.c:1892; battery_voltage in centivolts
        check("main.c:1892", 0..=6500, (800, 2200, 10, 1));
    }

    #[test]
    fn map_matches_am32_brushed_duty() {
        // main.c:2088 map(adjusted_input, 48, 2047, 0, max)
        for max in (0..=8000).step_by(7) {
            check("main.c:2088", 0..=2060, (48, 2047, 0, max));
        }
    }

    #[test]
    fn map_matches_am32_variable_pwm() {
        // main.c:2362 map(commutation_interval, 96, 200, ARR/2, ARR)
        for arr in (1000..=8000).step_by(3) {
            check("main.c:2362", 0..=400, (96, 200, arr / 2, arr));
        }
    }

    #[test]
    fn map_matches_am32_low_rpm_limit() {
        // main.c:2619; levels from motor_kv and motor_poles (main.c:815-816)
        for kv in (300..=10_220).step_by(40) {
            for poles in (2..=36).step_by(2) {
                let div = (32 / poles).max(1);
                let (lo, hi) = (kv / 100 / div, kv / 12 / div);
                check("main.c:2619", 0..=hi + 5, (lo, hi, 400, 2000));
            }
        }
    }

    #[test]
    fn map_matches_am32_temperature_limit() {
        // main.c:2628 / 2092; temperature limit is a u8
        for limit in 0..=255 {
            check("main.c:2628", -50..=300, (limit - 10, limit + 10, 1000, 1));
        }
    }

    #[test]
    fn map_matches_am32_filter_and_advance() {
        check("main.c:2634", 0..=70_000, (100, 500, 3, 12));
        check("main.c:2641", 0..=2100, (100, 2000, 13, 23));
    }

    #[test]
    fn map_matches_am32_sine_angle_and_step_delay() {
        check("main.c:2686", 900..=2100, (1000, 2000, 180, 360));
        check("main.c:2688", -10..=1100, (0, 1000, 0, 180));
        for poles in 1..=255 {
            check("main.c:2717", 0..=200, (48, 120, 7000 / poles, 810 / poles));
        }
    }

    #[test]
    fn map_matches_am32_signal_maps() {
        // signal.c:42 (one-shot) and signal.c:90-97 (servo thresholds)
        check("signal.c:42", 0..=1500, (243, 1200, 0, 2000));
        for low in (700..=1400).step_by(10) {
            for neutral in (1374..=1630).step_by(8) {
                check("signal.c:90", 0..=2600, (low, neutral, 0, 1000));
            }
        }
        for neutral in (1374..=1630).step_by(8) {
            for high in (1600..=2300).step_by(10) {
                check("signal.c:93", 0..=2600, (neutral + 1, high, 1001, 2000));
            }
        }
        for low in (700..=1400).step_by(10) {
            for high in (1600..=2300).step_by(10) {
                check("signal.c:97", 0..=2600, (low, high, 47, 2047));
            }
        }
    }

    #[test]
    fn map_matches_am32_on_random_wide_parameters() {
        // Includes reversed and degenerate ranges, which no call site uses.
        let mut s: u32 = 0x1234_5678;
        let mut r = |m: i32| {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            (s % (2 * m as u32 + 1)) as i32 - m
        };
        for _ in 0..200_000 {
            let (x, a, b, c, d) = (r(70_000), r(70_000), r(70_000), r(70_000), r(70_000));
            check("random", x..=x, (a, b, c, d));
        }
    }

    #[test]
    fn map_reversed_input_range_is_a_step_as_in_am32() {
        assert_eq!(map(500, 1047, 48, 0, 1000), 1000);
        assert_eq!(map(48, 1047, 48, 0, 1000), 1000);
        assert_eq!(map(47, 1047, 48, 0, 1000), 0);
    }

    #[test]
    fn map_endpoints() {
        assert_eq!(map(0, 0, 100, 0, 1000), 0);
        assert_eq!(map(100, 0, 100, 0, 1000), 1000);
        assert_eq!(map(-10, 0, 100, 0, 1000), 0);
        assert_eq!(map(200, 0, 100, 0, 1000), 1000);
        assert_eq!(map(50, 0, 100, 500, 500), 500);
    }

    #[test]
    fn get_abs_dif_matches_am32() {
        let edges = [
            i32::MIN / 2,
            -65_536,
            -1000,
            -1,
            0,
            1,
            1000,
            65_535,
            i32::MAX / 2,
        ];
        for &a in &edges {
            for &b in &edges {
                assert_eq!(get_abs_dif(a, b), am32_c::get_abs_dif(a, b));
            }
        }
        for a in (0..=65_535).step_by(97) {
            for b in (0..=65_535).step_by(89) {
                assert_eq!(get_abs_dif(a, b), am32_c::get_abs_dif(a, b));
            }
        }
    }

    #[test]
    fn update_crc8_matches_am32_exhaustively() {
        for crc in 0..=255u8 {
            for seed in 0..=255u8 {
                assert_eq!(update_crc8(crc, seed), am32_c::update_crc8(crc, seed));
            }
        }
    }

    #[test]
    fn get_crc8_matches_am32() {
        let buf: [u8; 255] = core::array::from_fn(|i| (i as u8).wrapping_mul(37) ^ 0x5A);
        for len in 0..=255u8 {
            assert_eq!(get_crc8(&buf[..len as usize]), am32_c::get_crc8(&buf, len));
        }
        assert_eq!(get_crc8(&[]), 0);
    }

    /// A shared clock read by both delays: each read advances it by a
    /// pseudo-random 0..=4 us, starting at `start_us`.
    fn exit_time(start_us: u32, seed: u32, wait: impl FnOnce(&mut dyn FnMut() -> u32)) -> u32 {
        let mut t = start_us;
        let mut s = seed | 1;
        let mut read = || {
            let now = t;
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            t = t.wrapping_add(s % 5);
            now
        };
        wait(&mut read);
        t
    }

    #[test]
    fn delay_micros_matches_am32_on_a_shared_clock() {
        const MHZ: u32 = 64;
        const SYST_MASK: u32 = 0x00FF_FFFF;
        let mut cases = 0u32;
        for us in (0..=3_000).chain((3_000..65_536).step_by(97)) {
            for (start, seed) in [(0, 1), (65_530, 7), (0x0003_FFF0, 99), (u32::MAX - 3, 5)] {
                let am32 = exit_time(start, seed, |read| {
                    am32_c::delay_micros(us, || read() as u16)
                });
                // SysTick: down-counter at MHZ counts per us, 24 bits.
                let rm32 = exit_time(start, seed, |read| {
                    delay_micros_on(us, MHZ, SYST_MASK, || {
                        0u32.wrapping_sub(read().wrapping_mul(MHZ)) & SYST_MASK
                    })
                });
                assert_eq!(rm32, am32, "us={us}, start={start}");
                cases += 1;
            }
        }
        assert!(cases > 10_000);
    }

    #[test]
    fn delay_millis_matches_am32() {
        for ms in 0..=300 {
            let (mut am32, mut rm32) = (0u32, 0u32);
            am32_c::delay_millis(ms, |us| am32 += us);
            delay_millis_on(ms, |us| rm32 += us);
            assert_eq!(rm32, am32, "ms={ms}");
        }
    }
}
