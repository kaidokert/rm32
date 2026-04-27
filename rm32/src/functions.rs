//! Utility functions matching C functions.c

/// Linear interpolation with clamping. i64 intermediate prevents overflow.
pub fn map(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> i32 {
    if in_max == in_min {
        return out_min;
    }
    let lo = in_min.min(in_max);
    let hi = in_min.max(in_max);
    let x = if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    };
    ((x - in_min) as i64 * (out_max - out_min) as i64 / (in_max - in_min) as i64 + out_min as i64)
        as i32
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_basic() {
        assert_eq!(map(50, 0, 100, 0, 1000), 500);
        assert_eq!(map(0, 0, 100, 0, 1000), 0);
        assert_eq!(map(100, 0, 100, 0, 1000), 1000);
    }

    #[test]
    fn test_map_clamp() {
        assert_eq!(map(-10, 0, 100, 0, 1000), 0);
        assert_eq!(map(200, 0, 100, 0, 1000), 1000);
    }

    #[test]
    fn test_get_abs_dif() {
        assert_eq!(get_abs_dif(10, 3), 7);
        assert_eq!(get_abs_dif(3, 10), 7);
        assert_eq!(get_abs_dif(5, 5), 0);
    }

    #[test]
    fn test_crc8() {
        let data = [0x01, 0x02, 0x03, 0x04];
        let crc = get_crc8(&data);
        assert_eq!(crc, get_crc8(&data));
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_map_inverted_range() {
        // Inverted input range: in_min=100 > in_max=0
        // map(50, 100, 0, 0, 1000): 50 is midpoint → 500
        assert_eq!(map(50, 100, 0, 0, 1000), 500);
        // Below range: clamped to 0 → maps to out_max=1000
        assert_eq!(map(-5, 100, 0, 0, 1000), 1000);
        // At in_min=100 → maps to out_min=0
        assert_eq!(map(100, 100, 0, 0, 1000), 0);
    }

    #[test]
    fn test_map_same_output() {
        assert_eq!(map(50, 0, 100, 500, 500), 500);
    }

    #[test]
    fn test_crc8_empty() {
        assert_eq!(get_crc8(&[]), 0);
    }

    #[test]
    fn test_crc8_single_byte() {
        let crc = get_crc8(&[0xFF]);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_abs_dif_large() {
        assert_eq!(get_abs_dif(0, -1000), 1000);
        assert_eq!(get_abs_dif(-1000, 0), 1000);
    }
}
