//! DShot protocol decode/encode.
//!
//! - Frame decoding from DMA buffer (computeDshotDMA equivalent)
//! - GCR telemetry response encoding (make_dshot_package equivalent)
//! - Command processing

/// GCR encode table for DShot telemetry
const GCR_ENCODE_TABLE: [u8; 16] = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111, 0b11010, 0b01001,
    0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
];

/// DShot command numbers
pub mod commands {
    pub const BEACON_1: u16 = 1;
    pub const BEACON_5: u16 = 5;
    pub const ESC_INFO: u16 = 6;
    pub const DIRECTION_NORMAL: u16 = 7;
    pub const DIRECTION_REVERSED: u16 = 8;
    pub const BIDIR_OFF: u16 = 9;
    pub const BIDIR_ON: u16 = 10;
    pub const SAVE_SETTINGS: u16 = 12;
    pub const EDT_ENABLE: u16 = 13;
    pub const EDT_DISABLE: u16 = 14;
    pub const DIRECTION_FWD: u16 = 20;
    pub const DIRECTION_REV: u16 = 21;
    pub const PROGRAMMING_MODE: u16 = 36;
}

/// Result of decoding a DShot frame
#[derive(Debug, Clone, Copy)]
pub enum DshotFrame {
    /// Valid throttle value (0-2047) with telemetry request flag
    Throttle { value: u16, telemetry: bool },
    /// Valid command (1-47)
    Command { cmd: u16, telemetry: bool },
    /// CRC mismatch
    BadCrc,
    /// Frame timing outside valid window
    InvalidTiming,
}

/// Decode a DShot frame from a DMA capture buffer.
/// `dma_buffer` contains 32 entries (16 pulse pairs).
/// `frametime_low`/`frametime_high` define the valid window.
pub fn decode_frame(
    dma_buffer: &[u32; 32],
    frametime_low: u16,
    frametime_high: u16,
    bidirectional: bool,
) -> DshotFrame {
    let frametime = (dma_buffer[31].wrapping_sub(dma_buffer[0])) as u16;
    let halfpulsetime = frametime >> 5;

    if frametime <= frametime_low || frametime >= frametime_high {
        return DshotFrame::InvalidTiming;
    }

    let mut dpulse = [0u8; 16];
    for i in 0..16 {
        let pdiff = (dma_buffer[i * 2 + 1].wrapping_sub(dma_buffer[i * 2])) as u16;
        dpulse[i] = if pdiff > halfpulsetime { 1 } else { 0 };
    }

    let calc_crc = (dpulse[0] ^ dpulse[4] ^ dpulse[8]) << 3
        | (dpulse[1] ^ dpulse[5] ^ dpulse[9]) << 2
        | (dpulse[2] ^ dpulse[6] ^ dpulse[10]) << 1
        | (dpulse[3] ^ dpulse[7] ^ dpulse[11]);

    let mut check_crc = dpulse[12] << 3 | dpulse[13] << 2 | dpulse[14] << 1 | dpulse[15];
    if bidirectional {
        check_crc = (!check_crc).wrapping_add(16);
    }

    if calc_crc != check_crc {
        return DshotFrame::BadCrc;
    }

    let value = (dpulse[0] as u16) << 10
        | (dpulse[1] as u16) << 9
        | (dpulse[2] as u16) << 8
        | (dpulse[3] as u16) << 7
        | (dpulse[4] as u16) << 6
        | (dpulse[5] as u16) << 5
        | (dpulse[6] as u16) << 4
        | (dpulse[7] as u16) << 3
        | (dpulse[8] as u16) << 2
        | (dpulse[9] as u16) << 1
        | (dpulse[10] as u16);

    let telemetry = dpulse[11] == 1;

    if value > 47 {
        DshotFrame::Throttle { value, telemetry }
    } else if value > 0 {
        DshotFrame::Command {
            cmd: value,
            telemetry,
        }
    } else {
        DshotFrame::Throttle {
            value: 0,
            telemetry,
        }
    }
}

/// GCR output bit shift — MCU-dependent.
/// F051/F031/CH32V203: 6 (multiply by 64)
/// All others (G071, G431, L431, F421, etc.): 7 (multiply by 128)
pub const GCR_SHIFT_F0: u8 = 6;
pub const GCR_SHIFT_G0: u8 = 7;

/// Encode a raw 12-bit value into GCR buffer for bidir DShot response.
/// Used for both eRPM and EDT frames.
pub fn encode_gcr_frame(value_12bit: u16, gcr_out: &mut [u32], padding: usize, gcr_shift: u8) {
    // Calculate checksum (XOR of nibbles, inverted)
    let mut csum = 0u16;
    let mut csum_data = value_12bit;
    for _ in 0..3 {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum = !csum & 0xF;

    let full_number = (value_12bit << 4) | csum;

    // GCR RLL encode 16 to 20 bit
    let gcr_number: u32 = (GCR_ENCODE_TABLE[(full_number >> 12) as usize] as u32) << 15
        | (GCR_ENCODE_TABLE[((full_number >> 8) & 0xF) as usize] as u32) << 10
        | (GCR_ENCODE_TABLE[((full_number >> 4) & 0xF) as usize] as u32) << 5
        | (GCR_ENCODE_TABLE[(full_number & 0xF) as usize] as u32);

    // GCR RLL encode 20 to 21 bit
    let multiplier = 1u32 << gcr_shift;
    gcr_out[padding] = 0;
    gcr_out[1 + padding] = multiplier;
    for i in (0..20).rev() {
        let bit = ((gcr_number >> i) & 1) ^ (gcr_out[padding + 20 - i] >> gcr_shift);
        gcr_out[padding + 20 - i + 1] = bit << gcr_shift;
    }
}

/// Convert commutation time to 12-bit eRPM value (shift + mantissa).
pub fn erpm_to_12bit(com_time: u16, running: bool) -> u16 {
    let period = if !running { 65535u16 } else { com_time };

    let mut shift_amount = 0u8;
    for i in (9..=15).rev() {
        if period >> i == 1 {
            shift_amount = (i + 1 - 9) as u8;
            break;
        }
    }

    ((shift_amount as u16) << 9) | (period >> shift_amount)
}

/// Encode an eRPM telemetry response into a GCR buffer.
/// `gcr_shift` is 6 for F051-like MCUs, 7 for G071-like MCUs.
pub fn encode_telemetry_with_shift(
    com_time: u16,
    running: bool,
    gcr_out: &mut [u32],
    padding: usize,
    gcr_shift: u8,
) {
    let value = erpm_to_12bit(com_time, running);
    encode_gcr_frame(value, gcr_out, padding, gcr_shift);
}

/// Convenience wrapper using F051-compatible shift (for backward compat with tests).
pub fn encode_telemetry(com_time: u16, running: bool, gcr_out: &mut [u32], padding: usize) {
    encode_telemetry_with_shift(com_time, running, gcr_out, padding, GCR_SHIFT_F0);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a DShot DMA buffer from an 11-bit value.
    fn build_frame(value: u16, telem: bool, inverted_crc: bool) -> [u32; 32] {
        let mut bits = [0u8; 16];
        for i in 0..11 {
            bits[i] = ((value >> (10 - i)) & 1) as u8;
        }
        bits[11] = if telem { 1 } else { 0 };

        let mut crc = (bits[0] ^ bits[4] ^ bits[8]) << 3
            | (bits[1] ^ bits[5] ^ bits[9]) << 2
            | (bits[2] ^ bits[6] ^ bits[10]) << 1
            | (bits[3] ^ bits[7] ^ bits[11]);
        if inverted_crc {
            crc = (!crc) & 0xF;
        }
        bits[12] = (crc >> 3) & 1;
        bits[13] = (crc >> 2) & 1;
        bits[14] = (crc >> 1) & 1;
        bits[15] = crc & 1;

        let mut buf = [0u32; 32];
        let mut base = 1000u32;
        for i in 0..16 {
            buf[i * 2] = base;
            buf[i * 2 + 1] = base + if bits[i] != 0 { 22 } else { 10 };
            base += 32;
        }
        buf
    }

    #[test]
    fn decode_throttle() {
        let buf = build_frame(100, false, false);
        match decode_frame(&buf, 400, 600, false) {
            DshotFrame::Throttle { value, telemetry } => {
                assert_eq!(value, 100);
                assert!(!telemetry);
            }
            other => panic!("Expected Throttle, got {:?}", other),
        }
    }

    #[test]
    fn decode_with_telemetry_bit() {
        let buf = build_frame(200, true, false);
        match decode_frame(&buf, 400, 600, false) {
            DshotFrame::Throttle { value, telemetry } => {
                assert_eq!(value, 200);
                assert!(telemetry);
            }
            other => panic!("Expected Throttle, got {:?}", other),
        }
    }

    #[test]
    fn decode_bad_crc() {
        let mut buf = build_frame(100, false, false);
        buf[31] = buf[30] + 22; // corrupt last pulse
        assert!(matches!(
            decode_frame(&buf, 400, 600, false),
            DshotFrame::BadCrc
        ));
    }

    #[test]
    fn decode_zero_throttle() {
        let buf = build_frame(0, false, false);
        match decode_frame(&buf, 400, 600, false) {
            DshotFrame::Throttle { value, .. } => assert_eq!(value, 0),
            other => panic!("Expected Throttle(0), got {:?}", other),
        }
    }

    #[test]
    fn decode_command() {
        let buf = build_frame(7, false, false);
        match decode_frame(&buf, 400, 600, false) {
            DshotFrame::Command { cmd, .. } => assert_eq!(cmd, 7),
            other => panic!("Expected Command, got {:?}", other),
        }
    }

    #[test]
    fn decode_invalid_timing() {
        let buf = build_frame(100, false, false);
        assert!(matches!(
            decode_frame(&buf, 50, 100, false),
            DshotFrame::InvalidTiming
        ));
    }

    #[test]
    fn decode_bidirectional_inverted_crc() {
        let buf = build_frame(300, false, true);
        match decode_frame(&buf, 400, 600, true) {
            DshotFrame::Throttle { value, .. } => assert_eq!(value, 300),
            other => panic!("Expected Throttle(300), got {:?}", other),
        }
    }

    #[test]
    fn encode_not_running_max_period() {
        let mut gcr = [0u32; 37];
        encode_telemetry(500, false, &mut gcr, 0);
        // Not running: period forced to 65535
        // shift_amount for 65535: bit 15 is set -> shift=7
        // dshot_number = (7<<9) | (65535>>7) = 3584|511 = 4095 = 0xFFF
    }

    #[test]
    fn encode_deterministic() {
        let mut gcr1 = [0u32; 37];
        let mut gcr2 = [0u32; 37];
        encode_telemetry(1000, true, &mut gcr1, 0);
        encode_telemetry(1000, true, &mut gcr2, 0);
        assert_eq!(gcr1, gcr2);
    }

    #[test]
    fn encode_different_periods() {
        let mut gcr1 = [0u32; 37];
        let mut gcr2 = [0u32; 37];
        encode_telemetry(200, true, &mut gcr1, 0);
        encode_telemetry(2000, true, &mut gcr2, 0);
        assert_ne!(gcr1, gcr2);
    }

    #[test]
    fn encode_has_data() {
        let mut gcr = [0u32; 37];
        encode_telemetry(500, true, &mut gcr, 0);
        assert!(gcr.iter().any(|&v| v != 0));
    }

    #[test]
    fn encode_shift_small_period() {
        let mut gcr = [0u32; 37];
        encode_telemetry(100, true, &mut gcr, 0);
        // Small period: shift_amount should be 0
    }

    #[test]
    fn encode_shift_large_period() {
        let mut gcr = [0u32; 37];
        encode_telemetry(30000, true, &mut gcr, 0);
        // Large period: nonzero shift
    }

    // --- Golden vector tests: verified against C firmware (MCU_F051, gcr_shift=6, padding=7) ---

    /// Helper: encode with C harness parameters (F051 path, padding=7).
    fn encode_c_compat(com_time: u16) -> [u32; 37] {
        let mut gcr = [0u32; 37];
        encode_telemetry_with_shift(com_time, true, &mut gcr, 7, GCR_SHIFT_F0);
        gcr
    }

    #[test]
    fn gcr_golden_com500_shift0() {
        let gcr = encode_c_compat(500);
        assert_eq!(erpm_to_12bit(500, true), 500); // shift=0, raw value
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 0, 64, 64, 0, 64, 64, 0, 64, 0, 64, 0, 64, 0, 0, 64, 0,
                0, 64, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com1000_shift1() {
        let gcr = encode_c_compat(1000);
        assert_eq!(erpm_to_12bit(1000, true), 1012); // shift=1
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 0, 0, 0, 64, 0, 0, 64, 0, 64, 0, 64, 0, 64, 64, 0, 64,
                64, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com65535_shift7() {
        let gcr = encode_c_compat(65535);
        assert_eq!(erpm_to_12bit(65535, true), 4095); // shift=7, max
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 64, 0, 64, 0, 64, 64, 0, 64, 0, 64, 64, 0, 64, 0, 64,
                0, 64, 64, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com100_shift0() {
        let gcr = encode_c_compat(100);
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 0, 64, 64, 64, 0, 64, 64, 0, 64, 64, 0, 64, 0, 0, 64,
                64, 0, 64, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com30000_shift6() {
        let gcr = encode_c_compat(30000);
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 64, 0, 64, 64, 0, 0, 64, 0, 0, 64, 0, 64, 0, 0, 64, 64,
                0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com200_shift0() {
        let gcr = encode_c_compat(200);
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 0, 64, 64, 64, 0, 64, 0, 64, 0, 0, 64, 0, 0, 64, 64,
                64, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com10000_shift5() {
        let gcr = encode_c_compat(10000);
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 64, 0, 0, 64, 0, 64, 64, 64, 0, 64, 0, 64, 64, 0, 0, 0,
                64, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_com1_shift0() {
        let gcr = encode_c_compat(1);
        assert_eq!(
            gcr,
            [
                0, 0, 0, 0, 0, 0, 0, 0, 64, 0, 64, 64, 64, 0, 64, 0, 0, 0, 64, 0, 64, 64, 0, 64,
                64, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn gcr_golden_not_running() {
        // Not running forces com_time to 65535 regardless of input
        let mut gcr = [0u32; 37];
        encode_telemetry_with_shift(500, false, &mut gcr, 7, GCR_SHIFT_F0);
        let expected = encode_c_compat(65535); // same as max
        assert_eq!(gcr, expected);
    }
}
