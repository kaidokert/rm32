//! CRSF (Crossfire) serial input protocol parser.
//!
//! CRSF is a serial protocol at 420kbaud used by some receivers
//! to send channel data to ESCs/flight controllers.
//!
//! Packet format:
//!   [0xC8] [length] [type] [payload...] [crc8]
//!
//! Channel data (type 0x16):
//!   22 bytes payload containing 16 × 11-bit channel values.
//!   Packed LSB-first across byte boundaries.
//!   Value range: 172 (min) to 1811 (max), center at 992.

/// CRSF sync byte
pub const CRSF_SYNC: u8 = 0xC8;

/// CRSF frame type: RC channels packed
pub const CRSF_FRAMETYPE_RC_CHANNELS: u8 = 0x16;

/// Maximum CRSF frame size (sync + length + type + 22 payload + crc)
pub const CRSF_MAX_FRAME_SIZE: usize = 26;

/// CRSF channel value range
pub const CRSF_CHANNEL_MIN: u16 = 172;
pub const CRSF_CHANNEL_MAX: u16 = 1811;
pub const CRSF_CHANNEL_CENTER: u16 = 992;

/// CRC-8/DVB-S2 lookup table (polynomial 0xD5)
const CRC8_TABLE: [u8; 256] = {
    let mut table = [0u8; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = i as u8;
        let mut j = 0;
        while j < 8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
            j += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

/// Compute CRC-8/DVB-S2 over a byte slice.
pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc = CRC8_TABLE[(crc ^ b) as usize];
    }
    crc
}

/// Result of parsing a CRSF frame.
#[derive(Debug)]
pub enum CrsfResult {
    /// Valid channel data — 16 channels, 11-bit each
    Channels([u16; 16]),
    /// Valid frame but not a channel data type
    OtherFrame(u8),
    /// CRC mismatch
    BadCrc,
    /// Not enough data or invalid sync
    Incomplete,
}

/// CRSF frame parser with internal byte buffer.
pub struct CrsfParser {
    buf: [u8; 64],
    pos: usize,
    /// Which channel index to use as throttle (default: 2)
    pub throttle_channel: u8,
}

impl CrsfParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; 64],
            pos: 0,
            throttle_channel: 2,
        }
    }

    /// Feed a byte into the parser. Returns Some(result) when a complete frame is available.
    pub fn feed(&mut self, byte: u8) -> Option<CrsfResult> {
        // Looking for sync byte
        if self.pos == 0 {
            if byte == CRSF_SYNC {
                self.buf[0] = byte;
                self.pos = 1;
            }
            return None;
        }

        self.buf[self.pos] = byte;
        self.pos += 1;

        // Have sync + length?
        if self.pos < 3 {
            return None;
        }

        let frame_len = self.buf[1] as usize; // length includes type + payload + crc
        let total_len = 2 + frame_len; // sync + length byte + frame_len

        // Sanity check
        if frame_len < 2 || total_len > 64 {
            self.pos = 0;
            return Some(CrsfResult::Incomplete);
        }

        // Wait for complete frame
        if self.pos < total_len {
            return None;
        }

        // Frame complete — validate CRC
        // CRC covers type + payload (bytes 2..total_len-1)
        let crc_idx = total_len - 1;
        let computed_crc = crc8_dvb_s2(&self.buf[2..crc_idx]);
        let received_crc = self.buf[crc_idx];

        self.pos = 0; // reset for next frame

        if computed_crc != received_crc {
            return Some(CrsfResult::BadCrc);
        }

        let frame_type = self.buf[2];
        if frame_type != CRSF_FRAMETYPE_RC_CHANNELS {
            return Some(CrsfResult::OtherFrame(frame_type));
        }

        // Parse 16 × 11-bit channels from 22 bytes (payload starts at byte 3)
        let payload = &self.buf[3..3 + 22];
        let channels = unpack_channels(payload);
        Some(CrsfResult::Channels(channels))
    }

    /// Map a CRSF channel value (172-1811) to ESC throttle (0-2047).
    pub fn channel_to_throttle(value: u16) -> u16 {
        if value <= CRSF_CHANNEL_MIN {
            return 0;
        }
        if value >= CRSF_CHANNEL_MAX {
            return 2047;
        }
        let range = (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) as u32;
        ((value - CRSF_CHANNEL_MIN) as u32 * 2047 / range) as u16
    }
}

/// Unpack 16 × 11-bit channels from 22 bytes (LSB-first bit packing).
fn unpack_channels(payload: &[u8]) -> [u16; 16] {
    let mut channels = [0u16; 16];
    // 11 bits per channel, packed LSB-first across byte boundaries
    // Channel 0: bits [0..10] of payload
    // Channel 1: bits [11..21]
    // etc.
    channels[0]  = ((payload[0] as u16)       | ((payload[1] as u16) << 8)) & 0x7FF;
    channels[1]  = ((payload[1] as u16) >> 3  | ((payload[2] as u16) << 5)) & 0x7FF;
    channels[2]  = ((payload[2] as u16) >> 6  | ((payload[3] as u16) << 2) | ((payload[4] as u16) << 10)) & 0x7FF;
    channels[3]  = ((payload[4] as u16) >> 1  | ((payload[5] as u16) << 7)) & 0x7FF;
    channels[4]  = ((payload[5] as u16) >> 4  | ((payload[6] as u16) << 4)) & 0x7FF;
    channels[5]  = ((payload[6] as u16) >> 7  | ((payload[7] as u16) << 1) | ((payload[8] as u16) << 9)) & 0x7FF;
    channels[6]  = ((payload[8] as u16) >> 2  | ((payload[9] as u16) << 6)) & 0x7FF;
    channels[7]  = ((payload[9] as u16) >> 5  | ((payload[10] as u16) << 3)) & 0x7FF;
    channels[8]  = ((payload[11] as u16)      | ((payload[12] as u16) << 8)) & 0x7FF;
    channels[9]  = ((payload[12] as u16) >> 3 | ((payload[13] as u16) << 5)) & 0x7FF;
    channels[10] = ((payload[13] as u16) >> 6 | ((payload[14] as u16) << 2) | ((payload[15] as u16) << 10)) & 0x7FF;
    channels[11] = ((payload[15] as u16) >> 1 | ((payload[16] as u16) << 7)) & 0x7FF;
    channels[12] = ((payload[16] as u16) >> 4 | ((payload[17] as u16) << 4)) & 0x7FF;
    channels[13] = ((payload[17] as u16) >> 7 | ((payload[18] as u16) << 1) | ((payload[19] as u16) << 9)) & 0x7FF;
    channels[14] = ((payload[19] as u16) >> 2 | ((payload[20] as u16) << 6)) & 0x7FF;
    channels[15] = ((payload[20] as u16) >> 5 | ((payload[21] as u16) << 3)) & 0x7FF;
    channels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_empty() {
        assert_eq!(crc8_dvb_s2(&[]), 0);
    }

    #[test]
    fn crc8_known_value() {
        // CRC-8/DVB-S2 of [0x16] (channel frame type byte)
        let crc = crc8_dvb_s2(&[0x16]);
        assert_ne!(crc, 0); // just verify it computes something
    }

    #[test]
    fn unpack_all_zeros() {
        let payload = [0u8; 22];
        let ch = unpack_channels(&payload);
        for &c in &ch {
            assert_eq!(c, 0);
        }
    }

    #[test]
    fn unpack_all_ones() {
        let payload = [0xFFu8; 22];
        let ch = unpack_channels(&payload);
        for &c in &ch {
            assert_eq!(c, 0x7FF); // 11-bit max = 2047
        }
    }

    #[test]
    fn unpack_channel_0_only() {
        let mut payload = [0u8; 22];
        // Channel 0 = 1000 (0x3E8) in bits [0..10]
        // 0x3E8 = 0b01111101000
        payload[0] = 0xE8; // lower 8 bits
        payload[1] = 0x03; // upper 3 bits
        let ch = unpack_channels(&payload);
        assert_eq!(ch[0], 1000);
        assert_eq!(ch[1], 0); // no bleed
    }

    #[test]
    fn channel_to_throttle_range() {
        assert_eq!(CrsfParser::channel_to_throttle(172), 0);
        assert_eq!(CrsfParser::channel_to_throttle(1811), 2047);
        assert_eq!(CrsfParser::channel_to_throttle(100), 0); // below min
        assert_eq!(CrsfParser::channel_to_throttle(2000), 2047); // above max
        // Mid-range check
        let mid = CrsfParser::channel_to_throttle(992);
        assert!(mid > 900 && mid < 1100, "mid={}", mid);
    }

    #[test]
    fn parser_valid_frame() {
        let mut parser = CrsfParser::new();

        // Build a valid CRSF RC channels frame
        let mut frame = [0u8; 26];
        frame[0] = CRSF_SYNC;
        frame[1] = 24; // length: type(1) + payload(22) + crc(1)
        frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
        // payload bytes 3..24: all channels at center (992 = 0x3E0)
        // For simplicity, just use zeros (all channels = 0)
        // CRC over bytes 2..24 (type + payload)
        frame[25] = crc8_dvb_s2(&frame[2..25]);

        // Feed bytes one at a time
        for i in 0..25 {
            assert!(parser.feed(frame[i]).is_none());
        }
        match parser.feed(frame[25]) {
            Some(CrsfResult::Channels(ch)) => {
                assert_eq!(ch[0], 0);
            }
            other => panic!("expected Channels, got {:?}", other),
        }
    }

    #[test]
    fn parser_bad_crc() {
        let mut parser = CrsfParser::new();

        let mut frame = [0u8; 26];
        frame[0] = CRSF_SYNC;
        frame[1] = 24;
        frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
        frame[25] = 0xFF; // wrong CRC

        for i in 0..25 {
            parser.feed(frame[i]);
        }
        match parser.feed(frame[25]) {
            Some(CrsfResult::BadCrc) => {}
            other => panic!("expected BadCrc, got {:?}", other),
        }
    }

    #[test]
    fn parser_resync_on_garbage() {
        let mut parser = CrsfParser::new();

        // Feed garbage then a valid frame
        for b in [0x00, 0x55, 0xAA, 0x12] {
            assert!(parser.feed(b).is_none());
        }

        // Now a valid frame
        let mut frame = [0u8; 26];
        frame[0] = CRSF_SYNC;
        frame[1] = 24;
        frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
        frame[25] = crc8_dvb_s2(&frame[2..25]);

        for &b in &frame {
            let _ = parser.feed(b);
        }
        // Parser should have synced and returned a result
    }
}
