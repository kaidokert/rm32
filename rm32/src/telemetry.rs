//! KISS ESC telemetry packet generation.

use crate::functions::get_crc8;

/// KISS telemetry packet (10 bytes)
#[repr(C, packed)]
pub struct KissTelemPacket {
    pub temperature: i8,
    pub voltage_h: u8,
    pub voltage_l: u8,
    pub current_h: u8,
    pub current_l: u8,
    pub consumption_h: u8,
    pub consumption_l: u8,
    pub erpm_h: u8,
    pub erpm_l: u8,
    pub crc: u8,
}

/// Build a KISS telemetry packet into a 10-byte buffer.
pub fn make_telem_package(
    buf: &mut [u8; 10],
    temp: i8,
    voltage: u16,
    current: u16,
    consumption: u16,
    e_rpm: u16,
) {
    buf[0] = temp as u8;
    buf[1] = (voltage >> 8) as u8;
    buf[2] = voltage as u8;
    buf[3] = (current >> 8) as u8;
    buf[4] = current as u8;
    buf[5] = (consumption >> 8) as u8;
    buf[6] = consumption as u8;
    buf[7] = (e_rpm >> 8) as u8;
    buf[8] = e_rpm as u8;
    buf[9] = get_crc8(&buf[..9]);
}

/// Build ESC info packet (48 bytes of EEPROM + CRC).
pub fn make_info_packet(buf: &mut [u8; 49], eeprom: &[u8]) {
    buf[..48].copy_from_slice(&eeprom[..48]);
    buf[48] = get_crc8(&buf[..48]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telem_packet_encoding() {
        let mut buf = [0u8; 10];
        make_telem_package(&mut buf, 25, 1680, 500, 100, 3000);
        assert_eq!(buf[0], 25); // temp
        assert_eq!(buf[1], 0x06); // voltage_h
        assert_eq!(buf[2], 0x90); // voltage_l
        assert_eq!(buf[3], 0x01); // current_h
        assert_eq!(buf[4], 0xF4); // current_l
        assert_eq!(buf[5], 0x00); // consumption_h
        assert_eq!(buf[6], 0x64); // consumption_l
        assert_eq!(buf[7], 0x0B); // erpm_h
        assert_eq!(buf[8], 0xB8); // erpm_l
        assert_ne!(buf[9], 0); // CRC non-zero
    }

    #[test]
    fn telem_crc_consistent() {
        let mut buf1 = [0u8; 10];
        let mut buf2 = [0u8; 10];
        make_telem_package(&mut buf1, 30, 1200, 0, 0, 0);
        make_telem_package(&mut buf2, 30, 1200, 0, 0, 0);
        assert_eq!(buf1[9], buf2[9]);
    }

    #[test]
    fn info_packet_copies_eeprom_and_adds_crc() {
        let mut eeprom = [0xAAu8; 192];
        let mut buf = [0u8; 49];
        make_info_packet(&mut buf, &eeprom);
        for i in 0..48 {
            assert_eq!(buf[i], 0xAA);
        }
        assert_ne!(buf[48], 0); // CRC non-zero for 0xAA data
    }

    #[test]
    fn info_packet_crc_matches_data() {
        let eeprom = [0x01u8; 192];
        let mut buf = [0u8; 49];
        make_info_packet(&mut buf, &eeprom);
        let expected_crc = crate::functions::get_crc8(&buf[..48]);
        assert_eq!(buf[48], expected_crc);
    }
}
