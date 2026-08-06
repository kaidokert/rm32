//! Portable reset-cause bitflags.
//!
//! Each MCU's `read_and_clear_reset_cause()` impl translates its native
//! RCC reset-flag register into this set, so callers (e.g. the firmware
//! main loop) can print or react to reset reasons without #[cfg]-gated
//! peripheral access in shared code.
//!
//! Not every chip has every flag. F0-series doesn't have BOR (just POR,
//! which we map to `BROWNOUT` as the closest equivalent). G0-series
//! doesn't have FIREWALL. L4/G4 have everything.

/// Set of reset-cause flags reported by an MCU on power-up.
///
/// Mirrors the common bits across STM32 RCC_CSR registers (and analogous
/// registers on other MCU families). Multiple flags can be set at once
/// (e.g. `software | NRST_pin` is typical for a probe-rs flash sequence).
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct ResetCause {
    flags: u8,
}

impl ResetCause {
    pub const LOW_POWER: Self = Self { flags: 1 << 0 };
    pub const WINDOW_WATCHDOG: Self = Self { flags: 1 << 1 };
    pub const INDEP_WATCHDOG: Self = Self { flags: 1 << 2 };
    pub const SOFTWARE: Self = Self { flags: 1 << 3 };
    pub const BROWNOUT: Self = Self { flags: 1 << 4 };
    pub const PIN: Self = Self { flags: 1 << 5 };
    pub const OPTION_BYTE: Self = Self { flags: 1 << 6 };
    pub const FIREWALL: Self = Self { flags: 1 << 7 };

    pub const fn empty() -> Self {
        Self { flags: 0 }
    }
    pub const fn bits(self) -> u8 {
        self.flags
    }
    pub const fn is_empty(self) -> bool {
        self.flags == 0
    }
    pub const fn contains(self, other: Self) -> bool {
        (self.flags & other.flags) == other.flags
    }
    pub fn insert(&mut self, other: Self) {
        self.flags |= other.flags;
    }

    /// Iterate set flags with their human-readable labels, in fixed order
    /// (high-severity / less-common first). Useful for direct log output.
    pub fn iter_labels(self) -> impl Iterator<Item = &'static str> {
        const ENTRIES: &[(ResetCause, &str)] = &[
            (ResetCause::LOW_POWER, "low-power"),
            (ResetCause::WINDOW_WATCHDOG, "window-watchdog"),
            (ResetCause::INDEP_WATCHDOG, "indep-watchdog"),
            (ResetCause::SOFTWARE, "software-reset"),
            (ResetCause::BROWNOUT, "brownout"),
            (ResetCause::PIN, "NRST-pin"),
            (ResetCause::OPTION_BYTE, "option-byte-loader"),
            (ResetCause::FIREWALL, "firewall"),
        ];
        ENTRIES
            .iter()
            .filter_map(move |(flag, label)| self.contains(*flag).then_some(*label))
    }
}

impl core::ops::BitOr for ResetCause {
    type Output = Self;
    fn bitor(self, other: Self) -> Self {
        Self {
            flags: self.flags | other.flags,
        }
    }
}

impl core::ops::BitOrAssign for ResetCause {
    fn bitor_assign(&mut self, other: Self) {
        self.flags |= other.flags;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_iter_yields_nothing() {
        assert_eq!(ResetCause::empty().iter_labels().count(), 0);
    }

    #[test]
    fn bitor_combines_flags() {
        let r = ResetCause::SOFTWARE | ResetCause::PIN;
        assert!(r.contains(ResetCause::SOFTWARE));
        assert!(r.contains(ResetCause::PIN));
        assert!(!r.contains(ResetCause::BROWNOUT));
    }

    #[test]
    fn iter_labels_in_order_low_to_firewall() {
        let r = ResetCause::PIN | ResetCause::SOFTWARE | ResetCause::LOW_POWER;
        let labels: heapless::Vec<&str, 8> = r.iter_labels().collect();
        // LOW_POWER comes first, then SOFTWARE, then PIN (high-bit order in ENTRIES).
        assert_eq!(labels[0], "low-power");
        assert_eq!(labels[1], "software-reset");
        assert_eq!(labels[2], "NRST-pin");
    }
}
