//! Sinusoidal startup phase position advancement.

/// Phase positions for sinusoidal drive (0-359 degrees).
#[derive(Clone, Default)]
pub struct PhasePositions {
    pub a: i16,
    pub b: i16,
    pub c: i16,
}

impl PhasePositions {
    /// Advance phase positions by one step.
    /// Forward=true decrements (motor convention), forward=false increments.
    pub fn advance(&mut self, forward: bool) {
        if !forward {
            self.a += 1;
            if self.a > 359 { self.a = 0; }
            self.b += 1;
            if self.b > 359 { self.b = 0; }
            self.c += 1;
            if self.c > 359 { self.c = 0; }
        } else {
            self.a -= 1;
            if self.a < 0 { self.a = 359; }
            self.b -= 1;
            if self.b < 0 { self.b = 359; }
            self.c -= 1;
            if self.c < 0 { self.c = 359; }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_decrements() {
        let mut p = PhasePositions { a: 100, b: 219, c: 339 };
        p.advance(true);
        assert_eq!(p.a, 99);
        assert_eq!(p.b, 218);
        assert_eq!(p.c, 338);
    }

    #[test]
    fn reverse_increments() {
        let mut p = PhasePositions { a: 100, b: 219, c: 339 };
        p.advance(false);
        assert_eq!(p.a, 101);
        assert_eq!(p.b, 220);
        assert_eq!(p.c, 340);
    }

    #[test]
    fn forward_wraps_0_to_359() {
        let mut p = PhasePositions { a: 0, b: 0, c: 0 };
        p.advance(true);
        assert_eq!(p.a, 359);
        assert_eq!(p.b, 359);
        assert_eq!(p.c, 359);
    }

    #[test]
    fn reverse_wraps_359_to_0() {
        let mut p = PhasePositions { a: 359, b: 359, c: 359 };
        p.advance(false);
        assert_eq!(p.a, 0);
        assert_eq!(p.b, 0);
        assert_eq!(p.c, 0);
    }
}
