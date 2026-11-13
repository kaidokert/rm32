//! Compile-time motor-control tuning policy (`TuningPolicy`).
//!
//! [`MainState`](crate::main_state::MainState) is parameterized by a
//! [`TuningPolicy`] type (defaulting to [`Am32Tuning`]) that configures three
//! optional closed-loop recovery behaviours beyond stock AM32: a
//! wrong-phase-orbit current trip, a post-lock reclimb duty clamp, and a
//! fast-rotor desync response that stays in interrupt mode with a half
//! duty kick. Every policy knob is an associated `const`, so
//! `if let Some(c) = T::ORBIT_TRIP` and `match T::DESYNC` resolve at compile
//! time: under [`Am32Tuning`] the extra branches compile out completely, and
//! under [`QualifiedRigTuning`] they fold to immediate constants.

/// Wrong-phase-orbit trip (divergence from AM32): the acceptance chain can
/// self-clock on switching artifacts in a wrong phase -- plausible
/// zero-crossings, current far above the duty-proportional norm, no
/// desync-detector jump. The trip is relative (fires when current exceeds
/// `duty * slope + offset_ma`, sustained `trip_ms` ticks while locked)
/// because a static ampere threshold cannot separate a wrong-phase orbit at
/// 70 % from legitimate 100 % draw. Response = full desync path: the demote
/// IS the phase reset.
pub struct OrbitTripConfig {
    pub slope: i32,
    pub offset_ma: i32,
    /// Consecutive 1 kHz ticks over the line before tripping (ms).
    pub trip_ms: u16,
    /// Rail voltage (cV) the line was calibrated on; the line scales by
    /// measured vbat relative to it (battery sessions run ~12 V where the
    /// same duty legitimately draws ~1.5x the current).
    pub cal_vbat_cv: i32,
    /// Commanded-transient suppression: while the applied duty moved more
    /// than this within the snapshot window, high current is expected (slam
    /// accel, ~10 A on battery) and the trip holds off.
    pub transient_duty: u16,
    /// The hold-off continues this long (ms) after the last tick that saw
    /// such a move. The snapshot window alone ends 0-100 ms after the duty
    /// settles (whatever the refresh phase happens to be), while the rotor is
    /// still accelerating and drawing high current past the steady-state
    /// orbit line.
    pub transient_hold_ms: u16,
}

/// Reclimb clamp (divergence from AM32): until `confirm_zc` zero crossings
/// confirm the lock, the duty ceiling is capped at `duty_cap`. Covers fresh
/// engage and post-fall recovery identically; the cap clears the startup
/// duty band while bounding the recovery acceleration surge.
pub struct ReclimbConfig {
    pub confirm_zc: u32,
    pub duty_cap: u16,
}

/// What a desync from interrupt mode does.
pub enum DesyncPolicy {
    /// AM32: demote to polling and drop the duty to min_startup/2.
    Am32,
    /// Fast-rotor desyncs (commutation interval below `max_ci_us`, current
    /// sane) stay in interrupt mode with a half duty kick. AM32 re-locks
    /// within ~1 ms at main-loop rate; rm32 polls on the 20 kHz tick grid,
    /// which cannot re-lock above ~800 Hz e, so a demoted fast rotor coasts
    /// for seconds. The detector then holds off for `rearm_holdoff_zc` zero
    /// crossings: the kick's own deceleration would refire it against a
    /// stale average (a self-loop); a real desync still trips on the first
    /// check past the holdoff.
    HalfKickStayInterrupt {
        max_ci_us: u32,
        rearm_holdoff_zc: u32,
    },
}

pub trait TuningPolicy {
    const ORBIT_TRIP: Option<OrbitTripConfig>;
    const RECLIMB: Option<ReclimbConfig>;
    const DESYNC: DesyncPolicy;
}

/// AM32's behaviour: no rig divergence.
pub struct Am32Tuning;

impl TuningPolicy for Am32Tuning {
    const ORBIT_TRIP: Option<OrbitTripConfig> = None;
    const RECLIMB: Option<ReclimbConfig> = None;
    const DESYNC: DesyncPolicy = DesyncPolicy::Am32;
}

/// Bench-qualified tuning preset enabling wrong-phase-orbit detection,
/// post-lock reclimb duty clamping, and fast-rotor stay-in-interrupt desync
/// recovery.
pub struct QualifiedRigTuning;

impl TuningPolicy for QualifiedRigTuning {
    const ORBIT_TRIP: Option<OrbitTripConfig> = Some(OrbitTripConfig {
        slope: 3,
        offset_ma: 2500,
        trip_ms: 30,
        cal_vbat_cv: 816,
        transient_duty: 150,
        transient_hold_ms: 300,
    });
    const RECLIMB: Option<ReclimbConfig> = Some(ReclimbConfig {
        confirm_zc: 1500,
        duty_cap: 800,
    });
    const DESYNC: DesyncPolicy = DesyncPolicy::HalfKickStayInterrupt {
        max_ci_us: 300,
        rearm_holdoff_zc: 100,
    };
}

/// Commutation interval (port timer ticks) below which a desync stays in
/// interrupt mode (`DesyncPolicy::HalfKickStayInterrupt`); 0 under the AM32
/// policy, so the branch is never taken.
#[inline(always)]
pub(crate) const fn stay_interrupt_ci<P: crate::platform::Platform, T: TuningPolicy>() -> u32 {
    match T::DESYNC {
        DesyncPolicy::Am32 => 0,
        DesyncPolicy::HalfKickStayInterrupt { max_ci_us, .. } => {
            crate::platform::us_to_ticks::<P>(max_ci_us)
        }
    }
}

/// Wrong-phase-orbit discriminator: current far above the
/// duty-proportional norm. The line scales with measured vbat since
/// current at a given duty scales with source voltage. Always false
/// without an orbit trip.
#[inline]
pub(crate) fn orbit_current<T: TuningPolicy>(current_ma: i32, duty: u16, vbat_cv: u16) -> bool {
    match orbit_line::<T>(duty, vbat_cv) {
        Some(line) => current_ma > line,
        None => false,
    }
}

/// The orbit trip's current line (mA) at this duty and rail, `None` without
/// an orbit trip.
#[inline]
pub(crate) fn orbit_line<T: TuningPolicy>(duty: u16, vbat_cv: u16) -> Option<i32> {
    let c = T::ORBIT_TRIP.as_ref()?;
    let line = (duty as i32) * c.slope + c.offset_ma;
    Some(if vbat_cv > 600 {
        line * (vbat_cv as i32) / c.cal_vbat_cv
    } else {
        line
    })
}

/// The wrong-phase-orbit trip's state machine (`OrbitTripConfig`), one
/// `tick` per 1 kHz main-loop tick.
#[derive(Default)]
pub(crate) struct OrbitGuard {
    trip_count: u16,
    duty_snap: u16,
    snap_age: u16,
    since_move_ms: u16,
}

/// One orbit-guard tick: `trip` = force the desync path now.
#[allow(dead_code)]
pub(crate) struct OrbitStep {
    pub trip: bool,
    /// Commanded-transient hold-off active this tick.
    pub held: bool,
    /// Milliseconds since the duty last moved past `transient_duty`.
    pub since_move_ms: u16,
    /// Consecutive locked, un-held ms over the line including this tick
    /// (`trip_ms` + 1 on the tripping tick, 0 when not over).
    pub over_ms: u16,
}

impl OrbitGuard {
    /// `locked`: running with the BEMF loop past 1000 zero crossings;
    /// `over_line`: current above `orbit_line`.
    #[inline]
    pub(crate) fn tick(
        &mut self,
        c: &OrbitTripConfig,
        locked: bool,
        duty: u16,
        over_line: bool,
    ) -> OrbitStep {
        self.snap_age += 1;
        if self.snap_age >= 100 {
            self.duty_snap = duty;
            self.snap_age = 0;
        }
        let moved = (duty as i32 - self.duty_snap as i32).unsigned_abs() > c.transient_duty as u32;
        self.since_move_ms = if moved {
            0
        } else {
            self.since_move_ms.saturating_add(1)
        };
        let held = moved || self.since_move_ms < c.transient_hold_ms;
        let mut trip = false;
        let mut over_ms = 0;
        if locked && !held && over_line {
            self.trip_count += 1;
            over_ms = self.trip_count;
            if self.trip_count > c.trip_ms {
                self.trip_count = 0;
                trip = true;
            }
        } else {
            self.trip_count = 0;
        }
        OrbitStep {
            trip,
            held,
            since_move_ms: self.since_move_ms,
            over_ms,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RIG: OrbitTripConfig = match QualifiedRigTuning::ORBIT_TRIP {
        Some(c) => c,
        None => panic!(),
    };

    /// Representative sagged 3S rail under a full-throttle slam (cV).
    const SLAM_VBAT_CV: u16 = 1132;

    /// Slam 10 % -> 100 %: duty ramps up over 50 ms, then the rotor keeps
    /// accelerating at ~12.8 A for 140 ms before the current falls to the
    /// 100 % running draw. `phase` offsets the slam against the guard's
    /// 100 ms snapshot cycle. Returns the trip tick.
    fn slam(guard: &mut OrbitGuard, c: &OrbitTripConfig, phase: u16) -> Option<u32> {
        slam_on(guard, c, phase, SLAM_VBAT_CV, 12_800, 140)
    }

    /// `slam` on a given rail with a given accel current and plateau (ms).
    fn slam_on(
        guard: &mut OrbitGuard,
        c: &OrbitTripConfig,
        phase: u16,
        vbat_cv: u16,
        accel_ma: i32,
        plateau_ms: i32,
    ) -> Option<u32> {
        let mut t = 0u32;
        let mut step = |duty: u16, current_ma: i32| {
            t += 1;
            let line = orbit_line::<QualifiedRigTuning>(duty, vbat_cv).unwrap();
            guard
                .tick(c, true, duty, current_ma > line)
                .trip
                .then_some(t)
        };
        for _ in 0..(1000 + phase) {
            if let Some(t) = step(200, 1500) {
                return Some(t);
            }
        }
        for i in 0..50u16 {
            if let Some(t) = step(200 + i * 36, accel_ma) {
                return Some(t);
            }
        }
        for i in 0..1000i32 {
            let current_ma = if i < plateau_ms {
                accel_ma
            } else {
                (accel_ma - (i - plateau_ms) * 50).max(7_000)
            };
            if let Some(t) = step(1998, current_ma) {
                return Some(t);
            }
        }
        None
    }

    /// The snapshot window alone ends at an arbitrary point 0-100 ms after
    /// the duty settles, so legitimate slam acceleration must not trip the
    /// orbit guard on any snapshot phase while `transient_hold_ms` is active.
    #[test]
    fn slam_acceleration_never_trips_at_any_snapshot_phase() {
        for phase in 0..100 {
            let mut g = OrbitGuard::default();
            assert_eq!(slam(&mut g, &RIG, phase), None, "phase {phase}");
        }
    }

    /// The same profile trips without the hold, so the test above covers the
    /// failure rather than a profile that never reaches the line.
    #[test]
    fn slam_profile_trips_without_the_hold() {
        let no_hold = OrbitTripConfig {
            transient_hold_ms: 0,
            ..RIG
        };
        let trips = (0..100)
            .filter(|&p| slam(&mut OrbitGuard::default(), &no_hold, p).is_some())
            .count();
        assert!(trips > 0, "no snapshot phase tripped without the hold");
    }

    /// On a sagged 10.6 V rail (15 % -> 100 % slam drawing ~12.7 A against a
    /// ~11.0 A line for a 20 ms plateau), ~1/3 of snapshot phases trip
    /// without `transient_hold_ms`, and zero phases trip with `RIG`'s hold.
    #[test]
    fn battery_slam_replay_trips_a_third_without_hold_never_with() {
        let phases = |c: &OrbitTripConfig| {
            (0..100)
                .filter(|&p| slam_on(&mut OrbitGuard::default(), c, p, 1060, 12_700, 20).is_some())
                .count()
        };
        let no_hold = OrbitTripConfig {
            transient_hold_ms: 0,
            ..RIG
        };
        let old = phases(&no_hold);
        assert!((20..=40).contains(&old), "no-hold trips on {old} phases");
        assert_eq!(phases(&RIG), 0);
    }

    /// The trip timer's watermark: `over_ms` counts the consecutive stretch,
    /// reaches `trip_ms` on a near miss and `trip_ms + 1` on the trip.
    #[test]
    fn over_ms_reports_the_trip_timer() {
        let mut g = OrbitGuard::default();
        for _ in 0..1000 {
            assert_eq!(g.tick(&RIG, true, 1000, false).over_ms, 0);
        }
        for n in 1..=RIG.trip_ms {
            assert_eq!(g.tick(&RIG, true, 1000, true).over_ms, n);
        }
        assert_eq!(g.tick(&RIG, true, 1000, false).over_ms, 0);
        for _ in 0..RIG.trip_ms {
            g.tick(&RIG, true, 1000, true);
        }
        let s = g.tick(&RIG, true, 1000, true);
        assert!(s.trip);
        assert_eq!(s.over_ms, RIG.trip_ms + 1);
    }

    /// A real orbit at steady duty still trips after `trip_ms` + 1 ticks.
    #[test]
    fn sustained_over_line_at_steady_duty_trips() {
        let mut g = OrbitGuard::default();
        for _ in 0..1000 {
            assert!(!g.tick(&RIG, true, 1000, false).trip);
        }
        let n = (1..=200)
            .find(|_| g.tick(&RIG, true, 1000, true).trip)
            .expect("no trip");
        assert_eq!(n, RIG.trip_ms as usize + 1);
    }

    /// Over the line but not locked, or for no more than `trip_ms`: no trip.
    #[test]
    fn unlocked_or_short_excursions_do_not_trip() {
        let mut g = OrbitGuard::default();
        for _ in 0..1000 {
            g.tick(&RIG, true, 1000, false);
        }
        for _ in 0..500 {
            assert!(!g.tick(&RIG, false, 1000, true).trip);
        }
        for _ in 0..20 {
            for _ in 0..RIG.trip_ms {
                assert!(!g.tick(&RIG, true, 1000, true).trip);
            }
            assert!(!g.tick(&RIG, true, 1000, false).trip);
        }
    }

    #[test]
    fn am32_tuning_has_no_orbit_line() {
        assert_eq!(orbit_line::<Am32Tuning>(1998, 1200), None);
        assert!(!orbit_current::<Am32Tuning>(i32::MAX, 0, 1200));
    }
}
