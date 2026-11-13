//! Compile-time motor-control tuning policy (`TuningPolicy`).
//!
//! [`MainState`](crate::main_state::MainState) is parameterized by a
//! [`TuningPolicy`] type (defaulting to [`Am32Tuning`]) that configures three
//! optional closed-loop recovery behaviours beyond stock AM32: a
//! duty-proportional overcurrent orbit trip, a post-lock reclimb duty clamp,
//! and a fast-rotor desync response that stays in interrupt mode with a half
//! duty kick. Every policy knob is an associated `const`, so
//! `if let Some(c) = T::ORBIT_TRIP` and `match T::DESYNC` resolve at compile
//! time: under [`Am32Tuning`] the extra branches compile out completely, and
//! under a custom policy they fold to immediate constants.

/// Wrong-phase-orbit trip configuration: if the BEMF acceptance chain locks
/// onto switching artifacts in the wrong commutation phase, zero-crossings can
/// remain regular enough to avoid the interval-jump desync detector while phase
/// current runs well above the normal steady-state draw for the applied duty.
///
/// The threshold is affine in duty (`duty * slope + offset_ma`, in milliamps,
/// scaled by measured battery voltage relative to `cal_vbat_cv`) and must be
/// exceeded for `trip_ms` consecutive 1 kHz ticks while locked before forcing
/// a full desync demotion.
pub struct OrbitTripConfig {
    /// Current threshold slope in milliamps per unit of duty (0..=2000).
    pub slope: i32,
    /// Base current threshold offset in milliamps at zero duty.
    pub offset_ma: i32,
    /// Consecutive 1 kHz ticks above the threshold line before tripping (ms).
    pub trip_ms: u16,
    /// Reference rail voltage in centivolts at which `slope` and `offset_ma`
    /// are specified; the threshold scales linearly with measured `vbat`
    /// relative to `cal_vbat_cv` so the policy tracks supply sag or different
    /// cell counts.
    pub cal_vbat_cv: i32,
    /// Commanded-transient suppression threshold: when applied duty changes by
    /// more than `transient_duty` within the 100 ms snapshot window, the trip
    /// is held off to allow acceleration current surges.
    pub transient_duty: u16,
    /// Additional hold-off duration (ms) after the last tick that detected a
    /// duty step above `transient_duty`, covering rotor acceleration after the
    /// commanded duty ramp finishes.
    pub transient_hold_ms: u16,
}

/// Post-lock duty ceiling clamp: until `confirm_zc` zero crossings confirm
/// closed-loop lock, the duty ceiling is capped at `duty_cap`. Applies both on
/// initial startup engage and after desync recovery to bound acceleration
/// before lock is established.
pub struct ReclimbConfig {
    pub confirm_zc: u32,
    pub duty_cap: u16,
}

/// Closed-loop desync recovery policy when a desync is detected in interrupt mode.
pub enum DesyncPolicy {
    /// Stock AM32 behaviour: demote to polling (`OldRoutine`) and drop duty to
    /// `min_startup / 2`.
    Am32,
    /// Keep high-speed desyncs (commutation interval below `max_ci_us` with
    /// phase current below the orbit threshold) in interrupt mode and halve the
    /// applied duty instead of demoting to 20 kHz BEMF polling. After a
    /// stay-in-interrupt recovery, the desync detector holds off for
    /// `rearm_holdoff_zc` zero crossings so deceleration from the duty step
    /// does not immediately re-trigger against a pre-kick interval average.
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

/// Stock AM32 closed-loop recovery policy (the default [`TuningPolicy`]).
pub struct Am32Tuning;

impl TuningPolicy for Am32Tuning {
    const ORBIT_TRIP: Option<OrbitTripConfig> = None;
    const RECLIMB: Option<ReclimbConfig> = None;
    const DESYNC: DesyncPolicy = DesyncPolicy::Am32;
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

/// Wrong-phase-orbit discriminator: returns `true` when `current_ma` exceeds
/// the policy's duty- and voltage-scaled threshold line. Always `false` when
/// `T::ORBIT_TRIP` is `None`.
#[inline]
pub(crate) fn orbit_current<T: TuningPolicy>(current_ma: i32, duty: u16, vbat_cv: u16) -> bool {
    match orbit_line::<T>(duty, vbat_cv) {
        Some(line) => current_ma > line,
        None => false,
    }
}

/// The orbit trip's current threshold line (mA) at `duty` and `vbat_cv`, or
/// `None` when `T::ORBIT_TRIP` is `None`.
#[inline]
pub(crate) fn orbit_line<T: TuningPolicy>(duty: u16, vbat_cv: u16) -> Option<i32> {
    let c = T::ORBIT_TRIP.as_ref()?;
    let line = (duty as i32) * c.slope + c.offset_ma;
    Some(if vbat_cv > 600 && c.cal_vbat_cv > 0 {
        ((line as i64) * (vbat_cv as i64) / (c.cal_vbat_cv as i64)) as i32
    } else {
        line
    })
}

/// Wrong-phase-orbit trip state machine (`OrbitTripConfig`), stepped once per
/// 1 kHz main-loop tick.
#[derive(Default)]
pub(crate) struct OrbitGuard {
    trip_count: u16,
    duty_snap: u16,
    snap_age: u16,
    since_move_ms: u16,
}

/// Result of one [`OrbitGuard::tick`] step.
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
    /// Step the guard for one 1 kHz tick.
    ///
    /// `locked`: running in closed-loop interrupt mode past 1000 zero crossings;
    /// `over_line`: phase current above [`orbit_line`].
    #[inline]
    pub(crate) fn tick(
        &mut self,
        c: &OrbitTripConfig,
        locked: bool,
        duty: u16,
        over_line: bool,
    ) -> OrbitStep {
        let moved = (duty as i32 - self.duty_snap as i32).unsigned_abs() > c.transient_duty as u32;
        self.snap_age += 1;
        if self.snap_age >= 100 {
            self.duty_snap = duty;
            self.snap_age = 0;
        }
        self.since_move_ms = if moved {
            0
        } else {
            self.since_move_ms.saturating_add(1)
        };
        let held = moved || self.since_move_ms < c.transient_hold_ms;
        let mut trip = false;
        let mut over_ms = 0;
        if locked && !held && over_line {
            self.trip_count = self.trip_count.saturating_add(1);
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
pub(crate) struct TestTuning;

#[cfg(test)]
impl TuningPolicy for TestTuning {
    const ORBIT_TRIP: Option<OrbitTripConfig> = Some(OrbitTripConfig {
        slope: 10,
        offset_ma: 10_000,
        trip_ms: 25,
        cal_vbat_cv: 1600,
        transient_duty: 150,
        transient_hold_ms: 200,
    });
    const RECLIMB: Option<ReclimbConfig> = Some(ReclimbConfig {
        confirm_zc: 1000,
        duty_cap: 800,
    });
    const DESYNC: DesyncPolicy = DesyncPolicy::HalfKickStayInterrupt {
        max_ci_us: 300,
        rearm_holdoff_zc: 100,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    const CFG: OrbitTripConfig = match TestTuning::ORBIT_TRIP {
        Some(c) => c,
        None => panic!(),
    };

    /// Simulate a throttle ramp (duty 200 -> 2000 over 50 ms) followed by an
    /// acceleration current plateau (`accel_ma` for `plateau_ms`, then settling
    /// below the steady-state threshold line). `phase` offsets the ramp start
    /// against the guard's 100 ms snapshot window.
    fn ramp_and_plateau(
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
            let line = orbit_line::<TestTuning>(duty, vbat_cv).unwrap();
            guard
                .tick(c, true, duty, current_ma > line)
                .trip
                .then_some(t)
        };
        for _ in 0..(1000 + phase) {
            if let Some(t) = step(200, 5_000) {
                return Some(t);
            }
        }
        for i in 0..50u16 {
            if let Some(t) = step(200 + i * 36, accel_ma) {
                return Some(t);
            }
        }
        for i in 0..500i32 {
            let current_ma = if i < plateau_ms {
                accel_ma
            } else {
                (accel_ma - (i - plateau_ms) * 200).max(20_000)
            };
            if let Some(t) = step(1998, current_ma) {
                return Some(t);
            }
        }
        None
    }

    #[test]
    fn step_response_holdoff_suppresses_trip_across_all_snapshot_phases() {
        for phase in 0..100 {
            let mut g = OrbitGuard::default();
            assert_eq!(
                ramp_and_plateau(&mut g, &CFG, phase, 1600, 36_000, 100),
                None,
                "phase {phase}"
            );
        }
    }

    #[test]
    fn step_response_trips_when_transient_hold_is_zero() {
        let no_hold = OrbitTripConfig {
            transient_hold_ms: 0,
            ..CFG
        };
        let trips = (0..100)
            .filter(|&p| {
                ramp_and_plateau(&mut OrbitGuard::default(), &no_hold, p, 1600, 36_000, 100)
                    .is_some()
            })
            .count();
        assert!(trips > 0, "no snapshot phase tripped without hold-off");
    }

    #[test]
    fn abrupt_step_on_snapshot_refresh_boundary_still_holds_off() {
        for phase in 0..=100 {
            let mut g = OrbitGuard::default();
            for _ in 0..phase {
                let s = g.tick(&CFG, true, 200, false);
                assert!(!s.trip);
            }
            // Single-tick step from 200 to 1000 (> transient_duty = 150),
            // including at phase == 99 where snap_age reaches 100 on this tick.
            let first = g.tick(&CFG, true, 1000, true);
            assert!(first.held, "phase {phase} must enter hold-off on step tick");
            assert!(!first.trip);
            for _ in 0..(CFG.trip_ms + 10) {
                assert!(
                    !g.tick(&CFG, true, 1000, true).trip,
                    "phase {phase} tripped during transient hold-off"
                );
            }
        }
    }

    #[test]
    fn over_ms_reports_the_trip_timer() {
        let mut g = OrbitGuard::default();
        for _ in 0..1000 {
            assert_eq!(g.tick(&CFG, true, 1000, false).over_ms, 0);
        }
        for n in 1..=CFG.trip_ms {
            assert_eq!(g.tick(&CFG, true, 1000, true).over_ms, n);
        }
        assert_eq!(g.tick(&CFG, true, 1000, false).over_ms, 0);
        for _ in 0..CFG.trip_ms {
            g.tick(&CFG, true, 1000, true);
        }
        let s = g.tick(&CFG, true, 1000, true);
        assert!(s.trip);
        assert_eq!(s.over_ms, CFG.trip_ms + 1);
    }

    #[test]
    fn sustained_over_line_at_steady_duty_trips() {
        let mut g = OrbitGuard::default();
        for _ in 0..1000 {
            assert!(!g.tick(&CFG, true, 1000, false).trip);
        }
        let n = (1..=200)
            .find(|_| g.tick(&CFG, true, 1000, true).trip)
            .expect("no trip");
        assert_eq!(n, CFG.trip_ms as usize + 1);
    }

    #[test]
    fn unlocked_or_short_excursions_do_not_trip() {
        let mut g = OrbitGuard::default();
        for _ in 0..1000 {
            g.tick(&CFG, true, 1000, false);
        }
        for _ in 0..500 {
            assert!(!g.tick(&CFG, false, 1000, true).trip);
        }
        for _ in 0..20 {
            for _ in 0..CFG.trip_ms {
                assert!(!g.tick(&CFG, true, 1000, true).trip);
            }
            assert!(!g.tick(&CFG, true, 1000, false).trip);
        }
    }

    #[test]
    fn orbit_line_scales_with_duty_and_rail_voltage() {
        // At reference rail (16.00 V), line = 1000 * 10 + 10_000 = 20_000 mA.
        assert_eq!(orbit_line::<TestTuning>(1000, 1600), Some(20_000));
        // At 24.00 V (1.5x reference), line scales to 30_000 mA.
        assert_eq!(orbit_line::<TestTuning>(1000, 2400), Some(30_000));
        // Uninitialized/low rail reading falls back to unscaled line.
        assert_eq!(orbit_line::<TestTuning>(1000, 0), Some(20_000));

        struct ZeroCalVbatTuning;
        impl TuningPolicy for ZeroCalVbatTuning {
            const ORBIT_TRIP: Option<OrbitTripConfig> = Some(OrbitTripConfig {
                cal_vbat_cv: 0,
                ..CFG
            });
            const RECLIMB: Option<ReclimbConfig> = None;
            const DESYNC: DesyncPolicy = DesyncPolicy::Am32;
        }
        assert_eq!(orbit_line::<ZeroCalVbatTuning>(1000, 1600), Some(20_000));
    }

    #[test]
    fn am32_tuning_has_no_orbit_line() {
        assert_eq!(orbit_line::<Am32Tuning>(1998, 1600), None);
        assert!(!orbit_current::<Am32Tuning>(i32::MAX, 0, 1600));
        assert_eq!(
            stay_interrupt_ci::<crate::platform::TestPlatform, Am32Tuning>(),
            0
        );
    }
}
