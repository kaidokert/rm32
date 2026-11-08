//! Execute the production GPIO and borrowed TIM1 adapters on observable registers.
//! This verifies software write order, not electrical timing or transistor behavior.
#![allow(dead_code, unexpected_cfgs)] // The included port modules also expose board-only helpers.

use rm32::bridge::Bridge;
use rm32::hal::{PhaseOutput, PwmMode};
use std::cell::RefCell;

#[path = "../../rm32_stm32/src/bridge.rs"]
mod bridge;
#[path = "../../rm32_stm32/src/emergency.rs"]
mod emergency;
#[path = "../../rm32_stm32/src/gpio_pin.rs"]
mod gpio_pin;
#[path = "../../rm32_stm32/src/gpio_regs.rs"]
mod gpio_regs;
#[path = "../../rm32_stm32/src/phase.rs"]
mod phase;

#[derive(Clone, Debug, PartialEq, Eq)]
enum Write {
    Mode(usize, u32),
    Bsrr(usize, u32),
    Odr(usize, u32),
    Compare(usize, u16),
    Period(u16),
    Divider(u16),
    DeadTime(u16),
    Latch,
    DelayMs(u32),
    Watchdog,
    Irq(bool),
}
#[derive(Clone)]
struct Registers {
    mode: [u32; 2],
    odr: [u32; 2],
    queued: [u16; 3],
    live: [u16; 3],
    writes: Vec<Write>,
}
impl Default for Registers {
    fn default() -> Self {
        Self {
            mode: [0x93ac_56f0, 0x6b93_a56c],
            odr: [0x1234, 0x4321],
            queued: [17, 23, 41],
            live: [7, 11, 13],
            writes: vec![],
        }
    }
}
thread_local! { static REGS: RefCell<Registers> = RefCell::new(Registers::default()); }

mod mcu {
    pub struct Port<const N: usize>;
    pub type PortA = Port<0>;
    pub type PortB = Port<1>;
    impl<const N: usize> super::gpio_regs::GpioPort for Port<N> {
        fn write_bsrr(value: u32) {
            super::REGS.with_borrow_mut(|r| {
                r.writes.push(super::Write::Bsrr(N, value));
                r.odr[N] = (r.odr[N] & !(value >> 16)) | (value & 0xffff);
            });
        }
        fn modify_moder(f: impl FnOnce(u32) -> u32) {
            super::REGS.with_borrow_mut(|r| {
                r.mode[N] = f(r.mode[N]);
                r.writes.push(super::Write::Mode(N, r.mode[N]));
            });
        }
        fn read_odr() -> u32 {
            super::REGS.with_borrow(|r| r.odr[N])
        }
        fn write_odr(value: u32) {
            super::REGS.with_borrow_mut(|r| {
                r.odr[N] = value;
                r.writes.push(super::Write::Odr(N, value));
            });
        }
    }
}

// Independent pin and commutation truth tables; do not derive expected values
// from step_legs, floating_phase, PHASE_ORDER or GpioPin constants.
const HIGH: [(usize, u8); 3] = [(0, 10), (0, 9), (0, 8)];
const LOW: [(usize, u8); 3] = [(1, 1), (1, 0), (0, 7)];
// (floating, low, driven), steps 1..6.
const ORDER: [[usize; 3]; 6] = [
    [2, 1, 0],
    [0, 1, 2],
    [1, 0, 2],
    [2, 0, 1],
    [0, 2, 1],
    [1, 2, 0],
];

/// Record a two-bit GPIO MODER update for `(port, pin)`.
fn mode(r: &mut Registers, (port, pin): (usize, u8), value: u32) {
    r.mode[port] = (r.mode[port] & !(3 << (pin * 2))) | (value << (pin * 2));
    r.writes.push(Write::Mode(port, r.mode[port]));
}

/// Record a GPIO BSRR set/reset write for `(port, pin)`.
fn level(r: &mut Registers, (port, pin): (usize, u8), high: bool) {
    let value = 1 << (pin + if high { 0 } else { 16 });
    r.writes.push(Write::Bsrr(port, value));
    r.odr[port] = (r.odr[port] & !(1 << pin)) | if high { 1 << pin } else { 0 };
}

/// Record a sequential floating (`low_on = false`) or solid-low (`low_on = true`) phase update.
fn solid(r: &mut Registers, phase: usize, low_on: bool) {
    if low_on {
        mode(r, LOW[phase], 1);
        level(r, LOW[phase], true);
        mode(r, HIGH[phase], 1);
        level(r, HIGH[phase], false);
    } else {
        level(r, LOW[phase], false);
        mode(r, LOW[phase], 1);
        level(r, HIGH[phase], false);
        mode(r, HIGH[phase], 1);
    }
}

/// Record a sequential PWM phase update for `(complementary, enable)`.
fn pwm(r: &mut Registers, phase: usize, complementary: bool, enable: bool) {
    match (enable, complementary) {
        (true, false) => {
            mode(r, LOW[phase], 1);
            level(r, LOW[phase], false);
        }
        (true, true) => {
            mode(r, LOW[phase], 1);
            level(r, LOW[phase], true);
        }
        (false, true) => {
            mode(r, LOW[phase], 2);
            level(r, LOW[phase], false);
        }
        (false, false) => {
            mode(r, LOW[phase], 1);
            level(r, LOW[phase], false);
        }
    }
    mode(r, HIGH[phase], 2);
}

/// Record a batched per-port commutation step with break-before-make port ordering.
fn atomic_step(r: &mut Registers, float: usize, low: usize, driven: usize, complementary: bool) {
    let mut bsrr = [0u32; 2];
    let mut moder_mask = [0u32; 2];
    let mut moder_val = [0u32; 2];
    let mut apply_pin = |(port, pin): (usize, u8), pin_mode: u32, high: bool| {
        moder_mask[port] |= 3 << (pin * 2);
        moder_val[port] |= pin_mode << (pin * 2);
        bsrr[port] |= 1 << (pin + if high { 0 } else { 16 });
    };
    apply_pin(HIGH[float], 1, false);
    apply_pin(LOW[float], 1, false);
    apply_pin(HIGH[low], 1, false);
    apply_pin(LOW[low], 1, true);
    apply_pin(HIGH[driven], 2, false);
    apply_pin(LOW[driven], if complementary { 2 } else { 1 }, false);

    let bsrr_ports = if (bsrr[0] & 0xffff) != 0 {
        [1, 0]
    } else {
        [0, 1]
    };
    for port in bsrr_ports {
        r.writes.push(Write::Bsrr(port, bsrr[port]));
        r.odr[port] = (r.odr[port] & !(bsrr[port] >> 16)) | (bsrr[port] & 0xffff);
    }
    let moder_ports = if (moder_val[1] & 0xaaaa_aaaa) == 0 {
        [1, 0]
    } else {
        [0, 1]
    };
    for port in moder_ports {
        r.mode[port] = (r.mode[port] & !moder_mask[port]) | moder_val[port];
        r.writes.push(Write::Mode(port, r.mode[port]));
    }
}

/// Count how many low-side gates are actively driven high in output mode.
fn active_low_side_gates(mode: &[u32; 2], odr: &[u32; 2]) -> usize {
    LOW.iter()
        .filter(|&&(port, pin)| {
            let pin_mode = (mode[port] >> (pin * 2)) & 0b11;
            let pin_high = ((odr[port] >> pin) & 1) != 0;
            pin_mode == 1 && pin_high
        })
        .count()
}

/// Assert that recorded registers match `expected`.
fn assert_registers(expected: &Registers) {
    REGS.with_borrow(|actual| {
        assert_eq!(actual.writes, expected.writes);
        assert_eq!(actual.mode, expected.mode);
        assert_eq!(actual.odr, expected.odr);
        assert_eq!(actual.queued, expected.queued);
        assert_eq!(actual.live, expected.live);
    });
}

#[test]
fn six_steps_preserve_every_gpio_write_in_both_directions_and_drive_modes() {
    for enable in [false, true] {
        for pwm_mode in [PwmMode::Diode, PwmMode::Complementary] {
            let complementary = pwm_mode.is_complementary();
            for forward in [false, true] {
                REGS.set(Registers::default());
                let mut expected = Registers::default();
                let mut sequential_final = Registers::default();
                let mut driver = if enable {
                    phase::G0APhaseDriver::new_bridge()
                } else {
                    phase::G0APhaseDriver::new()
                };
                // Start from a known all-off bridge state before stepping.
                driver.all_off();
                for p in 0..3 {
                    solid(&mut expected, p, false);
                    solid(&mut sequential_final, p, false);
                }
                // Exercise consecutive GPIO states in each rotation direction.
                let sequence = if forward {
                    [2, 3, 4, 5, 6, 1]
                } else {
                    [6, 5, 4, 3, 2, 1]
                };
                for step in sequence {
                    let prev_mode = expected.mode;
                    let prev_odr = expected.odr;
                    let write_start = expected.writes.len();
                    let [float, low, driven] = ORDER[usize::from(step - 1)];
                    solid(&mut sequential_final, float, false);
                    solid(&mut sequential_final, low, true);
                    pwm(&mut sequential_final, driven, complementary, enable);
                    if enable {
                        solid(&mut expected, float, false);
                        solid(&mut expected, low, true);
                        pwm(&mut expected, driven, complementary, enable);
                    } else {
                        atomic_step(&mut expected, float, low, driven, complementary);
                    }
                    driver.com_step(step, pwm_mode);
                    assert_registers(&expected);
                    // Final MODER/ODR state on the batched path must match the
                    // sequential per-pin reference bit-for-bit.
                    assert_eq!(expected.mode, sequential_final.mode);
                    assert_eq!(expected.odr, sequential_final.odr);
                    if !enable {
                        // Verify break-before-make: at no intermediate register
                        // write during com_step are two low-side gates driven
                        // high simultaneously.
                        let mut sim_mode = prev_mode;
                        let mut sim_odr = prev_odr;
                        for w in &expected.writes[write_start..] {
                            match *w {
                                Write::Bsrr(port, val) => {
                                    sim_odr[port] = (sim_odr[port] & !(val >> 16)) | (val & 0xffff);
                                }
                                Write::Mode(port, val) => {
                                    sim_mode[port] = val;
                                }
                                _ => {}
                            }
                            assert!(
                                active_low_side_gates(&sim_mode, &sim_odr) <= 1,
                                "low-side overlap on step {step} (forward={forward}, comp={complementary})"
                            );
                        }
                    }
                }
                for invalid in (0..=255).filter(|s| !(1..=6).contains(s)) {
                    driver.com_step(invalid, pwm_mode);
                    assert_registers(&expected); // Invalid steps remain no-ops.
                }
            }
        }
    }
}

#[test]
fn off_brake_sine_and_reentry_preserve_write_order() {
    for enable in [false, true] {
        for pwm_mode in [PwmMode::Diode, PwmMode::Complementary] {
            let complementary = pwm_mode.is_complementary();
            REGS.set(Registers::default());
            let mut expected = Registers::default();
            let mut driver = if enable {
                phase::G0APhaseDriver::new_bridge()
            } else {
                phase::G0APhaseDriver::new()
            };
            driver.all_off();
            for p in 0..3 {
                solid(&mut expected, p, false);
            }
            assert_registers(&expected);
            driver.full_brake();
            for p in 0..3 {
                solid(&mut expected, p, true);
            }
            assert_registers(&expected);
            driver.proportional_brake();
            if !enable {
                for pin in HIGH {
                    mode(&mut expected, pin, 1);
                    level(&mut expected, pin, false);
                }
                for pin in LOW {
                    mode(&mut expected, pin, 2);
                }
            }
            assert_registers(&expected);
            driver.all_off();
            for p in 0..3 {
                solid(&mut expected, p, false);
            }
            driver.all_pwm(pwm_mode);
            for p in 0..3 {
                pwm(&mut expected, p, complementary, enable);
            }
            assert_registers(&expected);
        }
    }
}

#[test]
fn complementary_sine_entry_after_active_brake_clears_low_latch_before_all_off() {
    REGS.set(Registers::default());
    let mut driver = phase::G0APhaseDriver::new();
    // Active brake sector 2 drives PB0 (Phase B low-side) solid high.
    driver.com_step(2, PwmMode::Complementary);
    REGS.with_borrow(|r| {
        assert_ne!(r.odr[1] & (1 << 0), 0, "PB0 ODR should be high in step 2");
    });
    // Entering complementary sine startup (`all_pwm(PwmMode::Complementary)`) must clear PB0's
    // ODR latch while switching to alternate mode so a subsequent `all_off()`
    // never exposes a high output latch while PB0 returns to output mode.
    driver.all_pwm(PwmMode::Complementary);
    REGS.with_borrow(|r| {
        assert_eq!(
            r.odr[1] & (1 << 0),
            0,
            "all_pwm(PwmMode::Complementary) must clear PB0 ODR latch"
        );
    });
    let (mut sim_mode, mut sim_odr, start) = REGS.with_borrow(|r| (r.mode, r.odr, r.writes.len()));
    driver.all_off();
    REGS.with_borrow(|r| {
        for w in &r.writes[start..] {
            match *w {
                Write::Bsrr(port, val) => {
                    sim_odr[port] = (sim_odr[port] & !(val >> 16)) | (val & 0xffff);
                }
                Write::Mode(port, val) => {
                    sim_mode[port] = val;
                }
                _ => {}
            }
            assert_eq!(
                active_low_side_gates(&sim_mode, &sim_odr),
                0,
                "all_off() after complementary sine must never glitch a low-side gate high"
            );
        }
    });
}

#[test]
fn bridge_enable_deasserts_enable_pins_when_switching_to_diode() {
    REGS.set(Registers::default());
    let mut driver = phase::G0APhaseDriver::new_bridge();
    driver.all_pwm(PwmMode::Complementary);
    REGS.with_borrow(|r| {
        for &(port, pin) in &LOW {
            assert_ne!(r.odr[port] & (1 << pin), 0);
        }
    });
    driver.all_pwm(PwmMode::Diode);
    REGS.with_borrow(|r| {
        for &(port, pin) in &LOW {
            assert_eq!(
                r.odr[port] & (1 << pin),
                0,
                "all_pwm(PwmMode::Diode) on bridge_enable must de-assert enable pin"
            );
        }
    });
}

#[test]
fn pulse_output_only_toggles_on_sectors_one_and_four() {
    REGS.set(Registers::default());
    let mut expected = Registers::default();
    let mut driver = phase::G0APhaseDriver::new();
    driver.pulse_toggle(1);
    assert_registers(&expected);
    driver.enable_pulse_output::<gpio_pin::PB10>();
    mode(&mut expected, (1, 10), 1);
    level(&mut expected, (1, 10), false);
    for step in 0..=255 {
        driver.pulse_toggle(step);
        if step == 1 || step == 4 {
            expected.odr[1] ^= 1 << 10;
            expected.writes.push(Write::Odr(1, expected.odr[1]));
        }
        assert_registers(&expected);
    }
}

struct Timer;
impl bridge::TimerPwm for Timer {
    fn set_compare1(&mut self, v: u16) {
        compare(0, v);
    }
    fn set_compare2(&mut self, v: u16) {
        compare(1, v);
    }
    fn set_compare3(&mut self, v: u16) {
        compare(2, v);
    }
    fn set_duty_all(&mut self, v: u16) {
        for p in 0..3 {
            compare(p, v);
        }
    }
    fn set_auto_reload(&mut self, v: u16) {
        REGS.with_borrow_mut(|r| r.writes.push(Write::Period(v)));
    }
    fn set_prescaler(&mut self, v: u16) {
        REGS.with_borrow_mut(|r| r.writes.push(Write::Divider(v)));
    }
    fn set_dead_time_override(&mut self, v: u16) {
        REGS.with_borrow_mut(|r| r.writes.push(Write::DeadTime(v)));
    }
    fn generate_update_event(&mut self) {
        REGS.with_borrow_mut(|r| {
            r.live = r.queued;
            r.writes.push(Write::Latch);
        });
    }
}
fn compare(p: usize, v: u16) {
    REGS.with_borrow_mut(|r| {
        r.queued[p] = v;
        r.writes.push(Write::Compare(p, v));
    });
}

#[test]
fn borrowed_bridge_keeps_compare_order_and_explicit_latch_boundary() {
    REGS.set(Registers::default());
    let mut timer = Timer;
    let mut driver = phase::G0APhaseDriver::new();
    let mut b = bridge::StmBridge {
        pwm: &mut timer,
        phase: &mut driver,
    };
    b.set_period(1999);
    b.set_phase_duties(101, 202, 303);
    REGS.with_borrow(|r| {
        assert_eq!(r.live, [7, 11, 13]);
        assert_eq!(r.queued, [101, 202, 303]);
        assert_eq!(
            r.writes,
            [
                Write::Period(1999),
                Write::Compare(0, 101),
                Write::Compare(1, 202),
                Write::Compare(2, 303)
            ]
        );
    });
    b.latch();
    b.set_duty_all(0);
    REGS.with_borrow(|r| assert_eq!(r.live, [101, 202, 303]));
    b.latch();
    b.all_off();
    let mut expected = Registers {
        queued: [0; 3],
        live: [0; 3],
        writes: vec![
            Write::Period(1999),
            Write::Compare(0, 101),
            Write::Compare(1, 202),
            Write::Compare(2, 303),
            Write::Latch,
            Write::Compare(0, 0),
            Write::Compare(1, 0),
            Write::Compare(2, 0),
            Write::Latch,
        ],
        ..Default::default()
    };
    for p in 0..3 {
        solid(&mut expected, p, false);
    }
    assert_registers(&expected);
    b.set_tone_divider(15);
    b.set_dead_time_override(67);
    expected
        .writes
        .extend([Write::Divider(15), Write::DeadTime(67)]);
    assert_registers(&expected);
}

struct System {
    irq: bool,
}
impl rm32::hal::System for System {
    fn reset(&mut self) -> ! {
        panic!("unexpected reset")
    }
    fn enable_irq(&mut self) {
        self.irq = true;
        REGS.with_borrow_mut(|r| r.writes.push(Write::Irq(true)));
    }
    fn disable_irq(&mut self) {
        self.irq = false;
        REGS.with_borrow_mut(|r| r.writes.push(Write::Irq(false)));
    }
    fn irqs_enabled(&self) -> bool {
        self.irq
    }
    fn start_watchdog(&mut self, _: u8, _: u16) {
        panic!("unexpected watchdog start")
    }
    fn reload_watchdog(&mut self) {
        REGS.with_borrow_mut(|r| r.writes.push(Write::Watchdog));
    }
    fn delay_micros(&mut self, _: u32) {
        panic!("unexpected microsecond delay")
    }
    fn delay_millis(&mut self, v: u32) {
        REGS.with_borrow_mut(|r| r.writes.push(Write::DelayMs(v)));
    }
}

#[test]
fn firmware_tones_keep_gpio_timer_and_irq_sequence() {
    for enable in [false, true] {
        for input_tune in [false, true] {
            REGS.set(Registers::default());
            let mut expected = Registers::default();
            let mut timer = Timer;
            let mut driver = if enable {
                phase::G0APhaseDriver::new_bridge()
            } else {
                phase::G0APhaseDriver::new()
            };
            let mut b = bridge::StmBridge {
                pwm: &mut timer,
                phase: &mut driver,
            };
            let mut sys = System { irq: input_tune };
            let sounds = rm32::sounds::Sounds::new(1999);
            let notes = if input_tune {
                sounds.play_input(&mut b, &mut sys);
                expected.writes.push(Write::Irq(false));
                [(80, 3, 100), (70, 3, 100), (40, 3, 100)]
            } else {
                sounds.play_startup(&mut b, &mut sys);
                [(55, 3, 200), (40, 5, 200), (25, 6, 200)]
            };
            for (divider, step, delay) in notes {
                expected
                    .writes
                    .extend([Write::Period(1999), Write::Divider(divider)]);
                for p in 0..3 {
                    expected.writes.push(Write::Compare(p, 15));
                }
                expected.queued = [15; 3];
                let [float, low, driven] = ORDER[step - 1];
                if enable {
                    solid(&mut expected, float, false);
                    solid(&mut expected, low, true);
                    pwm(&mut expected, driven, false, true);
                } else {
                    atomic_step(&mut expected, float, low, driven, false);
                }
                expected
                    .writes
                    .extend([Write::DelayMs(delay), Write::Watchdog]);
            }
            for p in 0..3 {
                solid(&mut expected, p, false);
            }
            expected
                .writes
                .extend([Write::Divider(0), Write::Period(1999)]);
            if input_tune {
                expected.writes.push(Write::Irq(true));
            }
            assert_eq!(sys.irq, input_tune);
            assert_registers(&expected);
        }
    }

    // Verify BlueJay EEPROM tune and play_note_freq also drive step 3 in diode mode.
    REGS.set(Registers::default());
    let mut expected = Registers::default();
    let mut timer = Timer;
    let mut driver = phase::G0APhaseDriver::new();
    let mut b = bridge::StmBridge {
        pwm: &mut timer,
        phase: &mut driver,
    };
    let mut sys = System { irq: true };
    let sounds = rm32::sounds::Sounds::new(1999);
    let mut tune = [0u8; 128];
    tune[0] = 1; // non-0xFF header activates BlueJay tune parser
    sounds.play_startup_with_tune(&mut b, &mut sys, &tune, 64);
    expected.writes.push(Write::Irq(false));
    let [float, low, driven] = ORDER[2]; // step 3
    atomic_step(&mut expected, float, low, driven, false);
    expected.writes.push(Write::Watchdog);
    for p in 0..3 {
        solid(&mut expected, p, false);
    }
    expected
        .writes
        .extend([Write::Divider(0), Write::Period(1999), Write::Irq(true)]);
    assert_registers(&expected);

    sounds.play_note_freq(&mut b, &mut sys, 1000, 25, 64);
    let reload = (64u32 * 100_000 / 1000) as u16;
    let scaled_vol = (15u32 * reload as u32 / 1999) as u16;
    expected.writes.extend([
        Write::Divider(9),
        Write::Period(reload),
        Write::Compare(0, scaled_vol),
        Write::Compare(1, scaled_vol),
        Write::Compare(2, scaled_vol),
    ]);
    expected.queued = [scaled_vol; 3];
    atomic_step(&mut expected, float, low, driven, false);
    expected.writes.push(Write::DelayMs(25));
    assert_registers(&expected);
}

#[test]
fn emergency_off_clears_odr_and_switches_moder_to_output() {
    use rm32::hal::EmergencyOff;

    let mut driver = phase::G0APhaseDriver::new();
    REGS.set(Registers::default());
    driver.all_pwm(PwmMode::Complementary);
    REGS.with_borrow_mut(|r| r.writes.clear());

    let mut expected = REGS.with_borrow(|r| r.clone());
    emergency::G0AEmergencyOff::emergency_off();

    let bsrr_a = (1 << (7 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16)) | (1 << (10 + 16));
    let bsrr_b = (1 << 16) | (1 << (1 + 16));
    expected.odr[0] &= !(bsrr_a >> 16);
    expected.odr[1] &= !(bsrr_b >> 16);
    expected.writes.push(Write::Bsrr(0, bsrr_a));
    expected.writes.push(Write::Bsrr(1, bsrr_b));

    let mask_a = (0b11 << 14) | (0b11 << 16) | (0b11 << 18) | (0b11 << 20);
    let out_a = (0b01 << 14) | (0b01 << 16) | (0b01 << 18) | (0b01 << 20);
    expected.mode[0] = (expected.mode[0] & !mask_a) | out_a;
    expected.mode[1] = (expected.mode[1] & !0b1111) | 0b0101;
    expected.writes.push(Write::Mode(0, expected.mode[0]));
    expected.writes.push(Write::Mode(1, expected.mode[1]));

    assert_registers(&expected);
}
