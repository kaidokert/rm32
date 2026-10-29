//! Execute the production GPIO and borrowed TIM1 adapters on observable registers.
//! This verifies software write order, not electrical timing or transistor behavior.
#![allow(dead_code)] // The included port modules also expose board-only helpers.

use rm32::bridge::Bridge;
use rm32::hal::PhaseOutput;
use std::cell::RefCell;

#[path = "../../rm32_stm32/src/bridge.rs"]
mod bridge;
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
fn mode(r: &mut Registers, (port, pin): (usize, u8), value: u32) {
    r.mode[port] = (r.mode[port] & !(3 << (pin * 2))) | (value << (pin * 2));
    r.writes.push(Write::Mode(port, r.mode[port]));
}
fn level(r: &mut Registers, (port, pin): (usize, u8), high: bool) {
    let value = 1 << (pin + if high { 0 } else { 16 });
    r.writes.push(Write::Bsrr(port, value));
    r.odr[port] = (r.odr[port] & !(1 << pin)) | if high { 1 << pin } else { 0 };
}
fn solid(r: &mut Registers, phase: usize, low_on: bool) {
    mode(r, LOW[phase], 1);
    level(r, LOW[phase], low_on);
    mode(r, HIGH[phase], 1);
    level(r, HIGH[phase], false);
}
fn pwm(r: &mut Registers, phase: usize, complementary: bool, enable: bool) {
    match (enable, complementary) {
        (true, false) => {} // Existing bridge-enable policy leaves the low pin alone.
        (true, true) => {
            mode(r, LOW[phase], 1);
            level(r, LOW[phase], true);
        }
        (false, true) => mode(r, LOW[phase], 2),
        (false, false) => {
            mode(r, LOW[phase], 1);
            level(r, LOW[phase], false);
        }
    }
    mode(r, HIGH[phase], 2);
}
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
        for complementary in [false, true] {
            for forward in [false, true] {
                REGS.set(Registers::default());
                let mut expected = Registers::default();
                let mut driver = if enable {
                    phase::G0APhaseDriver::new_bridge(complementary)
                } else {
                    phase::G0APhaseDriver::new(complementary)
                };
                // Exercise consecutive GPIO states in each rotation direction.
                let sequence = if forward {
                    [2, 3, 4, 5, 6, 1]
                } else {
                    [6, 5, 4, 3, 2, 1]
                };
                for step in sequence {
                    let [float, low, driven] = ORDER[usize::from(step - 1)];
                    solid(&mut expected, float, false);
                    solid(&mut expected, low, true);
                    pwm(&mut expected, driven, complementary, enable);
                    driver.com_step(step);
                    assert_registers(&expected);
                }
                for invalid in (0..=255).filter(|s| !(1..=6).contains(s)) {
                    driver.com_step(invalid);
                    assert_registers(&expected); // Invalid steps remain no-ops.
                }
            }
        }
    }
}

#[test]
fn off_brake_sine_and_reentry_preserve_write_order() {
    for enable in [false, true] {
        for complementary in [false, true] {
            REGS.set(Registers::default());
            let mut expected = Registers::default();
            let mut driver = if enable {
                phase::G0APhaseDriver::new_bridge(complementary)
            } else {
                phase::G0APhaseDriver::new(complementary)
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
            driver.all_pwm();
            for p in 0..3 {
                pwm(&mut expected, p, complementary, enable);
            }
            assert_registers(&expected);
        }
    }
}

#[test]
fn pulse_output_only_toggles_on_sectors_one_and_four() {
    REGS.set(Registers::default());
    let mut expected = Registers::default();
    let mut driver = phase::G0APhaseDriver::new(false);
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
    let mut driver = phase::G0APhaseDriver::new(false);
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
    let mut expected = Registers::default();
    expected.queued = [0; 3];
    expected.live = [0; 3];
    expected.writes = vec![
        Write::Period(1999),
        Write::Compare(0, 101),
        Write::Compare(1, 202),
        Write::Compare(2, 303),
        Write::Latch,
        Write::Compare(0, 0),
        Write::Compare(1, 0),
        Write::Compare(2, 0),
        Write::Latch,
    ];
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
    for input_tune in [false, true] {
        REGS.set(Registers::default());
        let mut expected = Registers::default();
        let mut timer = Timer;
        let mut driver = phase::G0APhaseDriver::new(false);
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
            solid(&mut expected, float, false);
            solid(&mut expected, low, true);
            pwm(&mut expected, driven, false, false);
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
