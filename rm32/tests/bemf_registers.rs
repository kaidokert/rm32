//! Run the production comparator adapters against a small PAC register recorder.
//! Register values and write order are checked; analog behavior and timing are not.
#![allow(dead_code, unsafe_op_in_unsafe_fn)] // Included ports expose constructors unused by some cases.
extern crate self as cortex_m;

use rm32::hal::{BemfSensor, Phase};
use std::sync::{
    Mutex,
    atomic::{AtomicU32, Ordering},
};

#[path = "../../rm32_stm32/src/adc_generic.rs"]
mod adc_generic;
#[path = "../../rm32_stm32/src/adc_hal.rs"]
mod adc_hal;
#[path = "../../rm32_stm32/src/comp_hal.rs"]
mod comp_hal;
#[path = "../../rm32_stm32/src/comparator.rs"]
mod comparator;
#[path = "../../rm32_stm32/src/dma_buf.rs"]
mod dma_buf;
#[path = "../../rm32_stm32/src/mcu_f051/comparator.rs"]
mod f051;
#[path = "../../rm32_stm32/src/mcu_g071/comparator.rs"]
mod g071;
#[path = "../../rm32_stm32/src/mcu_g071/adc.rs"]
mod g071_adc;
#[path = "../../rm32_stm32/src/mcu_g431/comparator.rs"]
mod g431;
#[path = "../../rm32_stm32/src/mcu_l431/comparator.rs"]
mod l431;
#[path = "../../rm32_stm32/src/regs.rs"]
mod regs;

pub mod asm {
    pub fn delay(_: u32) {}
    pub fn nop() {}
}

static SERIAL: Mutex<()> = Mutex::new(());
static WRITES: Mutex<Vec<(&str, u32)>> = Mutex::new(Vec::new());
static ADC_FAULT: AtomicU32 = AtomicU32::new(0);
struct Reg {
    name: &'static str,
    value: AtomicU32,
}
struct Read(u32);
impl Read {
    fn bits(&self) -> u32 {
        self.0
    }
}
struct Write(u32);
impl Write {
    unsafe fn bits(&mut self, v: u32) -> &mut Self {
        self.0 = v;
        self
    }
    fn en(&mut self) -> &mut Self {
        self
    }
    fn clear_bit(&mut self) -> &mut Self {
        self.0 = 0;
        self
    }
}
impl Reg {
    const fn new(name: &'static str) -> Self {
        Self {
            name,
            value: AtomicU32::new(0),
        }
    }
    fn get(&self) -> u32 {
        self.value.load(Ordering::Relaxed)
    }
    fn seed(&self, v: u32) {
        self.value.store(v, Ordering::Relaxed);
    }
    fn as_ptr(&self) -> *const u32 {
        &self.value as *const AtomicU32 as *const u32
    }
    fn read(&self) -> Read {
        Read(self.get())
    }
    fn write(&self, f: impl FnOnce(&mut Write) -> &mut Write) {
        let mut w = Write(0);
        f(&mut w);
        self.apply(w.0);
    }
    fn modify(&self, f: impl for<'a> FnOnce(&Read, &'a mut Write) -> &'a mut Write) {
        let mut w = Write(self.get());
        f(&self.read(), &mut w);
        self.apply(w.0);
    }
    fn apply(&self, v: u32) {
        WRITES.lock().unwrap().push((self.name, v));
        match self.name {
            // EXTI and ADC ISR pending bits are write-one-to-clear.
            "pr" | "pr1" | "rpr1" | "fpr1" | "isr" => self.seed(self.get() & !v),
            "cr" => {
                // RM0444: calibration and enable only complete when ADVREGEN (1 << 28) is set;
                // ADDIS (1 << 1) disables the ADC and clears ADEN (1 << 0).
                let advregen = (v & (1 << 28)) != 0;
                if (v & (1 << 31)) != 0 && advregen {
                    self.seed(v & !(1 << 31));
                } else if (v & (1 << 1)) != 0 {
                    self.seed(v & !((1 << 1) | 1));
                    BANK.isr.seed(BANK.isr.get() & !1);
                } else {
                    self.seed(v);
                }
                if (v & 1) != 0 && advregen {
                    BANK.isr.seed(BANK.isr.get() | 1);
                }
            }
            "cfgr1" => {
                self.seed(v);
                // RM0444: setting CHSELRMOD while enabled raises CCRDY (1 << 13).
                if (v & (1 << 21)) != 0 && (BANK.cr.get() & 1) != 0 {
                    BANK.isr.seed(BANK.isr.get() | (1 << 13));
                }
            }
            "chselr1" => {
                // RM0444 §15.3.8: CHSELR1 writes are ignored unless ADEN=1, CHSELRMOD=1,
                // and the previous CCRDY flag was cleared.
                let ready = (BANK.cr.get() & 1) != 0
                    && (BANK.cfgr1.get() & (1 << 21)) != 0
                    && (BANK.isr.get() & (1 << 13)) == 0;
                if ready {
                    let stored = if ADC_FAULT.load(Ordering::Relaxed) == 1 {
                        v ^ 0xF
                    } else {
                        v
                    };
                    self.seed(stored);
                    BANK.isr.seed(BANK.isr.get() | (1 << 13));
                }
            }
            _ => self.seed(v),
        }
        // Software triggering is sufficient here to test routing and masking.
        // Analog edges and NVIC dispatch are deliberately outside this model.
        match self.name {
            "swier" => BANK.pr.seed(BANK.pr.get() | v),
            "swier1" => {
                BANK.pr1.seed(BANK.pr1.get() | v);
                BANK.rpr1.seed(BANK.rpr1.get() | v);
            }
            _ => {}
        }
    }
}
macro_rules! bank {
    ($($name:ident),+) => {
        struct Bank { $($name: Reg),+ }
        static BANK: Bank = Bank { $($name: Reg::new(stringify!($name))),+ };
        impl Bank {
            $(fn $name(&self) -> &Reg { &self.$name })+
            fn reset(&self) {
                $(self.$name.seed(0);)+
                ADC_FAULT.store(0, Ordering::Relaxed);
                WRITES.lock().unwrap().clear();
            }
        }
    };
}
bank!(
    csr,
    comp2_csr,
    c1csr,
    c2csr,
    rtsr,
    ftsr,
    imr,
    pr,
    swier,
    rtsr1,
    ftsr1,
    imr1,
    pr1,
    rpr1,
    fpr1,
    swier1,
    apbenr2,
    ahbenr,
    moder,
    cfgr2,
    ccr,
    dr,
    smpr,
    cfgr1,
    chselr1,
    cr,
    isr,
    dmamux_ccr1,
    dma_cr,
    par,
    mar,
    ndtr
);
struct DmaCh2;
impl DmaCh2 {
    fn cr(&self) -> &Reg {
        &BANK.dma_cr
    }
    fn par(&self) -> &Reg {
        &BANK.par
    }
    fn mar(&self) -> &Reg {
        &BANK.mar
    }
    fn ndtr(&self) -> &Reg {
        &BANK.ndtr
    }
}
impl Bank {
    fn ch2(&self) -> DmaCh2 {
        DmaCh2
    }
}
mod pac {
    #[allow(clippy::upper_case_acronyms)]
    pub struct COMP;
    #[allow(clippy::upper_case_acronyms)]
    pub struct EXTI;
    #[allow(clippy::upper_case_acronyms)]
    pub struct ADC;
    #[allow(clippy::upper_case_acronyms)]
    pub struct DMA1;
    #[allow(clippy::upper_case_acronyms)]
    pub struct DMAMUX;
    #[allow(clippy::upper_case_acronyms)]
    pub struct GPIOA;
    #[allow(clippy::upper_case_acronyms)]
    pub struct RCC;
    impl COMP {
        pub const PTR: *const super::Bank = &super::BANK;
        pub fn ptr() -> *const super::Bank {
            Self::PTR
        }
    }
    impl EXTI {
        pub const PTR: *const super::Bank = &super::BANK;
        pub fn ptr() -> *const super::Bank {
            Self::PTR
        }
    }
    impl ADC {
        pub const PTR: *const super::Bank = &super::BANK;
        pub fn ptr() -> *const super::Bank {
            Self::PTR
        }
    }
    impl DMA1 {
        pub const PTR: *const super::Bank = &super::BANK;
        pub fn ptr() -> *const super::Bank {
            Self::PTR
        }
    }
    impl DMAMUX {
        pub fn ptr() -> *const Self {
            &Self
        }
        pub fn ccr(&self, _idx: usize) -> &super::Reg {
            &super::BANK.dmamux_ccr1
        }
    }
    impl GPIOA {
        pub const PTR: *const super::Bank = &super::BANK;
        pub fn ptr() -> *const super::Bank {
            Self::PTR
        }
    }
    impl RCC {
        pub const PTR: *const super::Bank = &super::BANK;
        pub fn ptr() -> *const super::Bank {
            Self::PTR
        }
    }
}
fn writes() -> Vec<(&'static str, u32)> {
    WRITES.lock().unwrap().clone()
}
fn pins(v: [u32; 3]) -> rm32::board::BemfPins {
    rm32::board::BemfPins {
        phase_a: v[0],
        phase_b: v[1],
        phase_c: v[2],
    }
}
const PHASES: [Phase; 3] = [Phase::A, Phase::B, Phase::C];
const SENTINEL: u32 = 0x9135_864a;
const OTHER_LINE: u32 = 1 << 3;

#[test]
fn floating_phase_matches_six_step_truth_table_and_invalid_fallback() {
    let expected = [Phase::C, Phase::A, Phase::B, Phase::C, Phase::A, Phase::B];
    for step in 0..=255 {
        let phase = if (1..=6).contains(&step) {
            expected[usize::from(step - 1)]
        } else {
            Phase::C
        };
        assert_eq!(rm32::hal::floating_phase(step), phase);
    }
}

#[derive(Clone, Copy)]
enum Chip {
    F051,
    G071,
    L431,
    G431,
}
impl Chip {
    fn codes(self) -> [u32; 3] {
        match self {
            Self::F051 => [0x54a1, 0x54b1, 0x54c1],
            Self::G071 => [3, 5, 7],
            Self::L431 => [0x103, 0x205, 0x307],
            Self::G431 => [(0x24 << 16), (0x38 << 16) | 1, (0x58 << 16) | 1],
        }
    }
    fn sensor(self) -> Box<dyn BemfSensor> {
        let p = pins(self.codes());
        match self {
            Self::F051 => Box::new(f051::new_comparator(p)),
            Self::G071 => Box::new(g071::new_comparator(p)),
            Self::L431 => Box::new(l431::new_comparator(p)),
            Self::G431 => Box::new(g431::new_comparator(p)),
        }
    }
    fn line(self, phase: usize) -> u32 {
        1 << match self {
            Self::F051 => 21,
            Self::G071 => 18,
            Self::L431 => 22,
            Self::G431 => {
                if phase == 0 {
                    21
                } else {
                    22
                }
            }
        }
    }
    fn lines(self) -> u32 {
        if matches!(self, Self::G431) {
            (1 << 21) | (1 << 22)
        } else {
            self.line(0)
        }
    }
    fn registers(self, phase: usize) -> (&'static Reg, &'static Reg, &'static Reg, &'static Reg) {
        let comp = match self {
            Self::F051 => &BANK.csr,
            Self::G071 | Self::L431 => &BANK.comp2_csr,
            Self::G431 => {
                if phase == 0 {
                    &BANK.c1csr
                } else {
                    &BANK.c2csr
                }
            }
        };
        if matches!(self, Self::F051) {
            (comp, &BANK.rtsr, &BANK.ftsr, &BANK.imr)
        } else {
            (comp, &BANK.rtsr1, &BANK.ftsr1, &BANK.imr1)
        }
    }
    fn expected_mux(self, phase: usize) -> u32 {
        let code = self.codes()[phase];
        match self {
            Self::F051 => (SENTINEL & 0xffff_0000) | code,
            Self::G071 => (SENTINEL & !0x3f0) | (code << 4) | 0x200,
            Self::L431 => (SENTINEL & !0x0600_0070) | ((code & 7) << 4) | ((code >> 8) << 25),
            Self::G431 => (SENTINEL & !0x7c) | (code >> 16) | 1,
        }
    }
    fn pending_regs(self) -> Vec<&'static Reg> {
        match self {
            Self::F051 => vec![&BANK.pr],
            Self::G071 => vec![&BANK.rpr1, &BANK.fpr1],
            _ => vec![&BANK.pr1],
        }
    }
}

fn check_routing(chip: Chip) {
    let _serial = SERIAL.lock().unwrap();
    let mut sensor = chip.sensor();
    for (phase, selection) in PHASES.into_iter().enumerate() {
        for rising in [false, true] {
            for enabled_edges in [0, chip.lines()] {
                BANK.reset();
                let (comp, rtsr, ftsr, imr) = chip.registers(phase);
                comp.seed(SENTINEL);
                let initial = OTHER_LINE | enabled_edges;
                rtsr.seed(initial);
                ftsr.seed(initial);
                imr.seed(OTHER_LINE);
                for r in chip.pending_regs() {
                    r.seed(initial);
                }
                sensor.select_phase(selection, rising);
                // G431's adapter compensates for its own comparator polarity.
                let r = if matches!(chip, Chip::G431) == rising {
                    initial | chip.line(phase)
                } else {
                    OTHER_LINE
                };
                let f = if matches!(chip, Chip::G431) == rising {
                    OTHER_LINE
                } else {
                    initial | chip.line(phase)
                };
                assert_eq!(
                    writes(),
                    [
                        (comp.name, chip.expected_mux(phase)),
                        (rtsr.name, r),
                        (ftsr.name, f)
                    ]
                );
                assert_eq!(imr.get(), OTHER_LINE); // Selection does not enable IRQs.
                for reg in chip.pending_regs() {
                    assert_eq!(reg.get(), initial);
                }
                for level in [false, true] {
                    comp.seed(if level { 1 << 30 } else { 0 });
                    // Opposite level on the inactive G431 comparator catches wrong routing.
                    if matches!(chip, Chip::G431) {
                        let other = if phase == 0 { &BANK.c2csr } else { &BANK.c1csr };
                        other.seed(if level { 0 } else { 1 << 30 });
                    }
                    assert_eq!(sensor.level(), level);
                    assert_eq!(sensor.filter_read(), level);
                }
                assert_eq!(writes().len(), 3); // Reads have no write side effects.
            }
        }
    }
}

fn check_irq(chip: Chip) {
    let _serial = SERIAL.lock().unwrap();
    let mut sensor = chip.sensor();
    for (phase, selection) in PHASES.into_iter().enumerate() {
        BANK.reset();
        sensor.select_phase(selection, true);
        WRITES.lock().unwrap().clear();
        let (_, _, _, imr) = chip.registers(phase);
        imr.seed(OTHER_LINE);
        for reg in chip.pending_regs() {
            reg.seed(OTHER_LINE);
        }
        assert!(!sensor.pending());
        for reg in chip.pending_regs() {
            for line in [chip.line(phase), chip.lines()] {
                reg.seed(OTHER_LINE | line);
                assert!(sensor.pending());
                sensor.clear_pending();
                assert!(!sensor.pending());
                assert_eq!(reg.get(), OTHER_LINE);
                assert_eq!(imr.get(), OTHER_LINE);
            }
        }
        WRITES.lock().unwrap().clear();
        sensor.enable_irq();
        assert_eq!(writes(), [(imr.name, OTHER_LINE | chip.line(phase))]);
        assert!(!sensor.pending());
        sensor.repend();
        let swier = if matches!(chip, Chip::F051) {
            "swier"
        } else {
            "swier1"
        };
        assert_eq!(writes().last(), Some(&(swier, chip.line(phase))));
        assert!(sensor.pending());
        // Masking must also discard stale pending edges, including the inactive
        // G431 line and both G071 edge banks, while preserving unrelated lines.
        imr.seed(OTHER_LINE | chip.lines());
        for reg in chip.pending_regs() {
            reg.seed(OTHER_LINE | chip.lines());
        }
        WRITES.lock().unwrap().clear();
        sensor.mask_irq();
        let mut expected = vec![(imr.name, OTHER_LINE)];
        expected.extend(chip.pending_regs().iter().map(|r| (r.name, chip.lines())));
        assert_eq!(writes(), expected);
        assert!(!sensor.pending());
        for reg in chip.pending_regs() {
            assert_eq!(reg.get(), OTHER_LINE);
        }
        sensor.enable_irq();
        assert!(!sensor.pending()); // Re-enable cannot revive a discarded edge.
    }
}

#[test]
fn f051_phase_selection_and_polarity() {
    check_routing(Chip::F051);
}
#[test]
fn g071_phase_selection_and_polarity() {
    check_routing(Chip::G071);
}
#[test]
fn l431_phase_selection_and_polarity() {
    check_routing(Chip::L431);
}
#[test]
fn g431_phase_selection_and_polarity() {
    check_routing(Chip::G431);
}
#[test]
fn f051_irq_lifecycle() {
    check_irq(Chip::F051);
}
#[test]
fn g071_irq_lifecycle() {
    check_irq(Chip::G071);
}
#[test]
fn l431_irq_lifecycle() {
    check_irq(Chip::L431);
}
#[test]
fn g431_irq_lifecycle() {
    check_irq(Chip::G431);
}

#[test]
fn g071_adc_init_enables_advregen_and_programs_sequence_with_ccrdy_handshake() {
    use rm32::hal::Adc;

    static BUF: dma_buf::DmaBuf<u16, 3> = dma_buf::DmaBuf::new();
    let _serial = SERIAL.lock().unwrap();
    BANK.reset();

    let mut adc = adc_generic::GenericAdc::new(
        g071_adc::G071AdcOps,
        &BUF,
        adc_hal::TempCalibration {
            cal1_val: 1034,
            cal2_val: 1381,
            cal1_temp: 30,
            cal2_temp: 130,
            cal_vref_mv: 3000,
        },
    );

    assert!(adc.init().is_ok());

    let cr_writes: Vec<u32> = writes()
        .into_iter()
        .filter_map(|(name, v)| (name == "cr").then_some(v))
        .collect();
    // power_up -> calibrate -> enable: every CR write keeps ADVREGEN (1 << 28) set.
    assert_eq!(
        cr_writes,
        vec![1 << 28, (1 << 28) | (1 << 31), (1 << 28) | 1]
    );
    // Sequence 4 | (6 << 4) | (12 << 8) | (0xF << 12) = 0xFC64 latched after CCRDY handshake,
    // and trailing CCRDY cleared.
    assert_eq!(BANK.chselr1.get(), 0x0000_FC64);
    assert_eq!(BANK.isr.get() & (1 << 13), 0);

    adc.start_conversion();
    assert_eq!(BANK.cr.get(), (1 << 28) | (1 << 2) | 1);

    // 3.0 V factory TS_CAL rescaled from 3.3 V VDDA: raw 912 -> 22 C (3.3 V identity on 3300 mV F051).
    let g0_temp = adc.calc_temperature(912);
    assert_eq!(g0_temp, rm32::units::DegreesCelsius(22));

    let f0_adc = adc_generic::GenericAdc::post_init(
        g071_adc::G071AdcOps,
        &BUF,
        adc_hal::TempCalibration {
            cal1_val: 700,
            cal2_val: 900,
            cal1_temp: 30,
            cal2_temp: 110,
            cal_vref_mv: 3300,
        },
    );
    assert_eq!(
        f0_adc.calc_temperature(800),
        rm32::units::DegreesCelsius(70)
    );
}

#[test]
fn g071_adc_enable_rejects_corrupted_chselr_readback() {
    use adc_hal::AdcPeripheral;

    let _serial = SERIAL.lock().unwrap();
    BANK.reset();
    ADC_FAULT.store(1, Ordering::Relaxed);

    g071_adc::G071AdcOps.power_up();
    assert!(g071_adc::G071AdcOps.calibrate().is_ok());
    match g071_adc::G071AdcOps.enable() {
        Err(regs::InitError::Timeout("ADC CHSELR readback")) => {}
        other => panic!("expected ADC CHSELR readback timeout, got {:?}", other),
    }
    // Failed sequence programming must disable the ADC (ADDIS clears ADEN) while keeping ADVREGEN.
    assert_eq!(BANK.cr.get(), 1 << 28);
}
