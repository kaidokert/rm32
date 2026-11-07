//! Build script: generates board config from YAML + auto-selects MCU feature.
//!
//! Usage: BOARD=boards/gen_64k_g071.yaml cargo build
//! If BOARD is not set, uses a default based on the active feature flag.

use serde::Deserialize;
use std::env;
use std::fs;
use std::path::Path;

/// BEMF pin config — simple format (single comparator, shared INP).
#[derive(Deserialize)]
struct BemfPinsSimple {
    phase_a: String,
    phase_b: String,
    phase_c: String,
    #[serde(default)]
    common: Option<String>,
}

/// BEMF pin config — dual-comparator format (G431: per-phase comp + INP).
#[derive(Deserialize)]
struct BemfPhaseDual {
    comp: u8,
    inm: String,
    inp: String,
}

/// BEMF pin config — dual-comparator format.
#[derive(Deserialize)]
struct BemfPinsDual {
    phase_a: BemfPhaseDual,
    phase_b: BemfPhaseDual,
    phase_c: BemfPhaseDual,
}

/// Wrapper that accepts either simple or dual format.
#[derive(Deserialize)]
#[serde(untagged)]
enum BemfPinsYaml {
    Simple(BemfPinsSimple),
    Dual(BemfPinsDual),
}

#[derive(Deserialize)]
struct BoardYaml {
    /// Reject the legacy x10 key instead of silently selecting a default.
    #[serde(default)]
    voltage_divider: Option<u16>,
    name: String,
    mcu: String,
    dead_time: u8,
    #[serde(default = "default_voltage_divider_x100")]
    voltage_divider_x100: u16,
    #[serde(default = "default_mv_per_amp")]
    millivolt_per_amp: u16,
    #[serde(default)]
    current_offset: i16,
    #[serde(default = "default_current_ch")]
    current_adc_channel: u8,
    #[serde(default = "default_voltage_ch")]
    voltage_adc_channel: u8,
    #[serde(default = "default_stall")]
    stall_protect_interval: u16,
    #[serde(default = "default_bemf")]
    min_bemf_counts: u8,
    #[serde(default)]
    has_led: bool,
    #[serde(default)]
    led_pin: Option<u8>,
    #[serde(default)]
    use_ntc: bool,
    #[serde(default)]
    inverted_input: bool,
    #[serde(default = "default_kv")]
    kv_divider: u8,
    #[serde(default)]
    startup_boost: bool,
    #[serde(default)]
    voltage_based_ramp: bool,
    #[serde(default)]
    pulse_output: bool,
    #[serde(default)]
    dual_adc: bool,
    #[serde(default)]
    bridge_enable: bool,
    #[serde(default)]
    custom_led: bool,
    bemf_pins: BemfPinsYaml,
    // ---- rm32_stm32-only board extensions (not part of rm32::board) ----
    /// false = no AM32 bootloader on this board: link the app at the
    /// flash base (0x08000000) instead of 0x08001000.
    #[serde(default = "default_true")]
    bootloader: bool,
    /// Gate-driver ENABLE pin (e.g. "PD1"), driven high only while armed.
    #[serde(default)]
    gate_enable_pin: Option<String>,
    /// Gate-driver nFAULT input (open-drain, active low, e.g. "PB14").
    #[serde(default)]
    nfault_pin: Option<String>,
}

fn default_true() -> bool {
    true
}

/// "PD1" -> (port index A=0.., pin number).
fn parse_pin(p: &str) -> (u8, u8) {
    let b = p.as_bytes();
    assert!(
        b.len() >= 3 && b[0] == b'P' && (b'A'..=b'F').contains(&b[1]),
        "bad pin name {p}"
    );
    let n: u8 = p[2..]
        .parse()
        .unwrap_or_else(|_| panic!("bad pin name {p}"));
    assert!(n < 16, "bad pin number in {p}");
    (b[1] - b'A', n)
}

fn pin_opt(p: &Option<String>) -> String {
    match p {
        Some(p) => {
            let (port, pin) = parse_pin(p);
            format!("Some(rm32_stm32::board_ext::PinRef {{ port: {port}, pin: {pin} }})")
        }
        None => "None".to_string(),
    }
}

fn default_voltage_divider_x100() -> u16 {
    1100
}
fn default_mv_per_amp() -> u16 {
    20
}
fn default_current_ch() -> u8 {
    4
}
fn default_voltage_ch() -> u8 {
    6
}
fn default_stall() -> u16 {
    6500
}
fn default_bemf() -> u8 {
    3
}
fn default_kv() -> u8 {
    1
}

fn enabled_mcu() -> &'static str {
    let mut enabled = Vec::new();
    if cfg!(feature = "stm32g071") {
        enabled.push("stm32g071");
    }
    if cfg!(feature = "stm32f051") {
        enabled.push("stm32f051");
    }
    if cfg!(feature = "stm32l431") {
        enabled.push("stm32l431");
    }
    if cfg!(feature = "stm32g431") {
        enabled.push("stm32g431");
    }

    match enabled.as_slice() {
        [mcu] => mcu,
        [] => panic!("No supported MCU feature is enabled"),
        _ => panic!("Exactly one MCU feature must be enabled; got {enabled:?}"),
    }
}

// ---- BEMF pin-to-register mapping per MCU ----
//
// Each MCU family encodes comparator input selection differently.
// These functions map symbolic pin names (e.g. "PB7") to the packed u32
// value that the per-MCU set_inmsel() expects.

fn validate_simple_common(mcu: &str, pins: &BemfPinsSimple) {
    let expected = match mcu {
        "stm32l431" => Some("PB4"),
        "stm32g071" => Some("PA3"),
        "stm32f051" => Some("PA1"),
        _ => None,
    };
    match (pins.common.as_deref(), expected) {
        (Some(actual), Some(expected)) if actual != expected => {
            panic!("Common BEMF pin {actual} does not match {mcu} comparator input {expected}")
        }
        (Some(actual), None) => panic!("Common BEMF pin {actual} is not supported for MCU {mcu}"),
        _ => {}
    }
}

/// STM32L431 COMP2 inverting input: 3-bit INMSEL + 2-bit INMESEL.
/// Packed as `(inmesel << 8) | inmsel`. set_inmsel unpacks both.
fn l431_comp2_inm(pin: &str) -> u32 {
    match pin {
        "PB7" => 0x007, // IO2: INMSEL=0b111, INMESEL=0b00
        "PA0" => 0x107, // IO3: INMSEL=0b111, INMESEL=0b01
        "PA4" => 0x207, // IO4: INMSEL=0b111, INMESEL=0b10
        "PA5" => 0x307, // IO5: INMSEL=0b111, INMESEL=0b11
        _ => panic!("Unknown L431 COMP2 INM pin: {pin}"),
    }
}

/// STM32G071 COMP2 inverting input: 4-bit INMSEL.
/// set_inmsel shifts this left by 4 into the register.
fn g071_comp2_inm(pin: &str) -> u32 {
    match pin {
        "PB3" => 6, // IO1: INMSEL=0b0110
        "PB7" => 7, // IO2: INMSEL=0b0111
        "PA2" => 8, // IO3: INMSEL=0b1000
        _ => panic!("Unknown G071 COMP2 INM pin: {pin}"),
    }
}

/// STM32F051 COMP1: full lower-16-bit CSR value including EN bit.
/// set_inmsel writes entire lower 16 bits.
fn f051_comp1_inm(pin: &str) -> u32 {
    match pin {
        "PA0" => 0b1000001, // INSEL=4 (IO1) + EN
        "PA4" => 0b1010001, // INSEL=5 (IO2) + EN
        "PA5" => 0b1100001, // INSEL=6 (IO3) + EN
        _ => panic!("Unknown F051 COMP1 INM pin: {pin}"),
    }
}

/// STM32G431 dual-comparator: packed `(config << 16) | comp_selector`.
/// config = (inmsel << 4) | (inpsel << 2), comp_selector = 0(COMP1) or 1(COMP2).
fn g431_dual_comp(comp: u8, inm: &str, inp: &str) -> u32 {
    let inmsel: u32 = match (comp, inm) {
        (1, "PA4") | (2, "PA5") => 0b110, // PA4/PA5
        (1, "PA0") | (2, "PA2") => 0b111, // PA0/PA2
        _ => panic!("Unknown G431 COMP{comp} INM pin: {inm}"),
    };
    let inpsel: u32 = match (comp, inp) {
        (1, "PA1") | (2, "PA3") => {
            if comp == 1 {
                0b00
            } else {
                0b01
            }
        } // IO1(COMP1) / IO2(COMP2)
        (1, "PA3") => 0b01, // IO2 on COMP1
        (2, "PA7") => 0b00, // IO1 on COMP2
        _ => panic!("Unknown G431 COMP{comp} INP pin: {inp}"),
    };
    let comp_sel: u32 = if comp == 2 { 1 } else { 0 };
    ((inmsel << 4 | inpsel << 2) << 16) | comp_sel
}

/// Resolve simple-format BEMF pins to packed u32 values based on MCU.
fn resolve_simple_bemf(mcu: &str, pins: &BemfPinsSimple) -> (u32, u32, u32) {
    validate_simple_common(mcu, pins);

    match mcu {
        "stm32l431" => (
            l431_comp2_inm(&pins.phase_a),
            l431_comp2_inm(&pins.phase_b),
            l431_comp2_inm(&pins.phase_c),
        ),
        "stm32g071" => (
            g071_comp2_inm(&pins.phase_a),
            g071_comp2_inm(&pins.phase_b),
            g071_comp2_inm(&pins.phase_c),
        ),
        "stm32f051" => (
            f051_comp1_inm(&pins.phase_a),
            f051_comp1_inm(&pins.phase_b),
            f051_comp1_inm(&pins.phase_c),
        ),
        _ => panic!("No BEMF pin mapping for MCU: {mcu}"),
    }
}

/// Resolve dual-format BEMF pins to packed u32 values.
fn resolve_dual_bemf(mcu: &str, pins: &BemfPinsDual) -> (u32, u32, u32) {
    match mcu {
        "stm32g431" => (
            g431_dual_comp(pins.phase_a.comp, &pins.phase_a.inm, &pins.phase_a.inp),
            g431_dual_comp(pins.phase_b.comp, &pins.phase_b.inm, &pins.phase_b.inp),
            g431_dual_comp(pins.phase_c.comp, &pins.phase_c.inm, &pins.phase_c.inp),
        ),
        _ => panic!("Dual-comp BEMF format not supported for MCU: {mcu}"),
    }
}

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest = Path::new(&out_dir).join("board_config.rs");

    println!("cargo:rerun-if-env-changed=BOARD");

    // Determine board YAML path
    let board_path = if let Ok(path) = env::var("BOARD") {
        path
    } else {
        // Default based on active feature
        let default = if cfg!(feature = "stm32g071") {
            "boards/gen_64k_g071.yaml"
        } else if cfg!(feature = "stm32f051") {
            "boards/siskin_f051.yaml"
        } else if cfg!(feature = "stm32l431") {
            "boards/neutron_l431.yaml"
        } else if cfg!(feature = "stm32g431") {
            "boards/protondrive_g431.yaml"
        } else {
            "boards/gen_64k_g071.yaml"
        };
        default.to_string()
    };

    println!("cargo:rerun-if-changed={}", board_path);

    let yaml = fs::read_to_string(&board_path)
        .unwrap_or_else(|e| panic!("Failed to read board config '{}': {}", board_path, e));
    let board: BoardYaml = serde_yaml::from_str(&yaml)
        .unwrap_or_else(|e| panic!("Failed to parse board config '{}': {}", board_path, e));
    assert!(
        board.voltage_divider.is_none(),
        "voltage_divider was replaced by voltage_divider_x100; multiply the old x10 value by 10"
    );
    assert!(
        board.voltage_divider_x100 > 0,
        "voltage_divider_x100 must be positive"
    );
    if board.min_bemf_counts > u8::MAX / 2 {
        panic!(
            "Board config '{}' min_bemf_counts={} overflows startup thresholds",
            board_path, board.min_bemf_counts
        );
    }

    let enabled_mcu = enabled_mcu();
    assert_eq!(
        board.mcu, enabled_mcu,
        "Board MCU '{}' does not match enabled MCU feature '{}'",
        board.mcu, enabled_mcu
    );
    if board.mcu != "stm32g071" {
        assert!(
            board.gate_enable_pin.is_none() && board.nfault_pin.is_none(),
            "Board config '{}': gate_enable_pin / nfault_pin are implemented for stm32g071 only (MCU is {})",
            board_path,
            board.mcu
        );
    }

    // Resolve BEMF pins from symbolic names to packed register values
    let (bemf_a, bemf_b, bemf_c) = match &board.bemf_pins {
        BemfPinsYaml::Simple(pins) => resolve_simple_bemf(&board.mcu, pins),
        BemfPinsYaml::Dual(pins) => resolve_dual_bemf(&board.mcu, pins),
    };

    // Generate the board config Rust code
    let led_pin = match board.led_pin {
        Some(p) => format!("Some({})", p),
        None => "None".to_string(),
    };

    let code = format!(
        r#"// Auto-generated by build.rs from {board_path}
// Do not edit manually.

/// Board configuration generated from YAML.
pub const BOARD: rm32::board::BoardConfig = rm32::board::BoardConfig {{
    name: "{name}",
    dead_time: {dead_time},
    voltage_divider_x100: {voltage_divider_x100},
    millivolt_per_amp: {millivolt_per_amp},
    current_offset: {current_offset},
    current_adc_channel: {current_adc_channel},
    voltage_adc_channel: {voltage_adc_channel},
    stall_protect_interval: {stall_protect_interval},
    min_bemf_counts: {min_bemf_counts},
    has_led: {has_led},
    led_pin: {led_pin},
    use_ntc: {use_ntc},
    inverted_input: {inverted_input},
    kv_divider: {kv_divider},
    startup_boost: {startup_boost},
    voltage_based_ramp: {voltage_based_ramp},
    pulse_output: {pulse_output},
    dual_adc: {dual_adc},
    bridge_enable: {bridge_enable},
    custom_led: {custom_led},
}};

/// rm32_stm32-only board extensions (see rm32_stm32::board_ext).
pub const BOARD_EXT: rm32_stm32::board_ext::BoardExt = rm32_stm32::board_ext::BoardExt {{
    gate_enable: {gate_enable},
    nfault: {nfault},
    bemf_pins: rm32_stm32::board_ext::BemfPins {{
        phase_a: {bemf_a:#010x},
        phase_b: {bemf_b:#010x},
        phase_c: {bemf_c:#010x},
    }},
}};
"#,
        board_path = board_path,
        name = board.name,
        dead_time = board.dead_time,
        voltage_divider_x100 = board.voltage_divider_x100,
        millivolt_per_amp = board.millivolt_per_amp,
        current_offset = board.current_offset,
        current_adc_channel = board.current_adc_channel,
        voltage_adc_channel = board.voltage_adc_channel,
        stall_protect_interval = board.stall_protect_interval,
        min_bemf_counts = board.min_bemf_counts,
        has_led = board.has_led,
        led_pin = led_pin,
        use_ntc = board.use_ntc,
        inverted_input = board.inverted_input,
        kv_divider = board.kv_divider,
        startup_boost = board.startup_boost,
        voltage_based_ramp = board.voltage_based_ramp,
        pulse_output = board.pulse_output,
        dual_adc = board.dual_adc,
        bridge_enable = board.bridge_enable,
        custom_led = board.custom_led,
        gate_enable = pin_opt(&board.gate_enable_pin),
        nfault = pin_opt(&board.nfault_pin),
    );

    fs::write(&dest, code).unwrap();

    // The board's `rm32::platform::Platform` constants, for the library's
    // `StmPlatform` (board_config.rs above is included by the binary only).
    let platform = format!(
        "// Auto-generated by build.rs from {board_path}
         pub const VOLTAGE_DIVIDER_X100: u16 = {};
         pub const MILLIVOLT_PER_AMP: u16 = {};
         pub const CURRENT_OFFSET: i16 = {};
         pub const USE_NTC: bool = {};
",
        board.voltage_divider_x100, board.millivolt_per_amp, board.current_offset, board.use_ntc,
    );
    fs::write(Path::new(&out_dir).join("board_platform.rs"), platform).unwrap();

    println!("cargo::rustc-check-cfg=cfg(rm32_bridge_enable)");
    if board.bridge_enable {
        println!("cargo:rustc-cfg=rm32_bridge_enable");
    }

    // Per-MCU memory.x (AM32-compatible layout — 4K bootloader at
    // 0x08000000, app at 0x08001000, EEPROM page reserved at the top
    // of the app region, matching each AM32 target's EEPROM_START_ADD).
    // Emitted into OUT_DIR so the one linked always matches the MCU
    // being built.
    let (flash_app_kb, ram_kb) = match board.mcu.as_str() {
        "stm32l431" => (58, 48), // EEPROM @ 0x0800F800; contiguous SRAM1 only
        "stm32g071" => (58, 36), // EEPROM @ 0x0800F800
        "stm32f051" => (60, 8),  // see warning below
        "stm32g431" => (58, 32), // EEPROM @ 0x0800F800
        other => panic!("No memory layout known for MCU '{other}'"),
    };
    if board.mcu == "stm32f051" {
        // The honest AM32 F051 app region is 27K (EEPROM @ 0x08007C00,
        // the 32K-part layout); the current image is ~34K and does not
        // fit. Linking against the 64K part's full span keeps the
        // cross-build green, but any image past 27K overlaps the AM32
        // EEPROM page. Real F051 support needs a size-reduction pass.
        println!(
            "cargo:warning=F051 image exceeds the 27K AM32 app region; \
             linking against the 64K-part span — EEPROM page overlaps. \
             Size reduction required before F051 hardware bringup."
        );
    }
    // No-bootloader boards (SWD-flashed at the flash base) get the 4K
    // the bootloader would occupy; the EEPROM page address is unchanged.
    let (origin, layout, flash_app_kb) = if board.bootloader {
        ("0x08001000", "AM32 4K bootloader below", flash_app_kb)
    } else {
        ("0x08000000", "no bootloader", flash_app_kb + 4)
    };
    let memory_x = format!(
        "/* Auto-generated by build.rs for {mcu} — do not edit.\n\
         * App @ {origin} ({layout}), EEPROM page above. */\n\
         MEMORY\n\
         {{\n\
         \x20 FLASH : ORIGIN = {origin}, LENGTH = {flash_app_kb}K\n\
         \x20 RAM   : ORIGIN = 0x20000000, LENGTH = {ram_kb}K\n\
         }}\n",
        mcu = board.mcu,
    );
    fs::write(Path::new(&out_dir).join("memory.x"), memory_x).unwrap();
    println!("cargo:rustc-link-search={}", out_dir);

    // Auto-set MCU feature based on YAML (informational — the feature must still
    // be passed via Cargo, but this validates consistency)
    let _expected_feature = format!("stm32{}", board.mcu.replace("stm32", ""));
    println!("cargo:rustc-env=BOARD_MCU={}", board.mcu);
    println!("cargo:rustc-env=BOARD_NAME={}", board.name);
}
