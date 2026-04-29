//! RM32 ESC firmware entry point — MCU-independent.
//!
//! All MCU-specific init is in `init::init()`.
//! This file only uses shared types and the `init::InitResult`.

#![no_std]
#![no_main]

use cortex_m_rt::entry;

use rm32::commutation::Commutation;
use rm32::config::EepromConfig;
use rm32::control::state::{BemfState, DutyState, Measurements, ProtectionState, TelemetryState};
use rm32::hal::{PwmOutput, System, TelemetryUart as _};
use rm32::pid::Pid;

use rm32::main_state::MainState;
use rm32_stm32::init::InitResult;
use rm32_stm32::isr::{self, IsrState};
use rm32_stm32::mcu::FlashStorage;
use rm32_stm32::mcu::{Chip, ChipConfig};

// Board configuration generated from YAML by build.rs.
// Override with: BOARD=boards/my_board.yaml cargo build
include!(concat!(env!("OUT_DIR"), "/board_config.rs"));

// Panic handler in rm32_stm32::panic — forces all FETs off before halting.
// Replaces panic_halt which halts without safing hardware.

#[entry]
fn main() -> ! {
    // --- MCU-specific init (clocks, GPIO, peripherals, NVIC) ---
    let InitResult {
        mut hal,
        mut sys,
        mut adc,
        mut telem,
    } = rm32_stm32::init::init(BOARD.dead_time);

    // --- WS2812 LED: boot indicator (dim red) ---
    let led_pin = rm32_stm32::ws2812_hal::GpioBPin::new(BOARD.led_pin.unwrap_or(8));
    let mut led = rm32_stm32::ws2812_hal::Ws2812Gpio::new(led_pin, Chip::CPU_FREQUENCY_MHZ);
    if BOARD.has_led {
        use rm32::ws2812::{LedStatus, send_status};
        cortex_m::interrupt::free(|_| send_status(&mut led, LedStatus::Boot));
    }

    // --- Startup tune (before peripherals move to ISR) ---
    if BOARD.bridge_enable {
        hal.phase = rm32_stm32::phase::G0APhaseDriver::new_bridge(false);
    }
    {
        use rm32::sounds::Sounds;
        let sounds = Sounds::new(Chip::TIM1_AUTORELOAD);
        sounds.play_startup(&mut hal.pwm, &mut hal.phase, &mut sys);
    }

    // --- RPM pulse output (debug): configure GPIO before phase moves to ISR ---
    if BOARD.pulse_output {
        hal.phase
            .enable_pulse_output::<rm32_stm32::gpio_pin::PB10>();
    }

    // --- Start IWDG watchdog (after startup tune, matching C sequencing) ---
    sys.start_watchdog(Chip::WDG_PRESCALER, Chip::WDG_RELOAD);

    // --- Configure input capture inversion before moving to ISR ---
    {
        use rm32::hal::InputCapture;
        hal.input.set_inverted(BOARD.inverted_input);
        hal.input.receive_dshot_dma();
    }

    // --- Build ISR state and move to global ---
    let isr_state = IsrState {
        commutation: Commutation::new(),
        bemf: BemfState::default(),
        duty: DutyState::default(),
        hal,
        cmd: rm32::dshot_commands::CommandProcessor::default(),
        edt: rm32::edt::EdtScheduler::default(),
        crsf: rm32::crsf::CrsfParser::new(),
        transfer: rm32::transfer::TransferState::default(),
        config: EepromConfig::default(),
        forward: true,
        edt_armed: false,
        tim1_arr: Chip::TIM1_AUTORELOAD,
        frametime_low: 400,
        frametime_high: 600,
        ten_khz_counter: 0,
        one_khz_loop_counter: 0,
        armed_timeout_count: 0,
        voltage_based_ramp: BOARD.voltage_based_ramp,
    };
    isr::init_isr_state(isr_state);

    // --- Build main loop state ---
    let mut main_state = MainState {
        protection: ProtectionState::default(),
        measurements: Measurements::default(),
        telemetry: TelemetryState::default(),
        config: EepromConfig::default(),
        current_pid: Pid::new(400, 0, 1000, 20000, 100000),
        speed_pid: Pid::new(10, 0, 100, 10000, 50000),
        stall_pid: Pid::new(1, 0, 50, 10000, 50000),
        e_rpm: 0,
        average_interval: 0,
        last_average_interval: 0,
        commutation_intervals: [0; 6],
        cell_count: 0,
        motor_kv: 2000,
        low_cell_volt_cutoff: 330,
        voltage_divider: BOARD.voltage_divider,
        millivolt_per_amp: BOARD.millivolt_per_amp,
        current_offset: BOARD.current_offset,
        stall_protection_adjust: 0,
        stall_protect_target_interval: BOARD.stall_protect_interval,
        use_speed_control_loop: false,
        speed_input_override: 0,
        target_e_com_time: 0,
        desync_check: false,
        current_filter: rm32::current::CurrentFilter::new(),
        voltage_filter: rm32::filter::EwmaPow2::new(),
        last_armed: false,
        just_armed: false,
        use_ntc: BOARD.use_ntc,
        led: rm32::main_state::NoLed,
        led_counter: 0,
        timer1_max_arr: Chip::TIM1_AUTORELOAD,
        cpu_mhz: Chip::CPU_FREQUENCY_MHZ as u8,
        ten_khz_counter: 0,
    };

    // --- Check bootloader device info for dynamic EEPROM address ---
    let eeprom_address = {
        const DEVINFO_MAGIC1: u32 = 0x5925_E3DA;
        const DEVINFO_MAGIC2: u32 = 0x4EB8_63D9;
        const DEVINFO_ADDR: u32 = 0x1000 - 32;
        // SAFETY: DEVINFO_ADDR points to a fixed bootloader info region in flash
        // (0x1000 - 32). This is memory-mapped, aligned, and always readable.
        let magic1 = unsafe { (DEVINFO_ADDR as *const u32).read_volatile() };
        let magic2 = unsafe { ((DEVINFO_ADDR + 4) as *const u32).read_volatile() };
        if magic1 == DEVINFO_MAGIC1 && magic2 == DEVINFO_MAGIC2 {
            const DEVICE_32K: u8 = 0x1F; // 32KB flash (F051)
            const DEVICE_64K: u8 = 0x35; // 64KB flash (G071)
            const DEVICE_128K: u8 = 0x2B; // 128KB flash (L431)
            // SAFETY: Magic validated above, so the bootloader info struct is present.
            // Offset 12 holds the device code byte; address is in flash, always readable.
            let device_code = unsafe { *((DEVINFO_ADDR + 8 + 4) as *const u8) };
            match device_code {
                DEVICE_32K => 0x0800_7C00u32,
                DEVICE_64K => 0x0800_F800u32,
                DEVICE_128K => 0x0801_F800u32,
                _ => Chip::EEPROM_START,
            }
        } else {
            Chip::EEPROM_START
        }
    };

    // --- Load EEPROM settings from flash ---
    let flash = FlashStorage::new();
    {
        use rm32::hal::Flash as _;
        flash.read(eeprom_address, main_state.config.as_bytes_mut());
    }
    // Validate and apply version migration
    if !main_state.config.is_valid() {
        main_state.config = EepromConfig::default();
    }
    main_state.config.apply_version_defaults();

    // Derive motor configuration from EEPROM + board (all math now in rm32, host-testable)
    let motor_cfg = main_state.config.derive_motor_config(
        Chip::TIM1_AUTORELOAD,
        BOARD.dead_time,
        BOARD.kv_divider,
        BOARD.startup_boost,
    );
    let minimum_duty_cycle = motor_cfg.minimum_duty;
    let min_startup_duty = motor_cfg.min_startup_duty;
    let startup_max_duty = motor_cfg.startup_max_duty;
    let timer1_max_arr = motor_cfg.timer1_max_arr;
    let dead_time_override = motor_cfg.dead_time_override;

    // Apply derived motor config to main state and PID controllers
    main_state.current_pid.kp = motor_cfg.current_kp;
    main_state.current_pid.ki = motor_cfg.current_ki;
    main_state.current_pid.kd = motor_cfg.current_kd;
    main_state.motor_kv = motor_cfg.motor_kv;
    main_state.low_cell_volt_cutoff = motor_cfg.low_cell_volt_cutoff;
    main_state.timer1_max_arr = motor_cfg.timer1_max_arr;

    // Propagate loaded config to ISR state (before interrupts enabled)
    isr::with_isr_state(|isr| {
        isr.config = main_state.config;
        isr.forward = main_state.config.dir_reversed == 0;
        // Apply timer1_max_arr from pwm_frequency config
        isr.tim1_arr = timer1_max_arr;
        // Apply startup duty from EEPROM
        isr.duty.minimum = minimum_duty_cycle;
        isr.duty.min_startup = min_startup_duty;
        isr.duty.startup_max = startup_max_duty;
        // Apply servo EEPROM calibration to transfer state
        if isr.config.eeprom_version > 0 {
            isr.transfer.servo.low_threshold = motor_cfg.servo_low;
            isr.transfer.servo.high_threshold = motor_cfg.servo_high;
            isr.transfer.servo.neutral = motor_cfg.servo_neutral;
            isr.transfer.servo.dead_band = isr.config.servo_dead_band;
        }
        // Apply dead-time override to duty thresholds
        if dead_time_override > 0 {
            isr.duty.min_startup += dead_time_override;
            isr.duty.minimum += dead_time_override;
            isr.duty.startup_max += dead_time_override;
        }
    });

    // Apply dead-time override via PwmOutput trait
    if dead_time_override > 0 {
        isr::with_isr_state(|isr| {
            isr.hal.pwm.set_dead_time_override(dead_time_override);
        });
    }

    // --- ADC + Telemetry (returned from init()) ---

    // --- Sine mode state ---
    let mut sine_positions = rm32::sine::PhasePositions {
        a: 0,
        b: 120,
        c: 240,
    };

    // --- Enable global interrupts ---
    // SAFETY: All ISR state has been initialized and moved to globals above.
    // NVIC priorities are configured. It is now safe to take interrupts.
    unsafe { cortex_m::interrupt::enable() };

    // --- Main loop ---
    let shared = isr::shared();
    loop {
        // Sine mode: step phases when stepper_sine is active
        if shared.stepper_sine() {
            use rm32::sine::{SineStepResult, sine_step};
            let (result, (ch1, ch2, ch3)) = sine_step(
                &mut sine_positions,
                shared.newinput(),
                shared.armed(),
                true, // forward — TODO: use ISR state forward flag
                main_state.config.motor_poles,
                5, // changeover_step
                BOARD.dead_time as i16,
                Chip::TIM1_AUTORELOAD,
                main_state.config.sine_mode_power,
            );
            // Apply PWM via PwmOutput trait (through ISR state)
            isr::with_isr_state(|isr| {
                isr.hal.pwm.set_compare1(ch1);
                isr.hal.pwm.set_compare2(ch2);
                isr.hal.pwm.set_compare3(ch3);
            });
            match result {
                SineStepResult::Continue(delay_us) => {
                    sys.delay_micros(delay_us as u32);
                }
                SineStepResult::Changeover {
                    commutation_interval,
                    ..
                } => {
                    shared.transition(rm32::motor_mode::MotorEvent::ExitSine);
                    shared.set_commutation_interval(commutation_interval);
                    shared.set_zero_crosses(20);
                }
                SineStepResult::Idle => {}
            }
        }

        main_state.tick(shared, &mut adc, &mut telem);

        // Arming feedback: cell count beeps + LED
        if main_state.just_armed {
            // Play motor beeps for cell count (or single beep if no LVC)
            isr::with_isr_state(|isr| {
                let sounds = rm32::sounds::Sounds::new(Chip::TIM1_AUTORELOAD);
                if main_state.cell_count > 0 {
                    for _ in 0..main_state.cell_count {
                        sounds.play_input(&mut isr.hal.pwm, &mut isr.hal.phase, &mut sys);
                        sys.delay_millis(100);
                        sys.reload_watchdog();
                    }
                } else {
                    sounds.play_input(&mut isr.hal.pwm, &mut isr.hal.phase, &mut sys);
                }
            });

            if BOARD.has_led {
                use rm32::ws2812::{LedStatus, send_status};
                cortex_m::interrupt::free(|_| send_status(&mut led, LedStatus::Armed));
            }
        }

        // WS2812 LED error indicator
        if BOARD.has_led {
            // Error LED on BEMF timeout (stuck rotor)
            if main_state.protection.bemf_timeout_happened > main_state.protection.bemf_timeout
                && main_state.config.stuck_rotor_protection != 0
            {
                use rm32::ws2812::{LedStatus, send_status};
                cortex_m::interrupt::free(|_| send_status(&mut led, LedStatus::Error));
            }
        }

        // Dynamic IRQ priority: swap DShot DMA vs commutation priority based on RPM.
        // Low eRPM: DShot DMA > commutation (don't drop input frames)
        // High eRPM: commutation > DShot (don't miss commutation steps)
        // No-op on most MCUs; L431 (M4F with preemption) does the actual swap.
        rm32_stm32::mcu::adjust_irq_priorities(
            shared.commutation_interval(),
            shared.dshot_telemetry(),
        );

        // EEPROM save on DShot command
        if shared.save_settings_flag() {
            shared.set_save_settings_flag(false);
            // Copy ISR config back and write to flash
            isr::with_isr_state(|isr| {
                main_state.config = isr.config;
            });
            let mut flash = FlashStorage::new();
            use rm32::hal::Flash as _;
            flash.write(eeprom_address, main_state.config.as_bytes());
        }

        // ESC info response on DShot command
        if shared.send_esc_info_flag() {
            shared.set_send_esc_info_flag(false);
            let mut info_pkt = [0u8; 49];
            rm32::telemetry::make_info_packet(&mut info_pkt, main_state.config.as_bytes());
            telem.send_dma(&info_pkt);
        }

        sys.reload_watchdog();
        cortex_m::asm::wfi();
    }
}
