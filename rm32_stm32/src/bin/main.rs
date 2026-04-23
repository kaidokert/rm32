//! RM32 ESC firmware entry point — MCU-independent.
//!
//! All MCU-specific init is in `init::init()`.
//! This file only uses shared types and the `init::InitResult`.

#![no_std]
#![no_main]

use cortex_m_rt::entry;

use rm32::commutation::Commutation;
use rm32::control::state::{BemfState, DutyState, Measurements, ProtectionState, TelemetryState};
use rm32::config::EepromConfig;
use rm32::pid::Pid;
use rm32::hal::{PwmOutput, PhaseOutput, System};

use rm32_stm32::isr::{self, IsrState, IsrHal};
use rm32_stm32::flash::FlashStorage;
use rm32_stm32::main_loop::MainState;
use rm32_stm32::config;

#[cfg(feature = "stm32g071")]
const BOARD: rm32::board::BoardConfig = rm32::board::GEN_64K_G071;
#[cfg(feature = "stm32f051")]
const BOARD: rm32::board::BoardConfig = rm32::board::SISKIN_F051;
#[cfg(feature = "stm32l431")]
const BOARD: rm32::board::BoardConfig = rm32::board::NEUTRON_L431;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop { cortex_m::asm::bkpt(); }
}

#[entry]
fn main() -> ! {
    // --- MCU-specific init (clocks, GPIO, peripherals, NVIC) ---
    let p = rm32_stm32::init::init();

    // --- WS2812 LED: boot indicator (dim red) ---
    let mut led = rm32_stm32::ws2812_hal::Ws2812Gpio::new(
        BOARD.led_pin.unwrap_or(8), // PB8 default
        config::CPU_FREQUENCY_MHZ as u32,
    );
    if BOARD.has_led {
        use rm32::ws2812::{WS2812Pin as _, send_status, LedStatus};
        cortex_m::interrupt::free(|_| send_status(&mut led, LedStatus::Boot));
    }

    // --- Startup tune (before peripherals move to ISR) ---
    let mut pwm = p.pwm;
    let mut phase = p.phase;
    let mut sys = p.sys;
    {
        use rm32::sounds::Sounds;
        let sounds = Sounds::new(config::TIM1_AUTORELOAD);
        sounds.play_startup(&mut pwm, &mut phase, &mut sys);
    }

    // --- Start IWDG watchdog (after startup tune, matching C sequencing) ---
    {
        const IWDG_BASE: u32 = 0x4000_3000;
        const KR: u32 = IWDG_BASE;       // Key register
        const PR: u32 = IWDG_BASE + 0x04; // Prescaler
        const RLR: u32 = IWDG_BASE + 0x08; // Reload
        const SR: u32 = IWDG_BASE + 0x0C; // Status

        #[cfg(feature = "stm32g071")]
        const IWDG_CONFIG: (u32, u32) = (0, 4095);   // prescaler /4, reload 4095 → ~410ms

        #[cfg(feature = "stm32f051")]
        const IWDG_CONFIG: (u32, u32) = (2, 4000);   // prescaler /16, reload 4000 → ~1600ms

        #[cfg(feature = "stm32l431")]
        const IWDG_CONFIG: (u32, u32) = (2, 4000);   // prescaler /16, reload 4000 → ~1600ms

        unsafe {
            (KR as *mut u32).write_volatile(0x5555);  // Unlock PR/RLR
            (PR as *mut u32).write_volatile(IWDG_CONFIG.0);
            (RLR as *mut u32).write_volatile(IWDG_CONFIG.1);
            // Wait for registers to update (PVU + RVU bits in SR)
            while (SR as *const u32).read_volatile() & 0x03 != 0 {}
            (KR as *mut u32).write_volatile(0xCCCC);  // Start IWDG
            (KR as *mut u32).write_volatile(0xAAAA);  // Initial reload
        }
    }

    // --- Build ISR state and move to global ---
    let isr_state = IsrState {
        commutation: Commutation::new(),
        bemf: BemfState::default(),
        duty: DutyState::default(),
        hal: IsrHal {
            pwm,
            comp: p.comp,
            interval: p.interval,
            com_timer: p.com_timer,
            phase,
            #[cfg(feature = "stm32g071")]
            input: {
                let mut ic = rm32_stm32::input_capture::DshotCapture::new();
                ic.init();
                use rm32::hal::InputCapture;
                ic.receive_dshot_dma();
                ic
            },
            #[cfg(feature = "stm32f051")]
            input: {
                let mut ic = rm32_stm32::input_capture_f051::F051DshotCapture::new();
                ic.init();
                use rm32::hal::InputCapture;
                ic.receive_dshot_dma();
                ic
            },
            #[cfg(feature = "stm32l431")]
            input: {
                let mut ic = rm32_stm32::input_capture_l431::L431DshotCapture::new();
                ic.init();
                use rm32::hal::InputCapture;
                ic.receive_dshot_dma();
                ic
            },
        },
        cmd: rm32::dshot_commands::CommandProcessor::default(),
        edt: rm32::edt::EdtScheduler::default(),
        crsf: rm32::crsf::CrsfParser::new(),
        transfer: rm32::transfer::TransferState::default(),
        config: EepromConfig::default(),
        forward: true,
        frametime_low: 400,
        frametime_high: 600,
        ten_khz_counter: 0,
        one_khz_loop_counter: 0,
        armed_timeout_count: 0,
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
        desync_check: false,
        current_filter: rm32::current::CurrentFilter::new(),
        voltage_filter: rm32::filter::EwmaPow2::new(),
    };

    // --- Load EEPROM settings from flash ---
    let flash = FlashStorage::new();
    {
        use rm32::hal::Flash as _;
        flash.read(config::EEPROM_START, main_state.config.as_bytes_mut());
    }
    // Validate and apply version migration
    if !main_state.config.is_valid() {
        main_state.config = EepromConfig::default();
    }
    main_state.config.apply_version_defaults();
    {
        let cfg = &main_state.config;
        main_state.current_pid.kp = (cfg.current_p as u32) * 2;
        main_state.current_pid.ki = cfg.current_i as u32;
        main_state.current_pid.kd = (cfg.current_d as u32) * 2;
        main_state.motor_kv = (cfg.motor_kv as u16) * 40 + 20;
        main_state.low_cell_volt_cutoff = cfg.low_cell_volt_cutoff as u16 + 250;
    }

    // Propagate loaded config to ISR state (before interrupts enabled)
    isr::with_isr_state(|isr| {
        isr.config = main_state.config.clone();
        isr.forward = main_state.config.dir_reversed == 0;
    });

    // --- ADC + Telemetry (already initialized by init(), create handles) ---
    #[cfg(feature = "stm32g071")]
    let (mut adc, mut telem) = (
        rm32_stm32::adc::AdcReader::post_init(),
        rm32_stm32::telemetry_uart::TelemUart::post_init(),
    );
    #[cfg(feature = "stm32f051")]
    let (mut adc, mut telem) = (
        rm32_stm32::adc_f051::F051Adc::post_init(),
        rm32_stm32::telemetry_uart_f051::F051TelemUart::post_init(),
    );
    #[cfg(feature = "stm32l431")]
    let (mut adc, mut telem) = (
        rm32_stm32::adc_l431::L431Adc::post_init(),
        rm32_stm32::telemetry_uart_l431::L431TelemUart::post_init(),
    );

    // --- Enable global interrupts ---
    unsafe { cortex_m::interrupt::enable() };

    // --- Main loop ---
    let shared = isr::shared();
    let mut last_armed = false;
    loop {
        main_state.tick(shared, &mut adc, &mut telem);

        // WS2812 LED status updates (on state transitions only)
        if BOARD.has_led {
            let armed = shared.armed();
            if armed && !last_armed {
                use rm32::ws2812::{send_status, LedStatus};
                cortex_m::interrupt::free(|_| send_status(&mut led, LedStatus::Armed));
            }
            last_armed = armed;
        }

        // Dynamic interrupt priority swap (L431 only — M4F has preemption)
        // Low eRPM: DShot DMA > commutation (don't drop input frames)
        // High eRPM: commutation > DShot (don't miss commutation steps)
        #[cfg(feature = "stm32l431")]
        {
            // NVIC_IPR base = 0xE000_E400, each IRQ has 1 byte
            // STM32L4 uses top 4 bits of priority byte (0x00 = highest, 0x10 = next)
            const NVIC_IPR: u32 = 0xE000_E400;
            const DMA1_CH5_IRQ: u32 = 15;     // IRQ number for DMA1_CH5
            const TIM1_UP_TIM16_IRQ: u32 = 25; // IRQ number for TIM1_UP_TIM16
            const COMP_IRQ: u32 = 55;          // IRQ number for COMP

            const DSHOT_PRIORITY_THRESHOLD: u32 = 60;
            unsafe {
                if shared.dshot_telemetry() && shared.commutation_interval() > DSHOT_PRIORITY_THRESHOLD {
                    // DShot DMA gets highest priority
                    ((NVIC_IPR + DMA1_CH5_IRQ) as *mut u8).write_volatile(0x00);
                    ((NVIC_IPR + TIM1_UP_TIM16_IRQ) as *mut u8).write_volatile(0x10);
                    ((NVIC_IPR + COMP_IRQ) as *mut u8).write_volatile(0x10);
                } else {
                    // Commutation + comparator get highest priority
                    ((NVIC_IPR + DMA1_CH5_IRQ) as *mut u8).write_volatile(0x10);
                    ((NVIC_IPR + TIM1_UP_TIM16_IRQ) as *mut u8).write_volatile(0x00);
                    ((NVIC_IPR + COMP_IRQ) as *mut u8).write_volatile(0x00);
                }
            }
        }

        sys.reload_watchdog();
        cortex_m::asm::wfi();
    }
}
