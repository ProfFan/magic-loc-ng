#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![feature(pointer_is_aligned_to)]
#![feature(impl_trait_in_assoc_type)]
#![feature(async_closure)]
#![feature(async_fn_traits)]
#![feature(let_chains)]
#![feature(ascii_char)]
#![feature(ascii_char_variants)]

mod configuration;
mod console;
mod display;
mod hist_buffer;
mod indicator;
mod inertial;
mod network;
mod ranging;
mod utils;

use core::cell::RefCell;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex, once_lock::OnceLock};
use esp_hal::sync::RawMutex as EspRawMutex;
use esp_hal_embassy::{Executor, InterruptExecutor};
use static_cell::StaticCell;

extern crate alloc;

use defmt as _;
use embassy_embedded_hal::shared_bus::{asynch::i2c::I2cDevice, blocking::spi::SpiDevice};
use embassy_executor::{SendSpawner, Spawner};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    cpu_control::{CpuControl, Stack},
    dma::*,
    dma_buffers,
    gpio::{Input, Level, Output, Pull},
    interrupt::{self, software::SoftwareInterruptControl},
    ledc::{LSGlobalClkSource, Ledc, LowSpeed},
    prelude::*,
    rng::Rng,
    spi::{
        master::{Spi, SpiDmaBus},
        SpiMode,
    },
    timer::{systimer::SystemTimer, timg::TimerGroup, AnyTimer},
    Async, Blocking,
};
use esp_wifi::{self};

// Stack for the second core
static mut APP_CORE_STACK: Stack<65536> = Stack::new();

static IMU_PUBSUB: OnceLock<
    embassy_sync::pubsub::PubSubChannel<
        esp_hal::sync::RawMutex,
        icm426xx::fifo::FifoPacket4,
        3,
        2,
        1,
    >,
> = OnceLock::new();

struct Dw3000Device {
    spi: SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Blocking>, Output<'static>>,
    rst: Output<'static>,
    irq: Input<'static>,
}

impl Dw3000Device {
    pub fn split_borrow(
        &mut self,
    ) -> (
        &mut SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Blocking>, Output<'static>>,
        &mut Output<'static>,
        &mut Input<'static>,
    ) {
        (&mut self.spi, &mut self.rst, &mut self.irq)
    }
}

static DW3000: OnceLock<Mutex<EspRawMutex, Dw3000Device>> = OnceLock::new();
static BMS_I2C: OnceLock<
    Mutex<EspRawMutex, esp_hal::i2c::master::I2c<'static, Async, esp_hal::peripherals::I2C0>>,
> = OnceLock::new();

#[main]
async fn main(spawner: Spawner) {
    let mut hal_config = esp_hal::Config::default();
    hal_config.cpu_clock = esp_hal::clock::CpuClock::Clock240MHz;
    // hal_config.psram = esp_hal::psram::PsramConfig {
    //     flash_frequency: esp_hal::psram::FlashFreq::FlashFreq80m,
    //     ram_frequency: esp_hal::psram::SpiRamFreq::Freq80m,
    //     ..Default::default()
    // };

    let peripherals = esp_hal::init(hal_config);

    // let (start, size) = esp_hal::psram::psram_raw_parts(&peripherals.PSRAM);
    // defmt::info!("PSRAM start: {:x}, size: {}", start, size);

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    let timer2: AnyTimer = systimer.alarm0.into();
    let timer3: AnyTimer = systimer.alarm1.into();
    esp_hal_embassy::init([timer0, timer1, timer2, timer3]);

    esp_alloc::heap_allocator!(128 * 1024);

    let config_store = alloc::sync::Arc::new(embassy_sync::mutex::Mutex::<EspRawMutex, _>::new(
        configuration::ConfigurationStore::new().unwrap(),
    ));

    config_store
        .lock()
        .await
        .registry
        .register::<u8>(b"MVERSION")
        .unwrap();

    config_store
        .lock()
        .await
        .registry
        .register::<u8>(b"SYS_ROLE")
        .unwrap();

    let mut buff = [0u8; 256];
    let ver = config_store
        .lock()
        .await
        .get::<u8>(&mut buff, b"MVERSION")
        .await
        .unwrap();

    defmt::info!("Configuration version: {}", ver);

    // Spawners
    static EXECUTOR_L2: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    let executor_l2 = EXECUTOR_L2.init(InterruptExecutor::new(sw_ints.software_interrupt1));
    let spawner_l2 = executor_l2.start(interrupt::Priority::Priority2);

    // Start the serial comm as early as possible
    // Since the `esp-println` impl will block if buffer becomes full
    spawner_l2
        .spawn(esp_fast_serial::serial_comm_task(peripherals.USB_DEVICE))
        .unwrap();

    let sclk = peripherals.GPIO33;
    let mosi = peripherals.GPIO40;
    let miso = peripherals.GPIO47;
    let imu_cs = peripherals.GPIO34;
    let imu_int = peripherals.GPIO48;
    let imu_clkin = peripherals.GPIO41;
    let baro_cs = peripherals.GPIO11;

    // Prevent the barometer from operating in I2C mode
    let mut baro_cs = Output::new(baro_cs, Level::High);
    baro_cs.set_low();
    Timer::after_millis(1).await;
    baro_cs.set_high();

    Timer::after_secs(2).await;

    // --- WIFI ---
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    // let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None);
    // let wifi_timer = PeriodicTimer::new(timg0.timer0.into());

    let wifi = peripherals.WIFI;
    let controller = esp_wifi::init(
        timg1.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    spawner
        .spawn(network::wifi_driver_task(
            config_store.clone(),
            controller,
            wifi,
            spawner,
        ))
        .unwrap();

    Timer::after_secs(1).await;

    // --- BMS ---
    let bms_sda = peripherals.GPIO1;
    let bms_scl = peripherals.GPIO2;

    let bms_i2c = esp_hal::i2c::master::I2c::<'static, _, _>::new_typed(
        peripherals.I2C0,
        esp_hal::i2c::master::Config {
            frequency: 100.kHz(),
            ..Default::default()
        },
    )
    .with_sda(bms_sda)
    .with_scl(bms_scl)
    .into_async();
    // let mut reg07 = [0u8; 1];

    // // Register 0x07, 1 byte
    // bms_i2c.write_read(0x6B, &[0x07], &mut reg07).await.ok();
    // bms_i2c
    //     .write(0x6B, &[0x07, reg07[0] | 0b00100000u8])
    //     // .write(0x6B, &[0x07, reg07[0] & 0b11011111u8])
    //     .await
    //     .ok();

    BMS_I2C
        .init(Mutex::<EspRawMutex, _>::new(bms_i2c))
        .unwrap_or(());

    // --- Display ---
    let display_sda = peripherals.GPIO6;
    let display_scl = peripherals.GPIO7;

    static I2C1: StaticCell<
        Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, Async, esp_hal::peripherals::I2C1>>,
    > = StaticCell::new();
    let i2c1 = I2C1.init(Mutex::<NoopRawMutex, _>::new(
        esp_hal::i2c::master::I2c::new_typed(
            peripherals.I2C1,
            esp_hal::i2c::master::Config {
                frequency: 400.kHz(),
                ..Default::default()
            },
        )
        .with_sda(display_sda)
        .with_scl(display_scl)
        .into_async(),
    ));

    // Scan I2C bus
    let display_i2c = I2cDevice::new(i2c1);

    spawner.spawn(display::display_task(display_i2c)).unwrap();

    // --- IMU ---
    // let dma = Dma::new(peripherals.DMA);
    let _dma_channel = peripherals.DMA_CH0;

    let spi = Spi::new_typed_with_config(
        peripherals.SPI2,
        esp_hal::spi::master::Config {
            frequency: 24.MHz(),
            mode: SpiMode::Mode0,
            ..Default::default()
        },
    )
    .with_sck(sclk)
    .with_miso(miso)
    .with_mosi(mosi);

    // LEDC for IMU 32kHz clock
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(esp_hal::ledc::timer::Number::Timer0);
    lstimer0
        .configure(esp_hal::ledc::timer::config::Config {
            duty: esp_hal::ledc::timer::config::Duty::Duty9Bit,
            clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
            frequency: 32.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.channel(esp_hal::ledc::channel::Number::Channel0, imu_clkin);
    channel0
        .configure(esp_hal::ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 50,
            pin_config: esp_hal::ledc::channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let imu_pubsub_ = embassy_sync::pubsub::PubSubChannel::new();

    let imu_pubsub = IMU_PUBSUB.get_or_init(|| imu_pubsub_);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let config_store_ = config_store.clone();
    static CPU1_SPAWNER: OnceLock<SendSpawner> = OnceLock::new();
    let cpu1_fnctn = move || {
        static EXECUTOR_CORE1: StaticCell<InterruptExecutor<2>> = StaticCell::new();
        let executor_core1 =
            EXECUTOR_CORE1.init(InterruptExecutor::new(sw_ints.software_interrupt2));
        let spawner_l2 = executor_core1.start(interrupt::Priority::Priority2);

        spawner_l2
            .spawn(inertial::imu_task(
                config_store_.clone(),
                Output::new(imu_cs, Level::High),
                Input::new(imu_int, Pull::Up),
                // dma_channel.configure(false, DmaPriority::Priority0).,
                spi,
                imu_pubsub,
            ))
            .unwrap();

        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(move |spawner| {
            let _ = CPU1_SPAWNER.init(spawner.make_send());

            let dw_cs = peripherals.GPIO8;
            let dw_rst = peripherals.GPIO9;
            let dw_irq = peripherals.GPIO15;

            let dw_sclk = peripherals.GPIO36;
            let dw_mosi = peripherals.GPIO35;
            let dw_miso = peripherals.GPIO37;

            let spi = Spi::new_with_config(
                peripherals.SPI3,
                esp_hal::spi::master::Config {
                    frequency: 24.MHz(),
                    mode: SpiMode::Mode0,
                    ..Default::default()
                },
            )
            .with_sck(dw_sclk)
            .with_miso(dw_miso)
            .with_mosi(dw_mosi);

            let dma_channel = peripherals.DMA_CH1;

            let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(1024);
            let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
            let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

            let bus = RefCell::new(
                spi.with_dma(dma_channel)
                    .with_buffers(dma_rx_buf, dma_tx_buf),
            );
            // .into_async();

            static BUS: StaticCell<
                embassy_sync::blocking_mutex::Mutex<
                    NoopRawMutex,
                    RefCell<SpiDmaBus<'static, Blocking>>,
                >,
            > = StaticCell::new();
            let bus: &'static embassy_sync::blocking_mutex::Mutex<_, _> =
                BUS.init_with(|| embassy_sync::blocking_mutex::Mutex::<NoopRawMutex, _>::new(bus));

            let dw_rst = Output::new(dw_rst, Level::High);
            let dw_irq = Input::new(dw_irq, Pull::Up);
            let dw_cs = Output::new(dw_cs, Level::High);

            // static SPI_DEV: StaticCell<
            //     embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<
            //         'static,
            //         NoopRawMutex,
            //         SpiDmaBus<'static, Async>,
            //         Output<'static>,
            //     >,
            // > = StaticCell::new();
            // let spi_dev = SPI_DEV
            //     .init(embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(bus, dw_cs));

            DW3000
                .init(Mutex::<_, _>::new(Dw3000Device {
                    spi: embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(
                        bus, dw_cs,
                    ),
                    rst: dw_rst,
                    irq: dw_irq,
                }))
                .unwrap_or(());

            // static STATE: StaticCell<ranging::uwb_driver::State<127, 1, 1>> = StaticCell::new();
            // let state = STATE.init(ranging::uwb_driver::State::<127, 1, 1>::new());

            // let (uwb_runner, uwb_device) = ranging::uwb_driver::new(state, spi_dev, dw_rst, dw_irq);

            // spawner.spawn(ranging::uwb_driver_task(uwb_runner)).unwrap();

            // UWB_DEVICE
            //     .init(Mutex::<EspRawMutex, _>::new(uwb_device))
            //     .unwrap();
        });
    };

    defmt::info!("Starting core 1");

    let _guard = cpu_control
        .start_app_core(
            unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) },
            cpu1_fnctn,
        )
        .unwrap();

    let led = peripherals.GPIO5;
    spawner
        .spawn(indicator::control_led(Output::new(led, Level::Low)))
        .ok();

    // Start the interactive console
    spawner
        .spawn(console::console(
            spawner,
            *CPU1_SPAWNER.get().await,
            config_store.clone(),
        ))
        .unwrap();

    // Wait until the boot sequence is complete
    Timer::after_secs(2).await;

    let role = config_store
        .lock()
        .await
        .get::<u8>(&mut buff, b"SYS_ROLE")
        .await
        .unwrap();

    defmt::info!("System role: {}", role);

    if let Some(role) = role {
        if role == 1 {
            // Master
            crate::console::apps::uwb_master::uwb_master_start(spawner, *CPU1_SPAWNER.get().await)
                .await
                .unwrap();

            // IMU Stream start
            crate::console::apps::imu_stream::imu_streamer_start(spawner)
                .await
                .unwrap();
        } else if role == 2 {
            // Client
            crate::console::apps::uwb_client::uwb_client_start(spawner, *CPU1_SPAWNER.get().await)
                .await
                .unwrap();

            // IMU Stream start
            crate::console::apps::imu_stream::imu_streamer_start(spawner)
                .await
                .unwrap();
        }
    }

    // A never-ending heartbeat
    loop {
        Timer::after_secs(5).await;
    }
}
